# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

from packaging.version import parse
import paho.mqtt
import paho.mqtt.client as mqtt
import time
import ruamel.yaml
from ruamel.yaml import CommentedMap, CommentedSeq
import json
import sys
import logging
from enum import IntEnum, Enum
import glob
import ssl
import threading

from abc import ABC, abstractmethod
from typing import Any, Optional, Callable


def _ruamel_yaml_to_native(obj):
    if isinstance(obj, CommentedMap):
        return {k: _ruamel_yaml_to_native(v) for k, v in obj.items()}
    elif isinstance(obj, CommentedSeq):
        return [_ruamel_yaml_to_native(v) for v in obj]
    elif isinstance(obj, dict):  # 念のため他の dict 型も対応
        return {k: _ruamel_yaml_to_native(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [_ruamel_yaml_to_native(v) for v in obj]
    else:
        return obj


def load_yaml_config(config_yaml_file: str) -> Optional[dict]:
    config_yaml = ruamel.yaml.YAML()

    try:
        with open(config_yaml_file) as file:
            config = _ruamel_yaml_to_native(config_yaml.load(file.read()))
        if config is None or type(config) != dict:
            return None

        return config

    except FileNotFoundError:
        return None


class RobotStatus(IntEnum):
    # For elevator
    HAS_ENTERED = 1
    HAS_GOT_OFF = 2
    CANCEL_TO_ENTER = 3
    CANCEL_TO_GET_OFF = 4
    KEEP_DOOR_OPEN = 5

    # For door
    START_TO_PASS = 0
    HAS_PASSED = 1
    CANCEL_TO_PASS = 3
    # KEEP_DOOR_OPEN = 5


class ResultCode(IntEnum):
    SUCCESS = 1
    FAIL = 2
    ERROR = 3
    UNDER_EMERGENCY = 99


class DeviceType(Enum):
    ELEVATOR = 'elevator'
    DOOR = 'door'
    ALARM = 'alarm'
    SEM = 'sem'


class LciFloorInfo:
    def __init__(self, floor_name: str, has_front_door: bool = True, has_rear_door: bool = False):
        self.floor_name = floor_name
        self.has_front_door = has_front_door
        self.has_rear_door = has_rear_door


class LciResponseEvent:
    """
    Customization of theading.Event to return response.

    To be tolerant against occasional delays and missed communication, it is important to synchronize the internal variables with remote LCI devices after communicaiton recovery.
    In this case, RequestElevatorStatus and RequestDoorStatus are needed to be sync API call.
    To implement this function, it is needed this customization.
    """

    def __init__(self) -> None:
        self._cond = threading.Condition(threading.Lock())
        self.responded_payload: dict | None = None

    def set(self, responded_payload: dict | None) -> None:
        with self._cond:
            self.responded_payload = responded_payload
            self._cond.notify_all()

    def wait(self, timeout=None) -> dict | None:
        with self._cond:
            signaled = (self.responded_payload is not None)
            if not signaled:
                self._cond.wait(timeout)
            return self.responded_payload


class LciContext(ABC):
    """
    Context class for LCI to support multiple devices (elevators, doors or so) with a single Robot Account.
    """
    _logger: Any

    _device_type: DeviceType

    _topic_prefix: str

    _context_lock: threading.RLock

    # Event to wait sychronous API response
    _response_event_pool: dict[tuple[str, float], LciResponseEvent]

    # To supress re-Registration soon after reset()
    # Without this function, OpenRMF messaging results in multiple Registration without enough interval even when the lift or the door are out of operation.
    # OpenRMF will send LiftRequest/DoorRequest before receiving LiftState/DoorState with the error code and consective error handlilng.

    _is_connected: bool
    _last_disconnected_timestamp: float

    _is_available: bool
    _last_unavailable_timestamp: float

    _is_registered: bool
    _last_get_unregisterd_timestamp: float

    def __init__(self, device_type: DeviceType, logger=None) -> None:
        if logger is None:
            self._logger = logging.getLogger('lci_client')
        else:
            self._logger = logger

        self._device_type = device_type

        self._context_lock = threading.RLock()

        self._response_event_pool = {}

        self._is_connected = False
        self._last_disconnected_timestamp = 0

        self._is_available = False
        self._last_unavailable_timestamp = 0

        self._is_registered = False
        self._last_get_unregisterd_timestamp = 0

    @abstractmethod
    def initialize(self, config: dict) -> bool:
        pass

    def get_device_type(self) -> DeviceType:
        return self._device_type

    def create_response_event(self, api: str, requested_timestamp: float) -> LciResponseEvent:
        # Called by the master thread
        response_event = LciResponseEvent()
        with self._context_lock:
            self._response_event_pool.update({
                (api, requested_timestamp): response_event
            })
        return response_event

    def remove_response_event(self, api: str, requested_timestamp: float) -> None:
        with self._context_lock:
            res_event = self._response_event_pool.pop(
                (api, requested_timestamp), None)

            # To avoid waiting forever, it is needed to notify the called thread.
            if res_event is not None:
                res_event.set(None)

    def msg_callback(self, response_api: str, res_payload: dict) -> None:
        # Called by the MQTT receiving thread to wake up the master thread

        if response_api.endswith('Result'):
            orig_api = response_api[:-6]
        elif response_api.endswith('Status'):
            orig_api = f'Request{response_api}'
        else:
            return

        try:
            requested_timestamp = float(res_payload['requested_timestamp'])
            with self._context_lock:
                res_event = self._response_event_pool.pop(
                    (orig_api, requested_timestamp))
                res_event.set(res_payload)
        except:
            pass

    def _get_result_code(self, res_payload: dict) -> ResultCode | None:
        try:
            return ResultCode(int(res_payload['result']))
        except:
            return None

    def proc_response_payload(self, api: str, res_payload: dict) -> bool:
        # Check and process response payload to change internal state and log.

        # Response from LCI device == LCI is connected.
        self.set_connected(True)

        result_code = self._get_result_code(res_payload)
        if result_code is None:
            # Bad paylaod
            self._logger.error(
                f'[{self._topic_prefix}] [lci, {api}, res: bad payload]')
            return False

        if result_code is ResultCode.UNDER_EMERGENCY:
            self.set_available(False)
            self._logger.error(
                f'[{self._topic_prefix}] [lci, {api}, res: emergency]')
            return False

        self.set_available(True)

        if result_code is ResultCode.ERROR:
            self._logger.error(
                f'[{self._topic_prefix}] [lci, {api}, res: error]')
            return False

        if result_code is ResultCode.FAIL:
            self.set_registered(False)
            self._logger.error(
                f'[{self._topic_prefix}] [lci, {api}, res: fail]')
            return False

        # ResultCode.SUCCESS
        match api:
            case 'Registration':
                dry_run = res_payload.get('dry_run', False)
                if not isinstance(dry_run, bool):
                    self._logger.error(
                        f'[{self._topic_prefix}] [lci, {api}, res: bad format] dry_run: {dry_run}')
                    return False

                if dry_run:
                    self._logger.debug(
                        f'[{self._topic_prefix}] [lci, {api}, res: success (dry_run)]')
                    return True

                self.set_registered(True)
                self._logger.debug(
                    f'[{self._topic_prefix}] [lci, {api}, res: success]')
                return True

            case 'Release':
                self.set_registered(False)
                self._logger.debug(
                    f'[{self._topic_prefix}] [lci, {api}, res: success]')
                return True

            case _:
                # Another payload processing will be covered by child classes.
                self._logger.debug(
                    f'[{self._topic_prefix}] [lci, {api}, res: success]')
                return True

    @abstractmethod
    def _reset(self) -> None:
        pass

    def set_connected(self, on_off: bool) -> None:
        with self._context_lock:
            if not on_off:
                # If disconnected, synchronization with LCI is disrupted.
                # Reset and wait for recovery.
                self._last_disconnected_timestamp = time.time()
                self.set_available(False)

            self._is_connected = on_off

    def set_available(self, on_off: bool) -> None:
        with self._context_lock:
            if not on_off:
                self._last_unavailable_timestamp = time.time()
                self.set_registered(False)

            self._is_available = on_off

    def set_registered(self, on_off: bool) -> None:
        with self._context_lock:
            if self._is_registered and not on_off:  # falling edge
                self._last_get_unregisterd_timestamp = time.time()

                self._reset()

            self._is_registered = on_off

    def is_connected(self) -> bool:
        return self._is_connected

    def is_available(self) -> bool:
        return self._is_available

    def is_registered(self) -> bool:
        return self._is_registered


class LciElevatorContext(LciContext):
    _bldg_id: str
    _bank_id: str
    _elevator_id: str
    _floor_list: list[LciFloorInfo]

    _is_robot_in_the_car: bool

    _current_floor: str
    _current_door: int

    _target_floor: str
    _target_door: int

    def __init__(self, bldg_id: str, logger=None) -> None:
        super().__init__(DeviceType.ELEVATOR, logger)
        self._bldg_id = bldg_id

    def initialize(self, config: dict) -> bool:
        self._bank_id = config.get('lci_bank_id', '')
        self._elevator_id = config.get('lci_elevator_id', '')
        floor_list_tmp = config.get('lci_floor_list', [])

        ret = True
        if self._bank_id == '':
            self._logger.error('[lci] <lci_bank_id> is not specified.')
            ret = False

        if self._elevator_id == '':
            self._logger.error(
                '[lci] <lci_elevator_id> is not specified. Use 0.')
            self._elevator_id = '0'
        else:
            self._elevator_id = str(self._elevator_id)

        if type(floor_list_tmp) is not list or len(floor_list_tmp) == 0:
            self._logger.error('[lci] <lci_floor_list> is not a valid list.')
            ret = False

        self._floor_list = []

        for f in floor_list_tmp:
            if isinstance(f, list) and len(f) == 3 and type(f[1]) == bool and type(f[2]) == bool:
                self._floor_list.append(LciFloorInfo(str(f[0]), f[1], f[2]))
            else:
                self._floor_list.append(LciFloorInfo(str(f), True, False))

        if 63 < len(self._floor_list):
            self._logger.error(
                '[lci] LCI supports no more than 63 levels of floors.')
            ret = False

        if ret == False:
            return False

        self._is_robot_in_the_car = False

        self._topic_prefix = f'/lci/{self._bldg_id}/{self._bank_id}/{self._elevator_id}'  # noqa

        self._current_floor = self._floor_list[0].floor_name
        self._current_door = 0
        self._target_floor = ''
        self._target_door = 0

        return True

    def _reset(self):
        with self._context_lock:
            self._is_robot_in_the_car = False

            # Set the door status closed for robots to refrain entering because ElevatorStatus is only updated during registered.
            # self._current_floor = self._floor_list[0].floor_name
            self._current_door = 0

            self._target_floor = ''
            self._target_door = 0

    def _set_current_floor_and_door(self, current_floor: str, current_door: int) -> None:
        with self._context_lock:
            self._current_floor = current_floor
            self._current_door = current_door

    def proc_response_payload(self, api: str, res_payload: dict) -> bool:
        ret = super().proc_response_payload(api, res_payload)

        if res_payload is None or not ret:
            return False

        # ResultCode.SUCCESS

        # No need of further processing for 'CallElevator' and 'RobotStatus'

        if api == 'RequestElevatorStatus':
            try:
                self._set_current_floor_and_door(
                    str(res_payload.get('floor', '')),
                    int(res_payload.get('door', 0)))
            except:
                self._logger.error(
                    f'[{self._topic_prefix}] [lci, {api}, res: bad format floor/door]')
                return False

        return True


class LciDoorContext(LciContext):
    _bldg_id: str
    _floor_id: str
    _door_id: str
    _door_type: str

    _current_door: int
    _current_lock: int

    def __init__(self, bldg_id: str, logger=None) -> None:
        super().__init__(DeviceType.DOOR, logger)
        self._bldg_id = bldg_id

    def initialize(self, config: dict) -> bool:
        self._floor_id = config.get('lci_floor_id', '')
        self._door_id = config.get('lci_door_id', '')
        self._door_type = config.get('lci_door_type', '')

        ret = True

        if self._floor_id == '':
            self._logger.error('[lci] <lci_floor_id> is not specified.')
            ret = False

        if self._door_id == '':
            self._logger.error('[lci] <lci_door_id> is not specified.')
            ret = False

        if self._door_type == '':
            self._logger.error('[lci] <lci_door_type> is not specified.')
            ret = False

        if ret == False:
            return False

        self._topic_prefix = f'/lci/{self._bldg_id}/{self._floor_id}/{self._door_id}'  # noqa

        self._current_door = 0
        self._current_lock = 1

        return True

    def _reset(self):
        with self._context_lock:
            self._current_door = 0
            self._current_lock = 1

    def proc_response_payload(self, api: str, res_payload: dict) -> bool:
        ret = super().proc_response_payload(api, res_payload)

        if res_payload is None or not ret:
            return False

        # ResultCode.SUCCESS

        # No need of further processing for 'OpenDoor' and 'RobotStatus'

        if api == 'RequestDoorStatus':
            try:
                if self._door_type == 'lock':
                    self._current_lock = int(res_payload.get('lock', 1))
                else:
                    self._current_door = int(res_payload.get('door', 0))
            except:
                self._logger.error(
                    f'[{self._topic_prefix}] [lci, {api}, res: bad format door/lock]')
                return False

        return True


class LciAlarmContext:
    _logger: Any
    _device_type: DeviceType

    _bldg_id: str
    _alarm_type: str
    _area_id: str

    _alarm_name: str

    # True: in alarm, False: not in alarm
    _alarm_state: bool

    _last_received_timestamp: float

    # (alarm_type, area_id, bool) --> None
    alarm_callback: Callable[[str, str, bool], None] | None

    def __init__(self, bldg_id: str, logger=None) -> None:
        self._logger = logger
        self._device_type = DeviceType.ALARM
        self._bldg_id = bldg_id
        self.alarm_callback = None

    def initialize(self, config: dict) -> bool:
        self._alarm_type = config.get('lci_alarm_type', '')
        self._area_id = config.get('lci_area_id', '')

        ret = True

        if self._alarm_type == '':
            self._logger.error('[lci] <lci_alarm_type> is not specified.')
            ret = False

        if self._area_id == '':
            self._logger.error('[lci] <lci_area_id> is not specified.')
            ret = False

        if ret == False:
            return False

        self._alarm_name = f'{self._alarm_type}/{self._area_id}'

        self._alarm_state = False
        self._last_received_timestamp = 0

        return True

    def _reset(self) -> None:
        self._alarm_state = False
        self._last_received_timestamp = 0

    def msg_callback(self, payload: dict) -> None:
        alarm_state = str(payload.get('alarm', None))
        alarm_type = payload.get('alarm_type', None)
        area_id = payload.get('area_id', None)
        dry_run = payload.get('dry_run', False)

        if alarm_type != self._alarm_type or area_id != self._area_id:
            # not for this context
            return

        alarm_state_bool = None
        if alarm_state == '99':
            alarm_state_bool = True
        elif alarm_state == '0':
            alarm_state_bool = False

        if alarm_state_bool is None:
            # unexpected value
            return

        self._last_received_timestamp = time.time()

        if dry_run is True:
            self._logger.info(
                f'[lci, {self._alarm_name}, dry_run] {alarm_state_bool}')
            return

        self._alarm_state = alarm_state_bool
        if self.alarm_callback is not None:
            try:
                self.alarm_callback(
                    self._alarm_type, self._area_id, alarm_state_bool)
                self._logger.info(
                    f'[lci, {self._alarm_name}] {alarm_state_bool}')
            except Exception as e:
                self._logger.error(
                    f'[lci, {self._alarm_name}] [Exception in alarm_callback()] {e}')


class LciSemContext(LciContext):
    _bldg_id: str
    _resource_id: str
    _resource_type: str
    _locked_by: str
    _expiration_time: float
    _max_expiration_time: float

    # Special extension for RMF. This value will be used in LCI RMF Adapter.
    _num_of_vdoor: int

    def __init__(self, bldg_id: str, logger=None) -> None:
        super().__init__(DeviceType.SEM, logger)
        self._bldg_id = bldg_id

    def initialize(self, config: dict) -> bool:
        self._resource_id = config.get('lci_resource_id', '')
        self._resource_type = config.get('lci_resource_type', '')
        self._locked_by = ''
        self._expiration_time = 0
        self._max_expiration_time = 0

        try:
            # Special extension for RMF. This value will be used in LCI RMF Adapter.
            self._num_of_vdoor = int(config.get('num_of_vdoor', 1))
        except:
            self._num_of_vdoor = 1

        ret = True

        if self._resource_id == '':
            self._logger.error('[lci] <lci_resource_id> is not specified.')
            ret = False

        if self._resource_type != 'binary':
            self._logger.error(
                f'[lci] resource_type: {self._resource_type} is not supported.')
            ret = False

        if ret == False:
            return False

        self._topic_prefix = f'/lci/{self._bldg_id}/sem/{self._resource_id}'  # noqa

        return True

    def _reset(self):
        self._locked_by = ''
        self._expiration_time = 0
        self._max_expiration_time = 0

    def proc_response_payload(self, api: str, res_payload: dict) -> bool:
        ret = super().proc_response_payload(api, res_payload)

        if res_payload is None or not ret:
            return False

        # ResultCode.SUCCESS

        match api:
            case 'Registration':
                if res_payload.get('dry_run', False):
                    return True

                self._locked_by = res_payload.get(
                    'requested_robot_id', '')
                self._expiration_time = res_payload.get(
                    'expiration_time', 0)
                self._max_expiration_time = res_payload.get(
                    'max_expiration_time', 0)
                return True

            case 'RequestResourceStatus':
                # RequestResourceStatus will be succeeded even when another robot take the resource.
                # By cheking the change of who uses this resource, make it as an error.

                locked_by = res_payload.get(
                    'robot_id', '')
                if self._locked_by != locked_by:
                    self.set_registered(False)
                    return False

                # update the expiration times
                self._expiration_time = res_payload.get(
                    'expiration_time', 0)
                self._max_expiration_time = res_payload.get(
                    'max_expiration_time', 0)
                return True

        return True


class LciClient:
    _logger: Any

    _mqtt_server: str
    _bldg_id: str
    _context_dict: dict[str, LciContext]
    _alarm_context_dict: dict[str, LciAlarmContext]

    _robot_id: str

    _mqtt_client: mqtt.Client

    _publish_lock: threading.Lock

    def __init__(self, logger=None) -> None:
        if logger is None:
            self._logger = logging.getLogger('lci_client')
        else:
            self._logger = logger

        self._context_dict = {}
        self._alarm_context_dict = {}

        self._publish_lock = threading.Lock()

    def initialize(self, config_file_path: str, cert_dir: str) -> bool:
        """
        Initalization of LciClient

        Args:
            config_file_path (str): Path of LCI server config file (server_config_<bldg_id>.yaml) for a single building provided by Octa Robotics
            cert_dir (str): Path of directory including the certificate files of LCI Robot Account provided by Octa Robotics
        """

        config = load_yaml_config(config_file_path)
        if config is None:
            self._logger.error(
                f'[lci] Failed to load yaml config: {config_file_path}')
            return False

        self._mqtt_server = config.get('lci_mqtt_server', '')
        self._bldg_id = config.get('lci_bldg_id', '')

        if self._mqtt_server == '':
            self._logger.error('[lci] <lci_mqtt_server> is not specified.')
            return False

        if self._bldg_id == '':
            self._logger.error('[lci] <lci_bldg_id> is not specified.')
            return False

        elevator_config = config.get('elevators', None)
        if type(elevator_config) is list:
            for ec in elevator_config:
                context = LciElevatorContext(self._bldg_id, self._logger)
                if context.initialize(ec):
                    context._bldg_id = self._bldg_id
                    self._context_dict.update({context._topic_prefix: context})

        door_config = config.get('doors', None)
        if type(door_config) is list:
            for dc in door_config:
                context = LciDoorContext(self._bldg_id, self._logger)
                if context.initialize(dc):
                    self._context_dict.update({context._topic_prefix: context})

        alarm_config = config.get('alarms', None)
        if type(alarm_config) is list:
            for ac in alarm_config:
                al_context = LciAlarmContext(self._bldg_id, self._logger)
                if al_context.initialize(ac):
                    self._alarm_context_dict.update(
                        {al_context._alarm_name: al_context})

        sem_config = config.get('sems', None)
        if type(sem_config) is list:
            for sc in sem_config:
                context = LciSemContext(self._bldg_id, self._logger)
                if context.initialize(sc):
                    self._context_dict.update({context._topic_prefix: context})

        try:
            v2_0_0 = parse('2.0.0')
            paho_version = parse(paho.mqtt.__version__)

            # Default client_id. It is needed for on-premise version
            self._robot_id = f'_DUMMYID'
            mqtt_port = 1883

            if cert_dir != None:
                try:
                    with open(cert_dir + '/ClientID') as file:
                        self._robot_id = file.readline().replace("\n", "")
                except Exception as e:
                    self._logger.warning(
                        f'[lci] Exception in LciClient: {e}. Use robot_id: {self._robot_id}')

            # reinitialize() does not support mqtt.CallbackAPIVersion.VERSION1.
            # so it is needed to re-create an instance.
            if v2_0_0 <= paho_version:
                self._mqtt_client = mqtt.Client(
                    mqtt.CallbackAPIVersion.VERSION1,
                    protocol=mqtt.MQTTv311,
                    client_id=self._robot_id,
                    userdata=self, clean_session=False)
            else:
                self._mqtt_client = mqtt.Client(
                    client_id=self._robot_id,
                    protocol=mqtt.MQTTv311, userdata=self, clean_session=False)

            if cert_dir != None:
                try:
                    ca_files = glob.glob(cert_dir + '/*.pem')
                    cert_files = glob.glob(
                        cert_dir + '/*certificate.pem.crt')
                    key_files = glob.glob(cert_dir + '/*private.pem.key')

                    self._mqtt_client.tls_set(
                        ca_files[0], cert_files[0], key_files[0],
                        tls_version=ssl.PROTOCOL_TLSv1_2)
                    mqtt_port = 8883

                except Exception as e:
                    self._logger.warning(
                        f'[lci] Exception in LciClient: {e}. Use Plain MQTT port: {mqtt_port}')

            self._mqtt_client.on_connect = self._on_connect
            self._mqtt_client.on_message = self._on_message
            self._mqtt_client.on_disconnect = self._on_disconnect

            # self.mqtt_client.enable_logger(self.logger)
            # self._mqtt_client.on_log = lambda c, u, l, b: print(b)

            # MQTT connection should be maintained.
            self._mqtt_client.reconnect_delay_set(min_delay=1, max_delay=5)

            self._mqtt_client.connect(
                str(self._mqtt_server), port=mqtt_port)
        except Exception as e:
            self._logger.error(f'[lci] Exception in LciClient: {e}')
            return False

        return True

    def get_contexts(self) -> dict[str, LciContext]:
        return self._context_dict

    def get_alarm_contexts(self) -> dict[str, LciAlarmContext]:
        return self._alarm_context_dict

    def start(self):
        self._mqtt_client.loop_start()

    def stop(self):
        self._mqtt_client.loop_stop()

    def _on_connect(self, client: mqtt.Client, userdata: 'LciClient', flags: dict, rc: int) -> None:
        self._logger.info(
            f'[lci] Connected to {self._mqtt_server} with client_id {self._robot_id}, result code {rc}')

        self._mqtt_client.subscribe([
            (f'/lci/{self._bldg_id}/+/+/RegistrationResult/{self._robot_id}', 1),
            (f'/lci/{self._bldg_id}/+/+/CallElevatorResult/{self._robot_id}', 1),
            (f'/lci/{self._bldg_id}/+/+/ElevatorStatus/{self._robot_id}', 1),
            (f'/lci/{self._bldg_id}/+/+/OpenDoorResult/{self._robot_id}', 1),
            (f'/lci/{self._bldg_id}/+/+/DoorStatus/{self._robot_id}', 1),
            (f'/lci/{self._bldg_id}/+/+/RobotStatusResult/{self._robot_id}', 1),
            (f'/lci/{self._bldg_id}/+/+/ReleaseResult/{self._robot_id}', 1),
            (f'/lci/{self._bldg_id}/alarm/message', 1),
        ])

        # Sem-specific topic
        self._mqtt_client.subscribe([
            (f'/lci/{self._bldg_id}/+/+/ResourceStatus/{self._robot_id}', 1),
        ])

    def _on_disconnect(self, client: mqtt.Client, userdata: 'LciClient', rc: int) -> None:
        self._logger.info(
            f'[lci] Disconnected from {self._mqtt_server} with client_id {self._robot_id}, result code ' + str(rc))

    def _on_message(self, client: mqtt.Client, userdata: 'LciClient', msg: mqtt.MQTTMessage):

        try:
            payload_kv = json.loads(msg.payload)
        except json.JSONDecodeError as e:
            self._logger.debug(
                f'[lci] Skip message with non-JSON payload: {e}')
            return

        if type(payload_kv) != dict:
            self._logger.error(
                f'[lci] MQTT Payload format error: {msg.topic}, {msg.payload}')
            return

        self._msg_callback(msg.topic, payload_kv)

    def _msg_callback(self, topic: str, payload_kv: dict) -> None:
        if topic == f'/lci/{self._bldg_id}/alarm/message':
            for al_context in self._alarm_context_dict.values():
                al_context.msg_callback(payload_kv)
            self._logger.debug(
                f'[/lci/{self._bldg_id}/alarm]   [mq_rcv] {payload_kv}')
            return

        token = topic.split('/')
        topic_prefix = '/'.join(token[0:5])
        api = token[-2]

        context = self._context_dict.get(topic_prefix, None)
        if context is None:
            self._logger.warning(
                f'[lci] No relevant context for {topic}, {payload_kv}')
            return

        context.msg_callback(api, payload_kv)
        self._logger.debug(
            f'[{context._topic_prefix}]   [mq_rcv] {topic}, {payload_kv}')

    def _publish(self, context: LciContext, api: str, payload: dict, timeout_sec: float = 0, wait_response: bool = True) -> dict | None:
        # returns response payload from LCI
        # If wait_response is False, this function always returns None

        if not self._mqtt_client.is_connected():
            self._logger.error(f'[{context._topic_prefix}]   [mq_send, not connected] {api}, {payload}')  # noqa
            return None

        topic = f'{context._topic_prefix}/{api}/{self._robot_id}'
        timestamp = int(time.time() * 1000)/1000

        if context._device_type == DeviceType.SEM:
            # Sem API does not require robot_id in the payload.
            payload.update({
                'timestamp': timestamp,
            })
        else:
            payload.update({
                'robot_id': self._robot_id,
                'timestamp': timestamp,
            })
        json_payload = json.dumps(payload)

        # To wait with timeout, LciResponseEvent (similar class to threading.Event) is registered to context.
        if wait_response:
            response_event = context.create_response_event(api, timestamp)

        with self._publish_lock:
            try:
                pub_info = self._mqtt_client.publish(
                    topic, json_payload, qos=1)
            except Exception as e:
                self._logger.error(
                    f'[{context._topic_prefix}]   [mq_send, failed] {e}')
                return None

        # Wait for completion of publish()
        start_time = time.time()
        while True:
            if start_time + 5.0 < time.time():
                # When publish() does not finish for 5 s, it should be considered disconnected.
                self._logger.error(f'[{context._topic_prefix}]   [mq_send, timeout ({time.time()-start_time:.03f})] {api}, {payload}')  # noqa
                context.remove_response_event(api, timestamp)
                try:
                    self._mqtt_client.disconnect()
                    # refresh socket
                    # reconnect() is to be automaticlally and repeatedly called in loop_forever the worker thread started by loop_start().
                    self._mqtt_client.reconnect()
                except Exception as e:
                    self._logger.warning(
                        f'[{context._topic_prefix}]   [mq_reconnect, exception] {e}')

                return None

            try:
                # When the state is disconnection, wait_for_publish() will return soon with Exception
                pub_info.wait_for_publish(timeout=0.2)

                if pub_info.is_published():
                    # Elapsed time between PUB and PUBACK
                    self._logger.info(f'[{context._topic_prefix}]   [mq_send ({time.time()-start_time:.03f})] {api}, {payload}')  # noqa
                    break

            except Exception:
                time.sleep(0.2)
                continue

        if not wait_response:
            return None

        # Wait for receiving response from LCI
        start_time = time.time()
        res_payload = response_event.wait(timeout_sec)
        if res_payload is not None:
            # Elapsed time between LCI's request and response
            self._logger.info(f'[{context._topic_prefix}]   [mq_recv ({time.time()-start_time:.03f})] {api}, {res_payload}')  # noqa
        else:
            self._logger.error(f'[{context._topic_prefix}]   [mq_recv, timeout ({time.time()-start_time:.03f})] {api}')  # noqa
        return res_payload

    def _sync_execute_with_retry(self, context: LciContext, api: str, payload: dict,
                                 response_timeout_sec: float, wait_response: bool,
                                 max_retry_num: int, retry_interval_sec: float) -> bool:
        # Retry until successfully receiving reponse from LCI devices or the specified number of retry.
        for i in range(max_retry_num):
            res_payload = self._publish(
                context, api, payload, response_timeout_sec, wait_response)
            if res_payload is None:
                self._logger.warning(
                    f'[{context._topic_prefix}] [lci, {api}, failed ({i+1}/{max_retry_num})]')

                if i + 1 < max_retry_num:
                    time.sleep(retry_interval_sec)
                continue

            if api == 'Registration':
                # To retry, hook RegistrationResult in the case that another robot uses the lift.

                result_code = context._get_result_code(res_payload)
                if result_code is not ResultCode.SUCCESS:
                    self._logger.warning(
                        f'[{context._topic_prefix}] [lci, {api}, failed ({i+1}/{max_retry_num})] {result_code}')

                    if i + 1 < max_retry_num:
                        time.sleep(retry_interval_sec)
                    continue

            # Sem API does not respond with requested_robot_id in payload
            if 'requested_robot_id' not in res_payload:
                res_payload['requested_robot_id'] = self._robot_id

            return context.proc_response_payload(api, res_payload)

        # This timeout may be because LCI server or device are not reachable.
        context.set_connected(False)
        self._logger.error(
            f'[{context._topic_prefix}] [lci, {api}, res: timeout]')
        return False

    def do_registration_elevator(self, context: LciElevatorContext, dry_run: bool = False) -> bool:
        # When return False, this funciton should be retried.

        if dry_run:
            payload = {'dry_run': True}
        else:
            payload = {}

        # max_retry_num: 7 with retry_interval_sec: 30 means 210s in total. It is enough time to wait for Release by another robot.
        return self._sync_execute_with_retry(context, 'Registration', payload,
                                             response_timeout_sec=185, wait_response=True,
                                             max_retry_num=6, retry_interval_sec=30)

    def do_registration_door(self, context: LciDoorContext, dry_run: bool = False) -> bool:
        # When return False, this funciton should be retried.

        if dry_run:
            payload = {'dry_run': True}
        else:
            payload = {}

        # max_retry_num: 9 with retry_interval_sec: 10 means 90s in total. It is enough time to wait for Release by another robot.
        return self._sync_execute_with_retry(context, 'Registration', payload,
                                             response_timeout_sec=10, wait_response=True,
                                             max_retry_num=9, retry_interval_sec=10)

    def do_registration_sem(self, context: LciSemContext, dry_run: bool = False) -> bool:
        # When return False, this funciton should be retried.

        if dry_run:
            payload = {'dry_run': True,
                       'timeout': 180}
        else:
            # Request 180s for timeout
            payload = {'timeout': 180}

        # max_retry_num: 18 with retry_interval_sec: 10 means 180s in total. It is enough time to wait for Release by another robot.
        return self._sync_execute_with_retry(context, 'Registration', payload,
                                             response_timeout_sec=10, wait_response=True,
                                             max_retry_num=18, retry_interval_sec=10)

    def do_release(self, context: LciContext, wait_response: bool = True) -> bool:
        return self._sync_execute_with_retry(context, 'Release', {},
                                             response_timeout_sec=10, wait_response=wait_response,
                                             max_retry_num=3, retry_interval_sec=5)

    def do_robot_status(self, context: LciContext, robot_status: RobotStatus, wait_response: bool = True) -> bool:
        return self._sync_execute_with_retry(context, 'RobotStatus', {'state': robot_status.value},
                                             response_timeout_sec=10, wait_response=wait_response,
                                             max_retry_num=3, retry_interval_sec=5)

    def do_call_elevator(self, context: LciElevatorContext,
                         origination: Optional[str], destination: Optional[str],
                         origination_door: Optional[int] = 0, destination_door: Optional[int] = 0,
                         direction: int = 0) -> bool:

        if context._target_floor == '':
            if origination is None or origination_door is None:
                self._logger.error(f'[{context._topic_prefix}] [1st CallElevator, bad params] origination: {origination}, origination_door: {origination_door}')  # noqa
                return False

            context._target_floor = origination
            context._target_door = origination_door
        else:
            if destination is None or destination_door is None:
                self._logger.error(f'[{context._topic_prefix}] [2nd CallElevator, bad params] destination: {destination}, destination_door: {destination_door}')  # noqa
                return False

            context._target_floor = destination
            context._target_door = destination_door

        payload: dict = {'direction': direction}

        if origination is not None and origination_door is not None:
            payload.update({
                'origination': origination,
                'origination_door': origination_door
            })

        if destination is not None and destination_door is not None:
            payload.update({
                'destination': destination,
                'destination_door': destination_door
            })

        return self._sync_execute_with_retry(context, 'CallElevator', payload,
                                             response_timeout_sec=10, wait_response=True,
                                             max_retry_num=3, retry_interval_sec=5)

    def do_request_elevator_status(self, context: LciElevatorContext) -> bool:
        return self._sync_execute_with_retry(context, 'RequestElevatorStatus', {},
                                             response_timeout_sec=5, wait_response=True,
                                             max_retry_num=1, retry_interval_sec=2)

    def do_open_door(self, context: LciDoorContext, direction: Optional[int] = None) -> bool:
        payload = {}

        if context._door_type == 'flap' and direction is not None:
            payload = {'direction': direction}

        return self._sync_execute_with_retry(context, 'OpenDoor', payload,
                                             response_timeout_sec=10, wait_response=True,
                                             max_retry_num=3, retry_interval_sec=5)

    def do_request_door_status(self, context: LciDoorContext) -> bool:
        return self._sync_execute_with_retry(context, 'RequestDoorStatus', {},
                                             response_timeout_sec=5, wait_response=True,
                                             max_retry_num=1, retry_interval_sec=2)

    def do_request_resource_status(self, context: LciSemContext) -> bool:
        return self._sync_execute_with_retry(context, 'RequestResourceStatus', {},
                                             response_timeout_sec=5, wait_response=True,
                                             max_retry_num=1, retry_interval_sec=2)


# Main routine
if __name__ == '__main__':

    if len(sys.argv) < 2:
        logging.error(
            'Please specify yaml config file as the 1 st command line parameter.')
        sys.exit(1)

    lci_client = LciClient()

    if not lci_client.initialize(config_file_path=sys.argv[1], cert_dir=sys.argv[2]):
        logging.error('LciClient initialization error')
        sys.exit(1)

    context_dict = lci_client.get_contexts()
    context: Optional[LciElevatorContext] = None
    for c in context_dict.values():
        if isinstance(c, LciElevatorContext):
            context = c
            break

    if context is None:
        logging.error('No Elevator Context in the config file')
        sys.exit(1)

    lci_client._logger.addHandler(
        logging.StreamHandler(sys.stdout)
    )
    lci_client._logger.setLevel(logging.DEBUG)

    lci_client.start()
    time.sleep(2)

    res = lci_client.do_registration(context)
    lci_client._logger.debug(res)
    time.sleep(1)

    res = lci_client.do_call_elevator(context, 'B1', '1')
    lci_client._logger.debug(res)
    time.sleep(1)

    res = lci_client.do_request_elevator_status(context)
    lci_client._logger.debug(res)
    time.sleep(1)

    res = lci_client.do_request_elevator_status(context)
    lci_client._logger.debug(res)
    time.sleep(1)

    res = lci_client.do_robot_status(context, RobotStatus.HAS_ENTERED)
    lci_client._logger.debug(res)
    time.sleep(1)

    res = lci_client.do_robot_status(context, RobotStatus.KEEP_DOOR_OPEN)
    lci_client._logger.debug(res)
    time.sleep(1)

    res = lci_client.do_robot_status(context, RobotStatus.HAS_GOT_OFF)
    lci_client._logger.debug(res)
    time.sleep(1)

    res = lci_client.do_release(context)
    lci_client._logger.debug(res)
    time.sleep(1)
