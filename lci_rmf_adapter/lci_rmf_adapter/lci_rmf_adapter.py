# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rmf_lift_msgs.msg import LiftState, LiftRequest
from rmf_door_msgs.msg import DoorState, DoorRequest, DoorMode
from std_msgs.msg import Bool
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.publisher import Publisher

from . import lci_client
import threading
from typing import Any
from abc import ABC, abstractmethod
import time
from typing import Callable
from queue import Queue
from enum import Enum


class RmfContext(ABC):
    _logger: Any

    _lci_context: lci_client.LciContext

    _is_occupied: bool
    _occupant_id: str
    _lock: threading.RLock

    # request queue to be dispatched from request_callback of Node
    # It is to serialize commands to LCI
    _request_queue: Queue[tuple[Callable, tuple]]

    # To limit RequestElevatorStatus and RequestDoorStatus when it is in processing.
    _in_request_proccessing: threading.Event

    # Flas to surpress RequestElevatorStatus or RequestDoorStatus when resetting.
    _under_resetting: bool

    # To detect a failure of RMF
    _last_recv_request_time: float

    def _log_info(self, message: str) -> None:
        self._logger.info(
            f'[{self._lci_context._topic_prefix}] {message}')

    def _log_error(self, message: str) -> None:
        self._logger.error(
            f'[{self._lci_context._topic_prefix}] {message}')

    def _log_debug(self, message: str) -> None:
        self._logger.debug(
            f'[{self._lci_context._topic_prefix}] {message}')

    def _log_warning(self, message: str) -> None:
        self._logger.warning(
            f'[{self._lci_context._topic_prefix}] {message}')

    def __init__(self, lci_context: lci_client.LciContext, logger=None) -> None:
        self._logger = logger

        self._lci_context = lci_context

        self._occupant_id = ''
        self._lock = threading.RLock()

        self._request_queue = Queue()
        threading.Thread(target=self._request_worker, daemon=True).start()

        self._in_request_proccessing = threading.Event()

        self._under_resetting = False

        self._last_recv_request_time = 0

    def _request_worker(self) -> None:
        while True:
            func, args = self._request_queue.get()
            self._in_request_proccessing.set()
            try:
                func(*args)
            except Exception as e:
                self._log_error(
                    f"[lra] Exception in {func} args={args}: {e}")
            finally:
                self._in_request_proccessing.clear()

    def put_request(self, target: Callable, args: tuple = ()) -> None:
        self._request_queue.put((target, args))

    def set_occupant(self, occupant_id: str) -> bool:
        with self._lock:
            if self.has_occupant() and self._occupant_id != occupant_id:
                return False

            self._occupant_id = occupant_id
            return True

    def reset(self) -> None:
        with self._lock:
            self._occupant_id = ''
            self._under_resetting = False
            self._last_recv_request_time = 0

    def get_occupant(self) -> str:
        return self._occupant_id

    def has_occupant(self) -> bool:
        return self._occupant_id != ''

    @abstractmethod
    def get_status(self) -> LiftState | DoorState | list[DoorState]:
        pass

    def need_to_hide_offline_mode(self, deadtime: float) -> bool:
        """
        LciContext does not automatically recover to connected status and emergency status unless sending a message.
        To induce RMF to send LiftRequest or DoorRequest, actual MODE_OFFLINE and MODE_EMERGENCY are to be hidden by MODE_AGV or MODE_CLOSED after enough duration.
        """
        current_time = time.time()
        return ((not self._lci_context.is_connected() and self._lci_context._last_disconnected_timestamp + deadtime < current_time) or
                (not self._lci_context.is_available() and self._lci_context._last_unavailable_timestamp + deadtime < current_time))


class RmfLiftContext(RmfContext):
    _lci_context: lci_client.LciElevatorContext
    _rmf_floor_list: list[str]

    # buffer to neglect mutiple same LiftRequest
    _destination_floor: str

    # Flag to disguise door_open_state to be closed for any reason
    _hide_door_open_state: bool

    def __init__(self, lci_context: lci_client.LciElevatorContext, logger=None) -> None:
        super().__init__(lci_context, logger)

        # convolve door direction into floor_name
        self._rmf_floor_list = []
        for f in self._lci_context._floor_list:
            floor_name = self._lci_context.get_floor_name_alias(f.floor_name)
            if f.has_front_door:
                self._rmf_floor_list.append(floor_name)
            if f.has_rear_door:
                self._rmf_floor_list.append(f'{floor_name}_r')

        self._destination_floor = ''

        self._hide_door_open_state = False

    def reset(self) -> None:
        with self._lock:
            super().reset()
            self._destination_floor = ''
            self._hide_door_open_state = False

    def _get_status(self) -> LiftState:
        lift_state = LiftState()
        lift_state.lift_name = self._lci_context._topic_prefix
        lift_state.available_floors = self._rmf_floor_list

        lift_state.available_modes = [
            LiftState.MODE_OFFLINE, LiftState.MODE_AGV, LiftState.MODE_EMERGENCY, LiftState.MODE_UNKNOWN]

        if not self._lci_context.is_connected():
            lift_state.current_mode = LiftState.MODE_OFFLINE
            lift_state.door_state = LiftState.DOOR_CLOSED
            return lift_state

        if not self._lci_context.is_available():
            lift_state.current_mode = LiftState.MODE_EMERGENCY
            lift_state.door_state = LiftState.DOOR_CLOSED
            return lift_state

        if time.time() < self._lci_context._last_get_unregisterd_timestamp + 10.0:
            """
            Lift and LCI require a deadtime for resetting sequence.
            To surpress LiftRequest until the end of deadtime 10s, MODE_UNKNOWN is used instead of MODE_AGV.

            This filter also works to force the interval of re-trial of sequence.
            """
            lift_state.current_mode = LiftState.MODE_UNKNOWN
            lift_state.door_state = LiftState.DOOR_CLOSED
            return lift_state

        lift_state.current_mode = LiftState.MODE_AGV

        # use floor name alias if specified in config
        current_floor = self._lci_context.get_floor_name_alias(
            self._lci_context._current_floor)
        target_floor = self._lci_context.get_floor_name_alias(
            self._lci_context._target_floor)

        # encode door direction to floor_name
        match self._lci_context._current_door:
            case 1:  # Front door opened
                lift_state.current_floor = current_floor
                lift_state.door_state = LiftState.DOOR_OPEN
            case 2:  # Rear door opened
                lift_state.current_floor = f'{current_floor}_r'  # noqa
                lift_state.door_state = LiftState.DOOR_OPEN
            case _:
                lift_state.current_floor = current_floor
                lift_state.door_state = LiftState.DOOR_CLOSED

        if not self._lci_context.is_registered() or self._hide_door_open_state:
            # door_state must be masked as DOOR_CLOSED
            # because _lci_context._current_door is not updated when it is not registered.
            # It also needed several seconds after CallElevator.
            lift_state.door_state = LiftState.DOOR_CLOSED

        if self._lci_context._target_door == 2:
            lift_state.destination_floor = f'{target_floor}_r'  # noqa
        else:
            lift_state.destination_floor = target_floor

        if lift_state.current_floor == lift_state.destination_floor and lift_state.door_state == LiftState.DOOR_OPEN:
            lift_state.motion_state = LiftState.MOTION_STOPPED
        else:
            lift_state.motion_state = LiftState.MOTION_UNKNOWN

        # lift_state.lift_time shall be set with rclpy.Time

        lift_state.session_id = self._occupant_id

        return lift_state

    def get_status(self) -> LiftState:
        lift_state = self._get_status()

        if self.need_to_hide_offline_mode(30.0):
            # To induce RMF to send LiftRequest, hide with MODE_AGV
            lift_state.current_mode = LiftState.MODE_AGV

        return lift_state

    def set_destination_floor(self, destination_floor: str) -> None:
        with self._lock:
            self._destination_floor = destination_floor

    def get_destination_floor(self) -> str:
        return self._destination_floor


class RmfDoorContext(RmfContext):
    _lci_context: lci_client.LciDoorContext
    _is_door_open_asked: bool

    def __init__(self, lci_context: lci_client.LciDoorContext, logger=None) -> None:
        super().__init__(lci_context, logger)
        self._is_door_open_asked = False

    def reset(self) -> None:
        with self._lock:
            super().reset()
            self._is_door_open_asked = False

    def _get_status(self) -> DoorState:
        door_state = DoorState()
        door_state.door_name = self._lci_context.get_topic_prefix_alias()

        if not self._lci_context.is_connected():
            door_state.current_mode.value = DoorMode.MODE_OFFLINE
            return door_state

        if not self._lci_context.is_available():
            door_state.current_mode.value = DoorMode.MODE_UNKNOWN
            return door_state

        if time.time() < self._lci_context._last_get_unregisterd_timestamp + 10.0:
            """
            Door and LCI require a deadtime for resetting sequence.
            To surpress DoorRequest until the end of deadtime 10s, MODE_UNKNOWN is used instead of MODE_CLOSED.

            This filter also works to force the interval of re-trial of sequence.
            """
            door_state.current_mode.value = DoorMode.MODE_UNKNOWN
            return door_state

        if not self._lci_context.is_registered():
            # current_mode must be masked as MODE_CLOSED
            door_state.current_mode.value = DoorMode.MODE_CLOSED
            return door_state

        if self._lci_context._current_door == 0:
            door_state.current_mode.value = DoorMode.MODE_CLOSED
        else:
            door_state.current_mode.value = DoorMode.MODE_OPEN

        # door_state.door_time shall be set with rclpy.Time

        return door_state

    def get_status(self) -> DoorState:
        door_state = self._get_status()

        if self.need_to_hide_offline_mode(10.0):
            # To induce RMF to send DoorRequest, hide with MODE_CLOSED
            door_state.current_mode.value = DoorMode.MODE_CLOSED

        return door_state

    def is_door_open_asked(self) -> bool:
        return self._is_door_open_asked

    def set_door_open_asked(self, onoff: bool) -> None:
        self._is_door_open_asked = onoff


class SemAgentState(Enum):
    WAIT_FOR_1st_VDOOR_OPEN = 0
    WAIT_FOR_1st_VDOOR_CLOSE = 1
    WAIT_FOR_2nd_VDOOR_OPEN = 2
    WAIT_FOR_2nd_VDOOR_CLOSE = 3

    def is_open_requested(self) -> bool:
        return self in (SemAgentState.WAIT_FOR_1st_VDOOR_CLOSE, SemAgentState.WAIT_FOR_2nd_VDOOR_CLOSE)

    def is_wait_for_open_request(self) -> bool:
        return self in (SemAgentState.WAIT_FOR_1st_VDOOR_OPEN, SemAgentState.WAIT_FOR_2nd_VDOOR_OPEN)

    def is_wait_for_close_request(self) -> bool:
        return self in (SemAgentState.WAIT_FOR_1st_VDOOR_CLOSE, SemAgentState.WAIT_FOR_2nd_VDOOR_CLOSE)

    def is_robot_in_area(self) -> bool:
        return self in (SemAgentState.WAIT_FOR_1st_VDOOR_CLOSE, SemAgentState.WAIT_FOR_2nd_VDOOR_OPEN)

    def need_check_request_timeout(self) -> bool:
        return self.is_open_requested()

    @staticmethod
    def initial_state() -> "SemAgentState":
        return SemAgentState.WAIT_FOR_1st_VDOOR_OPEN

    def open_requested(self) -> "SemAgentState":
        match self:
            case SemAgentState.WAIT_FOR_1st_VDOOR_OPEN:
                return SemAgentState.WAIT_FOR_1st_VDOOR_CLOSE
            case SemAgentState.WAIT_FOR_2nd_VDOOR_OPEN:
                return SemAgentState.WAIT_FOR_2nd_VDOOR_CLOSE
            case _:
                return self

    def close_requested(self) -> "SemAgentState":
        match self:
            case SemAgentState.WAIT_FOR_1st_VDOOR_CLOSE:
                return SemAgentState.WAIT_FOR_2nd_VDOOR_OPEN
            case SemAgentState.WAIT_FOR_2nd_VDOOR_CLOSE:
                return SemAgentState.initial_state()
            case _:
                return self


class RmfSemContext(RmfContext):
    _lci_context: lci_client.LciSemContext

    _num_of_vdoor: int
    _target_vdoor_id: int
    _sem_vdoor_state: SemAgentState

    def __init__(self, lci_context: lci_client.LciSemContext, logger=None) -> None:
        super().__init__(lci_context, logger)
        self._num_of_vdoor = lci_context._num_of_vdoor
        self._target_vdoor_id = 0
        self._sem_vdoor_state = SemAgentState.initial_state()

    def reset(self) -> None:
        with self._lock:
            super().reset()
            self._target_vdoor_id = 0
            self._sem_vdoor_state = SemAgentState.initial_state()

    def _create_status_list(self) -> list[DoorState]:
        ret = []
        for i in range(self._num_of_vdoor):
            door_state = DoorState()
            door_state.door_name = f'{self._lci_context._topic_prefix}/{i+1}'
            door_state.current_mode.value = DoorMode.MODE_CLOSED
            ret.append(door_state)
        return ret

    def is_request_acceptable(self) -> bool:
        return self.need_to_hide_offline_mode(10.0) or (self._lci_context.is_connected() and self._lci_context.is_available())

    def get_status(self) -> list[DoorState]:
        # All status are MODE_CLOSED here
        ret = self._create_status_list()

        if not self.is_request_acceptable():
            for st in ret:
                st.current_mode.value = DoorMode.MODE_OFFLINE
            return ret

        self._log_debug(
            f'target_vdoor: {self._target_vdoor_id}, is_registered: {self._lci_context.is_registered()}, occupant: {self.get_occupant()}, is_open_requested: {self._sem_vdoor_state.is_open_requested()}')

        # Virtual door status can be seen by other requester_id (other sender Nodes).
        # To avoid for other robots to wrongly enter to the area, the user shall assure to keep the sender Node single
        # by using Mutex Group of RMF.
        if self._lci_context.is_registered() and self.has_occupant() and not self._under_resetting and self._sem_vdoor_state.is_open_requested() and self._target_vdoor_id != 0:
            ret[self._target_vdoor_id - 1].current_mode.value = DoorMode.MODE_OPEN

        return ret

    def open_request_received(self, target_vdoor_id: int) -> bool:
        with self._lock:
            if self._sem_vdoor_state.is_wait_for_open_request():
                self._sem_vdoor_state = self._sem_vdoor_state.open_requested()
                self._target_vdoor_id = target_vdoor_id
                return True
        return False

    def close_request_received(self, target_vdoor_id: int) -> bool:
        with self._lock:
            if self._sem_vdoor_state.is_wait_for_close_request() and self._target_vdoor_id == target_vdoor_id:
                self._sem_vdoor_state = self._sem_vdoor_state.close_requested()
                self._target_vdoor_id = 0
                return True
        return False


class LciRmfAdapter(Node):
    _lci_client: lci_client.LciClient

    _lift_context_dict: dict[str, RmfLiftContext]
    _door_context_dict: dict[str, RmfDoorContext]
    _sem_context_dict: dict[str, RmfSemContext]

    _lift_state_pub: Publisher
    _door_state_pub: Publisher
    _fire_alarm_pub: Publisher

    _device_name_separater: str

    def __init__(self) -> None:
        super().__init__('LciRmfAdapter')

        # Parameters used in this Node
        self.declare_parameter('lci_server_config', '',
                               ParameterDescriptor(description='Path to server_config.yaml provided by Octa Robotics, which includes a single elevator setting.'))
        self.declare_parameter('lci_cert_dir',  '',
                               ParameterDescriptor(description='Path to a certificate directory including certificate files (*.pem, ClientID) provided by Octa Robotics.'))
        self.declare_parameter('lci_device_name_separater',  '/',
                               ParameterDescriptor(description='Separater character(s) of lift_name and door_name used within RMF (default: "/")'))
        self.declare_parameter('rmf_lift_requests_topic',  'adapter_lift_requests',
                               ParameterDescriptor(description='Topic name for LiftRequest used within RMF (default: "adapter_lift_requests")'))
        self.declare_parameter('rmf_door_requests_topic',  'adapter_door_requests',
                               ParameterDescriptor(description='Topic name for DoorRequest used within RMF (default: "adapter_door_requests")'))

        # Initializing LciClient
        # ros2 run lci_rmf_lift_adapter lci_rmf_lift_adapter --ros-args -p "lci_server_config:=<path to server_config.yaml>" -p "lci_cert_dir:=<path to cert dir including pem files>"
        server_config_file = self.get_parameter(
            'lci_server_config').value
        cert_dir = self.get_parameter('lci_cert_dir').value
        self._device_name_separater = self.get_parameter(
            'lci_device_name_separater').value
        lift_requests_topic = self.get_parameter(
            'rmf_lift_requests_topic').value
        door_requests_topic = self.get_parameter(
            'rmf_door_requests_topic').value

        self.get_logger().info(
            f'Use "{server_config_file}" and "{cert_dir}"')

        self._lci_client = lci_client.LciClient(self.get_logger())
        self._lci_client.initialize(server_config_file, cert_dir)

        lci_context_dict = self._lci_client.get_contexts()
        lci_alarm_context_dict = self._lci_client.get_alarm_contexts()

        self._lift_context_dict = {}
        self._door_context_dict = {}
        self._sem_context_dict = {}

        for name, l_context in lci_context_dict.items():
            if isinstance(l_context, lci_client.LciElevatorContext):
                self._lift_context_dict.update(
                    {name: RmfLiftContext(l_context, self.get_logger())})
            elif isinstance(l_context, lci_client.LciDoorContext):
                # LCI format topic_prefix is used as the context name.
                # It includes floor_id in the formart of "1F, 2F, ..."
                # When floor name alias was set, it should be used as door_name from RMF
                name_alias = l_context.get_topic_prefix_alias()
                self._door_context_dict.update(
                    {name_alias: RmfDoorContext(l_context, self.get_logger())})
            elif isinstance(l_context, lci_client.LciSemContext):
                self._sem_context_dict.update(
                    {name: RmfSemContext(l_context, self.get_logger())})

        def publish_fire_alarm(alarm_type: str, area_id: str, alarm_state: bool):
            if not alarm_state:
                # If "false" is published to "/fire_alarm_trigger", RMF resumes the tasks suspended by the first alarm.
                # This behavior is not safe especially when emergency situation.
                # Robot systems should resume only after human surveillance on the emergency situation.
                # Then, the human should manually send "false" by other means such as "rmf-web".
                # That is why the cancel signal is never published by LCI RFM Adapter.
                return

            if alarm_type != 'fire_alarm':
                return

            msg = Bool()
            msg.data = True
            self._fire_alarm_pub.publish(msg)
            self.get_logger().info(f'[/fire_alarm_trigger] {area_id}')

        for al_context in lci_alarm_context_dict.values():
            # set callbacks to publish fire_alarm
            al_context.alarm_callback = publish_fire_alarm

        self._lci_client.start()

        # Initializing ROS2 messaging
        pub_state_qos_profile = QoSProfile(
            depth=15, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.SYSTEM_DEFAULT)

        pub_request_qos_profile = QoSProfile(
            depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Message loss happened for more than 10 sec during a seesion. It was detected timeout by lci_rmf_adaptery and resulted in unintended resetting.
        # Relaxing reliability with BEST_EFFORT may improve this message loss.
        #
        # Some loss of message is not a problem.
        # LiftRequest and DoorRequest are sent in 1 Hz.
        # Their end of session with LiftRequest::REQUEST_END_SESSION and DoorMode::MODE_CLOSED are to be re-published until LiftState and DoorState indicate "reset".
        # https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/src/lift_supervisor/Node.cpp#L110
        # https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/src/door_supervisor/Node.cpp#L140
        sub_state_qos_profile = QoSProfile(
            depth=15, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.SYSTEM_DEFAULT)

        sub_request_qos_profile = QoSProfile(
            depth=0, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # To publish lift status to RMF
        self._lift_state_pub = self.create_publisher(
            LiftState,
            'lift_states',
            qos_profile=pub_state_qos_profile)

        # To publish door status to RMF
        self._door_state_pub = self.create_publisher(
            DoorState,
            'door_states',
            qos_profile=pub_state_qos_profile)

        # To publish fire alarm to RMF
        # RMF subscribes fire_alarm_trigger with TRANSIENT_LOCAL
        # https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/src/rmf_fleet_adapter/agv/Node.cpp#L73-L75
        self._fire_alarm_pub = self.create_publisher(
            Bool,
            'fire_alarm_trigger',
            qos_profile=pub_request_qos_profile)

        # Subscribe lift requests from RMF
        # https://osrf.github.io/ros2multirobotbook/integration_lifts.html
        self._lift_request_sub = self.create_subscription(
            LiftRequest,
            lift_requests_topic,
            self._lift_request_callback,
            qos_profile=sub_request_qos_profile)

        # Subscribe door requests from RMF
        # https://osrf.github.io/ros2multirobotbook/integration_doors.html
        # Strangely, the publisher of DoorRequest of RMF is created with default_qos: ReliabilityPolicy.RELIABLE and DurabilityPolicy.SYSTEM_DEFAULT.
        # https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/src/rmf_fleet_adapter/agv/Node.cpp#L49-L51
        # It seems a kind of bug in accordance with their design policy.
        # Anyway, LCI RMF Adapter is needed to compromise to accept relaxed durability policy.
        self._door_request_sub = self.create_subscription(
            DoorRequest,
            door_requests_topic,
            self._door_request_callback,
            qos_profile=sub_state_qos_profile)

        # To publish all LiftState and DoorState every second.
        self._pub_rmf_state_timer = self.create_timer(
            1.0, self._publish_rmf_states)

        # To send RequestElevatorStatus and RequestDoorStatus to LCI every 3 seconds when registered.
        self._sync_lci_status_timer = self.create_timer(
            2.0, self._sync_lci_status)

    def _publish_rmf_states(self):
        current_time = self.get_clock().now().to_msg()

        for rl_context in self._lift_context_dict.values():
            lift_state = rl_context.get_status()
            lift_state.lift_name = lift_state.lift_name.replace(
                '/', self._device_name_separater)
            lift_state.lift_time = current_time
            self._lift_state_pub.publish(lift_state)

        for rd_context in self._door_context_dict.values():
            door_state = rd_context.get_status()
            door_state.door_name = door_state.door_name.replace(
                '/', self._device_name_separater)
            door_state.door_time = current_time
            self._door_state_pub.publish(door_state)

        for rs_context in self._sem_context_dict.values():
            door_state_list = rs_context.get_status()
            for door_state in door_state_list:
                door_state.door_name = door_state.door_name.replace(
                    '/', self._device_name_separater)
                door_state.door_time = current_time
                self._door_state_pub.publish(door_state)

    def _sync_elevator_status(self, rl_context: RmfLiftContext) -> None:
        rl_context._log_debug('[lra] RequestElevatorStatus, start')
        ret = self._lci_client.do_request_elevator_status(
            rl_context._lci_context)

        if not ret:
            rl_context._log_debug(
                '[lra] RequestElevatorStatus, failed')
            rl_context.reset()
            return
        rl_context._log_debug('[lra] RequestElevatorStatus, success')

    def _sync_door_status(self, rd_context: RmfDoorContext) -> None:
        rd_context._log_debug('[lra] RequestDoorStatus, start')
        ret = self._lci_client.do_request_door_status(
            rd_context._lci_context)

        if not ret:
            rd_context._log_debug('[lra] RequestDoorStatus, failed')
            rd_context.reset()
            return
        rd_context._log_debug('[lra] RequestDoorStatus, success')

    def _sync_sem_status(self, rs_context: RmfSemContext) -> None:
        rs_context._log_debug('[lra] RequestResourceStatus, start')
        ret = self._lci_client.do_request_resource_status(
            rs_context._lci_context)

        if not ret:
            rs_context._log_debug('[lra] RequestResourceStatus, failed')
            rs_context.reset()
            return
        rs_context._log_debug('[lra] RequestResourceStatus, success')

    def _sync_lci_status(self):
        for rl_context in self._lift_context_dict.values():
            if (rl_context._lci_context.is_registered() and not rl_context._under_resetting and
                    rl_context._request_queue.empty() and not rl_context._in_request_proccessing.is_set()):
                rl_context.put_request(
                    self._sync_elevator_status, (rl_context,))

            if rl_context._lci_context.is_registered() and rl_context._last_recv_request_time + 10.0 < time.time():
                # RMF may be in hang. Timeout
                rl_context._log_error(
                    f'[lra, lift_req timeout] {rl_context.get_occupant()}')
                self.reset_lift(rl_context)

        for rd_context in self._door_context_dict.values():
            if (rd_context._lci_context.is_registered() and not rd_context._under_resetting and
                    rd_context._request_queue.empty() and not rd_context._in_request_proccessing.is_set()):
                rd_context.put_request(
                    self._sync_door_status, (rd_context,))

            if rd_context._lci_context.is_registered() and rd_context._last_recv_request_time + 10.0 < time.time():
                # RMF may be in hang. Timeout
                rd_context._log_error(
                    f'[lra, door_req timeout] {rd_context.get_occupant()}')
                self.reset_door(rd_context)

        for rs_context in self._sem_context_dict.values():
            if (rs_context._lci_context.is_registered() and not rs_context._under_resetting and
                    rs_context._request_queue.empty() and not rs_context._in_request_proccessing.is_set()):
                rs_context.put_request(
                    self._sync_sem_status, (rs_context,))

            if rs_context._lci_context.is_registered() and (rs_context._sem_vdoor_state.need_check_request_timeout() and
                                                            rs_context._last_recv_request_time + 10.0 < time.time()):
                # RMF may be in hang. Timeout
                rs_context._log_error(
                    f'[lra, door_req (sem) timeout] {rs_context.get_occupant()}')
                self.reset_sem(rs_context)

    ####
    # Lift

    def _lift_request_callback(self, msg: LiftRequest) -> None:
        lift_name_key = msg.lift_name.replace(self._device_name_separater, '/')

        rl_context = self._lift_context_dict.get(lift_name_key, None)
        if rl_context is None:
            return

        lift_state = rl_context.get_status()
        if lift_state.current_mode != LiftState.MODE_AGV:
            # neglect LiftRequest unless it is ready state == MODE_AGV
            return

        # session_id means the sender Node name
        if rl_context.get_occupant() == '':
            rl_context.set_occupant(msg.session_id)

        elif rl_context.get_occupant() != msg.session_id:
            rl_context._log_warning(
                f'[lra, lift_req, session_id mismatch] owned: {rl_context.get_occupant()}, req: {msg.session_id}')
            return

        match msg.request_type:
            case LiftRequest.REQUEST_END_SESSION:
                # REQUEST_END_SESSION should be always accepted.
                rl_context._log_info(
                    '[lra, lift_req accepted] REQUEST_END_SESSION')
                self.reset_lift(rl_context)

            case LiftRequest.REQUEST_AGV_MODE:
                """
                When the robot is at the lift hall, LiftRequest.destination_floor shall be set in '<origination>' or '<origination>:<destination>'.
                Especially when the backend lift API behind LCI requires <destination> information, '<origination>:<destination>' format is mandatory for the 1st CallElevator.

                After the robot entered the cage, LiftRequest.destination_floor shall be set in '<destination>' or '<destination>:<destination>'.

                Example 1:
                - At the lift hall: '1F'
                - In the cage: '3F'

                Example 2:
                - At the lift hall: '1F:3F'
                - In the cage: '3F'

                Example 3:
                - At the lift hall: '1F:3F'
                - In the cage: '3F:3F'
                """
                target_floor_list = msg.destination_floor.split(':')

                match len(target_floor_list):
                    case 1:
                        pass
                    case 2:
                        if not rl_context._lci_context.is_registered() and target_floor_list[0] == target_floor_list[1]:
                            rl_context._log_error(
                                f'[lra, lift_req, format error] destination_floor: {msg.destination_floor}. <origination> and <destination> indicated the same floor.')
                            return
                    case _:
                        rl_context._log_error(
                            f'[lra, lift_req, format error] destination_floor: {msg.destination_floor}. It must be <origination> for the 1st request, <destination> for the 2nd request or <origination>:<destination>.')
                        return

                for tf in target_floor_list:
                    if tf not in rl_context._rmf_floor_list:
                        rl_context._log_error(
                            f'[lra, lift_req, invalid floor name] {tf} is not in lift')
                        return

                rl_context._last_recv_request_time = time.time()

                if rl_context.get_destination_floor() == msg.destination_floor:
                    # Because RMF sends same LiftRequest in 1 Hz, lci-rmf-adapter has to neglect those redundant requests by checking whether LiftRequest.destination_floor changes.
                    return

                # Set destination_floor to detect its change
                rl_context.set_destination_floor(msg.destination_floor)

                if not rl_context._lci_context.is_registered():

                    # Registraiton and 1st CallElevator
                    if len(target_floor_list) == 2:
                        origination = target_floor_list[0]
                        destination = target_floor_list[1]
                    else:
                        origination = target_floor_list[0]
                        destination = None

                    rl_context._log_info(
                        '[lra, lift_req accepted] Registration, 1st CallElevator')
                    self.register_and_call_lift(
                        rl_context, origination, destination)
                    return

                else:
                    """
                    Here, changing msg.destination_floor means the robot entered the lift and requests destination floor.
                    That is, RobotStatus (HAS_ENTERED) and 2nd CallElevator are to be sent.
                    """

                    if len(target_floor_list) == 2:
                        destination = target_floor_list[1]
                    else:
                        destination = target_floor_list[0]

                    rl_context._log_info(
                        '[lra, lift_req accepted] RobotStatus.HAS_ENTERED, 2nd CallElevator')
                    self.enter_and_call_lift(rl_context, destination)
                    return

            case _:
                rl_context._log_warning(
                    f'[lra, lift_req, unsupported request_type] {msg.request_type}')

    def _prepare_call_elevator_params(self, origination: str | None, destination: str | None) -> tuple[str | None, str | None, int | None, int | None]:
        origination_door = None
        destination_door = None

        # decode door direction from floor_name
        if origination is not None:
            if origination.endswith('_r'):
                origination_door = 2
                origination = origination[0:-2]
            else:
                origination_door = 1

        if destination is not None:
            if destination.endswith('_r'):
                destination_door = 2
                destination = destination[0:-2]
            else:
                destination_door = 1
        return origination, destination, origination_door, destination_door

    def _register_and_call_lift(self, rl_context: RmfLiftContext, origination: str | None, destination: str | None) -> None:
        rl_context._log_info('[lra] init sync by RequestElevatorStatus')
        ret = self._lci_client.do_request_elevator_status(
            rl_context._lci_context)

        if ret:
            # Previous session was alive. Release is needed.
            rl_context._log_info(
                '[lra] old session is alive. Release')
            ret = self._lci_client.do_release(rl_context._lci_context)
            if not ret:
                rl_context._log_error('[lra] Release failed')
                rl_context.reset()
                return
            rl_context._log_info('[lra] Old session is stopped.')
            time.sleep(5)

        elif not rl_context._lci_context.is_connected():
            # do_request_elevator_status() failed because MQTT was disconnected.
            rl_context._log_error(
                '[lra] init sync failed. no connection.')
            rl_context.reset()
            return

        rl_context._log_debug('[lra] Registration, start')
        rl_context._hide_door_open_state = True
        ret = self._lci_client.do_registration_elevator(
            rl_context._lci_context)

        if not ret:
            rl_context._log_debug('[lra] Registration, failed')
            rl_context.reset()
            return
        rl_context._log_debug('[lra] Registration, success')

        origination, destination, origination_door, destination_door = self._prepare_call_elevator_params(
            origination, destination)

        # convert to LCI format floor_id
        if origination is not None:
            origination = rl_context._lci_context.get_lci_floor_name_by_alias(
                origination)

        if destination is not None:
            destination = rl_context._lci_context.get_lci_floor_name_by_alias(
                destination)

        rl_context._log_debug('[lra] 1st CallElevator, start')
        ret = self._lci_client.do_call_elevator(
            rl_context._lci_context,
            origination, destination,
            origination_door, destination_door)

        if not ret:
            rl_context._log_debug('[lra] 1st CallElevator, failed')
            rl_context.reset()
            return
        rl_context._log_debug('[lra] 1st CallElevator, success')

        # Hide door_open_state 2s after CallElevator to wait CarArrival and updated ElevatorStatus by RequestElevatorStatus
        time.sleep(2)

        self._sync_elevator_status(rl_context)
        rl_context._hide_door_open_state = False

    def register_and_call_lift(self, rl_context: RmfLiftContext, origination: str | None, destination: str | None) -> None:
        rl_context.put_request(
            target=self._register_and_call_lift, args=(rl_context, origination, destination))

    def _enter_and_call_lift(self, rl_context: RmfLiftContext, destination: str | None) -> None:
        rl_context._log_debug('[lra] RobotStatus.HAS_ENTERED, start')
        ret = self._lci_client.do_robot_status(
            rl_context._lci_context, lci_client.RobotStatus.HAS_ENTERED)

        if not ret:
            rl_context._log_debug('[lra] RobotStatus.HAS_ENTERED, failed')
            rl_context.reset()
            return
        rl_context._log_debug('[lra] RobotStatus.HAS_ENTERED, success')

        _, destination, _, destination_door = self._prepare_call_elevator_params(
            None, destination)

        # convert to LCI format floor_id
        if destination is not None:
            destination = rl_context._lci_context.get_lci_floor_name_by_alias(
                destination)

        rl_context._log_debug('[lra] 2nd CallElevator, start')
        ret = self._lci_client.do_call_elevator(
            rl_context._lci_context,
            None, destination,
            None, destination_door)

        if not ret:
            rl_context._log_debug('[lra] 2nd CallElevator, failed')
            rl_context.reset()
            return
        rl_context._log_debug('[lra] 2nd CallElevator, success')

    def enter_and_call_lift(self, rl_context: RmfLiftContext, destination: str | None) -> None:
        rl_context.put_request(
            target=self._enter_and_call_lift, args=(rl_context, destination))

    def _reset_lift(self, rl_context: RmfLiftContext) -> None:
        if rl_context._lci_context.is_registered():
            rl_context._under_resetting = True

            # RMF Lift API does not have information where the robot is. Then, HAS_GOT_OFF used for LCI to reset.

            rl_context._log_debug('[lra] RobotStatus HAS_GOT_OFF')
            self._lci_client.do_robot_status(
                rl_context._lci_context,
                lci_client.RobotStatus.HAS_GOT_OFF)
            time.sleep(1)

            rl_context._log_debug('[lra] Release')
            self._lci_client.do_release(rl_context._lci_context)

        rl_context.reset()

    def reset_lift(self, rl_context: RmfLiftContext) -> None:
        rl_context.put_request(target=self._reset_lift, args=(rl_context,))

    ####
    # Door

    def _door_request_callback(self, msg: DoorRequest) -> None:
        door_name_key: str = msg.door_name.replace(
            self._device_name_separater, '/')

        # door: `/lci/<bldg_id>/<floor_id>/<door_id>`
        # sem:  `/lci/<bldg_id>/sem/<resource_id>/<virtual door_id>`
        parts = door_name_key.split('/')
        if parts[3] == "sem":
            # This DoorRequest are to be handled by LCI Sem
            sem_name_key = '/'.join(parts[:-1])
            try:
                vdoor_id = int(parts[-1])
            except:
                return
            return self._sem_request_callback(sem_name_key, vdoor_id, msg)

        rd_context = self._door_context_dict.get(door_name_key, None)
        if rd_context is None:
            return

        door_state = rd_context.get_status()
        if door_state.current_mode.value in [DoorMode.MODE_UNKNOWN, DoorMode.MODE_OFFLINE]:
            return

        # requester_id means the sender Node name
        if rd_context.get_occupant() == '':
            rd_context.set_occupant(msg.requester_id)

        elif rd_context.get_occupant() != msg.requester_id:
            rd_context._log_warning(
                f'[lra, door_req, requester_id mismatch] owned: {rd_context.get_occupant()}, req: {msg.requester_id}')
            return

        match msg.requested_mode.value:
            case DoorMode.MODE_CLOSED:
                rd_context._log_info('[lra, door_req accepted] MODE_CLOSED')
                self.reset_door(rd_context)

            case DoorMode.MODE_OPEN:
                rd_context._last_recv_request_time = time.time()

                if rd_context.is_door_open_asked():
                    # LCI do not need multiple OpenDoor command because LCI holds the request of OpenDoor as an internal state.
                    return

                rd_context.set_door_open_asked(True)

                direction = None
                if rd_context._lci_context._door_type == 'flap':
                    # Although LCI supports directional doors, such as flap barrier turnstiles, which do not allow reverse passing, rmf_door_msgs does not.
                    # To manage this lack of functionality, lci_rmf_adapter assumes the entering direction (passing from a low-security area to a high-security area)
                    # is also applicable for the leaving direction (passing from a high-security area to a low-security area).
                    # When leaving, if the door detects reverse passing and closes itself, please use the correct direciton value `2` by extending LciRmfAdapter or other means.

                    # direction: 1 means the entering direciton
                    direction = 1

                    # direciton: 2 means the leaving direction
                    # direction = 2

                if not rd_context._lci_context.is_registered():
                    self.register_and_open_door(rd_context, direction)
                    return

            case _:
                rd_context._log_warning(
                    f'[lra, door_req, unsupported requested_mode] {msg.requested_mode.value}')

    def _register_and_open_door(self, rd_context: RmfDoorContext, direction: int | None) -> None:
        rd_context._log_info('[lra] init sync by RequestDoorStatus')
        ret = self._lci_client.do_request_door_status(
            rd_context._lci_context)

        if ret:
            # Previous session was alive. Release is needed.
            rd_context._log_info(
                '[lra] old session is alive. Release')
            ret = self._lci_client.do_release(rd_context._lci_context)
            if not ret:
                rd_context._log_error('[lra] Release failed')
                rd_context.reset()
                return
            rd_context._log_info('[lra] Old session is stopped.')
            time.sleep(5)

        elif not rd_context._lci_context.is_connected():
            # do_request_elevator_status() failed because MQTT was disconnected.
            rd_context._log_error(
                '[lra] init sync failed. no connection.')
            rd_context.reset()
            return

        rd_context._log_debug('[lra] Registration, start')

        ret = self._lci_client.do_registration_door(
            rd_context._lci_context)

        if not ret:
            rd_context._log_debug('[lra] Registration, failed')
            rd_context.reset()
            return
        rd_context._log_debug('[lra] Registration, success')

        rd_context._log_debug('[lra] OpenDoor, start')

        ret = self._lci_client.do_open_door(
            rd_context._lci_context, direction)

        if not ret:
            rd_context._log_debug('[lra] OpenDoor, failed')
            rd_context.reset()
            return
        rd_context._log_debug('[lra] OpenDoor, success')

    def register_and_open_door(self, rd_context: RmfDoorContext, direction: int | None) -> None:
        rd_context.put_request(
            target=self._register_and_open_door, args=(rd_context, direction))

    def _reset_door(self, rd_context: RmfDoorContext) -> None:
        if rd_context._lci_context.is_registered():
            rd_context._under_resetting = True

            rd_context._log_debug('[lra] Release')
            self._lci_client.do_release(rd_context._lci_context)

        rd_context.reset()

    def reset_door(self, rd_context: RmfDoorContext) -> None:
        rd_context.put_request(target=self._reset_door, args=(rd_context,))

    ####
    # Sem

    def _sem_request_callback(self, sem_name_key: str, vdoor_id: int, msg: DoorRequest) -> None:
        rs_context = self._sem_context_dict.get(sem_name_key, None)
        if rs_context is None:
            return

        if vdoor_id <= 0 or rs_context._num_of_vdoor < vdoor_id:
            return

        if not rs_context.is_request_acceptable():
            return

        # requester_id means the sender Node name
        if rs_context.get_occupant() == '':
            rs_context.set_occupant(msg.requester_id)

        elif rs_context.get_occupant() != msg.requester_id:
            rs_context._log_warning(
                f'[lra, door_req (sem/{vdoor_id}), requester_id mismatch] owned: {rs_context.get_occupant()}, req: {msg.requester_id}')
            return

        match msg.requested_mode.value:
            case DoorMode.MODE_CLOSED:
                if rs_context.close_request_received(vdoor_id):
                    rs_context._log_info(
                        f'[lra, door_req (sem/{vdoor_id}) accepted] MODE_CLOSED')

                    if not rs_context._sem_vdoor_state.is_robot_in_area():
                        # When the robot is outside of the area, Release shall be sent to LCI and reset.
                        self.reset_sem(rs_context)
                        return

            case DoorMode.MODE_OPEN:
                rs_context._last_recv_request_time = time.time()

                if rs_context.open_request_received(vdoor_id):
                    rs_context._log_info(
                        f'[lra, door_req (sem/{vdoor_id}) accepted] MODE_OPEN')

                    if not rs_context._lci_context.is_registered():
                        self.register_sem(rs_context)
                        return

            case _:
                rs_context._log_warning(
                    f'[lra, door_req (sem/{vdoor_id}), unsupported requested_mode] {msg.requested_mode.value}')

    def _register_sem(self, rs_context: RmfSemContext) -> None:
        if not rs_context.is_request_acceptable():
            # do_request_elevator_status() failed because MQTT was disconnected.
            rs_context._log_error(
                '[lra, sem] No connection.')
            rs_context.reset()
            return

        rs_context._log_debug('[lra, sem] Registration, start')

        ret = self._lci_client.do_registration_sem(
            rs_context._lci_context)

        if not ret:
            rs_context._log_debug('[lra, sem] Registration, failed')
            rs_context.reset()
            return
        rs_context._log_debug('[lra, sem] Registration, success')

    def register_sem(self, rs_context: RmfSemContext) -> None:
        rs_context.put_request(
            target=self._register_sem, args=(rs_context,))

    def _reset_sem(self, rs_context: RmfSemContext) -> None:
        if rs_context._lci_context.is_registered():
            rs_context._under_resetting = True

            rs_context._log_debug('[lra, sem] Release')
            self._lci_client.do_release(rs_context._lci_context)

        rs_context.reset()

    def reset_sem(self, rs_context: RmfSemContext) -> None:
        rs_context.put_request(target=self._reset_sem, args=(rs_context,))


def main(args=None):
    rclpy.init(args=args)
    lci_rmf_adapter = LciRmfAdapter()
    try:
        rclpy.spin(lci_rmf_adapter)
    finally:
        for rl_context in lci_rmf_adapter._lift_context_dict.values():
            lci_rmf_adapter.reset_lift(rl_context)
        for rd_context in lci_rmf_adapter._door_context_dict.values():
            lci_rmf_adapter.reset_door(rd_context)

        lci_rmf_adapter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
