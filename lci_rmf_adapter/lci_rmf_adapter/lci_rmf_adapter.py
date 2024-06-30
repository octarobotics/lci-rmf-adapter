# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rmf_lift_msgs.msg import LiftState, LiftRequest
from rmf_door_msgs.msg import DoorState, DoorRequest, DoorMode
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.publisher import Publisher

from . import lci_client
import threading
from typing import Any
from abc import ABC, abstractmethod
import time


class RmfContext(ABC):
    _logger: Any

    _lci_context: lci_client.LciContext

    _is_occupied: bool
    _occupant_id: str
    _lock: threading.Lock

    def __init__(self, lci_context: lci_client.LciContext, logger=None) -> None:
        self._logger = logger

        self._lci_context = lci_context
        self._lci_context.reset_callback = self.reset_occupant

        self._is_occupied = False
        self._occupant_id = ''
        self._lock = threading.Lock()

    def set_occupant(self, occupant_id: str) -> bool:
        with self._lock:
            if self._is_occupied and self._occupant_id != occupant_id:
                return False

            self._is_occupied = True
            self._occupant_id = occupant_id
            return True

    def reset_occupant(self) -> None:
        with self._lock:
            self._is_occupied = False
            self._occupant_id = ''

    def get_occupant(self) -> str:
        return self._occupant_id

    @abstractmethod
    def get_status(self) -> LiftState | DoorState:
        pass


class RmfLiftContext(RmfContext):
    _lci_context: lci_client.LciElevatorContext
    _rmf_floor_list: list[str]

    def __init__(self, lci_context: lci_client.LciElevatorContext, logger=None) -> None:
        super().__init__(lci_context, logger)

        self._rmf_floor_list = [
            x.floor_name for x in self._lci_context._floor_list]

    def get_status(self) -> LiftState:
        lift_state = LiftState()
        lift_state.lift_name = self._lci_context._topic_prefix
        lift_state.available_floors = [
            x.floor_name for x in self._lci_context._floor_list]
        lift_state.current_floor = self._lci_context._current_floor
        lift_state.destination_floor = self._lci_context._target_floor

        if self._lci_context._current_door in [1, 2]:
            lift_state.door_state = LiftState.DOOR_OPEN
        else:
            lift_state.door_state = LiftState.DOOR_CLOSED

        if lift_state.current_floor == lift_state.destination_floor and lift_state.door_state == LiftState.DOOR_OPEN:
            lift_state.motion_state = LiftState.MOTION_STOPPED
        else:
            lift_state.motion_state = LiftState.MOTION_UNKNOWN

        lift_state.available_modes = [
            LiftState.MODE_OFFLINE, LiftState.MODE_AGV, LiftState.MODE_EMERGENCY]

        if self._lci_context._is_available:
            lift_state.current_mode = LiftState.MODE_AGV
        else:
            lift_state.current_mode = LiftState.MODE_EMERGENCY

        # lift_state.current_mode will be overwritten with LiftState.MODE_OFFLINE if _lci_client is not connected.
        # lift_state.lift_time shall be set with rclpy.Time

        lift_state.session_id = self._occupant_id

        return lift_state


class RmfDoorContext(RmfContext):
    _lci_context: lci_client.LciDoorContext

    def __init__(self, lci_context: lci_client.LciDoorContext, logger=None) -> None:
        super().__init__(lci_context, logger)

    def get_status(self) -> DoorState:
        door_state = DoorState()
        door_state.door_name = self._lci_context._topic_prefix

        if self._lci_context._current_door == 0:
            door_state.current_mode = DoorMode(value=DoorMode.MODE_CLOSED)
        else:
            door_state.current_mode = DoorMode(value=DoorMode.MODE_OPEN)

        # door_state.current_mode will be overwritten with DoorMode.MODE_OFFLINE if _lci_client is not connected.
        # door_state.door_time shall be set with rclpy.Time

        return door_state


class LciRmfAdapter(Node):
    _lci_client: lci_client.LciClient

    _lift_context_dict: dict[str, RmfLiftContext]
    _door_context_dict: dict[str, RmfDoorContext]

    _lift_state_pub: Publisher
    _door_state_pub: Publisher

    def __init__(self) -> None:
        super().__init__('LciRmfAdapter')

        # Parameters used in this Node
        self.declare_parameter('lci_server_config', '',
                               ParameterDescriptor(description='Path to server_config.yaml provided by Octa Robotics, which includes a single elevator setting.'))
        self.declare_parameter('lci_cert_dir',  '',
                               ParameterDescriptor(description='Path to a certificate directory including certificate files (*.pem, ClientID) provided by Octa Robotics.'))

        # Initializing LciClient
        # ros2 run lci_rmf_lift_adapter lci_rmf_lift_adapter --ros-args -p "lci_server_config:=<path to server_config.yaml>" -p "lci_cert_dir:=<path to cert dir including pem files>"
        server_config_file = self.get_parameter(
            'lci_server_config').value
        cert_dir = self.get_parameter('lci_cert_dir').value

        self.get_logger().info(
            f'Use "{server_config_file}" and "{cert_dir}"')

        self._lci_client = lci_client.LciClient(self.get_logger())
        self._lci_client.initialize(server_config_file, cert_dir)

        lci_context_dict = self._lci_client.get_contexts()

        self._lift_context_dict = {}
        self._door_context_dict = {}

        for name, l_context in lci_context_dict.items():
            match l_context.get_device_type():
                case lci_client.DeviceType.ELEVATOR:
                    self._lift_context_dict.update(
                        {name: RmfLiftContext(l_context, self.get_logger())})
                case lci_client.DeviceType.DOOR:
                    self._door_context_dict.update(
                        {name: RmfDoorContext(l_context, self.get_logger())})

        self._lci_client.start()

        # Initializing ROS2 messaging
        state_qos_profile = QoSProfile(
            depth=15, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.SYSTEM_DEFAULT)

        request_qos_profile = QoSProfile(
            depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # To publish lift status to RMF
        self._lift_state_pub = self.create_publisher(
            LiftState,
            'lift_states',
            qos_profile=state_qos_profile)

        # To publish door status to RMF
        self._door_state_pub = self.create_publisher(
            DoorState,
            'door_states',
            qos_profile=state_qos_profile)

        # Subscribe lift requests from RMF
        # https://osrf.github.io/ros2multirobotbook/integration_lifts.html
        self._lift_request_sub = self.create_subscription(
            LiftRequest,
            'adapter_lift_requests',
            self._lift_request_callback,
            qos_profile=request_qos_profile)

        # Subscribe door requests from RMF
        # https://osrf.github.io/ros2multirobotbook/integration_doors.html
        self._door_request_sub = self.create_subscription(
            DoorRequest,
            'adapter_door_requests',
            self._door_request_callback,
            qos_profile=request_qos_profile)

        # To publish all LiftState and DoorState every second.
        self._pub_rmf_state_timer = self.create_timer(
            1.0, self._publish_rmf_states)

        # To send RequestElevatorStatus and RequestDoorStatus to LCI every 3 seconds when registered.
        self._sync_lci_status_timer = self.create_timer(
            3.0, self._sync_lci_status)

    def _publish_rmf_states(self):
        current_time = self.get_clock().now().to_msg()
        is_connected = self._lci_client._mqtt_client.is_connected()

        for rl_context in self._lift_context_dict.values():
            lift_state = rl_context.get_status()

            if not is_connected:
                lift_state.current_mode = LiftState.MODE_OFFLINE
            lift_state.lift_time = current_time

            self._lift_state_pub.publish(lift_state)

        for rd_context in self._door_context_dict.values():
            door_state = rd_context.get_status()

            if not is_connected:
                door_state.current_mode = DoorMode(value=DoorMode.MODE_OFFLINE)
            door_state.door_time = current_time

            self._door_state_pub.publish(door_state)

    def _sync_lci_status(self):
        for rl_context in self._lift_context_dict.values():
            if rl_context._lci_context._is_registered:
                self._lci_client.do_request_elevator_status(
                    rl_context._lci_context)

        for rd_context in self._door_context_dict.values():
            if rd_context._lci_context._is_registered:
                self._lci_client.do_request_door_status(
                    rd_context._lci_context)

    def _lift_request_callback(self, msg: LiftRequest) -> None:
        rl_context = self._lift_context_dict.get(msg.lift_name, None)
        if rl_context is None:
            return

        # session_id means the sender Node name
        if rl_context.get_occupant() == '':
            rl_context.set_occupant(msg.session_id)

        elif rl_context.get_occupant() != msg.session_id:
            self.get_logger().warning(
                f'[{msg.lift_name}] session_id mismatch: session is owned by {rl_context.get_occupant()} but requested from {msg.session_id}')
            return

        match msg.request_type:
            case LiftRequest.REQUEST_END_SESSION:
                self.reset_lift(rl_context)
                self.get_logger().info(
                    f'[{msg.lift_name}] Release')

            case LiftRequest.REQUEST_AGV_MODE:
                # destination_floor must be '<origination>:<destination>' format string
                # If the robot want to move from '1F' to '3F', destination_floor is '1F:3F'
                target_floor_list = msg.destination_floor.split(':')
                if len(target_floor_list) != 2:
                    self.get_logger().error(
                        f'[{msg.lift_name}] Format error of destination_floor ({msg.destination_floor}). It must be <origination>:<destination>')
                    return

                for tf in target_floor_list:
                    if tf not in rl_context._rmf_floor_list:
                        self.get_logger().error(
                            f'[{msg.lift_name}] Invalid floor name {tf}. It is not in floor_list)')
                        return

                if not rl_context._lci_context._is_registered:
                    res = self._lci_client.do_registration(
                        rl_context._lci_context)
                    if not res or not rl_context._lci_context._is_registered:
                        self.get_logger().warning(
                            f'[{msg.lift_name}] Registration failed: {res}')
                        return
                    self.get_logger().info(
                        f'[{msg.lift_name}] Registration')

                origination = target_floor_list[0]
                destination = target_floor_list[1]

                if rl_context._lci_context._target_floor == '':
                    # 1st CallElevator when the robot may be out of the cage.
                    self.get_logger().info(
                        f'[{msg.lift_name}] 1st CallElevator: {origination} to {destination}')
                else:
                    # 2nd CallElevator when the robot may be in the cage.
                    self._lci_client.do_robot_status(
                        rl_context._lci_context,
                        lci_client.RobotStatus.HAS_ENTERED)
                    self.get_logger().info(
                        f'[{msg.lift_name}] 2nd CallElevator: {origination} to {destination}')

                    time.sleep(1)

                res = self._lci_client.do_call_elevator(
                    rl_context._lci_context,
                    origination, destination)
                if not res:
                    self.get_logger().error(
                        f'[{msg.lift_name}] CallElevator failed')
                    self.reset_lift(rl_context)
                    self.get_logger().info(
                        f'[{msg.lift_name}] Release')

            case _:
                self.get_logger().warning(
                    f'[{msg.lift_name}] request_type {msg.request_type} is not supported.')

    def reset_lift(self, rl_context: RmfLiftContext):
        if rl_context._lci_context._is_registered:
            # RMF Lift API does not have information where the robot is. Then, HAS_GOT_OFF used for LCI to reset.
            self._lci_client.do_robot_status(
                rl_context._lci_context,
                lci_client.RobotStatus.HAS_GOT_OFF)
            time.sleep(1)

        self._lci_client.do_release(rl_context._lci_context)
        rl_context.reset_occupant()

    def _door_request_callback(self, msg: DoorRequest) -> None:
        rd_context = self._door_context_dict.get(msg.door_name, None)
        if rd_context is None:
            return

        # requester_id means the sender Node name
        if rd_context.get_occupant() == '':
            rd_context.set_occupant(msg.requester_id)

        elif rd_context.get_occupant() != msg.requester_id:
            self.get_logger().warning(
                f'[{msg.door_name}] requester_id mismatch: session is owned by {rd_context.get_occupant()} but requested from {msg.requester_id}')
            return

        match msg.requested_mode.value:
            case DoorMode.MODE_CLOSED:
                self.reset_door(rd_context)
                self.get_logger().info(f'[{msg.door_name}] Release')

            case DoorMode.MODE_OPEN:
                if not rd_context._lci_context._is_registered:
                    res = self._lci_client.do_registration(
                        rd_context._lci_context)
                    if not res or not rd_context._lci_context._is_registered:
                        self.get_logger().warning(
                            f'[{msg.door_name}] Registration failed: {res}')
                        return
                    self.get_logger().info(
                        f'[{msg.door_name}] Registration')

                res = self._lci_client.do_open_door(rd_context._lci_context)
                if not res:
                    self.get_logger().error(
                        f'[{msg.door_name}] OpenDoor failed')
                    self.reset_door(rd_context)
                    return
                self.get_logger().info(f'[{msg.door_name}] OpenDoor')

    def reset_door(self, rd_context: RmfDoorContext):
        self._lci_client.do_release(rd_context._lci_context)
        rd_context.reset_occupant()


def main(args=None):
    rclpy.init(args=args)
    lci_rmf_lift_adapter = LciRmfAdapter()
    rclpy.spin(lci_rmf_lift_adapter)
    lci_rmf_lift_adapter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
