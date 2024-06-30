# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rmf_door_msgs.msg import DoorState, DoorRequest, DoorMode
from rcl_interfaces.msg import ParameterDescriptor

import threading
from enum import Enum


class DoorUseState(Enum):
    INIT = 'init'
    WAIT_FOR_OPEN = 'open'


class LciRmfAdapterDoorTest(Node):
    _door_name: str
    _door_state: DoorState
    _door_state_lock: threading.Lock

    _use_state: DoorUseState

    def __init__(self) -> None:
        super().__init__('LciRmfLiftAdapterDoorTest')

        # Parameters used in this Node
        self.declare_parameter('door_name', '',
                               ParameterDescriptor(description='Lift name "/lci/<bldg_id>/<bank_id>/<elevator_id>" to specify LCI-ed elevator.'))

        self._door_name = self.get_parameter('door_name').value

        self._use_state = DoorUseState.INIT

        self.get_logger().info(
            f'[{self._door_name}] [UseDoor]')

        self._door_state = DoorState()
        self._door_state_lock = threading.Lock()

        state_qos_profile = QoSProfile(
            depth=15, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.SYSTEM_DEFAULT)

        request_qos_profile = QoSProfile(
            depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._door_state_sub = self.create_subscription(
            DoorState,
            'door_states',
            self._door_state_callback,
            qos_profile=state_qos_profile)

        self._door_request_pub = self.create_publisher(
            DoorRequest,
            'adapter_door_requests',
            qos_profile=request_qos_profile)

        self._sequence = self.create_timer(
            1.0, self._periodic)

    def _door_state_callback(self, msg: DoorState):
        if msg.door_name != self._door_name:
            return
        with self._door_state_lock:
            self._door_state = msg

    def request_door(self, open_close: bool):
        msg = DoorRequest()
        msg.door_name = self._door_name
        msg.request_time = self.get_clock().now().to_msg()
        msg.requester_id = self.get_name()
        if open_close:
            msg.requested_mode = DoorMode(value=DoorMode.MODE_OPEN)
        else:
            msg.requested_mode = DoorMode(value=DoorMode.MODE_CLOSED)

        self._door_request_pub.publish(msg)

    def is_open(self):
        with self._door_state_lock:
            return self._door_state.current_mode == DoorMode(value=DoorMode.MODE_OPEN)

    def _periodic(self):
        match self._use_state:
            case DoorUseState.INIT:
                self.get_logger().info(
                    f'[{self._door_name}] [OpenDoor]')
                self.request_door(open_close=True)
                self._use_state = DoorUseState.WAIT_FOR_OPEN

            case DoorUseState.WAIT_FOR_OPEN:
                if self.is_open():
                    self.get_logger().info(
                        f'[{self._door_name}] [DoorOpened]')

                    # The robot may pass the door here

                    self.get_logger().info(
                        f'[{self._door_name}] [CloseDoor]')
                    self.request_door(open_close=False)

                    self.get_logger().info(
                        f'[{self._door_name}] [Terminate]')

                    raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    lci_rmf_lift_adapter_door_test = LciRmfAdapterDoorTest()
    try:
        rclpy.spin(lci_rmf_lift_adapter_door_test)
    except SystemExit:
        pass
    lci_rmf_lift_adapter_door_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
