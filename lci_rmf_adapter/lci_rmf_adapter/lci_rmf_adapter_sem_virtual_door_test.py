# Copyright (c) 2026- Octa Robotics, Inc. All Rights Reserved.

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rmf_door_msgs.msg import DoorState, DoorRequest, DoorMode
from rcl_interfaces.msg import ParameterDescriptor

import threading
from enum import Enum


class SemVirtualDoorUseState(Enum):
    OPEN_1st_VDOOR = 'open_vdoor_1'
    OPEN_2nd_VDOOR = 'open_vdoor_2'

    def get_target_virtual_door_id(self, requester_index: int) -> int:
        if self is SemVirtualDoorUseState.OPEN_1st_VDOOR:
            if requester_index == 1:
                return 1
            else:
                return 2
        else:
            if requester_index == 1:
                return 3
            else:
                return 1


# simulated Mutex Group of RMF
lock = threading.Lock()
who_has_access_right = 0


class LciRmfAdapterSemVirtualDoorTest(Node):
    _resource_name: str
    _requester_index: int
    _door_name: str
    _door_state: DoorState
    _door_state_lock: threading.Lock

    _use_state: SemVirtualDoorUseState

    def __init__(self, requester_index: int) -> None:
        super().__init__(f'LciRmfAdapterTest_{requester_index}')
        self._requester_index = requester_index

        # Parameters used in this Node
        self.declare_parameter('resource_name', '',
                               ParameterDescriptor(description='Resource name "/lci/<bldg_id>/sem/<resource_id>" to specify a resource managed by LCI Sem'))

        self._resource_name = self.get_parameter('resource_name').value

        self._use_state = SemVirtualDoorUseState.OPEN_1st_VDOOR
        virtual_door_id = self._use_state.get_target_virtual_door_id(
            self._requester_index)
        self._door_name = f"{self._resource_name}/{virtual_door_id}"

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

        self.get_logger().info(
            f'[{self._requester_index}] [{self._door_name}] [current_mode] {self._door_state.current_mode.value}')

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
            return self._door_state.door_name == self._door_name and self._door_state.current_mode == DoorMode(value=DoorMode.MODE_OPEN)

    def _periodic(self):
        # simulated Mutex Group of RMF
        # By commenting out this block, the race condition easily happens.
        global who_has_access_right
        with lock:
            if who_has_access_right == 0:
                who_has_access_right = self._requester_index
            elif who_has_access_right != self._requester_index:
                return

        match self._use_state:
            case SemVirtualDoorUseState.OPEN_1st_VDOOR:

                if self.is_open():
                    self.get_logger().info(
                        f'[{self._requester_index}] [{self._door_name}] [DoorOpened]')

                    # The robot may pass the door here

                    self.get_logger().info(
                        f'[{self._requester_index}] [{self._door_name}] [CloseDoor]')
                    self.request_door(open_close=False)

                    # set next target
                    self._use_state = SemVirtualDoorUseState.OPEN_2nd_VDOOR
                    virtual_door_id = self._use_state.get_target_virtual_door_id(
                        self._requester_index)
                    self._door_name = f"{self._resource_name}/{virtual_door_id}"

                else:
                    self.get_logger().info(
                        f'[{self._requester_index}] [{self._door_name}] [OpenDoor]')
                    self.request_door(open_close=True)

            case SemVirtualDoorUseState.OPEN_2nd_VDOOR:

                if self.is_open():
                    self.get_logger().info(
                        f'[{self._requester_index}] [{self._door_name}] [DoorOpened]')

                    # The robot may pass the door here

                    self.get_logger().info(
                        f'[{self._requester_index}] [{self._door_name}] [CloseDoor]')
                    self.request_door(open_close=False)

                    self.get_logger().info(
                        f'[{self._requester_index}] [{self._door_name}] [Terminate]')

                    with lock:
                        who_has_access_right = 0
                    self._sequence.cancel()
                else:
                    self.get_logger().info(
                        f'[{self._requester_index}] [{self._door_name}] [OpenDoor]')
                    self.request_door(open_close=True)


def main(args=None):
    rclpy.init(args=args)
    lci_rmf_adapter_test_1 = LciRmfAdapterSemVirtualDoorTest(requester_index=1)
    lci_rmf_adapter_test_2 = LciRmfAdapterSemVirtualDoorTest(requester_index=2)

    executor = MultiThreadedExecutor()
    executor.add_node(lci_rmf_adapter_test_1)
    executor.add_node(lci_rmf_adapter_test_2)

    try:
        executor.spin()
    finally:
        executor.shutdown()

        lci_rmf_adapter_test_1.destroy_node()
        lci_rmf_adapter_test_2.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
