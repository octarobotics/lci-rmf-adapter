# Copyright (c) 2024- Octa Robotics, Inc. All Rights Reserved.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rmf_lift_msgs.msg import LiftState, LiftRequest
from rcl_interfaces.msg import ParameterDescriptor

import threading
from enum import Enum
import time


class LiftUseState(Enum):
    INIT = 'init'
    WAIT_FOR_ARRIVAL_TO_ORIGINATION = 'origination'
    WAIT_FOR_ARRIVAL_TO_DESTINATION = 'destination'


class LciRmfAdapterLiftTest(Node):
    _lift_name: str
    _lift_state: LiftState
    _lift_state_lock: threading.Lock

    _origination: str
    _destination: str

    _use_state: LiftUseState

    def __init__(self) -> None:
        super().__init__('LciRmfLiftAdapterLiftTest')

        # Parameters used in this Node
        self.declare_parameter('lift_name', '',
                               ParameterDescriptor(description='Lift name "/lci/<bldg_id>/<bank_id>/<elevator_id>" to specify LCI-ed elevator.'))
        self.declare_parameter('origination',  '',
                               ParameterDescriptor(description='Floor name of the origination.'))
        self.declare_parameter('destination',  '',
                               ParameterDescriptor(description='Floor name of the destination.'))

        self._lift_name = self.get_parameter('lift_name').value
        self._origination = self.get_parameter('origination').value
        self._destination = self.get_parameter('destination').value

        self._use_state = LiftUseState.INIT

        self.get_logger().info(
            f'[{self._lift_name}] [UseLift] {self._origination} to {self._destination}')

        self._lift_state = LiftState()
        self._lift_state_lock = threading.Lock()

        state_qos_profile = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.SYSTEM_DEFAULT)

        request_qos_profile = QoSProfile(
            depth=0, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._lift_state_sub = self.create_subscription(
            LiftState,
            'lift_states',
            self._lift_state_callback,
            qos_profile=state_qos_profile)

        self._lift_request_pub = self.create_publisher(
            LiftRequest,
            'adapter_lift_requests',
            qos_profile=request_qos_profile)

        self._sequence = self.create_timer(
            1.0, self._periodic)

    def _lift_state_callback(self, msg: LiftState):
        if msg.lift_name != self._lift_name:
            return
        with self._lift_state_lock:
            self._lift_state = msg

    def request_lift(self, destination_floor: str):
        msg = LiftRequest()
        msg.lift_name = self._lift_name
        msg.request_time = self.get_clock().now().to_msg()
        msg.session_id = self.get_name()
        msg.request_type = LiftRequest.REQUEST_AGV_MODE
        msg.destination_floor = destination_floor
        self._lift_request_pub.publish(msg)

    def release_lift(self):
        msg = LiftRequest()
        msg.lift_name = self._lift_name
        msg.request_time = self.get_clock().now().to_msg()
        msg.session_id = self.get_name()
        msg.request_type = LiftRequest.REQUEST_END_SESSION

        self._lift_request_pub.publish(msg)

    def is_arrived(self, target_floor: str):
        with self._lift_state_lock:
            return self._lift_state.session_id == self.get_name() and \
                self._lift_state.current_mode == LiftState.MODE_AGV and \
                self._lift_state.motion_state == LiftState.MOTION_STOPPED and \
                self._lift_state.door_state == LiftState.DOOR_OPEN and \
                self._lift_state.current_floor == target_floor

    def _periodic(self):
        match self._use_state:
            case LiftUseState.INIT:
                # reset to assure for LiftAdapter and LCI to start from the init state.
                self.release_lift()
                time.sleep(1)

                self.get_logger().info(
                    f'[{self._lift_name}] [CarCall] {self._origination}')
                self.request_lift(self._origination)
                self._use_state = LiftUseState.WAIT_FOR_ARRIVAL_TO_ORIGINATION

            case LiftUseState.WAIT_FOR_ARRIVAL_TO_ORIGINATION:
                if self.is_arrived(self._origination):
                    self.get_logger().info(
                        f'[{self._lift_name}] [CarArrival] {self._origination}')

                    # The robot may enter the cage here

                    self.get_logger().info(
                        f'[{self._lift_name}] [CarCall] {self._destination}')
                    self.request_lift(self._destination)
                    self._use_state = LiftUseState.WAIT_FOR_ARRIVAL_TO_DESTINATION

            case LiftUseState.WAIT_FOR_ARRIVAL_TO_DESTINATION:
                if self.is_arrived(self._destination):
                    self.get_logger().info(
                        f'[{self._lift_name}] [CarArrival] {self._destination}')

                    # The robot may get off the cage here

                    self.get_logger().info(
                        f'[{self._lift_name}] [Terminate]')

                    self.release_lift()
                    raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    lci_rmf_lift_adapter_lift_test = LciRmfAdapterLiftTest()
    try:
        rclpy.spin(lci_rmf_lift_adapter_lift_test)
    except SystemExit:
        pass
    lci_rmf_lift_adapter_lift_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
