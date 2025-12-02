# lci_rmf_adapter (RMF Lift and Door Adapter for LCI)
This package is an implementation of RMF Lift Adapter[^1] and Door Adapter[^2] for LCI.

LCI is the cloud-based service provided by Octa Robotics, Inc[^3] .

It supports,
- Multiple vendors of elevators (lifts) including Mitsubishi, Hitachi, Toshiba, Fujitec, OTIS and more
- Multiple vendors of doors and turnstiles including Nabco, Teraoka, Kumahira and more

The base protocol of LCI is RFA[^4] on MQTT.

This package bridges ROS2 (RMF Lift and Door Adapter) to LCI.


[^1]: https://osrf.github.io/ros2multirobotbook/integration_lifts.html
[^2]: https://osrf.github.io/ros2multirobotbook/integration_doors.html
[^3]: https://www.octa8.jp/service/
[^4]: RFA is the acronym of the Robot Friendly Asset Promotion Association of Japan. RFA publishes the standards for the interfaces between robots and elevators/doors. RFA standards are available for purchase from [their homepage](https://robot-friendly.org/publication/). English version is also available.


# System requirements
- ROS2 Humble or Jazzy
- Python packages
  - paho-mqtt == 1.6.1
  - ruamel.yaml

The more detail is exemplified in [Dockerfile](Dockerfile).


# Topics
| Message Types             | ROS2 Topic               | Description                                                              |
| ------------------------- | ------------------------ | ------------------------------------------------------------------------ |
| rmf_lift_msgs/LiftState   | `/lift_states`           | State of the lift published by the lift node                             |
| rmf_lift_msgs/LiftRequest | `/adapter_lift_requests` | Requests to be sent to the lift supervisor to request operation of lifts |
| rmf_door_msgs/DoorState   | `/door_states`           | State of the door published by the door node                             |
| rmf_door_msgs/DoorRequest | `/adapter_door_requests` | Requests to be sent to the door supervisor to request operation of doors |

# Limitation
## For Lift
- When the robot is at the origination floor, `LiftRequest.destination_floor` shall be `"<origination>"` or `"<origination>:<destination>"`.
- After the robot entered the car, `LiftRequest.destination_floor` shall be `"<destination>"` or `"<destination>:<destination>"`.
- **`"<origination>:<destination>"` format is required especially when the backend lift API of LCI requires both `"<origination>"` and `"<destination>"` at the first request.**
- The suffix `_r` is used for the floor name to specify the rear door. This rule applies `<origination>`, `<destination>`, `LiftRequest.destination_floor` and `LiftRequest.current_floor`.

For example, if the robot wants to use the lift from `B1` of the front door to `8F` of the rear door, `LiftRequest.destination_floor` can be,

  |           | At the origination floor | After entering the car |
  | --------- | ------------------------ | ---------------------- |
  | Example 1 | `B1`                     | `8F_r`                 |
  | Example 2 | `B1:8F_r`                | `8F_r`                 |
  | Example 3 | `B1:8F_r`                | `8F_r:8F_r`            |


- The arrival at the final destination can be detected by checking if `LiftRequest.current_floor` and `LiftRequest.door_state` are `8F_r` and `LiftState.DOOR_OPEN` respectively.

## For Door

Although LCI supports directional doors (e.g. flap barrier turnstiles), which do not allow reverse passing, rmf_door_msgs does not.

To make up for this lack of functionality, lci_rmf_adapter assumes the entering direction (passing from a low-security area to a high-security area) is also applicable for the leaving direction (passing from a high-security area to a low-security area).

In case that the door detects reverse passing and closes itself when the robot is leaving via the door, please use the correct direction value `2` by extending LciRmfAdapter or other means.
See *if*-block starting with `if rd_context._lci_context._door_type == 'flap':` in [lci_rmf_adapter.py](lci_rmf_adapter/lci_rmf_adapter/lci_rmf_adapter.py).

The directional doors are marked with `lci_door_type: flap` in the config files of LCI (see [LCI files](#lci-files)).


# LCI files
To configure lci_rmf_adapter, the following files are needed.

- `server_config.yaml` : Config file to specify what elevators and doors are robot-ready.
- Cert files: ClientID, client certificate and private key of an LCI Robot Account.

All of them will be provided by Octa Robotics.

[server_config_simulator.yaml](lci_config/server_config_simulator.yaml) is the config file for the simulator provided by Octa Robotics.

Please contact lci@octa8.jp to obtain the LCI Robot Account for development, which is only allowed to access the simulator.


# Docker
To use lci_rmf_adapter quickly, it is recommended to use Docker.

[container_manager.sh](container_manager.sh) provides an utility for Docker.

- Edit `ROS_DISTRO` in [container_manager.sh](container_manager.sh) to use a distribution other than humble.
- Edit `LCI_SERVER_CONFIG_YAML` and `LCI_CERT_DIR` in [start.sh](start.sh) to fit your environment.
- `sudo ./container_manager.sh start`
  - It will build and start the container.
- `sudo ./container_manager.sh login`
  - It will login you in the container environment.
- `./start.sh`
  - It will start the Node of lci_rmf_adapter.



# Client examples
## For Lift

ROS2 Node example is found in [the test code for lift](lci_rmf_adapter/lci_rmf_adapter/lci_rmf_adapter_lift_test.py).

Its entry point is [test_client_lift.sh](test_client_lift.sh)

Please edit `LIFT_NAME`, `ORIGINATION` and `DESTINATION` in [test_client_lift.sh](test_client_lift.sh) for trial.

`LIFT_NAME` is of the format `/lci/<bldg_id>/<bank_id>/<elevator_id>`.

## For Door

ROS2 Node example is found in [the test code for door](lci_rmf_adapter/lci_rmf_adapter/lci_rmf_adapter_door_test.py).

Its entry point is [test_client_door.sh](test_client_door.sh)

Please edit `DOOR_NAME` in [test_client_door.sh](test_client_door.sh) for trial

`DOOR_NAME` is of the format `/lci/<bldg_id>/<floor_id>/<door_id>`.


# Use LCI directly

LCI does not only support the elevators (lifts) and the doors but also,
- signals from alarm systems
- automatic setting and releasing of security systems
- the exclusive control of resoruces among multiple robot systems (**LCI Sem**)
- the messaging service between robots and humans (**LCI Bell**)

To use full functionaly of LCI, please use MQTT.

[lci_client.py](lci_rmf_adapter/lci_rmf_adapter/lci_client.py) is an implementation of LCI client without ROS2.
