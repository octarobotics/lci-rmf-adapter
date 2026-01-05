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

- To use jazzy, `export ROS_DISTRO=jazzy`.
- To use CycloneDDS, `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
  - If you want to use other cyclonedds.xml than cyclonedds.xml.default, `export DDS_CONFIG=<path to your cyclonedds.xml>`
- To use other ROS_DOMAIN_ID than 0, `export ROS_DOMAIN_ID=<your ROS_DOMAIN_ID>`.
- If you do not want to use the environment variables, edit [docker-compose.yml](docker-compose.yml) directly.
- Edit `LCI_SERVER_CONFIG_YAML` and `LCI_CERT_DIR` in [start.sh](start.sh) to fit your environment.
- `sudo -E ./container_manager.sh start`
  - It will build and start the container.
  - Do not forget `-E` option when you use the environment variables to configure lci_rmf_adapter.
- `sudo ./container_manager.sh login`
  - It will login you in the container environment.
- `./start.sh`
  - It will start the Node of lci_rmf_adapter.
- To automatically run `./start.sh` by `sudo -E ./container_manager.sh start`, `export RUN_ON_START=true` in advance.


# Error management

To help RMF to manage errors releated to lifts and doors, lci_rmf_adapter uses `LiftState.current_mode` and `DoorState.current_mode` as,

 | `LiftState.current_mode` | Description                                          |
 | ------------------------ | ---------------------------------------------------- |
 | `MODE_AGV`               | LCI device for the lift is online                    |
 | `MODE_OFFLINE`           | LCI device for the lift is not reachable             |
 | `MODE_EMERGENCY`         | The lift is not available due to emergency situation |
 | `MODE_UNKNOWN`           | In the deadtime after use                            |


 | `DoorState.current_mode` | Description                                                                               |
 | ------------------------ | ----------------------------------------------------------------------------------------- |
 | `MODE_CLOSED`            | LCI device for the door is online or the door is not available due to emergency situation |
 | `MODE_OPEN`              | The door is completely opened and a robot can go through it                               |
 | `MODE_OFFLINE`           | LCI device for the door is not reachable                                                  |
 | `MODE_UNKNOWN`           | In the deadtime after use                                                                 |



LCI's Elevator API and Door API are syncronous protocol based on MQTT, the asynchronous messaging protocol. lci_rmf_adapter applys appropriate retry and timeout both for LCI layer and MQTT layer. 

In `_publish()` of [lci_client.py](./lci_rmf_adapter/lci_rmf_adapter/lci_client.py),
- Timeout to receive PUBACK for the message. Timeout duration is fixed with 5s. Retry is managed by paho-mqtt.
- Timeout to receive the response from LCI devices. Timeout durations are set by `_sync_execute_with_retry()` with the maximum number of retry as,

 | LCI's command name    | Timeout duration | Max # of retry | Retry interval |
 | --------------------- | ---------------- | -------------- | -------------- |
 | Regsitration          | 180s             | 5              | 30             |
 | CallElevator          | 10s              | 2              | 5              |
 | RequestElevatorStatus | 5s               | 0              | -              |
 | RobotStatus           | 10s              | 2              | 5              |
 | Release               | 10s              | 2              | 5              |
 
If any command failed after the specified retries, lci_rmf_adapter sets its mode and publish `LiftState.current_mode` and `DoorState.current_mode` with `MODE_OFFLINE`. In the same manner, once a response from LCI indicates emergency, lci_rmf_adapter uses `MODE_EMERGENCY` for lifts and `MODE_UNKNOWN` for doors.

After a specific duration, `MODE_OFFLINE`, `MODE_EMERGENCY` and `MODE_UNKNOWN` will be overrided by `MODE_AGV` or `MODE_CLOSED` to inform "pseudo-online" to induce RMF to send LiftRequest or DoorRequest.

After use of a lift or a door, their internal states will get unstable for several seconds for resetting. To assure this pereiod, lci_rmf_adapter uses `MODE_UNKNOWN` until 10s passed after usage.

RMF and its peripheral module shall manage these errors to assure safe operations.


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

## Separater for device names
RMF Web have a trouble with slash separated lift_name and door_name because it may require those values to be URL safe.

To avoid their limitation, the separater replacement function is supported.
See `LCI_DEVICE_NAME_SEPARATER` in [start.sh](start.sh).

# Use LCI directly

LCI does not only support the elevators (lifts) and the doors but also,
- signals from alarm systems
- automatic setting and releasing of security systems
- the exclusive control of resoruces among multiple robot systems (**LCI Sem**)
- the messaging service between robots and humans (**LCI Bell**)

To use full functionaly of LCI, please use MQTT.

[lci_client.py](lci_rmf_adapter/lci_rmf_adapter/lci_client.py) is an implementation of LCI client without ROS2.
