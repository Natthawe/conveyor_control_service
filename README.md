## How to use?

### 1. Service Node (Server)
  - ros2 run conveyor_control conveyor_service_node

### 2. Service Call (Client)
  - ros2 service call /conveyor_command conveyor_control/srv/ConveyorCommand "{command: 0}"
  - ros2 service call /conveyor_command conveyor_control/srv/ConveyorCommand "{command: 1}"
  - ros2 service call /conveyor_command conveyor_control/srv/ConveyorCommand "{command: -1}"

### Check Service
  - ros2 service list
  - ros2 service type /conveyor_command

### Service Protocal
| Command | Meaning |
| ------- | ------- |
| `-1`    | Reverse |
| `0`     | Stop    |
| `1`     | Forward |

