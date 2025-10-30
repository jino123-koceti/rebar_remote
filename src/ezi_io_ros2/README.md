# EZI-IO ROS2 Driver

ROS2 Humble driver for **FASTECH EZI-IO-EN-L16O16N-T** I/O module via Modbus TCP.

Monitors limit sensors for XYZ stage control.

## Features

- ✅ ROS2 Humble compatible
- ✅ Modbus TCP communication
- ✅ Real-time limit sensor monitoring
- ✅ Individual sensor topics
- ✅ Diagnostic messages
- ✅ Configurable update rate (default 20Hz)

## Hardware Setup

### Connections
- **Device**: FASTECH EZI-IO-EN-L16O16N-T
- **Communication**: Ethernet (Modbus TCP)
- **IP Address**: 192.168.0.3 (상부체 스테이지)
- **Port**: 502
- **Slave ID**: 1

### Network Configuration
Ensure your computer is on the same network:
```bash
# Example: Set static IP on eth0
sudo ip addr add 192.168.0.100/24 dev eth0
# or configure via NetworkManager
```

## Dependencies

```bash
sudo apt install python3-pip
pip3 install pymodbus
```

## Installation

```bash
cd ~/ros2_ws/src
# Package already in workspace
cd ~/ros2_ws
colcon build --packages-select ezi_io_ros2
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch ezi_io_ros2 ezi_io.launch.py
```

### Custom IP Address

```bash
ros2 launch ezi_io_ros2 ezi_io.launch.py ip_address:=192.168.0.10
```

### Monitor Limit Sensors

```bash
# Watch all limits
ros2 topic echo /limit_sensors/x_min
ros2 topic echo /limit_sensors/x_max
ros2 topic echo /limit_sensors/y_min
ros2 topic echo /limit_sensors/y_max
ros2 topic echo /limit_sensors/z_min
ros2 topic echo /limit_sensors/z_max

# Watch diagnostics
ros2 topic echo /ezi_io/diagnostics
```

## Topics

### Published
| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/limit_sensors/x_min` | std_msgs/Bool | 20Hz | X축 원점 리미트 (IN02) |
| `/limit_sensors/x_max` | std_msgs/Bool | 20Hz | X축 최대 리미트 (IN03) |
| `/limit_sensors/y_min` | std_msgs/Bool | 20Hz | Y축 원점 리미트 (IN01) |
| `/limit_sensors/y_max` | std_msgs/Bool | 20Hz | Y축 최대 리미트 (IN00) |
| `/limit_sensors/z_min` | std_msgs/Bool | 20Hz | Z축 원점 리미트 (IN05) |
| `/limit_sensors/z_max` | std_msgs/Bool | 20Hz | Z축 최대 리미트 (IN06) |
| `/limit_sensors/yaw_min` | std_msgs/Bool | 20Hz | Yaw축 원점 리미트 (IN04) |
| `/ezi_io/diagnostics` | diagnostic_msgs/DiagnosticArray | 20Hz | 전체 센서 상태 |

**Note**: `True` = Limit triggered (sensor activated), `False` = Clear

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ip_address` | string | 192.168.0.3 | EZI-IO IP address |
| `port` | int | 502 | Modbus TCP port |
| `slave_id` | int | 1 | Modbus slave ID |
| `update_rate` | float | 20.0 | Update frequency (Hz) |
| `input_start_address` | int | 0 | First input register |
| `input_count` | int | 16 | Number of inputs to read |
| `limit_x_min_channel` | int | 2 | X축 원점 리미트 (IN02) |
| `limit_x_max_channel` | int | 3 | X축 최대 리미트 (IN03) |
| `limit_y_min_channel` | int | 1 | Y축 원점 리미트 (IN01) |
| `limit_y_max_channel` | int | 0 | Y축 최대 리미트 (IN00) |
| `limit_z_min_channel` | int | 5 | Z축 원점 리미트 (IN05) |
| `limit_z_max_channel` | int | 6 | Z축 최대 리미트 (IN06) |
| `limit_yaw_min_channel` | int | 4 | Yaw축 원점 리미트 (IN04) |

## Channel Mapping (실제 배선)

**상부체 스테이지 리미트 센서:**

```
IN00: Y축 최대 리미트
IN01: Y축 원점 리미트
IN02: X축 원점 리미트
IN03: X축 최대 리미트
IN04: Yaw축 원점 리미트
IN05: Z축 원점 리미트
IN06: Z축 최대 리미트
```

설정 파일 (`config/ezi_io.yaml`)에 반영됨

## Troubleshooting

### Cannot connect to EZI-IO
```bash
# Ping test
ping 192.168.0.2

# Check network interface
ip addr show

# Try port scan
nmap -p 502 192.168.0.2
```

### Wrong IP address
```bash
# Launch with custom IP
ros2 launch ezi_io_ros2 ezi_io.launch.py ip_address:=YOUR_IP
```

### No data on topics
1. Check Modbus connection in logs
2. Verify input channel mapping
3. Test with `test_ezi_io.py` script first

## Integration Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class LimitMonitor(Node):
    def __init__(self):
        super().__init__('limit_monitor')
        
        # Subscribe to limit sensors
        self.create_subscription(
            Bool, '/limit_sensors/x_min',
            lambda msg: self.limit_callback('X-', msg), 10)
        
        self.create_subscription(
            Bool, '/limit_sensors/x_max',
            lambda msg: self.limit_callback('X+', msg), 10)
    
    def limit_callback(self, axis, msg):
        if msg.data:
            self.get_logger().warn(f'Limit {axis} TRIGGERED!')
            # Stop motion, etc.
```

## Safety Integration

Use limit sensors to prevent overtravel:

```python
class SafeMotionController(Node):
    def __init__(self):
        self.limits = {
            'x_min': False, 'x_max': False,
            'y_min': False, 'y_max': False,
            'z_min': False, 'z_max': False,
        }
        
        for axis in self.limits.keys():
            self.create_subscription(
                Bool, f'/limit_sensors/{axis}',
                lambda msg, a=axis: self.update_limit(a, msg), 10)
    
    def update_limit(self, axis, msg):
        self.limits[axis] = msg.data
        if msg.data:
            self.emergency_stop()
    
    def can_move(self, direction):
        # direction: 'x+', 'x-', 'y+', 'y-', 'z+', 'z-'
        axis, sign = direction[0], direction[1]
        limit_key = f'{axis}_{"max" if sign == "+" else "min"}'
        return not self.limits[limit_key]
```

## License

MIT
