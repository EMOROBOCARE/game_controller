# EMOROBCARE LED Service

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Python Version](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A complete ROS 2 service package for controlling EMOROBOT LEDs with advanced effects and animations.

This package provides a powerful and easy-to-use interface to control RGB LEDs via ROS 2 services, wrapping the `emorobcare_leds` Python library. It supports individual LED control, synchronized animations, 12+ predefined effects, and custom color patterns with smooth fade transitions.

## Features

-   **Individual & Group Control**: Set single LEDs or all LEDs at once.
-   **Smooth Fade Transitions**: Configure fade times for smooth color changes.
-   **Built-in Effects**: 12+ effects like rainbow, cascade, heartbeat, and wave.
-   **State Queries**: Get the current color and status of all LEDs.
-   **Animation Control**: Easily stop, pause, and reset LED animations.
-   **Flexible Configuration**: Use environment variables or ROS 2 parameters to configure the I2C bus, address, and LED count.
-   **Robust Error Handling**: Comprehensive input validation with clear, actionable error messages.

---

## Quick Start

### 1. Installation

**Prerequisites**: First, ensure the `emorobcare_leds` library is installed system-wide.
```bash
cd /path/to/emorobcare_leds
sudo pip install -e .
```

**Build the Package**:
```bash
cd ~/your_ros2_ws
colcon build --packages-select emorobcare_led_service
source install/setup.bash
```

### 2. Start the Service Node

In one terminal, launch the service:
```bash
# Set your ROS_DOMAIN_ID if needed
export ROS_DOMAIN_ID=10
ros2 launch emorobcare_led_service led_service.launch.py
```

### 3. Control the LEDs

In a new terminal, call the services to control the LEDs.

**Set all LEDs to Red (RGB values):**
```bash
ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs \
  "{led_id: -1, r_values: [255], g_values: [0], b_values: [0], fade_time: 1.0}"
```

**Set all LEDs to Red (color name):**
```bash
ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString \
  "{led_id: -1, color_name: 'red', fade_time: 1.0}"
```

**Play the Rainbow Effect:**
```bash
ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect \
  "{effect_name: 'rainbow', duration: 5.0}"
```

**Turn Off all LEDs:**
```bash
ros2 service call /control_leds emorobcare_led_service/srv/ControlLEDs \
  "{command: 'turn_off_all', fade_time: 1.0}"
```

**Get Available Colors:**
```bash
ros2 topic echo /available_colors
```

---

## High-Level Python API

For simpler integration, use the provided `LEDClient` which accepts color names, hex codes, and RGB tuples.

### Example Usage

```python
#!/usr/bin/env python3
import rclpy
from emorobcare_led_service.led_client import LEDClient
import time

def main():
    rclpy.init()
    client = LEDClient()

    # Set all LEDs to blue with a fade
    print("Setting color to blue...")
    client.set_color('blue', fade_time=1.0)
    time.sleep(2)

    # Set individual colors
    print("Setting individual colors...")
    client.set_colors(['red', 'green', 'blue', 'yellow', 'magenta'], fade_time=1.0)
    time.sleep(3)

    # Play effects by name
    print("Playing rainbow effect...")
    client.rainbow(duration=5.0)
    
    print("Playing heartbeat effect...")
    client.heartbeat(color='purple', duration=4.0)

    # Show a progress bar
    print("Showing progress bar...")
    for i in range(11):
        client.progress_bar(progress=i/10.0, filled_color='success_green')
        time.sleep(0.3)

    # Turn off with a fade
    print("Turning off LEDs.")
    client.off(fade_time=2.0)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Supported Color Formats

The `LEDClient` accepts colors in multiple formats:
-   **Name**: `'red'`, `'warm_white'`, `'care_blue'`
-   **Hex**: `'#FF0000'`, `'#0078D7'`
-   **RGB Tuple**: `(255, 0, 0)`

---

## API Reference

The package exposes four ROS 2 services for low-level control and a high-level Python client for ease of use.

### Available Effects

| Effect Name | Description | Key Parameters |
| :--- | :--- | :--- |
| `rainbow` | Cycling rainbow colors across all LEDs. | `duration`, `speed` |
| `cascade` | A single color chasing across the LEDs. | `color1`, `direction`, `speed`, `cycles` |
| `heartbeat` | A pulsing/breathing effect. | `color1`, `duration`, `speed` |
| `wave` | A smooth wave pattern between two colors. | `color1`, `color2`, `duration`, `speed` |
| `blink` | Blinks the LEDs with a list of colors. | `colors`, `speed`, `cycles` |
| `blink_single` | Blinks only the first LED (LED 0). | `color1`, `speed`, `cycles` |
| `sparkle` | Random LEDs light up in random colors. | `colors`, `density`, `duration`, `speed` |
| `knight_rider` | A K.I.T.T.-style scanner effect. | `color1`, `speed`, `cycles` |
| `emergency` | Red and blue alternating emergency lights. | `cycles`, `speed` |
| `progress_bar` | Visual progress indicator (0.0 to 1.0). | `progress`, `color1` (filled), `color2` (empty) |
| `color_cycle` | Cycles all LEDs through a list of colors. | `colors`, `speed` (display time), `cycles` |

<br>

<details>
<summary><b>Click to see Low-Level Service Definitions</b></summary>

### Service Reference

#### `/set_leds` (`SetLEDs.srv`)
Set individual or all LED colors using RGB values with an optional fade.
-   **Request**: `led_id`, `r_values[]`, `g_values[]`, `b_values[]`, `fade_time`
-   **Response**: `success`, `message`, `current_r[]`, `current_g[]`, `current_b[]`

#### `/set_leds_string` (`SetLEDsString.srv`)
Set individual or all LED colors using predefined color names with an optional fade.
-   **Request**: `led_id`, `color_name`, `fade_time`
-   **Response**: `success`, `message`, `current_r[]`, `current_g[]`, `current_b[]`

#### `/play_effect` (`PlayEffect.srv`)
Play a predefined LED animation with customizable parameters.
-   **Request**: `effect_name`, `duration`, `speed`, `cycles`, `color1_r/g/b`, `colors_r/g/b[]`, etc.
-   **Response**: `success`, `message`, `actual_duration`

#### `/get_led_state` (`GetLEDState.srv`)
Query the current color and status of the LEDs.
-   **Request**: `get_colors`
-   **Response**: `success`, `message`, `num_leds`, `current_r/g/b[]`, `is_fading`

#### `/control_leds` (`ControlLEDs.srv`)
Stop animations or turn off all LEDs.
-   **Request**: `command` (`stop_fade`, `turn_off_all`), `fade_time`
-   **Response**: `success`, `message`

### Topics

#### `/available_colors` (`AvailableColors.msg`)
Publishes available color names and their RGB values for reference.
-   **Fields**: `color_names[]`, `r_values[]`, `g_values[]`, `b_values[]`, `descriptions[]`

### Available Color Names

The string-based service supports the following color names:

**Basic Colors**: `black`, `white`, `red`, `green`, `blue`, `yellow`, `magenta`, `cyan`

**Extended Colors**: `orange`, `pink`, `purple`, `violet`, `indigo`, `brown`, `beige`, `tan`, `gray`, `light_gray`, `dark_gray`

**White Variants**: `warm_white`, `cool_white`

**EMOROBOT Special Colors**: `care_blue`, `warning_red`, `success_green`

</details>

---

## Configuration

Configure the node using environment variables before launching:

```bash
export LED_I2C_BUS=1         # I2C bus number (default: 1)
export LED_I2C_ADDR=0x08     # I2C device address (default: 0x08)
export LED_NUM_LEDS=5        # Number of LEDs in the strip (default: 5)
```
Alternatively, pass them as ROS 2 parameters:
```bash
ros2 launch emorobcare_led_service led_service.launch.py num_leds:=5
```
---

## Examples & Testing

The `examples` directory contains scripts to demonstrate all functionalities.

```bash
cd ~/your_ros2_ws/src/emorobcare_led_service/examples

# Run a complete test of all low-level service calls (~4 minutes)
./test_all_functionalities.sh

# Run a demo using the high-level Python client
python3 high_level_client_demo.py
```

---

## Troubleshooting

-   **Service Not Found**: Ensure the service node is running (`ros2 node list`) and that the `ROS_DOMAIN_ID` is consistent across all terminals.
-   **I2C Communication Error**: Check that the Arduino is powered and correctly wired (SDA, SCL, GND). Verify the I2C address with `i2cdetect -y 1`.
-   **Permission Denied `/dev/i2c-1`**: Your user may not have I2C permissions. Add your user to the `i2c` group: `sudo usermod -aG i2c $USER` and then log out and log back in.
-   **Invalid Input Errors**: The service validates all inputs. If a call fails, the `message` field in the response will contain a detailed explanation of what went wrong (e.g., "Invalid LED ID", "RGB arrays must have equal length").
