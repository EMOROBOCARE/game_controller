# EMOROBCARE LED Service Examples

This directory contains a comprehensive unified test script for the EMOROBCARE LED Service that demonstrates **all high-level functionalities** using direct ROS 2 service calls.

## 🚀 Overview

The EMOROBCARE LED Service provides a complete ROS 2 interface for controlling RGB LEDs with advanced effects and anima tions. **All functionalities can be accessed directly via ROS 2 service calls** without requiring any Python client code.

## 📋 Available Services

The service exposes **4 main ROS 2 services**:

1. **`/set_leds`** (`SetLEDs`) - Set LED colors with optional fade transitions
2. **`/play_effect`** (`PlayEffect`) - Play 12+ predefined LED effects
3. **`/get_led_state`** (`GetLEDState`) - Query current LED state and colors
4. **`/control_leds`** (`ControlLEDs`) - Control animations (stop, turn off)

---

## 🧪 Unified Test Suite

### `unified_test.sh`
**Complete LED service test suite using ROS 2 service calls**

A comprehensive test script that covers **all 82 test cases** organized into 15 sections:

#### ✅ **Core Features Tested:**
- **Basic Color Control** - RED, GREEN, BLUE, WHITE, YELLOW, CYAN, MAGENTA
- **Special EMOROBOT Colors** - WARM_WHITE, COOL_WHITE, CARE_BLUE, WARNING_RED, SUCCESS_GREEN, ORANGE
- **Individual LED Control** - Set specific LEDs or all LEDs with different colors
- **LED State Queries** - Get current colors and animation status
- **12 LED Effects** - rainbow, cascade, heartbeat, wave, blink, sparkle, knight_rider, spain_flag, emergency, progress_bar, color_cycle
- **Progress Bar** - Visual progress indicators (0% to 100%)
- **Control Commands** - Stop fade animations, turn off LEDs
- **Error Handling** - Graceful failure handling with validation
- **Advanced Parameters** - Custom fade times, density, cycles, directions

#### 🎯 **Test Coverage:**
- **82 comprehensive tests** covering all service parameters
- **Error handling validation** - Tests invalid inputs and edge cases
- **Parameter variations** - Tests all configurable options
- **Service reliability** - Validates response handling and timeouts

---

## 🏃‍♂️ Quick Start

### 1. Start the LED Service

**Note:** You will need to restart the service if it's already running.

```bash
# Terminal 1: Start the LED service
source /path/to/game_controller_ws/install/setup.bash
export ROS_DOMAIN_ID=10
ros2 run emorobcare_led_service led_service_node
```

**Or use launch file:**
```bash
ros2 launch emorobcare_led_service led_service.launch.py
```

### 2. Verify Services

```bash
export ROS_DOMAIN_ID=10
ros2 service list | grep led
```

Expected output:
```
/control_leds
/get_led_state
/play_effect
/set_leds
```

### 3. Run Complete Test Suite

```bash
cd /path/to/game_controller_ws/src/emorobcare_led_service/examples
./unified_test.sh
```

**Duration:** ~4-5 minutes (includes delays to observe effects)

---

## 📖 Service Call Examples

### 🎨 **Basic Color Control**

**Set all LEDs to Red with fade:**
```bash
ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs \
  "{led_id: -1, r_values: [255], g_values: [0], b_values: [0], fade_time: 1.0}"
```

**Set each LED to different colors:**
```bash
ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs \
  "{led_id: -1, r_values: [255,0,0,255,128], g_values: [0,255,0,255,0], b_values: [0,0,255,0,128], fade_time: 1.0}"
```

**Set single LED (LED 0) to Cyan:**
```bash
ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs \
  "{led_id: 0, r_values: [0], g_values: [255], b_values: [255], fade_time: 0.5}"
```

### 🌈 **LED Effects**

**Rainbow effect:**
```bash
ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect \
  "{effect_name: 'rainbow', duration: 5.0, speed: 0.1}"
```

**Heartbeat effect (red):**
```bash
ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect \
  "{effect_name: 'heartbeat', duration: 3.0, color1_r: 255, color1_g: 0, color1_b: 0}"
```

**Cascade effect (forward, cyan):**
```bash
ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect \
  "{effect_name: 'cascade', cycles: 3, direction: 'forward', color1_r: 0, color1_g: 255, color1_b: 255, speed: 0.1}"
```

**Wave effect (blue to cyan):**
```bash
ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect \
  "{effect_name: 'wave', duration: 4.0, color1_r: 0, color1_g: 0, color1_b: 255, color2_r: 0, color2_g: 255, color2_b: 255}"
```

**Blink effect (multi-color):**
```bash
ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect \
  "{effect_name: 'blink', cycles: 5, speed: 0.3, colors_r: [255,0,0,255,255], colors_g: [0,255,0,255,0], colors_b: [0,0,255,0,255]}"
```

**Knight Rider effect:**
```bash
ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect \
  "{effect_name: 'knight_rider', cycles: 3, color1_r: 255, color1_g: 0, color1_b: 0, speed: 0.1}"
```

**Spain Flag effect:**
```bash
ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect \
  "{effect_name: 'spain_flag', duration: 3.0}"
```

**Emergency effect:**
```bash
ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect \
  "{effect_name: 'emergency', cycles: 10}"
```

**Progress Bar (75%):**
```bash
ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect \
  "{effect_name: 'progress_bar', progress: 0.75, color1_r: 34, color1_g: 139, color1_b: 34, color2_r: 220, color2_g: 20, color2_b: 60}"
```

### 📊 **State Queries**

**Get current LED state with colors:**
```bash
ros2 service call /get_led_state emorobcare_led_service/srv/GetLEDState \
  "{get_colors: true}"
```

**Get LED state without colors:**
```bash
ros2 service call /get_led_state emorobcare_led_service/srv/GetLEDState \
  "{get_colors: false}"
```

### 🎮 **Control Commands**

**Turn off all LEDs instantly:**
```bash
ros2 service call /control_leds emorobcare_led_service/srv/ControlLEDs \
  "{command: 'turn_off_all', fade_time: 0.0}"
```

**Turn off all LEDs with fade:**
```bash
ros2 service call /control_leds emorobcare_led_service/srv/ControlLEDs \
  "{command: 'turn_off_all', fade_time: 2.0}"
```

**Stop fade animation:**
```bash
ros2 service call /control_leds emorobcare_led_service/srv/ControlLEDs \
  "{command: 'stop_fade'}"
```

---

## 🔧 **Available Effects & Parameters**

### Effect Names:
- `rainbow` - Cycling rainbow colors
- `cascade` - Color chasing across LEDs
- `heartbeat` - Pulsing/breathing effect
- `wave` - Smooth wave between two colors
- `blink` - Multi-color blinking
- `blink_single` - Single LED blinking
- `sparkle` - Random LED sparkles
- `knight_rider` - K.I.T.T. scanner effect
- `spain_flag` - Spanish flag display (5 LEDs only)
- `emergency` - Red/blue emergency lights
- `progress_bar` - Progress indicator (0.0-1.0)
- `color_cycle` - Cycle through color list

### Common Parameters:
- `duration` - Effect duration in seconds
- `speed` - Animation speed (0.1-1.0)
- `cycles` - Number of repetitions
- `fade_time` - Fade transition time
- `direction` - "forward" or "backward" (for cascade)
- `density` - Sparkle density (0.0-1.0)
- `progress` - Progress bar value (0.0-1.0)
- `color1_r/g/b` - Primary color RGB values
- `color2_r/g/b` - Secondary color RGB values
- `colors_r/g/b[]` - Color arrays for multi-color effects

---

## 🛠️ **Advanced Configuration**

### Environment Variables:
```bash
export LED_I2C_BUS=1         # I2C bus number (default: 1)
export LED_I2C_ADDR=0x08     # I2C address (default: 0x08)
export LED_NUM_LEDS=5        # Number of LEDs (default: 5)
export ROS_DOMAIN_ID=10      # ROS 2 domain ID
```

### ROS 2 Parameters:
```bash
ros2 launch emorobcare_led_service led_service.launch.py \
  num_leds:=5 i2c_bus:=1 i2c_addr:=0x08
```

---

## 📋 **Test Sections Overview**

The unified test script is organized into 15 comprehensive sections:

1. **Basic Color Control** - Primary colors with fade
2. **Special EMOROBOT Colors** - Custom color definitions
3. **Individual LED Control** - Single and multi-LED control
4. **LED State Query** - State retrieval
5. **Basic LED Effects** - Rainbow and heartbeat
6. **Cascade Effects** - Directional cascading
7. **Wave Effects** - Color transitions
8. **Blink Effects** - Single and multi-color blinking
9. **Advanced Effects** - Sparkle, Knight Rider, Spain Flag, Emergency, Color Cycle
10. **Progress Bar Effect** - Progress indicators (0% to 100%)
11. **Control Commands** - Stop fade, turn off
12. **Error Handling** - Invalid input validation
13. **Advanced Parameter Testing** - Custom parameters
14. **State Query Variations** - Query options
15. **Final State Check** - Verification

---

## 🔍 **Troubleshooting**

### Services Not Found
```bash
ERROR: LED services not found.
```
**Solution:** Ensure the LED service is running and ROS_DOMAIN_ID matches:
```bash
ros2 run emorobcare_led_service led_service_node
export ROS_DOMAIN_ID=10
```

### Permission Denied
```bash
Permission denied: ./unified_test.sh
```
**Solution:**
```bash
chmod +x unified_test.sh
```

### I2C Communication Errors
**Solution:**
1. Check I2C connection to Arduino
2. Verify I2C address: `i2cdetect -y 1`
3. Check Arduino firmware
4. Verify user permissions: `sudo usermod -a -G i2c $USER`

---

## 📚 **Additional Resources**

- [../README.md](../README.md) - Full package documentation
- High-level Python API available in `emorobcare_led_service.led_client`
- Service definitions in `srv/` directory

---

**✨ All LED service functionalities are fully accessible via ROS 2 service calls! No additional dependencies required.**

---

## Quick Start Guide

### Prerequisites

1. **LED service must be running:**
```bash
# Terminal 1: Start the LED service
source /path/to/game_controller_ws/install/setup.bash
   export ROS_DOMAIN_ID=10
   ros2 run emorobcare_led_service led_service_node
   ```

2. **Or use launch file:**
   ```bash
   ros2 launch emorobcare_led_service led_service.launch.py
   ```

3. **Verify services are available:**
   ```bash
   export ROS_DOMAIN_ID=10
   ros2 service list | grep led
   ```
   
   Expected output:
   ```
   /control_leds
   /get_led_state
   /play_effect
   /set_leds
   ```

---

### Running the Test Suite

```bash
cd /path/to/game_controller_ws/src/emorobcare_led_service/examples
export ROS_DOMAIN_ID=10
./unified_test.sh
```

This will run through **all 66 tests** systematically, showing progress for each test and a summary at the end.

---

## Environment Setup

The script requires:

```bash
# Source ROS 2 workspace
source /path/to/game_controller_ws/install/setup.bash

# Set ROS domain ID (must match the service)
export ROS_DOMAIN_ID=10
```

**Note:** The script automatically sets these variables.

---

## Test Sections

The unified test script is organized into 12 sections:

1. **Basic Color Control** - Solid colors with fade (RED, GREEN, BLUE, WHITE, YELLOW, CYAN, MAGENTA)
2. **Special EMOROBOT Colors** - WARM_WHITE, COOL_WHITE, CARE_BLUE, WARNING_RED, SUCCESS_GREEN, ORANGE
3. **Individual LED Control** - Setting specific LEDs and multi-color patterns
4. **LED State Query** - Getting current LED state
5. **Basic Effects** - RAINBOW and HEARTBEAT variations
6. **Cascade Effects** - Forward and backward cascades with different colors
7. **Wave Effects** - Color transitions
8. **Blink Effects** - Single and multi-color blinking
9. **Advanced Effects** - SPARKLE, KNIGHT_RIDER, SPAIN_FLAG, EMERGENCY, COLOR_CYCLE
10. **Progress Bar** - Progress indicators from 0% to 100%
11. **Control Commands** - Stop fade, turn off
12. **Final State Check** - Verify final state

---

## Example Service Calls

### Set All LEDs to Red
```bash
ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs \
  "{led_id: -1, r_values: [255], g_values: [0], b_values: [0], fade_time: 1.0}"
```

### Set Each LED to Different Color
```bash
ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs \
  "{led_id: -1, r_values: [255,0,0,255,128], g_values: [0,255,0,255,0], b_values: [0,0,255,0,128], fade_time: 1.0}"
```

### Play Rainbow Effect
```bash
ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect \
  "{effect_name: 'rainbow', duration: 5.0, speed: 0.1}"
```

### Play Heartbeat (Red)
```bash
ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect \
  "{effect_name: 'heartbeat', duration: 3.0, color1_r: 255, color1_g: 0, color1_b: 0}"
```

### Play Progress Bar (75%)
```bash
ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect \
  "{effect_name: 'progress_bar', progress: 0.75, color1_r: 0, color1_g: 255, color1_b: 0, color2_r: 255, color2_g: 0, color2_b: 0}"
```

### Turn Off All LEDs
```bash
ros2 service call /control_leds emorobcare_led_service/srv/ControlLEDs \
  "{command: 'turn_off_all', fade_time: 2.0}"
```

---

## Troubleshooting

### Services Not Found

**Problem:** `ERROR: LED services not found.`

**Solution:**
1. Make sure the LED service is running:
   ```bash
   ros2 run emorobcare_led_service led_service_node
   ```

2. Check ROS_DOMAIN_ID matches:
   ```bash
   export ROS_DOMAIN_ID=10
   ros2 service list | grep led
   ```

3. Verify the service is on the same domain:
   ```bash
   ros2 node list
   ```

---

### Permission Denied

**Problem:** `Permission denied: ./unified_test.sh`

**Solution:**
```bash
chmod +x unified_test.sh
```

---

### I2C Communication Error

**Problem:** LED service reports I2C errors

**Solution:**
1. Check I2C connection to Arduino
2. Verify I2C address (default: 0x08):
   ```bash
   i2cdetect -y 1
   ```
3. Check Arduino firmware is running
4. Verify user has I2C permissions:
   ```bash
   sudo usermod -a -G i2c $USER
   # Log out and log back in
   ```

---

## See Also

- [../README.md](../README.md) - Full package documentation

---

**Happy LED controlling! 🎨✨**
