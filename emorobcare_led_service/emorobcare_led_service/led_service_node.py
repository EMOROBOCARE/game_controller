#!/usr/bin/env python3
"""
EMOROBCARE LED Service Node

ROS 2 service node for controlling EMOROBOT LEDs with advanced effects and animations.
This node wraps the emorobcare_leds library and exposes LED control via ROS 2 services.

Services:
    /set_leds (SetLEDs) - Set LED colors with optional fade
    /play_effect (PlayEffect) - Play predefined LED effects
    /get_led_state (GetLEDState) - Get current LED state
    /control_leds (ControlLEDs) - Control LED animations (stop, turn off)

Usage:
    ros2 run emorobcare_led_service led_service_node

Environment Variables:
    ROS_DOMAIN_ID - ROS 2 domain ID (default: 100)
    LED_I2C_BUS - I2C bus number (default: 1)
    LED_I2C_ADDR - I2C address (default: 0x08)
    LED_NUM_LEDS - Number of LEDs (default: 5)
"""

import rclpy
from rclpy.node import Node
import os
import sys
import time

def _env_truthy(name: str, default: str = "0") -> bool:
    value = str(os.environ.get(name, default)).strip().lower()
    return value in {"1", "true", "yes", "on"}


def _clamp8(value) -> int:
    return max(0, min(255, int(value)))


class _MockColor:
    def __init__(self, r: int, g: int, b: int) -> None:
        self._rgb = (_clamp8(r), _clamp8(g), _clamp8(b))

    def to_tuple(self):
        return self._rgb


class _MockColors:
    BLACK = _MockColor(0, 0, 0)
    WHITE = _MockColor(255, 255, 255)
    RED = _MockColor(255, 0, 0)
    GREEN = _MockColor(0, 255, 0)
    BLUE = _MockColor(0, 0, 255)
    YELLOW = _MockColor(255, 255, 0)
    MAGENTA = _MockColor(255, 0, 255)
    CYAN = _MockColor(0, 255, 255)
    ORANGE = _MockColor(255, 165, 0)
    PINK = _MockColor(255, 105, 180)
    PURPLE = _MockColor(128, 0, 128)
    VIOLET = _MockColor(148, 0, 211)
    INDIGO = _MockColor(75, 0, 130)
    BROWN = _MockColor(165, 42, 42)
    BEIGE = _MockColor(245, 245, 220)
    TAN = _MockColor(210, 180, 140)
    GRAY = _MockColor(128, 128, 128)
    LIGHT_GRAY = _MockColor(211, 211, 211)
    DARK_GRAY = _MockColor(64, 64, 64)
    WARM_WHITE = _MockColor(255, 244, 229)
    COOL_WHITE = _MockColor(240, 248, 255)
    CARE_BLUE = _MockColor(0, 120, 255)
    WARNING_RED = _MockColor(255, 32, 32)
    SUCCESS_GREEN = _MockColor(32, 200, 64)


class _MockLEDController:
    def __init__(self, i2c_bus: int = 1, i2c_addr: int = 0x08, num_leds: int = 5) -> None:
        del i2c_bus, i2c_addr
        self.num_leds = max(1, int(num_leds))
        self._fading = False
        self._colors = [(0, 0, 0)] * self.num_leds

    def _normalize_rgb(self, rgb):
        if hasattr(rgb, "to_tuple"):
            rgb = rgb.to_tuple()
        return (_clamp8(rgb[0]), _clamp8(rgb[1]), _clamp8(rgb[2]))

    def set_all_same_color(self, rgb, fade_time: float = 0.0, wait_response: bool = False):
        del wait_response
        self._fading = bool(float(fade_time) > 0.0)
        color = self._normalize_rgb(rgb)
        self._colors = [color] * self.num_leds
        return True

    def set_all_leds(self, colors, fade_time: float = 0.0, wait_response: bool = False):
        del wait_response
        self._fading = bool(float(fade_time) > 0.0)
        normalized = [self._normalize_rgb(c) for c in list(colors or [])]
        if not normalized:
            normalized = [(0, 0, 0)] * self.num_leds
        if len(normalized) < self.num_leds:
            normalized = (normalized * ((self.num_leds // len(normalized)) + 1))[: self.num_leds]
        self._colors = normalized[: self.num_leds]
        return True

    def set_led(self, led_id: int, rgb, fade_time: float = 0.0, wait_response: bool = False):
        del wait_response
        if led_id < 0 or led_id >= self.num_leds:
            return False
        self._fading = bool(float(fade_time) > 0.0)
        current = list(self._colors)
        current[int(led_id)] = self._normalize_rgb(rgb)
        self._colors = current
        return True

    def get_current_colors(self):
        return list(self._colors)

    def is_fading(self):
        return bool(self._fading)

    def stop_fade(self):
        self._fading = False
        return True

    def turn_off_all(self, fade_time: float = 0.0):
        self._fading = bool(float(fade_time) > 0.0)
        self._colors = [(0, 0, 0)] * self.num_leds
        return True

    def close(self):
        return None


class _MockLEDEffects:
    @staticmethod
    def _set_all(controller, color):
        return controller.set_all_same_color(color, fade_time=0.0, wait_response=False)

    @staticmethod
    def rainbow(controller, **kwargs):
        del kwargs
        return _MockLEDEffects._set_all(controller, _MockColors.CYAN.to_tuple())

    @staticmethod
    def cascade(controller, color=(0, 255, 255), **kwargs):
        del kwargs
        return _MockLEDEffects._set_all(controller, color)

    @staticmethod
    def heartbeat(controller, color=(255, 0, 0), **kwargs):
        del kwargs
        return _MockLEDEffects._set_all(controller, color)

    @staticmethod
    def wave(controller, color1=(0, 255, 255), color2=(0, 0, 255), **kwargs):
        del kwargs
        colors = []
        for i in range(controller.num_leds):
            colors.append(color1 if i % 2 == 0 else color2)
        return controller.set_all_leds(colors, fade_time=0.0, wait_response=False)

    @staticmethod
    def blink(controller, colors=None, **kwargs):
        del kwargs
        if colors:
            return controller.set_all_leds(colors, fade_time=0.0, wait_response=False)
        return _MockLEDEffects._set_all(controller, _MockColors.YELLOW.to_tuple())

    @staticmethod
    def blink_single(controller, led_id=0, color=(255, 255, 255), **kwargs):
        del kwargs
        return controller.set_led(int(led_id), color, fade_time=0.0, wait_response=False)

    @staticmethod
    def random_sparkle(controller, colors=None, **kwargs):
        del kwargs
        if not colors:
            colors = [_MockColors.WHITE.to_tuple()]
        return controller.set_all_leds(colors, fade_time=0.0, wait_response=False)

    @staticmethod
    def knight_rider(controller, color=(255, 0, 0), **kwargs):
        del kwargs
        return _MockLEDEffects._set_all(controller, color)

    @staticmethod
    def emergency_blink(controller, **kwargs):
        del kwargs
        return _MockLEDEffects._set_all(controller, _MockColors.WARNING_RED.to_tuple())

    @staticmethod
    def progress_bar(controller, progress=0.0, color_full=(0, 255, 0), color_empty=(0, 0, 0), **kwargs):
        del kwargs
        lit = int(round(max(0.0, min(1.0, float(progress))) * controller.num_leds))
        colors = []
        for idx in range(controller.num_leds):
            colors.append(color_full if idx < lit else color_empty)
        return controller.set_all_leds(colors, fade_time=0.0, wait_response=False)

    @staticmethod
    def color_cycle(controller, colors=None, **kwargs):
        del kwargs
        if not colors:
            colors = [_MockColors.RED.to_tuple(), _MockColors.GREEN.to_tuple(), _MockColors.BLUE.to_tuple()]
        return controller.set_all_leds(colors, fade_time=0.0, wait_response=False)


# Import LED controller library (assumed to be installed system-wide)
try:
    from emorobcare_leds import LEDController, Colors, LEDEffects
except ImportError as e:
    if _env_truthy("LED_USE_MOCK", "0"):
        print(f"WARN: Failed to import emorobcare_leds ({e}); using in-process mock backend.")
        LEDController = _MockLEDController
        Colors = _MockColors
        LEDEffects = _MockLEDEffects
    else:
        print(f"ERROR: Failed to import emorobcare_leds: {e}")
        print("Please ensure emorobcare_leds is installed system-wide:")
        print("  cd /home/nvidia/emorobcare_leds && sudo pip install -e .")
        print("Or set LED_USE_MOCK=1 for laptop mock mode.")
        sys.exit(1)

# Import custom service definitions
from emorobcare_led_service.srv import SetLEDs, SetLEDsString, PlayEffect, GetLEDState, ControlLEDs
from emorobcare_led_service.msg import AvailableColors


class LEDServiceNode(Node):
    """
    ROS 2 service node for controlling EMOROBOT LEDs.
    
    This node provides multiple services for comprehensive LED control including
    individual LED setting, animations, effects, and state queries.
    """
    
    def __init__(self):
        super().__init__('led_service_node')
        
        # Declare and get parameters
        self.declare_parameter('i2c_bus', int(os.environ.get('LED_I2C_BUS', '1')))
        self.declare_parameter('i2c_addr', int(os.environ.get('LED_I2C_ADDR', '8'), 16))
        self.declare_parameter('num_leds', int(os.environ.get('LED_NUM_LEDS', '5')))
        
        i2c_bus = self.get_parameter('i2c_bus').value
        i2c_addr = self.get_parameter('i2c_addr').value
        num_leds = self.get_parameter('num_leds').value
        
        # Initialize LED controller
        try:
            self.led_controller = LEDController(
                i2c_bus=i2c_bus,
                i2c_addr=i2c_addr,
                num_leds=num_leds
            )
            self.get_logger().info(
                f'LED Controller initialized: bus={i2c_bus}, addr=0x{i2c_addr:02X}, leds={num_leds}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to initialize LED controller: {e}')
            raise
        
        # Create services
        self.srv_set_leds = self.create_service(
            SetLEDs,
            'set_leds',
            self.handle_set_leds
        )
        
        self.srv_set_leds_string = self.create_service(
            SetLEDsString,
            'set_leds_string',
            self.handle_set_leds_string
        )
        
        self.srv_play_effect = self.create_service(
            PlayEffect,
            'play_effect',
            self.handle_play_effect
        )
        
        self.srv_get_state = self.create_service(
            GetLEDState,
            'get_led_state',
            self.handle_get_state
        )
        
        self.srv_control = self.create_service(
            ControlLEDs,
            'control_leds',
            self.handle_control
        )
        
        # Create publisher for available colors
        self.colors_publisher = self.create_publisher(
            AvailableColors,
            'available_colors',
            10  # QoS profile depth
        )
        
        # Publish available colors once at startup
        self.publish_available_colors()
        
        self.get_logger().info('LED Service Node ready. Services available:')
        self.get_logger().info('  - /set_leds (SetLEDs) - RGB values')
        self.get_logger().info('  - /set_leds_string (SetLEDsString) - Color names')
        self.get_logger().info('  - /play_effect (PlayEffect)')
        self.get_logger().info('  - /get_led_state (GetLEDState)')
        self.get_logger().info('  - /control_leds (ControlLEDs)')
        self.get_logger().info('Topics available:')
        self.get_logger().info('  - /available_colors (AvailableColors)')
    
    def handle_set_leds(self, request, response):
        """
        Handle SetLEDs service request.
        
        Sets individual or all LEDs to specified colors with optional fade effect.
        Supports both single color (applied to all LEDs) and color arrays (one per LED).
        """
        try:
            led_id = request.led_id
            fade_time = request.fade_time
            wait_response = request.wait_response
            
            # Validate LED ID
            if led_id < -1 or led_id >= self.led_controller.num_leds:
                response.success = False
                response.message = (
                    f"Invalid LED ID: {led_id}. "
                    f"Must be -1 (all LEDs) or 0-{self.led_controller.num_leds-1} (specific LED)."
                )
                self.get_logger().error(response.message)
                return response
            
            # Validate fade time
            if fade_time < 0:
                response.success = False
                response.message = f"Invalid fade_time: {fade_time}. Must be >= 0.0 seconds."
                self.get_logger().error(response.message)
                return response
            
            # Parse RGB values
            r_vals = list(request.r_values)
            g_vals = list(request.g_values)
            b_vals = list(request.b_values)
            
            # Check that RGB arrays are not empty
            if not r_vals or not g_vals or not b_vals:
                response.success = False
                response.message = "RGB values cannot be empty. Provide at least one value for each channel."
                self.get_logger().error(response.message)
                return response
            
            # Check that RGB arrays have the same length
            if not (len(r_vals) == len(g_vals) == len(b_vals)):
                response.success = False
                response.message = (
                    f"RGB arrays must have equal length. "
                    f"Got R:{len(r_vals)}, G:{len(g_vals)}, B:{len(b_vals)}"
                )
                self.get_logger().error(response.message)
                return response
            
            # Validate RGB value ranges (0-255)
            for i, (r, g, b) in enumerate(zip(r_vals, g_vals, b_vals)):
                if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
                    response.success = False
                    response.message = (
                        f"Invalid RGB values at index {i}: RGB({r}, {g}, {b}). "
                        f"All values must be in range 0-255."
                    )
                    self.get_logger().error(response.message)
                    return response
            
            # Set all LEDs
            if led_id == -1:
                # CASE 1: Single color for all LEDs (most common use case)
                if len(r_vals) == 1:
                    rgb = (r_vals[0], g_vals[0], b_vals[0])
                    self.get_logger().info(
                        f"Setting all {self.led_controller.num_leds} LEDs to RGB{rgb} "
                        f"with fade={fade_time:.2f}s"
                    )
                    success = self.led_controller.set_all_same_color(
                        rgb, fade_time=fade_time, wait_response=wait_response
                    )
                
                # CASE 2: Individual colors for each LED
                elif len(r_vals) == self.led_controller.num_leds:
                    colors = [(r, g, b) for r, g, b in zip(r_vals, g_vals, b_vals)]
                    self.get_logger().info(
                        f"Setting {self.led_controller.num_leds} LEDs to individual colors "
                        f"with fade={fade_time:.2f}s"
                    )
                    for i, color in enumerate(colors):
                        self.get_logger().debug(f"  LED {i}: RGB{color}")
                    success = self.led_controller.set_all_leds(
                        colors, fade_time=fade_time, wait_response=wait_response
                    )
                
                # CASE 3: Invalid array length
                else:
                    response.success = False
                    response.message = (
                        f"Invalid RGB array length for setting all LEDs. "
                        f"Expected 1 (same color for all) or {self.led_controller.num_leds} (one per LED), "
                        f"but got {len(r_vals)}."
                    )
                    self.get_logger().error(response.message)
                    return response
            
            # Set single LED
            else:
                if len(r_vals) != 1:
                    response.success = False
                    response.message = (
                        f"Invalid RGB array length for setting single LED. "
                        f"Expected 1 value, but got {len(r_vals)}."
                    )
                    self.get_logger().error(response.message)
                    return response
                
                rgb = (r_vals[0], g_vals[0], b_vals[0])
                self.get_logger().info(
                    f"Setting LED {led_id} to RGB{rgb} with fade={fade_time:.2f}s"
                )
                success = self.led_controller.set_led(
                    led_id, rgb, fade_time=fade_time, wait_response=wait_response
                )
            
            # Build response
            if success:
                response.success = True
                if led_id == -1:
                    if len(r_vals) == 1:
                        response.message = f"Successfully set all LEDs to RGB{rgb}"
                    else:
                        response.message = f"Successfully set all {self.led_controller.num_leds} LEDs to individual colors"
                else:
                    response.message = f"Successfully set LED {led_id} to RGB{rgb}"
                
                if fade_time > 0:
                    response.message += f" (fade: {fade_time:.2f}s)"
            else:
                response.success = False
                response.message = (
                    "Failed to update LEDs. I2C communication error. "
                    "Check hardware connection and Arduino firmware."
                )
                self.get_logger().error(response.message)
            
            # Return current colors
            current_colors = self.led_controller.get_current_colors()
            response.current_r = [c[0] for c in current_colors]
            response.current_g = [c[1] for c in current_colors]
            response.current_b = [c[2] for c in current_colors]
            
        except ValueError as e:
            self.get_logger().error(f"Validation error in set_leds service: {e}")
            response.success = False
            response.message = f"Validation error: {str(e)}"
        except Exception as e:
            self.get_logger().error(f"Unexpected error in set_leds service: {e}", exc_info=True)
            response.success = False
            response.message = f"Unexpected error: {str(e)}"
        
        return response
    
    def handle_set_leds_string(self, request, response):
        """
        Handle SetLEDsString service request.
        
        Sets individual or all LEDs to specified colors using color names with optional fade effect.
        This service only accepts color names, not RGB values.
        """
        try:
            led_id = request.led_id
            color_name = request.color_name.lower().strip()
            fade_time = request.fade_time
            wait_response = request.wait_response
            
            # Validate LED ID
            if led_id < -1 or led_id >= self.led_controller.num_leds:
                response.success = False
                response.message = (
                    f"Invalid LED ID: {led_id}. "
                    f"Must be -1 (all LEDs) or 0-{self.led_controller.num_leds-1} (specific LED)."
                )
                self.get_logger().error(response.message)
                return response
            
            # Validate color name
            if not color_name:
                response.success = False
                response.message = "Color name cannot be empty."
                self.get_logger().error(response.message)
                return response
            
            # Validate fade time
            if fade_time < 0:
                response.success = False
                response.message = f"Invalid fade_time: {fade_time}. Must be >= 0.0 seconds."
                self.get_logger().error(response.message)
                return response
            
            # Get available colors
            available_colors = self.get_available_colors_dict()
            
            if color_name not in available_colors:
                response.success = False
                response.message = (
                    f"Unknown color name: '{color_name}'. "
                    f"Available colors: {', '.join(sorted(available_colors.keys()))}"
                )
                self.get_logger().error(response.message)
                return response
            
            # Get RGB values from color name
            rgb = available_colors[color_name]
            
            # Set LEDs based on led_id
            if led_id == -1:
                # Set all LEDs to the same color
                self.get_logger().info(
                    f"Setting all {self.led_controller.num_leds} LEDs to '{color_name}' ({rgb}) "
                    f"with fade={fade_time:.2f}s"
                )
                success = self.led_controller.set_all_same_color(
                    rgb, fade_time=fade_time, wait_response=wait_response
                )
            else:
                # Set single LED
                self.get_logger().info(
                    f"Setting LED {led_id} to '{color_name}' ({rgb}) with fade={fade_time:.2f}s"
                )
                success = self.led_controller.set_led(
                    led_id, rgb, fade_time=fade_time, wait_response=wait_response
                )
            
            # Build response
            if success:
                response.success = True
                if led_id == -1:
                    response.message = f"Successfully set all LEDs to '{color_name}'"
                else:
                    response.message = f"Successfully set LED {led_id} to '{color_name}'"
                
                if fade_time > 0:
                    response.message += f" (fade: {fade_time:.2f}s)"
            else:
                response.success = False
                response.message = (
                    "Failed to update LEDs. I2C communication error. "
                    "Check hardware connection and Arduino firmware."
                )
                self.get_logger().error(response.message)
            
            # Return current colors
            current_colors = self.led_controller.get_current_colors()
            response.current_r = [c[0] for c in current_colors]
            response.current_g = [c[1] for c in current_colors]
            response.current_b = [c[2] for c in current_colors]
            
        except ValueError as e:
            self.get_logger().error(f"Validation error in set_leds_string service: {e}")
            response.success = False
            response.message = f"Validation error: {str(e)}"
        except Exception as e:
            self.get_logger().error(f"Unexpected error in set_leds_string service: {e}", exc_info=True)
            response.success = False
            response.message = f"Unexpected error: {str(e)}"
        
        return response
    
    def get_available_colors_dict(self):
        """
        Get a dictionary of available color names and their RGB values.
        
        Returns:
            dict: Dictionary mapping color names to RGB tuples
        """
        return {
            'black': Colors.BLACK.to_tuple(),
            'white': Colors.WHITE.to_tuple(),
            'red': Colors.RED.to_tuple(),
            'green': Colors.GREEN.to_tuple(),
            'blue': Colors.BLUE.to_tuple(),
            'yellow': Colors.YELLOW.to_tuple(),
            'magenta': Colors.MAGENTA.to_tuple(),
            'cyan': Colors.CYAN.to_tuple(),
            'orange': Colors.ORANGE.to_tuple(),
            'pink': Colors.PINK.to_tuple(),
            'purple': Colors.PURPLE.to_tuple(),
            'violet': Colors.VIOLET.to_tuple(),
            'indigo': Colors.INDIGO.to_tuple(),
            'brown': Colors.BROWN.to_tuple(),
            'beige': Colors.BEIGE.to_tuple(),
            'tan': Colors.TAN.to_tuple(),
            'gray': Colors.GRAY.to_tuple(),
            'light_gray': Colors.LIGHT_GRAY.to_tuple(),
            'dark_gray': Colors.DARK_GRAY.to_tuple(),
            'warm_white': Colors.WARM_WHITE.to_tuple(),
            'cool_white': Colors.COOL_WHITE.to_tuple(),
            'care_blue': Colors.CARE_BLUE.to_tuple(),
            'warning_red': Colors.WARNING_RED.to_tuple(),
            'success_green': Colors.SUCCESS_GREEN.to_tuple(),
        }
    
    def publish_available_colors(self):
        """
        Publish available colors to the /available_colors topic.
        """
        try:
            colors_dict = self.get_available_colors_dict()
            
            # Create message
            msg = AvailableColors()
            msg.color_names = list(colors_dict.keys())
            
            # Separate RGB values into individual arrays
            r_values = []
            g_values = []
            b_values = []
            
            for rgb in colors_dict.values():
                r_values.append(rgb[0])
                g_values.append(rgb[1])
                b_values.append(rgb[2])
            
            msg.r_values = r_values
            msg.g_values = g_values
            msg.b_values = b_values
            
            # Add descriptions
            descriptions = []
            for name in colors_dict.keys():
                if name.startswith('care_') or name.startswith('warning_') or name.startswith('success_'):
                    descriptions.append(f"EMOROBOT special color: {name.replace('_', ' ')}")
                elif name.endswith('_white'):
                    descriptions.append(f"White variant: {name.replace('_', ' ')}")
                elif name.endswith('_gray'):
                    descriptions.append(f"Gray variant: {name.replace('_', ' ')}")
                else:
                    descriptions.append(f"Standard color: {name}")
            
            msg.descriptions = descriptions
            
            # Publish message
            self.colors_publisher.publish(msg)
            self.get_logger().info(f"Published {len(colors_dict)} available colors to /available_colors topic")
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish available colors: {e}", exc_info=True)
    
    def handle_play_effect(self, request, response):
        """
        Handle PlayEffect service request.
        
        Plays predefined LED effects with customizable parameters.
        Supports 12+ built-in effects with comprehensive error handling.
        """
        try:
            effect_name = request.effect_name.lower().strip()
            
            # Validate effect name
            valid_effects = [
                'rainbow', 'cascade', 'heartbeat', 'wave', 'blink', 
                'blink_single', 'sparkle', 'knight_rider', 
                'emergency', 'progress_bar', 'color_cycle'
            ]
            
            if not effect_name:
                response.success = False
                response.message = "Effect name cannot be empty. Valid effects: " + ", ".join(valid_effects)
                response.actual_duration = 0.0
                self.get_logger().error(response.message)
                return response
            
            if effect_name not in valid_effects:
                response.success = False
                response.message = (
                    f"Unknown effect: '{effect_name}'. "
                    f"Valid effects are: {', '.join(valid_effects)}"
                )
                response.actual_duration = 0.0
                self.get_logger().error(response.message)
                return response
            
            # Parse and validate parameters
            duration = request.duration if request.duration > 0 else 3.0
            speed = request.speed if request.speed > 0 else 0.1
            cycles = request.cycles if request.cycles > 0 else 3
            fade_time = max(0.0, request.fade_time)
            direction = request.direction.lower() if request.direction else "forward"
            density = max(0.0, min(1.0, request.density if request.density > 0 else 0.3))
            progress = max(0.0, min(1.0, request.progress))
            
            # Validate direction
            if direction not in ['forward', 'backward']:
                response.success = False
                response.message = f"Invalid direction: '{direction}'. Must be 'forward' or 'backward'."
                response.actual_duration = 0.0
                self.get_logger().error(response.message)
                return response
            
            # Parse and validate colors
            color1 = (request.color1_r, request.color1_g, request.color1_b)
            color2 = (request.color2_r, request.color2_g, request.color2_b)
            
            # Validate RGB ranges
            for i, val in enumerate(color1):
                if not (0 <= val <= 255):
                    response.success = False
                    response.message = f"Invalid color1 component {i}: {val}. Must be 0-255."
                    response.actual_duration = 0.0
                    self.get_logger().error(response.message)
                    return response
            
            for i, val in enumerate(color2):
                if not (0 <= val <= 255):
                    response.success = False
                    response.message = f"Invalid color2 component {i}: {val}. Must be 0-255."
                    response.actual_duration = 0.0
                    self.get_logger().error(response.message)
                    return response
            
            # Parse color arrays
            colors = []
            if request.colors_r and request.colors_g and request.colors_b:
                if len(request.colors_r) == len(request.colors_g) == len(request.colors_b):
                    colors = [(r, g, b) for r, g, b in 
                             zip(request.colors_r, request.colors_g, request.colors_b)]
                    # Validate all colors in array
                    for idx, (r, g, b) in enumerate(colors):
                        if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
                            response.success = False
                            response.message = (
                                f"Invalid RGB values in colors array at index {idx}: RGB({r}, {g}, {b}). "
                                f"All values must be 0-255."
                            )
                            response.actual_duration = 0.0
                            self.get_logger().error(response.message)
                            return response
                else:
                    response.success = False
                    response.message = (
                        f"Color arrays must have equal length. "
                        f"Got R:{len(request.colors_r)}, G:{len(request.colors_g)}, B:{len(request.colors_b)}"
                    )
                    response.actual_duration = 0.0
                    self.get_logger().error(response.message)
                    return response
            
            # Use default colors if not provided
            if color1 == (0, 0, 0):
                color1 = Colors.CYAN
            if color2 == (0, 0, 0):
                color2 = Colors.BLUE
            if not colors:
                colors = [Colors.RED, Colors.GREEN, Colors.BLUE, Colors.YELLOW, Colors.MAGENTA]
            
            # Log effect execution
            self.get_logger().info(
                f"Playing effect '{effect_name}' with parameters: "
                f"duration={duration:.2f}s, speed={speed:.2f}, cycles={cycles}, "
                f"fade_time={fade_time:.2f}s"
            )
            
            start_time = time.time()
            success = False
            
            # Execute effect based on name
            if effect_name == 'rainbow':
                self.get_logger().debug(f"Rainbow: duration={duration}, speed={speed}, fade_time={fade_time}")
                success = LEDEffects.rainbow(
                    self.led_controller, duration=duration, speed=speed, fade_time=fade_time
                )
            
            elif effect_name == 'cascade':
                self.get_logger().debug(f"Cascade: color={color1}, direction={direction}, cycles={cycles}")
                success = LEDEffects.cascade(
                    self.led_controller, color=color1, direction=direction,
                    speed=speed, cycles=cycles, fade_time=fade_time
                )
            
            elif effect_name == 'heartbeat':
                self.get_logger().debug(f"Heartbeat: color={color1}, duration={duration}")
                success = LEDEffects.heartbeat(
                    self.led_controller, color=color1, duration=duration, speed=speed
                )
            
            elif effect_name == 'wave':
                self.get_logger().debug(f"Wave: color1={color1}, color2={color2}, duration={duration}")
                success = LEDEffects.wave(
                    self.led_controller, color1=color1, color2=color2,
                    speed=speed, duration=duration
                )
            
            elif effect_name == 'blink':
                # Ensure we have the right number of colors
                if len(colors) < self.led_controller.num_leds:
                    # Extend colors list by repeating
                    colors = (colors * ((self.led_controller.num_leds // len(colors)) + 1))[:self.led_controller.num_leds]
                elif len(colors) > self.led_controller.num_leds:
                    # Trim colors list
                    colors = colors[:self.led_controller.num_leds]
                
                self.get_logger().debug(f"Blink: {len(colors)} colors, cycles={cycles}")
                success = LEDEffects.blink(
                    self.led_controller, colors=colors,
                    on_time=speed, off_time=speed, fade_time=fade_time, cycles=cycles
                )
            
            elif effect_name == 'blink_single':
                # For single LED blink, use first LED (0) by default
                led_id = 0
                self.get_logger().debug(f"Blink Single: LED {led_id}, color={color1}, cycles={cycles}")
                success = LEDEffects.blink_single(
                    self.led_controller, led_id=led_id, color=color1,
                    on_time=speed, off_time=speed, fade_time=fade_time, cycles=cycles
                )
            
            elif effect_name == 'sparkle':
                self.get_logger().debug(f"Sparkle: {len(colors)} colors, density={density}, duration={duration}")
                success = LEDEffects.random_sparkle(
                    self.led_controller, colors=colors,
                    density=density, speed=speed, duration=duration
                )
            
            elif effect_name == 'knight_rider':
                self.get_logger().debug(f"Knight Rider: color={color1}, cycles={cycles}")
                success = LEDEffects.knight_rider(
                    self.led_controller, color=color1, speed=speed, cycles=cycles
                )
            
            elif effect_name == 'emergency':
                self.get_logger().debug(f"Emergency: cycles={cycles}")
                success = LEDEffects.emergency_blink(
                    self.led_controller, cycles=cycles
                )
            
            elif effect_name == 'progress_bar':
                self.get_logger().debug(f"Progress Bar: progress={progress:.2f}, full={color1}, empty={color2}")
                success = LEDEffects.progress_bar(
                    self.led_controller, progress=progress,
                    color_full=color1, color_empty=color2
                )
            
            elif effect_name == 'color_cycle':
                self.get_logger().debug(f"Color Cycle: {len(colors)} colors, cycle_time={speed}, cycles={cycles}")
                success = LEDEffects.color_cycle(
                    self.led_controller, colors=colors,
                    cycle_time=speed, cycles=cycles
                )
            
            actual_duration = time.time() - start_time
            
            if success:
                response.success = True
                response.message = f"Effect '{effect_name}' completed successfully in {actual_duration:.2f}s"
                response.actual_duration = actual_duration
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = (
                    f"Effect '{effect_name}' failed to execute. "
                    f"Check I2C connection and Arduino firmware."
                )
                response.actual_duration = actual_duration
                self.get_logger().error(response.message)
            
        except ValueError as e:
            self.get_logger().error(f"Validation error in play_effect service: {e}")
            response.success = False
            response.message = f"Validation error: {str(e)}"
            response.actual_duration = 0.0
        except Exception as e:
            self.get_logger().error(f"Unexpected error in play_effect service: {e}", exc_info=True)
            response.success = False
            response.message = f"Unexpected error: {str(e)}"
            response.actual_duration = 0.0
        
        return response
    
    def handle_get_state(self, request, response):
        """
        Handle GetLEDState service request.
        
        Returns current LED state including colors and animation status.
        """
        try:
            response.num_leds = self.led_controller.num_leds
            response.is_fading = self.led_controller.is_fading()
            
            if request.get_colors:
                try:
                    current_colors = self.led_controller.get_current_colors()
                    response.current_r = [c[0] for c in current_colors]
                    response.current_g = [c[1] for c in current_colors]
                    response.current_b = [c[2] for c in current_colors]
                    
                    self.get_logger().debug(
                        f"Retrieved state: {response.num_leds} LEDs, "
                        f"Fading: {response.is_fading}"
                    )
                except Exception as e:
                    response.success = False
                    response.message = f"Failed to retrieve LED colors: {str(e)}"
                    self.get_logger().error(response.message)
                    return response
            else:
                response.current_r = []
                response.current_g = []
                response.current_b = []
            
            response.success = True
            response.message = (
                f"State retrieved: {response.num_leds} LEDs, "
                f"Fading: {response.is_fading}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Unexpected error in get_state service: {e}", exc_info=True)
            response.success = False
            response.message = f"Unexpected error: {str(e)}"
            response.num_leds = 0
            response.is_fading = False
            response.current_r = []
            response.current_g = []
            response.current_b = []
        
        return response
    
    def handle_control(self, request, response):
        """
        Handle ControlLEDs service request.
        
        Controls LED animations (stop fade, turn off all).
        """
        try:
            command = request.command.lower().strip()
            fade_time = max(0.0, request.fade_time)
            
            # Validate command
            valid_commands = ['stop_fade', 'turn_off_all']
            
            if not command:
                response.success = False
                response.message = (
                    "Command cannot be empty. "
                    f"Valid commands: {', '.join(valid_commands)}"
                )
                self.get_logger().error(response.message)
                return response
            
            if command not in valid_commands:
                response.success = False
                response.message = (
                    f"Unknown command: '{command}'. "
                    f"Valid commands are: {', '.join(valid_commands)}"
                )
                self.get_logger().error(response.message)
                return response
            
            self.get_logger().info(f"Executing control command: '{command}' (fade_time={fade_time:.2f}s)")
            
            if command == 'stop_fade':
                try:
                    self.led_controller.stop_fade()
                    response.success = True
                    response.message = "Fade animation stopped successfully"
                    self.get_logger().info(response.message)
                except Exception as e:
                    response.success = False
                    response.message = f"Failed to stop fade animation: {str(e)}"
                    self.get_logger().error(response.message)
            
            elif command == 'turn_off_all':
                try:
                    success = self.led_controller.turn_off_all(fade_time=fade_time)
                    if success:
                        response.success = True
                        if fade_time > 0:
                            response.message = f"All LEDs turned off with {fade_time:.2f}s fade"
                        else:
                            response.message = "All LEDs turned off"
                        self.get_logger().info(response.message)
                    else:
                        response.success = False
                        response.message = (
                            "Failed to turn off LEDs. I2C communication error. "
                            "Check hardware connection."
                        )
                        self.get_logger().error(response.message)
                except Exception as e:
                    response.success = False
                    response.message = f"Failed to turn off LEDs: {str(e)}"
                    self.get_logger().error(response.message)
            
        except Exception as e:
            self.get_logger().error(f"Unexpected error in control service: {e}", exc_info=True)
            response.success = False
            response.message = f"Unexpected error: {str(e)}"
        
        return response
    
    def shutdown(self):
        """Clean up resources on shutdown."""
        self.get_logger().info('Shutting down LED service node...')
        try:
            self.led_controller.close()
        except:
            pass


def main(args=None):
    """Main entry point for LED service node."""
    
    # Set ROS domain ID if not already set
    if 'ROS_DOMAIN_ID' not in os.environ:
        os.environ['ROS_DOMAIN_ID'] = '100'
    
    rclpy.init(args=args)
    
    try:
        node = LEDServiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error running LED service node: {e}")
    finally:
        if 'node' in locals():
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
