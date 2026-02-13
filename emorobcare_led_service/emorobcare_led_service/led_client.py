#!/usr/bin/env python3
"""
High-level LED client with simplified API.

This module provides an easy-to-use interface for controlling LEDs
with color names, simplified methods, and intuitive parameters.

Usage:
    from emorobcare_led_service.led_client import LEDClient
    
    client = LEDClient()
    client.set_color('red')
    client.set_color('blue', fade_time=1.0)
    client.play_effect('rainbow', duration=5.0)
"""

import rclpy
from rclpy.node import Node
from emorobcare_led_service.srv import SetLEDs, PlayEffect, GetLEDState, ControlLEDs

try:
    from emorobcare_leds.colors import Colors, Color
    HAS_COLORS_LIB = True
except ImportError:
    HAS_COLORS_LIB = False
    Colors = None
    Color = None


class LEDClient(Node):
    """High-level LED client with simplified API."""
    
    # Build color name mappings from emorobcare_leds.Colors if available
    if HAS_COLORS_LIB:
        COLORS = {
            'off': Colors.BLACK.to_tuple(),
            'black': Colors.BLACK.to_tuple(),
            'white': Colors.WHITE.to_tuple(),
            'red': Colors.RED.to_tuple(),
            'green': Colors.GREEN.to_tuple(),
            'blue': Colors.BLUE.to_tuple(),
            'yellow': Colors.YELLOW.to_tuple(),
            'cyan': Colors.CYAN.to_tuple(),
            'magenta': Colors.MAGENTA.to_tuple(),
            'purple': Colors.PURPLE.to_tuple(),
            'orange': Colors.ORANGE.to_tuple(),
            'pink': Colors.PINK.to_tuple(),
            'violet': Colors.VIOLET.to_tuple(),
            'indigo': Colors.INDIGO.to_tuple(),
            'brown': Colors.BROWN.to_tuple(),
            'gray': Colors.GRAY.to_tuple(),
            'light_gray': Colors.LIGHT_GRAY.to_tuple(),
            'dark_gray': Colors.DARK_GRAY.to_tuple(),
            'beige': Colors.BEIGE.to_tuple(),
            'tan': Colors.TAN.to_tuple(),
            'warm_white': Colors.WARM_WHITE.to_tuple(),
            'cool_white': Colors.COOL_WHITE.to_tuple(),
            'care_blue': Colors.CARE_BLUE.to_tuple(),
            'warning_red': Colors.WARNING_RED.to_tuple(),
            'success_green': Colors.SUCCESS_GREEN.to_tuple(),
            'spain_red': Colors.SPAIN_RED.to_tuple(),
            'spain_yellow': Colors.SPAIN_YELLOW.to_tuple(),
        }
    else:
        # Fallback color definitions if emorobcare_leds not installed
        COLORS = {
            'off': (0, 0, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
            'red': (255, 0, 0),
            'green': (0, 255, 0),
            'blue': (0, 0, 255),
            'yellow': (255, 255, 0),
            'cyan': (0, 255, 255),
            'magenta': (255, 0, 255),
            'purple': (128, 0, 128),
            'orange': (255, 165, 0),
            'pink': (255, 192, 203),
            'violet': (148, 0, 211),
            'indigo': (75, 0, 130),
            'brown': (165, 42, 42),
            'gray': (128, 128, 128),
        }
    
    def __init__(self, node_name='led_client_high_level'):
        """Initialize the LED client."""
        super().__init__(node_name)
        
        # Create service clients
        self.set_leds_client = self.create_client(SetLEDs, 'set_leds')
        self.play_effect_client = self.create_client(PlayEffect, 'play_effect')
        self.get_state_client = self.create_client(GetLEDState, 'get_led_state')
        self.control_client = self.create_client(ControlLEDs, 'control_leds')
        
        # Wait for services with timeout
        self.get_logger().info('Waiting for LED services...')
        services = [
            self.set_leds_client,
            self.play_effect_client,
            self.get_state_client,
            self.control_client
        ]
        
        for service in services:
            if not service.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('LED services not available!')
                raise RuntimeError('LED services not available')
        
        self.get_logger().info('LED client ready!')
    
    def _parse_color(self, color):
        """
        Parse color input to RGB tuple.
        
        Args:
            color: Can be:
                - Color name string (e.g., 'red', 'blue')
                - Color object from emorobcare_leds.colors
                - RGB tuple (r, g, b)
                - RGB list [r, g, b]
                - Hex string (e.g., '#FF0000')
        
        Returns:
            Tuple of (r, g, b) values (0-255)
        """
        # Handle Color object from emorobcare_leds
        if HAS_COLORS_LIB and isinstance(color, Color):
            return color.to_tuple()
        
        if isinstance(color, str):
            # Check if hex color
            if color.startswith('#'):
                if HAS_COLORS_LIB:
                    # Use Colors.rgb_from_hex if available
                    return Colors.rgb_from_hex(color)
                else:
                    # Fallback hex parsing
                    color = color.lstrip('#')
                    if len(color) == 6:
                        r = int(color[0:2], 16)
                        g = int(color[2:4], 16)
                        b = int(color[4:6], 16)
                        return (r, g, b)
                    else:
                        raise ValueError(f"Invalid hex color: #{color}")
            
            # Check if color name
            color_lower = color.lower()
            if color_lower in self.COLORS:
                return self.COLORS[color_lower]
            else:
                raise ValueError(
                    f"Unknown color: '{color}'. "
                    f"Available colors: {', '.join(sorted(self.COLORS.keys()))}"
                )
        
        elif isinstance(color, (tuple, list)):
            if len(color) == 3:
                r, g, b = color
                if all(0 <= c <= 255 for c in color):
                    return (int(r), int(g), int(b))
                else:
                    raise ValueError(f"RGB values must be 0-255, got: {color}")
            else:
                raise ValueError(f"RGB tuple/list must have 3 values, got: {color}")
        
        else:
            raise ValueError(f"Invalid color format: {color}")
    
    def set_color(self, color, led_id=-1, fade_time=0.0):
        """
        Set LED(s) to a specific color.
        
        Args:
            color: Color name (e.g., 'red'), RGB tuple (255, 0, 0), or hex '#FF0000'
            led_id: LED index (0-4) or -1 for all LEDs (default: -1)
            fade_time: Fade duration in seconds (default: 0.0)
        
        Returns:
            bool: True if successful
        """
        r, g, b = self._parse_color(color)
        
        req = SetLEDs.Request()
        req.led_id = led_id
        req.r_values = [r]
        req.g_values = [g]
        req.b_values = [b]
        req.fade_time = fade_time
        req.wait_response = False
        
        future = self.set_leds_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            if not result.success:
                self.get_logger().error(f'Failed: {result.message}')
            return result.success
        else:
            self.get_logger().error('Service call failed')
            return False
    
    def set_colors(self, colors, fade_time=0.0):
        """
        Set each LED to a different color.
        
        Args:
            colors: List of colors (names, RGB tuples, or hex strings)
            fade_time: Fade duration in seconds (default: 0.0)
        
        Returns:
            bool: True if successful
        """
        rgb_colors = [self._parse_color(c) for c in colors]
        
        req = SetLEDs.Request()
        req.led_id = -1
        req.r_values = [c[0] for c in rgb_colors]
        req.g_values = [c[1] for c in rgb_colors]
        req.b_values = [c[2] for c in rgb_colors]
        req.fade_time = fade_time
        req.wait_response = False
        
        future = self.set_leds_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            if not result.success:
                self.get_logger().error(f'Failed: {result.message}')
            return result.success
        else:
            self.get_logger().error('Service call failed')
            return False
    
    def off(self, fade_time=0.0):
        """
        Turn off all LEDs.
        
        Args:
            fade_time: Fade duration in seconds (default: 0.0)
        
        Returns:
            bool: True if successful
        """
        req = ControlLEDs.Request()
        req.command = 'turn_off_all'
        req.fade_time = fade_time
        
        future = self.control_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        else:
            return False
    
    def play_effect(self, effect_name, duration=3.0, color1=None, color2=None, **kwargs):
        """
        Play a LED effect.
        
        Args:
            effect_name: Effect name (rainbow, cascade, heartbeat, wave, etc.)
            duration: Duration in seconds (default: 3.0)
            color1: Primary color (name, RGB, or hex) - optional
            color2: Secondary color (name, RGB, or hex) - optional
            **kwargs: Additional effect parameters (speed, cycles, direction, etc.)
        
        Returns:
            bool: True if successful
        """
        req = PlayEffect.Request()
        req.effect_name = effect_name
        req.duration = duration
        req.speed = kwargs.get('speed', 0.1)
        req.cycles = kwargs.get('cycles', 3)
        req.fade_time = kwargs.get('fade_time', 0.0)
        req.direction = kwargs.get('direction', 'forward')
        req.density = kwargs.get('density', 0.3)
        req.progress = kwargs.get('progress', 0.5)
        
        # Parse colors if provided
        if color1 is not None:
            r, g, b = self._parse_color(color1)
            req.color1_r = r
            req.color1_g = g
            req.color1_b = b
        
        if color2 is not None:
            r, g, b = self._parse_color(color2)
            req.color2_r = r
            req.color2_g = g
            req.color2_b = b
        
        # Handle color arrays
        if 'colors' in kwargs:
            colors = kwargs['colors']
            rgb_colors = [self._parse_color(c) for c in colors]
            req.colors_r = [c[0] for c in rgb_colors]
            req.colors_g = [c[1] for c in rgb_colors]
            req.colors_b = [c[2] for c in rgb_colors]
        
        future = self.play_effect_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=max(15.0, duration + 5.0))
        
        if future.result() is not None:
            result = future.result()
            if not result.success:
                self.get_logger().error(f'Failed: {result.message}')
            return result.success
        else:
            self.get_logger().error('Service call failed')
            return False
    
    def rainbow(self, duration=5.0):
        """Play rainbow effect."""
        return self.play_effect('rainbow', duration=duration)
    
    def heartbeat(self, color='red', duration=3.0):
        """Play heartbeat effect."""
        return self.play_effect('heartbeat', duration=duration, color1=color)
    
    def blink(self, colors=None, cycles=5):
        """Play blink effect."""
        if colors is None:
            colors = ['red', 'green', 'blue', 'yellow', 'magenta']
        return self.play_effect('blink', cycles=cycles, colors=colors)
    
    def cascade(self, color='cyan', direction='forward', cycles=3):
        """Play cascade effect."""
        return self.play_effect('cascade', color1=color, direction=direction, cycles=cycles)
    
    def wave(self, color1='blue', color2='cyan', duration=4.0):
        """Play wave effect."""
        return self.play_effect('wave', duration=duration, color1=color1, color2=color2)
    
    def knight_rider(self, color='red', cycles=3):
        """Play Knight Rider scanner effect."""
        return self.play_effect('knight_rider', color1=color, cycles=cycles)
    
    def emergency(self, cycles=10):
        """Play emergency blink effect."""
        return self.play_effect('emergency', cycles=cycles)
    
    def progress_bar(self, progress, filled_color='green', empty_color='red'):
        """
        Show progress bar.
        
        Args:
            progress: Progress value 0.0 to 1.0 (0% to 100%)
            filled_color: Color for completed portion
            empty_color: Color for remaining portion
        """
        return self.play_effect(
            'progress_bar',
            progress=progress,
            color1=filled_color,
            color2=empty_color
        )
    
    def get_state(self):
        """
        Get current LED state.
        
        Returns:
            dict: LED state information or None if failed
        """
        req = GetLEDState.Request()
        req.get_colors = True
        
        future = self.get_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            if result.success:
                return {
                    'num_leds': result.num_leds,
                    'colors': [
                        (result.current_r[i], result.current_g[i], result.current_b[i])
                        for i in range(len(result.current_r))
                    ],
                    'is_fading': result.is_fading
                }
            else:
                self.get_logger().error(f'Failed: {result.message}')
                return None
        else:
            self.get_logger().error('Service call failed')
            return None
    
    @classmethod
    def list_colors(cls):
        """Return list of available color names."""
        return sorted(cls.COLORS.keys())
    
    @classmethod
    def get_color_rgb(cls, color_name):
        """Get RGB values for a color name."""
        color_lower = color_name.lower()
        if color_lower in cls.COLORS:
            return cls.COLORS[color_lower]
        else:
            return None


def main():
    """Demo the high-level LED client."""
    rclpy.init()
    
    try:
        client = LEDClient()
        
        print("\n=== High-Level LED Client Demo ===\n")
        
        print("Available colors:", ', '.join(client.list_colors()[:10]), "...\n")
        
        print("Setting all LEDs to red...")
        client.set_color('red', fade_time=1.0)
        
        import time
        time.sleep(2)
        
        print("Setting all LEDs to blue...")
        client.set_color('blue', fade_time=1.0)
        
        time.sleep(2)
        
        print("Setting each LED to different colors...")
        client.set_colors(['red', 'green', 'blue', 'yellow', 'magenta'], fade_time=1.0)
        
        time.sleep(2)
        
        print("Playing rainbow effect...")
        client.rainbow(duration=3.0)
        
        time.sleep(1)
        
        print("Playing heartbeat (purple)...")
        client.heartbeat(color='purple', duration=3.0)
        
        time.sleep(1)
        
        print("Turning off...")
        client.off(fade_time=1.0)
        
        print("\nDemo complete!")
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
