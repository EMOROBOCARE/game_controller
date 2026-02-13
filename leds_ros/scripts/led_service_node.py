#!/usr/bin/env python3

import os
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from emorobcare_led_service.srv import ControlLEDs, GetLEDState, PlayEffect, SetLEDs


def _clamp_color(value: int) -> int:
    return max(0, min(255, int(value)))


class LEDServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("emorobcare_led_service")
        try:
            requested = int(os.environ.get("LED_NUM_LEDS", "5"))
        except ValueError:
            requested = 5
        self._num_leds = max(1, requested)
        self._is_fading = False
        self._active_effect = ""
        self._rgb: List[Tuple[int, int, int]] = [(0, 0, 0)] * self._num_leds

        self.create_service(SetLEDs, "/set_leds", self._handle_set_leds)
        self.create_service(PlayEffect, "/play_effect", self._handle_play_effect)
        self.create_service(ControlLEDs, "/control_leds", self._handle_control_leds)
        self.create_service(GetLEDState, "/get_led_state", self._handle_get_led_state)

        self.get_logger().info(
            f"LED service ready (num_leds={self._num_leds}, mock_backend=true)"
        )

    def _set_all(self, red: int, green: int, blue: int) -> None:
        rgb = (_clamp_color(red), _clamp_color(green), _clamp_color(blue))
        self._rgb = [rgb] * self._num_leds

    def _safe_rgb_from_request(self, request: SetLEDs.Request) -> Tuple[int, int, int]:
        red = request.r_values[0] if request.r_values else 0
        green = request.g_values[0] if request.g_values else 0
        blue = request.b_values[0] if request.b_values else 0
        return _clamp_color(red), _clamp_color(green), _clamp_color(blue)

    def _handle_set_leds(self, request: SetLEDs.Request, response: SetLEDs.Response):
        red, green, blue = self._safe_rgb_from_request(request)
        led_id = int(request.led_id)

        if led_id < 0:
            self._set_all(red, green, blue)
        elif led_id < self._num_leds:
            current = list(self._rgb)
            current[led_id] = (red, green, blue)
            self._rgb = current
        else:
            response.success = False
            response.message = f"Invalid led_id {led_id} (num_leds={self._num_leds})"
            return response

        self._is_fading = bool(float(request.fade_time) > 0.0)
        self._active_effect = ""
        response.success = True
        response.message = "LEDs updated"
        return response

    def _handle_play_effect(
        self, request: PlayEffect.Request, response: PlayEffect.Response
    ):
        name = str(request.effect_name or "").strip().lower()
        self._active_effect = name
        self._is_fading = bool(float(request.fade_time) > 0.0)

        if name == "progress_bar":
            progress = max(0.0, min(1.0, float(request.progress)))
            lit = int(round(progress * self._num_leds))
            rgb = []
            for idx in range(self._num_leds):
                rgb.append((0, 255, 0) if idx < lit else (0, 0, 0))
            self._rgb = rgb
        elif name in {"blink", "heartbeat", "cascade", "knight_rider"}:
            self._set_all(0, 128, 255)
        elif name == "color_cycle":
            self._set_all(255, 255, 0)
        else:
            self._set_all(64, 64, 64)

        response.success = True
        response.message = f"Effect '{name or 'default'}' applied"
        return response

    def _handle_control_leds(
        self, request: ControlLEDs.Request, response: ControlLEDs.Response
    ):
        command = str(request.command or "").strip().lower()
        if command in {"off", "turn_off_all", "turn_off"}:
            self._set_all(0, 0, 0)
            self._active_effect = ""
            self._is_fading = bool(float(request.fade_time) > 0.0)
            response.success = True
            response.message = "LEDs turned off"
            return response

        if command in {"on", "turn_on_all", "turn_on"}:
            self._set_all(255, 255, 255)
            self._active_effect = ""
            self._is_fading = bool(float(request.fade_time) > 0.0)
            response.success = True
            response.message = "LEDs turned on"
            return response

        response.success = False
        response.message = f"Unsupported command: {command}"
        return response

    def _handle_get_led_state(
        self, request: GetLEDState.Request, response: GetLEDState.Response
    ):
        del request
        response.success = True
        response.message = (
            f"active_effect={self._active_effect}"
            if self._active_effect
            else "state_ready"
        )
        response.num_leds = int(self._num_leds)
        response.is_fading = bool(self._is_fading)
        response.current_r = [int(rgb[0]) for rgb in self._rgb]
        response.current_g = [int(rgb[1]) for rgb in self._rgb]
        response.current_b = [int(rgb[2]) for rgb in self._rgb]
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LEDServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
