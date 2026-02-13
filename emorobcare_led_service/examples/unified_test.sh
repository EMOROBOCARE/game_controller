#!/bin/bash
# Unified LED Service Test Script
# All examples consolidated into a single script using ros2 service calls
# Tests all functionalities with error tracking and summary report

# Set environment
source ~/emorobot_ws/install/setup.bash
export ROS_DOMAIN_ID=10

# Error tracking
declare -a FAILED_TESTS
TEST_COUNT=0
FAILED_COUNT=0

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "======================================================"
echo "EMOROBCARE LED Service - Unified Test Suite"
echo "======================================================"
echo ""

# Helper function to run a test and track failures
run_test() {
    local test_name="$1"
    local service_call="$2"
    local wait_time="${3:-1}"
    
    TEST_COUNT=$((TEST_COUNT + 1))
    echo -n "[$TEST_COUNT] $test_name... "
    
    # Run the service call and capture output
    if output=$(eval "$service_call" 2>&1); then
        echo -e "${GREEN}OK${NC}"
        sleep "$wait_time"
        return 0
    else
        echo -e "${RED}FAILED${NC}"
        FAILED_TESTS+=("Test $TEST_COUNT: $test_name")
        FAILED_COUNT=$((FAILED_COUNT + 1))
        sleep 0.5
        return 1
    fi
}

# Check if services are available
echo "Checking if LED services are available..."
echo "Current ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

if ! ros2 service list 2>/dev/null | grep -q "set_leds"; then
    echo -e "${RED}ERROR: LED services not found.${NC}"
    echo "Please start the led_service_node first."
    echo "Make sure ROS_DOMAIN_ID is set correctly (should match the service)."
    exit 1
fi

echo -e "${GREEN}Services found. Starting tests...${NC}"
echo ""

# ============================================================
# TEST SECTION 1: Basic Color Control (RGB)
# ============================================================
echo "========================================="
echo "Section 1: Basic Color Control (RGB)"
echo "========================================="

run_test "Set all LEDs to RED with fade" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [255], g_values: [0], b_values: [0], fade_time: 1.0}'" \
    2

run_test "Set all LEDs to GREEN with fade" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [0], g_values: [255], b_values: [0], fade_time: 1.0}'" \
    2

run_test "Set all LEDs to BLUE with fade" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [0], g_values: [0], b_values: [255], fade_time: 1.0}'" \
    2

run_test "Set all LEDs to WHITE instantly" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [255], g_values: [255], b_values: [255], fade_time: 0.0}'" \
    1

run_test "Set all LEDs to YELLOW" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [255], g_values: [255], b_values: [0], fade_time: 1.0}'" \
    2

run_test "Set all LEDs to CYAN" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [0], g_values: [255], b_values: [255], fade_time: 1.0}'" \
    2

run_test "Set all LEDs to MAGENTA" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [255], g_values: [0], b_values: [255], fade_time: 1.0}'" \
    2

# ============================================================
# TEST SECTION 2: Special Colors
# ============================================================
echo ""
echo "========================================="
echo "Section 2: Special EMOROBOT Colors"
echo "========================================="

run_test "Set LEDs to WARM WHITE (180,140,60)" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [180], g_values: [140], b_values: [60], fade_time: 0.5}'" \
    2

run_test "Set LEDs to COOL WHITE (200,220,255)" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [200], g_values: [220], b_values: [255], fade_time: 0.5}'" \
    2

run_test "Set LEDs to CARE BLUE (0,120,215)" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [0], g_values: [120], b_values: [215], fade_time: 0.5}'" \
    2

run_test "Set LEDs to WARNING RED (220,20,60)" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [220], g_values: [20], b_values: [60], fade_time: 0.5}'" \
    2

run_test "Set LEDs to SUCCESS GREEN (34,139,34)" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [34], g_values: [139], b_values: [34], fade_time: 0.5}'" \
    2

run_test "Set LEDs to ORANGE (255,165,0)" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [255], g_values: [165], b_values: [0], fade_time: 0.5}'" \
    2

# ============================================================
# TEST SECTION 3: Individual LED Control
# ============================================================
echo ""
echo "========================================="
echo "Section 3: Individual LED Control"
echo "========================================="

run_test "Set each LED to different color (R,G,B,Y,M)" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [255,0,0,255,128], g_values: [0,255,0,255,0], b_values: [0,0,255,0,128], fade_time: 1.0}'" \
    2

run_test "Set LED 0 to CYAN" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: 0, r_values: [0], g_values: [255], b_values: [255], fade_time: 0.5}'" \
    2

run_test "Set LED 2 to WHITE" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: 2, r_values: [255], g_values: [255], b_values: [255], fade_time: 0.5}'" \
    2

run_test "Set LED 4 to MAGENTA" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: 4, r_values: [255], g_values: [0], b_values: [255], fade_time: 0.5}'" \
    2


# ============================================================
# TEST SECTION 4: LED State Query
# ============================================================
echo ""
echo "========================================="
echo "Section 4: LED State Query"
echo "========================================="

run_test "Get current LED state" \
    "ros2 service call /get_led_state emorobcare_led_service/srv/GetLEDState '{get_colors: true}'" \
    1

# ============================================================
# TEST SECTION 5: Basic Effects
# ============================================================
echo ""
echo "========================================="
echo "Section 5: Basic LED Effects"
echo "========================================="

run_test "Play RAINBOW effect (5s)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"rainbow\", duration: 5.0, speed: 0.1}'" \
    1

run_test "Play HEARTBEAT effect (red, 3s)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"heartbeat\", duration: 3.0, color1_r: 255, color1_g: 0, color1_b: 0}'" \
    1

run_test "Play HEARTBEAT effect (purple, 3s)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"heartbeat\", duration: 3.0, color1_r: 128, color1_g: 0, color1_b: 128}'" \
    1

run_test "Play HEARTBEAT effect (blue, 3s)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"heartbeat\", duration: 3.0, color1_r: 0, color1_g: 0, color1_b: 255}'" \
    1

# ============================================================
# TEST SECTION 6: Cascade Effects
# ============================================================
echo ""
echo "========================================="
echo "Section 6: Cascade Effects"
echo "========================================="

run_test "Play CASCADE (forward, cyan, 3 cycles)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"cascade\", cycles: 3, direction: \"forward\", color1_r: 0, color1_g: 255, color1_b: 255, speed: 0.1}'" \
    1

run_test "Play CASCADE (backward, red, 3 cycles)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"cascade\", cycles: 3, direction: \"backward\", color1_r: 255, color1_g: 0, color1_b: 0, speed: 0.1}'" \
    1

run_test "Play CASCADE (forward, green, 2 cycles)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"cascade\", cycles: 2, direction: \"forward\", color1_r: 0, color1_g: 255, color1_b: 0, speed: 0.15}'" \
    1

# ============================================================
# TEST SECTION 7: Wave Effects
# ============================================================
echo ""
echo "========================================="
echo "Section 7: Wave Effects"
echo "========================================="

run_test "Play WAVE (blue to cyan, 4s)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"wave\", duration: 4.0, color1_r: 0, color1_g: 0, color1_b: 255, color2_r: 0, color2_g: 255, color2_b: 255}'" \
    1

run_test "Play WAVE (red to yellow, 4s)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"wave\", duration: 4.0, color1_r: 255, color1_g: 0, color1_b: 0, color2_r: 255, color2_g: 255, color2_b: 0}'" \
    1

# ============================================================
# TEST SECTION 8: Blink Effects
# ============================================================
echo ""
echo "========================================="
echo "Section 8: Blink Effects"
echo "========================================="

run_test "Play BLINK (multi-color, 5 cycles)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"blink\", cycles: 5, speed: 0.3, fade_time: 0.1, colors_r: [255,0,0,255,255], colors_g: [0,255,0,255,0], colors_b: [0,0,255,0,255]}'" \
    1

run_test "Play BLINK (two colors, 3 cycles)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"blink\", cycles: 3, speed: 0.4, colors_r: [255,0], colors_g: [0,255], colors_b: [0,0]}'" \
    1

run_test "Play BLINK_SINGLE (LED 0, cyan, 10 cycles)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"blink_single\", cycles: 10, speed: 0.2, color1_r: 0, color1_g: 255, color1_b: 255}'" \
    1

run_test "Play BLINK_SINGLE (LED 0, red, 5 cycles)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"blink_single\", cycles: 5, speed: 0.3, color1_r: 255, color1_g: 0, color1_b: 0}'" \
    1

# ============================================================
# TEST SECTION 9: Advanced Effects
# ============================================================
echo ""
echo "========================================="
echo "Section 9: Advanced Effects"
echo "========================================="

run_test "Play SPARKLE (4s)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"sparkle\", duration: 4.0, density: 0.4, speed: 0.1, colors_r: [255,255,0], colors_g: [255,255,255], colors_b: [255,0,255]}'" \
    1

run_test "Play KNIGHT RIDER (red, 3 cycles)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"knight_rider\", cycles: 3, color1_r: 255, color1_g: 0, color1_b: 0, speed: 0.1}'" \
    1

run_test "Play KNIGHT RIDER (cyan, 2 cycles)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"knight_rider\", cycles: 2, color1_r: 0, color1_g: 255, color1_b: 255, speed: 0.15}'" \
    1


run_test "Play EMERGENCY (5 cycles)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"emergency\", cycles: 5}'" \
    1

run_test "Play EMERGENCY (10 cycles)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"emergency\", cycles: 10}'" \
    1

run_test "Play COLOR CYCLE (4 colors, 2 cycles)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"color_cycle\", cycles: 2, speed: 0.5, colors_r: [255,0,0,255], colors_g: [0,255,0,255], colors_b: [0,0,255,0]}'" \
    1

run_test "Play COLOR CYCLE (3 colors, 3 cycles)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"color_cycle\", cycles: 3, speed: 0.4, colors_r: [255,0,255], colors_g: [0,255,0], colors_b: [0,255,255]}'" \
    1

# ============================================================
# TEST SECTION 10: Progress Bar
# ============================================================
echo ""
echo "========================================="
echo "Section 10: Progress Bar Effect"
echo "========================================="

run_test "Progress Bar 0% (green/red)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"progress_bar\", progress: 0.0, color1_r: 0, color1_g: 255, color1_b: 0, color2_r: 255, color2_g: 0, color2_b: 0}'" \
    0.5

run_test "Progress Bar 20% (green/red)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"progress_bar\", progress: 0.2, color1_r: 0, color1_g: 255, color1_b: 0, color2_r: 255, color2_g: 0, color2_b: 0}'" \
    0.5

run_test "Progress Bar 40% (green/red)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"progress_bar\", progress: 0.4, color1_r: 0, color1_g: 255, color1_b: 0, color2_r: 255, color2_g: 0, color2_b: 0}'" \
    0.5

run_test "Progress Bar 60% (green/red)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"progress_bar\", progress: 0.6, color1_r: 0, color1_g: 255, color1_b: 0, color2_r: 255, color2_g: 0, color2_b: 0}'" \
    0.5

run_test "Progress Bar 80% (green/red)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"progress_bar\", progress: 0.8, color1_r: 0, color1_g: 255, color1_b: 0, color2_r: 255, color2_g: 0, color2_b: 0}'" \
    0.5

run_test "Progress Bar 100% (green/red)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"progress_bar\", progress: 1.0, color1_r: 0, color1_g: 255, color1_b: 0, color2_r: 255, color2_g: 0, color2_b: 0}'" \
    1

run_test "Progress Bar 50% (blue/yellow)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"progress_bar\", progress: 0.5, color1_r: 0, color1_g: 0, color1_b: 255, color2_r: 255, color2_g: 255, color2_b: 0}'" \
    1

run_test "Progress Bar 75% (success green/warning red)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"progress_bar\", progress: 0.75, color1_r: 34, color1_g: 139, color1_b: 34, color2_r: 220, color2_g: 20, color2_b: 60}'" \
    1

# ============================================================
# TEST SECTION 11: Control Commands
# ============================================================
echo ""
echo "========================================="
echo "Section 11: Control Commands"
echo "========================================="

run_test "Stop fade animation" \
    "ros2 service call /control_leds emorobcare_led_service/srv/ControlLEDs '{command: \"stop_fade\"}'" \
    1

run_test "Set LEDs to yellow (for fade test)" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [255], g_values: [255], b_values: [0], fade_time: 0.0}'" \
    1

run_test "Turn off all LEDs instantly" \
    "ros2 service call /control_leds emorobcare_led_service/srv/ControlLEDs '{command: \"turn_off_all\", fade_time: 0.0}'" \
    1

run_test "Set LEDs to white (for final fade test)" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [255], g_values: [255], b_values: [255], fade_time: 0.0}'" \
    1

run_test "Turn off all LEDs with 3s fade" \
    "ros2 service call /control_leds emorobcare_led_service/srv/ControlLEDs '{command: \"turn_off_all\", fade_time: 3.0}'" \
    1

# ============================================================
# TEST SECTION 12: Error Handling and Edge Cases
# ============================================================
echo ""
echo "========================================="
echo "Section 12: Error Handling and Edge Cases"
echo "========================================="

run_test "Test invalid LED ID (should fail gracefully)" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: 99, r_values: [255], g_values: [0], b_values: [0], fade_time: 0.0}'" \
    0.5

run_test "Test invalid RGB values (should fail gracefully)" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [300], g_values: [0], b_values: [0], fade_time: 0.0}'" \
    0.5

run_test "Test empty RGB arrays (should fail gracefully)" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [], g_values: [], b_values: [], fade_time: 0.0}'" \
    0.5

run_test "Test mismatched RGB array lengths (should fail gracefully)" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [255,0], g_values: [0], b_values: [0,0], fade_time: 0.0}'" \
    0.5

run_test "Test invalid effect name (should fail gracefully)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"invalid_effect\", duration: 1.0}'" \
    0.5

run_test "Test invalid direction (should fail gracefully)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"cascade\", direction: \"invalid\", cycles: 1}'" \
    0.5

run_test "Test invalid color values in effect (should fail gracefully)" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"heartbeat\", color1_r: 300, color1_g: 0, color1_b: 0}'" \
    0.5

run_test "Test invalid control command (should fail gracefully)" \
    "ros2 service call /control_leds emorobcare_led_service/srv/ControlLEDs '{command: \"invalid_command\"}'" \
    0.5

# ============================================================
# TEST SECTION 13: Advanced Parameter Testing
# ============================================================
echo ""
echo "========================================="
echo "Section 13: Advanced Parameter Testing"
echo "========================================="

run_test "Test fade_time parameter variations" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [128], g_values: [128], b_values: [128], fade_time: 2.5}'" \
    3

run_test "Test wait_response parameter" \
    "ros2 service call /set_leds emorobcare_led_service/srv/SetLEDs '{led_id: -1, r_values: [64], g_values: [64], b_values: [64], fade_time: 1.0, wait_response: true}'" \
    2

run_test "Test effect with custom density parameter" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"sparkle\", duration: 3.0, density: 0.8, speed: 0.2, colors_r: [255,255,255], colors_g: [255,255,255], colors_b: [255,255,255]}'" \
    1

run_test "Test effect with custom fade_time parameter" \
    "ros2 service call /play_effect emorobcare_led_service/srv/PlayEffect '{effect_name: \"blink\", cycles: 3, speed: 0.3, fade_time: 0.2, colors_r: [255,0], colors_g: [0,255], colors_b: [0,0]}'" \
    1


# ============================================================
# TEST SECTION 14: State Query Variations
# ============================================================
echo ""
echo "========================================="
echo "Section 14: State Query Variations"
echo "========================================="

run_test "Get LED state without colors" \
    "ros2 service call /get_led_state emorobcare_led_service/srv/GetLEDState '{get_colors: false}'" \
    0.5

run_test "Get LED state with colors" \
    "ros2 service call /get_led_state emorobcare_led_service/srv/GetLEDState '{get_colors: true}'" \
    0.5

# ============================================================
# TEST SECTION 15: String-Based Color Control
# ============================================================
echo ""
echo "========================================="
echo "Section 15: String-Based Color Control"
echo "========================================="

run_test "Set all LEDs to RED using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"red\", fade_time: 1.0}'" \
    2

run_test "Set all LEDs to GREEN using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"green\", fade_time: 1.0}'" \
    2

run_test "Set all LEDs to BLUE using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"blue\", fade_time: 1.0}'" \
    2

run_test "Set all LEDs to WHITE using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"white\", fade_time: 0.5}'" \
    1

run_test "Set all LEDs to YELLOW using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"yellow\", fade_time: 0.5}'" \
    1

run_test "Set all LEDs to CYAN using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"cyan\", fade_time: 0.5}'" \
    1

run_test "Set all LEDs to MAGENTA using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"magenta\", fade_time: 0.5}'" \
    1

# ============================================================
# TEST SECTION 16: Special EMOROBOT Colors (String Names)
# ============================================================
echo ""
echo "========================================="
echo "Section 16: Special EMOROBOT Colors (String Names)"
echo "========================================="

run_test "Set LEDs to WARM_WHITE using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"warm_white\", fade_time: 0.5}'" \
    2

run_test "Set LEDs to COOL_WHITE using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"cool_white\", fade_time: 0.5}'" \
    2

run_test "Set LEDs to CARE_BLUE using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"care_blue\", fade_time: 0.5}'" \
    2

run_test "Set LEDs to WARNING_RED using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"warning_red\", fade_time: 0.5}'" \
    2

run_test "Set LEDs to SUCCESS_GREEN using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"success_green\", fade_time: 0.5}'" \
    2

run_test "Set LEDs to ORANGE using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"orange\", fade_time: 0.5}'" \
    2

run_test "Set LEDs to PURPLE using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"purple\", fade_time: 0.5}'" \
    2

run_test "Set LEDs to PINK using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"pink\", fade_time: 0.5}'" \
    2

# ============================================================
# TEST SECTION 17: Individual LED Control (String Names)
# ============================================================
echo ""
echo "========================================="
echo "Section 17: Individual LED Control (String Names)"
echo "========================================="

run_test "Set LED 0 to RED using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: 0, color_name: \"red\", fade_time: 0.5}'" \
    2

run_test "Set LED 1 to GREEN using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: 1, color_name: \"green\", fade_time: 0.5}'" \
    2

run_test "Set LED 2 to BLUE using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: 2, color_name: \"blue\", fade_time: 0.5}'" \
    2

run_test "Set LED 3 to YELLOW using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: 3, color_name: \"yellow\", fade_time: 0.5}'" \
    2

run_test "Set LED 4 to MAGENTA using string name" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: 4, color_name: \"magenta\", fade_time: 0.5}'" \
    2

# ============================================================
# TEST SECTION 18: String Service Error Handling
# ============================================================
echo ""
echo "========================================="
echo "Section 18: String Service Error Handling"
echo "========================================="

run_test "Test invalid color name (should fail gracefully)" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"invalid_color\", fade_time: 0.0}'" \
    0.5

run_test "Test empty color name (should fail gracefully)" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"\", fade_time: 0.0}'" \
    0.5

run_test "Test invalid LED ID with string service (should fail gracefully)" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: 99, color_name: \"red\", fade_time: 0.0}'" \
    0.5

run_test "Test case insensitive color name (RED should work)" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"RED\", fade_time: 0.5}'" \
    1

run_test "Test color name with spaces trimmed (\"  blue  \" should work)" \
    "ros2 service call /set_leds_string emorobcare_led_service/srv/SetLEDsString '{led_id: -1, color_name: \"  blue  \", fade_time: 0.5}'" \
    1

# ============================================================
# TEST SECTION 19: Available Colors Topic
# ============================================================
echo ""
echo "========================================="
echo "Section 19: Available Colors Topic"
echo "========================================="

echo "Listening to available_colors topic for 3 seconds..."
timeout 3s ros2 topic echo /available_colors --once || echo "Topic message received or timeout"

run_test "Check if available_colors topic is publishing" \
    "ros2 topic info /available_colors" \
    0.5

# ============================================================
# TEST SECTION 20: Final State Check
# ============================================================
echo ""
echo "========================================="
echo "Section 20: Final State Check"
echo "========================================="

run_test "Get final LED state" \
    "ros2 service call /get_led_state emorobcare_led_service/srv/GetLEDState '{get_colors: true}'" \
    1

# ============================================================
# TEST SUMMARY
# ============================================================
echo ""
echo "======================================================"
echo "TEST SUMMARY"
echo "======================================================"
echo ""
echo "Total Tests: $TEST_COUNT"
echo -e "Passed: ${GREEN}$((TEST_COUNT - FAILED_COUNT))${NC}"
echo -e "Failed: ${RED}$FAILED_COUNT${NC}"
echo ""

if [ $FAILED_COUNT -eq 0 ]; then
    echo -e "${GREEN}Tests completed without errors!${NC}"
    echo ""
    echo "======================================================"
    exit 0
else
    echo -e "${RED}The following tests failed:${NC}"
    echo ""
    for test in "${FAILED_TESTS[@]}"; do
        echo -e "  ${RED}✗${NC} $test"
    done
    echo ""
    echo "======================================================"
    exit 1
fi
