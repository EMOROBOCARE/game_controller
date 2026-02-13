"""Launch file for game_controller node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for game_controller."""
    
    # Declare launch arguments
    difficulty_arg = DeclareLaunchArgument(
        "difficulty",
        default_value="basic",
        description="Game difficulty level (basic, intermediate, advanced)",
    )
    
    rounds_per_phase_arg = DeclareLaunchArgument(
        "rounds_per_phase",
        default_value="2",
        description="Number of rounds per phase",
    )
    
    phases_arg = DeclareLaunchArgument(
        "phases",
        default_value="['P1', 'P2', 'P3']",
        description="List of phases to include",
    )
    
    # Game controller node
    game_controller_node = Node(
        package="game_controller",
        executable="game_controller_node",
        name="game_controller",
        parameters=[
            {
                # Topics
                "topics.decision_state": "/decision/state",
                "topics.intents": "/intents",
                "topics.game_selector": "/game/game_selector",
                "topics.user_selector": "/game/user_selector",
                "topics.decision_events": "/decision/events",
                "topics.current_user": "/game/current_user",
                "input_bridge.enabled": False,
                
                # Generic UI
                "generic_ui.update_manifest_service": "/generic_ui/update_manifest",
                "generic_ui.manifest_timeout_sec": 5.0,
                
                # Auto-advance timeouts
                "auto_advance.phase_intro": 0.0,
                "auto_advance.round_setup": 0.05,
                "auto_advance.question_present": 0.05,
                "auto_advance.fail_l1": 2.0,
                "auto_advance.fail_l2": 2.0,
                "auto_advance.correct": 0.6,
                "auto_advance.phase_complete": 0.3,

                # Chatbot / speech behavior
                "chatbot.enabled": True,
                "chatbot.rephrase_service": "/chatbot/rephrase",
                "chatbot.evaluate_service": "/chatbot/evaluate_answer",
                "chatbot.service_wait_timeout_sec": 0.05,
                "chatbot.evaluate_speech_only": True,
                "tts.enabled": True,
                "tts.rephrase_question_enabled": True,
                "tts.rephrase_correct_enabled": True,
                
                # Game defaults
                "game_defaults.difficulty": LaunchConfiguration("difficulty"),
                "game_defaults.rounds_per_phase": LaunchConfiguration("rounds_per_phase"),
                "game_defaults.phases": LaunchConfiguration("phases"),
            }
        ],
        output="screen",
    )
    
    return LaunchDescription([
        difficulty_arg,
        rounds_per_phase_arg,
        phases_arg,
        game_controller_node,
    ])
