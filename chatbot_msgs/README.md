# chatbot_msgs

Message, service, and action definitions for chatbot integration in ROS 2.

## Table of Contents

1. [Overview](#overview)
2. [Dependencies](#dependencies)
3. [Package Contents](#package-contents)
4. [Building](#building)
5. [Testing Services](#testing-services)
6. [Examples](#examples)
7. [License](#license)

## Overview

The `chatbot_msgs` package provides standardized interfaces for integrating conversational AI systems with ROS 2-based robotic platforms. It defines message, service, and action definitions for dialogue interactions.

**Note:** This package only defines interfaces. For a working implementation, see the [`llm_bridge`](../../../src/llm_bridge) package which provides service implementations.

## Dependencies

- **hri_actions_msgs**: Human-Robot Interaction action definitions
- **unique_identifier_msgs**: UUID message types

## Package Contents

### Messages

- **`DialogueRole`**: Defines the role and configuration for a dialogue
  - `name`: Dialogue role name (e.g., `"__default__"` or `"__ask__"`)
  - `configuration`: Backend-specific JSON configuration

### Actions

- **`Dialogue`**: Initiates and manages a dialogue with a defined role
  - Request: `role` (DialogueRole), `locale` (string, optional)
  - Result: `results` (JSON string), `error_msg` (string)

### Services

| Service | Purpose | Implementation |
|---------|---------|----------------|
| **`KeywordAction`** | Generates a robot response for an action description | [`llm_bridge`](../../../src/llm_bridge) |
| **`DialogueInteraction`** | Adds entry to dialogue history and gets chatbot response | [`llm_bridge`](../../../src/llm_bridge) |
| **`GetResponse`** | Retrieves response from chatbot for given input | [`llm_bridge`](../../../src/llm_bridge) |
| **`Rephrase`** | Generates alternative phrasings of text | [`llm_bridge`](../../../src/llm_bridge) |
| **`EvaluateAnswer`** | Evaluates correctness/quality of an answer | [`llm_bridge`](../../../src/llm_bridge) |
| **`ResetModel`** | Resets the chatbot model to initial state | [`llm_bridge`](../../../src/llm_bridge) |

#### KeywordAction Service Details

```
Request:
  action_description (string): What the robot should do/say
  
Response:
  robot_response (string): Generated response
  error_msg (string): Error message if failed
```

**Examples:**
- "greet the boy, named 'Pepe' in a friendly way"
- "tell the child to sit down calmly"
- "encourage the child to try again"

## Building

```bash
colcon build --packages-select chatbot_msgs
source install/setup.bash
```

## Testing Services

**⚠️ Important:** `chatbot_msgs` only defines interfaces. You need a service provider to test these services.

### Recommended Service Provider

Use **[`llm_bridge`](../../../src/llm_bridge)** package which implements all these services:

```bash
# Terminal 1: Start the service provider
ros2 launch llm_bridge llm_bridge.launch.py
```

### Service Architecture

```
┌─────────────────┐         ┌──────────────┐         ┌─────────────────┐
│  chatbot_msgs   │         │  llm_bridge  │         │  External LLM   │
│  (Interfaces)   │ <------ │ (Implements) │ <-----> │   API Server    │
└─────────────────┘         └──────────────┘         └─────────────────┘
         ▲                                                     │
         │                                                     │
         └─────────────── Your ROS 2 Application ─────────────┘
```

- **`chatbot_msgs`**: Defines service contracts (this package)
- **`llm_bridge`**: Provides service implementations
- **Your App**: Consumes services using the defined interfaces

### Available Services

List all services:
```bash
ros2 service list
```

Check service interface:
```bash
ros2 service type /keyword_action
```

### Testing with Command Line

Start the service provider first (see [`llm_bridge`](../../../src/llm_bridge)):
```bash
ros2 launch llm_bridge llm_bridge.launch.py
```

Then test each service:

#### KeywordAction
```bash
ros2 service call /chatbot/keyword_action chatbot_msgs/srv/KeywordAction \
  '{action_description: "greet the boy, named Pepe in a friendly way"}'
```

#### GetResponse
```bash
ros2 service call /chatbot/get_response chatbot_msgs/srv/GetResponse \
  '{user_id: "user123", model: "default", locale: "en_US", input: "What is the weather?"}'
```

#### Rephrase
```bash
ros2 service call /chatbot/rephrase chatbot_msgs/srv/Rephrase \
  '{sentence: "Hello, how are you?", forbidden_expressions: [], n_alternatives: 3}'
```

#### EvaluateAnswer
```bash
ros2 service call /chatbot/evaluate_answer chatbot_msgs/srv/EvaluateAnswer \
  '{question: "What is 2+2?", answer: "4", expected_answer: "4"}'
```

#### ResetModel
```bash
ros2 service call /chatbot/reset chatbot_msgs/srv/ResetModel '{}'
```

## Examples

### Using in Your Package

Add to your `package.xml`:
```xml
<depend>chatbot_msgs</depend>
```

**Python:**
```python
from chatbot_msgs.action import Dialogue
from chatbot_msgs.msg import DialogueRole
from chatbot_msgs.srv import DialogueInteraction, KeywordAction
```

**C++:**
```cpp
#include "chatbot_msgs/action/dialogue.hpp"
#include "chatbot_msgs/msg/dialogue_role.hpp"
#include "chatbot_msgs/srv/dialogue_interaction.hpp"
```

### Starting a Dialogue (Python)

```python
import rclpy
from rclpy.action import ActionClient
from chatbot_msgs.action import Dialogue
from chatbot_msgs.msg import DialogueRole

rclpy.init()
node = rclpy.create_node('dialogue_client')
client = ActionClient(node, Dialogue, '/dialogue')
client.wait_for_server()

goal = Dialogue.Goal()
goal.role = DialogueRole(name='__default__')
goal.locale = 'en_US'

client.send_goal_async(goal)
rclpy.spin(node)
```

## License

Apache License 2.0 - See LICENSE file for details.

## Maintainers & Authors

- Séverin Lemaignan (severin.lemaignan@pal-robotics.com)
- Sara Cooper (sara.cooper@pal-robotics.com)
- Luka Juricic (luka.juricic@pal-robotics.com)
