# Colores CLI One-Liners (No UI Clicks)

All commands are intended to run from:

`/home/alono/EmorobCare/games_src/game_controller`

## 1) Start and Verify Stack

```bash
docker compose up -d
```

```bash
docker compose ps
```

```bash
docker compose exec -T game_controller env | rg -n "POSTGRES_|GAME_CONTROLLER_DB_ENABLED" -S
```

```bash
docker compose exec -T game_controller bash -lc 'python3 -c "import socket,os;h=os.getenv(\"POSTGRES_HOST\");p=int(os.getenv(\"POSTGRES_PORT\",\"5432\"));s=socket.socket();s.settimeout(2);s.connect((h,p));print(f\"db_tcp_ok {h}:{p}\");s.close()"'
```

## 2) Start Colores Full Game (P1..P6)

```bash
docker compose exec -T game_controller bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic pub --once /game/user_selector std_msgs/msg/String "data: \"1\""'
```

```bash
docker compose exec -T game_controller bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic pub --once /game/game_selector std_msgs/msg/String "{data: \"{\\\"game\\\":{\\\"slug\\\":\\\"colores\\\"},\\\"difficulty\\\":\\\"basic\\\",\\\"phases\\\":[\\\"P1\\\",\\\"P2\\\",\\\"P3\\\",\\\"P4\\\",\\\"P5\\\",\\\"P6\\\"],\\\"roundsPerPhase\\\":1}\"}"'
```

## 3) Observe Runtime State

```bash
docker compose exec -T decision_making bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo /decision/state'
```

```bash
docker compose logs -f --no-color game_controller communication_hub expressive_say_bridge tts_mock
```

## 4) Simulate User Input Without Screen

### 4.1 Through UI bridge path (`/ui/input -> /intents`)
The system should ignore touchscreen answer input in `P2` (speech-only), but accept it in other phases that allow touchscreen.
`game_controller` should log both reception and discard in `P2`.

```bash
docker compose exec -T game_controller bash -lc \
    'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    ros2 topic pub --once /ui/input std_msgs/msg/String "{data: '\''{\"label\":\"rojo\"}'\''}"'
```

### 4.2 Direct speech-like intent (`/intents`)
Use `speech` modality so input is accepted in `P2`.
```bash
docker compose exec -T communication_hub bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic pub --once /intents hri_actions_msgs/msg/Intent "{intent: \"__raw_user_input__\", data: \"{\\\"input\\\":\\\"rojo\\\"}\", modality: \"__modality_speech__\"}"'
```

## 5) Simulate Control Buttons (Pause/Resume/Skip/Reset/Stop)

Expected: `/decision/state` changes to `state=PAUSED` (transaction id increases).
```bash
docker compose exec -T game_controller bash -lc \
    'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    ros2 topic pub --once /ui/input std_msgs/msg/String "{data: '\''{\"action\":\"pause\"}'\''}"'
```
Verify:
```bash
docker compose exec -T decision_making bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo --once /decision/state'
```

Expected: `/decision/state` returns to `state=GAME` with `gameState=WAIT_INPUT`.
```bash
docker compose exec -T game_controller bash -lc \
    'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    ros2 topic pub --once /ui/input std_msgs/msg/String "{data: '\''{\"action\":\"resume\"}'\''}"'
```
Verify:
```bash
docker compose exec -T decision_making bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo --once /decision/state'
```

Expected: advances to the next phase (example: `P1 -> P2`) and returns to `WAIT_INPUT`.
```bash
docker compose exec -T game_controller bash -lc \
    'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    ros2 topic pub --once /ui/input std_msgs/msg/String "{data: '\''{\"action\":\"skip_phase\"}'\''}"'
```
Verify:
```bash
docker compose exec -T decision_making bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo --once /decision/state'
```

Expected: restarts gameplay from phase `P1` and returns to `state=GAME`, `gameState=WAIT_INPUT`.
```bash
docker compose exec -T game_controller bash -lc \
    'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    ros2 topic pub --once /ui/input std_msgs/msg/String "{data: '\''{\"action\":\"reset\"}'\''}"'
```
Verify:
```bash
docker compose exec -T decision_making bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo --once /decision/state'
```

Expected: `/decision/state` changes to `state=IDLE` (session cleared) and UI returns to game selection.
```bash
docker compose exec -T game_controller bash -lc \
    'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    ros2 topic pub --once /ui/input std_msgs/msg/String "{data: '\''{\"action\":\"stop\"}'\''}"'
```
Verify:
```bash
docker compose exec -T decision_making bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo --once /decision/state'
```

## 6) Simulate Robot Speech and Check Audio Path

### 6.1 Trigger speech via production action contract

```bash
docker compose exec -T game_controller bash -lc 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 action send_goal /expressive_say audio_tts_msgs/action/Communication "{text: \"Hola, esta es una prueba de voz\", language: \"es\"}"'
```

### 6.2 Confirm `/say` was received by TTS mock

```bash
curl -s http://localhost:8096/api/history
```

### 6.3 Browser audio

```bash
xdg-open http://localhost:8096
```

## 7) One-by-one full flow with modern phases (6 phases, 2 questions each = 12 total user inputs)

### 7.1 Start game (modern phase list)

```bash
docker compose exec -T game_controller bash -lc \
    'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    ros2 topic pub --once /game/user_selector std_msgs/msg/String "data: \"1\""'
docker compose exec -T game_controller bash -lc \
    'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
    ros2 topic pub --once /game/game_selector std_msgs/msg/String "{data: \"{\\\"game\\\":{\\\"slug\\\":\\\"colores\\\"},\\\"difficulty\\\":\\\"basic\\\",\\\"phases\\\":[\\\"P1\\\",\\\"P2\\\",\\\"P3\\\",\\\"P4\\\",\\\"P5\\\",\\\"P6\\\"],\\\"roundsPerPhase\\\":2}\"}"'
```

### 7.2 Keep these topics visible while you play

```bash
docker compose logs -f --no-color game_controller communication_hub decision_making
```

```bash
docker compose exec -T decision_making bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo /decision/state'
```

`game_controller` is the source of truth for the phase (`phase` field in `/decision/state`) and UI only renders that phase; it does not advance logic itself.

### 7.3 Manual one-by-one question flow (recommended: interact via UI, fallback CLI shown)

Use the UI buttons/matching controls for each `WAIT_INPUT` question, then keep going to the next `WAIT_INPUT`.
For phases that are hard to answer from shell (especially `P1`), use UI only.

If you want to drive the same flow from CLI while staying manual, use:

```bash
python3 - <<'PY'
#!/usr/bin/env python3
import json
import shlex
import subprocess
import time


def run_ros(service: str, command: str, timeout_sec: int = 20) -> str:
    full = [
        "docker", "compose", "exec", "-T", service,
        "bash", "-lc",
        "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && " + command,
    ]
    return subprocess.check_output(full, text=True, timeout=timeout_sec).strip()


def read_decision_state(timeout_sec: int = 20) -> dict:
    raw = run_ros("decision_making", f"timeout {timeout_sec} ros2 topic echo /decision/state --once", timeout_sec=timeout_sec + 2)
    text = raw.splitlines()[-1].strip()
    if text.startswith("data:"):
        text = text[len("data:"):].strip()
    if not text:
        raise RuntimeError("Empty /decision/state message")
    if text[0] in {'\"', "'"} and text[-1] == text[0]:
        text = text[1:-1]
    data = json.loads(text)
    if isinstance(data, str):
        data = json.loads(data)
    return data if isinstance(data, dict) else {}


def send_touch_input(value: str) -> None:
    payload = json.dumps({"label": value}, ensure_ascii=False)
    run_ros(
        "game_controller",
        f"ros2 topic pub --once /ui/input std_msgs/msg/String {shlex.quote(f\"{{data: '{payload}'}}\")}",
    )


def send_speech(value: str) -> None:
    intent_data = json.dumps({"input": value}, ensure_ascii=False)
    payload = f'{{intent: \"__raw_user_input__\", data: {shlex.quote(intent_data)}, modality: \"__modality_speech__\"}}'
    run_ros("communication_hub", f"ros2 topic pub --once /intents hri_actions_msgs/msg/Intent {shlex.quote(payload)}")


while True:
    state = read_decision_state()
    gs = str(state.get("gameState") or "")
    phase = str((state.get("payload") or {}).get("phase") or "")

    if state.get("state") == "IDLE":
        print("Game finished (IDLE).")
        break

    if gs != "WAIT_INPUT":
        print(f"state={state.get('state')} gameState={gs} phase={phase} waiting for WAIT_INPUT...")
        time.sleep(0.8)
        continue

    payload = state.get("payload") or {}
    question = payload.get("question") or {}
    if phase == "P1":
        print("[P1] Matching phase is best answered in UI. Finish it there, then press Enter to continue.")
        input("Press Enter after you answered in UI...")
        continue

    question_id = question.get("questionId", "?")
    prompt = question.get("prompt") or question.get("promptText") or question.get("text") or "(no prompt)"
    print(f"WAIT_INPUT phase={phase} qid={question_id} prompt={prompt}")

    options = question.get("options") or []
    if isinstance(options, list) and options:
        print("Options:")
        for idx, item in enumerate(options, 1):
            if isinstance(item, dict):
                print(f" {idx}) {item.get('label') or item.get('id')}")
    answer = input("Type one answer (or q to quit): ").strip()
    if answer.lower() in {"q", "quit"}:
        print("Stopped manually.")
        break
    if not answer:
        continue
    if phase == "P2":
        send_speech(answer)
    else:
        send_touch_input(answer)
    print(f"Sent one input for phase={phase}: {answer}")
    time.sleep(0.5)
PY
```

### 7.4 What to expect while you step through

Decision/game state topics:
- `/decision/state`: `state=GAME` with `gameState` cycling `PHASE_INTRO -> QUESTION_PRESENT -> WAIT_INPUT -> CORRECT/FAIL_L1 -> WAIT_INPUT`, and `PHASE_COMPLETE` between phases.
- In `QUESTION_PRESENT`, payload carries `payload.question` with `questionId`, `prompt`, and `options`.
- On each user action, `/decision/events` receives `USER_INTENT` and decision then transitions based on correctness (`CORRECT`, `FAIL_L1`, then back to `WAIT_INPUT`).

`game_controller` logs:
- `[GC] State update: system=..., game=..., tx=..., session=..., phase=...`
- `[GC] /ui/input received` or `[GC] Input from /intents`
- `[GC] /ui/input received` + `Translated event: { ... }`
- `[GC] Published /decision/events: type=USER_INTENT, tx=..., value=..., correct=...`
- `Sending manifest patch (...)` after any UI change

UI (`game_screen`) expectations:
- `QUESTION_PRESENT`: question text shown, options/items are shown, input is disabled.
- `WAIT_INPUT`: `P2` is speech-only in UI (`answerType=none`); `P5`/`P6` also render UI controls as disabled during speech-oriented flow.
- `FAIL_L1`: hint text, input enabled.
- `CORRECT`: feedback text and confetti effect, input disabled.
- `PHASE_COMPLETE`: phase-complete message, input disabled.


## 8) Stop Stack

```bash
docker compose down --remove-orphans
```
