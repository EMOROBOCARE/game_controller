# Game Controller Documentation Audit (2026-02-13)

## Scope
Reviewed all Markdown documentation under `games_src/game_controller` for link validity and process/code alignment (including nested package docs such as `chatbot_msgs/`, `emorobcare_led_service/`, and `game_controller/docs/`).

## What Was Fixed
1. `chatbot_msgs/README.md`
   - Repointed implementation references for `llm_bridge` from a non-existent local path to the actual sibling package path `../../../src/llm_bridge`.

2. `CLAUDE.md`
   - Corrected `Core Code Flow` links to package files in the nested `game_controller/` ROS2 package folder.

3. `emorobcare_led_service/examples/README.md`
   - Replaced stale `~/emorobot_ws` workspace path snippets with neutral `/path/to/game_controller_ws` placeholders.
   - Removed broken `../QUICKREF.md` and `../ERROR_REFERENCE.md` references from **See Also**.

## Verification Performed
- Markdown link audit over all `*.md` files in `games_src/game_controller` (relative links only):
  - Result: **no unresolved local targets** after fixes.
- Cross-check for known broken references to non-existent docs/files reported by `rg` and manual review.

## Current Discrepancies / Notes
- `leds_ros/` exists in `games_src/game_controller` but is not consumed by any compose file, Dockerfile, test harness, or runtime imports in the current stack.
  - This is already documented as redundant in `GAME_CONTROLLER_PACKAGE_SOURCE_ALIGNMENT.md`.
- Several process docs use environment-specific placeholders for local paths (for example `/path/to/...`).
  - This is intentional so commands work across developer machines, but must be substituted before execution.
- External integration contracts are authoritative in repo-root documents:
  - `INTEGRATION_CONTRACT.md`
  - `UI_integration.md`
  - `GC_integration.md`
  - These remain in `games_src/` and are referenced by game-controller docs.

