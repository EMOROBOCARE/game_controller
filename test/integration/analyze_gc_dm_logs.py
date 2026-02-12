#!/usr/bin/env python3
"""Analyze headless GC↔DM integration artifacts and write compact summaries.

This is a stdlib-only script intended to run locally after executing the Docker
integration compose (`docker-compose.gc_dm_integration.yml`).

Inputs (by default):
  - game_controller/test/results/manifest_log.jsonl
  - game_controller/test/results/decision_state_log.jsonl
  - game_controller/test/results/decision_event_log.jsonl

Outputs (overwritten):
  - game_controller/test/results/manifest_analysis.md
  - game_controller/test/results/manifest_analysis.json
"""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple


def _safe_json_loads(raw: str) -> Optional[Any]:
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return None


def _read_jsonl(path: str) -> List[Dict[str, Any]]:
    if not os.path.exists(path):
        return []
    rows: List[Dict[str, Any]] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = _safe_json_loads(line)
            if isinstance(obj, dict):
                rows.append(obj)
    return rows


def _write_text(path: str, content: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def _write_json(path: str, payload: Any) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")


def _as_dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _as_list(value: Any) -> List[Any]:
    return value if isinstance(value, list) else []


def _truncate(text: str, max_len: int = 80) -> str:
    t = (text or "").strip()
    if len(t) <= max_len:
        return t
    return t[: max_len - 1].rstrip() + "…"


def _find_instance(manifest: Dict[str, Any], instance_id: str) -> Optional[Dict[str, Any]]:
    instances = manifest.get("instances")
    if not isinstance(instances, list):
        return None
    for inst in instances:
        if isinstance(inst, dict) and inst.get("id") == instance_id:
            return inst
    return None


@dataclass(frozen=True)
class StepSummary:
    ts: float
    step: str
    system_state: str
    game_state: str
    manifest_hash: str
    component: str
    mode: str
    phase: str
    answer_type: str
    question_text: str
    items_count: int
    pause: Optional[bool]


def _extract_step_summary(rec: Dict[str, Any]) -> Tuple[Optional[StepSummary], List[Dict[str, str]]]:
    anomalies: List[Dict[str, str]] = []

    ts = float(rec.get("ts") or 0.0)
    step = str(rec.get("step") or "")
    decision = _as_dict(rec.get("decision"))
    manifest_hash = str(rec.get("manifest_hash") or "")
    manifest = _as_dict(rec.get("manifest"))

    system_state = str(decision.get("state") or "")
    game_state = str(decision.get("gameState") or "")

    # Basic manifest schema checks
    required_schema = {
        "version": int,
        "componentRegistry": dict,
        "ops": dict,
        "layout": dict,
        "instances": list,
        "ui": dict,
    }
    for key, typ in required_schema.items():
        if not isinstance(manifest.get(key), typ):
            anomalies.append({"where": step or "<unknown>", "reason": f"manifest missing/invalid '{key}'"})

    game_screen = _find_instance(manifest, "game_screen")
    user_panel = _find_instance(manifest, "user_panel")
    if game_screen is None or user_panel is None:
        anomalies.append(
            {"where": step or "<unknown>", "reason": "manifest missing required instances (user_panel/game_screen)"}
        )
        return None, anomalies

    component = str(game_screen.get("component") or "")
    cfg = _as_dict(game_screen.get("config"))
    if component not in {"GameSelector", "GameComponent"}:
        anomalies.append(
            {"where": step or "<unknown>", "reason": f"game_screen.component must be GameSelector/GameComponent (got {component})"}
        )

    mode = "menu" if component == "GameSelector" else "game" if component == "GameComponent" else ""
    phase = str(cfg.get("phase") or "")
    question = _as_dict(cfg.get("question"))
    question_text = str(question.get("text") or "")
    answer_type = str(cfg.get("answerType") or "")
    items = [o for o in _as_list(cfg.get("items")) if isinstance(o, dict)]
    pause_raw = cfg.get("pause")
    pause = bool(pause_raw) if pause_raw is not None else None

    if component == "GameSelector":
        if not isinstance(cfg.get("games"), list):
            anomalies.append({"where": step or "<unknown>", "reason": "GameSelector config.games missing/invalid"})
        if not str(cfg.get("startGameOpId") or "").strip():
            anomalies.append({"where": step or "<unknown>", "reason": "GameSelector config.startGameOpId missing"})

    if component == "GameComponent":
        if not str(cfg.get("answerOpId") or "").strip():
            anomalies.append({"where": step or "<unknown>", "reason": "GameComponent config.answerOpId missing"})
        if answer_type not in {"none", "button", "match"}:
            anomalies.append({"where": step or "<unknown>", "reason": f"GameComponent invalid answerType '{answer_type}'"})
        if not isinstance(cfg.get("items"), list):
            anomalies.append({"where": step or "<unknown>", "reason": "GameComponent config.items missing/invalid"})

    if system_state.upper() == "IDLE" and component != "GameSelector":
        anomalies.append({"where": step or "<unknown>", "reason": "IDLE should render GameSelector"})

    if system_state.upper() in {"GAME", "PAUSED"} and component != "GameComponent":
        anomalies.append({"where": step or "<unknown>", "reason": f"{system_state} should render GameComponent"})

    if mode == "game" and game_state.upper() in {"PHASE_INTRO", "QUESTION_PRESENT", "FAIL_L1", "FAIL_L2", "CORRECT", "PHASE_COMPLETE"}:
        if not question_text.strip():
            anomalies.append({"where": step or "<unknown>", "reason": "game mode should include non-empty question.text"})

    if system_state.upper() == "PAUSED" and pause is not True:
        anomalies.append({"where": step or "<unknown>", "reason": "PAUSED state should set pause=true"})
    if system_state.upper() == "GAME" and pause is True:
        anomalies.append({"where": step or "<unknown>", "reason": "GAME state should set pause=false"})

    return (
        StepSummary(
            ts=ts,
            step=step,
            system_state=system_state,
            game_state=game_state,
            manifest_hash=manifest_hash,
            component=component,
            mode=mode,
            phase=phase,
            answer_type=answer_type,
            question_text=_truncate(question_text, 96),
            items_count=len(items),
            pause=pause,
        ),
        anomalies,
    )


def _iter_game_control_events(events: Iterable[Dict[str, Any]]) -> Iterable[Tuple[float, str]]:
    for rec in events:
        ts = float(rec.get("ts") or 0.0)
        ev = _as_dict(rec.get("event"))
        etype = str(ev.get("type") or "").upper()
        payload = _as_dict(ev.get("payload"))
        if etype == "GAME_CONTROL":
            cmd = str(payload.get("command") or "").upper()
            if cmd:
                yield ts, cmd
        elif etype in {"PAUSE", "RESUME", "STOP", "EXIT", "RESET", "RESTART", "SKIP_PHASE", "SKIP"}:
            yield ts, etype


def _find_state_after(state_rows: List[Dict[str, Any]], ts: float) -> Optional[Dict[str, Any]]:
    for rec in state_rows:
        if float(rec.get("ts") or 0.0) >= ts:
            st = _as_dict(rec.get("state"))
            return st if st else None
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--results-dir",
        default=os.path.join("game_controller", "test", "results"),
        help="Directory containing manifest_log.jsonl and friends",
    )
    args = parser.parse_args()

    results_dir = args.results_dir
    manifest_rows = _read_jsonl(os.path.join(results_dir, "manifest_log.jsonl"))
    state_rows = _read_jsonl(os.path.join(results_dir, "decision_state_log.jsonl"))
    event_rows = _read_jsonl(os.path.join(results_dir, "decision_event_log.jsonl"))

    # Ensure ordering by ts.
    manifest_rows.sort(key=lambda r: float(r.get("ts") or 0.0))
    state_rows.sort(key=lambda r: float(r.get("ts") or 0.0))
    event_rows.sort(key=lambda r: float(r.get("ts") or 0.0))

    step_summaries: List[StepSummary] = []
    anomalies: List[Dict[str, str]] = []
    hashes: List[str] = []

    for rec in manifest_rows:
        summary, found = _extract_step_summary(rec)
        anomalies.extend(found)
        if summary is None:
            continue
        step_summaries.append(summary)
        if summary.manifest_hash:
            hashes.append(summary.manifest_hash)

    unique_hashes = sorted(set(hashes))

    control_events: List[Dict[str, Any]] = []
    for ts, cmd in _iter_game_control_events(event_rows):
        st = _find_state_after(state_rows, ts)
        payload = _as_dict((st or {}).get("payload")) if st else {}
        control_events.append(
            {
                "ts": ts,
                "command": cmd,
                "resulting_state": (st or {}).get("state") if st else None,
                "resulting_gameState": (st or {}).get("gameState") if st else None,
                "resulting_phase": payload.get("phase") if payload else None,
            }
        )

    analysis_json: Dict[str, Any] = {
        "total_records_manifest": len(manifest_rows),
        "total_records_state": len(state_rows),
        "total_records_events": len(event_rows),
        "unique_manifest_hashes": len(unique_hashes),
        "steps": [
            {
                "ts": s.ts,
                "step": s.step,
                "systemState": s.system_state,
                "gameState": s.game_state,
                "hash": s.manifest_hash,
                "component": s.component,
                "mode": s.mode,
                "phase": s.phase,
                "answerType": s.answer_type,
                "questionText": s.question_text,
                "itemsCount": s.items_count,
                "pause": s.pause,
            }
            for s in step_summaries
        ],
        "controls_events": control_events,
        "anomalies": anomalies,
    }

    md: List[str] = []
    md.append("# Manifest + event analysis (headless GC↔DM integration)")
    md.append("")
    md.append("## Counts")
    md.append(f"- manifest records: `{len(manifest_rows)}`")
    md.append(f"- decision state records: `{len(state_rows)}`")
    md.append(f"- decision event records: `{len(event_rows)}`")
    md.append(f"- unique manifest hashes: `{len(unique_hashes)}`")
    md.append("")

    md.append("## Timeline (by step)")
    md.append("")
    md.append("| step | system | gameState | component | mode | phase | answerType | items | pause | qText |")
    md.append("|---|---|---|---|---|---|---|---:|---|---|")
    for s in step_summaries:
        md.append(
            "| "
            + " | ".join(
                [
                    s.step,
                    s.system_state,
                    s.game_state,
                    s.component,
                    s.mode,
                    s.phase,
                    s.answer_type,
                    str(s.items_count),
                    str(s.pause),
                    _truncate(s.question_text, 48),
                ]
            )
            + " |"
        )
    md.append("")

    md.append("## Control commands observed")
    md.append("")
    if control_events:
        md.append("| ts | command | resulting_state | resulting_gameState | resulting_phase |")
        md.append("|---:|---|---|---|---|")
        for e in control_events[:30]:
            md.append(
                f"| {float(e.get('ts') or 0.0):.3f} | {e.get('command')} | {e.get('resulting_state')} | "
                f"{e.get('resulting_gameState')} | {e.get('resulting_phase')} |"
            )
        if len(control_events) > 30:
            md.append(f"- (truncated: {len(control_events)} total control events)")
    else:
        md.append("- (none)")
    md.append("")

    md.append("## Anomalies")
    md.append("")
    if anomalies:
        for a in anomalies[:50]:
            md.append(f"- `{a.get('where')}`: {a.get('reason')}")
        if len(anomalies) > 50:
            md.append(f"- (truncated: {len(anomalies)} anomalies)")
    else:
        md.append("- None detected by this script")
    md.append("")

    md_path = os.path.join(results_dir, "manifest_analysis.md")
    json_path = os.path.join(results_dir, "manifest_analysis.json")
    _write_text(md_path, "\n".join(md) + "\n")
    _write_json(json_path, analysis_json)

    print(md_path)
    print(json_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
