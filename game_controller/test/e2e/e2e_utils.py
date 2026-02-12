"""Shared helpers for E2E tests that run against Docker/Compose stacks.

The helpers are intentionally defensive:
- auto-detect docker compose v2 and docker-compose v1
- resolve container names by compose service labels instead of hardcoded names
- provide skip-friendly preflight checks for low-context CI runs
"""

from __future__ import annotations

import os
import re
import subprocess
from typing import Dict, List, Optional, Sequence


class DockerE2EContext:
    """Utility wrapper for docker/compose probing used by E2E tests."""

    def __init__(
        self,
        compose_file: Optional[str] = None,
        compose_project: Optional[str] = None,
    ):
        self.compose_file = compose_file if compose_file is not None else os.getenv("E2E_COMPOSE_FILE")
        self.compose_project = (
            compose_project if compose_project is not None else os.getenv("E2E_COMPOSE_PROJECT")
        )
        self._compose_command = self._detect_compose_command()
        self._container_cache: Dict[str, Optional[str]] = {}

        self.decision_service = os.getenv("E2E_DECISION_SERVICE", "decision_making")
        self.game_controller_service = os.getenv("E2E_GAME_CONTROLLER_SERVICE", "game_controller")
        self.backend_service = os.getenv("E2E_BACKEND_SERVICE", "backend")

        self._service_container_overrides: Dict[str, Optional[str]] = {
            self.decision_service: os.getenv("E2E_DECISION_CONTAINER"),
            self.game_controller_service: os.getenv("E2E_GAME_CONTROLLER_CONTAINER"),
            self.backend_service: os.getenv("E2E_BACKEND_CONTAINER"),
        }

    @staticmethod
    def _detect_compose_command() -> Optional[List[str]]:
        checks = (
            (["docker", "compose", "version"], ["docker", "compose"]),
            (["docker-compose", "version"], ["docker-compose"]),
        )
        for probe, command in checks:
            try:
                result = subprocess.run(probe, capture_output=True, text=True)
                if result.returncode == 0:
                    return command
            except FileNotFoundError:
                continue
        return None

    def compose_available(self) -> bool:
        return self._compose_command is not None

    @staticmethod
    def docker_available() -> bool:
        try:
            result = subprocess.run(["docker", "info"], capture_output=True, text=True)
            return result.returncode == 0
        except FileNotFoundError:
            return False

    def compose(self, args: Sequence[str], check: bool = False) -> subprocess.CompletedProcess:
        if self._compose_command is None:
            raise RuntimeError("Docker Compose is unavailable")
        cmd = [*self._compose_command]
        if self.compose_project:
            cmd.extend(["-p", self.compose_project])
        if self.compose_file:
            cmd.extend(["-f", self.compose_file])
        cmd.extend(args)
        return subprocess.run(cmd, check=check, capture_output=True, text=True)

    @staticmethod
    def _container_name_from_id(container_id: str) -> Optional[str]:
        try:
            result = subprocess.run(
                ["docker", "inspect", "--format", "{{.Name}}", container_id],
                capture_output=True,
                text=True,
            )
        except FileNotFoundError:
            return None
        if result.returncode != 0:
            return None
        name = result.stdout.strip()
        if name.startswith("/"):
            name = name[1:]
        return name or None

    @staticmethod
    def _container_running(container_name: str) -> bool:
        try:
            result = subprocess.run(
                ["docker", "inspect", "--format", "{{.State.Running}}", container_name],
                capture_output=True,
                text=True,
            )
        except FileNotFoundError:
            return False
        return result.returncode == 0 and result.stdout.strip().lower() == "true"

    def _compose_ps_container(self, service: str) -> Optional[str]:
        if self._compose_command is None:
            return None
        result = self.compose(["ps", "-q", service], check=False)
        if result.returncode != 0:
            return None
        container_id = result.stdout.strip().splitlines()[0] if result.stdout.strip() else ""
        if not container_id:
            return None
        return self._container_name_from_id(container_id)

    def _label_lookup_container(self, service: str) -> Optional[str]:
        cmd = [
            "docker",
            "ps",
            "--format",
            "{{.Names}}",
            "--filter",
            f"label=com.docker.compose.service={service}",
        ]
        if self.compose_project:
            cmd.extend(["--filter", f"label=com.docker.compose.project={self.compose_project}"])
        try:
            result = subprocess.run(cmd, capture_output=True, text=True)
        except FileNotFoundError:
            return None
        if result.returncode != 0:
            return None
        names = [line.strip() for line in result.stdout.splitlines() if line.strip()]
        return names[0] if names else None

    @staticmethod
    def _name_heuristic_container(service: str) -> Optional[str]:
        try:
            result = subprocess.run(
                ["docker", "ps", "--format", "{{.Names}}"],
                capture_output=True,
                text=True,
            )
        except FileNotFoundError:
            return None
        if result.returncode != 0:
            return None
        names = [line.strip() for line in result.stdout.splitlines() if line.strip()]
        if service in names:
            return service

        escaped = re.escape(service)
        patterns = (
            re.compile(rf".*[-_]{escaped}[-_]1$"),
            re.compile(rf".*[-_]{escaped}$"),
        )
        for pattern in patterns:
            for name in names:
                if pattern.match(name):
                    return name
        return None

    def resolve_service_container(self, service: str) -> Optional[str]:
        if service in self._container_cache:
            return self._container_cache[service]

        override = self._service_container_overrides.get(service)
        if override:
            self._container_cache[service] = override
            return override

        container = self._compose_ps_container(service)
        if not container:
            container = self._label_lookup_container(service)
        if not container:
            container = self._name_heuristic_container(service)

        self._container_cache[service] = container
        return container

    def preflight_reason(
        self,
        required_services: Sequence[str],
        require_compose: bool = False,
    ) -> Optional[str]:
        if not self.docker_available():
            return "Docker daemon is unavailable; skipping docker-dependent E2E tests."

        if require_compose and not self.compose_available():
            return "Docker Compose is unavailable; skipping compose-managed E2E tests."

        missing: List[str] = []
        for service in required_services:
            container = self.resolve_service_container(service)
            if not container:
                missing.append(f"{service} (container not found)")
                continue
            if not self._container_running(container):
                missing.append(f"{service} (container '{container}' is not running)")

        if missing:
            return (
                "E2E stack context not ready: "
                + ", ".join(missing)
                + ". Start the stack with docker compose and retry."
            )
        return None
