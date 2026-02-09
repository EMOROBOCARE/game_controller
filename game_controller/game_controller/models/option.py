"""Option models for game answers."""

from __future__ import annotations

from typing import Any, Optional

from pydantic import BaseModel, Field


class Option(BaseModel):
    """A single answer option."""

    id: Any
    label: str
    image_url: Optional[str] = Field(default=None, alias="imageUrl")
    correct: bool = False
    hidden: bool = False
    highlighted: bool = False
    disabled: bool = False

    model_config = {"populate_by_name": True}

    def to_ui_dict(self) -> dict[str, Any]:
        """Convert to dict for UI manifest."""
        return {
            "id": self.id,
            "label": self.label,
            "img": self.image_url or "",
            "correct": self.correct,
            "hidden": self.hidden,
            "highlighted": self.highlighted,
            "disabled": self.disabled,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Option:
        """Create from dict with flexible field names."""
        return cls(
            id=data.get("id", data.get("value", "")),
            label=data.get("label", ""),
            image_url=data.get("imageUrl", data.get("image_url", data.get("image"))),
            correct=bool(data.get("correct", False)),
            hidden=bool(data.get("hidden", False)),
            highlighted=bool(data.get("highlighted", False)),
            disabled=bool(data.get("disabled", False)),
        )


class OptionSet(BaseModel):
    """A set of answer options."""

    options: list[Option] = Field(default_factory=list)

    def correct_option(self) -> Optional[Option]:
        """Get the correct option, if any."""
        for opt in self.options:
            if opt.correct:
                return opt
        return None

    def correct_options(self) -> list[Option]:
        """Get all correct options."""
        return [opt for opt in self.options if opt.correct]

    def incorrect_options(self) -> list[Option]:
        """Get all incorrect options."""
        return [opt for opt in self.options if not opt.correct]

    def find_by_id(self, option_id: Any) -> Optional[Option]:
        """Find option by ID."""
        for opt in self.options:
            if opt.id == option_id:
                return opt
        return None

    def find_by_label(self, label: str) -> Optional[Option]:
        """Find option by label (case-insensitive)."""
        label_lower = label.lower()
        for opt in self.options:
            if opt.label.lower() == label_lower:
                return opt
        return None

    def to_ui_list(self) -> list[dict[str, Any]]:
        """Convert all options to UI format."""
        return [opt.to_ui_dict() for opt in self.options]
