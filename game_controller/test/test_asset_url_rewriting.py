"""Tests for asset URL rewriting and normalization rules."""

from __future__ import annotations

from game_controller.ui.manifest_builder import format_options_for_ui, rewrite_asset_url


def test_rewrite_defaults_cover_project_shared_and_symbolic_ids():
    assert rewrite_asset_url("assets/images/boat.png") == "/assets/images/boat.png"
    assert rewrite_asset_url("shared/icons/play.svg") == "/emorobcare-components/icons/play.svg"
    assert rewrite_asset_url("images/user.png") == "/emorobcare-components/images/user.png"
    assert rewrite_asset_url("red_circle") == "/assets/images/red_circle.png"
    assert rewrite_asset_url("http://example.com/image.png") == "http://example.com/image.png"
    assert rewrite_asset_url("/absolute/path.png") == "/absolute/path.png"
    assert rewrite_asset_url("") == ""


def test_rewrite_prefers_explicit_project_and_shared_bases(monkeypatch):
    monkeypatch.setenv("PROJECT_ASSET_BASE_URL", "https://project.cdn/assets")
    monkeypatch.setenv("SHARED_ASSET_BASE_URL", "https://shared.cdn/ui")
    monkeypatch.setenv("PROJECT_ASSET_IMAGE_DIR", "cards")
    monkeypatch.setenv("PROJECT_ASSET_ID_EXTENSION", "webp")

    assert rewrite_asset_url("assets/one.png") == "https://project.cdn/assets/one.png"
    assert rewrite_asset_url("images/user.png") == "https://shared.cdn/ui/images/user.png"
    assert rewrite_asset_url("token_id") == "https://project.cdn/assets/cards/token_id.webp"


def test_rewrite_supports_legacy_asset_cdn_alias(monkeypatch):
    monkeypatch.delenv("PROJECT_ASSET_BASE_URL", raising=False)
    monkeypatch.setenv("ASSET_CDN_URL", "https://legacy.cdn/base")

    assert rewrite_asset_url("assets/sounds/beep.mp3") == "https://legacy.cdn/base/sounds/beep.mp3"
    assert rewrite_asset_url("symbolic_image") == "https://legacy.cdn/base/images/symbolic_image.png"


def test_rewrite_supports_compose_project_asset_alias(monkeypatch):
    monkeypatch.setenv("PROJECT_ASSET_BASE_URL", "")
    monkeypatch.setenv("EMOROBCARE_PROJECT_ASSET_BASE_URL", "https://cdn.example.com/components/images")
    monkeypatch.delenv("ASSET_CDN_URL", raising=False)

    assert rewrite_asset_url("assets/colores.png") == "https://cdn.example.com/components/images/colores.png"
    assert rewrite_asset_url("blue_circle") == "https://cdn.example.com/components/images/blue_circle.png"


def test_format_options_for_ui_uses_rewrite_pipeline():
    options = [
        {"id": "1", "label": "Option 1", "imageUrl": "assets/images/opt1.png", "correct": True},
        {"id": "2", "label": "Option 2", "img": "images/shared.png", "correct": False},
    ]

    formatted = format_options_for_ui(options, include_correct=True)

    assert formatted[0]["img"] == "/assets/images/opt1.png"
    assert formatted[1]["img"] == "/emorobcare-components/images/shared.png"
    assert formatted[0]["correct"] is True
    assert formatted[1]["correct"] is False
