"""The Gamemaster's own window: which way it is shown."""

from robogame_sim import app_window


def test_app_mode_browser_with_own_profile():
    which = {"chromium": "/usr/bin/chromium"}.get
    command = app_window.browser_command("http://localhost:8766/", which)
    assert command[0] == "/usr/bin/chromium"
    assert "--app=http://localhost:8766/" in command
    assert any(part.startswith("--user-data-dir=") for part in command)


def test_no_browser_no_command():
    assert app_window.browser_command("http://x/", lambda name: None) is None


def test_no_way_to_show_ends_with_2(monkeypatch):
    monkeypatch.setattr(app_window, "show", lambda url: False)
    assert app_window.main(["http://localhost:8766/"]) == 2
