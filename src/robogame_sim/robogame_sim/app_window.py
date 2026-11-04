"""The Gamemaster's own window: the app's page with no tabs or address bar.

    python3 app_window.py http://localhost:8766/#editor

Shows the page in a pywebview window if pywebview is installed
(sudo apt install python3-webview, or pip install pywebview), or else in
Chromium, Chrome, Brave or Edge's app mode (--app) with its own profile,
so it is a window of its own and not a tab in a browser already open.
This process lasts as long as the window, so the Gamemaster knows the
window was closed when it ends. With neither, it says so and ends at
once (exit 2), and the Gamemaster opens a browser tab instead.

Only the standard library is used, so the desktop launcher can run this
file without ROS sourced.
"""

import os
import shutil
import signal
import subprocess
import sys

TITLE = "RoboGame Gamemaster"
SIZE = (1440, 900)
BROWSERS = ("chromium", "chromium-browser", "google-chrome",
            "google-chrome-stable", "brave-browser", "microsoft-edge")


def profile_dir():
    """The app-mode browser's own profile, kept apart from the user's own
    so the window is a process of its own that ends when it closes."""
    cache = os.environ.get("XDG_CACHE_HOME") or os.path.expanduser("~/.cache")
    return os.path.join(cache, "robogame", "window")


def browser_command(url, which=shutil.which):
    """The command for an app-mode window on url, or None."""
    for name in BROWSERS:
        path = which(name)
        if path:
            return [path, f"--app={url}", f"--user-data-dir={profile_dir()}",
                    f"--window-size={SIZE[0]},{SIZE[1]}",
                    "--no-first-run", "--no-default-browser-check"]
    return None


def available(which=shutil.which):
    """True if this computer can show the window: pywebview or an
    app-mode browser."""
    import importlib.util
    return (importlib.util.find_spec("webview") is not None
            or browser_command("x", which) is not None)


def show(url):
    """Show url in a window and return once it is closed. False if there
    is no way to show one."""
    try:
        import webview
    except ImportError:
        webview = None
    if webview is not None:
        try:
            webview.create_window(TITLE, url, width=SIZE[0], height=SIZE[1])
            webview.start()
            return True
        except Exception as error:  # no GUI toolkit for it, say
            print(f"pywebview could not open a window: {error}",
                  file=sys.stderr)
    command = browser_command(url)
    if command is None:
        return False
    browser = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
    # The Gamemaster stops this process when it quits: take the window too.
    signal.signal(signal.SIGTERM, lambda *_: browser.terminate())
    browser.wait()
    return True


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    url = argv[0] if argv else "http://localhost:8766/"
    if not show(url):
        print("No window: install python3-webview or Chromium. "
              "Opening the browser instead.", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
