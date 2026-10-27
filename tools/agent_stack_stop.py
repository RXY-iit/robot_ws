"""Quick Start client for the cooperative, session-scoped Agent shutdown API."""

import json
from pathlib import Path
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class AgentStackClient:
    def __init__(self, port=8765):
        self.origin = f"http://127.0.0.1:{int(port)}"
        self.opener = build_opener(ProxyHandler({}), NoRedirect())

    def request(self, path, token=None):
        headers = {"Origin": self.origin, "Cache-Control": "no-store"}
        data = None
        if token:
            headers.update({"Content-Type": "application/json", "X-Agent-Stop-Token": token})
            data = b'{"confirm":true}'
        try:
            with self.opener.open(Request(self.origin + path, data=data, headers=headers), timeout=3) as response:
                result = json.load(response)
        except HTTPError as exc:
            try:
                detail = json.load(exc).get("detail", "Stop API unavailable")
            except (ValueError, AttributeError):
                detail = "Stop API unavailable. Restart Agent stack once with the updated launcher."
            raise RuntimeError(detail) from exc
        if not isinstance(result, dict):
            raise RuntimeError("Invalid Agent stack response; no forced termination.")
        return result

    @staticmethod
    def _stack_exited(domain):
        from agent_stack_guard import acquire_launch_lock, find_existing

        workspace = Path(__file__).resolve().parents[1]
        if find_existing(workspace, domain):
            return False
        try:
            lock = acquire_launch_lock(workspace, domain)
        except RuntimeError:
            return False
        lock.close()
        return True

    def stop(self, mode, domain, progress, wait_sec=75):
        config = self.request("/api/config")
        token = config.get("stack_stop_token")
        if not token or not config.get("stack_mode"):
            raise RuntimeError("Restart Agent stack once with the updated launcher (Ctrl+C in its terminal first).")
        if config["stack_mode"] != mode or str(config.get("stack_domain_id")) != str(domain):
            raise RuntimeError("Mode / ROS_DOMAIN_ID mismatch. No stop request sent.")
        state = self.request("/api/stack/stop", token)
        deadline = time.monotonic() + wait_sec
        offline = False
        while True:
            status = state.get("state", "UNKNOWN")
            progress(f"{status}: {state.get('detail', '')}")
            if status == "BLOCKED":
                return
            offline = offline or status == "INTENTIONALLY_OFFLINE"
            if time.monotonic() >= deadline:
                raise RuntimeError("Shutdown not confirmed. Check the Agent terminal; no forced termination.")
            time.sleep(0.25)
            try:
                # Do not follow a replacement server after the requested session exits.
                if self.request("/api/config").get("stack_stop_token") != token:
                    raise RuntimeError("Agent session changed. Check the terminal; no further stop request sent.")
                state = self.request("/api/stack")
            except (URLError, OSError):
                if offline:
                    if self._stack_exited(domain):
                        progress("Agent shutdown acknowledged; processes exited and launch lock released. External robot processes unchanged.")
                        return
                    progress("Dashboard disconnected; waiting for Agent processes and launch lock to exit.")
                    continue
                raise RuntimeError("Dashboard disconnected before shutdown confirmation. Check the Agent terminal.")


def add_stop_control(window, grid, row, mode, get_domain):
    """Attach a shared GTK control; all HTTP work stays off the UI thread."""
    from gi.repository import GLib, Gtk

    button = Gtk.Button(label="Stop Agent stack")
    label = Gtk.Label(label="", xalign=0)
    if hasattr(label, "set_wrap"):
        label.set_wrap(True)
    else:
        label.set_line_wrap(True)
    label.set_max_width_chars(65)
    grid.attach(button, 0, row, 1, 1)
    grid.attach(label, 1, row, 1, 1)
    alive = [True]
    button.connect("destroy", lambda *_: alive.__setitem__(0, False))

    def update(message):
        if alive[0]:
            label.set_text(message)
        return False

    def work(domain):
        try:
            AgentStackClient().stop(mode, domain, lambda message: GLib.idle_add(update, message))
        except Exception as exc:
            GLib.idle_add(update, str(exc))
        finally:
            GLib.idle_add(lambda: button.set_sensitive(True) if alive[0] else False)

    def clicked(_button):
        domain = str(get_domain())
        dialog = Gtk.MessageDialog(transient_for=window, modal=True,
                                   message_type=Gtk.MessageType.WARNING,
                                   buttons=Gtk.ButtonsType.OK_CANCEL,
                                   text="Stop Agent stack?")
        dialog.format_secondary_text(
            f"Mode: {mode} / ROS_DOMAIN_ID: {domain}\n"
            "Cancel Mission and drain Recovery actions, then close Agent, Dashboard and Health Monitor.\n"
            "Isaac, external ROS nodes and Arm hardware power remain unchanged.")

        def response(_dialog, result):
            dialog.destroy()
            if result == Gtk.ResponseType.OK:
                button.set_sensitive(False)
                label.set_text("Requesting safe shutdown...")
                threading.Thread(target=work, args=(domain,), daemon=True).start()

        dialog.connect("response", response)
        dialog.show()

    button.connect("clicked", clicked)
    return button
