"""Reads natural language commands from stdin in a daemon thread."""

import queue
import sys
import threading


class CLIReader:
    """
    Reads lines from stdin in a background daemon thread and enqueues them.

    Usage:
        reader = CLIReader()
        reader.start()
        cmd = reader.get(timeout=5.0)  # blocks until input or timeout
    """

    def __init__(self):
        self._queue: queue.Queue[str] = queue.Queue()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def get(self, timeout: float = None) -> str:
        """Block until a command is available. Returns empty string on timeout."""
        try:
            return self._queue.get(timeout=timeout)
        except queue.Empty:
            return ''

    def _run(self) -> None:
        print('LLM Agent ready. Enter command (e.g. "빨간 컵 집어"):')
        while True:
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                line = line.strip()
                if line:
                    self._queue.put(line)
            except (EOFError, OSError):
                break
