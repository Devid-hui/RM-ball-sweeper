import os
import pty
import select
import subprocess
import sys
import threading
import unittest
from pathlib import Path


PROBE = Path(__file__).resolve().parents[1] / "tools/check_openrf1_uart.py"


def run_probe(responses, *args):
    master, slave = pty.openpty()
    commands = []
    stop = threading.Event()

    def controller():
        pending = b""
        while not stop.is_set():
            if not select.select([master], [], [], 0.05)[0]:
                continue
            pending += os.read(master, 256)
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                command = line.decode().strip()
                commands.append(command)
                reply = responses(command)
                if reply is not None:
                    os.write(master, (reply + "\r\n").encode())

    worker = threading.Thread(target=controller)
    worker.start()
    try:
        run = subprocess.run([sys.executable, str(PROBE), "--port", os.ttyname(slave),
                              "--timeout", "0.3", *args], capture_output=True, text=True, timeout=6)
        return run, commands
    finally:
        stop.set()
        worker.join()
        os.close(master)
        os.close(slave)


class UartProbeTest(unittest.TestCase):
    def test_recognizes_motor_firmware_without_moving_it(self):
        for version in ("COLLECT1", "ERR"):
            with self.subTest(version=version):
                replies = {"PING": "PONG", "VER": version, "ENC": "A=0 B=1 C=65535 D=23"}
                run, commands = run_probe(replies.get)
                self.assertEqual(run.returncode, 0, run.stderr)
                self.assertEqual(commands, ["PING", "VER", "ENC"])
                self.assertIn(replies["ENC"], run.stdout)

    def test_preserves_early_echo_firmware_check(self):
        run, commands = run_probe(lambda cmd: "ready\r\nPONG" if cmd == "PING" else "RX:" + cmd)
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertIn("HELLO_123", commands)

    def test_reports_wrapped_jog_delta_and_never_repeats_command(self):
        counts = iter(("A=7 B=9 C=2 D=0", "A=7 B=9 C=65530 D=0"))
        def reply(cmd):
            if cmd == "ENC":
                return next(counts)
            return {"PING": "PONG", "VER": "ERR", "JOG C": "DONE"}.get(cmd)
        run, commands = run_probe(reply, "--jog", "C")
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(commands, ["PING", "VER", "ENC", "JOG C", "ENC"])
        self.assertIn("C=-8", run.stdout)

    def test_bad_feedback_fails_before_motion(self):
        replies = {"PING": "PONG", "VER": "ERR", "ENC": "A=0 B=0 C=0 D=65536"}
        run, commands = run_probe(replies.get, "--jog", "D")
        self.assertNotEqual(run.returncode, 0)
        self.assertEqual(commands, ["PING", "VER", "ENC"])
