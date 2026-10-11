"""Forward shutdown to both Webots' launcher and its ROS controller child."""

import os
import signal
import subprocess
import sys
import time


def main():
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    process = subprocess.Popen(sys.argv[1:], start_new_session=True)
    try:
        while process.poll() is None and not stopping:
            time.sleep(0.05)
    finally:
        # The launcher can return while its ROS child is still alive.
        for signum, grace in ((signal.SIGINT, 1.0), (signal.SIGTERM, 1.0),
                              (signal.SIGKILL, 0.0)):
            try:
                os.killpg(process.pid, signum)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline:
                process.poll()
                try:
                    os.killpg(process.pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.05)
        process.wait()
    return 0 if stopping else process.returncode


if __name__ == '__main__':
    sys.exit(main())
