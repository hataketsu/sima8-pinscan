#!/usr/bin/env python3
"""Send commands to the ground unit and capture what it says back.

    console.py <port> <baud> <seconds> [command ...]

Each command is sent as its own line. Output is printed as it is collected, so
a command and the report line that follows it stay together.
"""
import os, sys, termios, select, time

port, baud, secs = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
cmds = sys.argv[4:]

SPEEDS = {9600: termios.B9600, 19200: termios.B19200, 38400: termios.B38400,
          57600: termios.B57600, 115200: termios.B115200}

fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd)
a[0] = a[1] = a[3] = 0
a[2] = termios.CLOCAL | termios.CREAD | termios.CS8
a[4] = a[5] = SPEEDS[baud]
a[6][termios.VMIN] = 0
a[6][termios.VTIME] = 0
termios.tcsetattr(fd, termios.TCSANOW, a)
termios.tcflush(fd, termios.TCIOFLUSH)

for c in cmds:
    os.write(fd, (c + "\r\n").encode())
    time.sleep(0.25)

buf = bytearray()
end = time.monotonic() + secs
while time.monotonic() < end:
    r, _, _ = select.select([fd], [], [], 0.2)
    if r:
        try:
            buf += os.read(fd, 4096)
        except BlockingIOError:
            pass
os.close(fd)
sys.stdout.write(buf.decode("ascii", "replace"))
