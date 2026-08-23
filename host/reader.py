import os, sys, termios, select, time

port, baud, secs, out = sys.argv[1], int(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
SPEEDS = {4800: termios.B4800, 9600: termios.B9600, 19200: termios.B19200,
          38400: termios.B38400, 57600: termios.B57600, 115200: termios.B115200}

fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
a = termios.tcgetattr(fd)
# raw 8N1, no flow control, no echo, no line processing
a[0] = 0                                  # iflag
a[1] = 0                                  # oflag
a[2] = termios.CLOCAL | termios.CREAD | termios.CS8
a[3] = 0                                  # lflag
a[4] = a[5] = SPEEDS[baud]                # ispeed / ospeed on the open fd
a[6][termios.VMIN] = 0
a[6][termios.VTIME] = 0
termios.tcsetattr(fd, termios.TCSANOW, a)
termios.tcflush(fd, termios.TCIFLUSH)

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
open(out, 'wb').write(buf)
print(f"baud={baud} bytes={len(buf)} ascii_hit={b'PINSCAN' in buf}")
