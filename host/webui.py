#!/usr/bin/env python3
"""Local control panel for the ground unit.

    webui.py [port] [http_port]

Opens the ground unit's USB CDC port, serves a page on localhost, and relays
commands to it. It has to run locally: a hosted page cannot open a serial port.
"""
import json, os, select, sys, termios, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem00011"
HTTP = int(sys.argv[2]) if len(sys.argv) > 2 else 8088

state = {
    "connected": False, "sent": 0, "failed": 0, "addr0": "", "config": "",
    "status": "", "fifo": "", "throttle": 0, "armed": False,
    "roll": 128, "pitch": 128, "yaw": 128, "lines": [], "updated": 0,
}
lock = threading.Lock()
fd = None


def serial_open(path):
    # 115200 is arbitrary for CDC, but never 1200: that is the reboot-to-
    # bootloader convention and would drop the device mid-session.
    f = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(f)
    a[0] = a[1] = a[3] = 0
    a[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    a[4] = a[5] = termios.B115200
    a[6][termios.VMIN] = 0
    a[6][termios.VTIME] = 0
    termios.tcsetattr(f, termios.TCSANOW, a)
    return f


def parse(line):
    with lock:
        state["lines"] = (state["lines"] + [line])[-40:]
        state["updated"] = time.time()
        if line.startswith("tx sent="):
            for tok in line.split():
                if "=" not in tok:
                    continue
                k, v = tok.split("=", 1)
                k = k.replace("tx ", "")
                if k in ("sent", "failed"):
                    try:
                        state[k] = int(v)
                    except ValueError:
                        pass
                elif k in ("ADDR0", "CONFIG", "STATUS", "FIFO"):
                    state[k.lower()] = v
        elif line.startswith("ARMED"):
            state["armed"] = True
        elif line.startswith("DISARMED"):
            state["armed"] = False
            state["throttle"] = 0


def reader():
    global fd
    buf = b""
    while True:
        if fd is None:
            try:
                fd = serial_open(PORT)
                with lock:
                    state["connected"] = True
            except OSError:
                with lock:
                    state["connected"] = False
                time.sleep(1.0)
                continue
        try:
            r, _, _ = select.select([fd], [], [], 0.3)
            if not r:
                continue
            chunk = os.read(fd, 4096)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", "replace").strip()
                if text:
                    parse(text)
        except OSError:
            try:
                os.close(fd)
            except OSError:
                pass
            fd = None
            with lock:
                state["connected"] = False


def send(cmd):
    global fd
    if fd is None:
        return False
    try:
        os.write(fd, (cmd + "\r\n").encode())
        return True
    except OSError:
        return False


PAGE = """<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>sima8 ground</title><style>
:root{--bg:#0f1115;--panel:#171a21;--line:#262b36;--fg:#e6e9ef;--dim:#8b93a7;
--ok:#3ecf8e;--warn:#f0a500;--bad:#ff5c5c;--acc:#5b9dff}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
font:14px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace}
.wrap{max-width:940px;margin:0 auto;padding:20px}
h1{font-size:17px;margin:0 0 4px;letter-spacing:.04em}
.sub{color:var(--dim);margin-bottom:18px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px;margin-bottom:16px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px}
.k{color:var(--dim);font-size:11px;text-transform:uppercase;letter-spacing:.08em}
.v{font-size:22px;margin-top:4px}
.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}
.row{display:flex;gap:10px;align-items:center;margin:10px 0}
label{width:74px;color:var(--dim)}
input[type=range]{flex:1;accent-color:var(--acc)}
.num{width:46px;text-align:right}
button{background:var(--acc);color:#06101f;border:0;border-radius:6px;
padding:10px 16px;font:inherit;font-weight:700;cursor:pointer}
button.danger{background:var(--bad);color:#1a0505}
button.ghost{background:transparent;color:var(--fg);border:1px solid var(--line)}
pre{background:#0b0d11;border:1px solid var(--line);border-radius:8px;padding:12px;
max-height:260px;overflow:auto;margin:0;color:var(--dim);font-size:12px}
</style></head><body><div class=wrap>
<h1>sima8 ground station</h1>
<div class=sub id=port></div>

<div class=grid>
<div class=card><div class=k>link</div><div class="v" id=conn>...</div></div>
<div class=card><div class=k>armed</div><div class="v" id=arm>...</div></div>
<div class=card><div class=k>packets sent</div><div class=v id=sent>0</div></div>
<div class=card><div class=k>failed</div><div class=v id=failed>0</div></div>
</div>

<div class=card>
<div class=row><label>throttle</label><input type=range id=t min=0 max=255 value=0>
<span class=num id=tv>0</span></div>
<div class=row><label>roll</label><input type=range id=r min=0 max=255 value=128>
<span class=num id=rv>128</span></div>
<div class=row><label>pitch</label><input type=range id=p min=0 max=255 value=128>
<span class=num id=pv>128</span></div>
<div class=row><label>yaw</label><input type=range id=y min=0 max=255 value=128>
<span class=num id=yv>128</span></div>
<div class=row style="margin-top:14px">
<button id=armbtn>ARM</button>
<button class=danger id=disarm>DISARM</button>
<button class=ghost id=boot>bootloader</button>
</div>
<div class=sub style="margin:8px 0 0">Arming is refused unless throttle is 0.
DISARM also forces throttle to 0.</div>
</div>

<div class=card style="margin-top:12px"><div class=k>radio registers</div>
<div class=row><span id=regs class=v style="font-size:14px">-</span></div></div>

<pre id=log></pre>
</div><script>
const $=id=>document.getElementById(id);
$('port').textContent='serial: '+location.search.slice(1)||'';
function cmd(c){fetch('/api/cmd',{method:'POST',body:c});}
function bind(id,letter,out){
  const el=$(id);
  el.addEventListener('input',()=>{ $(out).textContent=el.value; });
  el.addEventListener('change',()=>{ cmd(letter+el.value); });
}
bind('t','t','tv'); bind('r','r','rv'); bind('p','p','pv'); bind('y','y','yv');
$('armbtn').onclick=()=>cmd('a');
$('disarm').onclick=()=>{ $('t').value=0; $('tv').textContent='0'; cmd('d'); };
$('boot').onclick=()=>{ if(confirm('Reboot into the bootloader?')) cmd('b'); };
async function tick(){
  try{
    const s=await (await fetch('/api/state')).json();
    $('conn').textContent=s.connected?'connected':'no port';
    $('conn').className='v '+(s.connected?'ok':'bad');
    $('arm').textContent=s.armed?'ARMED':'safe';
    $('arm').className='v '+(s.armed?'warn':'ok');
    $('sent').textContent=s.sent;
    $('failed').textContent=s.failed;
    $('failed').className='v '+(s.failed?'bad':'ok');
    $('regs').textContent='ADDR0='+s.addr0+'  CONFIG='+s.config+
      '  STATUS='+s.status+'  FIFO='+s.fifo;
    $('log').textContent=s.lines.join('\\n');
    $('log').scrollTop=$('log').scrollHeight;
  }catch(e){ $('conn').textContent='server gone'; $('conn').className='v bad'; }
}
setInterval(tick,500); tick();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith("/api/state"):
            with lock:
                body = json.dumps(state).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        cmd = self.rfile.read(n).decode("ascii", "replace").strip()
        ok = send(cmd) if cmd else False
        self.send_response(200 if ok else 503)
        self.send_header("Content-Length", "0")
        self.end_headers()


if __name__ == "__main__":
    threading.Thread(target=reader, daemon=True).start()
    print(f"serial : {PORT}")
    print(f"open   : http://127.0.0.1:{HTTP}/")
    ThreadingHTTPServer(("127.0.0.1", HTTP), H).serve_forever()
