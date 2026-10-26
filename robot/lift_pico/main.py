# main.py — Pico Lift CLI with Encoder + Height + Safe Limit Toggle (MicroPython)
# Now using BTS7960 H-bridge instead of mechanical relays.
#
# Motor driver pins (BTS7960, logic side):
#   RPWM = GP2
#   LPWM = GP3
#   R_EN = GP4
#   L_EN = GP5
#
# Encoder: A=GP27, B=GP28 (pull-ups).

import machine, utime, sys, json
try:
    import uselect as select
except ImportError:
    import select

# ===== Config =====
# Old relay pins (no longer used for motor power – keep them unconnected or for debugging only)
UP_PIN_NUM = 0         # (was relay UP)
DN_PIN_NUM = 1         # (was relay DOWN)
ACTIVE_HIGH = True     # kept for compatibility with gpio/debug commands
DEADTIME_MS = 10       # break-before-make when reversing

# BTS7960 motor driver pins (logic side)
RPWM_PIN_NUM = 2       # GP2 -> BTS7960 RPWM
LPWM_PIN_NUM = 3       # GP3 -> BTS7960 LPWM
LEN_PIN_NUM  = 4       # GP4 -> BTS7960 R_EN
REN_PIN_NUM  = 5       # GP5 -> BTS7960 L_EN

MOTOR_PWM_FREQ = 20000   # 20 kHz
MOTOR_SPEED    = 1.0     # duty (0.0–1.0)

ENC_A_PIN_NUM = 27       # GP27
ENC_B_PIN_NUM = 28       # GP28

MM_PER_COUNTS  = 0.20018
HOME_HEIGHT_MM = 0.0

# Limit/stop when no encoder ticks while moving:
LIMIT_ENABLED     = True
NO_TICK_LIMIT_MS  = 100
GRACE_MS          = 100

# Status codes
STATUS_READY, STATUS_MAX, STATUS_MIN, STATUS_MOVING, STATUS_HOME = 0, 1, 2, 3, 4

DIR_OFF, DIR_UP, DIR_DOWN = 0, 1, 2
last_move = DIR_OFF

# Motion timing & arming for limit logic
move_start_ms = utime.ticks_ms()
armed_by_tick = False

# ===== GPIO: Legacy relay pins (debug only) =====
up_pin = machine.Pin(UP_PIN_NUM, machine.Pin.OUT)
dn_pin = machine.Pin(DN_PIN_NUM, machine.Pin.OUT)

def _apply(pin, on):
    if ACTIVE_HIGH:
        pin.value(1 if on else 0)
    else:
        pin.value(0 if on else 1)

def relay_on(pin):  _apply(pin, True)
def relay_off(pin): _apply(pin, False)

# ===== BTS7960 Motor Driver =====
rpwm = machine.PWM(machine.Pin(RPWM_PIN_NUM))
lpwm = machine.PWM(machine.Pin(LPWM_PIN_NUM))
rpwm.freq(MOTOR_PWM_FREQ)
lpwm.freq(MOTOR_PWM_FREQ)

ren = machine.Pin(REN_PIN_NUM, machine.Pin.OUT)
len_ = machine.Pin(LEN_PIN_NUM, machine.Pin.OUT)

def bts_init():
    ren.value(1)
    len_.value(1)
    rpwm.duty_u16(0)
    lpwm.duty_u16(0)

def bts_drive(direction):
    duty = int(65535 * MOTOR_SPEED)
    if direction == DIR_UP:
        rpwm.duty_u16(duty)
        lpwm.duty_u16(0)
    elif direction == DIR_DOWN:
        lpwm.duty_u16(duty)
        rpwm.duty_u16(0)
    else:
        rpwm.duty_u16(0)
        lpwm.duty_u16(0)

bts_init()

def set_move(direction):
    global last_move, move_start_ms, armed_by_tick

    if (last_move == DIR_UP and direction == DIR_DOWN) or (last_move == DIR_DOWN and direction == DIR_UP):
        bts_drive(DIR_OFF)
        utime.sleep_ms(DEADTIME_MS)

    bts_drive(DIR_OFF)

    if direction in (DIR_UP, DIR_DOWN):
        bts_drive(direction)
        move_start_ms = utime.ticks_ms()
        armed_by_tick = False

    last_move = direction

    dir_name = "OFF"
    if direction == DIR_UP:
        dir_name = "UP"
    elif direction == DIR_DOWN:
        dir_name = "DOWN"
    print("MOVE ->", dir_name)

# ===== Encoder (simple A-channel tick ISR) =====
ENC_A = machine.Pin(ENC_A_PIN_NUM, machine.Pin.IN, machine.Pin.PULL_UP)
ENC_B = machine.Pin(ENC_B_PIN_NUM, machine.Pin.IN, machine.Pin.PULL_UP)

encoder_count = 0
last_tick_ms = utime.ticks_ms()

def enc_isr(pin):
    global encoder_count, last_tick_ms, armed_by_tick

    if last_move == DIR_UP:
        encoder_count += 1
    elif last_move == DIR_DOWN:
        encoder_count -= 1
    else:
        return

    last_tick_ms = utime.ticks_ms()
    armed_by_tick = True

ENC_A.irq(trigger=machine.Pin.IRQ_RISING, handler=enc_isr)

# ===== Height via offset =====
offset_counts = 0.0

def get_height_mm():
    return (encoder_count - offset_counts) * MM_PER_COUNTS

def set_height_mm(mm):
    global offset_counts
    offset_counts = encoder_count - (mm / MM_PER_COUNTS)

# ===== Serial non-blocking line input =====
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)
_buf = ""

def readline_nb():
    global _buf
    if not poll.poll(0):
        return None
    ch = sys.stdin.read(1)
    if not ch:
        return None
    if ch == "\r":
        return None
    if ch == "\n":
        s = _buf.strip()
        _buf = ""
        return s if s else None
    _buf += ch
    return None

# ===== Reporting =====
def report_json(status_val):
    print(json.dumps({
        "status": int(status_val),
        "Height": float("{:.2f}".format(get_height_mm())),
        "Count": int(encoder_count),
        "Limit": {"enabled": LIMIT_ENABLED, "grace_ms": GRACE_MS}
    }))

def report_line(status_val):
    names = {0:"READY",1:"MAX",2:"MIN",3:"MOVING",4:"HOMING"}
    print("[{}] Height: {:.2f} mm | Count: {} | Limit:{} (grace {} ms{})".format(
        names.get(int(status_val), "?"), get_height_mm(), encoder_count,
        "ON" if LIMIT_ENABLED else "OFF", GRACE_MS,
        ", armed" if armed_by_tick else ""))

# ===== CLI =====
def help_text():
    print("Commands:")
    print("  up / down / stop")
    print("  status | count")
    print("  set <mm> | zero")
    print("  home | homeheight <mm>")
    print("  json on|off | rate <hz>")
    print("  limit on|off")
    print("  grace <ms>")
    print("  gpio up 0|1 | gpio down 0|1")
    print("  active high|low")
    print("  test")
    print("  help")

def prompt():
    sys.stdout.write("> ")
    try:
        sys.stdout.flush()
    except:
        pass

status = STATUS_READY
homing = False
REPORT_HZ = 100
REPORT_DT_MS = int(1000 / REPORT_HZ)
last_report_ms = utime.ticks_ms()
json_stream = False

def cmd_gpio(which, val):
    v = 1 if int(val) != 0 else 0
    if which == "up":
        set_move(DIR_UP if v == 1 else DIR_OFF)
    elif which == "down":
        set_move(DIR_DOWN if v == 1 else DIR_OFF)
    print("GPIO cmd -> which={}, val={}, last_move={}".format(which, v, last_move))

def handle_cmd(cmd):
    global homing, status, REPORT_DT_MS, REPORT_HZ, HOME_HEIGHT_MM, ACTIVE_HIGH
    global LIMIT_ENABLED, GRACE_MS, json_stream
    c = cmd.strip().lower()

    if c == "up":
        homing = False
        set_move(DIR_UP)
        status = STATUS_MOVING

    elif c == "down":
        homing = False
        set_move(DIR_DOWN)
        status = STATUS_MOVING

    elif c == "stop":
        set_move(DIR_OFF)
        status = STATUS_READY

    elif c == "status":
        report_line(status)

    elif c == "count":
        print("Count:", encoder_count)

    elif c.startswith("set "):
        try:
            mm = float(c.split(None, 1)[1])
            set_height_mm(mm)
            report_line(status)
        except:
            print("ERR: usage: set <mm>")

    elif c == "zero":
        set_height_mm(0.0)
        report_line(status)

    elif c == "home":
        homing = True
        set_move(DIR_DOWN)
        status = STATUS_HOME

    elif c.startswith("homeheight "):
        try:
            HOME_HEIGHT_MM = float(c.split(None, 1)[1])
            print("HOME_HEIGHT_MM set to {:.2f} mm".format(HOME_HEIGHT_MM))
        except:
            print("ERR: usage: homeheight <mm>")

    elif c.startswith("json"):
        parts = c.split()
        if len(parts) == 2 and parts[1] in ("on", "off"):
            json_stream = (parts[1] == "on")
            print("JSON stream:", "ON" if json_stream else "OFF")
        else:
            print("ERR: usage: json on|off")

    elif c.startswith("rate "):
        try:
            hz = int(float(c.split(None, 1)[1]))
            hz = 1 if hz < 1 else (50 if hz > 50 else hz)
            REPORT_HZ = hz
            REPORT_DT_MS = int(1000 / REPORT_HZ)
            print("Report rate set to {} Hz".format(REPORT_HZ))
        except:
            print("ERR: usage: rate <hz>")

    elif c.startswith("limit "):
        parts = c.split()
        if len(parts) == 2 and parts[1] in ("on", "off"):
            LIMIT_ENABLED = (parts[1] == "on")
            print("Limit:", "ON" if LIMIT_ENABLED else "OFF")
        else:
            print("ERR: usage: limit on|off")

    elif c.startswith("grace "):
        try:
            GRACE_MS = int(float(c.split(None, 1)[1]))
            if GRACE_MS < 0:
                GRACE_MS = 0
            print("Grace set to {} ms".format(GRACE_MS))
        except:
            print("ERR: usage: grace <ms>")

    elif c.startswith("gpio "):
        parts = c.split()
        if len(parts) == 3 and parts[1] in ("up", "down"):
            try:
                cmd_gpio(parts[1], int(parts[2]))
            except:
                print("ERR: gpio up|down 0|1")
        else:
            print("ERR: gpio up|down 0|1")

    elif c.startswith("active "):
        parts = c.split()
        if len(parts) == 2 and parts[1] in ("high", "low"):
            relay_off(up_pin)
            relay_off(dn_pin)
            ACTIVE_HIGH = (parts[1] == "high")
            print("Polarity set to ACTIVE_{}".format("HIGH" if ACTIVE_HIGH else "LOW"))
        else:
            print("ERR: active high|low")

    elif c == "test":
        print("Test pulse up/down via set_move()...")
        set_move(DIR_UP);   utime.sleep_ms(120); set_move(DIR_OFF); utime.sleep_ms(150)
        set_move(DIR_DOWN); utime.sleep_ms(120); set_move(DIR_OFF)
        print("Test done.")

    elif c in ("help", "h", "?"):
        help_text()

    else:
        print("Unknown cmd. Type 'help'")

print("Lift CLI ready. BTS7960 motor driver: RPWM=GP{}, LPWM=GP{}, REN=GP{}, LEN=GP{}."
      .format(RPWM_PIN_NUM, LPWM_PIN_NUM, REN_PIN_NUM, LEN_PIN_NUM))
print("Encoder: A=GP{}, B=GP{}.".format(ENC_A_PIN_NUM, ENC_B_PIN_NUM))
print("Limit default = {}. Enable/disable with: limit on|off".format("ON" if LIMIT_ENABLED else "OFF"))
print("Type 'help' for commands.")
prompt()

status = STATUS_READY
last_report_ms = utime.ticks_ms()

while True:
    line = readline_nb()
    if line is not None:
        handle_cmd(line)
        prompt()

    now = utime.ticks_ms()

    if LIMIT_ENABLED and last_move in (DIR_UP, DIR_DOWN):
        since_move_ms = utime.ticks_diff(now, move_start_ms)
        no_ticks_ms   = utime.ticks_diff(now, last_tick_ms)

        if since_move_ms >= GRACE_MS and armed_by_tick and no_ticks_ms >= NO_TICK_LIMIT_MS:
            prev_dir = last_move
            set_move(DIR_OFF)

            if homing:
                irq = machine.disable_irq()
                encoder_count = 0
                offset_counts = 0.0
                armed_by_tick = False
                last_tick_ms = now
                machine.enable_irq(irq)

                set_height_mm(HOME_HEIGHT_MM)
                homing = False
                status = STATUS_MIN
                print("HOMED -> encoder_count=0, height set to {:.2f} mm".format(HOME_HEIGHT_MM))

            else:
                if prev_dir == DIR_DOWN:
                    set_height_mm(0.0)
                    status = STATUS_MIN
                else:
                    status = STATUS_MAX

    if last_move == DIR_OFF and not homing and status not in (STATUS_MIN, STATUS_MAX):
        status = STATUS_READY

    if utime.ticks_diff(now, last_report_ms) >= REPORT_DT_MS:
        last_report_ms = now
        if json_stream:
            report_json(status)
        else:
            if status in (STATUS_MOVING, STATUS_HOME):
                report_line(status)

    utime.sleep_ms(1)
