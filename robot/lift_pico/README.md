# Pico lift firmware

`main.py` is the MicroPython program on the lift's Raspberry Pi Pico (`/dev/ttyACM0`,
`usb-MicroPython_Board_in_FS_mode_e66440a2cb2f4a2b`), copied off the board on 2026-10-26.
It's the only copy in version control. Nothing here flashes it.

What `robot/base_motor.py`'s `PicoLift` relies on:

- Height is an incremental count (encoder A rising edges, direction taken from the
  commanded direction, 0.20018 mm/count) that starts at 0 wherever the carriage is at power-on.
- No limit switches. With `limit on` (the default) the motor stops when encoder ticks stop
  for 100 ms while driving. That stop only arms after the first tick of a move. A stall while
  driving down re-zeroes the height and reports `[MIN]`. `home` is "down until that stall".
- `grace <ms>` is echoed in every status line and resets to 100 at boot. `PicoLift` sets it to
  101 to mark "zeroed at the bottom stop since this boot" (see its docstring).

To read it again (stop robot/yor.py first, lift stationary): interrupt with Ctrl-C, enter the
raw REPL (Ctrl-A), run `print(open('main.py').read())`, then Ctrl-B and Ctrl-D to soft-reset.
A soft reset re-zeroes the height, so home the lift again afterwards.
