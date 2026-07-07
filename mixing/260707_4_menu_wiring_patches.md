# PATCH 4 of 5 — Menu wiring (three small edits)

## 4a. `ros2_ws/src/assembly_robot/assembly_robot/LiquidRobot.py`
Low-level manual transfer in the liquid robot submenu (hardware only — no
inventory or tip bookkeeping, consistent with the rest of this menu).

**Edit 1 — the prompt string in `liquid_robot_command_loop`.** Insert one line
after `[J] to dispense liquid with volume to the post.`:

```
[X] to transfer liquid with volume from vial (x1,y1) to vial (x2,y2).
```

**Edit 2 — the command handler.** Insert this `elif` block after the
`elif input_str == "J":` block:

```python
            elif input_str == "X":
                print("Source vial:")
                sx, sy = get_liquid_coords()
                if sx is None or sy is None:
                    continue
                print("Destination vial:")
                dx, dy = get_liquid_coords()
                if dx is None or dy is None:
                    continue
                if (sx, sy) == (dx, dy):
                    print("Source and destination vials must differ.")
                    continue
                volume = get_volume()
                if volume is not None:
                    liquidRobot.MG400.get_liquid(sx, sy, volume)
                    liquidRobot.MG400.dispense_liquid_to_vial(dx, dy, volume)
```

Note the existing `get_volume()` already enforces the 1–200 uL pipette limit,
so no chunking is needed here; for larger volumes use the mixing menu.

---

## 4b. `ros2_ws/src/assembly_robot/assembly_robot/app.py`
Two edits.

**Edit 1 — import.** After the line
`from .LiquidRobot import LiquidRobot, liquid_robot_command_loop` add:

```python
from .mixing import mixing_menu
```

**Edit 2 — main menu.** In `command_loop`, add to the prompt string (e.g.
after the `[B]atch recipes...` line):

```
[F]ormulate: mix solvents/electrolytes into a vial
```

and add the handler (e.g. after the `elif user_input == "b":` block):

```python
        elif user_input == "f":
            mixing_menu(batterylab.liquid_robot, batterylab.get_logger())
```

(`command_loop` already lower-cases and truncates input to one character, so
"f" is the trigger.)

---

## 4c. `ros2_ws/src/assembly_robot/setup.py`
Add the standalone mixing app to `console_scripts` (inside the existing list):

```python
            'mix_app = assembly_robot.mixing:main',
```

This gives you `ros2 run assembly_robot mix_app` — it initializes ONLY the
MG400 + Sartorius, so you can mix electrolytes without powering the two
Meca500s and the crimper (unlike the full `app`).
