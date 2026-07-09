# BatteryLab System Startup Guide

Here are the steps to bring up the BatteryLab system before running anything. Starting it is a two-part process: first launch the background services on the three Raspberry Pis, then start the main program.

## The three Raspberry Pis and their roles

| Alias | Location | Role |
|---|---|---|
| `rasp5-hobbs` | Outside the main breadboard | Base machine — you work from here and SSH into the other two |
| `rasp4` | Mounted on the linear rail | Linear rail + assembly-arm camera + suction |
| `rasp5` | Liquid-handling breadboard | Zaber rail control + lookup camera + dispenser |

All three share `ROS_DOMAIN_ID=42` so they can talk to each other — the launch files set this automatically, so you don't need to do anything.

## Step 1 — Start the background services (do everything from rasp5-hobbs)

**On rasp5-hobbs itself** (tower camera + focus/white-balance setup):

```bash
ros2 launch battery_lab_bringup out_rasp.launch.py
```

**SSH into rasp4** (assembly-arm camera + suction pump) — it's passwordless, so it connects directly:

```bash
ros2 launch battery_lab_bringup rail_rasp.launch.py
```

**SSH into rasp5** (Zaber rail at 40 mm/s + lookup camera + Sartorius dispenser):

```bash
ros2 launch battery_lab_bringup board_rasp.launch.py
```

## Step 2 — Start the main program

Back on rasp5-hobbs, open one more terminal and run:

```bash
ros2 run assembly_robot app
```

When the command menu appears, the system is up and ready.
