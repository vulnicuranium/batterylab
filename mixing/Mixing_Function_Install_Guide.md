# BatteryLab: Adding the Solvent-Mixing Function to the Electrolyte Arm

*Installation instructions with sanity checkpoints and expected outputs*

**Companion files:** `1_mg400_new_primitives.py`, `2_operations_add_to_vial.py`, `3_mixing.py`, `4_menu_wiring_patches.md`, `5_example_mix_recipes.json`

---

## 0. First, why GitHub changes don't show up on the station computer

GitHub does not push code to computers. Each machine (the station laptop, each Raspberry Pi) has its own local clone of the repository, and that clone only changes when someone runs `git pull` on that machine. When Jared pushed his changes last week, that updated the copy on GitHub only. The station computer will keep running its old local copy forever until you pull.

To bring the station computer (or a Pi) up to date:

```bash
cd ~/Research/BatteryLab
git status                    # see what branch you are on and any local edits
git stash                     # ONLY if git status shows local edits you want to keep temporarily
git pull origin main          # download and apply the latest commits from GitHub
git stash pop                 # ONLY if you stashed above
```

Two extra gotchas specific to this system:

- Pulling updates the source files, but the ROS 2 workspace runs from the `install/` directory (copy-install, not symlink-install). After any pull that touches `ros2_ws/src`, you must rebuild: `colcon build --packages-select assembly_robot` (and re-source `install/setup.bash`). Otherwise the robot keeps running the pre-pull code even though the files in `src/` look new.
- Changes under the `BatteryLab/` python package (`robots/`, `electrolyte_planner/`, `solvency/`) take effect in the lab venv only if the package was installed editable (`pip install -e`). If it was installed normally, re-run `pip install ~/Research/BatteryLab` after pulling. Step 6 below makes it editable so this stops being an issue.

So the likely explanation for what you saw: nobody ran `git pull` on the station computer after Jared pushed, and even if they had, a colcon rebuild may also have been needed.

---

## 1. Prerequisites and setup

All work happens on rasp5-hobbs (user `yuanjian`) unless noted. Make sure MecaPortal is in monitoring mode before launching the full app later; the standalone mixing app only talks to the MG400 and the Sartorius pipette.

```bash
cd ~/Research/BatteryLab
git pull origin main                      # start from the latest code (see Section 0)
git checkout -b feature/solvent-mixing    # keep your changes on a branch
source ~/Research/BatteryLab/lab_venv/bin/activate
source ~/.ros_env_setup.sh
```

> **CHECKPOINT 1 — environment sanity**
>
> Run:
>
> ```bash
> ros2 pkg list | grep battery
> python3 -c "import BatteryLab; print(BatteryLab.__file__)"
> ```
>
> **Expected output:** the first command prints `battery_lab_bringup`, `battery_lab_custom_msg` (and related packages). The second prints a path ending in `BatteryLab/__init__.py` under `~/Research/BatteryLab` (or the venv site-packages). If the second command fails with `ModuleNotFoundError`, run `pip install -e ~/Research/BatteryLab` now.

---

## 2. Patch 1: new MG400 motion primitives

**File to edit:** `BatteryLab/robots/MG400.py`. Open `1_mg400_new_primitives.py` from the patch bundle and copy everything from the line `DISPENSE_LEVEL = 0.5` to the end of the file. Paste it inside the MG400 class, directly after the existing `add_liquid_to_post` method (around line 381). The pasted block is already indented at class level; do not re-indent.

**What this adds:** `dispense_liquid_to_vial` (hover above a destination vial, dispense, blow out), `pipette_mix_at_vial` (aspirate/dispense cycles to homogenize, since the platform has no stirrer), and a small pose-interpolation helper. Two tunable constants sit at the top of the block: `DISPENSE_LEVEL` (how far the tip descends into the destination vial while dispensing; 0 = taught down pose, 1 = up pose) and `MIX_BLOWOUT_LEVEL`.

> **CHECKPOINT 2 — patch 1 applied correctly (no robot needed)**
>
> Run:
>
> ```bash
> python3 -c "from BatteryLab.robots.MG400 import MG400; \
> print(hasattr(MG400,'dispense_liquid_to_vial'), hasattr(MG400,'pipette_mix_at_vial'), MG400.DISPENSE_LEVEL)"
> ```
>
> **Expected output:** `True True 0.5`. A SyntaxError or IndentationError means the block was pasted at the wrong indentation (it must line up with the other methods, 4 spaces). `False` means it was pasted outside the class body (e.g. after the module-level `main_loop` function at the bottom of the file).

---

## 3. Patch 2: add_to_vial inventory operation

**File to edit:** `BatteryLab/electrolyte_planner/operations.py`. Open `2_operations_add_to_vial.py` and append the code portion (from the line `from .models import VIAL_MAX_VOLUME_UL` down to, but not including, the final comment block) to the end of `operations.py`.

Then make two edits in `BatteryLab/electrolyte_planner/__init__.py`:

- Change `from .operations import clear_vial, set_vial_contents` to `from .operations import add_to_vial, clear_vial, set_vial_contents`
- Add `"add_to_vial",` to the `__all__` list (next to `"clear_vial"` is fine).

**What this adds:** the planner package can currently overwrite a vial or subtract from it, but nothing can add liquid while tracking the resulting mixture. `add_to_vial` does volume-weighted averaging of solvent fractions and salt molarities, keyed by component name (so it never needs PubChem/network), enforces the 1500 µL capacity, and preserves the previous contents in `previous_electrolyte`.

> **CHECKPOINT 3 — composition math is right**
>
> Run this one-liner (it mixes 700 µL water + 300 µL DMSO in a fake in-memory vial):
>
> ```bash
> python3 - <<'EOF'
> from BatteryLab.electrolyte_planner import ElectrolyteSpec, Inventory, add_to_vial
> inv = Inventory()
> w = ElectrolyteSpec(name='water', v={'water':1.0}, use_pubchem=False)
> d = ElectrolyteSpec(name='dimethyl sulfoxide', v={'dimethyl sulfoxide':1.0}, use_pubchem=False)
> inv = add_to_vial(inv, 2, 0, w, 700.0, mixed_name='test_mix')
> inv = add_to_vial(inv, 2, 0, d, 300.0, mixed_name='test_mix')
> v = inv.vials[0]
> print(v.current_electrolyte.name, v.volume_ul, dict(v.current_electrolyte.v))
> EOF
> ```
>
> **Expected output:** `test_mix 1000.0 {'water': 0.7, 'dimethyl sulfoxide': 0.3}` (dict key order may differ). An ImportError of `add_to_vial` means the `__init__.py` edits were missed. A NameError about `Optional`/`ElectrolyteSpec` means the code was appended to the wrong file — it relies on imports already present at the top of `operations.py`.

---

## 4. Patch 3: the mixing module (new file)

Copy `3_mixing.py` to `ros2_ws/src/assembly_robot/assembly_robot/mixing.py` (keep the filename `mixing.py`, drop the `3_` prefix). Nothing to edit inside it.

**What this adds:** recipe planning in three modes (explicit `mix_plan` volumes; solvent fractions matched to pure stock vials, which bypasses the LP solver entirely; or the existing LP planner as fallback), transfers chunked to the 200 µL pipette limit, per-substance tip management, per-chunk inventory bookkeeping, text and movement simulation, an interactive menu, and a standalone entry point.

---

## 5. Patch 4: menu wiring (three small edits)

Follow `4_menu_wiring_patches.md` exactly. In summary:

- `LiquidRobot.py`: add the `[X]` line to the submenu prompt string and the `[X]` elif handler (manual vial-to-vial transfer, hardware only).
- `app.py`: add `from .mixing import mixing_menu` near the other relative imports; add the `[F]ormulate` line to the main-menu prompt and the `elif user_input == "f":` handler.
- `setup.py` (assembly_robot package): add `'mix_app = assembly_robot.mixing:main',` to `console_scripts`.

---

## 6. Reinstall and rebuild

Patches 1–2 live in the BatteryLab python package (installed into the lab venv); patches 3–4 live in the ROS workspace (copy-installed by colcon). Both need refreshing:

```bash
source ~/Research/BatteryLab/lab_venv/bin/activate
pip install -e ~/Research/BatteryLab
source ~/.ros_env_setup.sh
cd ~/Research/BatteryLab/ros2_ws
colcon build --packages-select assembly_robot
source install/setup.bash
```

> **CHECKPOINT 4 — build picked everything up**
>
> Run:
>
> ```bash
> ros2 pkg executables assembly_robot
> ```
>
> **Expected output** includes the new line `assembly_robot mix_app` alongside the existing app / assembly_robot / crimper_robot / liquid_robot entries. If `mix_app` is missing: the `setup.py` edit was skipped, or colcon build failed — scroll up in the build output for a red error. If colcon reports a Python syntax error in `mixing.py`, the file was truncated during copy; re-copy it whole.

---

## 7. Runtime verification (in this order, no shortcuts)

### 7.1 Stock the inventory

Launch the full app (`ros2 run assembly_robot app`; MecaPortal in monitoring mode) or edit through the menu later. Use `[E]lectrolyte vial manager → [A]dd` to register each physical stock vial: one entry per pure solvent, with `v = {solvent name: 1.0}`, no salts, and the real loaded volume in µL. Use consistent lowercase names — they must match the recipe JSON exactly (e.g. always `water`, `dimethyl sulfoxide`, `acetonitrile`).

> **CHECKPOINT 5 — inventory state**
>
> In the vial manager, press `[V]`. **Expected output:** a text table with one row per stock vial, status OK, `current_solution` showing your solvent names, `remaining_uL` showing what you loaded. The file behind it is `~/.batterylab/electrolyte_inventory.json` — you can `cat` it to double-check.

### 7.2 Text simulation (no hardware motion)

Run `ros2 run assembly_robot mix_app` (only the MG400 initializes — the Meca500s and crimper can stay off). Choose `[F]ile` and point it at your edited copy of `5_example_mix_recipes.json`, confirm a destination vial, then choose `[T]`.

> **CHECKPOINT 6 — simulation output**
>
> Expected output for a 700 µL water + 300 µL DMSO recipe into vial (2,0):
>
> ```text
> --- Mixing simulation for recipe: aw_cal_dmso_30 -> vial (2, 0) ---
>  - Acquire tip for 'water'
>    aspirate 175 uL of 'water' from vial (0,0) -> dispense + blowout into vial (2, 0)
>    (x4 chunks of 175)
>  - Return tip to rack (substance change)
>  - Acquire tip for 'dimethyl sulfoxide'
>    aspirate 150 uL ... (x2 chunks)
>  - Return final tip to rack
>  - (optional) pipette-mix vial (2, 0): 5 x 200 uL with a fresh tip
> --- End simulation: 1000.0 uL total into vial (2, 0) ---
> ```
>
> **Things to verify:** chunk sizes are near-equal and never exceed 200; every source vial coordinate matches where that solvent physically sits; the largest-volume solvent is added first (fractions mode); tip count = number of distinct solvents + 1 for the mix step. A `NOT feasible` line names the exact problem (missing stock, deficit, or destination capacity) — fix the inventory or recipe and re-run.

### 7.3 Movement simulation (arm moves, pipette does not)

Same recipe, choose `[M]`. The arm performs every approach and hover but never aspirates, and nothing is written to the inventory or tip files.

> **CHECKPOINT 7 — geometry**
>
> Watch for, in order: (1) tip pickup at the expected rack position; (2) descent into each source vial's taught down pose; (3) at the destination, the tip should stop clearly INSIDE the vial mouth but ABOVE where the liquid surface will be at maximum fill. If it would dip into liquid on a full vial, raise `DISPENSE_LEVEL` in `MG400.py` (e.g. 0.6–0.7) and a rebuild is NOT needed for this file — just restart `mix_app` (the package is pip-installed editable). **Expected terminal output** ends with: `Movement simulation finished with status: ok`. Also confirm afterwards that `[V]iew vial status` shows unchanged volumes — mime mode must not consume anything.

### 7.4 First real transfer: gravimetric water check

Choose `[I]nteractive` in `mix_app` and enter a single transfer: source `water`, 100 µL, into a pre-weighed empty vial. Answer "n" to the pipette-mix question. Choose `[G]` and confirm.

> **CHECKPOINT 8 — accuracy**
>
> **Expected:** terminal reports `Mixing finished with status: ok`; the balance shows the destination vial gained 100 mg ± a few mg (water density ≈ 1.00 g/mL at room temperature). The inventory view should show the source water vial down by exactly 100 µL and the destination at 100 µL with your chosen mixture name. If the delivered mass is short by more than ~5%, suspect the blowout — check that liquid actually leaves the tip at the destination and that `DISPENSE_LEVEL` isn't so high that droplets miss the vial.

### 7.5 First real mix: two solvents + a_w cross-check

Run one full recipe (e.g. 700 µL water + 300 µL DMSO) with pipette-mix enabled. Then:

> **CHECKPOINT 9 — end-to-end**
>
> **Expected:** status ok; the inventory view prints automatically after a successful mix and the destination row shows the recipe name, 1000.0 µL, and (in the JSON file) `v = {water: 0.70, dimethyl sulfoxide: 0.30}`. Measure water activity on the mixed vial and compare with the same composition from your manual prep sheet — agreement within your instrument's repeatability is the pass criterion. Also check the tip rack (`[T]ip manager, [V]`): expect one tip marked for each solvent plus one marked with the recipe name (the mixing tip).

### 7.6 Only then: the full batch

Load the real 9-recipe batch file (each recipe with its own destination coordinates), run `[T]` once for the whole batch to sanity-check aggregate stock usage, then `[G]` recipe by recipe. Cap each destination vial promptly — THF, DME, and acetonitrile lose volume to evaporation while sitting open.

---

## 8. What happens if something fails mid-run

The executor books each chunk into the inventory only after both the aspiration and the dispense of that chunk succeed, and saves to disk immediately. So if the run aborts (status string like `failed_to_get_liquid`), the inventory file matches physical reality: completed chunks are counted, the failed one is not. Read the status string, fix the cause (a common one after a robot fault: remove any attached tip before re-homing, as the log warns), and re-run a corrected recipe for the remaining volume. Statuses you may see: `failed_preflight`, `failed_to_acquire_tip`, `failed_to_get_tip`, `failed_to_get_liquid`, `failed_to_dispense_to_vial`, `failed_to_return_tip`, `failed_pipette_mix`, `ok_but_not_homogenized`, `ok`.
