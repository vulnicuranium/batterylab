# ============================================================================
# PATCH 3 of 5 — NEW FILE
# Save as: ros2_ws/src/assembly_robot/assembly_robot/mixing.py
#
# Adds a "mix solvents into a vial" capability to the BatteryLab app:
#   * three ways to specify a mix (explicit volumes / volume fractions /
#     the existing LP planner),
#   * chunked vial-to-vial transfers within the 200 uL pipette limit,
#   * per-substance tip management via the existing TipRack,
#   * inventory bookkeeping (sources consumed, destination composition
#     tracked via add_to_vial) after every successful chunk,
#   * optional pipette-mix homogenization at the end,
#   * text simulation, movement (mime) simulation, and real execution,
#   * an interactive menu, plus a standalone `mix_app` entry point that
#     only initializes the MG400 (no Meca500s / crimper needed).
#
# Requires PATCH 1 (MG400.dispense_liquid_to_vial / pipette_mix_at_vial)
# and PATCH 2 (electrolyte_planner.add_to_vial).
# ============================================================================
"""Solvent/electrolyte mixing into destination vials using the liquid robot."""

import json
import math
from pathlib import Path
from typing import Optional, Sequence

from BatteryLab.electrolyte_planner import (
    ElectrolyteSpec,
    Inventory,
    TipRack,
    add_to_vial,
    evaluate_formulation,
    load_inventory_state,
    load_tip_rack_state,
    print_vial_statuses,
    save_inventory_state,
    save_tip_rack_state,
)

from .LiquidRobot import LiquidRobot, MAX_PIPETTE_VOLUME

# Volumes below this are at/under the accuracy floor of the 200 uL rLine head;
# the executor warns but still proceeds (tune to taste).
MIN_ACCURATE_TRANSFER_UL = 10
# Defaults for the optional post-transfer pipette-mix homogenization step.
DEFAULT_MIX_CYCLES = 5
DEFAULT_MIX_FRACTION = 0.5  # aspirate up to 50% of the final volume per cycle


# ---------------------------------------------------------------------------
# Tip helpers (mirror the private helpers in app.py; kept local to avoid a
# circular import between app.py and this module — candidates for a shared
# helpers module in a later refactor).
# ---------------------------------------------------------------------------

def _tip_index_to_coordinates(tip_index: int) -> tuple[int, int]:
    if not (0 <= tip_index <= 95):
        raise ValueError(f"Tip index must be 0-95, got {tip_index}")
    return tip_index % 12, tip_index // 12


def _tip_index_for_substance_or_error(tip_rack: TipRack, substance_name: Optional[str]) -> int:
    tip_index = tip_rack.find_clean_tip_for_substance(substance_name)
    if tip_index is not None:
        return tip_index
    if substance_name is None:
        raise RuntimeError("No clean pipette tips are available for this mix.")
    raise RuntimeError(
        f"No clean pipette tips are available for substance '{substance_name}' without contamination."
    )


# ---------------------------------------------------------------------------
# Planning: turn a recipe into [{'source_solution', 'volume_ul'}, ...]
# ---------------------------------------------------------------------------

def _norm(name: str) -> str:
    return str(name).strip().lower()


def _is_pure_stock(spec: ElectrolyteSpec) -> Optional[str]:
    """Return the single solvent name if this spec is a pure solvent stock
    (exactly one solvent, no salts, no additives), else None."""
    if spec is None:
        return None
    if spec.s or spec.a:
        return None
    if len(spec.v) != 1:
        return None
    return next(iter(spec.v.keys()))


def build_mix_plan(recipe: dict, inventory: Inventory) -> list[dict]:
    """Resolve a recipe into per-solution volumes: [{'source_solution', 'volume_ul'}].

    Three modes, tried in this order:

    1. EXPLICIT ("mix_plan" key): the recipe directly lists volumes per stock
       solution name. Most robust; recommended for NeuriCo-generated batches.
         "mix_plan": [{"source_solution": "DMSO", "volume_ul": 300.0}, ...]

    2. FRACTIONS ("target_electrolyte" with solvent fractions only): each
       target solvent is matched to a PURE single-solvent stock in inventory
       (case-insensitive match on the stock's solvent name or solution name),
       and volume = normalized fraction x target volume. This bypasses the LP
       entirely — no more "No optimal solution found" for the common case of
       mixing pure stocks. Refuses targets with salts/additives (use mode 1
       or 3, or stock a salt concentrate).

    3. PLANNER (fallback): the existing evaluate_formulation LP, aggregated
       per solution. Works when the vial bank can satisfy the target exactly.
    """
    # --- Mode 1: explicit volumes ---
    if recipe.get("mix_plan"):
        plan = []
        for i, entry in enumerate(recipe["mix_plan"], start=1):
            name = str(entry.get("source_solution", "")).strip()
            vol = float(entry.get("volume_ul", 0.0) or 0.0)
            if not name:
                raise ValueError(f"mix_plan entry {i} is missing 'source_solution'")
            if vol <= 0:
                raise ValueError(f"mix_plan entry {i} ('{name}') must have volume_ul > 0")
            plan.append({"source_solution": name, "volume_ul": vol})
        return plan

    target = recipe.get("target_electrolyte") or {}
    total_volume = float(target.get("volume") or 0.0)
    if total_volume <= 0:
        raise ValueError(
            "Recipe needs either a 'mix_plan' or a 'target_electrolyte' with a positive 'volume'."
        )

    # --- Mode 2: fractions against pure stocks ---
    fractions = dict(target.get("v") or {})
    has_salts = bool(target.get("s")) or bool(target.get("a"))
    if fractions and not has_salts:
        # Index the pure stocks available in inventory.
        pure_stock_lookup: dict[str, str] = {}  # normalized key -> canonical solution name
        for vial in inventory.vials:
            spec = vial.current_electrolyte
            if spec is None or vial.volume_ul <= 0:
                continue
            solvent = _is_pure_stock(spec)
            if solvent is None:
                continue
            pure_stock_lookup.setdefault(_norm(solvent), spec.name)
            pure_stock_lookup.setdefault(_norm(spec.name), spec.name)

        frac_sum = sum(fractions.values())
        if frac_sum <= 0:
            raise ValueError("Solvent fractions must sum to a positive value.")

        plan, missing = [], []
        for solvent_name, fraction in fractions.items():
            if fraction <= 0:
                continue
            stock_name = pure_stock_lookup.get(_norm(solvent_name))
            if stock_name is None:
                missing.append(solvent_name)
                continue
            plan.append(
                {
                    "source_solution": stock_name,
                    "volume_ul": (fraction / frac_sum) * total_volume,
                }
            )
        if missing:
            raise ValueError(
                "No pure stock vial found for solvent(s): "
                + ", ".join(missing)
                + ". Load them via the Electrolyte vial manager, or provide an explicit 'mix_plan'."
            )
        # Largest additions first: puts the bulk solvent down before minors,
        # which improves dissolution and keeps drops from sitting on dry glass.
        plan.sort(key=lambda item: -item["volume_ul"])
        return plan

    # --- Mode 3: full LP planner ---
    plan_result = evaluate_formulation(inventory, recipe)
    if hasattr(plan_result, "model_dump"):
        plan_result = plan_result.model_dump()
    if not plan_result.get("feasible", False):
        raise ValueError(
            f"LP planner found the recipe infeasible: {plan_result.get('issues', [])}. "
            "Consider providing an explicit 'mix_plan'."
        )
    per_solution: dict[str, float] = {}
    for instr in plan_result.get("instructions", []):
        name = instr["source_solution"]
        per_solution[name] = per_solution.get(name, 0.0) + float(instr["volume_ul"])
    return [
        {"source_solution": name, "volume_ul": vol} for name, vol in per_solution.items()
    ]


def allocate_transfers(
    mix_plan: Sequence[dict],
    inventory: Inventory,
    dest: tuple[int, int],
) -> list[dict]:
    """Expand per-solution volumes into per-vial transfers, excluding the
    destination vial as a source. Greedy allocation in (x, y) order, matching
    the existing planner's convention."""
    transfers: list[dict] = []
    step = 1
    for entry in mix_plan:
        name = entry["source_solution"]
        remaining = float(entry["volume_ul"])
        candidates = sorted(
            (
                v
                for v in inventory.vials_for_solution(name)
                if (v.x_ind, v.y_ind) != dest
            ),
            key=lambda v: (v.x_ind, v.y_ind),
        )
        available = sum(v.volume_ul for v in candidates)
        if available < remaining - 1e-6:
            raise ValueError(
                f"Not enough '{name}' outside the destination vial: need "
                f"{remaining:.1f} uL, have {available:.1f} uL. Refill and update inventory."
            )
        for vial in candidates:
            if remaining <= 1e-9:
                break
            take = min(vial.volume_ul, remaining)
            if take > 0:
                transfers.append(
                    {
                        "step_index": step,
                        "source_solution": name,
                        "source_x_ind": vial.x_ind,
                        "source_y_ind": vial.y_ind,
                        "volume_ul": take,
                    }
                )
                step += 1
                remaining -= take
    return transfers


def _fair_chunks(volume_ul: float, max_chunk: float = MAX_PIPETTE_VOLUME) -> list[int]:
    """Split a volume into near-equal integer-uL chunks, each <= max_chunk.
    Near-equal chunks avoid a tiny (inaccurate) remainder transfer."""
    total = int(round(volume_ul))
    if total <= 0:
        return []
    n_chunks = max(1, math.ceil(total / max_chunk))
    base = total // n_chunks
    remainder = total - base * n_chunks
    return [base + 1] * remainder + [base] * (n_chunks - remainder)


# ---------------------------------------------------------------------------
# Preflight + text simulation
# ---------------------------------------------------------------------------

def preflight_mix(
    recipe: dict,
    inventory: Inventory,
    tip_rack: TipRack,
    dest: tuple[int, int],
) -> tuple[list[dict], list[str]]:
    """Validate a mix and return (transfers, warnings). Raises on hard errors."""
    warnings: list[str] = []
    mix_plan = build_mix_plan(recipe, inventory)
    transfers = allocate_transfers(mix_plan, inventory, dest)

    total_volume = sum(t["volume_ul"] for t in transfers)

    # Destination capacity
    dx, dy = dest
    dest_volume, dest_capacity, dest_name = 0.0, 1500.0, None
    for vial in inventory.vials:
        if (vial.x_ind, vial.y_ind) == dest:
            dest_volume = vial.volume_ul
            dest_capacity = vial.capacity_ul
            dest_name = vial.electrolyte_name()
            break
    if dest_volume + total_volume > dest_capacity:
        raise ValueError(
            f"Destination vial ({dx}, {dy}) cannot hold the mix: "
            f"{dest_volume:.1f} uL present + {total_volume:.1f} uL new > "
            f"capacity {dest_capacity:.1f} uL."
        )
    if dest_name is not None and dest_volume > 0:
        warnings.append(
            f"Destination vial ({dx}, {dy}) already contains {dest_volume:.1f} uL of "
            f"'{dest_name}' — the new mix will be combined with it."
        )

    # Tip availability (one tip per distinct source substance, reusing
    # same-substance tips per TipRack policy).
    clean = sum(1 for tip in tip_rack.tips if tip.current_substance_name is None)
    assigned = {
        tip.current_substance_name
        for tip in tip_rack.tips
        if tip.current_substance_name is not None
    }
    needed = 0
    for name in dict.fromkeys(t["source_solution"] for t in transfers):
        if name not in assigned:
            needed += 1
    if needed > clean:
        raise ValueError(
            f"Not enough clean tips: {needed} new substance(s) but only {clean} clean tip(s)."
        )
    # +1 clean tip if the user wants the pipette-mix step (checked in the menu).

    for t in transfers:
        if t["volume_ul"] < MIN_ACCURATE_TRANSFER_UL:
            warnings.append(
                f"Transfer of {t['volume_ul']:.1f} uL of '{t['source_solution']}' is below "
                f"the ~{MIN_ACCURATE_TRANSFER_UL} uL accuracy floor of the pipette."
            )
    return transfers, warnings


def simulate_mixing(recipe: dict, inventory: Inventory, tip_rack: TipRack, dest: tuple[int, int]) -> None:
    """Print the full action sequence without touching hardware or state."""
    name = recipe.get("recipe_name", "<unnamed>")
    print(f"\n--- Mixing simulation for recipe: {name} -> vial {dest} ---")
    try:
        transfers, warnings = preflight_mix(recipe, inventory, tip_rack, dest)
    except Exception as e:
        print(f"NOT feasible: {e}")
        return
    for w in warnings:
        print(f"WARNING: {w}")
    current = None
    for t in transfers:
        if t["source_solution"] != current:
            if current is not None:
                print(" - Return tip to rack (substance change)")
            current = t["source_solution"]
            print(f" - Acquire tip for '{current}'")
        chunks = _fair_chunks(t["volume_ul"])
        for chunk in chunks:
            print(
                f"   aspirate {chunk} uL of '{t['source_solution']}' from vial "
                f"({t['source_x_ind']},{t['source_y_ind']}) -> dispense + blowout into vial {dest}"
            )
    if current is not None:
        print(" - Return final tip to rack")
    total = sum(t["volume_ul"] for t in transfers)
    mix_vol = min(int(MAX_PIPETTE_VOLUME), max(1, int(DEFAULT_MIX_FRACTION * total)))
    print(
        f" - (optional) pipette-mix vial {dest}: {DEFAULT_MIX_CYCLES} x {mix_vol} uL with a fresh tip"
    )
    print(f"--- End simulation: {total:.1f} uL total into vial {dest} ---\n")


# ---------------------------------------------------------------------------
# Executor
# ---------------------------------------------------------------------------

def _recipe_label_spec(recipe: dict, source_spec: ElectrolyteSpec) -> ElectrolyteSpec:
    """The composition attributed to one transferred aliquot = the source
    vial's own spec (works for pure stocks and pre-mixed sources alike)."""
    return source_spec


def mix_electrolyte_in_vial(
    liquid_robot: LiquidRobot,
    recipe: dict,
    dest: tuple[int, int],
    logger,
    mime: bool = False,
    do_pipette_mix: bool = True,
    mix_cycles: int = DEFAULT_MIX_CYCLES,
) -> str:
    """Execute a mixing recipe into the destination vial.

    Mirrors app.dispense_electrolyte_recipe's structure (tip-per-substance,
    fail-fast with status strings, inventory consumed only after successful
    hardware actions) but the destination is a vial in the liquid case rather
    than the assembly post, transfers are chunked to the 200 uL pipette limit,
    and the destination vial's composition/volume are tracked via add_to_vial.

    With mime=True the arm performs all approach movements but the pipette
    never actuates and no inventory/tip state is persisted.
    """
    name = str(recipe.get("recipe_name", "<unnamed>"))
    extra = "SIMULATING (mime):  " if mime else ""
    logger.info(f"----- {extra}Mixing recipe '{name}' into vial {dest} -----")

    inventory = load_inventory_state()
    try:
        tip_rack = load_tip_rack_state()
    except Exception as e:
        logger.error(f"Error loading tip rack state: {e}")
        return "failed_to_load_tip_rack"

    try:
        transfers, warnings = preflight_mix(recipe, inventory, tip_rack, dest)
    except Exception as e:
        logger.error(f"Mix preflight failed for '{name}': {e}")
        return "failed_preflight"
    for w in warnings:
        logger.warning(w)

    mg = liquid_robot.MG400
    logger.info("Re-homing liquid robot before mixing.")
    mg.move_home()

    dx, dy = dest
    current_substance: Optional[str] = None
    tip_index: Optional[int] = None
    tip_x = tip_y = None
    dispensed_total = 0.0

    def _persist_state():
        if mime:
            return
        try:
            save_inventory_state(inventory)
        except Exception as e:
            logger.warning(f"Failed to save inventory state: {e}")
        try:
            save_tip_rack_state(tip_rack)
        except Exception as e:
            logger.warning(f"Failed to save tip rack state: {e}")

    for t in transfers:
        sx, sy = int(t["source_x_ind"]), int(t["source_y_ind"])
        src = t["source_solution"]

        # -- Tip management on substance change (same policy as recipe dispenser)
        if src != current_substance:
            if tip_index is not None:
                logger.info(f"Returning tip {tip_index} to rack (substance change)")
                try:
                    mg.drop_tip(tip_x, tip_y)
                except Exception as e:
                    logger.error(f"Movement to return tip failed: {e}")
                    return "failed_to_return_tip"
                tip_rack.mark_tip_used(tip_index, current_substance)
                _persist_state()
            current_substance = src
            try:
                tip_index = _tip_index_for_substance_or_error(tip_rack, current_substance)
                tip_x, tip_y = _tip_index_to_coordinates(tip_index)
            except Exception as e:
                logger.error(f"Failed to find a tip for '{current_substance}': {e}")
                return "failed_to_acquire_tip"
            logger.info(f"Using tip {tip_index} for substance '{current_substance}'")
            try:
                mg.get_tip(tip_x, tip_y)
            except Exception as e:
                logger.error(f"Failed to get tip {tip_index}: {e}")
                logger.warning("WARNING: remove tip before continuing to avoid collisions.")
                return "failed_to_get_tip"

        # Look up the source vial's spec BEFORE consuming from it, so the
        # aliquot's composition is attributed correctly.
        source_spec: Optional[ElectrolyteSpec] = None
        for vial in inventory.vials:
            if vial.x_ind == sx and vial.y_ind == sy:
                source_spec = vial.current_electrolyte
                break
        if source_spec is None:
            logger.error(f"No solution recorded at source vial ({sx}, {sy}).")
            return "failed_missing_source_spec"

        # -- Chunked transfer
        for chunk in _fair_chunks(t["volume_ul"]):
            logger.info(
                f"Transferring {chunk} uL of '{src}' from ({sx}, {sy}) to vial ({dx}, {dy})"
            )
            try:
                mg.get_liquid(sx, sy, chunk, mime=mime)
            except Exception as e:
                logger.error(f"Movement/aspiration at vial ({sx}, {sy}) failed: {e}")
                return "failed_to_get_liquid"
            try:
                mg.dispense_liquid_to_vial(dx, dy, chunk, blowout=True, mime=mime)
            except Exception as e:
                logger.error(f"Failed to dispense into vial ({dx}, {dy}): {e}")
                return "failed_to_dispense_to_vial"

            if not mime:
                # Book-keep only after both hardware actions succeeded, one
                # chunk at a time, so a mid-run failure leaves the inventory
                # file matching physical reality.
                try:
                    inventory.consume_solution_from_vial(sx, sy, float(chunk))
                except Exception as e:
                    logger.error(f"Failed to consume from vial ({sx}, {sy}): {e}")
                try:
                    inventory = add_to_vial(
                        inventory,
                        x_ind=dx,
                        y_ind=dy,
                        electrolyte=_recipe_label_spec(recipe, source_spec),
                        volume_ul=float(chunk),
                        mixed_name=name,
                    )
                except Exception as e:
                    logger.error(f"Failed to record addition to vial ({dx}, {dy}): {e}")
                _persist_state()
                dispensed_total += chunk

    # -- Return final transfer tip
    if tip_index is not None:
        logger.info(f"Returning final tip {tip_index} to rack")
        try:
            mg.drop_tip(tip_x, tip_y)
            mg.move_home()
        except Exception as e:
            logger.error(f"Failed returning tip ({tip_x}, {tip_y}) / re-homing: {e}")
            logger.warning("WARNING: remove tip before continuing to avoid collisions.")
            return "failed_to_return_final_tip"
        tip_rack.mark_tip_used(tip_index, current_substance)
        _persist_state()

    # -- Optional homogenization with a fresh tip
    if do_pipette_mix:
        final_volume = dispensed_total if not mime else sum(
            t["volume_ul"] for t in transfers
        )
        mix_volume = min(int(MAX_PIPETTE_VOLUME), max(1, int(DEFAULT_MIX_FRACTION * final_volume)))
        try:
            mix_tip = _tip_index_for_substance_or_error(tip_rack, name)
            mtx, mty = _tip_index_to_coordinates(mix_tip)
        except Exception as e:
            logger.warning(f"Skipping pipette-mix, no tip available: {e}")
            logger.info(f"----- Finished mixing '{name}' (unmixed) -----\n")
            return "ok_but_not_homogenized"
        logger.info(
            f"Pipette-mixing vial ({dx}, {dy}): {mix_cycles} x {mix_volume} uL with tip {mix_tip}"
        )
        try:
            mg.get_tip(mtx, mty)
            mg.pipette_mix_at_vial(dx, dy, mix_volume, cycles=mix_cycles, mime=mime)
            mg.drop_tip(mtx, mty)
            mg.move_home()
        except Exception as e:
            logger.error(f"Pipette-mix step failed: {e}")
            logger.warning("WARNING: verify tip state before continuing to avoid collisions.")
            return "failed_pipette_mix"
        tip_rack.mark_tip_used(mix_tip, name)
        _persist_state()

    logger.info(f"----- Finished mixing recipe '{name}' into vial {dest} -----\n")
    return "ok"


# ---------------------------------------------------------------------------
# Interactive menu + standalone app
# ---------------------------------------------------------------------------

def _read_int_or_cancel(prompt: str) -> Optional[int]:
    while True:
        raw = input(prompt).strip()
        if raw.lower() in {"q", "x", "cancel"}:
            return None
        try:
            value = int(raw)
        except ValueError:
            print("Please enter an integer, or 'q' to cancel.")
            continue
        if value < 0:
            print("Please enter a value >= 0.")
            continue
        return value


def _read_recipes_interactively() -> Optional[list[dict]]:
    """Build a single explicit-volume recipe from terminal input."""
    name = input("Name for the new mixture (used to label the vial): ").strip()
    if not name or name.lower() in {"q", "x", "cancel"}:
        return None
    mix_plan = []
    print("Enter each source solution and volume. Press Enter on an empty name to finish.")
    while True:
        source = input("  Source solution name (as loaded in inventory): ").strip()
        if not source:
            break
        if source.lower() in {"q", "x", "cancel"}:
            return None
        raw_vol = input(f"  Volume of '{source}' in uL: ").strip()
        try:
            vol = float(raw_vol)
            if vol <= 0:
                raise ValueError
        except ValueError:
            print("  Volume must be a positive number; entry discarded.")
            continue
        mix_plan.append({"source_solution": source, "volume_ul": vol})
    if not mix_plan:
        print("No transfers entered.")
        return None
    return [{"recipe_name": name, "mix_plan": mix_plan}]


def _load_mix_recipes_from_file() -> Optional[list[dict]]:
    raw = input("Path to mixing recipes JSON file: ").strip()
    if not raw or raw.lower() in {"q", "x", "cancel"}:
        return None
    path = Path(raw).expanduser()
    if not path.exists():
        print(f"File not found: {path}")
        return None
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if isinstance(payload, dict):
        payload = payload.get("recipes", [payload])
    if not isinstance(payload, list) or not payload:
        print("Recipes file must contain a recipe object or a non-empty list of them.")
        return None
    return payload


def _resolve_destination(recipe: dict) -> Optional[tuple[int, int]]:
    dest = recipe.get("destination") or {}
    if "x_ind" in dest and "y_ind" in dest:
        dx, dy = int(dest["x_ind"]), int(dest["y_ind"])
        confirm = input(
            f"Recipe '{recipe.get('recipe_name', '<unnamed>')}' targets vial "
            f"({dx}, {dy}). Use it? (y/n, default y): "
        ).strip().lower()
        if confirm in {"", "y", "yes"}:
            return dx, dy
    dx = _read_int_or_cancel("Destination vial index x: ")
    if dx is None:
        return None
    dy = _read_int_or_cancel("Destination vial index y: ")
    if dy is None:
        return None
    return dx, dy


def mixing_menu(liquid_robot: LiquidRobot, logger) -> None:
    """Interactive workflow: load recipes -> per recipe: pick destination,
    preflight, simulate, execute."""
    try:
        inventory = load_inventory_state()
    except Exception as e:
        print(f"Unable to open mixing menu (inventory load failed): {e}")
        return

    print("\n================ Solvent / Electrolyte Mixing ================")
    print_vial_statuses(inventory)
    source_choice = input(
        "\nRecipe source: [F]ile (JSON), [I]nteractive volumes, [Enter] to cancel: "
    ).strip().lower()
    if source_choice == "f":
        recipes = _load_mix_recipes_from_file()
    elif source_choice == "i":
        recipes = _read_recipes_interactively()
    else:
        print("Canceled.")
        return
    if not recipes:
        print("Canceled.")
        return

    for idx, recipe in enumerate(recipes, start=1):
        name = str(recipe.get("recipe_name", f"mix_{idx}"))
        print(f"\n----- Recipe {idx}/{len(recipes)}: '{name}' -----")
        dest = _resolve_destination(recipe)
        if dest is None:
            print("Skipped (no destination).")
            continue

        # Fresh state each recipe: prior mixes change the inventory.
        inventory = load_inventory_state()
        try:
            tip_rack = load_tip_rack_state()
            transfers, warnings = preflight_mix(recipe, inventory, tip_rack, dest)
        except Exception as e:
            print(f"Preflight failed: {e}")
            continue
        total = sum(t["volume_ul"] for t in transfers)
        print(f"Plan: {len(transfers)} vial draw(s), {total:.1f} uL total into vial {dest}.")
        for w in warnings:
            print(f"WARNING: {w}")

        while True:
            action = input(
                "Choose action: [T]ext simulation, [M]ovement simulation (no liquid), "
                "[G]o (real transfer), [S]kip recipe: "
            ).strip().lower()
            if action == "t":
                simulate_mixing(recipe, inventory, tip_rack, dest)
                continue
            if action == "m":
                status = mix_electrolyte_in_vial(
                    liquid_robot, recipe, dest, logger, mime=True
                )
                print(f"Movement simulation finished with status: {status}")
                continue
            if action == "g":
                do_mix = input(
                    "Run pipette-mix homogenization at the end? (y/n, default y): "
                ).strip().lower() in {"", "y", "yes"}
                confirm = input(
                    f"CONFIRM: transfer {total:.1f} uL into vial {dest} now? (y/n): "
                ).strip().lower()
                if confirm != "y":
                    print("Not confirmed.")
                    continue
                status = mix_electrolyte_in_vial(
                    liquid_robot, recipe, dest, logger, mime=False, do_pipette_mix=do_mix
                )
                print(f"Mixing finished with status: {status}")
                if status.startswith("ok"):
                    inventory = load_inventory_state()
                    print_vial_statuses(inventory)
                break
            if action == "s":
                print("Skipped.")
                break
            print("Invalid choice.")

    print("================ Mixing session complete ================\n")


def main():
    """Standalone entry point: only the liquid robot is initialized, so
    electrolytes can be mixed without powering the Meca500s / crimper."""
    import rclpy

    rclpy.init()
    liquid_robot = LiquidRobot(ip="192.168.0.107")
    ok = liquid_robot.initialize_robot()
    if not ok:
        print("Failed to initialize the Liquid Robot, program aborted!")
        rclpy.shutdown()
        return
    try:
        mixing_menu(liquid_robot, liquid_robot.logger)
    finally:
        liquid_robot.disconnect()
        print("MG400 disconnected safely.")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
