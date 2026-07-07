# ============================================================================
# PATCH 2 of 5 — BatteryLab/electrolyte_planner/operations.py
#
# Append everything below to the END of operations.py, then add the exports
# listed at the bottom of this file to electrolyte_planner/__init__.py.
#
# Why this exists: the planner package can overwrite a vial (set_vial_contents)
# and subtract from a vial (Inventory.consume_solution_from_vial), but there is
# no operation for ADDING liquid to a vial while tracking what the resulting
# mixture is. add_to_vial fills that gap so a robot-mixed vial is immediately
# usable as a source for later recipes / cell dispensing.
#
# The composition combine is done by NAME-keyed volumetric averaging (same math
# as solvency.core.Electrolyte.combine) rather than by round-tripping through
# solvency Electrolyte objects. Reason: Electrolyte.__init__ resolves SMILES
# for every component and will hit PubChem (or raise, if offline and no
# local_smiles) — we do not want an inventory bookkeeping step to depend on
# network access on the Pi. Consequence: component NAMES must be used
# consistently across stock vials (e.g. always "water", not sometimes "H2O").
# local_smiles dicts from both sides are merged so SMILES-based planning still
# works on the result.
# ============================================================================

from .models import VIAL_MAX_VOLUME_UL


def _combine_specs_by_name(
    spec_a: Optional[ElectrolyteSpec],
    volume_a_ul: float,
    spec_b: ElectrolyteSpec,
    volume_b_ul: float,
    mixed_name: Optional[str] = None,
) -> ElectrolyteSpec:
    """Volumetrically combine two electrolyte specs, keyed by component name.

    Solvent volume fractions (v) and salt/additive molarities (s, a) are
    averaged weighted by volume, mirroring solvency.core.Electrolyte.combine.
    spec_a may be None / volume_a_ul may be 0 (empty destination vial).
    """
    if volume_b_ul <= 0:
        raise ValueError("Added volume must be positive")
    if spec_a is None or volume_a_ul <= 0:
        result = spec_b.model_copy(deep=True) if hasattr(spec_b, "model_copy") else spec_b.copy(deep=True)
        result.volume = float(volume_b_ul)
        if mixed_name:
            result.name = mixed_name
        return result

    total = float(volume_a_ul) + float(volume_b_ul)

    def _weighted(dict_a, dict_b):
        merged = {}
        for key in set(dict_a.keys()).union(dict_b.keys()):
            merged[key] = (
                dict_a.get(key, 0.0) * volume_a_ul + dict_b.get(key, 0.0) * volume_b_ul
            ) / total
        return merged

    name = mixed_name or f"{spec_a.name} + {spec_b.name}"
    local_smiles = {**(spec_a.local_smiles or {}), **(spec_b.local_smiles or {})}
    return ElectrolyteSpec(
        name=name,
        volume=total,
        v=_weighted(spec_a.v, spec_b.v),
        s=_weighted(spec_a.s, spec_b.s),
        a=_weighted(spec_a.a, spec_b.a),
        local_smiles=local_smiles or None,
        use_pubchem=bool(spec_a.use_pubchem or spec_b.use_pubchem),
    )


def add_to_vial(
    inventory_data: Inventory | dict,
    x_ind: int,
    y_ind: int,
    electrolyte: ElectrolyteSpec | dict,
    volume_ul: float,
    *,
    mixed_name: Optional[str] = None,
    create_if_missing: bool = True,
) -> Inventory:
    """Add `volume_ul` of `electrolyte` to the vial at (x_ind, y_ind), updating
    the vial's composition to the volumetric mixture of old + new contents.

    Raises ValueError if the addition would exceed the vial's capacity.
    Pass `mixed_name` to keep the vial labeled with a stable recipe name while
    it is being built up over several additions.
    """
    inventory = _to_inventory(inventory_data)
    electrolyte_spec = _to_electrolyte(electrolyte)

    if x_ind < 0 or y_ind < 0:
        raise ValueError("x_ind and y_ind must be >= 0")
    if volume_ul <= 0:
        raise ValueError("volume_ul must be positive")

    target = _find_vial(inventory, x_ind, y_ind)
    if target is None:
        if not create_if_missing:
            raise KeyError(f"vial coordinates not found: x_ind={x_ind}, y_ind={y_ind}")
        if volume_ul > VIAL_MAX_VOLUME_UL:
            raise ValueError(
                f"Cannot add {volume_ul:.1f} uL: exceeds vial capacity {VIAL_MAX_VOLUME_UL:.1f} uL"
            )
        new_spec = _combine_specs_by_name(None, 0.0, electrolyte_spec, volume_ul, mixed_name)
        inventory.vials.append(
            VialContents(
                x_ind=x_ind,
                y_ind=y_ind,
                current_electrolyte=new_spec,
                previous_electrolyte=None,
                volume_ul=volume_ul,
            )
        )
        return Inventory(**(inventory.model_dump() if hasattr(inventory, "model_dump") else inventory.dict()))

    if target.volume_ul + volume_ul > target.capacity_ul:
        raise ValueError(
            f"Cannot add {volume_ul:.1f} uL to vial ({x_ind}, {y_ind}): "
            f"{target.volume_ul:.1f} uL present, capacity {target.capacity_ul:.1f} uL"
        )

    if target.current_electrolyte is not None and target.volume_ul > 0:
        # Keep history so cleaning/reuse decisions can see what was here before
        # this mixing session started (only recorded on the first addition that
        # changes identity).
        if (
            target.previous_electrolyte is None
            and mixed_name is not None
            and target.current_electrolyte.name != mixed_name
        ):
            target.previous_electrolyte = target.current_electrolyte

    combined = _combine_specs_by_name(
        target.current_electrolyte if target.volume_ul > 0 else None,
        target.volume_ul,
        electrolyte_spec,
        volume_ul,
        mixed_name,
    )
    target.current_electrolyte = combined
    target.volume_ul = float(target.volume_ul) + float(volume_ul)

    return Inventory(**(inventory.model_dump() if hasattr(inventory, "model_dump") else inventory.dict()))


# ============================================================================
# Also update BatteryLab/electrolyte_planner/__init__.py:
#
#   1. change:   from .operations import clear_vial, set_vial_contents
#      to:       from .operations import add_to_vial, clear_vial, set_vial_contents
#
#   2. add       "add_to_vial",
#      to the __all__ list (anywhere, e.g. next to "clear_vial").
# ============================================================================
