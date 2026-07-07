# ============================================================================
# PATCH 1 of 5 — BatteryLab/robots/MG400.py
#
# Paste the three methods below into the MG400 class, directly AFTER the
# existing `add_liquid_to_post` method (around line 381 on main).
# No other changes to this file are required.
#
# Design notes:
#  * dispense_liquid_to_vial mirrors the geometry of move_to_liquid/get_liquid
#    but expels liquid INTO a vial instead of aspirating from it. The tip stops
#    at a partial descent level (default 0.5, i.e. halfway between the taught
#    "down" and "up" poses) so it stays ABOVE the destination liquid surface.
#    This keeps the tip uncontaminated by the destination mixture, so the same
#    tip can keep being reused for its source solvent.  TUNE DISPENSE_LEVEL
#    ON HARDWARE: it must clear the max fill line of a 1500 uL vial but stay
#    inside the vial mouth so droplets can't splash out.
#  * Blowout at the DESTINATION (not the source, unlike return_liquid) so the
#    full aspirated volume ends up in the mix. This keeps the inventory math
#    exact: consume(V) at source == add(V) at destination.
#  * pipette_mix_at_vial does aspirate/dispense cycles at the bottom of the
#    destination vial to homogenize (there is no stirrer/vortexer on the
#    platform). The tip used for this touches the mixture, so the caller must
#    treat it as contaminated with the mixture (mark it used accordingly).
# ============================================================================

    # Fraction of the down->up travel at which the tip hovers while dispensing
    # into a vial. 0.0 = taught down pose (in the liquid), 1.0 = taught up pose.
    # Must be above the liquid line of a full (1500 uL) vial. Tune on hardware.
    DISPENSE_LEVEL = 0.5
    # Level to retreat to before blowing out after a pipette-mix cycle, so the
    # blowout does not blast bubbles/splash at the bottom of the vial.
    MIX_BLOWOUT_LEVEL = 0.5

    def _liquid_pose_at_level(self, x, y, level: float):
        """Interpolated pose between the taught down (level=0) and up (level=1)
        poses of the liquid-case well (x, y). Z is interpolated; XY/R follow
        the down pose, matching the convention used in move_to_liquid."""
        idx = self.get_liquid_index(x, y)
        up_pos = self.liquid_poses_up[idx]
        down_pos = self.liquid_poses_down[idx]
        return [
            down_pos[0],
            down_pos[1],
            (up_pos[2] - down_pos[2]) * level + down_pos[2],
            down_pos[3],
        ]

    def dispense_liquid_to_vial(self, x, y, volume, level=None, blowout=True, mime=False):
        """Dispense `volume` uL from the currently-attached tip into the vial
        at liquid-case coordinates (x, y).

        The tip descends only to `level` (default DISPENSE_LEVEL) so it does
        not touch the destination liquid. With `blowout=True` (default) the
        pipette runs a blowout after dispensing so the entire aspirated volume
        is transferred. With `mime=True` the arm gestures at the vial but the
        pipette does not actuate (movement-simulation mode).
        """
        self._validate_liquid_index(x, y)
        self._validate_volume(volume)
        if level is None:
            level = self.DISPENSE_LEVEL
        if not (0.0 <= level <= 1.0):
            raise ValueError(f"Dispense level must be within [0, 1], got {level}")

        # Approach from the safe up pose, then descend to the hover level.
        self.move_to_liquid(x, y, level=1)
        self.dashboard.SpeedL(3)
        idx = self.get_liquid_index(x, y)
        self.movectl.MovL(*self._liquid_pose_at_level(x, y, level))
        self.movectl.Sync()
        if not mime:
            self.sartorius_rline.dispense(volume)
            if blowout:
                self.sartorius_rline.blowout()
        self.movectl.MovL(*self.liquid_poses_up[idx])
        self.movectl.Sync()
        self.logger.info(
            f"Dispensed {volume} uL into vial ({x}, {y}) at level {level:.2f}"
            f"{' [MIME]' if mime else ''}."
        )
        self.move_home()

    def pipette_mix_at_vial(self, x, y, volume, cycles=5, mime=False):
        """Homogenize the vial at (x, y) by repeated aspirate/dispense cycles
        at the taught down pose. `volume` uL is drawn and expelled `cycles`
        times, then the arm retreats to MIX_BLOWOUT_LEVEL and blows out to
        clear the tip. The tip becomes contaminated with the mixture: the
        caller is responsible for marking it used for the mixture substance.
        """
        self._validate_liquid_index(x, y)
        self._validate_volume(volume)
        if cycles <= 0:
            raise ValueError(f"Mixing cycles must be a positive integer, got {cycles}")

        self.move_to_liquid(x, y, level=1)
        self.dashboard.SpeedL(3)
        idx = self.get_liquid_index(x, y)
        self.movectl.MovL(*self.liquid_poses_down[idx])
        self.movectl.Sync()
        if not mime:
            for _ in range(cycles):
                self.sartorius_rline.aspirate(volume)
                self.sartorius_rline.dispense(volume)
        # Retreat above the liquid line before blowing out residual.
        self.movectl.MovL(*self._liquid_pose_at_level(x, y, self.MIX_BLOWOUT_LEVEL))
        self.movectl.Sync()
        if not mime:
            self.sartorius_rline.blowout()
        self.movectl.MovL(*self.liquid_poses_up[idx])
        self.movectl.Sync()
        self.logger.info(
            f"Pipette-mixed vial ({x}, {y}) with {cycles} cycles of {volume} uL"
            f"{' [MIME]' if mime else ''}."
        )
        self.move_home()
