"""Backend-independent behavior port of the inspected AutoBio instrument logic.

This module computes decisions only. A backend MUST apply and validate the
physical lock/actuator; a boolean alone is never physical-task acceptance.
Coordinates are original MuJoCo joint coordinates, radians (not USD degrees).
"""
from dataclasses import dataclass
import math


def detent_index(q_rad:float,low_rad:float,high_rad:float,steps:int)->int:
    """Python UI state: NumPy rint's ties-to-even, clamped to the endpoints.

    Do not use C round here: the original passive plugin and UI deliberately
    have different rounding rules at exact half-step boundaries.
    """
    if not all(math.isfinite(x) for x in (q_rad,low_rad,high_rad)):
        raise ValueError("Nonfinite detent coordinate")
    if not isinstance(steps,int) or steps<2 or high_rad<=low_rad:
        raise ValueError("Detent requires an increasing range and >=2 steps")
    spacing=(high_rad-low_rad)/(steps-1)
    return min(steps-1,max(0,round((q_rad-low_rad)/spacing)))


@dataclass(frozen=True)
class LockDecision:
    enabled:bool
    bad_locking:bool


class C1000LockController:
    """Source ThermalCyclerBioradC1000._reset/_update semantics, with latch.

    Lever state1 clears the bad-lock latch and disables the lid equality.
    Trying to lock with the lid open latches bad_locking until an unlock.
    In branches with no source assignment, retain current constraint state.
    """
    def __init__(self,lid_min_rad:float):
        if not math.isfinite(lid_min_rad):raise ValueError("Nonfinite lid limit")
        self.lid_min_rad=lid_min_rad
        self.bad_locking=False

    def reset(self,lid_q_rad:float,lever_state:int,current_enabled:bool)->LockDecision:
        self.bad_locking=False
        return self.update(lid_q_rad,lever_state,current_enabled)

    def update(self,lid_q_rad:float,lever_state:int,current_enabled:bool)->LockDecision:
        if not math.isfinite(lid_q_rad) or lever_state not in (0,1):
            raise ValueError("Invalid C1000 state")
        enabled=bool(current_enabled)
        if lever_state==1:
            self.bad_locking=False
            enabled=False
        elif lid_q_rad>self.lid_min_rad+0.01:
            self.bad_locking=True
        elif not self.bad_locking:
            enabled=True
        return LockDecision(enabled,self.bad_locking)


@dataclass(frozen=True)
class VortexDecision:
    selected_rpm:float
    target_velocity_rad_s:float


def vortex_command(switch_state:int,knob_state:int,platform_touch:bool)->VortexDecision:
    """Source mapping: touch/off/on, 0RPM or 600..3200RPM, any contact touch.

    Touch detection has no force threshold in the original Python source.
    The backend supplies platform_touch from real platform-body contacts.
    """
    if switch_state not in (0,1,2) or knob_state not in range(11):
        raise ValueError("Invalid vortex detent state")
    rpm=0. if knob_state==0 else 600.+(knob_state-1)*(3200.-600.)/9
    running=switch_state==2 or (switch_state==0 and bool(platform_touch))
    return VortexDecision(rpm,rpm*2*math.pi/60 if running else 0.)
