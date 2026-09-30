"""Pulse-derived commanded reference arithmetic. Must match the firmware exactly."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "analysis"))
from common.motor_reference import (angle_to_pulses, effective_commanded_deg,
                                    PULSES_PER_REV, DEG_PER_PULSE,
                                    QUANTIZATION_BOUND_DEG)

def test_resolution():
    assert PULSES_PER_REV == 25600
    assert abs(DEG_PER_PULSE - 0.0140625) < 1e-12
    assert abs(QUANTIZATION_BOUND_DEG - 0.00703125) < 1e-12

def test_pulse_counts():
    for deg, pul in ((5, 356), (10, 711), (15, 1067), (20, 1422), (-30, -2133)):
        assert angle_to_pulses(deg) == pul, deg

def test_effective_commanded_values():
    for deg, eff in ((5, 5.0062500), (10, 9.9984375),
                     (15, 15.0046875), (20, 19.9968750), (-30, -29.9953125)):
        assert abs(effective_commanded_deg(deg) - eff) < 1e-9, deg

def test_quantization_is_bounded():
    for deg in range(-40, 41):
        assert abs(effective_commanded_deg(deg) - deg) <= QUANTIZATION_BOUND_DEG + 1e-12

def test_sign_symmetry():
    for deg in (5, 10, 15, 20, 30):
        assert angle_to_pulses(-deg) == -angle_to_pulses(deg)
