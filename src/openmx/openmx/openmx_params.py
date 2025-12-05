"""
Shared robot parameters for OpenManipulator-X
"""
import math

# Linkage lengths (in meters)
L0 = 0.036076
L1 = 0.096326 - L0
L2V = 0.128000
L2H = 0.024000
L2 = math.sqrt(L2V**2 + L2H**2)
L3 = 0.124000
L4 = 0.133400

def get_robot_params():
    """Return dictionary of robot parameters"""
    return {
        'L0': L0,
        'L1': L1,
        'L2': L2,
        'L3': L3,
        'L4': L4
    }