"""universal-form kinematics: the minimal manipulator chain and its closed-form IK.

See the repo README for *why* the chain looks like this. The short version:

    base (x,y,yaw) + lift (z) + arm[ shoulder-yaw, shoulder-pitch, elbow, spherical-wrist ]

The arm's three wrist axes intersect at a point (Pieper), so IK is closed-form.
"""

from .arm import Arm
from .form import UniversalForm
from . import se3

__all__ = ["Arm", "UniversalForm", "se3"]
