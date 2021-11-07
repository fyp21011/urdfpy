import os
if 'DISPLAY' not in os.environ or len(os.environ['DISPLAY']) == 0:
    os.environ['PYOPENGL_PLATFORM'] = 'egl'

from urdfpy.urdf import (URDFType,
                   Box, Cylinder, Sphere, Mesh, Geometry,
                   Texture, Material,
                   Collision, Visual, Inertial,
                   JointCalibration, JointDynamics, JointLimit, JointMimic,
                   SafetyController, Actuator, TransmissionJoint,
                   Transmission, Joint, Link, URDF)
from .utils import (rpy_to_matrix, matrix_to_rpy, xyz_rpy_to_matrix,
                    matrix_to_xyz_rpy)
from .version import __version__

__all__ = [
    'URDFType', 'Box', 'Cylinder', 'Sphere', 'Mesh', 'Geometry',
    'Texture', 'Material', 'Collision', 'Visual', 'Inertial',
    'JointCalibration', 'JointDynamics', 'JointLimit', 'JointMimic',
    'SafetyController', 'Actuator', 'TransmissionJoint',
    'Transmission', 'Joint', 'Link', 'URDF',
    'rpy_to_matrix', 'matrix_to_rpy', 'xyz_rpy_to_matrix', 'matrix_to_xyz_rpy',
    '__version__'
]
