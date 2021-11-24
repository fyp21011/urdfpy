from collections import OrderedDict
import copy
import os
import time

from lxml import etree as ET
import networkx as nx
import numpy as np
import open3d as o3d
import six

from urdfpy.urdf.base import URDFType
from urdfpy.urdf.joint import (
    JointCalibration,
    JointDynamics,
    JointLimit,
    JointMimic,
    SafetyController
)
from urdfpy.urdf.link import Collision, Inertial, Material, Visual
from urdfpy.urdf.transmission import TransmissionJoint, Actuator
from urdfpy.utils import parse_origin, unparse_origin, configure_origin


class Transmission(URDFType):
    """An element that describes the relationship between an actuator and a
    joint.

    Parameters
    ----------
    name : str
        The name of this transmission.
    trans_type : str
        The type of this transmission.
    joints : list of :class:`.TransmissionJoint`
        The joints connected to this transmission.
    actuators : list of :class:`.Actuator`
        The actuators connected to this transmission.
    """
    _ATTRIBS = {
        'name': (str, True),
    }
    _ELEMENTS = {
        'joints': (TransmissionJoint, True, True),
        'actuators': (Actuator, True, True),
    }
    _TAG = 'transmission'

    def __init__(self, name, trans_type, joints=None, actuators=None):
        self.name = name
        self.trans_type = trans_type
        self.joints = joints
        self.actuators = actuators

    @property
    def name(self):
        """str : The name of this transmission.
        """
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def trans_type(self):
        """str : The type of this transmission.
        """
        return self._trans_type

    @trans_type.setter
    def trans_type(self, value):
        self._trans_type = str(value)

    @property
    def joints(self):
        """:class:`.TransmissionJoint` : The joints the transmission is
        connected to.
        """
        return self._joints

    @joints.setter
    def joints(self, value):
        if value is None:
            value = []
        else:
            value = list(value)
            for v in value:
                if not isinstance(v, TransmissionJoint):
                    raise TypeError(
                        'Joints expects a list of TransmissionJoint'
                    )
        self._joints = value

    @property
    def actuators(self):
        """:class:`.Actuator` : The actuators the transmission is connected to.
        """
        return self._actuators

    @actuators.setter
    def actuators(self, value):
        if value is None:
            value = []
        else:
            value = list(value)
            for v in value:
                if not isinstance(v, Actuator):
                    raise TypeError(
                        'Actuators expects a list of Actuator'
                    )
        self._actuators = value

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        kwargs['trans_type'] = node.find('type').text
        return Transmission(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(path)
        ttype = ET.Element('type')
        ttype.text = self.trans_type
        node.append(ttype)
        return node

    def copy(self, prefix='', scale=None):
        """Create a deep copy with the prefix applied to all names.

        Parameters
        ----------
        prefix : str
            A prefix to apply to all names.

        Returns
        -------
        :class:`.Transmission`
            A deep copy.
        """
        return Transmission(
            name='{}{}'.format(prefix, self.name),
            trans_type=self.trans_type,
            joints=[j.copy(prefix) for j in self.joints],
            actuators=[a.copy(prefix) for a in self.actuators],
        )


class Joint(URDFType):
    """A connection between two links.

    There are several types of joints, including:

    - ``fixed`` - a joint that cannot move.
    - ``prismatic`` - a joint that slides along the joint axis.
    - ``revolute`` - a hinge joint that rotates about the axis with a limited
      range of motion.
    - ``continuous`` - a hinge joint that rotates about the axis with an
      unlimited range of motion.
    - ``planar`` - a joint that moves in the plane orthogonal to the axis.
    - ``floating`` - a joint that can move in 6DoF.

    Parameters
    ----------
    name : str
        The name of this joint.
    parent : str
        The name of the parent link of this joint.
    child : str
        The name of the child link of this joint.
    joint_type : str
        The type of the joint. Must be one of :obj:`.Joint.TYPES`.
    axis : (3,) float, optional
        The axis of the joint specified in joint frame. Defaults to
        ``[1,0,0]``.
    origin : (4,4) float, optional
        The pose of the child link with respect to the parent link's frame.
        The joint frame is defined to be coincident with the child link's
        frame, so this is also the pose of the joint frame with respect to
        the parent link's frame.
    limit : :class:`.JointLimit`, optional
        Limit for the joint. Only required for revolute and prismatic
        joints.
    dynamics : :class:`.JointDynamics`, optional
        Dynamics for the joint.
    safety_controller : :class`.SafetyController`, optional
        The safety controller for this joint.
    calibration : :class:`.JointCalibration`, optional
        Calibration information for the joint.
    mimic : :class:`JointMimic`, optional
        Joint mimicry information.
    """
    TYPES = ['fixed', 'prismatic', 'revolute',
             'continuous', 'floating', 'planar']
    _ATTRIBS = {
        'name': (str, True),
    }
    _ELEMENTS = {
        'dynamics': (JointDynamics, False, False),
        'limit': (JointLimit, False, False),
        'mimic': (JointMimic, False, False),
        'safety_controller': (SafetyController, False, False),
        'calibration': (JointCalibration, False, False),
    }
    _TAG = 'joint'

    def __init__(self, name, joint_type, parent, child, axis=None, origin=None,
                 limit=None, dynamics=None, safety_controller=None,
                 calibration=None, mimic=None):
        self.name = name
        self.parent = parent
        self.child = child
        self.joint_type = joint_type
        self.axis = axis
        self.origin = origin
        self.limit = limit
        self.dynamics = dynamics
        self.safety_controller = safety_controller
        self.calibration = calibration
        self.mimic = mimic

    @property
    def name(self):
        """str : Name for this joint.
        """
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def joint_type(self):
        """str : The type of this joint.
        """
        return self._joint_type

    @joint_type.setter
    def joint_type(self, value):
        value = str(value)
        if value not in Joint.TYPES:
            raise ValueError('Unsupported joint type {}'.format(value))
        self._joint_type = value

    @property
    def parent(self):
        """str : The name of the parent link.
        """
        return self._parent

    @parent.setter
    def parent(self, value):
        self._parent = str(value)

    @property
    def child(self):
        """str : The name of the child link.
        """
        return self._child

    @child.setter
    def child(self, value):
        self._child = str(value)

    @property
    def axis(self):
        """(3,) float : The joint axis in the joint frame.
        """
        return self._axis

    @axis.setter
    def axis(self, value):
        if value is None:
            value = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        else:
            value = np.asanyarray(value, dtype=np.float64)
            if value.shape != (3,):
                raise ValueError('Invalid shape for axis, should be (3,)')
            value = value / np.linalg.norm(value)
        self._axis = value

    @property
    def origin(self):
        """(4,4) float : The pose of child and joint frames relative to the
        parent link's frame.
        """
        return self._origin

    @origin.setter
    def origin(self, value):
        self._origin = configure_origin(value)

    @property
    def limit(self):
        """:class:`.JointLimit` : The limits for this joint.
        """
        return self._limit

    @limit.setter
    def limit(self, value):
        if value is None:
            if self.joint_type in ['prismatic', 'revolute']:
                raise ValueError('Require joint limit for prismatic and '
                                 'revolute joints')
        elif not isinstance(value, JointLimit):
            raise TypeError('Expected JointLimit type')
        self._limit = value

    @property
    def dynamics(self):
        """:class:`.JointDynamics` : The dynamics for this joint.
        """
        return self._dynamics

    @dynamics.setter
    def dynamics(self, value):
        if value is not None:
            if not isinstance(value, JointDynamics):
                raise TypeError('Expected JointDynamics type')
        self._dynamics = value

    @property
    def safety_controller(self):
        """:class:`.SafetyController` : The safety controller for this joint.
        """
        return self._safety_controller

    @safety_controller.setter
    def safety_controller(self, value):
        if value is not None:
            if not isinstance(value, SafetyController):
                raise TypeError('Expected SafetyController type')
        self._safety_controller = value

    @property
    def calibration(self):
        """:class:`.JointCalibration` : The calibration for this joint.
        """
        return self._calibration

    @calibration.setter
    def calibration(self, value):
        if value is not None:
            if not isinstance(value, JointCalibration):
                raise TypeError('Expected JointCalibration type')
        self._calibration = value

    @property
    def mimic(self):
        """:class:`.JointMimic` : The mimic for this joint.
        """
        return self._mimic

    @mimic.setter
    def mimic(self, value):
        if value is not None:
            if not isinstance(value, JointMimic):
                raise TypeError('Expected JointMimic type')
        self._mimic = value

    def is_valid(self, cfg):
        """Check if the provided configuration value is valid for this joint.

        Parameters
        ----------
        cfg : float, (2,) float, (6,) float, or (4,4) float
            The configuration of the joint.

        Returns
        -------
        is_valid : bool
            True if the configuration is valid, and False otherwise.
        """
        if self.joint_type not in ['fixed', 'revolute']:
            return True
        if self.joint_limit is None:
            return True
        cfg = float(cfg)
        lower = -np.infty
        upper = np.infty
        if self.limit.lower is not None:
            lower = self.limit.lower
        if self.limit.upper is not None:
            upper = self.limit.upper
        return (cfg >= lower and cfg <= upper)

    def get_child_pose(self, cfg=None):
        """Computes the child pose relative to a parent pose for a given
        configuration value.

        Parameters
        ----------
        cfg : float, (2,) float, (6,) float, or (4,4) float
            The configuration values for this joint. They are interpreted
            based on the joint type as follows:

            - ``fixed`` - not used.
            - ``prismatic`` - a translation along the axis in meters.
            - ``revolute`` - a rotation about the axis in radians.
            - ``continuous`` - a rotation about the axis in radians.
            - ``planar`` - the x and y translation values in the plane.
            - ``floating`` - the xyz values followed by the rpy values,
              or a (4,4) matrix.

            If ``cfg`` is ``None``, then this just returns the joint pose.

        Returns
        -------
        pose : (4,4) float
            The pose of the child relative to the parent.
        """
        if cfg is None:
            return self.origin
        elif self.joint_type == 'fixed':
            return self.origin
        elif self.joint_type in ['revolute', 'continuous']:
            if cfg is None:
                angle = 0.0
            else:
                angle = float(cfg)
            R = np.eye(4)
            R[:3, :3] = o3d.geometry.get_rotation_matrix_from_axis_angle(angle * self.axis)
            return self.origin.dot(R)
        elif self.joint_type == 'prismatic':
            if cfg is None:
                cfg = 0.0
            else:
                cfg = float(cfg)
            translation = np.eye(4, dtype=np.float64)
            translation[:3,3] = self.axis * cfg
            return self.origin.dot(translation)
        elif self.joint_type == 'planar':
            if cfg is None:
                cfg = np.zeros(2, dtype=np.float64)
            else:
                cfg = np.asanyarray(cfg, dtype=np.float64)
            if cfg.shape != (2,):
                raise ValueError(
                    '(2,) float configuration required for planar joints'
                )
            translation = np.eye(4, dtype=np.float64)
            translation[:3,3] = self.origin[:3,:2].dot(cfg)
            return self.origin.dot(translation)
        elif self.joint_type == 'floating':
            if cfg is None:
                cfg = np.zeros(6, dtype=np.float64)
            else:
                cfg = configure_origin(cfg)
            if cfg is None:
                raise ValueError('Invalid configuration for floating joint')
            return self.origin.dot(cfg)
        else:
            raise ValueError('Invalid configuration')

    def get_child_poses(self, cfg, n_cfgs):
        """Computes the child pose relative to a parent pose for a given set of 
        configuration values.

        Parameters
        ----------
        cfg : (n,) float or None
            The configuration values for this joint. They are interpreted
            based on the joint type as follows:

            - ``fixed`` - not used.
            - ``prismatic`` - a translation along the axis in meters.
            - ``revolute`` - a rotation about the axis in radians.
            - ``continuous`` - a rotation about the axis in radians.
            - ``planar`` - Not implemented.
            - ``floating`` - Not implemented.

            If ``cfg`` is ``None``, then this just returns the joint pose.

        Returns
        -------
        poses : (n,4,4) float
            The poses of the child relative to the parent.
        """
        if cfg is None:
            return np.tile(self.origin, (n_cfgs, 1, 1))
        elif self.joint_type == 'fixed':
            return np.tile(self.origin, (n_cfgs, 1, 1))
        elif self.joint_type in ['revolute', 'continuous']:
            if cfg is None:
                cfg = np.zeros(n_cfgs)
            return np.matmul(self.origin, self._rotation_matrices(cfg, self.axis))
        elif self.joint_type == 'prismatic':
            if cfg is None:
                cfg = np.zeros(n_cfgs)
            translation = np.tile(np.eye(4), (n_cfgs, 1, 1))
            translation[:,:3,3] = self.axis * cfg[:,np.newaxis]
            return np.matmul(self.origin, translation)
        elif self.joint_type == 'planar':
            raise NotImplementedError()
        elif self.joint_type == 'floating':
            raise NotImplementedError()
        else:
            raise ValueError('Invalid configuration')

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        kwargs['joint_type'] = str(node.attrib['type'])
        kwargs['parent'] = node.find('parent').attrib['link']
        kwargs['child'] = node.find('child').attrib['link']
        axis = node.find('axis')
        if axis is not None:
            axis = np.fromstring(axis.attrib['xyz'], sep=' ')
        kwargs['axis'] = axis
        kwargs['origin'] = parse_origin(node)
        return Joint(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(path)
        parent = ET.Element('parent')
        parent.attrib['link'] = self.parent
        node.append(parent)
        child = ET.Element('child')
        child.attrib['link'] = self.child
        node.append(child)
        if self.axis is not None:
            axis = ET.Element('axis')
            axis.attrib['xyz'] = np.array2string(self.axis)[1:-1]
            node.append(axis)
        node.append(unparse_origin(self.origin))
        node.attrib['type'] = self.joint_type
        return node

    def _rotation_matrices(self, angles, axis):
        """Compute rotation matrices from angle/axis representations.

        Parameters
        ----------
        angles : (n,) float
            The angles.
        axis : (3,) float
            The axis.

        Returns
        -------
        rots : (n,4,4)
            The rotation matrices
        """
        axis = axis / np.linalg.norm(axis)
        sina = np.sin(angles)
        cosa = np.cos(angles)
        M = np.tile(np.eye(4), (len(angles), 1, 1))
        M[:,0,0] = cosa
        M[:,1,1] = cosa
        M[:,2,2] = cosa
        M[:,:3,:3] += (
            np.tile(np.outer(axis, axis), (len(angles), 1, 1)) *
            (1.0 - cosa)[:, np.newaxis, np.newaxis]
        )
        M[:,:3,:3] += np.tile(np.array([
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0]]
        ), (len(angles), 1, 1)) * sina[:, np.newaxis, np.newaxis]
        return M

    def copy(self, prefix='', scale=None):
        """Create a deep copy of the joint with the prefix applied to all names.

        Parameters
        ----------
        prefix : str
            A prefix to apply to all joint and link names.

        Returns
        -------
        :class:`.Joint`
            A deep copy of the joint.
        """
        origin = self.origin.copy()
        if scale is not None:
            if not isinstance(scale, (list, np.ndarray)):
                scale = np.repeat(scale, 3)
            origin[:3,3] *= scale
        cpy = Joint(
            name='{}{}'.format(prefix, self.name),
            joint_type=self.joint_type,
            parent='{}{}'.format(prefix, self.parent),
            child='{}{}'.format(prefix, self.child),
            axis=self.axis.copy(),
            origin=origin,
            limit=(self.limit.copy(prefix, scale) if self.limit else None),
            dynamics=(self.dynamics.copy(prefix,scale) if self.dynamics else None),
            safety_controller=(self.safety_controller.copy(prefix, scale) if
                               self.safety_controller else None),
            calibration=(self.calibration.copy(prefix, scale) if self.calibration else None),
            mimic=(self.mimic.copy(prefix=prefix, scale=scale) if self.mimic else None)
        )
        return cpy


class Link(URDFType):
    """A link of a rigid object.

    Parameters
    ----------
    name : str
        The name of the link.
    inertial : :class:`.Inertial`, optional
        The inertial properties of the link.
    visuals : list of :class:`.Visual`, optional
        The visual properties of the link.
    collsions : list of :class:`.Collision`, optional
        The collision properties of the link.
    """

    _ATTRIBS = {
        'name': (str, True),
    }
    _ELEMENTS = {
        'inertial': (Inertial, False, False),
        'visuals': (Visual, False, True),
        'collisions': (Collision, False, True),
    }
    _TAG = 'link'

    def __init__(self, name, inertial, visuals, collisions):
        self.name = name
        self.inertial = inertial
        self.visuals = visuals
        self.collisions = collisions

        self._collision_mesh = None

    @property
    def name(self):
        """str : The name of this link.
        """
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def inertial(self):
        """:class:`.Inertial` : Inertial properties of the link.
        """
        return self._inertial

    @inertial.setter
    def inertial(self, value):
        if value is not None and not isinstance(value, Inertial):
            raise TypeError('Expected Inertial object')
        # Set default inertial
        if value is None:
            value = Inertial(mass=1.0, inertia=np.eye(3))
        self._inertial = value

    @property
    def visuals(self):
        """list of :class:`.Visual` : The visual properties of this link.
        """
        return self._visuals

    @visuals.setter
    def visuals(self, value):
        if value is None:
            value = []
        else:
            value = list(value)
            for v in value:
                if not isinstance(v, Visual):
                    raise ValueError('Expected list of Visual objects')
        self._visuals = value

    @property
    def collisions(self):
        """list of :class:`.Collision` : The collision properties of this link.
        """
        return self._collisions

    @collisions.setter
    def collisions(self, value):
        if value is None:
            value = []
        else:
            value = list(value)
            for v in value:
                if not isinstance(v, Collision):
                    raise ValueError('Expected list of Collision objects')
        self._collisions = value

    @property
    def collision_mesh(self):
        """:class:`~trimesh.base.Trimesh` : A single collision mesh for
        the link, specified in the link frame, or None if there isn't one.
        """
        if len(self.collisions) == 0:
            return None
        if self._collision_mesh is None:
            meshesList = []
            # list of meshes from ALL collision objects
            for c in self.collisions:
                m = c.geometry.meshes
                # NOTE: m here is a o3d.geometry.TriangleMesh
                m = m.copy()
                pose = c.origin
                if c.geometry.mesh is not None:
                    # c.geometry.mesh  is  the  Mesh  object,  an
                    # instance of the derived class from URDFType
                    if c.geometry.mesh.scale is not None:
                        S = np.eye(4)
                        S[:3,:3] = np.diag(c.geometry.mesh.scale)
                        pose = pose.dot(S)
                m.transform(pose)
                meshesList.append(m)
            if len(meshesList) == 0:
                return None
            self._collision_mesh = (meshesList[0] + meshesList[1:])
        return self._collision_mesh

    def copy(self, prefix='', scale=None, collision_only=False):
        """Create a deep copy of the link.

        Parameters
        ----------
        prefix : str
            A prefix to apply to all joint and link names.

        Returns
        -------
        link : :class:`.Link`
            A deep copy of the Link.
        """
        inertial = self.inertial.copy() if self.inertial is not None else None
        cm = self._collision_mesh
        if scale is not None:
            if self.collision_mesh is not None and self.inertial is not None:
                sm = np.eye(4)
                if not isinstance(scale, (list, np.ndarray)):
                    scale = np.repeat(scale, 3)
                sm[:3,:3] = np.diag(scale)
                cm = self.collision_mesh.copy()
                cm.density = self.inertial.mass / cm.volume
                cm.apply_transform(sm)
                cmm = np.eye(4)
                cmm[:3,3] = cm.center_mass
                inertial = Inertial(mass=cm.mass, inertia=cm.moment_inertia,
                                    origin=cmm)

        visuals = None
        if not collision_only:
            visuals=[v.copy(prefix=prefix, scale=scale) for v in self.visuals]

        cpy = Link(
            name='{}{}'.format(prefix, self.name),
            inertial=inertial,
            visuals=visuals,
            collisions=[v.copy(prefix=prefix, scale=scale) for v in self.collisions],
        )
        cpy._collision_mesh = cm
        return cpy


class URDF(URDFType):
    """The top-level URDF specification.

    The URDF encapsulates an articulated object, such as a robot or a gripper.
    It is made of links and joints that tie them together and define their
    relative motions.

    Parameters
    ----------
    name : str
        The name of the URDF.
    links : list of :class:`.Link`
        The links of the URDF.
    joints : list of :class:`.Joint`, optional
        The joints of the URDF.
    transmissions : list of :class:`.Transmission`, optional
        The transmissions of the URDF.
    materials : list of :class:`.Material`, optional
        The materials for the URDF.
    other_xml : str, optional
        A string containing any extra XML for extensions.
    """
    _ATTRIBS = {
        'name': (str, True),
    }
    _ELEMENTS = {
        'links': (Link, True, True),
        'joints': (Joint, False, True),
        'transmissions': (Transmission, False, True),
        'materials': (Material, False, True),
    }
    _TAG = 'robot'

    def __init__(self, name, links, joints=None,
                 transmissions=None, materials=None, other_xml=None):
        if joints is None:
            joints = []
        if transmissions is None:
            transmissions = []
        if materials is None:
            materials = []

        self.name = name
        self.other_xml = other_xml

        # No setters for these
        self._links = list(links)
        self._joints = list(joints)
        self._transmissions = list(transmissions)
        self._materials = list(materials)

        # Set up private helper maps from name to value
        self._link_map = {}
        self._joint_map = {}
        self._transmission_map = {}
        self._material_map = {}

        for x in self._links:
            if x.name in self._link_map:
                raise ValueError('Two links with name {} found'.format(x.name))
            self._link_map[x.name] = x

        for x in self._joints:
            if x.name in self._joint_map:
                raise ValueError('Two joints with name {} '
                                 'found'.format(x.name))
            self._joint_map[x.name] = x

        for x in self._transmissions:
            if x.name in self._transmission_map:
                raise ValueError('Two transmissions with name {} '
                                 'found'.format(x.name))
            self._transmission_map[x.name] = x

        for x in self._materials:
            if x.name in self._material_map:
                raise ValueError('Two materials with name {} '
                                 'found'.format(x.name))
            self._material_map[x.name] = x

        # Synchronize materials between links and top-level set
        self._merge_materials()

        # Validate the joints and transmissions
        actuated_joints = self._validate_joints()
        self._validate_transmissions()

        # Create the link graph and base link/end link sets
        self._G = nx.DiGraph()

        # Add all links
        for link in self.links:
            self._G.add_node(link)

        # Add all edges from CHILDREN TO PARENTS, with joints as their object
        for joint in self.joints:
            parent = self._link_map[joint.parent]
            child = self._link_map[joint.child]
            self._G.add_edge(child, parent, joint=joint)

        # Validate the graph and get the base and end links
        self._base_link, self._end_links = self._validate_graph()

        # Cache the paths to the base link
        self._paths_to_base = nx.shortest_path(
            self._G, target=self._base_link
        )

        self._actuated_joints = self._sort_joints(actuated_joints)

        # Cache the reverse topological order (useful for speeding up FK,
        # as we want to start at the base and work outward to cache
        # computation.
        self._reverse_topo = list(reversed(list(nx.topological_sort(self._G))))

    @property
    def name(self):
        """str : The name of the URDF.
        """
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def links(self):
        """list of :class:`.Link` : The links of the URDF.

        This returns a copy of the links array which cannot be edited
        directly. If you want to add or remove links, use
        the appropriate functions.
        """
        return copy.copy(self._links)

    @property
    def link_map(self):
        """dict : Map from link names to the links themselves.

        This returns a copy of the link map which cannot be edited
        directly. If you want to add or remove links, use
        the appropriate functions.
        """
        return copy.copy(self._link_map)

    @property
    def joints(self):
        """list of :class:`.Joint` : The links of the URDF.

        This returns a copy of the joints array which cannot be edited
        directly. If you want to add or remove joints, use
        the appropriate functions.
        """
        return copy.copy(self._joints)

    @property
    def joint_map(self):
        """dict : Map from joint names to the joints themselves.

        This returns a copy of the joint map which cannot be edited
        directly. If you want to add or remove joints, use
        the appropriate functions.
        """
        return copy.copy(self._joint_map)

    @property
    def transmissions(self):
        """list of :class:`.Transmission` : The transmissions of the URDF.

        This returns a copy of the transmissions array which cannot be edited
        directly. If you want to add or remove transmissions, use
        the appropriate functions.
        """
        return copy.copy(self._transmissions)

    @property
    def transmission_map(self):
        """dict : Map from transmission names to the transmissions themselves.

        This returns a copy of the transmission map which cannot be edited
        directly. If you want to add or remove transmissions, use
        the appropriate functions.
        """
        return copy.copy(self._transmission_map)

    @property
    def materials(self):
        """list of :class:`.Material` : The materials of the URDF.

        This returns a copy of the materials array which cannot be edited
        directly. If you want to add or remove materials, use
        the appropriate functions.
        """
        return copy.copy(self._materials)

    @property
    def material_map(self):
        """dict : Map from material names to the materials themselves.

        This returns a copy of the material map which cannot be edited
        directly. If you want to add or remove materials, use
        the appropriate functions.
        """
        return copy.copy(self._material_map)

    @property
    def other_xml(self):
        """str : Any extra XML that belongs with the URDF.
        """
        return self._other_xml

    @other_xml.setter
    def other_xml(self, value):
        self._other_xml = value

    @property
    def actuated_joints(self):
        """list of :class:`.Joint` : The joints that are independently
        actuated.

        This excludes mimic joints and fixed joints. The joints are listed
        in topological order, starting from the base-most joint.
        """
        return self._actuated_joints

    @property
    def actuated_joint_names(self):
        """list of :class:`.Joint` : The names of joints that are independently
        actuated.

        This excludes mimic joints and fixed joints. The joints are listed
        in topological order, starting from the base-most joint.
        """
        return [j.name for j in self._actuated_joints]

    def cfg_to_vector(self, cfg):
        """Convert a configuration dictionary into a configuration vector.

        Parameters
        ----------
        cfg : dict or None
            The configuration value.

        Returns
        -------
        vec : (n,) float
            The configuration vector, or None if no actuated joints present.
        """
        if cfg is None:
            if len(self.actuated_joints) > 0:
                return np.zeros(len(self.actuated_joints))
            else:
                return None
        elif isinstance(cfg, (list, tuple, np.ndarray)):
            return np.asanyarray(cfg)
        elif isinstance(cfg, dict):
            vec = np.zeros(len(self.actuated_joints))
            for i, jn in enumerate(self.actuated_joint_names):
                if jn in cfg:
                    vec[i] = cfg[jn]
            return vec
        else:
            raise ValueError('Invalid configuration: {}'.format(cfg))

    @property
    def base_link(self):
        """:class:`.Link`: The base link for the URDF.

        The base link is the single link that has no parent.
        """
        return self._base_link

    @property
    def end_links(self):
        """list of :class:`.Link`: The end links for the URDF.

        The end links are the links that have no children.
        """
        return self._end_links

    @property
    def joint_limit_cfgs(self):
        """tuple of dict : The lower-bound and upper-bound joint configuration
        maps.

        The first map is the lower-bound map, which maps limited joints to
        their lower joint limits.
        The second map is the upper-bound map, which maps limited joints to
        their upper joint limits.
        """
        lb = {}
        ub = {}
        for joint in self.actuated_joints:
            if joint.limit is not None:
                if joint.limit.lower is not None:
                    lb[joint] = joint.limit.lower
                if joint.limit.upper is not None:
                    ub[joint] = joint.limit.upper
        return (lb, ub)

    @property
    def joint_limits(self):
        """(n,2) float : A lower and upper limit for each joint.
        """
        limits = []
        for joint in self.actuated_joints:
            limit = [-np.infty, np.infty]
            if joint.limit is not None:
                if joint.limit.lower is not None:
                    limit[0] = joint.limit.lower
                if joint.limit.upper is not None:
                    limit[1] = joint.limit.upper
            limits.append(limit)
        return np.array(limits)

    def link_fk(self, cfg=None, link=None, links=None, use_names=False):
        """Computes the poses of the URDF's links via forward kinematics.

        Parameters
        ----------
        cfg : dict or (n), float
            A map from joints or joint names to configuration values for
            each joint, or a list containing a value for each actuated joint
            in sorted order from the base link.
            If not specified, all joints are assumed to be in their default
            configurations.
        link : str or :class:`.Link`
            A single link or link name to return a pose for.
        links : list of str or list of :class:`.Link`
            The links or names of links to perform forward kinematics on.
            Only these links will be in the returned map. If neither
            link nor links are specified all links are returned.
        use_names : bool
            If True, the returned dictionary will have keys that are string
            link names rather than the links themselves.

        Returns
        -------
        fk : dict or (4,4) float
            A map from links to 4x4 homogenous transform matrices that
            position them relative to the base link's frame, or a single
            4x4 matrix if ``link`` is specified.
        """
        # Process config value
        joint_cfg = self._process_cfg(cfg)

        # Process link set
        link_set = set()
        if link is not None:
            if isinstance(link, six.string_types):
                link_set.add(self._link_map[link])
            elif isinstance(link, Link):
                link_set.add(link)
        elif links is not None:
            for lnk in links:
                if isinstance(lnk, six.string_types):
                    link_set.add(self._link_map[lnk])
                elif isinstance(lnk, Link):
                    link_set.add(lnk)
                else:
                    raise TypeError('Got object of type {} in links list'
                                    .format(type(lnk)))
        else:
            link_set = self.links

        # Compute forward kinematics in reverse topological order
        fk = OrderedDict()
        for lnk in self._reverse_topo:
            if lnk not in link_set:
                continue
            pose = np.eye(4, dtype=np.float64)
            path = self._paths_to_base[lnk]
            for i in range(len(path) - 1):
                child = path[i]
                parent = path[i + 1]
                joint = self._G.get_edge_data(child, parent)['joint']

                cfg = None
                if joint.mimic is not None:
                    mimic_joint = self._joint_map[joint.mimic.joint]
                    if mimic_joint in joint_cfg:
                        cfg = joint_cfg[mimic_joint]
                        cfg = joint.mimic.multiplier * cfg + joint.mimic.offset
                elif joint in joint_cfg:
                    cfg = joint_cfg[joint]
                pose = joint.get_child_pose(cfg).dot(pose)

                # Check existing FK to see if we can exit early
                if parent in fk:
                    pose = fk[parent].dot(pose)
                    break
            fk[lnk] = pose

        if link:
            if isinstance(link, six.string_types):
                return fk[self._link_map[link]]
            else:
                return fk[link]
        if use_names:
            return {ell.name: fk[ell] for ell in fk}
        return fk

    def link_fk_batch(self, cfgs=None, link=None, links=None, use_names=False):
        """Computes the poses of the URDF's links via forward kinematics in a batch.

        Parameters
        ----------
        cfgs : dict, list of dict, or (n,m), float
            One of the following: (A) a map from joints or joint names to vectors
            of joint configuration values, (B) a list of maps from joints or joint names
            to single configuration values, or (C) a list of ``n`` configuration vectors,
            each of which has a vector with an entry for each actuated joint.
        link : str or :class:`.Link`
            A single link or link name to return a pose for.
        links : list of str or list of :class:`.Link`
            The links or names of links to perform forward kinematics on.
            Only these links will be in the returned map. If neither
            link nor links are specified all links are returned.
        use_names : bool
            If True, the returned dictionary will have keys that are string
            link names rather than the links themselves.

        Returns
        -------
        fk : dict or (n,4,4) float
            A map from links to a (n,4,4) vector of homogenous transform matrices that
            position the links relative to the base link's frame, or a single
            nx4x4 matrix if ``link`` is specified.
        """
        joint_cfgs, n_cfgs = self._process_cfgs(cfgs)

        # Process link set
        link_set = set()
        if link is not None:
            if isinstance(link, six.string_types):
                link_set.add(self._link_map[link])
            elif isinstance(link, Link):
                link_set.add(link)
        elif links is not None:
            for lnk in links:
                if isinstance(lnk, six.string_types):
                    link_set.add(self._link_map[lnk])
                elif isinstance(lnk, Link):
                    link_set.add(lnk)
                else:
                    raise TypeError('Got object of type {} in links list'
                                    .format(type(lnk)))
        else:
            link_set = self.links

        # Compute FK mapping each link to a vector of matrices, one matrix per cfg
        fk = OrderedDict()
        for lnk in self._reverse_topo:
            if lnk not in link_set:
                continue
            poses = np.tile(np.eye(4, dtype=np.float64), (n_cfgs, 1, 1))
            path = self._paths_to_base[lnk]
            for i in range(len(path) - 1):
                child = path[i]
                parent = path[i + 1]
                joint = self._G.get_edge_data(child, parent)['joint']

                cfg_vals = None
                if joint.mimic is not None:
                    mimic_joint = self._joint_map[joint.mimic.joint]
                    if mimic_joint in joint_cfgs:
                        cfg_vals = joint_cfgs[mimic_joint]
                        cfg_vals = joint.mimic.multiplier * cfg_vals + joint.mimic.offset
                elif joint in joint_cfgs:
                    cfg_vals = joint_cfgs[joint]
                poses = np.matmul(joint.get_child_poses(cfg_vals, n_cfgs), poses)

                if parent in fk:
                    poses = np.matmul(fk[parent], poses)
                    break
            fk[lnk] = poses

        if link:
            if isinstance(link, six.string_types):
                return fk[self._link_map[link]]
            else:
                return fk[link]
        if use_names:
            return {ell.name: fk[ell] for ell in fk}
        return fk

    def visual_mesh_fk(self, cfg=None, links=None):
        """Computes the poses of the URDF's visual trimeshes using fk.

        Parameters
        ----------
        cfg : dict or (n), float
            A map from joints or joint names to configuration values for
            each joint, or a list containing a value for each actuated joint
            in sorted order from the base link.
            If not specified, all joints are assumed to be in their default
            configurations.
        links : list of str or list of :class:`.Link`
            The links or names of links to perform forward kinematics on.
            Only trimeshes from these links will be in the returned map.
            If not specified, all links are returned.

        Returns
        -------
        fk : dict
            A map from :class:`~trimesh.base.Trimesh` objects that are
            part of the visual geometry of the specified links to the
            4x4 homogenous transform matrices that position them relative
            to the base link's frame.
        """
        lfk = self.link_fk(cfg=cfg, links=links)

        fk = OrderedDict()
        for link in lfk:
            for visual in link.visuals:
                geometryMesh = visual.geometry.meshes
                pose = lfk[link].dot(visual.origin)
                if visual.geometry.mesh is not None:
                    if visual.geometry.mesh.scale is not None:
                        S = np.eye(4, dtype=np.float64)
                        S[:3,:3] = np.diag(visual.geometry.mesh.scale)
                        pose = pose.dot(S)
                fk[geometryMesh] = pose
        return fk

    def collision_mesh_fk(self, cfg=None, links=None):
        """Computes the poses of the URDF's collision trimeshes using fk.

        Parameters
        ----------
        cfg : dict or (n), float
            A map from joints or joint names to configuration values for
            each joint, or a list containing a value for each actuated joint
            in sorted order from the base link.
            If not specified, all joints are assumed to be in their default
            configurations.
        links : list of str or list of :class:`.Link`
            The links or names of links to perform forward kinematics on.
            Only trimeshes from these links will be in the returned map.
            If not specified, all links are returned.

        Returns
        -------
        fk : dict
            A map from :class:`~trimesh.base.Trimesh` objects that are
            part of the collision geometry of the specified links to the
            4x4 homogenous transform matrices that position them relative
            to the base link's frame.
        """
        lfk = self.link_fk(cfg=cfg, links=links)

        fk = OrderedDict()
        for link in lfk:
            pose = lfk[link]
            cm = link.collision_mesh
            if cm is not None:
                fk[cm] = pose
        return fk

    def collision_mesh_fk_batch(self, cfgs=None, links=None):
        """Computes the poses of the URDF's collision trimeshes using fk.

        Parameters
        ----------
        cfgs : dict, list of dict, or (n,m), float
            One of the following: (A) a map from joints or joint names to vectors
            of joint configuration values, (B) a list of maps from joints or joint names
            to single configuration values, or (C) a list of ``n`` configuration vectors,
            each of which has a vector with an entry for each actuated joint.
        links : list of str or list of :class:`.Link`
            The links or names of links to perform forward kinematics on.
            Only trimeshes from these links will be in the returned map.
            If not specified, all links are returned.

        Returns
        -------
        fk : dict
            A map from :class:`~trimesh.base.Trimesh` objects that are
            part of the collision geometry of the specified links to the
            4x4 homogenous transform matrices that position them relative
            to the base link's frame.
        """
        lfk = self.link_fk_batch(cfgs=cfgs, links=links)

        fk = OrderedDict()
        for link in lfk:
            poses = lfk[link]
            cm = link.collision_mesh
            if cm is not None:
                fk[cm] = poses
        return fk

    def copy(self, name=None, prefix='', scale=None, collision_only=False):
        """Make a deep copy of the URDF.

        Parameters
        ----------
        name : str, optional
            A name for the new URDF. If not specified, ``self.name`` is used.
        prefix : str, optional
            A prefix to apply to all names except for the base URDF name.
        scale : float or (3,) float, optional
            A scale to apply to the URDF.
        collision_only : bool, optional
            If True, all visual geometry is redirected to the collision geometry.

        Returns
        -------
        copy : :class:`.URDF`
            The copied URDF.
        """
        return URDF(
            name = (name if name else self.name),
            links=[v.copy(prefix, scale, collision_only) for v in self.links],
            joints=[v.copy(prefix, scale) for v in self.joints],
            transmissions=[v.copy(prefix, scale) for v in self.transmissions],
            materials=[v.copy(prefix, scale) for v in self.materials],
            other_xml=self.other_xml
        )

    def save(self, file_obj):
        """Save this URDF to a file.

        Parameters
        ----------
        file_obj : str or file-like object
            The file to save the URDF to. Should be the path to the
            ``.urdf`` XML file. Any paths in the URDF should be specified
            as relative paths to the ``.urdf`` file instead of as ROS
            resources.

        Returns
        -------
        urdf : :class:`.URDF`
            The parsed URDF.
        """
        if isinstance(file_obj, six.string_types):
            path, _ = os.path.split(file_obj)
        else:
            path, _ = os.path.split(os.path.realpath(file_obj.name))

        node = self._to_xml(None, path)
        tree = ET.ElementTree(node)
        tree.write(file_obj, pretty_print=True,
                   xml_declaration=True, encoding='utf-8')

    def join(self, other, link, origin=None, name=None, prefix=''):
        """Join another URDF to this one by rigidly fixturing the two at a link.

        Parameters
        ----------
        other : :class:.`URDF`
            Another URDF to fuze to this one.
        link : :class:`.Link` or str
            The link of this URDF to attach the other URDF to.
        origin : (4,4) float, optional
            The location in this URDF's link frame to attach the base link of the other
            URDF at.
        name : str, optional
            A name for the new URDF.
        prefix : str, optional
            If specified, all joints and links from the (other) mesh will be pre-fixed
            with this value to avoid name clashes.

        Returns
        -------
        :class:`.URDF`
            The new URDF.
        """
        myself = self.copy()
        other = other.copy(prefix=prefix)

        # Validate
        link_names = set(myself.link_map.keys())
        other_link_names = set(other.link_map.keys())
        if len(link_names.intersection(other_link_names)) > 0:
            raise ValueError('Cannot merge two URDFs with shared link names')

        joint_names = set(myself.joint_map.keys())
        other_joint_names = set(other.joint_map.keys())
        if len(joint_names.intersection(other_joint_names)) > 0:
            raise ValueError('Cannot merge two URDFs with shared joint names')

        links = myself.links + other.links
        joints = myself.joints + other.joints
        transmissions = myself.transmissions + other.transmissions
        materials = myself.materials + other.materials

        if name is None:
            name = self.name

        # Create joint that links the two rigidly
        joints.append(Joint(
            name='{}_join_{}{}_joint'.format(self.name, prefix, other.name),
            joint_type='fixed',
            parent=link if isinstance(link, str) else link.name,
            child=other.base_link.name,
            origin=origin
        ))

        return URDF(name=name, links=links, joints=joints, transmissions=transmissions,
                    materials=materials)

    def _merge_materials(self):
        """Merge the top-level material set with the link materials.
        """
        for link in self.links:
            for v in link.visuals:
                if v.material is None:
                    continue
                if v.material.name in self.material_map:
                    v.material = self._material_map[v.material.name]
                else:
                    self._materials.append(v.material)
                    self._material_map[v.material.name] = v.material

    @staticmethod
    def load(file_obj):
        """Load a URDF from a file.

        Parameters
        ----------
        file_obj : str or file-like object
            The file to load the URDF from. Should be the path to the
            ``.urdf`` XML file. Any paths in the URDF should be specified
            as relative paths to the ``.urdf`` file instead of as ROS
            resources.

        Returns
        -------
        urdf : :class:`.URDF`
            The parsed URDF.
        """
        if isinstance(file_obj, six.string_types):
            if os.path.isfile(file_obj):
                parser = ET.XMLParser(remove_comments=True,
                                      remove_blank_text=True)
                tree = ET.parse(file_obj, parser=parser)
                path, _ = os.path.split(file_obj)
            else:
                raise ValueError('{} is not a file'.format(file_obj))
        else:
            parser = ET.XMLParser(remove_comments=True, remove_blank_text=True)
            tree = ET.parse(file_obj, parser=parser)
            path, _ = os.path.split(file_obj.name)

        node = tree.getroot()
        return URDF._from_xml(node, path)

    def _validate_joints(self):
        """Raise an exception of any joints are invalidly specified.

        Checks for the following:

        - Joint parents are valid link names.
        - Joint children are valid link names that aren't the same as parent.
        - Joint mimics have valid joint names that aren't the same joint.

        Returns
        -------
        actuated_joints : list of :class:`.Joint`
            The joints in the model that are independently controllable.
        """
        actuated_joints = []
        for joint in self.joints:
            if joint.parent not in self._link_map:
                raise ValueError('Joint {} has invalid parent link name {}'
                                 .format(joint.name, joint.parent))
            if joint.child not in self._link_map:
                raise ValueError('Joint {} has invalid child link name {}'
                                 .format(joint.name, joint.child))
            if joint.child == joint.parent:
                raise ValueError('Joint {} has matching parent and child'
                                 .format(joint.name))
            if joint.mimic is not None:
                if joint.mimic.joint not in self._joint_map:
                    raise ValueError(
                        'Joint {} has an invalid mimic joint name {}'
                        .format(joint.name, joint.mimic.joint)
                    )
                if joint.mimic.joint == joint.name:
                    raise ValueError(
                        'Joint {} set up to mimic itself'
                        .format(joint.mimic.joint)
                    )
            elif joint.joint_type != 'fixed':
                actuated_joints.append(joint)

        # Do a depth-first search
        return actuated_joints

    def _sort_joints(self, joints):
        """Sort joints by ascending distance from the base link (topologically).

        Parameters
        ----------
        joints : list of :class:`.Joint`
            The joints to sort.

        Returns
        -------
        joints : list of :class:`.Joint`
            The sorted joints.
        """
        lens = []
        for joint in joints:
            child_link = self._link_map[joint.child]
            lens.append(len(self._paths_to_base[child_link]))
        order = np.argsort(lens)
        return np.array(joints)[order].tolist()

    def _validate_transmissions(self):
        """Raise an exception of any transmissions are invalidly specified.

        Checks for the following:

        - Transmission joints have valid joint names.
        """
        for t in self.transmissions:
            for joint in t.joints:
                if joint.name not in self._joint_map:
                    raise ValueError('Transmission {} has invalid joint name '
                                     '{}'.format(t.name, joint.name))

    def _validate_graph(self):
        """Raise an exception if the link-joint structure is invalid.

        Checks for the following:

        - The graph is connected in the undirected sense.
        - The graph is acyclic in the directed sense.
        - The graph has only one base link.

        Returns
        -------
        base_link : :class:`.Link`
            The base link of the URDF.
        end_links : list of :class:`.Link`
            The end links of the URDF.
        """

        # Check that the link graph is weakly connected
        if not nx.is_weakly_connected(self._G):
            link_clusters = []
            for cc in nx.weakly_connected_components(self._G):
                cluster = []
                for n in cc:
                    cluster.append(n.name)
                link_clusters.append(cluster)
            message = ('Links are not all connected. '
                       'Connected components are:')
            for lc in link_clusters:
                message += '\n\t'
                for n in lc:
                    message += ' {}'.format(n)
            raise ValueError(message)

        # Check that link graph is acyclic
        if not nx.is_directed_acyclic_graph(self._G):
            raise ValueError('There are cycles in the link graph')

        # Ensure that there is exactly one base link, which has no parent
        base_link = None
        end_links = []
        for n in self._G:
            if len(nx.descendants(self._G, n)) == 0:
                if base_link is None:
                    base_link = n
                else:
                    raise ValueError('Links {} and {} are both base links!'
                                     .format(n.name, base_link.name))
            if len(nx.ancestors(self._G, n)) == 0:
                end_links.append(n)
        return base_link, end_links

    def _process_cfg(self, cfg):
        """Process a joint configuration spec into a dictionary mapping
        joints to configuration values.
        """
        joint_cfg = {}
        if cfg is None:
            return joint_cfg
        if isinstance(cfg, dict):
            for joint in cfg:
                if isinstance(joint, six.string_types):
                    joint_cfg[self._joint_map[joint]] = cfg[joint]
                elif isinstance(joint, Joint):
                    joint_cfg[joint] = cfg[joint]
        elif isinstance(cfg, (list, tuple, np.ndarray)):
            if len(cfg) != len(self.actuated_joints):
                raise ValueError('Cfg must have same length as actuated joints '
                                 'if specified as a numerical array')
            for joint, value in zip(self.actuated_joints, cfg):
                joint_cfg[joint] = value
        else:
            raise TypeError('Invalid type for config')
        return joint_cfg

    def _process_cfgs(self, cfgs):
        """Process a list of joint configurations into a dictionary mapping joints to
        configuration values.

        This should result in a dict mapping each joint to a list of cfg values, one
        per joint.
        """
        joint_cfg = {j : [] for j in self.actuated_joints}
        n_cfgs = None
        if isinstance(cfgs, dict):
            for joint in cfgs:
                if isinstance(joint, six.string_types):
                    joint_cfg[self._joint_map[joint]] = cfgs[joint]
                else:
                    joint_cfg[joint] = cfgs[joint]
                if n_cfgs is None:
                    n_cfgs = len(cfgs[joint])
        elif isinstance(cfgs, (list, tuple, np.ndarray)):
            n_cfgs = len(cfgs)
            if isinstance(cfgs[0], dict):
                for cfg in cfgs:
                    for joint in cfg:
                        if isinstance(joint, six.string_types):
                            joint_cfg[self._joint_map[joint]].append(cfg[joint])
                        else:
                            joint_cfg[joint].append(cfg[joint])
            elif cfgs[0] is None:
                pass
            else:
                cfgs = np.asanyarray(cfgs, dtype=np.float64)
                for i, j in enumerate(self.actuated_joints):
                    joint_cfg[j] = cfgs[:,i]
        else:
            raise ValueError('Incorrectly formatted config array')

        for j in joint_cfg:
            if len(joint_cfg[j]) == 0:
                joint_cfg[j] = None
            elif len(joint_cfg[j]) != n_cfgs:
                raise ValueError('Inconsistent number of configurations for joints')

        return joint_cfg, n_cfgs

    @classmethod
    def _from_xml(cls, node, path):
        valid_tags = set(['joint', 'link', 'transmission', 'material'])
        kwargs = cls._parse(node, path)

        extra_xml_node = ET.Element('extra')
        for child in node:
            if child.tag not in valid_tags:
                extra_xml_node.append(child)

        data = ET.tostring(extra_xml_node)
        kwargs['other_xml'] = data
        return URDF(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(path)
        if self.other_xml:
            extra_tree = ET.fromstring(self.other_xml)
            for child in extra_tree:
                node.append(child)
        return node
