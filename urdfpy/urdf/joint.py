from urdfpy.urdf.base import URDFType


class JointCalibration(URDFType):
    """The reference positions of the joint.

    Parameters
    ----------
    rising : float, optional
        When the joint moves in a positive direction, this position will
        trigger a rising edge.
    falling :
        When the joint moves in a positive direction, this position will
        trigger a falling edge.
    """
    _ATTRIBS = {
        'rising': (float, False),
        'falling': (float, False)
    }
    _TAG = 'calibration'

    def __init__(self, rising=None, falling=None):
        self.rising = rising
        self.falling = falling

    @property
    def rising(self):
        """float : description.
        """
        return self._rising

    @rising.setter
    def rising(self, value):
        if value is not None:
            value = float(value)
        self._rising = value

    @property
    def falling(self):
        """float : description.
        """
        return self._falling

    @falling.setter
    def falling(self, value):
        if value is not None:
            value = float(value)
        self._falling = value

    def copy(self, prefix='', scale=None):
        """Create a deep copy of the visual with the prefix applied to all names.

        Parameters
        ----------
        prefix : str
            A prefix to apply to all joint and link names.

        Returns
        -------
        :class:`.JointCalibration`
            A deep copy of the visual.
        """
        return JointCalibration(
            rising=self.rising,
            falling=self.falling,
        )


class JointDynamics(URDFType):
    """The dynamic properties of the joint.

    Parameters
    ----------
    damping : float
        The damping value of the joint (Ns/m for prismatic joints,
        Nms/rad for revolute).
    friction : float
        The static friction value of the joint (N for prismatic joints,
        Nm for revolute).
    """
    _ATTRIBS = {
        'damping': (float, False),
        'friction': (float, False),
    }
    _TAG = 'dynamics'

    def __init__(self, damping, friction):
        self.damping = damping
        self.friction = friction

    @property
    def damping(self):
        """float : The damping value of the joint.
        """
        return self._damping

    @damping.setter
    def damping(self, value):
        if value is not None:
            value = float(value)
        self._damping = value

    @property
    def friction(self):
        """float : The static friction value of the joint.
        """
        return self._friction

    @friction.setter
    def friction(self, value):
        if value is not None:
            value = float(value)
        self._friction = value

    def copy(self, prefix='', scale=None):
        """Create a deep copy of the visual with the prefix applied to all names.

        Parameters
        ----------
        prefix : str
            A prefix to apply to all joint and link names.

        Returns
        -------
        :class:`.JointDynamics`
            A deep copy of the visual.
        """
        return JointDynamics(
            damping=self.damping,
            friction=self.friction,
        )


class JointLimit(URDFType):
    """The limits of the joint.

    Parameters
    ----------
    effort : float
        The maximum joint effort (N for prismatic joints, Nm for revolute).
    velocity : float
        The maximum joint velocity (m/s for prismatic joints, rad/s for
        revolute).
    lower : float, optional
        The lower joint limit (m for prismatic joints, rad for revolute).
    upper : float, optional
        The upper joint limit (m for prismatic joints, rad for revolute).
    """

    _ATTRIBS = {
        'effort': (float, True),
        'velocity': (float, True),
        'lower': (float, False),
        'upper': (float, False),
    }
    _TAG = 'limit'

    def __init__(self, effort, velocity, lower=None, upper=None):
        self.effort = effort
        self.velocity = velocity
        self.lower = lower
        self.upper = upper

    @property
    def effort(self):
        """float : The maximum joint effort.
        """
        return self._effort

    @effort.setter
    def effort(self, value):
        self._effort = float(value)

    @property
    def velocity(self):
        """float : The maximum joint velocity.
        """
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        self._velocity = float(value)

    @property
    def lower(self):
        """float : The lower joint limit.
        """
        return self._lower

    @lower.setter
    def lower(self, value):
        if value is not None:
            value = float(value)
        self._lower = value

    @property
    def upper(self):
        """float : The upper joint limit.
        """
        return self._upper

    @upper.setter
    def upper(self, value):
        if value is not None:
            value = float(value)
        self._upper = value

    def copy(self, prefix='', scale=None):
        """Create a deep copy of the visual with the prefix applied to all names.

        Parameters
        ----------
        prefix : str
            A prefix to apply to all joint and link names.

        Returns
        -------
        :class:`.JointLimit`
            A deep copy of the visual.
        """
        return JointLimit(
            effort=self.effort,
            velocity=self.velocity,
            lower=self.lower,
            upper=self.upper,
        )


class JointMimic(URDFType):
    """A mimicry tag for a joint, which forces its configuration to
    mimic another joint's.

    This joint's configuration value is set equal to
    ``multiplier * other_joint_cfg + offset``.

    Parameters
    ----------
    joint : str
        The name of the joint to mimic.
    multiplier : float
        The joint configuration multiplier. Defaults to 1.0.
    offset : float, optional
        The joint configuration offset. Defaults to 0.0.
    """
    _ATTRIBS = {
        'joint': (str, True),
        'multiplier': (float, False),
        'offset': (float, False),
    }
    _TAG = 'mimic'

    def __init__(self, joint, multiplier=None, offset=None):
        self.joint = joint
        self.multiplier = multiplier
        self.offset = offset

    @property
    def joint(self):
        """float : The name of the joint to mimic.
        """
        return self._joint

    @joint.setter
    def joint(self, value):
        self._joint = str(value)

    @property
    def multiplier(self):
        """float : The multiplier for the joint configuration.
        """
        return self._multiplier

    @multiplier.setter
    def multiplier(self, value):
        if value is not None:
            value = float(value)
        else:
            value = 1.0
        self._multiplier = value

    @property
    def offset(self):
        """float : The offset for the joint configuration
        """
        return self._offset

    @offset.setter
    def offset(self, value):
        if value is not None:
            value = float(value)
        else:
            value = 0.0
        self._offset = value

    def copy(self, prefix='', scale=None):
        """Create a deep copy of the joint mimic with the prefix applied to all names.

        Parameters
        ----------
        prefix : str
            A prefix to apply to all joint and link names.

        Returns
        -------
        :class:`.JointMimic`
            A deep copy of the joint mimic.
        """
        return JointMimic(
            joint='{}{}'.format(prefix, self.joint),
            multiplier=self.multiplier,
            offset=self.offset
        )


class SafetyController(URDFType):
    """A controller for joint movement safety.

    Parameters
    ----------
    k_velocity : float
        An attribute specifying the relation between the effort and velocity
        limits.
    k_position : float, optional
        An attribute specifying the relation between the position and velocity
        limits. Defaults to 0.0.
    soft_lower_limit : float, optional
        The lower joint boundary where the safety controller kicks in.
        Defaults to 0.0.
    soft_upper_limit : float, optional
        The upper joint boundary where the safety controller kicks in.
        Defaults to 0.0.
    """
    _ATTRIBS = {
        'k_velocity': (float, True),
        'k_position': (float, False),
        'soft_lower_limit': (float, False),
        'soft_upper_limit': (float, False),
    }
    _TAG = 'safety_controller'

    def __init__(self, k_velocity, k_position=None, soft_lower_limit=None,
                 soft_upper_limit=None):
        self.k_velocity = k_velocity
        self.k_position = k_position
        self.soft_lower_limit = soft_lower_limit
        self.soft_upper_limit = soft_upper_limit

    @property
    def soft_lower_limit(self):
        """float : The soft lower limit where the safety controller kicks in.
        """
        return self._soft_lower_limit

    @soft_lower_limit.setter
    def soft_lower_limit(self, value):
        if value is not None:
            value = float(value)
        else:
            value = 0.0
        self._soft_lower_limit = value

    @property
    def soft_upper_limit(self):
        """float : The soft upper limit where the safety controller kicks in.
        """
        return self._soft_upper_limit

    @soft_upper_limit.setter
    def soft_upper_limit(self, value):
        if value is not None:
            value = float(value)
        else:
            value = 0.0
        self._soft_upper_limit = value

    @property
    def k_position(self):
        """float : A relation between the position and velocity limits.
        """
        return self._k_position

    @k_position.setter
    def k_position(self, value):
        if value is not None:
            value = float(value)
        else:
            value = 0.0
        self._k_position = value

    @property
    def k_velocity(self):
        """float : A relation between the effort and velocity limits.
        """
        return self._k_velocity

    @k_velocity.setter
    def k_velocity(self, value):
        self._k_velocity = float(value)

    def copy(self, prefix='', scale=None):
        """Create a deep copy of the visual with the prefix applied to all names.

        Parameters
        ----------
        prefix : str
            A prefix to apply to all joint and link names.

        Returns
        -------
        :class:`.SafetyController`
            A deep copy of the visual.
        """
        return SafetyController(
            k_velocity=self.k_velocity,
            k_position=self.k_position,
            soft_lower_limit=self.soft_lower_limit,
            soft_upper_limit=self.soft_upper_limit,
        )

