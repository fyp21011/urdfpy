from lxml import etree as ET

from urdfpy.urdf.base import URDFType

class Actuator(URDFType):
    """An actuator.

    Parameters
    ----------
    name : str
        The name of this actuator.
    mechanicalReduction : str, optional
        A specifier for the mechanical reduction at the joint/actuator
        transmission.
    hardwareInterfaces : list of str, optional
        The supported hardware interfaces to the actuator.
    """
    _ATTRIBS = {
        'name': (str, True),
    }
    _TAG = 'actuator'

    def __init__(self, name, mechanicalReduction=None,
                 hardwareInterfaces=None):
        self.name = name
        self.mechanicalReduction = mechanicalReduction
        self.hardwareInterfaces = hardwareInterfaces

    @property
    def name(self):
        """str : The name of this actuator.
        """
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def mechanicalReduction(self):
        """str : A specifier for the type of mechanical reduction.
        """
        return self._mechanicalReduction

    @mechanicalReduction.setter
    def mechanicalReduction(self, value):
        if value is not None:
            value = str(value)
        self._mechanicalReduction = value

    @property
    def hardwareInterfaces(self):
        """list of str : The supported hardware interfaces.
        """
        return self._hardwareInterfaces

    @hardwareInterfaces.setter
    def hardwareInterfaces(self, value):
        if value is None:
            value = []
        else:
            value = list(value)
            for i, v in enumerate(value):
                value[i] = str(v)
        self._hardwareInterfaces = value

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        mr = node.find('mechanicalReduction')
        if mr is not None:
            mr = float(mr.text)
        kwargs['mechanicalReduction'] = mr
        hi = node.findall('hardwareInterface')
        if len(hi) > 0:
            hi = [h.text for h in hi]
        kwargs['hardwareInterfaces'] = hi
        return Actuator(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(path)
        if self.mechanicalReduction is not None:
            mr = ET.Element('mechanicalReduction')
            mr.text = str(self.mechanicalReduction)
            node.append(mr)
        if len(self.hardwareInterfaces) > 0:
            for hi in self.hardwareInterfaces:
                h = ET.Element('hardwareInterface')
                h.text = hi
                node.append(h)
        return node

    def copy(self, prefix='', scale=None):
        """Create a deep copy of the visual with the prefix applied to all names.

        Parameters
        ----------
        prefix : str
            A prefix to apply to all joint and link names.

        Returns
        -------
        :class:`.Actuator`
            A deep copy of the visual.
        """
        return Actuator(
            name='{}{}'.format(prefix, self.name),
            mechanicalReduction=self.mechanicalReduction,
            hardwareInterfaces=self.hardwareInterfaces.copy(),
        )


class TransmissionJoint(URDFType):
    """A transmission joint specification.

    Parameters
    ----------
    name : str
        The name of this actuator.
    hardwareInterfaces : list of str, optional
        The supported hardware interfaces to the actuator.
    """
    _ATTRIBS = {
        'name': (str, True),
    }
    _TAG = 'joint'

    def __init__(self, name, hardwareInterfaces):
        self.name = name
        self.hardwareInterfaces = hardwareInterfaces

    @property
    def name(self):
        """str : The name of this transmission joint.
        """
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def hardwareInterfaces(self):
        """list of str : The supported hardware interfaces.
        """
        return self._hardwareInterfaces

    @hardwareInterfaces.setter
    def hardwareInterfaces(self, value):
        if value is None:
            value = []
        else:
            value = list(value)
            for i, v in enumerate(value):
                value[i] = str(v)
        self._hardwareInterfaces = value

    @classmethod
    def _from_xml(cls, node, path):
        kwargs = cls._parse(node, path)
        hi = node.findall('hardwareInterface')
        if len(hi) > 0:
            hi = [h.text for h in hi]
        kwargs['hardwareInterfaces'] = hi
        return TransmissionJoint(**kwargs)

    def _to_xml(self, parent, path):
        node = self._unparse(path)
        if len(self.hardwareInterfaces) > 0:
            for hi in self.hardwareInterfaces:
                h = ET.Element('hardwareInterface')
                h.text = hi
                node.append(h)
        return node

    def copy(self, prefix='', scale=None):
        """Create a deep copy with the prefix applied to all names.

        Parameters
        ----------
        prefix : str
            A prefix to apply to all names.

        Returns
        -------
        :class:`.TransmissionJoint`
            A deep copy.
        """
        return TransmissionJoint(
            name='{}{}'.format(prefix, self.name),
            hardwareInterfaces=self.hardwareInterfaces.copy(),
        )

