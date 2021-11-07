from lxml import etree as ET
import numpy as np

class URDFType(object):
    """Abstract base class for all URDF types.

    This has useful class methods for automatic parsing/unparsing
    of XML trees.

    There are three overridable class variables:

    - ``_ATTRIBS`` - This is a dictionary mapping attribute names to a tuple,
      ``(type, required)`` where ``type`` is the Python type for the
      attribute and ``required`` is a boolean stating whether the attribute
      is required to be present.
    - ``_ELEMENTS`` - This is a dictionary mapping element names to a tuple,
      ``(type, required, multiple)`` where ``type`` is the Python type for the
      element, ``required`` is a boolean stating whether the element
      is required to be present, and ``multiple`` is a boolean indicating
      whether multiple elements of this type could be present.
      Elements are child nodes in the XML tree, and their type must be a
      subclass of :class:`.URDFType`.
    - ``_TAG`` - This is a string that represents the XML tag for the node
      containing this type of object.
    """
    _ATTRIBS = {}   # Map from attrib name to (type, required)
    _ELEMENTS = {}  # Map from element name to (type, required, multiple)
    _TAG = ''       # XML tag for this element

    def __init__(self):
        pass

    @classmethod
    def _parse_attrib(cls, val_type, val):
        """Parse an XML attribute into a python value.

        Parameters
        ----------
        val_type : :class:`type`
            The type of value to create.
        val : :class:`object`
            The value to parse.

        Returns
        -------
        val : :class:`object`
            The parsed attribute.
        """
        if val_type == np.ndarray:
            val = np.fromstring(val, sep=' ')
        else:
            val = val_type(val)
        return val

    @classmethod
    def _parse_simple_attribs(cls, node):
        """Parse all attributes in the _ATTRIBS array for this class.

        Parameters
        ----------
        node : :class:`lxml.etree.Element`
            The node to parse attributes for.

        Returns
        -------
        kwargs : dict
            Map from attribute name to value. If the attribute is not
            required and is not present, that attribute's name will map to
            ``None``.
        """
        kwargs = {}
        for a in cls._ATTRIBS:
            t, r = cls._ATTRIBS[a]  # t = type, r = required (bool)
            if r:
                try:
                    v = cls._parse_attrib(t, node.attrib[a])
                except Exception:
                    raise ValueError(
                        'Missing required attribute {} when parsing an object '
                        'of type {}'.format(a, cls.__name__)
                    )
            else:
                v = None
                if a in node.attrib:
                    v = cls._parse_attrib(t, node.attrib[a])
            kwargs[a] = v
        return kwargs

    @classmethod
    def _parse_simple_elements(cls, node, path):
        """Parse all elements in the _ELEMENTS array from the children of
        this node.

        Parameters
        ----------
        node : :class:`lxml.etree.Element`
            The node to parse children for.
        path : str
            The string path where the XML file is located (used for resolving
            the location of mesh or image files).

        Returns
        -------
        kwargs : dict
            Map from element names to the :class:`URDFType` subclass (or list,
            if ``multiple`` was set) created for that element.
        """
        kwargs = {}
        for a in cls._ELEMENTS:
            t, r, m = cls._ELEMENTS[a]
            if not m:
                v = node.find(t._TAG)
                if r or v is not None:
                    v = t._from_xml(v, path)
            else:
                vs = node.findall(t._TAG)
                if len(vs) == 0 and r:
                    raise ValueError(
                        'Missing required subelement(s) of type {} when '
                        'parsing an object of type {}'.format(
                            t.__name__, cls.__name__
                        )
                    )
                v = [t._from_xml(n, path) for n in vs]
            kwargs[a] = v
        return kwargs

    @classmethod
    def _parse(cls, node, path):
        """Parse all elements and attributes in the _ELEMENTS and _ATTRIBS
        arrays for a node.

        Parameters
        ----------
        node : :class:`lxml.etree.Element`
            The node to parse.
        path : str
            The string path where the XML file is located (used for resolving
            the location of mesh or image files).

        Returns
        -------
        kwargs : dict
            Map from names to Python classes created from the attributes
            and elements in the class arrays.
        """
        kwargs = cls._parse_simple_attribs(node)
        kwargs.update(cls._parse_simple_elements(node, path))
        return kwargs

    @classmethod
    def _from_xml(cls, node, path):
        """Create an instance of this class from an XML node.

        Parameters
        ----------
        node : :class:`lxml.etree.Element`
            The node to parse.
        path : str
            The string path where the XML file is located (used for resolving
            the location of mesh or image files).

        Returns
        -------
        obj : :class:`URDFType`
            An instance of this class parsed from the node.
        """
        return cls(**cls._parse(node, path))

    def _unparse_attrib(self, val_type, val):
        """Convert a Python value into a string for storage in an
        XML attribute.

        Parameters
        ----------
        val_type : :class:`type`
            The type of the Python object.
        val : :class:`object`
            The actual value.

        Returns
        -------
        s : str
            The attribute string.
        """
        if val_type == np.ndarray:
            val = np.array2string(val)[1:-1]
        else:
            val = str(val)
        return val

    def _unparse_simple_attribs(self, node):
        """Convert all Python types from the _ATTRIBS array back into attributes
        for an XML node.

        Parameters
        ----------
        node : :class:`object`
            The XML node to add the attributes to.
        """
        for a in self._ATTRIBS:
            t, r = self._ATTRIBS[a]
            v = getattr(self, a, None)
            if r or v is not None:
                node.attrib[a] = self._unparse_attrib(t, v)

    def _unparse_simple_elements(self, node, path):
        """Unparse all Python types from the _ELEMENTS array back into child
        nodes of an XML node.

        Parameters
        ----------
        node : :class:`object`
            The XML node for this object. Elements will be added as children
            of this node.
        path : str
            The string path where the XML file is being written to (used for
            writing out meshes and image files).
        """
        for a in self._ELEMENTS:
            t, r, m = self._ELEMENTS[a]
            v = getattr(self, a, None)
            if not m:
                if r or v is not None:
                    node.append(v._to_xml(node, path))
            else:
                vs = v
                for v in vs:
                    node.append(v._to_xml(node, path))

    def _unparse(self, path):
        """Create a node for this object and unparse all elements and
        attributes in the class arrays.

        Parameters
        ----------
        path : str
            The string path where the XML file is being written to (used for
            writing out meshes and image files).

        Returns
        -------
        node : :class:`lxml.etree.Element`
            The newly-created node.
        """
        node = ET.Element(self._TAG)
        self._unparse_simple_attribs(node)
        self._unparse_simple_elements(node, path)
        return node

    def _to_xml(self, parent, path):
        """Create and return an XML node for this object.

        Parameters
        ----------
        parent : :class:`lxml.etree.Element`
            The parent node that this element will eventually be added to.
            This base implementation doesn't use this information, but
            classes that override this function may use it.
        path : str
            The string path where the XML file is being written to (used for
            writing out meshes and image files).

        Returns
        -------
        node : :class:`lxml.etree.Element`
            The newly-created node.
        """
        return self._unparse(path)

