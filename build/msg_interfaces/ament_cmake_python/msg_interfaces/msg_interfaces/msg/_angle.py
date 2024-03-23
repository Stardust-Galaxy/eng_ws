# generated from rosidl_generator_py/resource/_idl.py.em
# with input from msg_interfaces:msg/Angle.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Angle(type):
    """Metaclass of message 'Angle'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('msg_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'msg_interfaces.msg.Angle')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__angle
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__angle
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__angle
            cls._TYPE_SUPPORT = module.type_support_msg__msg__angle
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__angle

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Angle(metaclass=Metaclass_Angle):
    """Message class 'Angle'."""

    __slots__ = [
        '_found',
        '_quaternion1',
        '_quaternion2',
        '_quaternion3',
        '_quaternion4',
        '_x',
        '_y',
        '_z',
    ]

    _fields_and_field_types = {
        'found': 'int8',
        'quaternion1': 'double',
        'quaternion2': 'double',
        'quaternion3': 'double',
        'quaternion4': 'double',
        'x': 'double',
        'y': 'double',
        'z': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.found = kwargs.get('found', int())
        self.quaternion1 = kwargs.get('quaternion1', float())
        self.quaternion2 = kwargs.get('quaternion2', float())
        self.quaternion3 = kwargs.get('quaternion3', float())
        self.quaternion4 = kwargs.get('quaternion4', float())
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.z = kwargs.get('z', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.found != other.found:
            return False
        if self.quaternion1 != other.quaternion1:
            return False
        if self.quaternion2 != other.quaternion2:
            return False
        if self.quaternion3 != other.quaternion3:
            return False
        if self.quaternion4 != other.quaternion4:
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def found(self):
        """Message field 'found'."""
        return self._found

    @found.setter
    def found(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'found' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'found' field must be an integer in [-128, 127]"
        self._found = value

    @builtins.property
    def quaternion1(self):
        """Message field 'quaternion1'."""
        return self._quaternion1

    @quaternion1.setter
    def quaternion1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'quaternion1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'quaternion1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._quaternion1 = value

    @builtins.property
    def quaternion2(self):
        """Message field 'quaternion2'."""
        return self._quaternion2

    @quaternion2.setter
    def quaternion2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'quaternion2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'quaternion2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._quaternion2 = value

    @builtins.property
    def quaternion3(self):
        """Message field 'quaternion3'."""
        return self._quaternion3

    @quaternion3.setter
    def quaternion3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'quaternion3' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'quaternion3' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._quaternion3 = value

    @builtins.property
    def quaternion4(self):
        """Message field 'quaternion4'."""
        return self._quaternion4

    @quaternion4.setter
    def quaternion4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'quaternion4' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'quaternion4' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._quaternion4 = value

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y = value

    @builtins.property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._z = value
