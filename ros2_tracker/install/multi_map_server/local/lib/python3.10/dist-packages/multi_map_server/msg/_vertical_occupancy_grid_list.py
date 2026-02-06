# generated from rosidl_generator_py/resource/_idl.py.em
# with input from multi_map_server:msg/VerticalOccupancyGridList.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'upper'
# Member 'lower'
# Member 'mass'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_VerticalOccupancyGridList(type):
    """Metaclass of message 'VerticalOccupancyGridList'."""

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
            module = import_type_support('multi_map_server')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'multi_map_server.msg.VerticalOccupancyGridList')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__vertical_occupancy_grid_list
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__vertical_occupancy_grid_list
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__vertical_occupancy_grid_list
            cls._TYPE_SUPPORT = module.type_support_msg__msg__vertical_occupancy_grid_list
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__vertical_occupancy_grid_list

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class VerticalOccupancyGridList(metaclass=Metaclass_VerticalOccupancyGridList):
    """Message class 'VerticalOccupancyGridList'."""

    __slots__ = [
        '_x',
        '_y',
        '_upper',
        '_lower',
        '_mass',
    ]

    _fields_and_field_types = {
        'x': 'float',
        'y': 'float',
        'upper': 'sequence<int32>',
        'lower': 'sequence<int32>',
        'mass': 'sequence<int32>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.upper = array.array('i', kwargs.get('upper', []))
        self.lower = array.array('i', kwargs.get('lower', []))
        self.mass = array.array('i', kwargs.get('mass', []))

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
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.upper != other.upper:
            return False
        if self.lower != other.lower:
            return False
        if self.mass != other.mass:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
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
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y = value

    @builtins.property
    def upper(self):
        """Message field 'upper'."""
        return self._upper

    @upper.setter
    def upper(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'upper' array.array() must have the type code of 'i'"
            self._upper = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'upper' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._upper = array.array('i', value)

    @builtins.property
    def lower(self):
        """Message field 'lower'."""
        return self._lower

    @lower.setter
    def lower(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'lower' array.array() must have the type code of 'i'"
            self._lower = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'lower' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._lower = array.array('i', value)

    @builtins.property
    def mass(self):
        """Message field 'mass'."""
        return self._mass

    @mass.setter
    def mass(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'mass' array.array() must have the type code of 'i'"
            self._mass = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'mass' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._mass = array.array('i', value)
