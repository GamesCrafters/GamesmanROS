# generated from rosidl_generator_py/resource/_idl.py.em
# with input from gamesmanros_interfaces:msg/PieceDetection.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PieceDetection(type):
    """Metaclass of message 'PieceDetection'."""

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
            module = import_type_support('gamesmanros_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'gamesmanros_interfaces.msg.PieceDetection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__piece_detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__piece_detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__piece_detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__piece_detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__piece_detection

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PieceDetection(metaclass=Metaclass_PieceDetection):
    """Message class 'PieceDetection'."""

    __slots__ = [
        '_tag_id',
        '_cell_index',
        '_board_x',
        '_board_y',
        '_offset_mm',
    ]

    _fields_and_field_types = {
        'tag_id': 'int32',
        'cell_index': 'int32',
        'board_x': 'double',
        'board_y': 'double',
        'offset_mm': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.tag_id = kwargs.get('tag_id', int())
        self.cell_index = kwargs.get('cell_index', int())
        self.board_x = kwargs.get('board_x', float())
        self.board_y = kwargs.get('board_y', float())
        self.offset_mm = kwargs.get('offset_mm', float())

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
        if self.tag_id != other.tag_id:
            return False
        if self.cell_index != other.cell_index:
            return False
        if self.board_x != other.board_x:
            return False
        if self.board_y != other.board_y:
            return False
        if self.offset_mm != other.offset_mm:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def tag_id(self):
        """Message field 'tag_id'."""
        return self._tag_id

    @tag_id.setter
    def tag_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'tag_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'tag_id' field must be an integer in [-2147483648, 2147483647]"
        self._tag_id = value

    @builtins.property
    def cell_index(self):
        """Message field 'cell_index'."""
        return self._cell_index

    @cell_index.setter
    def cell_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cell_index' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'cell_index' field must be an integer in [-2147483648, 2147483647]"
        self._cell_index = value

    @builtins.property
    def board_x(self):
        """Message field 'board_x'."""
        return self._board_x

    @board_x.setter
    def board_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'board_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'board_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._board_x = value

    @builtins.property
    def board_y(self):
        """Message field 'board_y'."""
        return self._board_y

    @board_y.setter
    def board_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'board_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'board_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._board_y = value

    @builtins.property
    def offset_mm(self):
        """Message field 'offset_mm'."""
        return self._offset_mm

    @offset_mm.setter
    def offset_mm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'offset_mm' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'offset_mm' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._offset_mm = value
