# generated from rosidl_generator_py/resource/_idl.py.em
# with input from gamesmanros_interfaces:msg/GameState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GameState(type):
    """Metaclass of message 'GameState'."""

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
                'gamesmanros_interfaces.msg.GameState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__game_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__game_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__game_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__game_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__game_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GameState(metaclass=Metaclass_GameState):
    """Message class 'GameState'."""

    __slots__ = [
        '_position',
        '_available_moves',
        '_is_robot_turn',
        '_game_over',
    ]

    _fields_and_field_types = {
        'position': 'string',
        'available_moves': 'sequence<string>',
        'is_robot_turn': 'boolean',
        'game_over': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.position = kwargs.get('position', str())
        self.available_moves = kwargs.get('available_moves', [])
        self.is_robot_turn = kwargs.get('is_robot_turn', bool())
        self.game_over = kwargs.get('game_over', bool())

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
        if self.position != other.position:
            return False
        if self.available_moves != other.available_moves:
            return False
        if self.is_robot_turn != other.is_robot_turn:
            return False
        if self.game_over != other.game_over:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'position' field must be of type 'str'"
        self._position = value

    @builtins.property
    def available_moves(self):
        """Message field 'available_moves'."""
        return self._available_moves

    @available_moves.setter
    def available_moves(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'available_moves' field must be a set or sequence and each value of type 'str'"
        self._available_moves = value

    @builtins.property
    def is_robot_turn(self):
        """Message field 'is_robot_turn'."""
        return self._is_robot_turn

    @is_robot_turn.setter
    def is_robot_turn(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_robot_turn' field must be of type 'bool'"
        self._is_robot_turn = value

    @builtins.property
    def game_over(self):
        """Message field 'game_over'."""
        return self._game_over

    @game_over.setter
    def game_over(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'game_over' field must be of type 'bool'"
        self._game_over = value
