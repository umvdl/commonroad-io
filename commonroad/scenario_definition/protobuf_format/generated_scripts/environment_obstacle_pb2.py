# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: environment_obstacle.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import commonroad.scenario_definition.protobuf_format.generated_scripts.obstacle_pb2 as obstacle__pb2
import commonroad.scenario_definition.protobuf_format.generated_scripts.util_pb2 as util__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='environment_obstacle.proto',
  package='commonroad',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x1a\x65nvironment_obstacle.proto\x12\ncommonroad\x1a\x0eobstacle.proto\x1a\nutil.proto\"\xa3\x01\n\x13\x45nvironmentObstacle\x12\x1f\n\x17\x65nvironment_obstacle_id\x18\x01 \x02(\r\x12@\n\robstacle_type\x18\x02 \x02(\x0e\x32).commonroad.ObstacleTypeEnum.ObstacleType\x12)\n\x0eobstacle_shape\x18\x03 \x02(\x0b\x32\x11.commonroad.Shape'
  ,
  dependencies=[obstacle__pb2.DESCRIPTOR,util__pb2.DESCRIPTOR,])




_ENVIRONMENTOBSTACLE = _descriptor.Descriptor(
  name='EnvironmentObstacle',
  full_name='commonroad.EnvironmentObstacle',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='environment_obstacle_id', full_name='commonroad.EnvironmentObstacle.environment_obstacle_id', index=0,
      number=1, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='obstacle_type', full_name='commonroad.EnvironmentObstacle.obstacle_type', index=1,
      number=2, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='obstacle_shape', full_name='commonroad.EnvironmentObstacle.obstacle_shape', index=2,
      number=3, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=71,
  serialized_end=234,
)

_ENVIRONMENTOBSTACLE.fields_by_name['obstacle_type'].enum_type = obstacle__pb2._OBSTACLETYPEENUM_OBSTACLETYPE
_ENVIRONMENTOBSTACLE.fields_by_name['obstacle_shape'].message_type = util__pb2._SHAPE
DESCRIPTOR.message_types_by_name['EnvironmentObstacle'] = _ENVIRONMENTOBSTACLE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

EnvironmentObstacle = _reflection.GeneratedProtocolMessageType('EnvironmentObstacle', (_message.Message,), {
  'DESCRIPTOR' : _ENVIRONMENTOBSTACLE,
  '__module__' : 'environment_obstacle_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.EnvironmentObstacle)
  })
_sym_db.RegisterMessage(EnvironmentObstacle)


# @@protoc_insertion_point(module_scope)
