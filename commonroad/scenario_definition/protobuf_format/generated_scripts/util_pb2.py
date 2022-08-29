# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: util.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='util.proto',
  package='commonroad',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\nutil.proto\x12\ncommonroad\"\x1d\n\x05Point\x12\t\n\x01x\x18\x01 \x02(\x01\x12\t\n\x01y\x18\x02 \x02(\x01\"b\n\tRectangle\x12\x0e\n\x06length\x18\x01 \x02(\x01\x12\r\n\x05width\x18\x02 \x02(\x01\x12!\n\x06\x63\x65nter\x18\x03 \x01(\x0b\x32\x11.commonroad.Point\x12\x13\n\x0borientation\x18\x04 \x01(\x01\";\n\x06\x43ircle\x12\x0e\n\x06radius\x18\x01 \x02(\x01\x12!\n\x06\x63\x65nter\x18\x02 \x01(\x0b\x32\x11.commonroad.Point\".\n\x07Polygon\x12#\n\x08vertices\x18\x01 \x03(\x0b\x32\x11.commonroad.Point\"/\n\nShapeGroup\x12!\n\x06shapes\x18\x01 \x03(\x0b\x32\x11.commonroad.Shape\"\xb9\x01\n\x05Shape\x12*\n\trectangle\x18\x01 \x01(\x0b\x32\x15.commonroad.RectangleH\x00\x12$\n\x06\x63ircle\x18\x02 \x01(\x0b\x32\x12.commonroad.CircleH\x00\x12&\n\x07polygon\x18\x03 \x01(\x0b\x32\x13.commonroad.PolygonH\x00\x12-\n\x0bshape_group\x18\x04 \x01(\x0b\x32\x16.commonroad.ShapeGroupH\x00\x42\x07\n\x05shape\"-\n\x0fIntegerInterval\x12\r\n\x05start\x18\x01 \x02(\x05\x12\x0b\n\x03\x65nd\x18\x02 \x02(\x05\"+\n\rFloatInterval\x12\r\n\x05start\x18\x01 \x02(\x01\x12\x0b\n\x03\x65nd\x18\x02 \x02(\x01\"o\n\x16IntegerExactOrInterval\x12\x0f\n\x05\x65xact\x18\x01 \x01(\x05H\x00\x12/\n\x08interval\x18\x02 \x01(\x0b\x32\x1b.commonroad.IntegerIntervalH\x00\x42\x13\n\x11\x65xact_or_interval\"k\n\x14\x46loatExactOrInterval\x12\x0f\n\x05\x65xact\x18\x01 \x01(\x01H\x00\x12-\n\x08interval\x18\x02 \x01(\x0b\x32\x19.commonroad.FloatIntervalH\x00\x42\x13\n\x11\x65xact_or_interval\"\x1d\n\x0bIntegerList\x12\x0e\n\x06values\x18\x01 \x03(\x05\"\x1b\n\tFloatList\x12\x0e\n\x06values\x18\x01 \x03(\x05\"S\n\tTimeStamp\x12\x0c\n\x04year\x18\x01 \x01(\r\x12\r\n\x05month\x18\x02 \x01(\r\x12\x0b\n\x03\x64\x61y\x18\x03 \x01(\r\x12\x0c\n\x04hour\x18\x04 \x01(\r\x12\x0e\n\x06minute\x18\x05 \x01(\r'
)




_POINT = _descriptor.Descriptor(
  name='Point',
  full_name='commonroad.Point',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='commonroad.Point.x', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='y', full_name='commonroad.Point.y', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
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
  serialized_start=26,
  serialized_end=55,
)


_RECTANGLE = _descriptor.Descriptor(
  name='Rectangle',
  full_name='commonroad.Rectangle',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='length', full_name='commonroad.Rectangle.length', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='width', full_name='commonroad.Rectangle.width', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='center', full_name='commonroad.Rectangle.center', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='orientation', full_name='commonroad.Rectangle.orientation', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_start=57,
  serialized_end=155,
)


_CIRCLE = _descriptor.Descriptor(
  name='Circle',
  full_name='commonroad.Circle',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='radius', full_name='commonroad.Circle.radius', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='center', full_name='commonroad.Circle.center', index=1,
      number=2, type=11, cpp_type=10, label=1,
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
  serialized_start=157,
  serialized_end=216,
)


_POLYGON = _descriptor.Descriptor(
  name='Polygon',
  full_name='commonroad.Polygon',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='vertices', full_name='commonroad.Polygon.vertices', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=218,
  serialized_end=264,
)


_SHAPEGROUP = _descriptor.Descriptor(
  name='ShapeGroup',
  full_name='commonroad.ShapeGroup',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='shapes', full_name='commonroad.ShapeGroup.shapes', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=266,
  serialized_end=313,
)


_SHAPE = _descriptor.Descriptor(
  name='Shape',
  full_name='commonroad.Shape',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='rectangle', full_name='commonroad.Shape.rectangle', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='circle', full_name='commonroad.Shape.circle', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='polygon', full_name='commonroad.Shape.polygon', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='shape_group', full_name='commonroad.Shape.shape_group', index=3,
      number=4, type=11, cpp_type=10, label=1,
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
    _descriptor.OneofDescriptor(
      name='shape', full_name='commonroad.Shape.shape',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=316,
  serialized_end=501,
)


_INTEGERINTERVAL = _descriptor.Descriptor(
  name='IntegerInterval',
  full_name='commonroad.IntegerInterval',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='start', full_name='commonroad.IntegerInterval.start', index=0,
      number=1, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='end', full_name='commonroad.IntegerInterval.end', index=1,
      number=2, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
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
  serialized_start=503,
  serialized_end=548,
)


_FLOATINTERVAL = _descriptor.Descriptor(
  name='FloatInterval',
  full_name='commonroad.FloatInterval',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='start', full_name='commonroad.FloatInterval.start', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='end', full_name='commonroad.FloatInterval.end', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
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
  serialized_start=550,
  serialized_end=593,
)


_INTEGEREXACTORINTERVAL = _descriptor.Descriptor(
  name='IntegerExactOrInterval',
  full_name='commonroad.IntegerExactOrInterval',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='exact', full_name='commonroad.IntegerExactOrInterval.exact', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='interval', full_name='commonroad.IntegerExactOrInterval.interval', index=1,
      number=2, type=11, cpp_type=10, label=1,
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
    _descriptor.OneofDescriptor(
      name='exact_or_interval', full_name='commonroad.IntegerExactOrInterval.exact_or_interval',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=595,
  serialized_end=706,
)


_FLOATEXACTORINTERVAL = _descriptor.Descriptor(
  name='FloatExactOrInterval',
  full_name='commonroad.FloatExactOrInterval',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='exact', full_name='commonroad.FloatExactOrInterval.exact', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='interval', full_name='commonroad.FloatExactOrInterval.interval', index=1,
      number=2, type=11, cpp_type=10, label=1,
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
    _descriptor.OneofDescriptor(
      name='exact_or_interval', full_name='commonroad.FloatExactOrInterval.exact_or_interval',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=708,
  serialized_end=815,
)


_INTEGERLIST = _descriptor.Descriptor(
  name='IntegerList',
  full_name='commonroad.IntegerList',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='values', full_name='commonroad.IntegerList.values', index=0,
      number=1, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=817,
  serialized_end=846,
)


_FLOATLIST = _descriptor.Descriptor(
  name='FloatList',
  full_name='commonroad.FloatList',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='values', full_name='commonroad.FloatList.values', index=0,
      number=1, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=848,
  serialized_end=875,
)


_TIMESTAMP = _descriptor.Descriptor(
  name='TimeStamp',
  full_name='commonroad.TimeStamp',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='year', full_name='commonroad.TimeStamp.year', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='month', full_name='commonroad.TimeStamp.month', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='day', full_name='commonroad.TimeStamp.day', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='hour', full_name='commonroad.TimeStamp.hour', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='minute', full_name='commonroad.TimeStamp.minute', index=4,
      number=5, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
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
  serialized_start=877,
  serialized_end=960,
)

_RECTANGLE.fields_by_name['center'].message_type = _POINT
_CIRCLE.fields_by_name['center'].message_type = _POINT
_POLYGON.fields_by_name['vertices'].message_type = _POINT
_SHAPEGROUP.fields_by_name['shapes'].message_type = _SHAPE
_SHAPE.fields_by_name['rectangle'].message_type = _RECTANGLE
_SHAPE.fields_by_name['circle'].message_type = _CIRCLE
_SHAPE.fields_by_name['polygon'].message_type = _POLYGON
_SHAPE.fields_by_name['shape_group'].message_type = _SHAPEGROUP
_SHAPE.oneofs_by_name['shape'].fields.append(
  _SHAPE.fields_by_name['rectangle'])
_SHAPE.fields_by_name['rectangle'].containing_oneof = _SHAPE.oneofs_by_name['shape']
_SHAPE.oneofs_by_name['shape'].fields.append(
  _SHAPE.fields_by_name['circle'])
_SHAPE.fields_by_name['circle'].containing_oneof = _SHAPE.oneofs_by_name['shape']
_SHAPE.oneofs_by_name['shape'].fields.append(
  _SHAPE.fields_by_name['polygon'])
_SHAPE.fields_by_name['polygon'].containing_oneof = _SHAPE.oneofs_by_name['shape']
_SHAPE.oneofs_by_name['shape'].fields.append(
  _SHAPE.fields_by_name['shape_group'])
_SHAPE.fields_by_name['shape_group'].containing_oneof = _SHAPE.oneofs_by_name['shape']
_INTEGEREXACTORINTERVAL.fields_by_name['interval'].message_type = _INTEGERINTERVAL
_INTEGEREXACTORINTERVAL.oneofs_by_name['exact_or_interval'].fields.append(
  _INTEGEREXACTORINTERVAL.fields_by_name['exact'])
_INTEGEREXACTORINTERVAL.fields_by_name['exact'].containing_oneof = _INTEGEREXACTORINTERVAL.oneofs_by_name['exact_or_interval']
_INTEGEREXACTORINTERVAL.oneofs_by_name['exact_or_interval'].fields.append(
  _INTEGEREXACTORINTERVAL.fields_by_name['interval'])
_INTEGEREXACTORINTERVAL.fields_by_name['interval'].containing_oneof = _INTEGEREXACTORINTERVAL.oneofs_by_name['exact_or_interval']
_FLOATEXACTORINTERVAL.fields_by_name['interval'].message_type = _FLOATINTERVAL
_FLOATEXACTORINTERVAL.oneofs_by_name['exact_or_interval'].fields.append(
  _FLOATEXACTORINTERVAL.fields_by_name['exact'])
_FLOATEXACTORINTERVAL.fields_by_name['exact'].containing_oneof = _FLOATEXACTORINTERVAL.oneofs_by_name['exact_or_interval']
_FLOATEXACTORINTERVAL.oneofs_by_name['exact_or_interval'].fields.append(
  _FLOATEXACTORINTERVAL.fields_by_name['interval'])
_FLOATEXACTORINTERVAL.fields_by_name['interval'].containing_oneof = _FLOATEXACTORINTERVAL.oneofs_by_name['exact_or_interval']
DESCRIPTOR.message_types_by_name['Point'] = _POINT
DESCRIPTOR.message_types_by_name['Rectangle'] = _RECTANGLE
DESCRIPTOR.message_types_by_name['Circle'] = _CIRCLE
DESCRIPTOR.message_types_by_name['Polygon'] = _POLYGON
DESCRIPTOR.message_types_by_name['ShapeGroup'] = _SHAPEGROUP
DESCRIPTOR.message_types_by_name['Shape'] = _SHAPE
DESCRIPTOR.message_types_by_name['IntegerInterval'] = _INTEGERINTERVAL
DESCRIPTOR.message_types_by_name['FloatInterval'] = _FLOATINTERVAL
DESCRIPTOR.message_types_by_name['IntegerExactOrInterval'] = _INTEGEREXACTORINTERVAL
DESCRIPTOR.message_types_by_name['FloatExactOrInterval'] = _FLOATEXACTORINTERVAL
DESCRIPTOR.message_types_by_name['IntegerList'] = _INTEGERLIST
DESCRIPTOR.message_types_by_name['FloatList'] = _FLOATLIST
DESCRIPTOR.message_types_by_name['TimeStamp'] = _TIMESTAMP
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Point = _reflection.GeneratedProtocolMessageType('Point', (_message.Message,), {
  'DESCRIPTOR' : _POINT,
  '__module__' : 'util_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.Point)
  })
_sym_db.RegisterMessage(Point)

Rectangle = _reflection.GeneratedProtocolMessageType('Rectangle', (_message.Message,), {
  'DESCRIPTOR' : _RECTANGLE,
  '__module__' : 'util_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.Rectangle)
  })
_sym_db.RegisterMessage(Rectangle)

Circle = _reflection.GeneratedProtocolMessageType('Circle', (_message.Message,), {
  'DESCRIPTOR' : _CIRCLE,
  '__module__' : 'util_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.Circle)
  })
_sym_db.RegisterMessage(Circle)

Polygon = _reflection.GeneratedProtocolMessageType('Polygon', (_message.Message,), {
  'DESCRIPTOR' : _POLYGON,
  '__module__' : 'util_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.Polygon)
  })
_sym_db.RegisterMessage(Polygon)

ShapeGroup = _reflection.GeneratedProtocolMessageType('ShapeGroup', (_message.Message,), {
  'DESCRIPTOR' : _SHAPEGROUP,
  '__module__' : 'util_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.ShapeGroup)
  })
_sym_db.RegisterMessage(ShapeGroup)

Shape = _reflection.GeneratedProtocolMessageType('Shape', (_message.Message,), {
  'DESCRIPTOR' : _SHAPE,
  '__module__' : 'util_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.Shape)
  })
_sym_db.RegisterMessage(Shape)

IntegerInterval = _reflection.GeneratedProtocolMessageType('IntegerInterval', (_message.Message,), {
  'DESCRIPTOR' : _INTEGERINTERVAL,
  '__module__' : 'util_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.IntegerInterval)
  })
_sym_db.RegisterMessage(IntegerInterval)

FloatInterval = _reflection.GeneratedProtocolMessageType('FloatInterval', (_message.Message,), {
  'DESCRIPTOR' : _FLOATINTERVAL,
  '__module__' : 'util_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.FloatInterval)
  })
_sym_db.RegisterMessage(FloatInterval)

IntegerExactOrInterval = _reflection.GeneratedProtocolMessageType('IntegerExactOrInterval', (_message.Message,), {
  'DESCRIPTOR' : _INTEGEREXACTORINTERVAL,
  '__module__' : 'util_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.IntegerExactOrInterval)
  })
_sym_db.RegisterMessage(IntegerExactOrInterval)

FloatExactOrInterval = _reflection.GeneratedProtocolMessageType('FloatExactOrInterval', (_message.Message,), {
  'DESCRIPTOR' : _FLOATEXACTORINTERVAL,
  '__module__' : 'util_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.FloatExactOrInterval)
  })
_sym_db.RegisterMessage(FloatExactOrInterval)

IntegerList = _reflection.GeneratedProtocolMessageType('IntegerList', (_message.Message,), {
  'DESCRIPTOR' : _INTEGERLIST,
  '__module__' : 'util_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.IntegerList)
  })
_sym_db.RegisterMessage(IntegerList)

FloatList = _reflection.GeneratedProtocolMessageType('FloatList', (_message.Message,), {
  'DESCRIPTOR' : _FLOATLIST,
  '__module__' : 'util_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.FloatList)
  })
_sym_db.RegisterMessage(FloatList)

TimeStamp = _reflection.GeneratedProtocolMessageType('TimeStamp', (_message.Message,), {
  'DESCRIPTOR' : _TIMESTAMP,
  '__module__' : 'util_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.TimeStamp)
  })
_sym_db.RegisterMessage(TimeStamp)


# @@protoc_insertion_point(module_scope)
