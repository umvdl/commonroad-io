# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: location.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import commonroad.scenario_definition.protobuf_format.generated_scripts.util_pb2 as util__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='location.proto',
  package='commonroad',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n\x0elocation.proto\x12\ncommonroad\x1a\nutil.proto\"=\n\rTimeOfDayEnum\",\n\tTimeOfDay\x12\t\n\x05NIGHT\x10\x00\x12\x07\n\x03\x44\x41Y\x10\x01\x12\x0b\n\x07UNKNOWN\x10\x02\"l\n\x0bWeatherEnum\"]\n\x07Weather\x12\t\n\x05SUNNY\x10\x00\x12\x0e\n\nLIGHT_RAIN\x10\x01\x12\r\n\tHEAVY_AIN\x10\x02\x12\x07\n\x03\x46OG\x10\x03\x12\x08\n\x04SNOW\x10\x04\x12\x08\n\x04HAIL\x10\x05\x12\x0b\n\x07UNKNOWN\x10\x06\"l\n\x0fUndergroundEnum\"Y\n\x0bUnderground\x12\x07\n\x03WET\x10\x00\x12\t\n\x05\x43LEAN\x10\x01\x12\t\n\x05\x44IRTY\x10\x02\x12\x0b\n\x07\x44\x41MAGED\x10\x03\x12\x08\n\x04SNOW\x10\x04\x12\x07\n\x03ICE\x10\x05\x12\x0b\n\x07UNKNOWN\x10\x06\"}\n\x11GeoTransformation\x12\x15\n\rgeo_reference\x18\x01 \x01(\t\x12\x15\n\rx_translation\x18\x02 \x01(\x01\x12\x15\n\ry_translation\x18\x03 \x01(\x01\x12\x12\n\nz_rotation\x18\x04 \x01(\x01\x12\x0f\n\x07scaling\x18\x05 \x01(\x01\"\xdc\x01\n\x0b\x45nvironment\x12#\n\x04time\x18\x01 \x01(\x0b\x32\x15.commonroad.TimeStamp\x12\x38\n\x0btime_of_day\x18\x02 \x01(\x0e\x32#.commonroad.TimeOfDayEnum.TimeOfDay\x12\x30\n\x07weather\x18\x03 \x01(\x0e\x32\x1f.commonroad.WeatherEnum.Weather\x12<\n\x0bunderground\x18\x04 \x01(\x0e\x32\'.commonroad.UndergroundEnum.Underground\"\xb5\x01\n\x08Location\x12\x13\n\x0bgeo_name_id\x18\x01 \x02(\x05\x12\x14\n\x0cgps_latitude\x18\x02 \x02(\x01\x12\x15\n\rgps_longitude\x18\x03 \x02(\x01\x12\x39\n\x12geo_transformation\x18\x04 \x01(\x0b\x32\x1d.commonroad.GeoTransformation\x12,\n\x0b\x65nvironment\x18\x05 \x01(\x0b\x32\x17.commonroad.Environment')
  ,
  dependencies=[util__pb2.DESCRIPTOR,])



_TIMEOFDAYENUM_TIMEOFDAY = _descriptor.EnumDescriptor(
  name='TimeOfDay',
  full_name='commonroad.TimeOfDayEnum.TimeOfDay',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NIGHT', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DAY', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=2, number=2,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=59,
  serialized_end=103,
)
_sym_db.RegisterEnumDescriptor(_TIMEOFDAYENUM_TIMEOFDAY)

_WEATHERENUM_WEATHER = _descriptor.EnumDescriptor(
  name='Weather',
  full_name='commonroad.WeatherEnum.Weather',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='SUNNY', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LIGHT_RAIN', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='HEAVY_AIN', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FOG', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SNOW', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='HAIL', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=6, number=6,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=120,
  serialized_end=213,
)
_sym_db.RegisterEnumDescriptor(_WEATHERENUM_WEATHER)

_UNDERGROUNDENUM_UNDERGROUND = _descriptor.EnumDescriptor(
  name='Underground',
  full_name='commonroad.UndergroundEnum.Underground',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='WET', index=0, number=0,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CLEAN', index=1, number=1,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DIRTY', index=2, number=2,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DAMAGED', index=3, number=3,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SNOW', index=4, number=4,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ICE', index=5, number=5,
      serialized_options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=6, number=6,
      serialized_options=None,
      type=None),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=234,
  serialized_end=323,
)
_sym_db.RegisterEnumDescriptor(_UNDERGROUNDENUM_UNDERGROUND)


_TIMEOFDAYENUM = _descriptor.Descriptor(
  name='TimeOfDayEnum',
  full_name='commonroad.TimeOfDayEnum',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _TIMEOFDAYENUM_TIMEOFDAY,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=42,
  serialized_end=103,
)


_WEATHERENUM = _descriptor.Descriptor(
  name='WeatherEnum',
  full_name='commonroad.WeatherEnum',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _WEATHERENUM_WEATHER,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=105,
  serialized_end=213,
)


_UNDERGROUNDENUM = _descriptor.Descriptor(
  name='UndergroundEnum',
  full_name='commonroad.UndergroundEnum',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _UNDERGROUNDENUM_UNDERGROUND,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=215,
  serialized_end=323,
)


_GEOTRANSFORMATION = _descriptor.Descriptor(
  name='GeoTransformation',
  full_name='commonroad.GeoTransformation',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='geo_reference', full_name='commonroad.GeoTransformation.geo_reference', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='x_translation', full_name='commonroad.GeoTransformation.x_translation', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='y_translation', full_name='commonroad.GeoTransformation.y_translation', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='z_rotation', full_name='commonroad.GeoTransformation.z_rotation', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='scaling', full_name='commonroad.GeoTransformation.scaling', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
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
  serialized_start=325,
  serialized_end=450,
)


_ENVIRONMENT = _descriptor.Descriptor(
  name='Environment',
  full_name='commonroad.Environment',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='time', full_name='commonroad.Environment.time', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='time_of_day', full_name='commonroad.Environment.time_of_day', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='weather', full_name='commonroad.Environment.weather', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='underground', full_name='commonroad.Environment.underground', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
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
  serialized_start=453,
  serialized_end=673,
)


_LOCATION = _descriptor.Descriptor(
  name='Location',
  full_name='commonroad.Location',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='geo_name_id', full_name='commonroad.Location.geo_name_id', index=0,
      number=1, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gps_latitude', full_name='commonroad.Location.gps_latitude', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='gps_longitude', full_name='commonroad.Location.gps_longitude', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='geo_transformation', full_name='commonroad.Location.geo_transformation', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='environment', full_name='commonroad.Location.environment', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
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
  serialized_start=676,
  serialized_end=857,
)

_TIMEOFDAYENUM_TIMEOFDAY.containing_type = _TIMEOFDAYENUM
_WEATHERENUM_WEATHER.containing_type = _WEATHERENUM
_UNDERGROUNDENUM_UNDERGROUND.containing_type = _UNDERGROUNDENUM
_ENVIRONMENT.fields_by_name['time'].message_type = util__pb2._TIMESTAMP
_ENVIRONMENT.fields_by_name['time_of_day'].enum_type = _TIMEOFDAYENUM_TIMEOFDAY
_ENVIRONMENT.fields_by_name['weather'].enum_type = _WEATHERENUM_WEATHER
_ENVIRONMENT.fields_by_name['underground'].enum_type = _UNDERGROUNDENUM_UNDERGROUND
_LOCATION.fields_by_name['geo_transformation'].message_type = _GEOTRANSFORMATION
_LOCATION.fields_by_name['environment'].message_type = _ENVIRONMENT
DESCRIPTOR.message_types_by_name['TimeOfDayEnum'] = _TIMEOFDAYENUM
DESCRIPTOR.message_types_by_name['WeatherEnum'] = _WEATHERENUM
DESCRIPTOR.message_types_by_name['UndergroundEnum'] = _UNDERGROUNDENUM
DESCRIPTOR.message_types_by_name['GeoTransformation'] = _GEOTRANSFORMATION
DESCRIPTOR.message_types_by_name['Environment'] = _ENVIRONMENT
DESCRIPTOR.message_types_by_name['Location'] = _LOCATION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

TimeOfDayEnum = _reflection.GeneratedProtocolMessageType('TimeOfDayEnum', (_message.Message,), dict(
  DESCRIPTOR = _TIMEOFDAYENUM,
  __module__ = 'location_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.TimeOfDayEnum)
  ))
_sym_db.RegisterMessage(TimeOfDayEnum)

WeatherEnum = _reflection.GeneratedProtocolMessageType('WeatherEnum', (_message.Message,), dict(
  DESCRIPTOR = _WEATHERENUM,
  __module__ = 'location_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.WeatherEnum)
  ))
_sym_db.RegisterMessage(WeatherEnum)

UndergroundEnum = _reflection.GeneratedProtocolMessageType('UndergroundEnum', (_message.Message,), dict(
  DESCRIPTOR = _UNDERGROUNDENUM,
  __module__ = 'location_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.UndergroundEnum)
  ))
_sym_db.RegisterMessage(UndergroundEnum)

GeoTransformation = _reflection.GeneratedProtocolMessageType('GeoTransformation', (_message.Message,), dict(
  DESCRIPTOR = _GEOTRANSFORMATION,
  __module__ = 'location_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.GeoTransformation)
  ))
_sym_db.RegisterMessage(GeoTransformation)

Environment = _reflection.GeneratedProtocolMessageType('Environment', (_message.Message,), dict(
  DESCRIPTOR = _ENVIRONMENT,
  __module__ = 'location_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.Environment)
  ))
_sym_db.RegisterMessage(Environment)

Location = _reflection.GeneratedProtocolMessageType('Location', (_message.Message,), dict(
  DESCRIPTOR = _LOCATION,
  __module__ = 'location_pb2'
  # @@protoc_insertion_point(class_scope:commonroad.Location)
  ))
_sym_db.RegisterMessage(Location)


# @@protoc_insertion_point(module_scope)
