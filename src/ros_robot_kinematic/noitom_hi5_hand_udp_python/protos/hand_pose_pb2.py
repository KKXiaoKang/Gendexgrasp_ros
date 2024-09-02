# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: protos/hand_pose.proto
# Protobuf Python Version: 4.25.4
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x16protos/hand_pose.proto\"\xfd\x04\n\x11LejuHandPoseEvent\x12\x11\n\ttimestamp\x18\x01 \x01(\x03\x12&\n\x05poses\x18\x02 \x03(\x0b\x32\x17.LejuHandPoseEvent.Pose\x12\x1c\n\x14IsDataHighConfidence\x18\x03 \x01(\x08\x12\x16\n\x0eIsHandTracking\x18\x04 \x01(\x08\x12\x32\n\rleft_joystick\x18\x05 \x01(\x0b\x32\x1b.LejuHandPoseEvent.Joystick\x12\x33\n\x0eright_joystick\x18\x06 \x01(\x0b\x32\x1b.LejuHandPoseEvent.Joystick\x1a\xd9\x01\n\x04Pose\x12\x32\n\x08position\x18\x01 \x01(\x0b\x32 .LejuHandPoseEvent.Pose.Position\x12\x36\n\nquaternion\x18\x02 \x01(\x0b\x32\".LejuHandPoseEvent.Pose.Quaternion\x1a+\n\x08Position\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\t\n\x01z\x18\x03 \x01(\x02\x1a\x38\n\nQuaternion\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\t\n\x01z\x18\x03 \x01(\x02\x12\t\n\x01w\x18\x04 \x01(\x02\x1a\xb1\x01\n\x08Joystick\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\x0f\n\x07trigger\x18\x03 \x01(\x02\x12\x0c\n\x04grip\x18\x04 \x01(\x02\x12\x1a\n\x12\x66irstButtonPressed\x18\x05 \x01(\x08\x12\x1b\n\x13secondButtonPressed\x18\x06 \x01(\x08\x12\x1a\n\x12\x66irstButtonTouched\x18\x07 \x01(\x08\x12\x1b\n\x13secondButtonTouched\x18\x08 \x01(\x08\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'protos.hand_pose_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_LEJUHANDPOSEEVENT']._serialized_start=27
  _globals['_LEJUHANDPOSEEVENT']._serialized_end=664
  _globals['_LEJUHANDPOSEEVENT_POSE']._serialized_start=267
  _globals['_LEJUHANDPOSEEVENT_POSE']._serialized_end=484
  _globals['_LEJUHANDPOSEEVENT_POSE_POSITION']._serialized_start=383
  _globals['_LEJUHANDPOSEEVENT_POSE_POSITION']._serialized_end=426
  _globals['_LEJUHANDPOSEEVENT_POSE_QUATERNION']._serialized_start=428
  _globals['_LEJUHANDPOSEEVENT_POSE_QUATERNION']._serialized_end=484
  _globals['_LEJUHANDPOSEEVENT_JOYSTICK']._serialized_start=487
  _globals['_LEJUHANDPOSEEVENT_JOYSTICK']._serialized_end=664
# @@protoc_insertion_point(module_scope)
