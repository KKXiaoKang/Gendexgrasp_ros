#!/bin/bash
protoc -I=. --csharp_out=. --python_out=. ./protos/hand_pose.proto
protoc -I=. --python_out=. ./protos/hand_pose.proto
