#!/bin/sh
# compile xacro files into raw URDFs
# note that we run each command more than once to resolve nested macros

mkdir -p compiled
xacro xacro/thing_no_wheels.urdf.xacro -o compiled/thing_no_wheels.urdf
xacro compiled/thing_no_wheels.urdf -o compiled/thing_no_wheels.urdf

xacro xacro/thing_pyb.urdf.xacro -o compiled/thing_pyb.urdf
xacro compiled/thing_pyb.urdf -o compiled/thing_pyb.urdf

xacro xacro/ur10.urdf.xacro -o compiled/ur10.urdf
xacro compiled/ur10.urdf -o compiled/ur10.urdf
