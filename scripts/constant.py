#!/usr/bin/env python2
# FILE			: constant.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, December 14 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

_PARING_ORDER = ( ("x" , 0) , ("y" , 1) , ("z" , 2) , ("roll" , 3) , ("pitch" , 4) , ("yaw" , 5) )

class Interface( object ):
    _TIMEOUT = 1.0
    _TOPIC_OUTPUT_ERROR = "control/system/error"
    _TOPIC_INPUT_STATE = "localize/zeabus"
    _TOPIC_INPUT_TARGET_VELOCITY = "control/interface/velocity"
    _TOPIC_INPUT_MASK = "control/mask"
    _RATE = 15
    _FRAME_ERROR_PARENT = "base_link"
    _FRAME_ERROR_CHILD = "base_link_target"

class System( object ):
    _PACKAGE = "zeabus_control"
    _DIRECTORY = "parameter"
    _FILE_TABLE = "throttle_force_table.txt"
    _FILE_TUNE = "pid_value.yaml"
    _TIMEOUT = 1.0
    _RATE = 15
    _TOPIC_INPUT_ERROR = Interface._TOPIC_OUTPUT_ERROR
    _TOPIC_INPUT_STATE = Interface._TOPIC_INPUT_STATE
    _TOPIC_INPUT_CURRENT_FORCE = "/control/force/current"
    _TOPIC_INPUT_ADDITION_FORCE = "/control/system/force"
    _TOPIC_OUTPUT_TARGET_FORCE = "/control/force/target"
    _TOPIC_OUTPUT_COMMAND_THROTTLE = "/control/thruster/throttle"
    _TOPIC_INPUT_ACTIVATE = "control/activate"
