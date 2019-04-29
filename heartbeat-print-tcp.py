#!/usr/bin/python

# This file is part of the MAVLink Router project
#
# Copyright (C) 2016  Intel Corporation. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function

import pymavlink.mavutil as mavutil
import sys
import time

mav = mavutil.mavlink_connection('tcp:' + '127.0.0.1:5760')
mav.wait_heartbeat()

if __name__ == "__main__":
    m = mav.recv_match(type='HEARTBEAT', blocking=True)
    if m is not None:
	print(m)
	mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
	time.sleep(3)
	mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)
