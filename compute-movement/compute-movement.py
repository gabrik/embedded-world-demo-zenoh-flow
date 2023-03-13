#
# Copyright (c) 2022 ZettaScale Technology
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
# which is available at https://www.apache.org/licenses/LICENSE-2.0.
#
# SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#
# Contributors:
#   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
#


from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

#ROS2 type magic
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

from geometry_msgs.msg import Twist, Vector3
import json
import asyncio

SLEEP_TIME = 2
ANGULAR_SCALE = 0.5
LINEAR_SCALE = 0.2


# to publish a Twist message to ROS2
def make_twist(linear, angular):
    print("Creating twist: {} - {}".format(linear, angular))
    t = Twist(linear=Vector3(x=linear, y=0.0, z=0.0),
        angular=Vector3(x=0.0, y=0.0, z=angular))
    return t


class ComputeMovement(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        # this gets the type magic from ROS2
        check_for_type_support(Twist)
        self.output = outputs.get("twist", None)
        self.in_stream = inputs.get("button", None)

        if self.in_stream is None:
            raise ValueError("No input 'button' found")
        if self.output is None:
            raise ValueError("No output 'twist' found")

        self.sleep_time = SLEEP_TIME
        self.linear_scale = LINEAR_SCALE
        self.linear_scale = ANGULAR_SCALE

        if configuration is not None:
            self.sleep_time = configuration.get('time', SLEEP_TIME)
            self.linear_scale = configuration.get('linear_scale', LINEAR_SCALE)
            self.linear_scale = configuration.get('angular_scale', ANGULAR_SCALE)

    def finalize(self) -> None:
        return None

    async def iteration(self) -> None:
        data_msg = await self.in_stream.recv()
        # this gets the configured degree of lidar scan
        # from -30 to 30 and sends them downstream

        # deserialize ROS2 message
        button_status = json.loads(data_msg.data.decode("utf-8"))

        print(f'Button Status {button_status}')
        if button_status['action'] == 'single':
            print('   => single')
            t = make_twist(0.0, 1.0 * self.angular_scale)
            serialized_t = _rclpy.rclpy_serialize(t, type(t))
            await self.output.send(serialized_t)

            await asyncio.sleep(self.sleep_time)
            t = make_twist(0.0, 0.0)
            serialized_t = _rclpy.rclpy_serialize(t, type(t))
            await self.output.send(serialized_t)

        elif button_status['action'] == 'double':
            print('   => double')
            t = make_twist(0.0, -1.0 * self.angular_scale)
            serialized_t = _rclpy.rclpy_serialize(t, type(t))
            await self.output.send(serialized_t)
            await asyncio.sleep(self.sleep_time)
            t = make_twist(0.0, 0.0)
            serialized_t = _rclpy.rclpy_serialize(t, type(t))
            await self.output.send(serialized_t)
        else:
            t = make_twist(0.0, 0.0)
            serialized_t = _rclpy.rclpy_serialize(t, type(t))
            await self.output.send(serialized_t)

        # return _rclpy.rclpy_serialize(ls, type(ls))
        # this deserializes
        # return _rclpy.rclpy_deserialize(serialized_message, message_type)

        # await self.output.send(new_serialized_scan)
        return None


def register():
    return ComputeMovement
