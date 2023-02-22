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

# ROS2 type magic
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class ComputeMinimumDistance(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        # this gets the type magic from ROS2
        check_for_type_support(LaserScan)
        check_for_type_support(Float32)

        self.output = outputs.get("distance", None)
        self.in_stream = inputs.get("scan", None)

        if self.in_stream is None:
            raise ValueError("No input 'scan' found")
        if self.output is None:
            raise ValueError("No output 'distance' found")

    def finalize(self) -> None:
        return None

    async def iteration(self) -> None:
        data_msg = await self.in_stream.recv()
        # this gets the minimum distance from the input scan

        # deserialize ROS2 message
        scan = _rclpy.rclpy_deserialize(data_msg.data, LaserScan)

        # get minimum angle
        distance = min(scan.ranges)

        # create message
        min_distance = Float32()
        min_distance.data = distance

        # serialize new ROS2 message
        serialized_distance = _rclpy.rclpy_serialize(min_distance, Float32)

        await self.output.send(serialized_distance)
        return None


def register():
    return ComputeMinimumDistance
