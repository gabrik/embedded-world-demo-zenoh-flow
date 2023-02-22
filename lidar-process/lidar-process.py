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

from sensor_msgs.msg import LaserScan

PROXIMITY_SCAN_ANGLE = 60


class LidarProcess(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        # this gets the type magic from ROS2
        check_for_type_support(LaserScan)
        self.output = outputs.get("front_scan", None)
        self.in_stream = inputs.get("scan", None)

        if self.in_stream is None:
            raise ValueError("No input 'scan' found")
        if self.output is None:
            raise ValueError("No output 'front_scan' found")

        if configuration is not None:
            self.interest_angle = configuration.get('angle', PROXIMITY_SCAN_ANGLE)

    def finalize(self) -> None:
        return None

    async def iteration(self) -> None:
        data_msg = await self.in_stream.recv()
        # this gets the configured degree of lidar scan
        # from -30 to 30 and sends them downstream

        # deserialize ROS2 message
        scan = _rclpy.rclpy_deserialize(data_msg.data, LaserScan)

        # get the configured degrees
        new_range = scan.ranges[0:round(self.interest_angle/2)] + scan.ranges[round(360-self.interest_angle/2):359]
        new_intensities = scan.intensities[0:round(self.interest_angle/2)] + scan.intensities[round(360-self.interest_angle/2):359]

        # create new messge
        new_scan = LaserScan()
        new_scan.header = scan.header
        new_scan.angle_min = float(360-self.interest_angle/2)
        new_scan.angle_max = float(self.interest_angle/2)
        new_scan.angle_increment = scan.angle_increment
        new_scan.time_increment = scan.time_increment
        new_scan.scan_time: new_scan.scan_time
        new_scan.range_min = min(new_range)
        new_scan.range_max = max(new_range)
        new_scan.ranges = new_range
        new_scan.intensities = new_intensities

        # serialize new ROS2 message
        new_serialized_scan = _rclpy.rclpy_serialize(new_scan, LaserScan)

        # this serializes
        # return _rclpy.rclpy_serialize(ls, type(ls))
        # this deserializes
        # return _rclpy.rclpy_deserialize(serialized_message, message_type)

        await self.output.send(new_serialized_scan)
        return None


def register():
    return LidarProcess
