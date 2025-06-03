#!/usr/bin/env python3

# Copyright 2024 ICUBE Laboratory, University of Strasbourg
# License: Apache License, Version 2.0
# Author: Thibault Poignonec (thibault.poignonec@gmail.com)

from copy import deepcopy
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from interactive_markers.interactive_marker_server import (
    InteractiveMarkerServer
)
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker
)


class InteractiveCartesianReference(Node):

    def __init__(self):
        super().__init__('interactive_cartesian_reference_node')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/cartesian_reference',
            5
        )

        # create an interactive marker server
        self.marker_server = \
            InteractiveMarkerServer(self, 'interactive_markers')

        # create an interactive marker for our server
        origin_pendulum = np.array([0., -0.2, 2.])
        self.marker_position = origin_pendulum + np.array([0., 0., -2.])
        self.ref_position_3D = deepcopy(self.marker_position)

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "world"
        int_marker.name = "interactive_reference_marker"
        int_marker.description = "Current reference position"
        int_marker.scale = 0.1
        int_marker.pose.position.x = self.marker_position[0]
        int_marker.pose.position.y = self.marker_position[1]
        int_marker.pose.position.z = self.marker_position[2]

        # create a marker
        marker = Marker()
        marker.type = Marker.SPHERE
        # marker.always_visible = True
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.
        marker.color.b = 0.
        marker.color.a = .6

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(marker)
        int_marker.controls.append(visual_control)

        def add_control(x, y, z):
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = x
            control.orientation.y = y
            control.orientation.z = z
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control)

        add_control(1.0, 0.0, 0.0)
        add_control(0.0, 1.0, 0.0)

        self.marker_server.insert(int_marker)
        self.marker_server.setCallback(
            int_marker.name, self.process_marker_feedback)

        # 'commit' changes and send to all clients
        self.marker_server.applyChanges()

        self.Ts = 0.004  # 250 Hz
        self.alpha_filter = 1 - np.exp(-self.Ts / 0.1)
        self.timer = self.create_timer(self.Ts, self.update)

    def update(self):
        # simple low-pass filter
        self.ref_position_3D = self.alpha_filter * self.marker_position + \
            (1 - self.alpha_filter) * self.ref_position_3D

        # publish reference position
        msg = Float64MultiArray()
        ref_position = np.array([
            self.ref_position_3D[0],
            self.ref_position_3D[2]
        ])  # in XZ plane
        msg.data = [
            ref_position[0], ref_position[1],
            0.0, 0.0  # no velocity for now
        ]
        self.publisher_.publish(msg)

    def process_marker_feedback(self, feedback):
        self.marker_position[0] = feedback.pose.position.x
        self.marker_position[1] = feedback.pose.position.y
        self.marker_position[2] = feedback.pose.position.z


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveCartesianReference()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
