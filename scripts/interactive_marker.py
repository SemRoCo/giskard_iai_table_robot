#!/usr/bin/env python

import numpy as np
import actionlib
import rospy
from collections import defaultdict

from actionlib.simple_action_client import SimpleActionClient
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
from giskard_msgs.msg._Controller import Controller
from giskard_msgs.msg._ControllerListAction import ControllerListAction
from giskard_msgs.msg._ControllerListGoal import ControllerListGoal
from giskard_msgs.msg._ControllerListResult import ControllerListResult
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from sensor_msgs.msg._JointState import JointState
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg._InteractiveMarker import InteractiveMarker
from visualization_msgs.msg._InteractiveMarkerControl import InteractiveMarkerControl
from visualization_msgs.msg._InteractiveMarkerFeedback import InteractiveMarkerFeedback
from visualization_msgs.msg._Marker import Marker



class InteractiveMarkerGoal(object):
    def __init__(self, root_link, tip_links):
        # tf
        self.tfBuffer = Buffer(rospy.Duration(1))
        self.tf_listener = TransformListener(self.tfBuffer)

        # giskard goal client
        # self.client = SimpleActionClient('move', ControllerListAction)
        self.client = SimpleActionClient('qp_controller/command', ControllerListAction)
        self.client.wait_for_server()

        # marker server
        self.server = InteractiveMarkerServer("eef_control")
        self.menu_handler = MenuHandler()

        for tip_link in tip_links:
            int_marker = self.make6DofMarker(InteractiveMarkerControl.MOVE_ROTATE_3D, root_link, tip_link)
            self.server.insert(int_marker, self.process_feedback(self.server, self.client, root_link, tip_link))
            self.menu_handler.apply(self.server, int_marker.name)

        self.server.applyChanges()


    def transformPose(self, target_frame, pose, time=None):
        transform = self.tfBuffer.lookup_transform(target_frame,
                                                   pose.header.frame_id,
                                                   pose.header.stamp if time is not None else rospy.Time(0),
                                                   rospy.Duration(1.0))
        new_pose = do_transform_pose(pose, transform)
        return new_pose

    def makeBox(self, msg):
        marker = Marker()

        marker.type = Marker.SPHERE
        marker.scale.x = msg.scale * 0.2
        marker.scale.y = msg.scale * 0.2
        marker.scale.z = msg.scale * 0.2
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.5

        return marker

    def makeBoxControl(self, msg):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.makeBox(msg))
        msg.controls.append(control)
        return control

    def make6DofMarker(self, interaction_mode, root_link, tip_link):
        def normed_q(x,y,z,w):
            return np.array([x,y,z,w])/np.linalg.norm([x,y,z,w])

        int_marker = InteractiveMarker()

        p = PoseStamped()
        p.header.frame_id = tip_link
        p.pose.orientation.w = 1

        int_marker.header.frame_id = tip_link
        int_marker.pose.orientation.w = 1
        int_marker.pose = self.transformPose(root_link, p).pose
        int_marker.header.frame_id = root_link
        int_marker.scale = .2

        int_marker.name = "eef_{}_to_{}".format(root_link, tip_link)

        # insert a box
        self.makeBoxControl(int_marker)
        int_marker.controls[0].interaction_mode = interaction_mode

        if interaction_mode != InteractiveMarkerControl.NONE:
            control_modes_dict = {
                InteractiveMarkerControl.MOVE_3D: "MOVE_3D",
                InteractiveMarkerControl.ROTATE_3D: "ROTATE_3D",
                InteractiveMarkerControl.MOVE_ROTATE_3D: "MOVE_ROTATE_3D"}
            int_marker.name += "_" + control_modes_dict[interaction_mode]

        control = InteractiveMarkerControl()
        control.orientation = Quaternion(0, 0, 0, 1)
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation = Quaternion(0, 0, 0, 1)
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation = Quaternion(*normed_q(0,1,0,1))
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation = Quaternion(*normed_q(0,1,0,1))
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation = Quaternion(*normed_q(0,0,1,1))
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation = Quaternion(*normed_q(0,0,1,1))
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        return int_marker

    class process_feedback(object):
        def __init__(self, i_server, client, root_link, tip_link):
            self.i_server = i_server
            self.client = client
            self.tip_link = tip_link
            self.root_link = root_link

        def __call__(self, feedback):
            if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                print('interactive goal update')
                goal = ControllerListGoal()
                goal.type = ControllerListGoal.STANDARD_CONTROLLER
                # asd = 'asd'
                # translation
                controller = Controller()
                controller.type = Controller.TRANSLATION_3D
                controller.tip_link = self.tip_link
                controller.root_link = self.root_link

                controller.goal_pose.header = feedback.header
                controller.goal_pose.pose = feedback.pose

                controller.p_gain = 3
                controller.enable_error_threshold = True
                controller.threshold_value = 0.05
                controller.weight = 1.0
                goal.controllers.append(controller)

                # rotation
                controller = Controller()
                controller.type = Controller.ROTATION_3D
                controller.tip_link = self.tip_link
                controller.root_link = self.root_link

                controller.goal_pose.header = feedback.header
                controller.goal_pose.pose = feedback.pose

                controller.p_gain = 3
                controller.enable_error_threshold = True
                controller.threshold_value = 0.2
                controller.weight = 1.0
                goal.controllers.append(controller)

                self.client.send_goal(goal)
            self.i_server.applyChanges()

if __name__ == '__main__':
    rospy.init_node('interak')

    interactive_marker_server = InteractiveMarkerGoal('base_link', ['gripper_tool_frame'])
    rospy.spin()
