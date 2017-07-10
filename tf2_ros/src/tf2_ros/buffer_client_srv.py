#! /usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#* 
#* Author: Eitan Marder-Eppstein
#***********************************************************
PKG = 'tf2_ros'
import roslib; roslib.load_manifest(PKG)
import rospy
import tf2_py as tf2
import tf2_ros

from tf2_msgs.srv import LookupTransform, LookupTransformRequest, LookupTransformRequest
from actionlib_msgs.msg import GoalStatus

class BufferClientSrv(tf2_ros.BufferInterface):
    """
    Service client-based implementation of BufferInterface.
    """
    def __init__(self, service_name='/tf_buffer/lookup_transform', persistent=True):
        """
        .. function:: __init__(ns, check_frequency = 10.0, timeout_padding = rospy.Duration.from_sec(2.0))

            Constructor.

            :param ns: The namespace in which to look for a BufferServer.
            :param check_frequency: How frequently to check for updates to known transforms.
            :param timeout_padding: A constant timeout to add to blocking calls.
        """
        tf2_ros.BufferInterface.__init__(self)
        self.client = rospy.ServiceProxy('/tf_buffer/lookup_transform', LookupTransform, persistent=persistent)

    def wait_for_server(self, timeout = rospy.Duration()):
        """
        Block until the action server is ready to respond to requests. 

        :param timeout: Time to wait for the server.
        :return: True if the server is ready, false otherwise.
        :rtype: bool
        """
        return self.client.wait_for_service(timeout)

    # lookup, simple api 
    def lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
        """
        Get the transform from the source frame to the target frame.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest) 
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        req = LookupTransformRequest()
        req.target_frame = target_frame;
        req.source_frame = source_frame;
        req.source_time = time;
        req.timeout = timeout;
        req.advanced = False;
        res = self.client(req)
        return res.transform

    # lookup, advanced api 
    def lookup_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout=rospy.Duration(0.0)):
        """
        Get the transform from the source frame to the target frame using the advanced API.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (0 will get the latest) 
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest) 
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :return: The transform between the frames.
        :rtype: :class:`geometry_msgs.msg.TransformStamped`
        """
        req = LookupTransformRequest()
        req.target_frame = target_frame
        req.source_frame = source_frame
        req.source_time = source_time
        req.timeout = timeout
        req.target_time = target_time
        req.fixed_frame = fixed_frame
        req.advanced = True

        return self.__process_goal(goal)

    # can, simple api
    def can_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
        """
        Check if a transform from the source frame to the target frame is possible.

        :param target_frame: Name of the frame to transform into.
        :param source_frame: Name of the input frame.
        :param time: The time at which to get the transform. (0 will get the latest) 
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :param return_debug_type: (Optional) If true, return a tuple representing debug information.
        :return: True if the transform is possible, false otherwise.
        :rtype: bool
        """
        try:
            self.lookup_transform(target_frame, source_frame, time, timeout)
            return True
        except tf2.TransformException:
            return False

    
    # can, advanced api
    def can_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame, timeout=rospy.Duration(0.0)):
        """
        Check if a transform from the source frame to the target frame is possible (advanced API).

        Must be implemented by a subclass of BufferInterface.

        :param target_frame: Name of the frame to transform into.
        :param target_time: The time to transform to. (0 will get the latest) 
        :param source_frame: Name of the input frame.
        :param source_time: The time at which source_frame will be evaluated. (0 will get the latest) 
        :param fixed_frame: Name of the frame to consider constant in time.
        :param timeout: (Optional) Time to wait for the target frame to become available.
        :param return_debug_type: (Optional) If true, return a tuple representing debug information.
        :return: True if the transform is possible, false otherwise.
        :rtype: bool
        """
        try:
            self.lookup_transform_full(target_frame, target_time, source_frame, source_time, fixed_frame, timeout)
            return True
        except tf2.TransformException:
            return False

