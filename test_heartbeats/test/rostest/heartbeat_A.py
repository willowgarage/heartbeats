#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from heartbeats.heartbeat_node import HeartbeatNode
import unittest

class HeartbeatTester(unittest.TestCase): #unittest.TestCase
    def on_connect(self, node_name):
        rospy.loginfo("Connected to node %s ", node_name)
        self.assertTrue('HeartbeatB' in node_name)
        #Note: HeartbeatB will disconnect as soon as A connects to it by exitting, which should trigger disconnect
        #and conclude test

    def on_disconnect(self, node_name):
        rospy.loginfo("Disconnected from node %s", node_name)   
    
        if 'HeartbeatB' in node_name:           
            rospy.signal_shutdown('HeartbeatA has detetcted HeartbeatB disconnected so the test is over.')
        else:
            self.fail('Disconnected from some node that is not HeartbeatB, test failed!')
            rospy.signal_shutdown('Test Failed')

    def test_basic_connect_disconnect(self):
        n = HeartbeatNode(heartbeat_topic = "heartbeats", publish_rate_in_hertz = 5, connect_cb = self.on_connect, disconnect_cb = self.on_disconnect, minimum_desired_to_effective_rate_ratio = 0.45)
        n.start()
        while not rospy.is_shutdown(): #Needed for ctrl-c technology
            rospy.sleep(0.1)


if __name__ == "__main__":
    import rostest
    print 'meow1'
    rospy.init_node('HeartbeatA', anonymous=False)
    print 'meow2'
    rostest.rosrun('heartbeats', 'test_heartbeat_basic', HeartbeatTester)
    print 'meow3'
    rospy.loginfo("Heartbeat A script exiting...")
    print 'meow4'
    #rospy.init_node('HeartbeatA', anonymous=False)
    #x = HeartbeatTester()
    #x.test_basic_connect_disconnect()
