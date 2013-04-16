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
import uuid
import threading
from msg import Heartbeat, HeartbeatUpdate

def default_on_disconnect(node_name):
	"""
	This no_op function serves as a prototype of the callback expected for disconnect event
	"""
	rospy.loginfo("Disconnected from node %s.", node_name)

def default_on_connect(node_name):
	"""
	This no_op function serves as a prototype of the callback expected for connect event
	"""
	rospy.loginfo("Connected to node %s.", node_name)
 
"""
class HeartbeatWithDirtyBit
---------------------------
* Encapsulates a Heartbeat msg along with a dirty bit (i.e. flag) to indicate if
* connect callback has been invoked
"""
class HeartbeatWithDirtyBit:
	def __init__(self, heartbeat_msg, is_dirty):
		self._is_dirty = is_dirty
		self._heartbeat_msg = heartbeat_msg

	def get_heartbeat_msg(self):
		return self._heartbeat_msg

	def is_dirty(self):
		return self._is_dirty

	def mark_as_dirty(self):
		self._is_dirty = True

"""
class HeartbeatNode
-------------------
* Encapsulates a beacon that broadcasts a signal indicating it's alive over a heartbeat_topic
* All subscribes to heartbeat_topic and intercepts heartbeats of other nodes to determine if alive
* Keeps track of latest heartbeats from each node so as to echo itt back to them in its own heartbeat broadcasts
* Invokes callbacks when disconnects and connects happen from nodes
"""

class HeartbeatNode(threading.Thread):
	def __init__(self, heartbeat_topic = "heartbeats", publish_rate_in_hertz = 1, connect_cb = default_on_connect, disconnect_cb = default_on_disconnect, minimum_desired_to_effective_rate_ratio = 0.25):
		"""
		The constructor for this class
		heartbeat_topic - The topic on which all nodes broadcast and listen for heartbeats
		publish_rate_in_hertz - The rate this node desired to broadcast pulses at
		connect_cb - The callback invoked when a connection a node occurs
		disconnect_cb - The callback invoked when disconnection from node occurs
		minimum_desired_to_effective_rate_ratio - The ratio we are willing to accept in hearing the pulse echos to qualify a connection, by default 0.25 (i.e. 25 percent of desired rate)
		"""
		threading.Thread.__init__(self)
		self._lock = threading.RLock()

		self._node_name = rospy.get_name()
		self._heartbeat_topic = heartbeat_topic
		self._publish_rate_in_hertz = publish_rate_in_hertz
		self._connect_cb = connect_cb
		self._disconnect_cb = disconnect_cb		
		self._sequence_number = -1
		self._latest_incoming_heartbeats = {} #dict node_name -> Heartbeat msg, these contain the latest sequence numbers of each external node
		self._latest_echos_from_other_nodes = {} #dict node_name -> HeartbeatWithDirtyBit object, the object is needed to track a dirty bit connection callbacks
		
		#Publish and subscribe to same topic to push out heartbeats and hear echos
		self._heartbeat_publisher = rospy.Publisher(heartbeat_topic, HeartbeatUpdate)	
		self._heartbeat_subscriber = rospy.Subscriber(heartbeat_topic, HeartbeatUpdate, self.on_incoming_heartbeat_update)

		self._time_of_last_publish_in_utc_seconds = 0
		self._minimum_desired_to_effective_rate_ratio = minimum_desired_to_effective_rate_ratio

	def get_minimum_desired_to_effective_rate_ratio(self):
		"""
		Tolerance ratio of effective (actual) rate at which echos are received versus the rate at which we're publishing them.
		Ideally we want this ratio to be 1.0 or higher meaning the echo rate is fast as or faster than we're publishing.
		In reality it will be lower particularly in degraded lossy networked (e.g. wireless). This threshold sets the bar for
		what qualifies as a "disconnect". For example, 0.5 means you're willing to accept that echos come in at a rate of 50%
		of the heartbeat publish rate.
		"""
		return self._minimum_desired_to_effective_rate_ratio

	def get_publish_rate_in_hertz(self):
		"""
		The rate at which heartbeats are broadcast
		"""
		return self._publish_rate_in_hertz

	def get_node_name(self):
		"""
		The name of this node == rospy.get_name()
		"""
		with self._lock:
			return self._node_name

	def get_next_sequence_number(self):
		"""
		Returns next sequence number used to tag heartbeat broadcasts
		"""
		with self._lock:
			self._sequence_number = self._sequence_number + 1
			return self._sequence_number

	def on_incoming_heartbeat_update(self, heartbeat_update_msg):
		"""
		The callback for the subscriber on the heartbeat topic. Used to determine connections to other nodes and to echo back to them their beats
		"""
		if not heartbeat_update_msg.node_name == self.get_node_name():
			with self._lock:
				self_node_name = self.get_node_name()
				incoming_node_name = heartbeat_update_msg.node_name
				echos = heartbeat_update_msg.echoed_heartbeats
				for e in echos:
					if e.node_name == incoming_node_name: #This contains the latest sequence number from the other noe
						self._latest_incoming_heartbeats[incoming_node_name] = e
					elif e.node_name == self_node_name: #This is the reflection from the other node
						if(self._latest_echos_from_other_nodes.has_key(incoming_node_name)):
							last_echo = self._latest_echos_from_other_nodes[incoming_node_name]
							self._latest_echos_from_other_nodes[incoming_node_name] = HeartbeatWithDirtyBit(e, is_dirty=last_echo.is_dirty())
						else:
							self._latest_echos_from_other_nodes[incoming_node_name] = HeartbeatWithDirtyBit(e, is_dirty=False)

		#else:
		#	rospy.loginfo(self.get_label() + "Ignoring own heartbeat emission.")

	#Connection Establish/Timeout Event Handling
	#==========================================================================	
	def is_connected_to_node(self, node_name):
		"""
		Indicates if a node is alive. This is implied if there is an echo in the latest echos dictionary
		"""
		with self._lock:
			return self._latest_echos_from_other_nodes.has_key(node_name)

	def check_for_timeouts_and_invoke_callbacks_as_necessary(self):
		"""
		Iterate through echos from each node. Those that have time difference > 1 / frequency * some constant means disconnect, less means connect.
		On disconnect, we remove the echo from the list to signify connection breakage. Callbacks are invoked here.
		"""
		with self._lock:
			all_echos = self._latest_echos_from_other_nodes
			keys_to_purge_after_loop = []
			minimum_desired_to_effective_rate_ratio = self.get_minimum_desired_to_effective_rate_ratio()
			for k in all_echos:
				hb_echo_obj = self._latest_echos_from_other_nodes[k]
				effective_rate = 1 / (rospy.get_time() - hb_echo_obj.get_heartbeat_msg().local_utc_timestamp)
				desired_rate = self.get_publish_rate_in_hertz()				
				ratio_effective_to_desired = effective_rate / desired_rate
				#rospy.loginfo(self.get_label() + "effective = %f, desired = %f, ratio = %f", effective_rate, desired_rate, ratio_effective_to_desired)
				has_timedout = ratio_effective_to_desired < minimum_desired_to_effective_rate_ratio
				if has_timedout:
					keys_to_purge_after_loop.append(k)
					self._disconnect_cb(k) #invoke on_disconnect()
				elif not has_timedout and not hb_echo_obj.is_dirty():
					hb_echo_obj.mark_as_dirty()
					self._connect_cb(k) #invoke on_connect() 
			for k in keys_to_purge_after_loop:
				self._latest_echos_from_other_nodes.pop(k)



	#Outgoing Heartbeat Pulse
	#==========================================================================
	def form_outgoing_self_heartbeat(self):
		"""
		This has the heartbeat info information for this node with latest sequence number 
		and time of publish
		"""
		with self._lock:
			h = Heartbeat()
			h.node_name = self.get_node_name()
			h.publish_rate_in_hertz = self.get_publish_rate_in_hertz()
			h.sequence_number = self.get_next_sequence_number() #Note: This creates a side-effect that increments sequence number
			h.local_utc_timestamp = rospy.get_time()
			return h

	def form_list_of_latest_heartbeat_echos(self):
		"""
		Takes the latest incoming heartbeat values and puts them into a list that 
		will be echoed back to those nodes
		"""
		with self._lock:
			echoed_heartbeats = []
			for k in self._latest_incoming_heartbeats:
				hb_echo_msg = self._latest_incoming_heartbeats[k]
				echoed_heartbeats.append(hb_echo_msg)
			return echoed_heartbeats

	def form_heartbeat_update_for_publication(self):
		"""
		Populates a HeartbeatUpdate.msg structure filled with echos for all nodes
		that this node has heard from as well as the latest heartbeat pulse from
		this node to others.
		"""
		with self._lock:
			hb_update = HeartbeatUpdate()
			hb_update.node_name = self.get_node_name()
			hb_update.echoed_heartbeats = []
			hb_update.echoed_heartbeats.append(self.form_outgoing_self_heartbeat())
			hb_update.echoed_heartbeats = hb_update.echoed_heartbeats + self.form_list_of_latest_heartbeat_echos()
			return hb_update

	def find_fastest_heartbeat_of_all_known_heartbeat_nodes(self):
		"""
		Determines the faster publish rate of all known heartbeat nodes. This is
		so as to push out updates fast enough so external node doesn't timeout with
		this node
		"""
		with self._lock: 
			fastest_publish_rate = 0
			for k in self._latest_incoming_heartbeats:
				candidate_rate = self._latest_incoming_heartbeats[k].publish_rate_in_hertz
				if candidate_rate > fastest_publish_rate:
					fastest_publish_rate = candidate_rate
			return fastest_publish_rate

	def push_out_heartbeat_update_if_necessary(self):
		"""
		Sends out the pulse at self.publish_rate_in_hertz().
		TODO: The heartbeat is pushed out not necessarily at publish rate, 
		but at the rate of the fastest node in the network. This is so as 
		to guarantee heartbeats are echoed fast enough otherwise faster publishers 
		would disconnect from slower publishes.	We could redesign the protocol to 
		have separate messages for echos to eliminate this limitation, but fine 
		for now.
		"""
		with self._lock:
			fastest_heartbeat_rate = self.find_fastest_heartbeat_of_all_known_heartbeat_nodes()
			delay_between_messages = 1.0 / max(self.get_publish_rate_in_hertz(), fastest_heartbeat_rate) #Hope my math is ok here: msg_transmit_duration = 1 / f
			current_time = rospy.get_time()
			if (current_time - self._time_of_last_publish_in_utc_seconds > delay_between_messages):
				self._heartbeat_publisher.publish(self.form_heartbeat_update_for_publication())
				self._time_of_last_publish_in_utc_seconds = current_time

	#Driving thread
	#==========================================================================
	def run(self):
		"""
		Thread entry point for HeartbeatNode which drives heartbeat as well as connect
		and disconnect callbacks
		"""
		rospy.loginfo(self.get_label() + "Starting up heartbeat beacon...")
		r = rospy.Rate(150) #150 Hz should be more than enough...famous last words
		while not rospy.is_shutdown():			
			self.check_for_timeouts_and_invoke_callbacks_as_necessary()
			self.push_out_heartbeat_update_if_necessary()
			r.sleep()
		rospy.loginfo(self.get_label() + "Heartbeat beacon shutdown.")

	def get_label(self):
		return "HeartbeatNode (" + self.get_node_name() + "): "