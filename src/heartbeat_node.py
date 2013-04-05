import rospy
import uuid
import threading
from functools import partial
from msg import Heartbeat, 

def no_op(node_name, unique_id):
	pass

class HeartbeatNode(threading.Thread):
	def __init__(self, node_name, heartbeat_topic = "heartbeats", connect_cb = no_op, disconnect_cb = no_op):
		self._node_name = node_name
		
		self._heartbeat_topic = heartbeat_topic
		
		self._heartbeat_publisher = rospy.Publisher(heartbeat_topic)	
		self._heartbeat_subscriber = rospy.Subscriber(heartbeat_topic, cb = partial(self, self.on_incoming_heartbeat_update))

		self._connect_cb = connect_cb
		self._disconnect_cb = disconnect_cb		

		self._is_connected = False
		
		self._unique_id = uuid.get_uuid4()
		
		self._sequence_number = -1

		self._other_heartbeats = {} #dict unique_id -> Heartbeat msg list

	def get_unique_id(self):
		return self._unique_id

	def get_next_sequence_number(self):
		self._sequence_number = self._sequence_number + 1
		return self._sequence_number

	def form_heartbeat(self):
		h = Heartbeat()
		h.unique_id = self.get_unique_id()
		h.all_heartbeats = self.form_all_heartbeats()
		return h

	def form_all_heartbeats(self):
		all_heartbeats = []
		return all_heartbeats

	def on_incoming_heartbeat_update(heartbeat_update_msg):
		unique_id = heartbeat_update_msg.unique_id
		if(self._other_heartbeats.contains_key(unique_id))

		_other_heartbeats 

	def run(self):
		#Check for timeouts
		pass
