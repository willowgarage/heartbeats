import rospy
import uuid
import threading
from functools import partial
from msg import Heartbeat, HeartbeatUpdate

#Note: This no_op function serves as a prototype of the callback expected for connect and disconnect
def no_op(node_name, unique_id):
	pass

class HeartbeatEcho:
	def __init__(self, echo_msg):
		self._is_dirty = True
		self._echo_msg = echo_msg

	def get_echo_msg(self):
		return self._echo_msg

	def is_dirty(self):
		return self._is_dirty

	def reset_dirty_bit(self):
		self._is_dirty = False

class HeartbeatNode(threading.Thread):
	def __init__(self, node_name, heartbeat_topic = "heartbeats", publish_rate_in_hertz = 1, connect_cb = no_op, disconnect_cb = no_op):
		self._lock = threading.RLock

		self._node_name = node_name
		self._heartbeat_topic = heartbeat_topic
		self._publish_rate_in_hertz = publish_rate_in_hertz
		self._connect_cb = connect_cb
		self._disconnect_cb = disconnect_cb		
		self._unique_id = uuid.get_uuid4()
		self._sequence_number = -1

		self._latest_incoming_heartbeats = {} #dict unique_id -> Heartbeat msg, these contain the latest sequence numbers of each external node
		self._latest_echos_from_other_nodes = {} #dict unique_id -> HeartbeatEcho object, the object is needed to track a dirty bit connection callbacks
		
		self._heartbeat_publisher = rospy.Publisher(heartbeat_topic)	
		self._heartbeat_subscriber = rospy.Subscriber(heartbeat_topic, cb = partial(self, self.on_incoming_heartbeat_update))

	def get_node_name(self):
		with self._lock:

	def get_publish_rate_in_hertz(self):
		return self._publish_rate_in_hertz

	def get_unique_id(self):
		with self._lock:
			return self._unique_id

	def get_next_sequence_number(self):
		with self._lock:
			self._sequence_number = self._sequence_number + 1
			return self._sequence_number

	def form_outgoing_self_heartbeat(self):
		with self._lock:
			h = Heartbeat()
			h.unique_id = self.get_unique_id()
			h.node_name = self.get_node_name()
			h.publish_rate_in_hertz = self.get_publish_rate_in_hertz()
			h.sequence_number = self.get_next_sequence_number() #Note: This creates a side-effect that increments sequence number
			h.local_utc_timestamp = rospy.get_time()
			return h

	def form_list_of_latest_heartbeat_echos(self):
		with self._lock:
			echoed_heartbeats = []
			for k in self._latest_incoming_heartbeats:
				hb_echo = self._latest_incoming_heartbeats[k]
				echoed_heartbeats.append(hb_echo.get_echo_msg())

	def form_heartbeat_update_for_publication(self):
		with self._lock
			hb_update = HeartbeatUpdate()
			hb_update.unique_id = self.get_unique_id()
			hb_update.echoed_heartbeats = []
			hb_update.echoed_heartbeats.append(self.form_outgoing_self_heartbeat())
			hb_update.echoed_heartbeats = hb_update.echoed_heartbeats + self.form_list_of_latest_heartbeat_echos()

	def find_latest_incoming_heartbeat_for_node(self, node_name):
		with self._lock:
			for k in self._latest_incoming_heartbeats:
				hb_echo = self._latest_incoming_heartbeats[k]
				if(hh_echo.get_echo_msg().node_name == node_name)
					return hb_echo

	def is_connected_to_node(self, node_name):
		with self._lock
			hb_echo = self.find_latest_incoming_heartbeat_for_node(node_name)
			echo_timestamp = hb_echo.get_echo_msg().local_utc_timestamp
			current_time = rospy.get_time()
			current_time - 

	def on_incoming_heartbeat_update(heartbeat_update_msg):
		with self._lock:
			#The idea here is to inspect the incoming update, look for the echo from the heartbeat you emitted
			self_unique_id = self.get_unique_id()
			unique_id = heartbeat_update_msg.unique_id
			echos = heartbeat_update_msg.echoed_heartbeats
			for e in echos:
				if e.unique_id == self_unique_id:
					self._latest_incoming_heartbeats[unique_id] = e
				elseif e.unique_id == self_unique_id
					self._latest_echos_from_other_nodes[unique_id] = HeartbeatEcho(e)
					return
			#If we reached this point, it means that the node that sent the message has not received any heartbeats yet from us, so we don't bother adding an entry for it

	def check_for_timeouts_and_invoke_callbacks_as_necessary(self):
		with self._lock:
			#Note: What's interesting here is that the threshold for a timeout is determined by the publish rate each node. In theory, we should
			#be be matching the publish rate 
			for k in self._latest_incoming_heartbeats:
				heartbeat_echo = self._latest_incoming_heartbeats[k]
				if heartbeat_echo


	def run(self):
		rospy.Rate(self.get_publish_rate_in_hertz())
		self.check_for_timeouts_and_invoke_callbacks_as_necessary()
		r = rospy.Rate(10) # 10hz
   			2 while not rospy.is_shutdown():
   3     pub.publish("hello")
   4     r.sleep()
