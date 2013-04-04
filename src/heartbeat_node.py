import rospy
import uuid


class HeartbeatStats:
	def __init__(self):
		self._heartbeats = {}

	def update(heartbeat_msg):
		self._heartbeats[heartbeat_msg.identifier] = heartbeat_msg

	def get_rate


class Heartbeat:
	def __init__(self, connect_cb, disconnect_cb):
		self._connect_cb = connect_cb
		self._disconnect_cb = disconnect_cb
		self._is_connected = False
		self._identifier = uuid.get_uuid4()

	def get_identifer():
		return self._identifier




	def start(self, connect_cb, disconnect_cb):
