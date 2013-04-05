# HeartbeatUpdate.msg
# -------------------
# This message encodes the pulse HeartbeatNodes emit to signal their aliveness and connectivity. The message encodes their identity in the form
# of a unique_id and a list of Heartbeat messages representing echos (i.e acks) of pulses from other nodes. Each Heartbeat message indicates a
# sequence number and local timestamp (we can't trust clocks are synched) which indicates the latest pulse that a node received. The
# HeartbeatNode class uses the echo to compute time elapsed since last update. If the elapsed time exceeds a threshold, it will trigger a callback

string unique_id
Heartbeat[] echoed_heartbeats