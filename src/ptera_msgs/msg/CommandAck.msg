
std_msgs/Header header
string command           # Echoed command, e.g. "find_object"
string type              # Object type, e.g. "apple"
string ack_status        # One of: "accepted", "rejected", "invalid"
string reason            # Optional explanation, e.g. "unsupported object type"
