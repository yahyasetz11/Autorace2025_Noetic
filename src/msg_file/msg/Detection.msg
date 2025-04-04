# Message for sign detection
string sign          # Type of sign detected (e.g., "left", "right", "stop")
float32 confidence   # Confidence level of detection (0.0-1.0)
int32 x              # X coordinate of sign in image
int32 y              # Y coordinate of sign in image
int32 width          # Width of detection bounding box
int32 height         # Height of detection bounding box