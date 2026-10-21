# Full board observation from vision_node.
std_msgs/Header header

# One character per cell: '1' player one, '2' player two, '-' empty.
# Kept for consumers that only need occupancy.
string occupancy

# Per-piece detections, which occupancy cannot express: piece identity and
# true position. Used to correct pick coordinates.
PieceDetection[] pieces

bool  board_located  # were all four corner markers visible this frame
int32 frames_voted   # how many frames this state was agreed across
