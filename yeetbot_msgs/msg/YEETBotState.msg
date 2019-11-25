# Enum for each possible state the robot can be in

# The idle state, waiting for something to happen
int8 IDLE                   =   0

# States to do with user requests for tools
int8 RECEIVING_REQUEST      =   1

# States to do with returning tools
int8 RECEIVING_TOOL_EARLY   =   2
int8 RECEIVING_TOOL_ON_TIME =   3
int8 RECEIVING_TOOL_LATE    =   4

# States to do with travelling
int8 TRAVELLING             =   5

# ==============================================================
# ==============================================================
# The actual state of the robot
int8 current_state
