# Constants
uint8 TYPE_NEW_RANGE = 0
uint8 TYPE_NEW_DEVICE = 1
uint8 TYPE_INACTIVE_DEVICE = 2

# Default
Header header

# Custom fields
uint8 type
int16 tag_address
int16 anchor_address
float32 range
float32 receive_power
float32 first_path_power
float32 receive_quality
float32 temperature
float32 voltage
uint16 tag_antenna_delay