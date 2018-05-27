uint8 TYPE_GENERIC = 0
uint8 TYPE_3DR_RADIO = 1
uint8 TYPE_UBIQUITY_BULLET = 2
uint8 TYPE_WIRE = 3
uint8 TYPE_USB = 4
uint8 TYPE_IRIDIUM = 5

uint8 TELEMETRY_STATUS_REMOTE_TYPE_GCS = 4                  # MAV_TYPE_GCS
uint8 TELEMETRY_STATUS_REMOTE_TYPE_ONBOARD_CONTROLLER = 18  # MAV_TYPE_ONBOARD_CONTROLLER

uint8 MAVLINK_MODE_NORMAL = 0
uint8 MAVLINK_MODE_CUSTOM = 1
uint8 MAVLINK_MODE_ONBOARD = 2
uint8 MAVLINK_MODE_OSD = 3
uint8 MAVLINK_MODE_MAGIC = 4
uint8 MAVLINK_MODE_CONFIG = 5
uint8 MAVLINK_MODE_IRIDIUM = 6
uint8 MAVLINK_MODE_MINIMAL = 7

uint8 MAVLINK_DEVICE_UNKNOWN = 0
uint8 MAVLINK_DEVICE_UDP = 1
uint8 MAVLINK_DEVICE_TCP = 2
uint8 MAVLINK_DEVICE_UART = 3


uint64 heartbeat_time			# Time of last received heartbeat from remote system

uint8 channel

uint8 mode				# mavlink mode

uint8 device_type

uint8 link_type			# type of the radio hardware
uint8 remote_type		# type of the remote system (defined in MAVLink MAV_TYPE ENUM)

float32 rate_tx
float32 rate_txerr

float32 rate_rx

float32 rate_mult


uint64 loop_perf
uint64 txerr_perf

bool forwarding
bool flow_control
bool ftp_enabled
bool transmitting

uint8 version



# ftp enabled
# transmitting enabled
# mode
# mavlink version
# transport protocol
#  uart number? 
# topics #?
# version
# update rate?
