
# roughly matches Mavlink RADIO_STATUS (#109)

uint8 rssi				# local signal strength
uint8 remote_rssi			# remote signal strength

uint8 txbuf				# how full the tx buffer is as a percentage

uint8 noise				# background noise level
uint8 remote_noise			# remote background noise level

uint16 rxerrors				# receive errors

uint16 fixed				# count of error corrected packets


uint8 system_id				# system id of the remote system
uint8 component_id			# component id of the remote system
