uint64 timestamp		# time since system start (microseconds)

float32[24] states		# Internal filter states
uint8 n_states		# Number of states effectively used

uint8 STATE_QUAT_X = 0
uint8 STATE_QUAT_Y = 1
uint8 STATE_QUAT_Z = 2
uint8 STATE_QUAT_W = 3
uint8 STATE_VEL_N = 4
uint8 STATE_VEL_E = 5
uint8 STATE_VEL_D = 6
uint8 STATE_POS_N = 7
uint8 STATE_POS_E = 8
uint8 STATE_POS_D = 9
uint8 STATE_GYRO_DELTA_ANG_BIAS_X = 10
uint8 STATE_GYRO_DELTA_ANG_BIAS_Y = 11
uint8 STATE_GYRO_DELTA_ANG_BIAS_Z = 12
uint8 STATE_ACCEL_DELTA_VEL_BIAS_X = 13
uint8 STATE_ACCEL_DELTA_VEL_BIAS_Y = 14
uint8 STATE_ACCEL_DELTA_VEL_BIAS_Z = 15
uint8 STATE_EARTH_MAG_X = 16
uint8 STATE_EARTH_MAG_Y = 17
uint8 STATE_EARTH_MAG_Z = 18
uint8 STATE_MAG_BIAS_X = 19
uint8 STATE_MAG_BIAS_Y = 20
uint8 STATE_MAG_BIAS_Z = 21
uint8 STATE_WIND_N = 22
uint8 STATE_WIND_E = 23

float32[3] vibe			# IMU vibration metrics in the following array locations
# 0 : Gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
# 1 : Gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
# 2 : Accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)

float32[24] covariances	# Diagonal Elements of Covariance Matrix

float32 pos_horiz_accuracy # 1-Sigma estimated horizontal position accuracy relative to the estimators origin (m)
float32 pos_vert_accuracy # 1-Sigma estimated vertical position accuracy relative to the estimators origin (m)

float32 mag_test_ratio # ratio of the largest magnetometer innovation component to the innovation test limit
float32 vel_test_ratio # ratio of the largest velocity innovation component to the innovation test limit
float32 pos_test_ratio # ratio of the largest horizontal position innovation component to the innovation test limit
float32 hgt_test_ratio # ratio of the vertical position innovation to the innovation test limit
float32 tas_test_ratio # ratio of the true airspeed innovation to the innovation test limit
float32 hagl_test_ratio # ratio of the height above ground innovation to the innovation test limit
float32 beta_test_ratio # ratio of the synthetic sideslip innovation to the innovation test limit

float32 time_slip # cumulative amount of time in seconds that the EKF inertial calculation has slipped relative to system time

# legacy local position estimator (LPE) flags
uint8 health_flags		# Bitmask to indicate sensor health states (vel, pos, hgt)
uint8 timeout_flags		# Bitmask to indicate timeout flags (vel, pos, hgt)
