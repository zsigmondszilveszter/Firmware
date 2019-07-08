# Szilveszter Zsigmond 2019-03-19


add_definitions(
	-D__PX4_POSIX_SZILVESZTER
	-D__PX4_LINUX

	# For DriverFramework
	-D__DF_LINUX
	-D__DF_SZILVESZTER
)

px4_add_board(
	PLATFORM posix
	VENDOR Szilveszter
	MODEL zybozynq7000
	LABEL SzilvLinux

	DRIVERS
		linux_sbus
		linux_pwm_out

	DF_DRIVERS # NOTE: DriverFramework is migrating to intree PX4 drivers
		lsm6ds33
		lis3mdl
		bmp180
		mag3110
		mpu9250

	MODULES
		attitude_estimator_q
		commander
		dataman
		ekf2
		events
		fw_att_control
		fw_pos_control_l1
		gnd_att_control
		gnd_pos_control
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		mavlink
		mc_att_control
		mc_pos_control
		navigator
		sensors

	SYSTEMCMDS
		esc_calib
		led_control
		mixer
		motor_ramp
		param
		perf
		reboot
		shutdown
		top
		topic_listener
		tune_control
		ver

	EXAMPLES
		accel_test
		gyro_test
		baro_test
		mag_test
		szilveszter_logger
)