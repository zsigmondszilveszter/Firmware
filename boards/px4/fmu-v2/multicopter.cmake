
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v2
	LABEL multicopter
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	IO px4_io-v2_default

	SERIAL_PORTS
		GPS1:/dev/ttyS0
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS3

	DRIVERS
		barometer/ms5611
		camera_trigger
		distance_sensor # all available distance sensor drivers
		gps
		imu/l3gd20
		imu/lsm303d
		imu/mpu6000
		imu/mpu9250
		irlock
		magnetometer/hmc5883
		px4flow
		px4fmu
		px4io
		rgbled
		stm32
		stm32/adc
		stm32/tone_alarm
		vmount

	MODULES
		#attitude_estimator_q
		camera_feedback
		commander
		dataman
		ekf2
		events
		land_detector
		landing_target_estimator
		load_mon
		#local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_pos_control
		navigator
		sensors

	SYSTEMCMDS
		bl_update
		#config
		#dumpfile
		#esc_calib
		hardfault_log
		#led_control
		mixer
		#motor_ramp
		#motor_test
		mtd
		#nshterm
		param
		perf
		pwm
		reboot
		#sd_bench
		top
		#topic_listener
		tune_control
		ver

	)
