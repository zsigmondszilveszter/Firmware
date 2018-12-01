
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v5
	LABEL multicopter
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	IO px4_io-v2_default
	#TESTING
	UAVCAN_INTERFACES 2

	SERIAL_PORTS
		GPS1:/dev/ttyS0
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS3

	DRIVERS
		barometer # all available barometer drivers
		batt_smbus
		blinkm
		camera_trigger
		distance_sensor # all available distance sensor drivers
		gps
		imu/adis16448
		imu/bmi055
		imu/bmi160
		imu/bma180
		imu/mpu6000
		imu/mpu9250
		irlock
		magnetometer # all available magnetometer drivers
		mkblctrl
		oreoled
		pca8574
		pca9685
		pmw3901
		pwm_input
		pwm_out_sim
		px4flow
		px4fmu
		px4io
		rc_input
		rgbled
		rgbled_ncp5623c
		rgbled_pwm
		roboclaw
		stm32
		stm32/adc
		stm32/tone_alarm
		tap_esc
		telemetry # all available telemetry drivers
		test_ppm
		vmount

	MODULES
		attitude_estimator_q
		camera_feedback
		commander
		dataman
		ekf2
		events
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_pos_control
		navigator
		position_estimator_inav
		sensors
		uavcan
		wind_estimator

	SYSTEMCMDS
		bl_update
		config
		dumpfile
		esc_calib
		hardfault_log
		led_control
		mixer
		motor_ramp
		motor_test
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		sd_bench
		shutdown
		#tests # tests and test runner
		top
		topic_listener
		tune_control
		usb_connected
		ver

	)
