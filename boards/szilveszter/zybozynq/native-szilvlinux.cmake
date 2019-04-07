# Szilveszter Zsigmond 2019-15-19
#
#	Input:
#		PLATFORM		: PX4 platform name (posix, nuttx, qurt)
#		VENDOR			: name of board vendor/manufacturer/brand/etc
#		MODEL			: name of board model
#		LABEL			: optional label, set to default if not specified
#		TOOLCHAIN		: cmake toolchain
#		ARCHITECTURE		: name of the CPU CMake is building for (used by the toolchain)
#		ROMFSROOT		: relative path to the ROMFS root directory (currently NuttX only)
#		IO			: name of IO board to be built and included in the ROMFS (requires a valid ROMFSROOT)
#		BOOTLOADER		: bootloader file to include for flashing via bl_update (currently NuttX only)
#		UAVCAN_INTERFACES	: number of interfaces for UAVCAN
#		DRIVERS			: list of drivers to build for this board (relative to src/drivers)
#		MODULES			: list of modules to build for this board (relative to src/modules)
#		SYSTEMCMDS		: list of system commands to build for this board (relative to src/systemcmds)
#		EXAMPLES		: list of example modules to build for this board (relative to src/examples)
#		SERIAL_PORTS		: mapping of user configurable serial ports and param facing name
#		DF_DRIVERS		: list of DriverFramework device drivers (includes DriverFramework driver and wrapper)
#		CONSTRAINED_FLASH	: flag to enable constrained flash options (eg limit init script status text)
#		TESTING			: flag to enable automatic inclusion of PX4 testing modules


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

	DF_DRIVERS # NOTE: DriverFramework is migrating to intree PX4 drivers
		lsm6ds33
		lis3mdl
		bmp180

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
		param
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
)