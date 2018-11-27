uint64 timestamp		# time since system start (microseconds)

# filter control status to indicate ekf logic state
bool status_tilt_align				# 0 - true if the filter tilt alignment is complete
bool status_yaw_align				# 1 - true if the filter yaw alignment is complete
bool status_gps						# 2 - true if gps measurements are being fused
bool status_opt_flow				# 3 - true if optical flow measurements are being fused
bool status_mag_hdg					# 4 - true if a simple magnetic yaw heading is being fused
bool status_mag_3d					# 5 - true if 3-axis magnetometer measurement are being fused
bool status_mag_dec					# 6 - true if synthetic magnetic declination measurements are being fused
bool status_in_air					# 7 - true when the vehicle is airborne
bool status_wind					# 8 - true when wind velocity is being estimated
bool status_baro_hgt				# 9 - true when baro height is being fused as a primary height reference
bool status_rng_hgt					# 10 - true when range finder height is being fused as a primary height reference
bool status_gps_hgt					# 11 - true when gps height is being fused as a primary height reference
bool status_ev_pos					# 12 - true when local position data from external vision is being fused
bool status_ev_yaw					# 13 - true when yaw data from external vision measurements is being fused
bool status_ev_hgt					# 14 - true when height data from external vision measurements is being fused
bool status_fuse_beta				# 15 - true when synthetic sideslip measurements are being fused
bool status_update_mag_states_only	# 16 - true when only the magnetometer states are updated by the magnetometer
bool status_fixed_wing				# 17 - true when the vehicle is operating as a fixed wing vehicle
bool status_mag_fault				# 18 - true when the magnetomer has been declared faulty and is no longer being used
bool status_fuse_aspd				# 19 - true when airspeed measurements are being fused
bool status_gnd_effect				# 20 - true when protection from ground effect induced static pressure rise is active
bool status_rng_stuck				# 21 - true when rng data wasn't ready for more than 10s and new rng values haven't changed enough
bool status_gps_yaw					# 22 - true when yaw (not ground course) data from a gps receiver is being fused

# fault status
bool bad_mag_x			# 0 - true if the fusion of the magnetometer X-axis has encountered a numerical error
bool bad_mag_y			# 1 - true if the fusion of the magnetometer Y-axis has encountered a numerical error
bool bad_mag_z			# 2 - true if the fusion of the magnetometer Z-axis has encountered a numerical error
bool bad_hdg			# 3 - true if the fusion of the heading angle has encountered a numerical error
bool bad_mag_decl		# 4 - true if the fusion of the magnetic declination has encountered a numerical error
bool bad_airspeed		# 5 - true if fusion of the airspeed has encountered a numerical error
bool bad_sideslip		# 6 - true if fusion of the synthetic sideslip constraint has encountered a numerical error
bool bad_optflow_x		# 7 - true if fusion of the optical flow X axis has encountered a numerical error
bool bad_optflow_y		# 8 - true if fusion of the optical flow Y axis has encountered a numerical error
bool bad_vel_n			# 9 - true if fusion of the North velocity has encountered a numerical error
bool bad_vel_e			# 10 - true if fusion of the East velocity has encountered a numerical error
bool bad_vel_d			# 11 - true if fusion of the Down velocity has encountered a numerical error
bool bad_pos_n			# 12 - true if fusion of the North position has encountered a numerical error
bool bad_pos_e			# 13 - true if fusion of the East position has encountered a numerical error
bool bad_pos_d			# 14 - true if fusion of the Down position has encountered a numerical error
bool bad_acc_bias		# 15 - true if bad delta velocity bias estimates have been detected

# innovation check flags to indicate pass/fail status of innovation consistency checks
bool reject_vel_ned		# 0 - true if velocity observations have been rejected
bool reject_pos_ne		# 1 - true if horizontal position observations have been rejected
bool reject_pos_d		# 2 - true if true if vertical position observations have been rejected
bool reject_mag_x		# 3 - true if the x magnetometer observation has been rejected
bool reject_mag_y		# 4 - true if the y magnetometer observation has been rejected
bool reject_mag_z		# 5 - true if the z magnetometer observation has been rejected
bool reject_yaw			# 6 - true if the yaw observation has been rejected
bool reject_airspeed	# 7 - true if the airspeed observation has been rejected
bool reject_sideslip	# 8 - true if the synthetic sideslip observation has been rejected
bool reject_hagl		# 9 - true if the height above ground observation has been rejected
bool reject_optflow_x	# 10 - true if the x optical flow observation has been rejected
bool reject_optflow_y	# 11 - true if the y optical flow observation has been rejected

# status of various gps quality checks
bool gps_check_fail_gps_fix				# 0 : insufficient fix type (no 3d solution)
bool gps_check_fail_min_sat_count		# 1 : minimum required sat count fail
bool gps_check_fail_min_gdop			# 2 : minimum required gdop fail
bool gps_check_fail_max_horz_err		# 3 : maximum allowed horizontal position error fail
bool gps_check_fail_max_vert_err		# 4 : maximum allowed vertical position error fail
bool gps_check_fail_max_spd_err			# 5 : maximum allowed speed error fail
bool gps_check_fail_max_horz_drift		# 6 : maximum allowed horizontal position drift fail - requires stationary vehicle
bool gps_check_fail_max_vert_drift		# 7 : maximum allowed vertical position drift fail - requires stationary vehicle
bool gps_check_fail_max_horz_spd_err	# 8 : maximum allowed horizontal speed fail - requires stationary vehicle
bool gps_check_fail_max_vert_spd_err	# 9 : maximum allowed vertical velocity discrepancy fail

bool pre_flight_horizontal_failure	# true when estimator has failed pre-flight checks and the vehicle should not be flown regardless of flight mode
bool pre_flight_vertical_failure	# true when estimator has failed pre-flight checks and the vehicle should not be flown regardless of flight mode
