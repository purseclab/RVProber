"""
Jan 22, 2023
* (completed) To-do list
- make 'rvprober -a GPS_spoofing' (VVV)
- run a simulator with the attack (VVV)
- store the RV's states (VVV)
- run a simulator without the attack (VVV)
- store the RV's states (VVV)
- add other physical states (EKF lane switching, sensor states) (VVV)
- (i) measure the RV's states under the normal, (ii) re-execute SITL, (iii) measure the RV's states under the attack (VVV)
- Test different RV types (e.g., rover, plane, so on), i.e., implementing initial conditions to test the attack (VVV)
- Get filtered positions rather than raw positions (VVV)
- Add 'disarm' state because 'rover' and 'ArduSub' don't have 'landing' stage (VVV)
- Implement 'optical sensor spoofing' attacks (VVV)
- Store execution results in a separated folder (VVV)
- Test different flight modes per each vehicle type (VVV)
* ------------------------------------
* ------------ To-do list ------------
- Compare these states
- Set thresholds to distinguish between benign and attacking symptoms.
- Distinguish between 'failsafe warning' and 'triggering failsafe'
- Implementing 'test different HW configurations (i.e., attaching different sensors into RVs)'
"""

from pymavlink import mavutil
import time
from subprocess import *
import sys, os, getopt
import datetime

master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

RVPROBER_HOME = ""
ARDUPILOT_HOME = ""
logs_path = ""
wipe_eeprom = "off"
intensity_of_attack = 1
current_target_mode = 5
start_flight_mode = 0
start_sw_version = 1
mutate_environment = "off"
selected_attack_profile = ""

#https://ardupilot.org/dev/docs/apmcopter-adding-a-new-flight-mode.html
arducopter_mode_list = [0, 1, 2, 4, 5, 6, 7, 9, 11, 14, 15, 16, 17, 19, 20, 21, 24]
ardurover_mode_list = [0, 1, 3, 4, 5, 7, 11, 12, 15]
ardusub_mode_list = [0, 1, 2, 4, 7, 16, 19]

def open_probing(attack_type, test_time, normal, vehicle_type, sw_path, intensity_of_attack):

	global current_target_mode
	global logs_path
	global mutate_environment

	if normal == "yes":
		if mutate_environment == "off":
			c = 'gnome-terminal -- python2 ' + RVPROBER_HOME + 'ArduPilot/fuzzing.py' + ' -a ' + attack_type + ' -i ' + str(intensity_of_attack) + ' -t ' + str(
				test_time) + ' -n yes' + ' -v ' + vehicle_type + ' -d ' + str(logs_path) + ' -f ' + str(current_target_mode) + ' -s ' + str(sw_path) + ' &'
		elif mutate_environment == "on":
			c = 'gnome-terminal -- python2 ' + RVPROBER_HOME + 'ArduPilot/fuzzing.py' + ' -a ' + attack_type + ' -i ' + str(intensity_of_attack) + ' -t ' + str(
				test_time) + ' -n yes' + ' -v ' + vehicle_type + ' -d ' + str(logs_path) + ' -f ' + str(current_target_mode) + ' -s ' + str(sw_path) + ' -e' + ' &'

	elif normal == "no":
		if mutate_environment == "off":
			c = 'gnome-terminal -- python2 ' + RVPROBER_HOME + 'ArduPilot/fuzzing.py' + ' -a ' + attack_type + ' -i ' + str(intensity_of_attack) + ' -t ' + str(
				test_time) + ' -n no' + ' -v ' + vehicle_type + ' -d ' + str(logs_path) + ' -f ' + str(current_target_mode) + ' -s ' + str(sw_path) + ' &'
		elif mutate_environment == "on":
			c = 'gnome-terminal -- python2 ' + RVPROBER_HOME + 'ArduPilot/fuzzing.py' + ' -a ' + attack_type + ' -i ' + str(intensity_of_attack) + ' -t ' + str(
				test_time) + ' -n no' + ' -v ' + vehicle_type + ' -d ' + str(logs_path) + ' -f ' + str(current_target_mode) + ' -s ' + str(sw_path) + ' -e' + ' &'

	handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
	print(c)
#-------------------------------------------------------------------------------------------------------------
def open_simulator(vehicle_type, mutate_hw, allow_wipe_eeprom, sw_path):
	global RVPROBER_HOME
	global wipe_eeprom

	c = 'gnome-terminal -- python2 ' + RVPROBER_HOME + 'ArduPilot/open_simulator.py' + ' -v ' + str(vehicle_type) + ' -s ' + str(sw_path)

	if wipe_eeprom == "on" and allow_wipe_eeprom is True:
		c = c + ' -w'

	if mutate_hw is True:
		c = c + ' -h'

	c = c + ' &'
	handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
	print(c)

	# Waiting for initializing SITL
	time.sleep(55)

#-------------------------------------------------------------------------------------------------------------
def close_simulator():
	open("shared_variables.txt", "w").close()
	f = open("shared_variables.txt", "w")
	f.write("reboot")
	f.close()

#-------------------------------------------------------------------------------------------------------------
def differential_testing(attack_type, test_time, vehicle_type, probing_target, ardupilot_path, intensity_of_attack):

	while True:
		time.sleep(0.2)

		f = open("simulator_states.txt", "r")
		read_line = f.read()

		if read_line == "Complete under the normal":
			f.close()
			open("simulator_states.txt", "w").close()

			if probing_target == 2:
				open_simulator(vehicle_type, False, False, ardupilot_path)
			else:
				open_simulator(vehicle_type, False, True, ardupilot_path)

			# 2-3) Profile the selected attack under the *attack* conditions
			open_probing(attack_type, test_time, "no", vehicle_type, ardupilot_path, intensity_of_attack)

		elif read_line == "Complete under the attack":
			f.close()
			open("simulator_states.txt", "w").close()
			print("***[Probing %s is completed]***" % vehicle_type)
			break
			time.sleep(3)

		f.close()
#-------------------------------------------------------------------------------------------------------------
def change_parameter_value(name, value, type):
	"""
	ArduPilot only supports MAV_PARAM_TYPE_INT8, MAV_PARAM_TYPE_INT16, MAV_PARAM_TYPE_INT32 and MAV_PARAM_TYPE_REAL32 (aka float) types
	"""
	if type == "int":
		master.mav.param_set_send(master.target_system, master.target_component,
							  name,
							  value,
							  mavutil.mavlink.MAV_PARAM_TYPE_INT32)

	elif type == "float":
		master.mav.param_set_send(master.target_system, master.target_component,
							  name,
							  value,
							  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

#-------------------------------------------------------------------------------------------------------------
def add_range_finder_sensor():
	change_parameter_value('SIM_SONAR_SCALE', 10, "int")
	time.sleep(0.1)
	change_parameter_value('RNGFND1_TYPE', 1, "int")
	time.sleep(0.1)
	change_parameter_value('RNGFND1_SCALING', 10, "int")
	time.sleep(0.1)
	change_parameter_value('RNGFND1_PIN', 0, "int")
	time.sleep(0.1)
	change_parameter_value('RNGFND1_MAX_CM', 5000, "int")
	time.sleep(0.1)
	change_parameter_value('RNGFND1_MIN_CM', 0, "int")
	time.sleep(0.1)

#-------------------------------------------------------------------------------------------------------------
def add_optical_flow_sensor():
	change_parameter_value('SIM_FLOW_ENABLE', 1, "int")
	time.sleep(0.1)
	change_parameter_value('FLOW_TYPE', 10, "int")
	time.sleep(0.1)

	# Fusing optical flow sensor data
	change_parameter_value('EK3_SRC1_VELXY', 5, "int")
	time.sleep(0.1)
	change_parameter_value('EK3_SRC1_POSXY', 0, "int")
	time.sleep(0.1)
	change_parameter_value('EK3_SRC_OPTIONS', 0, "int")
	time.sleep(0.1)
	change_parameter_value('EK3_SRC1_POSZ', 1, "int")
	time.sleep(0.1)
	change_parameter_value('EK3_SRC1_VELZ', 0, "int")
	time.sleep(0.1)
	change_parameter_value('EK3_SRC1_YAW', 1, "int")
	time.sleep(0.1)

	#change_parameter_value('ARMING_CHECK', 0, "int")
	#change_parameter_value('SIM_GPS_TYPE', 2, "int")

#-------------------------------------------------------------------------------------------------------------
def add_gps_for_yaw_sensor():
	change_parameter_value('EK3_SRC1_YAW', 2, "int")
	time.sleep(0.1)
	change_parameter_value('GPS_AUTO_CONFIG', 0, "int")
	time.sleep(0.1)
	change_parameter_value('GPS_TYPE', 17, "int")
	time.sleep(0.1)
	change_parameter_value('GPS_TYPE2', 18, "int")
	time.sleep(0.1)
	change_parameter_value('GPS_POS1_Y', -0.2, "float")
	time.sleep(0.1)
	change_parameter_value('GPS_POS2_Y', 0.2, "float")
	time.sleep(0.1)
	change_parameter_value('SIM_GPS_POS_Y', -0.2, "float")
	time.sleep(0.1)
	change_parameter_value('SIM_GPS2_POS_Y', 0.2, "float")
	time.sleep(0.1)
	change_parameter_value('SIM_GPS2_DISABLE', 0, "int")
	time.sleep(0.1)
	change_parameter_value('SIM_GPS2_HDG', 1, "int")
	time.sleep(0.1)

#-------------------------------------------------------------------------------------------------------------
def create_log_folder():
	global RVPROBER_HOME
	global logs_path

	e = datetime.datetime.now()

	#mode = 0o777
	directory = 'logs/' + str(e.month) + '_' + str(e.day) + '_' + str(e.year) + '_' + str(e.hour) + '_' + str(e.minute) + '_' + str(e.second)

	path = os.path.join(RVPROBER_HOME, directory)
	os.mkdir(path)

	logs_path = RVPROBER_HOME + directory
	print("Directory '%s' (%s) created to store experiment results" % (directory, logs_path))

# -------------------------------------------------------------------------------------------------------------
def main(argv):
	global RVPROBER_HOME
	global ARDUPILOT_HOME
	global wipe_eeprom
	global intensity_of_attack
	global logs_path
	global current_target_mode
	global start_flight_mode
	global start_sw_version
	global mutate_environment
	global selected_attack_profile

	attack_type = ""
	attack_profile = "off"
	test_time = 0
	probing_target = ""
	target_vehicle_type = "ArduCopter"

	# (Start) Parse command line arguments (i.e., input and output file)
	try:
		opts, args = getopt.getopt(argv, "ha:p:t:m:wi:v:f:s:e", ["attack_type=","attack_profile=","test_time=","probing_target=","intensity_of_attack=","vehicle_type=","flight_mode=","sw_version="])

	except getopt.GetoptError:
		print("rvprober.py -a <GPS_spoofing/GPS_jamming> -p")
		sys.exit(2)

	for opt, arg in opts:
		if opt == '-h':
			print("rvprober.py -a <choose one of the following attacks: GPS_spoofing/GPS_jamming> -p (turn on attack profile)")
			sys.exit()
		if opt in ("-a", "--attack_type"):
			print(arg)
			attack_type = arg
		if opt in ("-p", "--attack_profile"):
			attack_profile = "on"
			selected_attack_profile = str(arg)
			if selected_attack_profile == "attack_effect":
				print("[rvprober.py] Profiling the effect of the attack")
			elif selected_attack_profile == "attack_mutate":
				print("[rvprober.py] Mutating attack parameters: (1) # of compromised sensors, (2) duration, (3) intensity, and (4) strategy)")

		if opt in ("-t", "--test_time"):
			test_time = int(arg)
			print("[rvprober.py] User wants to test this attack for %d seconds" % test_time)
		if opt in ("-m", "--probing_target"):
			probing_target = int(arg)
			if probing_target == 1:
				print("[rvprober.py] Probing *types of RVs*")
			elif probing_target == 2:
				print("[rvprober.py] Probing *hardware configurations*")
			elif probing_target == 4:
				print("[rvprober.py] Probing *flight modes*")
			elif probing_target == 5:
				print("[rvprober.py] Probing *software versions*")
			elif probing_target == 6:
				print("[rvprober.py] Probing *configuration parameters*")
			elif probing_target == 7:
				print("[rvprober.py] Probing *environmental conditions*")
			elif probing_target == 10:
				print("[rvprober.py] Probing *all preconditions*")
		if opt == '-w':
			wipe_eeprom = "on"
			print("[rvprober.py] Wipe EEPROM and reload configuration parameters")

		if opt in ("-i", "--intensity_of_attack"):
			intensity_of_attack = float(arg)
			print("[rvprober.py] intensity of attack:%d" % intensity_of_attack)

		if opt in ("-v", "--vehicle_type"):
			target_vehicle_type = str(arg)
			print("[rvprober.py] *%s* will be firstly tested" % target_vehicle_type)

		if opt in ("-f", "--flight_mode"):
			start_flight_mode = int(arg)
			print("[rvprober.py] probing from *%s* flight mode" % start_flight_mode)

		if opt in ("-s", "--sw_version"):
			start_sw_version = int(arg)
			print("[rvprober.py] probing from *%s* software version" % start_sw_version)

		if opt == '-e':
			mutate_environment = "on"
			print("[rvprober.py] mutate environment")

	open("testing_attack_type.txt", "w").close()

	f_attack_type = open("testing_attack_type.txt", "w")
	if attack_type == 'GPS_spoofing':
		print("[rvprober.py] User chooses GPS spoofing attacks")
		f_attack_type.write('GPS_spoofing')
	elif attack_type == 'GPS_jamming':
		print("[rvprober.py] User chooses GPS jamming attack")
		f_attack_type.write('GPS_jamming')
	elif attack_type == 'spoofing_gyro':
		print("[rvprober.py] User chooses gyroscope attack")
		f_attack_type.write('spoofing_gyro')
	elif attack_type == 'spoofing_mag':
		print("[rvprober.py] User chooses magnetometer attack")
		f_attack_type.write('spoofing_mag')
	elif attack_type == 'disconnecting_network':
		print("[rvprober.py] User chooses disconnecting_network attack")
		f_attack_type.write('disconnecting_network')
	elif attack_type == 'spoofing_opticalflow':
		print("[rvprober.py] User chooses opticalflow attack")
		f_attack_type.write('spoofing_opticalflow')

	f_attack_type.close()
	# (End) Parse command line arguments (i.e., input and output file)
	
	#----------------------------------------------------------------------
	#----------------------------------------------------------------------
	#----------------------------------------------------------------------
	RVPROBER_HOME = os.getenv("RVPROBER_HOME")

	if RVPROBER_HOME is None:
	    raise Exception("RVPROBER_HOME environment variable is not set!")

	ARDUPILOT_HOME = os.getenv("ARDUPILOT_HOME")

	if ARDUPILOT_HOME is None:
	    raise Exception("ARDUPILOT_HOME environment variable is not set!")

	open("restart.txt", "w").close()
	open("simulator_states.txt", "w").close()

	if test_time == 0:
		test_time = 60

	# Create a directory to store expriment results
	create_log_folder()

	# To-do
	#-------------------------------------------------------------------------------------------------------------------
	# -------------------------------------------------------------------------------------------------------------------
	# 1-1) When the user chooses 'attack profile' step
	if (attack_profile == "on") and (selected_attack_profile == "attack_effect"):
		print("[rvprober.py] Probing the effect of the attack!")

		# Profile the selected attack under the *normal* conditions
		open_simulator(target_vehicle_type, False, True, "ARDUPILOT_HOME")

		c = 'gnome-terminal -- python2 ' + RVPROBER_HOME + 'ArduPilot/fuzzing.py' + ' -a ' + str(attack_type) \
			+ ' -i ' + str(intensity_of_attack) + ' -p ' + '-t ' + str(test_time) + ' -n yes' + ' -v' \
			+ str(target_vehicle_type) + ' -d ' + str(logs_path) + ' -s ' + "ARDUPILOT_HOME" + ' &'
		handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
		print(c)

		# Wait for finishing the attack profile under the normal conditions
		differential_testing(attack_type, test_time, target_vehicle_type, probing_target, "ARDUPILOT_HOME", intensity_of_attack)

	# 1-2) When the user chooses 'attack mutate' step
	elif (attack_profile == "on") and (selected_attack_profile == "attack_mutate"):
		maximum_fail_cases = 5
		fail_cases_cnt = 0

		# Mutating attack parameters: (1) # of compromised sensors, (2) duration (VVV), (3) intensity (VVV), and (4) strategy)
		while True:

			if intensity_of_attack > 0:
				print("[Current Attack Parameters] duration:%d, intensity:%d, strategy: Normal attack" %(test_time, intensity_of_attack))
			elif intensity_of_attack < 0:
				print("[Current Attack Parameters] duration:%d, intensity:%d, strategy: Hill climbing attack" % (test_time, intensity_of_attack))

			attack_level = intensity_of_attack
			# 1-2-1) Mutating 'intensity of the attack'
			while True:
				print("[Attack Parameters] intensity:%d" % attack_level)

				# Profile the selected attack on different software version
				open_simulator(target_vehicle_type, False, True, "ARDUPILOT_HOME")
				open_probing(attack_type, test_time, "yes", target_vehicle_type, "ARDUPILOT_HOME", attack_level)

				# Run two simulators (one with and without the attack)
				differential_testing(attack_type, test_time, target_vehicle_type, probing_target, "ARDUPILOT_HOME", attack_level)

				f = open("attack_result.txt", "r")
				attack_result = f.read()

				if attack_result == "attack_fail":
					attack_level = attack_level + 1
					fail_cases_cnt = fail_cases_cnt + 1
					print("[Attack Parameters] The attack fail, let's try intensity (%d)." % attack_level)

				elif attack_result == "attack_success":
					print("[Attack Parameters] The attack success, intensity (%d) is enough." % attack_level)
					break

				open("attack_result.txt", "w").close()

				if fail_cases_cnt >= maximum_fail_cases:
					print("[Attack Parameters] Fail to find a proper intensity of the attack :(")
					break

			# 1-2-2) Mutating 'attack duration'
			time.sleep(10)
			attack_duration = test_time
			previous_attack_result = ""
			fail_cases_cnt = 0
			while True:
				print("[Attack Parameters] attack duration:%d" % attack_duration)

				# Profile the selected attack on different software version
				open_simulator(target_vehicle_type, False, True, "ARDUPILOT_HOME")
				open_probing(attack_type, attack_duration, "yes", target_vehicle_type, "ARDUPILOT_HOME", attack_level)

				# Run two simulators (one with and without the attack)
				differential_testing(attack_type, attack_duration, target_vehicle_type, probing_target, "ARDUPILOT_HOME", attack_level)

				f = open("attack_result.txt", "r")
				attack_result = f.read()

				if attack_result == "attack_fail":

					fail_cases_cnt = fail_cases_cnt + 1

					if previous_attack_result == "attack_success":
						print("[Attack Parameters] attack_duration (%d) is the minimum." % (attack_duration * 2))
						break

					previous_attack_result = "attack_fail"
					attack_duration = attack_duration * 2
					print("[Attack Parameters] The attack fail, let's try attack_duration (%d)." % attack_duration)

				elif attack_result == "attack_success":
					if previous_attack_result == "attack_fail":
						print("[Attack Parameters] attack_duration (%d) is the minimum." % attack_duration)
						break

					previous_attack_result = "attack_success"
					attack_duration = attack_duration / 2
					print("[Attack Parameters] The attack success, attack_duration (%d) is enough." % attack_duration)

				open("attack_result.txt", "w").close()

				if fail_cases_cnt >= maximum_fail_cases:
					print("[Attack Parameters] Fail to find a proper attack duration :(")
					break

			# 1-2-3) Mutating 'attack strategy'
			time.sleep(10)
			attack_strategy = intensity_of_attack
			previous_attack_result = ""
			attack_result = ""
			for i in range(1,3):
				if attack_strategy > 0:
					print("[Attack Parameters] strategy: Normal Attack")
				elif attack_strategy < 0:
					print("[Attack Parameters] strategy: Hill climbing Attack")

				# Profile the selected attack on different software version
				open_simulator(target_vehicle_type, False, True, "ARDUPILOT_HOME")
				open_probing(attack_type, test_time, "yes", target_vehicle_type, "ARDUPILOT_HOME", attack_strategy)

				# Run two simulators (one with and without the attack)
				differential_testing(attack_type, test_time, target_vehicle_type, probing_target, "ARDUPILOT_HOME", attack_strategy)

				f = open("attack_result.txt", "r")
				attack_result = f.read()
				attack_strategy = attack_strategy * -1

				if attack_result == "attack_fail":
					previous_attack_result = "attack_fail"
					print("[Attack Parameters] The attack fail, let's try intensity (%d)." % attack_strategy)

				elif attack_result == "attack_success":
					previous_attack_result = "attack_success"
					print("[Attack Parameters] The attack success, intensity (%d) is enough." % attack_strategy)

				open("attack_result.txt", "w").close()

			if attack_result == "attack_success" and previous_attack_result == "attack_success":
				print("[Attack Parameters] The attack is independent on the attack strategy.")
			else:
				print("[Attack Parameters] The attack is dependent on the attack strategy.")

			# End 'attack mutate' step
			break

	# -------------------------------------------------------------------------------------------------------------------
	# -------------------------------------------------------------------------------------------------------------------
	"""
		# 'probing_target': 
		1: Type of an RV
		2: Hardware configuration
		4: Flight mode
		5: Software version
		6: Parameters
		7: Environmental conditions
		10: Test all
	"""

	# 2) When the user chooses 'probing types of RVs'
	vehicle_type_list = []
	if target_vehicle_type == "ArduCopter":
		vehicle_type_list = ['ArduCopter', 'APMrover2', 'ArduSub']
	elif target_vehicle_type == "APMrover2":
		vehicle_type_list = ['APMrover2', 'ArduSub', 'ArduCopter']
	elif target_vehicle_type == "ArduSub":
		vehicle_type_list = ['ArduSub', 'ArduCopter', 'APMrover2']

	if probing_target == 1:
		for vehicle_type in vehicle_type_list:
			print("*[Probing types of RVs] Current selected type: %s*" %vehicle_type)
			# 2-1) Profile the selected attack for each vehicle type under the *normal* conditions
			open_simulator(vehicle_type, False, True, "ARDUPILOT_HOME")
			open_probing(attack_type, test_time, "yes", vehicle_type, "ARDUPILOT_HOME", intensity_of_attack)

			# 2-2) Run two simulators (one with and without the attack)
			differential_testing(attack_type, test_time, vehicle_type, probing_target, "ARDUPILOT_HOME", intensity_of_attack)


	# 3) When the user chooses 'different hardware configurations'
	elif probing_target == 2:
		hw_configuration = ['optical_flow_sensor', 'two_gps_sensors']
		#hw_configuration = ['two_gps_sensors', 'optical_flow_sensor']

		for hw_config in hw_configuration:
			print("*[Probing different HW] Add %s*" % hw_config)

			# 3-1) Add a range finder and optical flow sensors into the RV
			for iteration in range(1,3):
				if iteration == 1:
					open_simulator(target_vehicle_type, True, True, "ARDUPILOT_HOME")
				elif iteration == 2:
					open_simulator(target_vehicle_type, True, False, "ARDUPILOT_HOME")

				master.wait_heartbeat()
				time.sleep(5)

				if hw_config == "optical_flow_sensor":
					add_range_finder_sensor()
					time.sleep(3)
					add_optical_flow_sensor()
					time.sleep(10)
				elif hw_config == "two_gps_sensors":
					add_gps_for_yaw_sensor()
					time.sleep(13)

				print("[Probing different HW] Complete to attach %s (%d/2)" % (hw_config, iteration))

				close_simulator() # For closing the simulator
				time.sleep(5)

			# 3-2) From now on, the RV is equipped with the opticalflow sensor.
			open_simulator(target_vehicle_type, True, False, "ARDUPILOT_HOME")

			if hw_config == "optical_flow_sensor":
				add_optical_flow_sensor()
			elif hw_config == "two_gps_sensors":
				add_gps_for_yaw_sensor()

			# 3-3) Run two simulators (one with and without the attack)
			open_probing(attack_type, test_time, "yes", target_vehicle_type, "ARDUPILOT_HOME", intensity_of_attack)
			differential_testing(attack_type, test_time, target_vehicle_type, probing_target, "ARDUPILOT_HOME", intensity_of_attack)

	# 4) When the user chooses 'flight modes'
	elif probing_target == 4:
		mode_list = []
		if target_vehicle_type == "ArduCopter":
			mode_list = arducopter_mode_list
		elif target_vehicle_type == "APMrover2":
			mode_list = ardurover_mode_list
		elif target_vehicle_type == "ArduSub":
			mode_list = ardusub_mode_list

		for mode in mode_list:
			if mode >= start_flight_mode:
				current_target_mode = mode
				print("*[Probing flight modes] Current selected mode: %d*" % mode)
				# 4-1) Profile the selected attack for a flight mode under the *normal* conditions
				open_simulator(target_vehicle_type, False, True, "ARDUPILOT_HOME")
				open_probing(attack_type, test_time, "yes", target_vehicle_type, "ARDUPILOT_HOME", intensity_of_attack)

				# 2-2) Run two simulators (one with and without the attack)
				differential_testing(attack_type, test_time, target_vehicle_type, probing_target, "ARDUPILOT_HOME", intensity_of_attack)

	# 5) When the user chooses 'software versions'
	elif probing_target == 5:
		version_cnt = 1
		ardupilot_path = ""

		if start_sw_version > version_cnt:
			version_cnt = start_sw_version

		while True:

			if target_vehicle_type == "ArduCopter":
				ardupilot_path = "ARDUCOPTER_"
			elif target_vehicle_type == "APMrover2":
				ardupilot_path = "ARDUROVER_"
			elif target_vehicle_type == "ArduSub":
				ardupilot_path = "ARDUSUB_"

			ardupilot_path = ardupilot_path + str(version_cnt)

			if os.getenv(ardupilot_path) is None:
				print("[DEBUG] There is no environment variable!")
				break
			else:
				version_cnt = version_cnt + 1

			print("*[Probing software versions] Current version: %s (%s)*" % (ardupilot_path, os.getenv(ardupilot_path)))
			# 5-1) Profile the selected attack on different software version
			open_simulator(target_vehicle_type, False, True, ardupilot_path)
			open_probing(attack_type, test_time, "yes", target_vehicle_type, ardupilot_path, intensity_of_attack)

			# 5-2) Run two simulators (one with and without the attack)
			differential_testing(attack_type, test_time, target_vehicle_type, probing_target, ardupilot_path, intensity_of_attack)




if __name__ == "__main__":
   main(sys.argv[1:])