"""
export RVPROBER_PX4_HOME=/home/hskim/RVProber/RVProber_PX4/
export PX4_HOME=/home/hskim/PX4/px4_1_15_0/
"""

from pymavlink import mavutil
import time
from subprocess import *
import sys, os, getopt
import datetime
import random

connection_string = '0.0.0.0:14540'
master = mavutil.mavlink_connection('udp:' + connection_string)

RVPROBER_PX4_HOME = ""
PX4_HOME = ""
logs_path = ""
intensity_of_attack = 1
current_target_mode = 5
start_flight_mode = 0
start_sw_version = ""
mutate_env_intensity = 0
mutate_env_unit = 0
selected_attack_profile = ""
number_of_compromised_sensor = 0

PX4_mode_list = ["manual", "stabilized", "acro", "altitude", "position", "hold", "mission", "return", "follow_me"]
Default_flight_mode = "mission"

# -------------------------------------------------------------------------------------------------------------
def add_noise_to_gazebo(attack_type, number_of_compromised_sensor, intensity_of_attack, mutate_env_intensity, sw_path):


	c = 'gnome-terminal -- python2 ' + RVPROBER_PX4_HOME + 'noise_gazebo.py' + ' -a ' + attack_type + ' -n ' + str(number_of_compromised_sensor) \
			+ ' -i ' + str(intensity_of_attack) + ' -s ' + str(sw_path) + ' -e ' + str(mutate_env_intensity) + ' &'

	handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
	print(c)

#-------------------------------------------------------------------------------------------------------------
def open_simulator(vehicle_type, mutate_hw, sw_path):
	global RVPROBER_PX4_HOME

	#print(RVPROBER_PX4_HOME)

	c = 'gnome-terminal -- python2 ' + RVPROBER_PX4_HOME + 'open_simulator.py' + ' -v ' + str(vehicle_type) + ' -s ' + str(sw_path)

	if mutate_hw is True:
		c = c + ' -h'

	c = c + ' &'
	handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
	print(c)

	# Waiting for initializing Gazebo simulator
	time.sleep(30)

#-------------------------------------------------------------------------------------------------------------
def open_probing(target_vehicle_type, test_time, flight_mode, selected_config_param):

	if selected_config_param == "":
		c = 'gnome-terminal -- python2 ' + RVPROBER_PX4_HOME + 'fuzzing.py' + ' -t ' + str(test_time) + ' -v ' \
			+ str(target_vehicle_type) + ' -f ' + str(flight_mode) + ' &'
	else:
		c = 'gnome-terminal -- python2 ' + RVPROBER_PX4_HOME + 'fuzzing.py' + ' -t ' + str(test_time) + ' -v ' \
			+ str(target_vehicle_type) + ' -f ' + str(flight_mode) + ' -c ' + str(selected_config_param) + ' &'

	handle = Popen(c, stdin=PIPE, stderr=PIPE, stdout=PIPE, shell=True)
	print(c)

#-------------------------------------------------------------------------------------------------------------
def close_simulator():
	open("shared_variables.txt", "w").close()
	f = open("shared_variables.txt", "w")
	f.write("reboot")
	f.close()

#-------------------------------------------------------------------------------------------------------------
def create_log_folder():
	global RVPROBER_PX4_HOME
	global logs_path

	e = datetime.datetime.now()

	#mode = 0o777
	directory = 'logs/' + str(e.month) + '_' + str(e.day) + '_' + str(e.year) + '_' + str(e.hour) + '_' + str(e.minute) + '_' + str(e.second)

	path = os.path.join(RVPROBER_PX4_HOME, directory)
	os.mkdir(path)

	logs_path = RVPROBER_PX4_HOME + directory
	print("Directory '%s' (%s) created to store experiment results" % (directory, logs_path))
#-------------------------------------------------------------------------------------------------------------
def wait_for_simulation_result():
	while True:
		f = open("restart.txt", "r")
		if f.read() == "restart":
			break

		time.sleep(0.1)
# -------------------------------------------------------------------------------------------------------------
def main(argv):
	global RVPROBER_PX4_HOME
	global PX4_HOME
	global intensity_of_attack
	global logs_path
	global current_target_mode
	global start_flight_mode
	global start_sw_version
	global mutate_env_intensity
	global selected_attack_profile
	global number_of_compromised_sensor
	global mutate_env_unit

	attack_type = ""
	attack_profile = "off"
	test_time = 0
	probing_target = -1
	target_vehicle_type = "ArduCopter"

	# (Start) Parse command line arguments (i.e., input and output file)
	try:
		opts, args = getopt.getopt(argv, "ha:p:t:m:wi:v:f:s:e:n:", ["attack_type=","attack_profile=","test_time=","probing_target=","intensity_of_attack=","vehicle_type=","flight_mode=","sw_version=","env_intensity=","number_of_compromised_sensor="])

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
			start_sw_version = str(arg)
			print("[rvprober.py] probing from *%s* software version" % start_sw_version)

		if opt in ("-n", "--number_of_compromised_sensor"):
			number_of_compromised_sensor = int(arg)
			print("[rvprober.py] the number of compromised sensors:%d" % number_of_compromised_sensor)

		if opt in ("-e", "--env_intensity"):
			# When the user wants to probe the environmental conditions
			if '_' in str(arg):
				arguments = arg.split("_")
				mutate_env_unit = int(arguments[0])
				mutate_env_intensity = int(arguments[1])
				print("[rvprober.py] mutate environment [unit:%d, intensity:%d]" % (mutate_env_unit, mutate_env_intensity))
			else:
				mutate_env_intensity = int(arg)
				print("[rvprober.py] mutate environment:%d" % mutate_env_intensity)

	open("testing_attack_type.txt", "w").close()

	f_attack_type = open("testing_attack_type.txt", "w")
	if attack_type == 'gps':
		print("[rvprober.py] User chooses GPS spoofing attacks")
		f_attack_type.write('gps')
	elif attack_type == 'imu':
		print("[rvprober.py] User chooses IMU attacks")
		f_attack_type.write('imu')
	elif attack_type == 'mag':
		print("[rvprober.py] User chooses magnetometer attacks")
		f_attack_type.write('mag')
	elif attack_type == 'baro':
		print("[rvprober.py] User chooses barometer attacks")
		f_attack_type.write('baro')
	elif attack_type == 'opticalflow':
		print("[rvprober.py] User chooses opticalflow attacks")
		f_attack_type.write('opticalflow')
	elif attack_type == 'EMI':
		print("[rvprober.py] User chooses EMI attacks")
		f_attack_type.write('EMI')

	f_attack_type.close()
	# (End) Parse command line arguments (i.e., input and output file)
	
	#----------------------------------------------------------------------
	#----------------------------------------------------------------------
	#----------------------------------------------------------------------
	RVPROBER_PX4_HOME = os.getenv("RVPROBER_PX4_HOME")

	# if the user does not specify a software version, then we will test PX4_HOME as the software version.
	if start_sw_version == "":
		start_sw_version = os.getenv("PX4_HOME")

	"""
	(Feb 22, 2024) To-do list
	- (To-do) Support the simulation of waves (e.g., https://github.com/srmainwaring/asv_wave_sim)
	"""

	open("restart.txt", "w").close()
	open("simulator_states.txt", "w").close()

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

	vehicle_type_list = []
	if target_vehicle_type == "gazebo_iris_opt_flow":
		vehicle_type_list = ['gazebo_iris_opt_flow', 'gazebo_rover', 'gazebo_uuv_hippocampus']
	elif target_vehicle_type == "gazebo_rover":
		vehicle_type_list = ['gazebo_rover', 'gazebo_uuv_hippocampus', 'gazebo_iris_opt_flow']
	elif target_vehicle_type == "gazebo_uuv_hippocampus":
		vehicle_type_list = ['gazebo_uuv_hippocampus', 'gazebo_iris_opt_flow', 'gazebo_rover']

	Default_vehicle_type = "gazebo_iris_opt_flow"

	# -------------------------------------------------------------------------------------------------------------------
	# Probing different types of an RV
	if probing_target == 1:
		for vehicle_type in vehicle_type_list:

			time.sleep(3)
			open("restart.txt", "w").close()

			print("*[Probing types of RVs] Current selected type: %s*" % vehicle_type)
			add_noise_to_gazebo(attack_type, number_of_compromised_sensor, intensity_of_attack, mutate_env_intensity, start_sw_version)
			time.sleep(5)
			open_simulator(vehicle_type, False, start_sw_version)
			open_probing(vehicle_type, test_time, Default_flight_mode, "")

			wait_for_simulation_result()

	# -------------------------------------------------------------------------------------------------------------------
	# Probing different hardware configurations of RVs
	elif probing_target == 2:
		vehicle_type_list = []
		if target_vehicle_type == "gazebo_iris_opt_flow":
			vehicle_type_list = ['gazebo_typhoon_h480', 'gazebo_standard_vtol', 'gazebo_iris_opt_flow']
		elif target_vehicle_type == "gazebo_rover":
			vehicle_type_list = ['gazebo_r1_rover', 'gazebo_rover']
		elif target_vehicle_type == "gazebo_uuv_hippocampus":
			vehicle_type_list = ['gazebo_boat', 'gazebo_uuv_hippocampus']

		for vehicle_type in vehicle_type_list:

			time.sleep(3)
			open("restart.txt", "w").close()

			print("*[Probing different hardware configurations of RVs] Current selected type: %s*" % vehicle_type)
			add_noise_to_gazebo(attack_type, number_of_compromised_sensor, intensity_of_attack, mutate_env_intensity, start_sw_version)
			time.sleep(5)
			open_simulator(vehicle_type, False, start_sw_version)
			open_probing(vehicle_type, test_time, Default_flight_mode, "")

			wait_for_simulation_result()

	# -------------------------------------------------------------------------------------------------------------------
	# Probing different 'flight modes'
	elif probing_target == 4:
		for mode in PX4_mode_list:

			time.sleep(3)
			open("restart.txt", "w").close()

			print("*[Probing flight modes] Current selected mode: %s*" % mode)
			add_noise_to_gazebo(attack_type, number_of_compromised_sensor, intensity_of_attack, mutate_env_intensity, start_sw_version)
			time.sleep(5)
			open_simulator(Default_vehicle_type, False, start_sw_version)
			open_probing(Default_vehicle_type, test_time, mode, "")

			wait_for_simulation_result()
	# -------------------------------------------------------------------------------------------------------------------
	# Probing different 'software version'
	elif probing_target == 5:
		software_version_list = ['PX4_1', 'PX4_2', 'PX4_3', 'PX4_4', 'PX4_5', 'PX4_6']
		for sw_version in software_version_list:

			time.sleep(3)
			open("restart.txt", "w").close()

			print("*[Probing software version] Current selected software version: %s*" % sw_version)
			add_noise_to_gazebo(attack_type, number_of_compromised_sensor, intensity_of_attack, mutate_env_intensity, sw_version)
			time.sleep(5)
			open_simulator(Default_vehicle_type, False, sw_version)
			open_probing(Default_vehicle_type, test_time, Default_flight_mode, "")

			wait_for_simulation_result()
	# -------------------------------------------------------------------------------------------------------------------
	# Probing different 'configuration parameters'
	elif probing_target == 6:
		line_number = len(open("list_of_params.txt", 'r').readlines())
		lines = open("list_of_params.txt", 'r').readlines()

		for line in open("list_of_params.txt", 'r').readlines():

			index = random.randint(1, line_number)
			row = lines[index].rstrip().split(',')
			print("* [Probing configuration parameters] Current selected parameter: %s, Default value: %f *" % (row[0], float(row[2])))

			time.sleep(3)
			open("restart.txt", "w").close()

			add_noise_to_gazebo(attack_type, number_of_compromised_sensor, intensity_of_attack, mutate_env_intensity, start_sw_version)
			time.sleep(5)
			open_simulator(Default_vehicle_type, False, start_sw_version)
			open_probing(Default_vehicle_type, test_time, Default_flight_mode, row[0])

			wait_for_simulation_result()
	# -------------------------------------------------------------------------------------------------------------------
	# Probing different 'environmental conditions' (e.g., wind speed and wave speed)
	elif probing_target == 7:

		for intensity in range(1, mutate_env_intensity+1):
			time.sleep(3)
			open("restart.txt", "w").close()

			env_intensity_current = mutate_env_unit * intensity
			print("*[Probing environmental conditions] Current selected intensity: %d*" % env_intensity_current)
			add_noise_to_gazebo(attack_type, number_of_compromised_sensor, intensity_of_attack, env_intensity_current, start_sw_version)
			time.sleep(5)
			open_simulator(Default_vehicle_type, False, start_sw_version)
			open_probing(Default_vehicle_type, test_time, Default_flight_mode, "")

			wait_for_simulation_result()
	# -------------------------------------------------------------------------------------------------------------------
	# The user does not specify the type of an RV; thus we will test a drone.
	elif probing_target == -1:
		add_noise_to_gazebo(attack_type, number_of_compromised_sensor, intensity_of_attack, mutate_env_intensity, start_sw_version)
		time.sleep(5)
		open_simulator("gazebo_iris_opt_flow", False, start_sw_version)
		open_probing("gazebo_iris_opt_flow", test_time, Default_flight_mode, "")
	# -------------------------------------------------------------------------------------------------------------------


if __name__ == "__main__":
   main(sys.argv[1:])