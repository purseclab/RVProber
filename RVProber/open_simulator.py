from subprocess import *

import time
import sys, os, getopt
import signal

def main(argv):
	wipe_eeprom = "off"
	mutate_hw = "off"
	software_path = ""

	# (Start) Parse command line arguments (i.e., input and output file)
	try:
		opts, args = getopt.getopt(argv, "v:whs:", ["vehicle_type=","sw_version="])

	except getopt.GetoptError:
		print("open_simulator.py -v <ArduCopter, Blimp, Rover, ArduSub>")
		sys.exit(2)

	for opt, arg in opts:
		if opt in ("-v", "--vehicle_type"):
			vehicle_target = str(arg)
			if vehicle_target == "ArduCopter":
				print("[open_simulator.py] Probing *ArduCopter* type of RVs")
			elif vehicle_target == "APMrover2":
				print("[open_simulator.py] Probing *Rover* type of RVs")
			elif vehicle_target == "ArduSub":
				print("[open_simulator.py] Probing *ArduSub* type of RVs")
		if opt == '-w':
			wipe_eeprom = "on"
			print("[open_simulator.py] Wipe EEPROM and reload configuration parameters")
		if opt == '-h':
			mutate_hw = "on"
			print("[open_simulator.py] Test different hardware configurations")
		if opt in ("-s", "--sw_version"):
			software_path = str(arg)

	# (End) Parse command line arguments (i.e., input and output file)

	ARDUPILOT_HOME = os.getenv(software_path)
	if software_path is None:
		raise Exception("ARDUPILOT_HOME environment variable is not set!")

	if wipe_eeprom == "off":
		c = ARDUPILOT_HOME + 'Tools/autotest/sim_vehicle.py -v ' + vehicle_target + ' --console --map'
	elif wipe_eeprom == "on":
		c = ARDUPILOT_HOME + 'Tools/autotest/sim_vehicle.py -v ' + vehicle_target + ' --console --map -w'

	handle = Popen(c, shell=True)

	while True:

		f = open("shared_variables.txt", "r")

		if f.read() == "reboot":

			open("shared_variables.txt", "w").close()

			open("restart.txt", "w").close()
			fi = open("restart.txt", "w")
			fi.write("restart")
			fi.flush()
			time.sleep(0.2)
			fi.close()
			time.sleep(0.2)

			os.killpg(os.getpgid(handle.pid), signal.SIGTERM)
			time.sleep(0.2)
			os.killpg(os.getpgid(handle.pid), signal.SIGTERM)
			time.sleep(0.2)
			os.killpg(os.getpgid(handle.pid), signal.SIGTERM)

		time.sleep(0.1)


if __name__ == "__main__":
   main(sys.argv[1:])