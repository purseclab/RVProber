from subprocess import *

import time
import sys, os, getopt
import signal

def main(argv):
	PX4_HOME = ""
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
			print("[open_simulator.py] Probing type of RVs: %s" % vehicle_target)
		if opt == '-h':
			mutate_hw = "on"
			print("[open_simulator.py] Test different hardware configurations")
		if opt in ("-s", "--sw_version"):
			software_path = str(arg)
			PX4_HOME = os.getenv(software_path)

	# (End) Parse command line arguments (i.e., input and output file)

	if PX4_HOME == "":
		PX4_HOME = os.getenv("PX4_HOME")

	if software_path is None:
		raise Exception("PX4_HOME environment variable is not set!")

	print("PX4_HOME:%s\n" % PX4_HOME)
	#c = 'cd ' + PX4_HOME + ' && HEADLESS=1 make px4_sitl gazebo_iris_opt_flow'
	#c = 'cd ' + PX4_HOME + ' && make px4_sitl gazebo_iris_opt_flow'


	c = 'cd ' + PX4_HOME + ' && make px4_sitl ' + vehicle_target


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