"""
        Author: Hyungsub Kim
        Date: 05/20/2020
        Name of file: fuzzing.py
        Goal: Main loop of PGFUZZ for PX4
"""

import sys, os, getopt
from optparse import OptionParser

import time
import random
import threading
import subprocess

# Tell python where to find mavlink so we can import it
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../mavlink'))

from pymavlink import mavutil, mavwp
from pymavlink import mavextra
from pymavlink import mavexpression

import re
import math

# ------------------------------------------------------------------------------------
# Global variables
# python /usr/local/bin/mavproxy.py --mav=/dev/tty.usbserial-DN01WM7R --baudrate 57600 --out udp:127.0.0.1:14540 --out udp:127.0.0.1:14550
connection_string = '0.0.0.0:14540'
master = mavutil.mavlink_connection('udp:' + connection_string)
home_altitude = 0
home_lat = 0
home_lon = 0
current_rc_3 = 0
takeoff = 0
drone_status = 0
wp_cnt = 0
lat = 0
lon = 0
# if 0: up / 1: down
goal_throttle = 0
executing_commands = 0
PARAM_MIN = 1
PARAM_MAX = 10000
required_min_thr = 975

current_roll = 0.0
current_pitch = 0.0
current_heading = 0.0

current_rc_1 = 0
current_rc_2 = 0
current_rc_3 = 0
current_rc_4 = 0

alt_error = 0.0
roll_error = 0.0
pitch_error = 0.0
heading_error = 0.0
stable_counter = 0
ground_speed = 0.0

vertical_speed = 0

alt_series = 0.0
roll_series = 0.0
pitch_series = 0.0
heading_series = 0.0
position_cnt = 0
lat_series = 0.0
lon_series = 0.0

circle_radius_current = 0.0
circle_radius_previous = 0.0

rollspeed_current = 0.0
rollspeed_previous = 0.0
pitchspeed_current = 0.0
pitchspeed_previous = 0.0
yawspeed_current = 0.0
yawspeed_previous = 0.0

# states - 0: turn off, 1: turn on
Parachute_on = 0
Armed = 0
current_flight_mode = ""
previous_flight_mode = ""
previous_altitude = 0
current_altitude = 0
current_alt = 0.0
previous_alt = 0.0
GPS_status = 1
Accel_status = 1
Gyro_status = 1
Baro_status = 1
PreArm_error = 0
system_time = 0
mission_cnt = 0
num_GPS = 0
alt_GPS_series = 0.0
vertical_speed_series = 0.0
gps_message_cnt = 0
gps_failsafe_error = 0
gps_failsafe_count = 0

# Distance
P = []
Global_distance = 0
Previous_distance = []

target_param = ""
target_param_ready = 0
target_param_value = 0

REBOOT_START = 0

# A pair of input (input, value)
Current_input = ""
Current_input_val = ""
Guidance_decision = None
count_main_loop = 0

# Heartbeat
heartbeat_cnt = 1
RV_alive = 0

# Policy
Precondition_path = ""
# Current_policy = "PX.RTL1"
Current_policy = "PX.ALTITUDE1"
Current_policy_P_length = 4

# Debug parameter
PRINT_DEBUG = 0

base_mode = [157, 193, 209, 217]
flight_mode = ["manual", "stabilized", "acro", "rattitude", "altitude", "offboard", "position", "hold", "mission",
               "return", "follow_me"]

RTL_alt = 10

paramsName = []
target_roll = 1500
target_pitch = 1500
target_yaw = 1500


# ------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------
# If the RV does not response within 5 seconds, we consider the RV's program crashed.
def check_liveness():
    global heartbeat_cnt

    end_flag = 0
    while end_flag == 0:

        if heartbeat_cnt < 1:
            store_mutated_inputs()
            # The RV software is crashed
            f = open("shared_variables.txt", "w")
            f.write("reboot")
            f.close()
            end_flag = 1
        else:
            heartbeat_cnt = 0

        time.sleep(5)


# ------------------------------------------------------------------------------------
def set_preconditions(filepath):
    for line in open(filepath, 'r').readlines():
        row = line.rstrip().split(' ')

        master.mav.param_set_send(master.target_system, master.target_component,
                                  row[0],
                                  float(row[1]),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(1)

        print("[Set_preconditions] %s = %s" % (row[0], row[1]))

# ------------------------------------------------------------------------------------
def re_launch():
    global goal_throttle
    global home_altitude
    global current_altitude
    global current_flight_mode
    global Parachute_on
    global count_main_loop
    global GPS_status
    global Accel_status
    global Gyro_status
    global Baro_status
    global PreArm_error
    global RV_alive

    RV_alive = 0
    count_main_loop = 0
    Parachute_on = 0
    GPS_status = 1
    Accel_status = 1
    Gyro_status = 1
    Baro_status = 1
    PreArm_error = 0

    print("#-----------------------------------------------------------------------------")
    print("#-----------------------------------------------------------------------------")
    print("#------------------------- RE-LAUNCH the vehicle -----------------------------")
    print("#-----------------------------------------------------------------------------")
    print("#-----------------------------------------------------------------------------")

    time.sleep(5)

    # Step 1. Reboot the RV's control program
    f = open("shared_variables.txt", "w")
    f.write("reboot")
    f.close()

    sys.exit()


# ------------------------------------------------------------------------------------
def verify_real_number(item):
    """ Method to find if an 'item'is real number"""

    item = str(item).strip()
    if not (item):
        return False
    elif (item.isdigit()):
        return True
    elif re.match(r"\d+\.*\d*", item) or re.match(r"-\d+\.*\d*", item):
        return True
    else:
        return False

# ------------------------------------------------------------------------------------
def cmd_set_home(home_location, altitude):
    # print '--- ', master.target_system, ',', master.target_component
    print("--- %d, %d", master.target_system, master.target_component)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1,  # set position
        0,  # param1
        0,  # param2
        0,  # param3
        0,  # param4
        home_location[0],  # lat
        home_location[1],  # lon
        altitude)  # alt


# ------------------------------------------------------------------------------------
def read_params():
    global paramsName

    for line in open('default_params.txt', 'r').readlines():
        row = line.rstrip().split(' ')
        paramsName.append(row[0])

# ------------------------------------------------------------------------------------
def change_flight_mode(mode):
    global base_mode

    if mode == "manual":
        main_mode = 1
        sub_mode = 0
    elif mode == "stabilized":
        main_mode = 7
        sub_mode = 0
    elif mode == "acro":
        main_mode = 5
        sub_mode = 0
    elif mode == "rattitude":
        main_mode = 8
        sub_mode = 0
    elif mode == "altitude":
        main_mode = 2
        sub_mode = 0
    elif mode == "offboard":
        main_mode = 6
        sub_mode = 0
    elif mode == "position":
        main_mode = 3
        sub_mode = 0
    elif mode == "hold":
        main_mode = 4
        sub_mode = 3
    elif mode == "mission":
        main_mode = 4
        sub_mode = 4
    elif mode == "return":
        main_mode = 4
        sub_mode = 5
    elif mode == "follow_me":
        main_mode = 4
        sub_mode = 8

    for i in range(0, 4):
        # Debug
        # print("[DEBUG] i:%d, base_mode:%d, main_mode:%d, sub_mode:%d" %(i, base_mode[i], main_mode, sub_mode))

        master.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            base_mode[i], main_mode, sub_mode, 0, 0, 0, 0)

        time.sleep(1)


# ------------------------------------------------------------------------------------
# --------------------- (Start) READ Robotic Vehicle's states ------------------------
def handle_heartbeat(msg):
    global heartbeat_cnt
    heartbeat_cnt += 1

    global current_flight_mode
    global previous_flight_mode

    if mavutil.mode_string_v10(msg) != "Mode(0x000000c0)":
        current_flight_mode = mavutil.mode_string_v10(msg)

    if (previous_flight_mode != mavutil.mode_string_v10(msg)) and previous_flight_mode != current_flight_mode:
        previous_flight_mode = current_flight_mode

    global drone_status
    drone_status = msg.system_status
    # print("[DEBUG] Drone status: %d, mavlink version: %d" % (drone_status, msg.mavlink_version))

    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

    # print("[DEBUG] Mode: %s, is_armed: %d, is_enabled: %d" % (current_flight_mode, is_armed, is_enabled))


# ------------------------------------------------------------------------------------
def handle_rc_raw(msg):
    global current_rc_1
    global current_rc_2
    global current_rc_3
    global current_rc_4

    current_rc_1 = msg.chan1_raw
    current_rc_2 = msg.chan2_raw
    current_rc_3 = msg.chan3_raw
    current_rc_4 = msg.chan4_raw

    channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)


# ------------------------------------------------------------------------------------
def handle_hud(msg):
    global current_yaw
    global alt_series
    global stable_counter
    global current_altitude
    global previous_altitude
    global current_heading
    global ground_speed

    hud_data = (msg.airspeed, msg.groundspeed, msg.heading,
                msg.throttle, msg.alt, msg.climb)

    # print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
    # print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data

    # print("Alt: %f" %msg.alt)

    ground_speed = msg.groundspeed  # m/s
    previous_altitude = current_altitude
    current_altitude = msg.alt

    alt_series += current_altitude
    stable_counter += 1
    # print("[DEBUG] previous_altitude:%f, current_altitude:%f" % (current_altitude, previous_altitude))

    current_heading = (msg.heading - 359)


# ------------------------------------------------------------------------------------
def handle_attitude(msg):
    global current_roll
    global current_pitch

    global rollspeed_current
    global rollspeed_previous
    global pitchspeed_current
    global pitchspeed_previous
    global yawspeed_current
    global yawspeed_previous

    attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed,
                     msg.pitchspeed, msg.yawspeed)
    # print "Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd"
    # print "%0.6f\t%0.6f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data

    rollspeed_previous = rollspeed_current
    pitchspeed_previous = pitchspeed_current
    yawspeed_previous = yawspeed_current

    rollspeed_current = msg.rollspeed
    pitchspeed_current = msg.pitchspeed
    yawspeed_current = msg.yawspeed

    current_roll = (msg.roll * 180) / math.pi
    current_pitch = (msg.pitch * 180) / math.pi


# ------------------------------------------------------------------------------------
def handle_target(msg):
    global current_roll
    global current_pitch
    global current_altitude
    global alt_error
    global roll_error
    global pitch_error
    global heading_error

    global alt_series
    global roll_series
    global pitch_series
    global heading_series
    global stable_counter

    # print("altitude error:%f" %msg.alt_error)
    # msg.alt_error
    """
	nav_roll	float	deg	Current desired roll
	nav_pitch	float	deg	Current desired pitch
	nav_bearing	int16_t	deg	Current desired heading
	target_bearing	int16_t	deg	Bearing to current waypoint/target
	wp_dist	uint16_t	m	Distance to active waypoint
	alt_error	float	m	Current altitude error
	aspd_error	float	m/s	Current airspeed error
	xtrack_error	float	m	Current crosstrack error on x-y plane
	"""
    reference = (msg.nav_roll, msg.nav_pitch, msg.nav_bearing, msg.alt_error, msg.aspd_error, msg.xtrack_error)
    # print "\nRF_Roll\tRF_Pitch\tRF_Head\tRF_Alt\tRF_Spd\tRF_XY"
    # print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % reference

    # print("\nRoll error:%f, pitch error:%f, heading error:%f\n" %(abs(msg.nav_roll - current_roll), abs(msg.nav_pitch - current_pitch), abs(msg.nav_bearing - current_heading)))

    alt_error += msg.alt_error
    roll_error += abs(msg.nav_roll - current_roll)
    pitch_error += abs(msg.nav_pitch - current_pitch)
    heading_error += abs(msg.nav_bearing - current_heading)
    stable_counter += 1

    # Compute moving average of each state
    alt_series += (current_altitude + msg.alt_error)
    roll_series += msg.nav_roll
    pitch_series += msg.nav_pitch
    heading_series += msg.nav_bearing

    # print("[DEBUG] current_altitude:%f, alt_error:%f" %(current_altitude, msg.alt_error))
    # print("Desired heading:%f, current heading:%f" %(msg.nav_bearing, current_heading))


# ------------------------------------------------------------------------------------
def handle_param(msg):
    global target_param
    global target_param_ready
    global target_param_value

    message = msg.to_dict()
    if message['param_id'].decode("utf-8") == target_param:
        target_param_ready = 1
        target_param_value = message['param_value']
    else:
        target_param_ready = 0


# ------------------------------------------------------------------------------------
def handle_position(msg):
    position_data = (msg.lat, msg.lon)

    # print(msg)
    # print("lat:%f, lon:%f" %(float(msg.lat), float(msg.lon)))

    global position_cnt
    global lat_series
    global lon_series
    global vertical_speed

    position_cnt += 1
    lat_series += msg.lat
    lon_series += msg.lon

    vertical_speed = msg.vz


# ------------------------------------------------------------------------------------
def handle_status(msg):
    global Parachute_on
    global GPS_status
    global Accel_status
    global Gyro_status
    global PreArm_error
    global Baro_status
    global drone_status
    global takeoff
    global gps_failsafe_error

    status_data = (msg.severity, msg.text)
    print("[status_text] %s" % msg.text)

    # Detecting a depolyed parachute
    if "Parachute: Released" in msg.text:
        Parachute_on = 1

    # Detecting an error on GPS
    elif "NavEKF" in msg.text and "lane switch" in msg.text:
        GPS_status = 0

    # Detecting an error on GPS
    elif "stopping navigation" in msg.text or "GPS data stopped" in msg.text:
        GPS_status = 0

    # Detecting an error on gyro sensor
    elif "Vibration compensation ON" in msg.text:
        Gyro_status = 0

    # Detecting an error on accelerometer sensor
    elif "EKF primary changed" in msg.text:
        Accel_status = 0

    # Detecting error messages related to a barometer sensor
    elif "PreArm: Waiting for Nav Checks" in msg.text:
        Baro_status = 0

    # Detecting a PreArm check error
    elif "PreArm: Check" in msg.text:
        PreArm_error = 1

    # Detecting landing on the ground
    elif "Landing detected" in msg.text:
        drone_status = 3  # 3: The drone is landed on the ground
        print("[DEBUG] The drone is landed on the ground!")

    # Detecting takeoff
    elif "Takeoff to" in msg.text:
        takeoff = 1  # 1: The drone is taking off from the ground
        print("[DEBUG] The drone is taking off from the ground!")

    # Detecting a GPS failsafe
    elif "Failsafe enabled: no global position" in msg.text:
        gps_failsafe_error = 1

    # ------------------------------------------------------------------------------------


def handle_time(msg):
    global system_time

    # time_data = (msg.time_unix_usec, msg.time_boot_ms)
    system_time = msg.time_unix_usec

    # print "[system_time] UNIX time:%d, Boot time:%d" % time_data


# ------------------------------------------------------------------------------------
def handle_mission(msg):
    global mission_cnt

    mission_cnt = msg.count
    print("[MISSION_COUNT]%d" % mission_cnt)


# ------------------------------------------------------------------------------------
def handle_gps(msg):
    global num_GPS
    global alt_GPS_series
    global gps_message_cnt

    gps_message_cnt += 1
    num_GPS = int(msg.satellites_visible)
    alt_GPS_series += int(msg.alt / 1000)  # convert mm unit to meter unit

    # print("# of GPS:%d, GPS altitude:%d"%(num_GPS, alt_GPS))


# ------------------------------------------------------------------------------------
def handle_circle_status(msg):
    # Positive values orbit clockwise, negative values orbit counter-clockwise.
    ## https://mavlink.io/en/messages/common.html#ORBIT_EXECUTION_STATUS
    global circle_radius_current
    global circle_radius_previous

    circle_radius_previous = circle_radius_current
    circle_radius_current = msg.radius

    # print("[Debug-circle mode] radius:%f, X:%d, Y:%d, Z:%f" % (msg.radius, msg.lon, msg.x, msg.y, msg.z))


# ------------------------------------------------------------------------------------
def read_loop():
    while True:
        # grab a mavlink message
        msg = master.recv_match(blocking=True)

        # handle the message based on its type
        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()
        elif msg_type == "RC_CHANNELS":
            handle_rc_raw(msg)
        elif msg_type == "VFR_HUD":
            handle_hud(msg)
        elif msg_type == "ATTITUDE":
            handle_attitude(msg)
        elif msg_type == "NAV_CONTROLLER_OUTPUT":
            handle_target(msg)
        elif msg_type == "GLOBAL_POSITION_INT":
            handle_position(msg)
        elif msg_type == "STATUSTEXT":
            handle_status(msg)
        elif msg_type == "SYSTEM_TIME":
            handle_time(msg)
        elif msg_type == "MISSION_COUNT":
            handle_mission(msg)
        elif msg_type == "PARAM_VALUE":
            handle_param(msg)
        elif msg_type == "GPS_RAW_INT":
            handle_gps(msg)
        elif msg_type == "HEARTBEAT":
            handle_heartbeat(msg)
        elif msg_type == "ORBIT_EXECUTION_STATUS":
            handle_circle_status(msg)

# ------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------
# ---------------(Start) Calculate propositional and global distances-----------------
def calculate_distance(guidance):
    # State global variables
    global alt_series
    global roll_series
    global pitch_series
    global heading_series
    global stable_counter

    global lat_series
    global lon_series
    global position_cnt
    global home_lat
    global home_lon

    global current_alt
    global previous_alt
    global Previous_distance
    global ground_speed

    global num_GPS
    global alt_GPS_series
    global gps_message_cnt

    global circle_radius_current
    global circle_radius_previous

    global current_rc_1
    global current_rc_2
    global current_rc_3
    global current_rc_4

    global rollspeed_current
    global rollspeed_previous
    global pitchspeed_current
    global pitchspeed_previous
    global yawspeed_current
    global yawspeed_previous

    global takeoff
    global vertical_speed
    global gps_failsafe_error
    global gps_failsafe_count

    rollspeed_current = round(rollspeed_current, 0)
    rollspeed_previous = round(rollspeed_previous, 0)
    pitchspeed_current = round(pitchspeed_current, 0)
    pitchspeed_previous = round(pitchspeed_previous, 0)
    yawspeed_current = round(yawspeed_current, 0)
    yawspeed_previous = round(yawspeed_previous, 0)

    if stable_counter > 0:
        alt_avg = alt_series / stable_counter
        roll_avg = roll_series / stable_counter
        pitch_avg = pitch_series / stable_counter
        heading_avg = heading_series / stable_counter
    else:
        alt_avg = 0
        roll_avg = 0
        pitch_avg = 0
        heading_avg = 0

    if position_cnt > 0:
        lat_avg = lat_series / position_cnt
        lon_avg = lon_series / position_cnt

    else:
        lat_avg = 0
        lon_avg = 0

    lat_avg = lat_avg / 1000
    lat_avg = lat_avg * 1000
    lon_avg = lon_avg / 1000
    lon_avg = lon_avg * 1000

    previous_alt = current_alt
    current_alt = math.ceil(alt_avg)

    # Calculate a relative altitude from the home postion
    current_alt = current_alt - home_altitude

    if gps_message_cnt > 0:
        alt_GPS_avg = alt_GPS_series / gps_message_cnt
    else:
        alt_GPS_avg = alt_GPS_series

    # print('[Debug] stable_counter:%d' %stable_counter)
    # print('[Debug] alt_avg:%f (previous_alt:%f, current_alt:%f), roll_avg:%f, pitch_avg:%f, heading_avg:%f' %(alt_avg, previous_alt, current_alt, roll_avg, pitch_avg, heading_avg))
    print('[Debug] lat_avg:%f, home_lat:%f, lon_avg:%f, home_lon:%f' % (lat_avg, home_lat, lon_avg, home_lon))

    position_cnt = 0
    stable_counter = 0
    alt_series = 0
    roll_series = 0
    pitch_series = 0
    heading_series = 0
    lat_series = 0
    lon_series = 0
    alt_GPS_series = 0
    gps_message_cnt = 0

    global target_param
    global target_param_ready
    global target_param_value

    global Parachute_on
    global Armed
    global current_flight_mode
    global previous_flight_mode
    global previous_altitude
    global current_altitude
    global goal_throttle

    global P
    global Global_distance

    target_param_ready = 0
    # ----------------------- (start) PX.CHUTE1 policy -----------------------
    # ----------------------- (start) PX.CHUTE1 policy -----------------------
    # Propositional distances
    # 0: turn off, 1: turn on

    # P1
    if Parachute_on == 1:
        P[0] = 1
    else:
        P[0] = -1
    # P2
    if Armed == 0:
        P[1] = 1
    else:
        P[1] = -1
    # P3
    if current_flight_mode == "FLIP" or current_flight_mode == "ACRO":
        P[2] = 1
    else:
        P[2] = -1

    # P4
    if current_alt > 0:
        P[3] = (current_alt - previous_alt) / current_alt
    else:
        P[3] = 0

    # P5
    # Request parameter
    target_param_ready = 0
    target_param_value = 0

    master.mav.param_request_read_send(master.target_system, master.target_component, 'CHUTE_ALT_MIN', -1)

    target_param = "CHUTE_ALT_MIN"
    count = 0
    while target_param_ready == 0 and count < 5:
        time.sleep(1)
        count += 1

    if target_param_value > 0:
        P[4] = (target_param_value - current_alt) / target_param_value
    else:
        P[4] = 0

    Global_distance = -1 * (min(P[0], max(P[1], P[2], P[3], P[4])))

    print_distance(G_dist=Global_distance, P_dist=P, length=5, policy="PX.CHUTE", guid=guidance)

    # ----------------------- (end) PX.CHUTE1 policy -----------------------
    # ----------------------- (end) PX.CHUTE1 policy -----------------------

    target_param_ready = 0
    lat_avg = lat_avg / 1000
    lat_avg = lat_avg * 1000
    lon_avg = lon_avg / 1000
    lon_avg = lon_avg * 1000
    # ----------------------- (start) PX.RTL1 policy -----------------------
    # ----------------------- (start) PX.RTL1 policy -----------------------
    # P1
    if lat_avg == home_lat and lon_avg == home_lon:
        P[0] = -1
    else:
        P[0] = 1

    # P2
    # Request parameter
    target_param_ready = 0
    target_param_value = 0

    master.mav.param_request_read_send(
        master.target_system, master.target_component, 'RTL_ALT', -1)

    target_param = "RTL_ALT"
    count = 0
    while target_param_ready == 0 and count < 5:
        time.sleep(1)
        count += 1

    # Convert centimeters to meters
    target_param_value = target_param_value / 100

    if target_param_value > 0:
        P[1] = (target_param_value - current_alt) / target_param_value
    else:
        P[1] = 0

    print("[Debug] RTL_ALT:%f, current_alt:%f" % (target_param_value, current_alt))

    if current_flight_mode == "RTL":
        P[2] = 1
    else:
        P[2] = -1

    if previous_alt != 0:
        P[3] = (previous_alt - current_alt) / previous_alt
    else:
        P[3] = 0

    Global_distance = -1 * (min(P[0], P[1], P[2], P[3]))

    print_distance(G_dist=Global_distance, P_dist=P, length=4, policy="PX.RTL1", guid=guidance)
    # ----------------------- (end) PX.RTL1 policy -----------------------
    # ----------------------- (end) PX.RTL1 policy -----------------------

    target_param_ready = 0
    # ----------------------- (start) PX.ALTITUDE1 policy -----------------------
    # ----------------------- (start) PX.ALTITUDE1 policy -----------------------
    # Propositional distances
    # 0: turn off, 1: turn on

    # print("[DEBUG] Mode:%s, Throttle:%d, Alt_t:%f, Alt_prev:%f" %(current_flight_mode, goal_throttle, current_alt, previous_alt))

    # P1: Mode_t = ALTITUDE
    if current_flight_mode == "ALTCTL":
        P[0] = 1
    else:
        P[0] = -1
    # P2: Throttle_t = 1500
    if goal_throttle == 1500:
        P[1] = 1
    else:
        P[1] = -1
    # P3: ALT_t = ALT_(t-1)
    if current_alt == previous_alt:
        P[2] = -1
    else:
        P[2] = 1

    Global_distance = -1 * (min(P[0], P[1], P[2]))

    print_distance(G_dist=Global_distance, P_dist=P, length=3, policy="PX.ALTITUDE", guid=guidance)

    # ----------------------- (end) PX.ALTITUDE1 policy -----------------------
    # ----------------------- (end) PX.ALTITUDE1 policy -----------------------

    # ----------------------- (start) PX.HOLD1 policy -----------------------
    # ----------------------- (start) PX.HOLD1 policy -----------------------
    # P0: Mode_t = LOITER
    if current_flight_mode == "LOITER":
        P[0] = 1
    else:
        P[0] = -1

    # P1: Yaw_t = Yaw_(t-1)
    if round(yawspeed_current, 1) == round(yawspeed_previous, 1):
        P[1] = -1
    else:
        P[1] = 1

    # P2: POS_t = POS_(t-1)
    if round(ground_speed, 0) == 0:
        P[2] = -1
    else:
        P[2] = 1

    # P3: ALT_t = ALT_(t-1)
    if round(current_altitude, 0) == round(previous_altitude, 0):
        P[3] = -1
    else:
        P[3] = 1

    if PRINT_DEBUG == 1:
        print("[Debug] Mode_t:%s, Yaw_t:%f, Yaw_(t-1):%f, ground speed:%f, ALT_t:%f, ALT_(t-1):%f" % (
            current_flight_mode, round(yawspeed_current, 1), round(yawspeed_previous, 1), round(ground_speed, 0),
            round(current_altitude, 0), round(previous_altitude, 0)))

    Global_distance = -1 * (min(P[0], max(P[1], P[2], P[3])))

    print_distance(G_dist=Global_distance, P_dist=P, length=4, policy="PX.HOLD1", guid=guidance)
    # ----------------------- (end) PX.HOLD1 policy -----------------------
    # ----------------------- (end) PX.HOLD1 policy -----------------------

    # ----------------------- (start) PX.HOLD2 policy -----------------------
    # ----------------------- (start) PX.HOLD2 policy -----------------------
    # P0: Mode_t = LOITER
    if current_flight_mode == "LOITER":
        P[0] = 1
    else:
        P[0] = -1

    # P1: 'MIS_LTRMIN_ALT' != -1
    # Request parameter
    target_param_ready = 0
    target_param_value = 0

    master.mav.param_request_read_send(
        master.target_system, master.target_component, 'MIS_LTRMIN_ALT', -1)

    target_param = "MIS_LTRMIN_ALT"
    count = 0
    while target_param_ready == 0 and count < 5:
        time.sleep(1)
        count += 1

    if target_param_value != -1:
        P[1] = 1
    else:
        P[1] = -1

    # P3: ALT_t >= ALT_(t-1)
    ## ALT_(t-1) - ALT_t / ALT_(t-1)
    if (previous_altitude == 0) or (previous_altitude - current_altitude == 0):
        P[2] = 0
    else:
        P[2] = round((previous_altitude - current_altitude) / previous_altitude, 0)

    if PRINT_DEBUG == 1:
        print("[Debug] Mode_t:%s, MIS_LTRMIN_ALT:%d, ALT_t:%f, ALT_(t-1):%f" % (
            current_flight_mode, target_param_value, round(current_altitude, 0), round(previous_altitude, 0)))

    Global_distance = -1 * (min(P[0], P[1], P[2]))

    print_distance(G_dist=Global_distance, P_dist=P, length=3, policy="PX.HOLD2", guid=guidance)
    # ----------------------- (end) PX.HOLD2 policy -----------------------
    # ----------------------- (end) PX.HOLD2 policy -----------------------

    # ----------------------- (start) PX.POSITION1 policy -----------------------
    # ----------------------- (start) PX.POSITION1 policy -----------------------
    # P0: Mode_t = POSITION
    if current_flight_mode == "POSCTL":
        P[0] = 1
    else:
        P[0] = -1

    # P1: POS_t = POS_(t-1)
    if round(ground_speed, 0) == 0:
        P[1] = -1
    else:
        P[1] = 1

    if PRINT_DEBUG == 1:
        print("[Debug] Mode_t:%s, ground speed:%f," % (current_flight_mode, round(ground_speed, 0)))

    Global_distance = -1 * (min(P[0], P[1]))

    print_distance(G_dist=Global_distance, P_dist=P, length=2, policy="PX.POSITION1", guid=guidance)
    # ----------------------- (end) PX.POSITION1 policy -----------------------
    # ----------------------- (end) PX.POSITION1 policy -----------------------

    # ----------------------- (start) PX.TAKEOFF1 policy -----------------------
    # ----------------------- (start) PX.TAKEOFF1 policy -----------------------
    # P0: Command_t = takeoff
    if takeoff == 1:
        P[0] = 1
    else:
        P[0] = -1

    # P1: ALT_t <= MIS_TAKEOFF_ALT
    # Request parameter
    target_param_ready = 0
    target_param_value = 0

    master.mav.param_request_read_send(master.target_system, master.target_component, 'MIS_TAKEOFF_ALT', -1)

    target_param = "MIS_TAKEOFF_ALT"
    count = 0
    while target_param_ready == 0 and count < 5:
        time.sleep(1)
        count += 1

    if (current_alt == 0) or (current_alt - target_param_value == 0):
        P[1] = 0
    else:
        P[1] = round(current_alt - target_param_value) / current_alt

    if PRINT_DEBUG == 1:
        print("[Debug] Takeoff:%d, ALT_t:%f, MIS_TAKEOFF_ALT:%f," % (takeoff, current_alt, target_param_value))

    Global_distance = -1 * (min(P[0], P[1]))

    print_distance(G_dist=Global_distance, P_dist=P, length=2, policy="PX.TAKEOFF1", guid=guidance)
    # ----------------------- (end) PX.TAKEOFF1 policy -----------------------
    # ----------------------- (end) PX.TAKEOFF1 policy -----------------------

    # ----------------------- (start) PX.TAKEOFF2 policy -----------------------
    # ----------------------- (start) PX.TAKEOFF2 policy -----------------------
    # P0: Command_t = takeoff
    if takeoff == 1:
        P[0] = 1
    else:
        P[0] = -1

    # P1: vertical_speed  = MPC_TKO_SPEED
    # Request parameter
    target_param_ready = 0
    target_param_value = 0

    master.mav.param_request_read_send(master.target_system, master.target_component, 'MPC_TKO_SPEED', -1)

    target_param = "MPC_TKO_SPEED"
    count = 0
    while target_param_ready == 0 and count < 5:
        time.sleep(1)
        count += 1

    if round(vertical_speed - target_param_value, 0) == 0:
        P[1] = -1
    else:
        P[1] = 1

    if PRINT_DEBUG == 1:
        print(
            "[Debug] Takeoff:%d, vertical_speed:%f, MPC_TKO_SPEED:%f," % (takeoff, vertical_speed, target_param_value))

    Global_distance = -1 * (min(P[0], P[1]))

    print_distance(G_dist=Global_distance, P_dist=P, length=2, policy="PX.TAKEOFF2", guid=guidance)
    # ----------------------- (end) PX.TAKEOFF2 policy -----------------------
    # ----------------------- (end) PX.TAKEOFF2 policy -----------------------

    takeoff = 0

    # commander mode auto:land
    # ----------------------- (start) PX.LAND1 policy -----------------------
    # ----------------------- (start) PX.LAND1 policy -----------------------
    # P0: Mode_t = LAND
    if current_flight_mode == "LAND":
        P[0] = 1
    else:
        P[0] = -1

    # P1: vertical_speed  = MPC_LAND_SPEED
    # Request parameter
    target_param_ready = 0
    target_param_value = 0

    master.mav.param_request_read_send(master.target_system, master.target_component, 'MPC_LAND_SPEED', -1)

    target_param = "MPC_LAND_SPEED"
    count = 0
    while target_param_ready == 0 and count < 5:
        time.sleep(1)
        count += 1

    if round(abs(vertical_speed / 100) - abs(target_param_value), 0) == 0:
        P[1] = -1
    else:
        P[1] = 1

    if PRINT_DEBUG == 1:
        print("[Debug] Mode_t:%s, Takeoff:%d, vertical_speed:%f, MPC_LAND_SPEED:%f," % (
        current_flight_mode, takeoff, round(abs(vertical_speed / 100), 0), round(abs(target_param_value), 0)))

    Global_distance = -1 * (min(P[0], P[1]))

    print_distance(G_dist=Global_distance, P_dist=P, length=2, policy="PX.LAND1", guid=guidance)
    # ----------------------- (end) PX.LAND1 policy -----------------------
    # ----------------------- (end) PX.LAND1 policy -----------------------

    # ----------------------- (start) PX.ORBIT6 policy -----------------------
    # ----------------------- (start) PX.ORBIT6 policy -----------------------
    # P0: Mode_t = ORBIT
    if current_flight_mode == "ORBIT":
        P[0] = 1
    else:
        P[0] = -1

    # P1: ground_speed < 2 m/s
    ## (ground_speed - 2) / ground_speed
    if (ground_speed == 0) or (ground_speed - 2 == 0):
        P[1] = 0
    else:
        P[1] = (ground_speed - 2) / ground_speed

    if PRINT_DEBUG == 1:
        print("[Debug] Mode_t:%s, ground_speed:%f," % (current_flight_mode, ground_speed))

    Global_distance = -1 * (min(P[0], P[1]))

    print_distance(G_dist=Global_distance, P_dist=P, length=2, policy="PX.ORBIT6", guid=guidance)
    # ----------------------- (end) PX.ORBIT6 policy -----------------------
    # ----------------------- (end) PX.ORBIT6 policy -----------------------

    # ----------------------- (start) PX.ORBIT1 policy -----------------------
    # ----------------------- (start) PX.ORBIT1 policy -----------------------
    # P0: Mode_t = ORBIT
    if current_flight_mode == "ORBIT":
        P[0] = 1
    else:
        P[0] = -1

    # P1: RC_pitch < 1,500
    ## (1,500 - RC_pitch) / 1,500
    ## current_rc_2: RC_pitch
    if current_rc_2 == 1500:
        P[1] = 0
    else:
        P[1] = (1500 - current_rc_2) / 1500

    # P2: Circle_radius_t > 0
    if circle_radius_current > 0:
        P[2] = 1
    else:
        P[2] = -1

    # P3: Circle_radius_t < Circle_radius_(t-1)
    ## ( Circle_radius_(t-1) - Circle_radius_t ) / Circle_radius_(t-1)

    if circle_radius_previous == 0 or (circle_radius_previous - circle_radius_current) == 0:
        P[3] = 0
    else:
        P[3] = (circle_radius_previous - circle_radius_current) / circle_radius_previous

    if PRINT_DEBUG == 1:
        print("[Debug] RC_pitch:%d, circle_radius_t:%f, circle_radius_(t-1):%f" % (
            current_rc_2, circle_radius_current, circle_radius_previous))

    Global_distance = -1 * (min(P[0], P[1], P[2], P[3]))

    print_distance(G_dist=Global_distance, P_dist=P, length=4, policy="PX.ORBIT1", guid=guidance)
    # ----------------------- (end) PX.ORBIT1 policy -----------------------
    # ----------------------- (end) PX.ORBIT1 policy -----------------------

    # ----------------------- (start) PX.ORBIT2 policy -----------------------
    # ----------------------- (start) PX.ORBIT2 policy -----------------------
    # P0: Mode_t = ORBIT
    if current_flight_mode == "ORBIT":
        P[0] = 1
    else:
        P[0] = -1

    # P1: RC_pitch > 1,500
    ## (RC_pitch - 1,500) / RC_pitch
    ## current_rc_2: RC_pitch

    if current_rc_2 == 0 or current_rc_2 == 1500:
        P[1] = 0
    else:
        P[1] = (current_rc_2 - 1500) / current_rc_2

    # P2: Circle_radius_t > Circle_radius_(t-1)
    ## ( Circle_radius_(t-1) - Circle_radius_t ) / Circle_radius_(t-1)

    if circle_radius_previous == 0 or (circle_radius_previous - circle_radius_current) == 0:
        P[2] = 0
    else:
        P[2] = (circle_radius_previous - circle_radius_current) / circle_radius_previous

    if PRINT_DEBUG == 1:
        print("[Debug] RC_pitch:%d, circle_radius_t:%f, circle_radius_(t-1):%f" % (
            current_rc_2, circle_radius_current, circle_radius_previous))

    Global_distance = -1 * (min(P[0], P[1], P[2]))

    print_distance(G_dist=Global_distance, P_dist=P, length=3, policy="PX.ORBIT2", guid=guidance)
    # ----------------------- (end) PX.ORBIT2 policy -----------------------
    # ----------------------- (end) PX.ORBIT2 policy -----------------------

    # ----------------------- (start) PX.ORBIT3 policy -----------------------
    # ----------------------- (start) PX.ORBIT3 policy -----------------------
    # P0: Mode_t = ORBIT
    if current_flight_mode == "ORBIT":
        P[0] = 1
    else:
        P[0] = -1

    # P1: RC_roll > 1,500
    ## (RC_roll - 1,500) / RC_proll
    ## current_rc_1: RC_roll

    if current_rc_1 == 0 or current_rc_1 == 1500:
        P[1] = 0
    else:
        P[1] = (current_rc_1 - 1500) / current_rc_1

    # P2: circle_detection_t = clockwise
    ## circle_radius_current > 0: clockwise, circle_radius_current < 0: counter-clockwise
    if circle_radius_current > 0:
        P[2] = 1
    else:
        P[2] = -1

    # P3: Circle_speed_t > Circle_speed_(t-1)
    ## There is no direct way to measure the circle speed; thus, let's assume that it's aways hold.
    P[3] = -1

    Global_distance = -1 * (min(P[0], P[1], P[2], P[3]))

    print_distance(G_dist=Global_distance, P_dist=P, length=4, policy="PX.ORBIT3", guid=guidance)
    # ----------------------- (end) PX.ORBIT3 policy -----------------------
    # ----------------------- (end) PX.ORBIT3 policy -----------------------

    # ----------------------- (start) PX.ORBIT4-5 policy -----------------------
    # ----------------------- (start) PX.ORBIT4-5 policy -----------------------
    # P0: Mode_t = ORBIT
    if current_flight_mode == "ORBIT":
        P[0] = 1
    else:
        P[0] = -1

    # P1: RC_roll < 1,500
    ## (1,500 - RC_roll) / 1,500
    ## current_rc_1: RC_roll

    if (1500 - current_rc_1) == 0:
        P[1] = 0
    else:
        P[1] = (1500 - current_rc_1) / 1500

    # P2: circle_detection_t = counter-clockwise
    ## circle_radius_current > 0: clockwise, circle_radius_current < 0: counter-clockwise
    if circle_radius_current < 0:
        P[2] = 1
    else:
        P[2] = -1

    # P3: Circle_speed_t < Circle_speed_(t-1)
    ## There is no direct way to measure the circle speed; thus, let's assume that it's aways hold.
    P[3] = -1

    Global_distance = -1 * (min(P[0], P[1], P[2], P[3]))

    print_distance(G_dist=Global_distance, P_dist=P, length=4, policy="PX.ORBIT4-5", guid=guidance)
    # ----------------------- (end) PX.ORBIT4-5 policy -----------------------
    # ----------------------- (end) PX.ORBIT4-5 policy -----------------------

    # ----------------------- (start) PX.RTL5 policy -----------------------
    # ----------------------- (start) PX.RTL5 policy -----------------------
    # P0: Mode_(t-1) = RTL ^ Mode_t = LAND
    if previous_flight_mode == "RTL" and current_flight_mode == "LAND":
        P[0] = 1
    else:
        P[0] = -1

    # P1: ALT_t = Ground_ALT
    if round(current_altitude, 1) == round(home_altitude, 1):
        P[1] = 1
    else:
        P[1] = 0

    # P2: Disarm = on
    if Armed == 0:
        P[2] = -1
    else:
        P[2] = 1

    if PRINT_DEBUG == 1:
        print("[Debug] Mode_(t-1):%s, Mode_t:%s, ALT_t:%f, Ground_ALT:%f" % (
        previous_flight_mode, current_flight_mode, round(current_altitude, 1), round(home_altitude, 1)))

    Global_distance = -1 * (min(P[0], P[1], P[2]))

    print_distance(G_dist=Global_distance, P_dist=P, length=3, policy="PX.RTL5", guid=guidance)
    # ----------------------- (end) PX.RTL5 policy -----------------------
    # ----------------------- (end) PX.RTL5 policy -----------------------

    # ----------------------- (start) PX.RTL4 policy -----------------------
    # ----------------------- (start) PX.RTL4 policy -----------------------
    # P0: Mode_t = RTL
    if current_flight_mode == "RTL":
        P[0] = 1
    else:
        P[0] = -1

    # P1: RTL_DESCEND_ALT = -1
    # Request parameter
    target_param_ready = 0
    target_param_value = 0

    master.mav.param_request_read_send(master.target_system, master.target_component, 'RTL_DESCEND_ALT', -1)

    target_param = "RTL_DESCEND_ALT"
    count = 0
    while target_param_ready == 0 and count < 5:
        time.sleep(1)
        count += 1

    if target_param_value == -1:
        P[1] = 1
    else:
        P[1] = -1

    # P2: POS_t = POS_(t-1)
    if round(ground_speed, 0) == 0:
        P[2] = -1
    else:
        P[2] = 1

    # P3: ALT_t = ALT_(t-1)
    if round(current_altitude, 0) == round(previous_altitude, 0):
        P[3] = -1
    else:
        P[3] = 1

    if PRINT_DEBUG == 1:
        print("[Debug] Mode_t:%s, RTL_DESCEND_ALT:%f, ground_speed:%f, ALT_t:%f, ALT_(t-1):%f" % (
        current_flight_mode, target_param_value, round(ground_speed, 0), round(current_altitude, 0),
        round(previous_altitude, 0)))

    Global_distance = -1 * (min(P[0], P[1], max(P[2], P[3])))

    print_distance(G_dist=Global_distance, P_dist=P, length=4, policy="PX.RTL4", guid=guidance)
    # ----------------------- (end) PX.RTL4 policy -----------------------
    # ----------------------- (end) PX.RTL4 policy -----------------------

    lat_avg = lat_avg / 1000
    lat_avg = lat_avg * 1000
    lon_avg = lon_avg / 1000
    lon_avg = lon_avg * 1000

    # ----------------------- (start) PX.RTL3 policy -----------------------
    # ----------------------- (start) PX.RTL3 policy -----------------------
    # P0: Mode_t = RTL
    if previous_flight_mode == "RTL":
        P[0] = 1
    else:
        P[0] = -1

    # P1: ALT_t >= RTL_DESCEND_ALT
    # Request parameter
    target_param_ready = 0
    target_param_value = 0

    master.mav.param_request_read_send(master.target_system, master.target_component, 'RTL_DESCEND_ALT', -1)

    target_param = "RTL_DESCEND_ALT"
    count = 0
    while target_param_ready == 0 and count < 5:
        time.sleep(1)
        count += 1

    ## (ALT_t - RTL_DESCEND_ALT) / ALT_t
    if (current_altitude == 0) or (current_altitude - target_param_value == 0):
        P[1] = 0
    else:
        P[1] = (current_altitude - target_param_value) / current_altitude

    # P2: POS_t = home_position
    if lat_avg == home_lat and lon_avg == home_lon:
        P[2] = 1
    else:
        P[2] = -1

    # P3: current_flight_mode = LAND
    if current_flight_mode == "LAND":
        P[3] = -1
    else:
        P[3] = 1

    if PRINT_DEBUG == 1:
        print("[Debug] Mode_(t-1):%s, Mode_t:%s, RTL_DESCEND_ALT:%f, ALT_t:%f, ALT_(t-1):%f" % (
        previous_flight_mode, current_flight_mode, target_param_value, round(current_altitude, 0),
        round(previous_altitude, 0)))
        print("[Debug] lat:%f, lon:%f, home_lat:%f, home_lon:%f" % (lat_avg, lon_avg, home_lat, home_lon))

    Global_distance = -1 * (min(P[0], P[1], P[2], P[3]))

    print_distance(G_dist=Global_distance, P_dist=P, length=4, policy="PX.RTL3", guid=guidance)
    # ----------------------- (end) PX.RTL3 policy -----------------------
    # ----------------------- (end) PX.RTL3 policy -----------------------

    lat_avg = lat_avg / 1000
    lat_avg = lat_avg * 1000
    lon_avg = lon_avg / 1000
    lon_avg = lon_avg * 1000

    # ----------------------- (start) PX.RTL2 policy -----------------------
    # ----------------------- (start) PX.RTL2 policy -----------------------
    # P0: Mode_t = RTL
    if current_flight_mode == "RTL":
        P[0] = 1
    else:
        P[0] = -1

    # P1: ALT_t >= RTL_DESCEND_ALT
    # Request parameter
    target_param_ready = 0
    target_param_value = 0

    master.mav.param_request_read_send(master.target_system, master.target_component, 'RTL_DESCEND_ALT', -1)

    target_param = "RTL_DESCEND_ALT"
    count = 0
    while target_param_ready == 0 and count < 5:
        time.sleep(1)
        count += 1

    ## (ALT_t - RTL_DESCEND_ALT) / ALT_t
    if (current_altitude == 0) or (current_altitude - target_param_value == 0):
        P[1] = 0
    else:
        P[1] = (current_altitude - target_param_value) / current_altitude

    # P2: POS_t != home_position
    if lat_avg != home_lat and lon_avg != home_lon:
        P[2] = 1
    else:
        P[2] = -1

    # P3: POS_t != POS_(t-1)
    if round(ground_speed, 0) != 0:
        P[3] = -1
    else:
        P[3] = 1

    # P4: ALT_t = ALT_(t-1)
    if round(current_altitude, 0) == round(previous_altitude, 0):
        P[4] = -1
    else:
        P[4] = 1

    if PRINT_DEBUG == 1:
        print("[Debug] Mode_t:%s, RTL_DESCEND_ALT:%f, ALT_t:%f, ALT_(t-1):%f" % (
        current_flight_mode, target_param_value, round(current_altitude, 0), round(previous_altitude, 0)))
        print("[Debug] lat:%f, lon:%f, home_lat:%f, home_lon:%f, ground_speed:%d" % (
        lat_avg, lon_avg, home_lat, home_lon, round(ground_speed, 0)))

    Global_distance = -1 * (min(P[0], P[1], P[2], max(P[3], P[4])))

    print_distance(G_dist=Global_distance, P_dist=P, length=5, policy="PX.RTL2", guid=guidance)
    # ----------------------- (end) PX.RTL2 policy -----------------------
    # ----------------------- (end) PX.RTL2 policy -----------------------

    # ----------------------- (start) PX.GPS.FS1 policy -----------------------
    # ----------------------- (start) PX.GPS.FS1 policy -----------------------
    # P0: GPS_loss = on
    if GPS_status == 0:
        P[0] = 1
    else:
        P[0] = -1

    # P1: [0, k] GPS_failsafe = on

    # Request parameter
    target_param_ready = 0
    target_param_value = 0

    master.mav.param_request_read_send(master.target_system, master.target_component, 'COM_POS_FS_DELAY', -1)

    target_param = "COM_POS_FS_DELAY"
    count = 0
    while target_param_ready == 0 and count < 5:
        time.sleep(1)
        count += 1

    if gps_failsafe_error == 1:
        P[1] = -1
    else:
        P[1] = 1

    Global_distance = -1 * (min(P[0], P[1]))

    if (Global_distance < 0) and (gps_failsafe_count < target_param_value):
        gps_failsafe_count = gps_failsafe_count + 1
        Global_distance = -1 * Global_distance

    if PRINT_DEBUG == 1:
        print("[Debug] GPS_loss:%d, GPS_failsafe:%d, COM_POS_FS_DELAY:%d" % (
        GPS_status, gps_failsafe_error, target_param_value))

    print_distance(G_dist=Global_distance, P_dist=P, length=2, policy="PX.GPS.FS1", guid=guidance)
    # ----------------------- (end) PX.GPS.FS1 policy -----------------------
    # ----------------------- (end) PX.GPS.FS1 policy -----------------------

    # ----------------------- (start) PX.GPS.FS2 policy -----------------------
    # ----------------------- (start) PX.GPS.FS2 policy -----------------------
    # P0: GPS_failsafe = on
    if gps_failsafe_error == 1:
        P[0] = 1
    else:
        P[0] = -1

    # P1: RC_t = on
    if (current_rc_1 != 0) or (current_rc_2 != 0) or (current_rc_3 != 0) or (current_rc_4 != 0):
        P[1] = 1
    else:
        P[1] = -1

    # P2: Mode_t = ALTITUDE
    if current_flight_mode == "ALTCTL":
        P[2] = -1
    else:
        P[2] = 1

    Global_distance = -1 * (min(P[0], P[1], P[2]))

    if PRINT_DEBUG == 1:
        print("[Debug] GPS_failsafe:%d, RC3:%d, Mode_t:%s" % (gps_failsafe_error, current_rc_3, current_flight_mode))

    print_distance(G_dist=Global_distance, P_dist=P, length=3, policy="PX.GPS.FS2", guid=guidance)
    # ----------------------- (end) PX.GPS.FS2 policy -----------------------
    # ----------------------- (end) PX.GPS.FS2 policy -----------------------

    # ----------------------- (start) PX.GPS.FS3 policy -----------------------
    # ----------------------- (start) PX.GPS.FS3 policy -----------------------
    # P0: GPS_failsafe = on
    if gps_failsafe_error == 1:
        P[0] = 1
    else:
        P[0] = -1

    # P1: RC_t = off
    if (current_rc_1 == 0) and (current_rc_2 == 0) and (current_rc_3 == 0) and (current_rc_4 == 0):
        P[1] = 1
    else:
        P[1] = -1

    # P2: Mode_t = LAND
    if current_flight_mode == "LAND":
        P[2] = -1
    else:
        P[2] = 1

    Global_distance = -1 * (min(P[0], P[1], P[2]))

    # if PRINT_DEBUG == 1:
    print("[Debug] GPS_failsafe:%d, RC3:%d, Mode_t:%s" % (gps_failsafe_error, current_rc_3, current_flight_mode))

    print_distance(G_dist=Global_distance, P_dist=P, length=3, policy="PX.GPS.FS3", guid=guidance)
    # ----------------------- (end) PX.GPS.FS3 policy -----------------------
    # ----------------------- (end) PX.GPS.FS3 policy -----------------------

    target_param_ready = 0
    target_param = ""
    target_param_value = 0


# ------------------------------------------------------------------------------------
# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if id < 1:
        print("Channel does not exist.")
        return

    # We only have 8 channels
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    if id < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id - 1] = pwm

        # global master

        master.mav.rc_channels_override_send(
            master.target_system,  # target_system
            master.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.


# ------------------------------------------------------------------------------------
def throttle_th():
    global goal_throttle

    while True:
        set_rc_channel_pwm(3, goal_throttle)
        time.sleep(0.2)

# ------------------------------------------------------------------------------------
def main(argv):
    selected_config_param = ""
    # (Start) Parse command line arguments (i.e., input and output file)
    try:
        opts, args = getopt.getopt(argv, "ha:pt:n:v:i:d:f:s:ec:",
                                   ["attack_type=", "test_time=", "under_normal=", "vehicle_type=",
                                    "intensity_of_attack=", "directory=", "mode=", "--sw_version", "--config_param"])

    except getopt.GetoptError:
        print("fuzzing.py -a <GPS_spoofing/GPS_jamming> -p -t <test time> -n <yes/no>")
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print("fuzzing.py -a <choose one of the following attacks: GPS_spoofing/GPS_jamming>")
            sys.exit()
        if opt in ("-a", "--attack_type"):
            attack_type = arg
        if opt == '-p':
            attack_profile = "on"
            print("[fuzzing.py] Attack profile is turned on.")
        if opt in ("-t", "--test_time"):
            test_time = int(arg)
            print("[fuzzing.py] User wants to test this attack for %d seconds" % test_time)
        if opt in ("-n", "--under_normal"):
            under_normal = str(arg)
            if under_normal == "yes":
                print("[fuzzing.py] This simulation is for measuring the RV's states under the *normal*.")
            elif under_normal == "no":
                print("[fuzzing.py] This simulation is for measuring the RV's states under the *attack*.")
        if opt in ("-v", "--vehicle_type"):
            target_vehicle_type = str(arg)
            print("[fuzzing.py] Current selected type: %s" % target_vehicle_type)
        if opt in ("-i", "--intensity_of_attack"):
            intensity_of_attack = float(arg)
            print("[fuzzing.py] intensity of attack:%d" % intensity_of_attack)
        if opt in ("-d", "--directory"):
            logs_path = str(arg)
            print("[fuzzing.py] log directory path:%s" % logs_path)
        if opt in ("-f", "--mode"):
            target_flight_mode = str(arg)
            print("[fuzzing.py] target flight mode:%s" % target_flight_mode)
        if opt in ("-s", "--sw_version"):
            software_path = str(arg)
        if opt == '-e':
            environment_mutate = "on"
        if opt in ("-c", "--config_param"):
            selected_config_param = str(arg)

    # (End) Parse command line arguments (i.e., input and output file)


    master.wait_heartbeat()

    # request data to be sent at the given rate
    """
    for i in range(0, 3):
        master.mav.request_data_stream_send(master.target_system, master.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 6, 1)
    """
    message = master.recv_match(type='VFR_HUD', blocking=True)
    home_altitude = message.alt
    print("home_altitude: %f" % home_altitude)

    """
    message = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    home_lat = message.lat
    home_lat = home_lat / 1000
    home_lat = home_lat * 1000
    home_lon = message.lon
    home_lon = home_lon / 1000
    home_lon = home_lon * 1000
    print("home_lat: %f, home_lon: %f" % (home_lat, home_lon))
    """

    if (target_vehicle_type == "gazebo_uuv_hippocampus") or (target_vehicle_type == "gazebo_boat"):
        # Waypoints for ground and aerial vehicles
        waypoints = [
            (47.3334475, 8.5471141),
            (47.335880359491306, 8.549119400387639),
            (47.334880359491306, 8.549119400387639),
            (47.333880359491306, 8.549119400387639),
            (47.332880359491306, 8.549119400387639),
        ]
    else:
        # Waypoints for underwater vehicles
        waypoints = [
            (47.3977432, 8.5455942),
            (47.39840927148607, 8.538724386611221),
            (47.395953691724266, 8.53900320417631),
            (47.396011708552194, 8.545342729320595),
            (47.393217509130956, 8.53959763645247),
        ]

    wp = mavwp.MAVWPLoader()
    seq = 0
    for waypoint in enumerate(waypoints):
        seq = waypoint[0]
        lat, lon = waypoint[1]
        # lon = waypoint[1]
        altitude = 520
        autocontinue = 1
        current = 0
        param1 = 15.0  # minimum pitch
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        if seq == 0:  # first waypoint to takeoff
            current = 1
            p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, seq, frame,
                                                             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                                             current, autocontinue, 0, 0, 0, 0, lat, lon, altitude)
        elif seq == len(waypoints) - 1: # last waypoint to land
            p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, seq, frame,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                current, autocontinue, 0, 0, 0, 0, lat, lon, altitude)
        else:
            p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, seq, frame,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                current, autocontinue, 0, 0, 0, 0, lat, lon, altitude)

        wp.add(p)

    # Send Home location
    home_location = waypoints[0]
    cmd_set_home(home_location, 0)
    msg = master.recv_match(type=['COMMAND_ACK'], blocking=True)
    print("%s" % msg)
    print("Set home location: %f, %f" % (home_location[0], home_location[1]))

    # time.sleep(1)

    # Send Waypoint to airframe
    print("Send Waypoint to airframe")
    master.waypoint_clear_all_send()
    master.waypoint_count_send(wp.count())

    for i in range(wp.count()):
        msg = master.recv_match(type=['MISSION_REQUEST'], blocking=True)
        print("%s" % msg)
        master.mav.send(wp.wp(msg.seq))
        print("Sending waypoint %d" % msg.seq)

    msg = master.recv_match(type=['MISSION_ACK'], blocking=True)  # OKAY
    print("%s, %d" % (msg, msg.type))

    # Read Waypoint from airframe
    master.waypoint_request_list_send()
    waypoint_count = 0

    msg = master.recv_match(type=['MISSION_COUNT'], blocking=True)
    waypoint_count = msg.count
    print("%d", msg.count)

    for i in range(waypoint_count):
        master.waypoint_request_send(i)
        msg = master.recv_match(type=['MISSION_ITEM'], blocking=True)
        print("Receving waypoint %d" % msg.seq)
        print("%s" % msg)

    master.mav.mission_ack_send(master.target_system, master.target_component, 0)  # OKAY

    # Change Mission Mode
    PX4_CUSTOM_MAIN_MODE_AUTO = 4.0
    PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4.0
    PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5.0

    auto_mode_flags = mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED | mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

    PX4_MAV_MODE = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_mode_flags

    # master.mav.set_mode_send(1, PX4_MAV_MODE, PX4_CUSTOM_MAIN_MODE_AUTO)

    # msg = master.recv_match(type=['COMMAND_ACK'], blocking=True)
    # print ("%s", msg)

    # Send Heartbeat
    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4)

    master.wait_heartbeat()
    print("HEARTBEAT OK\n")

    change_flight_mode("hold")
    #time.sleep(5)

    # Arming
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 21196, 0, 0, 0, 0, 0)

    msg = master.recv_match(type=['COMMAND_ACK'], blocking=True)
    print("%s" % msg)
    master.motors_armed_wait()

    print("%d" % master.target_system)
    print("%d" % master.target_component)

    master.mav.command_long_send(
                    master.target_system,  # target_system
                    master.target_component, # target_component
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # command
                    0, # confirmation
                    1, # param1
                    0, # param2
                    0, # param3
                    0, # param4
                    0, # param5
                    0, # param6
                    50) # param7- altitude
    msg = master.recv_match(type=['COMMAND_ACK'],blocking=True)
    print ("%s" % msg)

    goal_throttle = 1800
    t1 = threading.Thread(target=throttle_th, args=())
    t1.daemon = True
    t1.start()

    t2 = threading.Thread(target=read_loop, args=())
    t2.daemon = True
    t2.start()

    change_flight_mode("altitude")
    time.sleep(1)

    change_flight_mode("mission")
    # MISSION START
    master.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
    #msg = master.recv_match(type=['COMMAND_ACK'],blocking=True)
    # print msg

    #time.sleep(10)
    # change_flight_mode("altitude")
    change_flight_mode("mission")


    # Check liveness of the RV software
    t3 = threading.Thread(target=check_liveness, args=())
    t3.daemon = True
    t3.start()

    time.sleep(2)
    change_flight_mode(target_flight_mode)
    #--------------------------------------------------------------------------------------------------
    # Mutating a selected configuration parameter
    if selected_config_param:
        for line in open("list_of_params.txt", 'r').readlines():
            if selected_config_param in line:
                row = line.rstrip().split(',')
                param_name = str(row[0])
                param_range = str(row[1])
                param_default = float(row[2])

                param_min = 0.0
                param_max = 10.0
                # Case 1: There is no any specified range.
                if param_range == "":
                    print("[Debug] param:%s does not have a specified range." % param_name)

                # Case 2: The range only specifies the minimum.
                elif "min" in param_range:
                    range_of_param = param_range.rstrip().split(':')
                    param_min = float(range_of_param[1])

                # Case 3: The range only specifies the maximum.
                elif "max" in param_range:
                    range_of_param = param_range.rstrip().split(':')
                    param_max = float(range_of_param[1])

                # Case 4: The range specifies the minimum and maximum.
                else:
                    range_of_param = param_range.rstrip().split(' ')
                    param_min = float(range_of_param[0])
                    param_max = float(range_of_param[1])

                param_value = random.uniform(param_min, param_max)

                print("[Debug] Assigning value:%f to param:%s" % (param_value, param_name))

                master.mav.param_set_send(master.target_system, master.target_component, param_name, param_value,
                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    #--------------------------------------------------------------------------------------------------
    # Main loop
    cnt = 0
    while True:
        time.sleep(1)
        print("[Debug] [vehicle_status:%d] [remaining test time:%d]" % (drone_status, test_time - cnt))

        cnt = cnt +1

        if cnt >= test_time:

            if selected_config_param:
                print("[Debug] Assigning default value:%f to param:%s" % (param_default, param_name))

                master.mav.param_set_send(master.target_system, master.target_component, param_name, param_default,
                                          mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                time.sleep(3)

            # For closing the simulator
            open("shared_variables.txt", "w").close()
            f = open("shared_variables.txt", "w")
            f.write("reboot")
            f.flush()
            time.sleep(1)
            f.close()
            time.sleep(1)
            break

if __name__ == "__main__":
    main(sys.argv[1:])