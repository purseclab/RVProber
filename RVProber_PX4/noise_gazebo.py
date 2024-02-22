# Date: 3/12/2023
# Author: Hyungsub Kim
# Goal: In order to inject noises into sensors, this python script re-writes Gazebo plugin files.

"""
(Feb 18, 2024) To-do list
- [V] Insert wind plugin
- [V] Enable magnetometer attacks
- [V] Testing other software versions (e.g., 1.9, 1.10, 1.11, 1.12, 1.13)
- [V] Create a bach script to automatically download different PX4 versions
- [V] Enable all other attacks
- Create an interface to receive a user's inputs
- Enable testing multiple software versions

"""

import sys, os, getopt
import shutil
import time

"""
Execute the following command on a terminal
export PX4_HOME=/home/hskim/PX4
export PX4_1=/home/hskim/PX4/px4_1_15_0
export PX4_2=/home/hskim/PX4/px4_1_14_1
export PX4_3=/home/hskim/PX4/px4_1_13_3
export PX4_4=/home/hskim/PX4/px4_1_13_2
export PX4_5=/home/hskim/PX4/px4_1_13_1
export PX4_6=/home/hskim/PX4/px4_1_13_0
export PX4_7=/home/hskim/PX4/px4_1_12_3
export PX4_8=/home/hskim/PX4/px4_1_12_2
export PX4_9=/home/hskim/PX4/px4_1_12_1
export PX4_10=/home/hskim/PX4/px4_1_12_0
"""

content_imu = []
content_mag = []
content_baro = []
content_gps = []
content_opticalflow = []
content_world = []
content_preflight = []


file_imu_intact = ""
file_imu = ""

file_mag_intact = ""
file_mag = ""

file_baro_intact = ""
file_baro = ""

file_gps_intact = ""
file_gps = ""

file_opticalflow_intact = ""
file_opticalflow = ""

world_file_intact = ""
world_file_new = ""

preflight_file_intact = ""
preflight_file_new = ""

attack_type = ""
intensity_of_attack = 0
software_path = ""
environment_mutate = 0
number_of_compromised_sensor = 0

def find_file_path(target):
    global file_imu_intact
    global file_imu

    global file_mag_intact
    global file_mag

    global file_baro_intact
    global file_baro

    global file_gps_intact
    global file_gps

    global file_opticalflow_intact
    global file_opticalflow

    global world_file_new
    global world_file_intact

    global preflight_file_intact
    global preflight_file_new

    PX4_PATH = os.getenv(target)
    print(PX4_PATH);

    if PX4_PATH is None:
        raise Exception("PX4_PATH environment variable is not set!")

    file_imu_intact = PX4_PATH + '/Tools/sitl_gazebo/src/gazebo_imu_plugin_intact.cpp'
    file_imu = PX4_PATH + '/Tools/sitl_gazebo/src/gazebo_imu_plugin.cpp'

    file_mag_intact = PX4_PATH + '/Tools/sitl_gazebo/src/gazebo_magnetometer_plugin_intact.cpp'
    file_mag = PX4_PATH + '/Tools/sitl_gazebo/src/gazebo_magnetometer_plugin.cpp'

    file_baro_intact = PX4_PATH + '/Tools/sitl_gazebo/src/gazebo_barometer_plugin_intact.cpp'
    file_baro = PX4_PATH + '/Tools/sitl_gazebo/src/gazebo_barometer_plugin.cpp'

    file_gps_intact = PX4_PATH + '/Tools/sitl_gazebo/src/gazebo_gps_plugin_intact.cpp'
    file_gps = PX4_PATH + '/Tools/sitl_gazebo/src/gazebo_gps_plugin.cpp'

    file_opticalflow_intact = PX4_PATH + '/Tools/sitl_gazebo/src/gazebo_opticalflow_plugin_intact.cpp'
    file_opticalflow = PX4_PATH + '/Tools/sitl_gazebo/src/gazebo_opticalflow_plugin.cpp'

    world_file_new = PX4_PATH + '/Tools/sitl_gazebo/worlds/empty.world'
    world_file_intact = PX4_PATH + '/Tools/sitl_gazebo/worlds/empty_intact.world'

    preflight_file_new = PX4_PATH + '/src/modules/commander/Commander.cpp'
    preflight_file_intact = PX4_PATH + '/src/modules/commander/Commander_intact.cpp'

    # The least version of PX4 uses a different path.
    if os.path.isfile(file_imu) != True:

        file_imu_intact = PX4_PATH + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_imu_plugin_intact.cpp'
        file_imu = PX4_PATH + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_imu_plugin.cpp'

        file_mag_intact = PX4_PATH + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_magnetometer_plugin_intact.cpp'
        file_mag = PX4_PATH + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_magnetometer_plugin.cpp'

        file_baro_intact = PX4_PATH + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_barometer_plugin_intact.cpp'
        file_baro = PX4_PATH + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_barometer_plugin.cpp'

        file_gps_intact = PX4_PATH + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_gps_plugin_intact.cpp'
        file_gps = PX4_PATH + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_gps_plugin.cpp'

        file_opticalflow_intact = PX4_PATH + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_opticalflow_plugin_intact.cpp'
        file_opticalflow = PX4_PATH + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src/gazebo_opticalflow_plugin.cpp'

        world_file_new = PX4_PATH + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world'
        world_file_intact = PX4_PATH + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty_intact.world'

        # When 'if condition' below is satisfied, it means that there is no an intact file.
        if os.path.isfile(file_imu_intact) != True:
            shutil.copy(file_imu, file_imu_intact)

        if os.path.isfile(file_mag_intact) != True:
            shutil.copy(file_mag, file_mag_intact)

        if os.path.isfile(file_baro_intact) != True:
            shutil.copy(file_baro, file_baro_intact)

        if os.path.isfile(file_gps_intact) != True:
            shutil.copy(file_gps, file_gps_intact)

        if os.path.isfile(file_opticalflow_intact) != True:
            shutil.copy(file_opticalflow, file_opticalflow_intact)

        if os.path.isfile(world_file_intact) != True:
            shutil.copy(world_file_new, world_file_intact)

        if os.path.isfile(preflight_file_intact) != True:
            shutil.copy(preflight_file_new, preflight_file_intact)

    else:
        # When 'if condition' below is satisfied, it means that there is no an intact file.
        if os.path.isfile(file_imu_intact) != True:
            shutil.copy(file_imu, file_imu_intact)

        if os.path.isfile(file_mag_intact) != True:
            shutil.copy(file_mag, file_mag_intact)

        if os.path.isfile(file_baro_intact) != True:
            shutil.copy(file_baro, file_baro_intact)

        if os.path.isfile(file_gps_intact) != True:
            shutil.copy(file_gps, file_gps_intact)

        if os.path.isfile(file_opticalflow_intact) != True:
            shutil.copy(file_opticalflow, file_opticalflow_intact)

        if os.path.isfile(world_file_intact) != True:
            shutil.copy(world_file_new, world_file_intact)

        if os.path.isfile(preflight_file_intact) != True:
            shutil.copy(preflight_file_new, preflight_file_intact)

# -------------------- (Start) [1] Inserting noises to IMU --------------------
def imu(compromised_sensor_cnt, intensity):
    global file_imu_intact
    global file_imu
    global content_imu

    # To simulate sound noise attacks, I increase the intensity of noises on gyroscope and accelerometer sensors.
    gyro_target = "gyroscope_bias_[i] = phi_g_d * gyroscope_bias_[i] +"
    gyro_add_noise = ["gyroscope_bias_[i] = phi_g_d * gyroscope_bias_[i] +",
                      "sigma_b_g_d * standard_normal_distribution_(random_generator_) * " + str(intensity) + ";", "}",
                      "else {", "gyroscope_bias_[i] = phi_g_d * gyroscope_bias_[i] +",
                      "sigma_b_g_d * standard_normal_distribution_(random_generator_);", "}"]

    accelerometer_target = "accelerometer_bias_[i] = phi_a_d * accelerometer_bias_[i] +"
    accelerometer_add_noise = ["accelerometer_bias_[i] = phi_a_d * accelerometer_bias_[i] +",
                               "sigma_b_a_d * standard_normal_distribution_(random_generator_) * " + str(
                                   intensity) + ";",
                               "}",
                               "else {", "accelerometer_bias_[i] = phi_a_d * accelerometer_bias_[i] +",
                               "sigma_b_g_d * standard_normal_distribution_(random_generator_);", "}"]

    with open(file_imu_intact) as fp:
        line = fp.readline()
        content_imu.append(line)

        while line:

            # print("Line {}: {}".format(cnt, line.strip()))
            line = fp.readline()

            # ---------- Add noises to gyro sensor(s) ----------
            if gyro_target in line:
                print("We find the gyro target location.");

                condition = ""
                if compromised_sensor_cnt == 0:
                    condition = "if (i < 0) {"
                elif compromised_sensor_cnt == 1:
                    condition = "if (i <= 0) {"
                elif compromised_sensor_cnt == 2:
                    condition = "if (i <= 1) {"
                elif compromised_sensor_cnt == 3:
                    condition = "if (i <= 2) {"

                content_imu.append(condition)
                for index in range(7):
                    content_imu.append(gyro_add_noise[index])

                line = fp.readline()
            # ------------------------------------------------------------


            # ---------- Add noises to accelerometer sensor(s) ----------
            elif accelerometer_target in line:
                print("We find the accelerometer target location.");

                condition = ""
                if compromised_sensor_cnt == 0:
                    condition = "if (i < 0) {"
                elif compromised_sensor_cnt == 1:
                    condition = "if (i <= 0) {"
                elif compromised_sensor_cnt == 2:
                    condition = "if (i <= 1) {"
                elif compromised_sensor_cnt == 3:
                    condition = "if (i <= 2) {"

                content_imu.append(condition)
                for index in range(7):
                    content_imu.append(accelerometer_add_noise[index])

                line = fp.readline()
            # ------------------------------------------------------------

            content_imu.append(line)

    """
    cnt = 1
    for i in content_imu:
        print("Line {}: {}".format(cnt, i.strip()))
        cnt += 1
    """

    with open(file_imu, 'w') as f:
        for item in content_imu:
            f.write("%s" % item)

    #content_imu.clear()
# -------------------- (End) [1] Inserting noises to IMU --------------------

# -------------------- (Start) [2] Inserting noises to magnetometer --------------------
def mag(compromised_sensor_cnt, intensity):
    global file_mag_intact
    global file_mag
    global content_mag

    # To simulate sound noise attacks, I increase the intensity of noises on gyroscope and accelerometer sensors.
    mag_target = "bias_[i] = phi_d * bias_[i] + sigma_b_d * standard_normal_distribution_(random_generator_);"
    mag_add_noise = ["bias_[i] = phi_d * bias_[i] + sigma_b_d * standard_normal_distribution_(random_generator_) * " + str(intensity) + ";",
                     "}",
                     "else {",
                     "bias_[i] = phi_d * bias_[i] + sigma_b_d * standard_normal_distribution_(random_generator_);",
                     "}"]

    with open(file_mag_intact) as fp:
        line = fp.readline()
        content_mag.append(line)

        while line:

            # print("Line {}: {}".format(cnt, line.strip()))
            line = fp.readline()

            # ---------- Add noises to gyro sensor(s) ----------
            if mag_target in line:
                print("We find the mag target location.");

                condition = ""
                if compromised_sensor_cnt == 0:
                    condition = "if (i < 0) {"
                elif compromised_sensor_cnt == 1:
                    condition = "if (i <= 0) {"
                elif compromised_sensor_cnt == 2:
                    condition = "if (i <= 1) {"
                elif compromised_sensor_cnt == 3:
                    condition = "if (i <= 2) {"

                content_mag.append(condition)
                for index in range(5):
                    content_mag.append(mag_add_noise[index])

                line = fp.readline()
            # ------------------------------------------------------------

            content_mag.append(line)
    """
    cnt = 1
    for i in content_mag:
        print("Line {}: {}".format(cnt, i.strip()))
        cnt += 1
    """

    with open(file_mag, 'w') as f:
        for item in content_mag:
            f.write("%s" % item)

    #content_mag.clear()
# -------------------- (End) [2] Inserting noises to magnetometer --------------------

# -------------------- (Start) [3] Inserting noises to barometer --------------------
def baro(intensity):
    global file_baro_intact
    global file_baro
    global content_baro

    # To simulate sound noise attacks, I increase the intensity of noises on gyroscope and accelerometer sensors.
    baro_target = "const float abs_pressure_noise = 1.0f * (float)y1;"
    baro_add_noise = "const float abs_pressure_noise = 1.0f * (float)y1 * " + str(intensity) + ";"

    with open(file_baro_intact) as fp:
        line = fp.readline()
        content_baro.append(line)

        while line:

            # print("Line {}: {}".format(cnt, line.strip()))
            line = fp.readline()

            # ---------- Add noises to gyro sensor(s) ----------
            if baro_target in line:
                print("We find the baro target location.");
                content_baro.append(baro_add_noise)

                line = fp.readline()
            # ------------------------------------------------------------

            content_baro.append(line)
    """
    cnt = 1
    for i in content_baro:
        print("Line {}: {}".format(cnt, i.strip()))
        cnt += 1
    """

    with open(file_baro, 'w') as f:
        for item in content_baro:
            f.write("%s" % item)

    #content_baro.clear()
# -------------------- (End) [3] Inserting noises to barometer --------------------

# -------------------- (Start) [4] Inserting noises to GPS --------------------
def gps(x, y, z, intensity):
    global file_gps_intact
    global file_gps
    global content_gps

    # To simulate GPS attacks, I increase the intensity of noises on GPS readings.
    ## Position
    gps_target_position_x = "noise_gps_pos_.X() = gps_xy_noise_density_ * sqrt(dt) * randn_(rand_);"
    gps_add_noise_position_x = "noise_gps_pos_.X() = gps_xy_noise_density_ * sqrt(dt) * randn_(rand_) * " + str(intensity) + ";\n"

    gps_target_position_y = "noise_gps_pos_.Y() = gps_xy_noise_density_ * sqrt(dt) * randn_(rand_);"
    gps_add_noise_position_y = "noise_gps_pos_.Y() = gps_xy_noise_density_ * sqrt(dt) * randn_(rand_) * " + str(intensity) + ";\n"

    gps_target_position_z = "noise_gps_pos_.Z() = gps_z_noise_density_ * sqrt(dt) * randn_(rand_);"
    gps_add_noise_position_z = "noise_gps_pos_.Z() = gps_z_noise_density_ * sqrt(dt) * randn_(rand_) * " + str(intensity) + ";\n"

    ## Velocity
    gps_target_velocity_x = "noise_gps_vel_.X() = gps_vxy_noise_density_ * sqrt(dt) * randn_(rand_);"
    gps_add_velocity_x = "noise_gps_vel_.X() = gps_vxy_noise_density_ * sqrt(dt) * randn_(rand_) * " + str(intensity) + ";\n"

    gps_target_velocity_y = "noise_gps_vel_.Y() = gps_vxy_noise_density_ * sqrt(dt) * randn_(rand_);"
    gps_add_velocity_y = "noise_gps_vel_.Y() = gps_vxy_noise_density_ * sqrt(dt) * randn_(rand_) * " + str(intensity) + ";\n"

    gps_target_velocity_z = "noise_gps_vel_.Z() = gps_vz_noise_density_ * sqrt(dt) * randn_(rand_);"
    gps_add_velocity_z = "noise_gps_vel_.Z() = gps_vz_noise_density_ * sqrt(dt) * randn_(rand_) * " + str(intensity) + ";\n"

    ## Random walk
    gps_target_rnd_walk_x = "random_walk_gps_.X() = gps_xy_random_walk_ * sqrt(dt) * randn_(rand_);"
    gps_add_rnd_walk_x = "random_walk_gps_.X() = gps_xy_random_walk_ * sqrt(dt) * randn_(rand_) * " + str(intensity) + ";\n"

    gps_target_rnd_walk_y = "random_walk_gps_.Y() = gps_xy_random_walk_ * sqrt(dt) * randn_(rand_);"
    gps_add_rnd_walk_y = "random_walk_gps_.Y() = gps_xy_random_walk_ * sqrt(dt) * randn_(rand_) * " + str(intensity) + ";\n"

    gps_target_rnd_walk_z = "random_walk_gps_.Z() = gps_z_random_walk_ * sqrt(dt) * randn_(rand_);"
    gps_add_rnd_walk_z = "random_walk_gps_.Z() = gps_z_random_walk_ * sqrt(dt) * randn_(rand_) * " + str(intensity) + ";\n"


    with open(file_gps_intact) as fp:
        line = fp.readline()
        content_gps.append(line)

        while line:

            # print("Line {}: {}".format(cnt, line.strip()))
            line = fp.readline()

            # ---------- Add noises to x-axis ----------
            if x == 1:
                if gps_target_position_x in line:
                    print("We find the gps (X) target location.");
                    content_gps.append(gps_add_noise_position_x)
                    line = fp.readline()
                elif gps_target_velocity_x in line:
                    content_gps.append(gps_add_velocity_x)
                    line = fp.readline()
                elif gps_target_rnd_walk_x in line:
                    content_gps.append(gps_add_rnd_walk_x)
                    line = fp.readline()
            # -------------------------------------------

            # ---------- Add noises to y-axis ----------
            if y == 1:
                if gps_target_position_y in line:
                    print("We find the gps (Y) target location.");
                    content_gps.append(gps_add_noise_position_y)
                    line = fp.readline()
                elif gps_target_velocity_y in line:
                    content_gps.append(gps_add_velocity_y)
                    line = fp.readline()
                elif gps_target_rnd_walk_y in line:
                    content_gps.append(gps_add_rnd_walk_y)
                    line = fp.readline()
            # -------------------------------------------

            # ---------- Add noises to z-axis ----------
            if z == 1:
                if gps_target_position_z in line:
                    print("We find the gps (Z) target location.");
                    content_gps.append(gps_add_noise_position_z)
                    line = fp.readline()
                elif gps_target_velocity_z in line:
                    content_gps.append(gps_add_velocity_z)
                    line = fp.readline()
                elif gps_target_rnd_walk_z in line:
                    content_gps.append(gps_add_rnd_walk_z)
                    line = fp.readline()
            # -------------------------------------------

            content_gps.append(line)
    """
    cnt = 1
    for i in content_mag:
        print("Line {}: {}".format(cnt, i.strip()))
        cnt += 1
    """

    with open(file_gps, 'w') as f:
        for item in content_gps:
            f.write("%s" % item)

    #content_gps.clear()
# -------------------- (End) [4] Inserting noises to GPS --------------------

# -------------------- (Start) [5] Inserting noises to optical flow --------------------
def optical_flow(x, y, intensity):
    global file_opticalflow_intact
    global file_opticalflow
    global content_opticalflow

    # To simulate GPS attacks, I increase the intensity of noises on GPS readings.
    ## Position
    opticalflow_target_x = "opticalFlow_message.set_integrated_x(quality ? flow_x_ang : 0.0f);"
    opticalflow_add_noise_x = "opticalFlow_message.set_integrated_x(quality ? flow_x_ang * " + str(intensity) + ": 0.0f);\n"

    opticalflow_target_y = "opticalFlow_message.set_integrated_y(quality ? flow_y_ang : 0.0f);"
    opticalflow_add_noise_y = "opticalFlow_message.set_integrated_y(quality ? flow_y_ang * " + str(intensity) + ": 0.0f);\n"

    with open(file_opticalflow_intact) as fp:
        line = fp.readline()
        content_opticalflow.append(line)

        while line:

            # print("Line {}: {}".format(cnt, line.strip()))
            line = fp.readline()

            # ---------- Add noises to x-axis ----------
            if x == 1:
                if opticalflow_target_x in line:
                    print("We find the opticalflow (X) target location.");
                    content_opticalflow.append(opticalflow_add_noise_x)
                    line = fp.readline()
            # -------------------------------------------

            # ---------- Add noises to y-axis ----------
            if y == 1:
                if opticalflow_target_y in line:
                    print("We find the opticalflow (Y) target location.");
                    content_opticalflow.append(opticalflow_add_noise_y)
                    line = fp.readline()
            # -------------------------------------------

            content_opticalflow.append(line)
    """
    cnt = 1
    for i in content_mag:
        print("Line {}: {}".format(cnt, i.strip()))
        cnt += 1
    """

    with open(file_opticalflow, 'w') as f:
        for item in content_opticalflow:
            f.write("%s" % item)

    #content_opticalflow.clear()
# -------------------- (End) [5] Inserting noises to optical flow --------------------

# -------------------- (Start) [6] EMI on SPI/I2C --------------------
def EMI(intensity):

    # * EMI attacks on SPI/I2C compromise all different types of sensor readings *

    # Insert noises to imu
    ## Function arguments: [The number of compromised sensors, Intensity]
    imu(3, intensity)

    # Insert noises to mag
    ## Function arguments: [The number of compromised sensors, Intensity]
    mag(3, intensity)

    # Insert noises to baro
    ## Function arguments: [Intensity]
    baro(intensity)

    # Insert noises to GPS
    ## Function arguments: [X, Y, Z, intensity]
    gps(1, 1, 1, intensity)

    # Insert noises to opticalflow
    ## Function arguments: [X, Y, intensity]
    optical_flow(1, 1, intensity)

# -------------------- (Start) [6] EMI on SPI/I2C --------------------

# -------------------- (Start) [7] Inserting noises to environment --------------------
def environment(intensity):
    global world_file_new
    global world_file_intact
    global content_world

    # To simulate sound noise attacks, I increase the intensity of noises on gyroscope and accelerometer sensors.
    ## The maximum wind speed will be two time larger than the mean wind speed.
    wind_target = "<physics name='default_physics' default='0' type='ode'>\n"
    wind_add_noise = ["    <plugin name='wind_plugin' filename='libgazebo_wind_plugin.so'>\n",
                      "      <frameId>base_link</frameId>\n",
                      "      <robotNamespace/>\n",
                      "      <windVelocityMean>" + str(intensity) + "</windVelocityMean>\n",
                      "      <windVelocityMax>" + str(intensity)*2 + "</windVelocityMax>\n",
                      "      <windVelocityVariance>" + str(intensity) + "</windVelocityVariance>\n",
                      "      <windDirectionMean>0 1 0</windDirectionMean>\n",
                      "      <windDirectionVariance>0</windDirectionVariance>\n",
                      "      <windGustStart>0</windGustStart>\n",
                      "      <windGustDuration>0</windGustDuration>\n",
                      "      <windGustVelocityMean>0</windGustVelocityMean>\n",
                      "      <windGustVelocityMax>20.0</windGustVelocityMax>\n",
                      "      <windGustVelocityVariance>0</windGustVelocityVariance>\n",
                      "      <windGustDirectionMean>1 0 0</windGustDirectionMean>\n",
                      "      <windGustDirectionVariance>0</windGustDirectionVariance>\n",
                      "      <windPubTopic>world_wind</windPubTopic>\n",
                      "    </plugin>\n"]


    with open(world_file_intact) as fp:
        line = fp.readline()
        content_world.append(line)

        while line:

            # print("Line {}: {}".format(cnt, line.strip()))
            line = fp.readline()

            # ---------- Insert wind plugin ----------
            if wind_target in line:
                print("We find the wind target location.");

                for index in range(17):
                    content_world.append(wind_add_noise[index])

            content_world.append(line)
    """
    cnt = 1
    for i in content_world:
        print("Line {}: {}".format(cnt, i.strip()))
        cnt += 1
    """

    with open(world_file_new, 'w') as f:
        for item in content_world:
            f.write("%s" % item)

    #content_world.clear()

# -------------------- (End) [7] Inserting noises to environment --------------------

# -------------------- (Start) [8] Disable preflight checks for 'forced arming' --------------------
def disable_preflight_check():
    global preflight_file_intact
    global preflight_file_new
    global content_preflight

    # To simulate sound noise attacks, I increase the intensity of noises on gyroscope and accelerometer sensors.
    preflight_target_1 = "// allow a grace period for re-arming:"
    preflight_new_1 = "\nrun_preflight_checks = false;\n"

    preflight_target_2 = "case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM:"
    preflight_target_3 = "break;"
    preflight_new_2 = "\ncmd_result = 1;\n"

    second_target_on = 0
    with open(preflight_file_intact) as fp:
        line = fp.readline()
        content_preflight.append(line)

        while line:

            # print("Line {}: {}".format(cnt, line.strip()))
            line = fp.readline()

            # ---------- Disable preflight checks ----------
            if preflight_target_1 in line:
                print("We find the first preflight target location.");
                content_preflight.append(preflight_new_1)

            if preflight_target_2 in line:
                second_target_on = 1

            if (second_target_on == 1) and (preflight_target_3 in line):
                print("We find the second preflight target location.");
                second_target_on = 0
                content_preflight.append(preflight_new_2)

            content_preflight.append(line)

    """
    cnt = 1
    for i in content_preflight:
        print("Line {}: {}".format(cnt, i.strip()))
        cnt += 1
    """

    with open(preflight_file_new, 'w') as f:
        for item in content_preflight:
            f.write("%s" % item)

    #content_preflight.clear()
# -------------------- (End) [8] Disable preflight checks for 'forced arming' --------------------

def main(argv):

    global attack_type
    global intensity_of_attack
    global software_path
    global environment_mutate
    global number_of_compromised_sensor

    # ---------- (Start) Parse command line arguments (i.e., input and output file) ----------
    try:
        opts, args = getopt.getopt(argv, "ha:pt:n:v:i:d:f:s:e:n:",
                                   ["attack_type=", "test_time=", "under_normal=", "vehicle_type=",
                                    "intensity_of_attack=", "directory=", "mode=", "--sw_version", "--intensity_of_env=","--number_of_compromised_sensor="])

    except getopt.GetoptError:
        print("noise_gazebo.py -a <GPS_spoofing/GPS_jamming> -p -t <test time> -n <yes/no>")
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print("noise_gazebo.py -a <choose one of the following attacks: GPS_spoofing/GPS_jamming>")
            sys.exit()
        if opt in ("-a", "--attack_type"):
            attack_type = arg
        if opt in ("-i", "--intensity_of_attack"):
            intensity_of_attack = float(arg)
            print("[noise_gazebo.py] intensity of attack:%d" % intensity_of_attack)
        if opt in ("-s", "--sw_version"):
            software_path = str(arg)
        if opt in ("-e", "--intensity_of_env"):
            environment_mutate = int(arg)
            print("[noise_gazebo.py] intensity of environment:%d" % environment_mutate)
        if opt in ("-n", "--number_of_compromised_sensor"):
            number_of_compromised_sensor = int(arg)

        # ---------- (End) Parse command line arguments (i.e., input and output file) ----------



    # (1)
    find_file_path(software_path)

    # (2) Disable preflight checks for 'forced arming'
    disable_preflight_check()

    # (3) Insert noises to imu
    ## Function arguments: [The number of compromised sensors, Intensity]
    if attack_type == "imu":
        imu(number_of_compromised_sensor, intensity_of_attack)

    # (4) Insert noises to mag
    ## Function arguments: [The number of compromised sensors, Intensity]
    if attack_type == "mag":
        mag(number_of_compromised_sensor, intensity_of_attack)

    # (5) Insert noises to baro
    ## Function arguments: [Intensity]
    if attack_type == "baro":
        baro(intensity_of_attack)

    # (6) Insert noises to GPS
    ## Function arguments: [X, Y, Z, intensity]
    if attack_type == "gps":
        gps(1, 1, 1, intensity_of_attack)

    # (7) Insert noises to opticalflow
    ## Function arguments: [X, Y, intensity]
    if attack_type == "opticalflow":
        optical_flow(1, 1, intensity_of_attack)

    # (8) EMI on SPI/I2C
    ## Function arguments: [Intensity]
    if attack_type == "EMI":
        EMI(intensity_of_attack)

    # (8) Insert noises to environment
    ## Function arguments: [Mean wind speed]
    environment(environment_mutate)

    #time.sleep(10)

if __name__ == "__main__":
    main(sys.argv[1:])
