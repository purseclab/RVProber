from concurrent.futures import ProcessPoolExecutor, as_completed
import traceback
import matplotlib.pyplot as plt
import pandas as pd
import pickle
from tqdm import tqdm
import json
import os
import px4tools
import numpy as np
from html import escape
from pyulog.px4 import PX4ULog
from pyulog import *
import sys

# adding FlightReview module to the system path
sys.path.insert(0, '.\\flight_review-main\\app\\plot_app')
from config_tables import flight_modes_table
from db_entry import *
from helper import get_airframe_name, get_flight_mode_changes, load_ulog_file

pickle.HIGHEST_PROTOCOL = 4

# Gathering one row worth of data
def extract_fields_from_log(log_metadata, log_link, ulog):
    row_data = {}
    row_data['link'] = log_link
    row_data['upload_date'] = log_metadata["log_date"]
    row_data['description'] = log_metadata["log_description"].replace("<wbr />", "")
    row_data['rating'] = log_metadata["rating"]
    row_data['error'] = log_metadata["error_labels"]

    px4_ulog = PX4ULog(ulog)

    # GPS Present
    try:
        gps_data = ulog.get_dataset('vehicle_gps_position')
        row_data['gps_data'] = "Yes"
    except:
        row_data['gps_data'] = "No"

    # Airframe
    try:
        row_data['airframe_type'] = px4_ulog.get_mav_type()
    except:
        row_data['airframe_type'] = log_metadata['airframe_type']
    try:
        row_data['airframe_name'] = get_airframe_name(ulog)[0]
    except:
        row_data['airframe_name'] = log_metadata['airframe_name']

    # Hardware
    try:
        row_data['hardware'] = ulog.msg_info_dict['ver_hw']
    except:
        row_data['hardware'] = log_metadata['ver_hw']

    # Software
    branch_info = ''
    if 'ver_sw_branch' in ulog.msg_info_dict:
        branch_info = ' (branch: ' + ulog.msg_info_dict['ver_sw_branch']+')'
    if 'ver_sw' in ulog.msg_info_dict:
        ver_sw = escape(ulog.msg_info_dict['ver_sw'])
        row_data['software'] = ver_sw + branch_info

    # Flight Duration
    m, s = divmod(int((ulog.last_timestamp - ulog.start_timestamp)/1e6), 60)
    h, m = divmod(m, 60)
    row_data['flight_duration'] = '{:d}:{:02d}:{:02d}'.format(h, m, s)

    # Logging Start Time
    try:
        # get the first non-zero timestamp
        gps_data = ulog.get_dataset('vehicle_gps_position')
        indices = np.nonzero(gps_data.data['time_utc_usec'])
        if len(indices[0]) > 0:
            # we use the timestamp from the log and then convert it with JS to
            # display with local timezone.
            # In addition we add a tooltip to show the timezone from the log
            logging_start_time = int(gps_data.data['time_utc_usec'][indices[0][0]] / 1000000)

            utc_offset_min = ulog.initial_parameters.get('SDLOG_UTC_OFFSET', 0)
            utctimestamp = datetime.datetime.utcfromtimestamp(
                logging_start_time+utc_offset_min*60).replace(tzinfo=datetime.timezone.utc)

            row_data['start_time'] = str(utctimestamp.strftime('%d-%m-%Y %H:%M'))
    except:
        # Ignore. Eg. if topic not found
        row_data['start_time'] = log_metadata['start_time']

    # Flight Modes
    try:
        flight_mode_changes = get_flight_mode_changes(ulog)
        flight_mode_changes = filter(lambda elem: elem[1] != -1, flight_mode_changes)
        flight_mode_names = map(lambda elem: flight_modes_table[elem[1]][0], flight_mode_changes)
        row_data['flight_modes'] = list(set(flight_mode_names))
    except:
        row_data['flight_modes'] = log_metadata['flight_modes']

    # Altitude Information
    try:
        local_pos = ulog.get_dataset('vehicle_local_position')
        pos_z = local_pos.data['z']
        row_data['altitude_min'] = np.amin(pos_z)
        row_data['altitude_max'] = np.amax(pos_z)
        row_data['altitude_avg'] = np.mean(pos_z)
    except:
        row_data['altitude_min'] = ""
        row_data['altitude_max'] = ""
        row_data['altitude_avg'] = ""

    # Parameters
    row_data['parameters'] = ulog.initial_parameters

    # Terrain Following
    try:
        if row_data['parameters']['MPC_ALT_MODE'] in [1, 2]:
            row_data['terrain_following'] = "Yes"
        else:
            row_data['terrain_following'] = "No"
    except:
        row_data['terrain_following'] = "No"

    # Terrain Following
    try:
        if row_data['parameters']['COM_OBS_AVOID'] == 1:
            row_data['object_avoidance'] = "Yes"
        else:
            row_data['object_avoidance'] = "No"
    except:
        row_data['object_avoidance'] = "No"

    # Remote control stuff
    remote_control = False
    try:
        # Get manual control values
        df = pd.DataFrame(ulog.get_dataset('manual_control_setpoint').data)
        df = df[['y', 'x', 'r', 'z', 'aux1', 'aux2']]
        for (i, value) in df.std().items():
            if value != 0:
                remote_control = True
                break
    except:
        try:
            # Get raw radio control values
            df_data = ulog.get_dataset('rc_channels').data
            df = pd.DataFrame(df_data)
            num_rc_channels = min(8, np.amax(df_data['channel_count']))
            channel_cols = ['channels['+str(i)+']' for i in range(num_rc_channels)]
            df = df[channel_cols]
            for (i, value) in df.std().items():
                if value != 0:
                    remote_control = True
                    break
        except:
            pass
    row_data['remote_control'] = remote_control

    kill_switch_engaged = False
    try:
        # Get flight mode/ killswitch values
        df_data = ulog.get_dataset('manual_control_switches').data
        df = pd.DataFrame(df_data)
        df = df[['mode_slot', 'kill_switch']]
        df['kill_switch'] = df['kill_switch'] == 1
        kill_switch_engaged = True in df['kill_switch'].unique()
    except:
        pass
    row_data['kill_switch_engaged'] = kill_switch_engaged

    # Parse console messages from OS to capture GPS module information
    console_messages = []
    row_data['gps_metadata'] = ""
    if 'boot_console_output' in ulog.msg_info_multiple_dict:
        console_output = ulog.msg_info_multiple_dict['boot_console_output'][0]
        console_output = ''.join(console_output)
        console_messages = console_output.split("\n")
    if len(console_messages) > 0:
        row_data['gps_metadata'] = "; ".join([f"<{message}>" for message in console_messages if "INFO  [gps]" in message])

    # Stuff that I did not find
    row_data['gyroscope'] = ""
    row_data['mag_accel'] = ""
    row_data['barometer'] = ""
    row_data['compass'] = ""
    row_data['soft_release_date'] = ""
    row_data['waypoints'] = ""

    # Returning one row of data
    return row_data


def process_log_file(log_metadata):
    try:
        log_file = os.path.join("C:\\px4_logs", log_metadata['log_id'] + ".ulg")
        log_link = f"https://review.px4.io/plot_app?log={log_metadata['log_id']}"
        ulog = load_ulog_file(log_file)
        processed_data = extract_fields_from_log(log_metadata, log_link, ulog)
        return processed_data
    except Exception as e:
        return None


if __name__ == "__main__":
    log_db_file = "F:\\flight_review-main\\app\\download_db.json"
    log_db = json.load(open(log_db_file))

    results = []
    failed = []
    with tqdm(total=len(log_db)) as pbar:
        with ProcessPoolExecutor(max_workers=os.cpu_count()//2) as ex:
            futures = [ex.submit(process_log_file, log_metadata) for log_metadata in log_db]
            for future in as_completed(futures):
                processed_data = future.result()
                pbar.update(1)
                if processed_data:
                    results.append(processed_data)                

    # Save output to an Excel Sheet
    df = pd.DataFrame.from_dict(results)
    new_df = df[['link', 'upload_date', 'gps_data', 'gps_metadata', 'description', 'airframe_type', 'airframe_name', 'hardware', 'gyroscope', 'mag_accel', 'barometer', 'compass', 'software', 'flight_duration', 'start_time', 'rating',
                 'error', 'flight_modes', 'soft_release_date', 'remote_control', 'kill_switch_engaged', 'altitude_min', 'altitude_avg', 'altitude_max', 'waypoints', 'terrain_following', 'object_avoidance', 'parameters']]

    # Handle mistyped hardware names
    new_df['hardware'] = new_df['hardware'].replace(['PX4FMU_V2'], 'PX4_FMU_V2')
    new_df['hardware'] = new_df['hardware'].replace(['PX4FMU_V4'], 'PX4_FMU_V4')
    new_df['hardware'] = new_df['hardware'].replace(['PX4FMU_V4PRO'], 'PX4_FMU_V4PRO')
    new_df['hardware'] = new_df['hardware'].replace(['PX4FMU_V5'], 'PX4_FMU_V5')

    # Update the sensors available in each hardware based on "hardware_lookup.json"
    hw_lookup = json.load(open("hardware_lookup.json"))
    for hw in hw_lookup:
        g = hw_lookup[hw]["gyroscope"]
        m = hw_lookup[hw]["mag_accel"]
        b = hw_lookup[hw]["barometer"]
        c = hw_lookup[hw]["compass"]
        if g or m or b or c:
            print(f"Updating for {hw}")
            hw_mask = new_df['hardware'] == hw
            new_df.loc[hw_mask, "gyroscope"] = g
            new_df.loc[hw_mask, "mag_accel"] = m
            new_df.loc[hw_mask, "barometer"] = b
            new_df.loc[hw_mask, "compass"] = c

    # Saving processed DataFrame to file
    new_df.to_hdf('parsed_df2.hdf', 'df')
    new_df.to_excel('parsed_df2.xlsx', index=False, engine='xlsxwriter')
