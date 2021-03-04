# FUNCTIONS TO CLEAN RAW DATA FILES

# import required packages
import glob
import os
import pandas as pd
import numpy as np
from math import radians, cos, sin, asin, sqrt

# This function converts units in raw drone log files to metric and drops unneeded data columns.
def unit_conversion(directory, new_directory):
    # get list of drone log files from provided directory (directory must ONLY contain drone log files)
    files = glob.glob(directory+'*.csv')
    # make new directory if it doesn't already exist
    if not os.path.exists(new_directory):
        os.makedirs(new_directory)
    for i in files:
        # for each file, load into a data frame and strip white spaces from column names
        df = pd.read_csv(i)
        df.columns = df.columns.str.strip()
        # convert imperial unit columns to metric
        df['altitude_m'] = df['altitude(feet)'] *0.3048
        df['ascent_m'] = df['ascent(feet)'] *0.3048
        df['speed_kph'] = df['speed(mph)'] *1.60934
        df['distance_m'] = df['distance(feet)'] *0.3048
        df['max_altitude_m'] = df['max_altitude(feet)'] *0.3048
        df['max_ascent_m'] = df['max_ascent(feet)'] *0.3048
        df['max_speed_kph'] = df['max_speed(mph)'] *1.60934
        df['max_distance_m'] = df['max_distance(feet)'] *0.3048
        # rename columns for consistency
        df['time_millisecond'] = df['time(millisecond)']
        df['datetime_utc'] = df['datetime(utc)']
        df['gimbal_pitch_degrees'] = df['gimbal_pitch(degrees)']

        # drop imperial unit columns & columns not needed in analyses, and reorder remaining columns
        cols_to_keep = ['latitude', 'longitude', 'time_millisecond', 'datetime_utc', 'altitude_m',
        'ascent_m', 'speed_kph', 'distance_m', 'max_altitude_m', 'max_ascent_m', 'max_speed_kph',
        'max_distance_m', 'rc_throttle', 'gimbal_pitch_degrees', 'battery_percent', 'flycState', 'message']
        df.drop(df.columns.difference(cols_to_keep), axis = 1, inplace = True)
        df = df[cols_to_keep]
        # save the new dataframe in a new folder with '_metric' appended to the filename
        df.to_csv(new_directory + os.path.splitext(os.path.basename(i))[0] + '_metric.csv', index = False)

# This function calculates the distance between two GPS points. Source: https://stackoverflow.com/a/45395941
def haversine(lat1, lon1, lat2, lon2):
    R = 6378.137 # Earth's radius at equator
    if np.isnan([lat1, lon1, lat2, lon2]).any():
        result = np.nan
    else:
        dLat = radians(lat2 - lat1)
        dLon = radians(lon2 - lon1)
        lat1 = radians(lat1)
        lat2 = radians(lat2)

        a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
        c = 2*asin(sqrt(a))
        result = R * c

    return result
