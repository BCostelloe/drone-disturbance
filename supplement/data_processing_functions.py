# FUNCTIONS TO PROCESS CLEAN DATA FILES PRIOR TO DATA ANALYSIS

# import required packages
import numpy as np
import pandas as pd
import glob
import matplotlib.pyplot as plt
from data_cleaning_functions import haversine
import os


# This function converts the keypoint timestamps in video_keypoints.csv from mm:ss to milliseconds
def convert_keypoints(data):
    new_data = data

    minutes = [i.split(':')[0] if i is not np.nan else np.nan for i in new_data['launch']]
    minutes2 = pd.to_numeric(minutes, errors='coerce')
    new_data['minutes'] = minutes2
    seconds = [i.split(':')[1] if i is not np.nan else np.nan for i in new_data['launch']]
    seconds2 = pd.to_numeric(seconds, errors='coerce')
    new_data['seconds'] = seconds2

    new_data['launchtime_vid'] = (new_data['minutes'] * 60000) + (new_data['seconds'] * 1000)


    minutes = [i.split(':')[0] if i is not np.nan else np.nan for i in new_data['gimbal_down']]
    minutes2 = pd.to_numeric(minutes, errors='coerce')
    new_data['minutes'] = minutes2
    seconds = [i.split(':')[1] if i is not np.nan else np.nan for i in new_data['gimbal_down']]
    seconds2 = pd.to_numeric(seconds, errors='coerce')
    new_data['seconds'] = seconds2

    new_data['gimbaldown_vid'] = (new_data['minutes'] * 60000) + (new_data['seconds'] * 1000)


    minutes = [i.split(':')[0] if i is not np.nan else np.nan for i in new_data['gohome']]
    minutes2 = pd.to_numeric(minutes, errors='coerce')
    new_data['minutes'] = minutes2
    seconds = [i.split(':')[1] if i is not np.nan else np.nan for i in new_data['gohome']]
    seconds2 = pd.to_numeric(seconds, errors='coerce')
    new_data['seconds'] = seconds2

    new_data['gohome_vid'] = (new_data['minutes'] * 60000) + (new_data['seconds'] * 1000)


    minutes = [i.split(':')[0] if i is not np.nan else np.nan for i in new_data['landed']]
    minutes2 = pd.to_numeric(minutes, errors='coerce')
    new_data['minutes'] = minutes2
    seconds = [i.split(':')[1] if i is not np.nan else np.nan for i in new_data['landed']]
    seconds2 = pd.to_numeric(seconds, errors='coerce')
    new_data['seconds'] = seconds2

    new_data['landed_vid'] = (new_data['minutes'] * 60000) + (new_data['seconds'] * 1000)

    cols_to_keep = ['flight', 'flight_type', 'launchtime_vid', 'gimbaldown_vid', 'gohome_vid',
    'landed_vid', 'exclude', 'exclude_log', 'note']
    new_data.drop(new_data.columns.difference(cols_to_keep), axis = 1, inplace = True)
    new_data = new_data[cols_to_keep]

    #new_data = new_data.drop(['minutes', 'seconds'], axis = 1)
    return(new_data)


# This function identifies the flight keypoints in the drone log files and extracts the corresponding times
def extract_keypoints(directory, data): # directory is where the metric logfiles are stored
                                    # data is the dataframe containing the video keypoints in milliseconds
    # get list of log files
    files = glob.glob(directory+'*.csv')

    # create empty columns to store the log keypoints in
    data.loc[:,'launchtime_log'] = np.nan
    data.loc[:,'gimbaldown_log'] = np.nan
    data.loc[:,'gohome_log'] = np.nan
    data.loc[:,'landed_log'] = np.nan
    data.loc[:,'landed_log_type'] = np.nan # the landed keypoint is inconsistent in the drone logs. There are several
                                        # ways of extracting it, so this is a column where I denote which method
                                        # was used for each flight.
    # Work through each file and extract log keypoints
    for i in files:
        # read in a log file
        df = pd.read_csv(i)
        # get the name of this flight by slicing up the filename
        new_name = i.split('ob')[1]
        final_name = new_name.split('_')[0]
        # check that the log includes the drone's launch
        if max(df['ascent_m']) > 0:
            # find the time value for when ascent exceeds 0 m for the first time & assign this as the flight's launchtime
            data.loc[data['flight'] == final_name, 'launchtime_log'] = df[df.ascent_m > 0].iloc[0]['time_millisecond']
        # check that the gimbal actually does point directly down at some point
        if min(df['gimbal_pitch_degrees']) <= -90:
            # find the time value for when the gimbal is pointed straight down for the first time after launch & assign this as the flight's gimbaldown time
            data.loc[data['flight'] == final_name, 'gimbaldown_log'] = df[(df['gimbal_pitch_degrees'] == -90) & (df['ascent_m'] > 0)].iloc[0]['time_millisecond']
        # check that GoHome exists in the drone log
        if "Go_Home" in df['flycState'].values:
            # find the time value for when GoHome is logged for the first time & assign this as the flight's gohome time
            data.loc[data['flight'] == final_name, 'gohome_log'] = df[df.flycState == 'Go_Home'].iloc[0]['time_millisecond']
        # check if the drone ever logs an ascent of 0 during the landing sequence
        if ((df['ascent_m'] <= 0) & (df['max_distance_m'] > 100)).any():
            # find the time value for when ascent is 0 but max distance is big
            subset = df[(df['ascent_m'] <= 0) & (df['max_distance_m'] > 100)]
            min_ascent = np.min(subset['ascent_m'])
            data.loc[data['flight'] == final_name, 'landed_log'] = subset[subset['ascent_m'] == min_ascent].iloc[0]['time_millisecond']
            # note which landing time criteria were used
            data.loc[data['flight'] == final_name, 'landed_log_type'] = 'ascent_m_0'
        # if the drone doesn't log ascent_m == 0 during landing, check if the rc_throttle is ever == 364
        elif ((df['rc_throttle'] == 364) & (df['max_distance_m'] > 100) & (df['ascent_m'] < 5)).any():
            # set the landing point as the time when the rc_throttle ==364 with the lowest ascent_m
            subset = df[(df['rc_throttle'] == 364) & (df['max_distance_m'] > 100)]
            min_ascent = np.min(subset['ascent_m'])
            data.loc[data['flight'] == final_name, 'landed_log'] = df[(df['rc_throttle'] == 364) &
                                                                      (df['max_distance_m'] > 100) &
                                                                      (df['ascent_m'] == min_ascent)].iloc[0]['time_millisecond']
            # note which landing time criteria were used
            data.loc[data['flight'] == final_name, 'landed_log_type'] = 'rc_throttle_364'
        # if the drone doesn't log ascent_m==0 and the rc_throttle never reaches 364 during landing, check if
        # the rc_throttle is 1024 during landing
        elif ((df['rc_throttle'] == 1024) & (df['max_distance_m'] > 100) & (df['ascent_m'] < 10)).any():
            # set the landing point as the time when the rc_throttle <=1024 with the lowest ascent_m
            subset = df[(df['rc_throttle'] <= 1024) & (df['max_distance_m'] > 100)]
            min_ascent = np.min(subset['ascent_m'])
            data.loc[data['flight'] == final_name, 'landed_log'] = df[(df['rc_throttle'] <= 1024) &
                                                                      (df['max_distance_m'] > 100) &
                                                                      (df['ascent_m'] == min_ascent)].iloc[0]['time_millisecond']
            # note which landing time criteria were used
            data.loc[data['flight'] == final_name, 'landed_log_type'] = 'rc_throttle_1024'
    return(data)

# This function allows for visual comparison of the time gaps between keypoints in the log and videos to check
# if any video keypoints are misidentified or entered incorrectly.
def compare_keypoints(flight, df):
    # 1. Look up the flight in the keypoint_differences dataframe
    plt.figure(figsize = (12,12))
    plt.scatter(df.loc[flight]['log'], df.loc[flight]['video'])
    plt.plot([0, 1300000], [0, 1300000], color = 'r')
    plt.xlabel('drone_log')
    plt.ylabel('video_scoring')

    print(abs(df.loc[flight]['log'] - df.loc[flight]['video'])/1000)

# This function takes the dataframe that contains the video and log keypoints and calculates a time offset that
# best aligns the drone log and video timestamps. It calculates the time difference between corresponding
# keypoints in the video and drone log and then determines which of these time differences results in the least
# average discrepancy between all keypoint pairs.
def get_offset(df): # df is the dataframe that contains the video and log keypoints
    # get list of flights
    flight_list = df['flight'].unique()
    # create new columns to store chosen offset values, max and mean discrepancies, and the keypoint with the
    # highest discrepancy
    df.loc[:,'offset'] = np.nan
    df.loc[:,'max_discrep'] = np.nan
    df.loc[:,'mean_discrep'] = np.nan
    df.loc[:,'max_discrep_point'] = np.nan
    for i in flight_list:
        # extract subset of data for this flight
        test = df[df['flight'] == i].iloc[0]
        # calculate potential offsets based on each available keypoint
        launch_gap = test['launchtime_vid'] - test['launchtime_log']
        gimbal_gap = test['gimbaldown_vid'] - test['gimbaldown_log']
        home_gap = test['gohome_vid'] - test['gohome_log']
        land_gap = test['landed_vid'] - test['landed_log']

        # create a DataFrame to compare the different offsets
        launch = [(test['launchtime_log'] + launch_gap)- test['launchtime_vid'],
                  (test['launchtime_log'] + gimbal_gap) - test['launchtime_vid'],
                  (test['launchtime_log'] + home_gap) - test['launchtime_vid'],
                  (test['launchtime_log'] + land_gap) - test['launchtime_vid']]
        gimbal = [(test['gimbaldown_log'] + launch_gap)- test['gimbaldown_vid'],
                  (test['gimbaldown_log'] + gimbal_gap) - test['gimbaldown_vid'],
                  (test['gimbaldown_log'] + home_gap) - test['gimbaldown_vid'],
                  (test['gimbaldown_log'] + land_gap) - test['gimbaldown_vid']]
        home = [(test['gohome_log'] + launch_gap)- test['gohome_vid'],
                (test['gohome_log'] + gimbal_gap) - test['gohome_vid'],
                (test['gohome_log'] + home_gap) - test['gohome_vid'],
                (test['gohome_log'] + land_gap) - test['gohome_vid']]
        land = [(test['landed_log'] + launch_gap)- test['landed_vid'],
                (test['landed_log'] + gimbal_gap) - test['landed_vid'],
                (test['landed_log'] + home_gap) - test['landed_vid'],
                (test['landed_log'] + land_gap) - test['landed_vid']]
        testdf = pd.DataFrame(data = [launch, gimbal, home, land],
                              columns = ['launch', 'gimbal', 'home', 'land'])
        # copy the data frame, but with absolute values instead of true values
        testdf_copy = testdf.abs()

        testdf['gap'] = ['launch_gap', 'gimbal_gap', 'home_gap', 'land_gap']
        testdf['gapval'] = [launch_gap, gimbal_gap, home_gap, land_gap]
        testdf = testdf[['gap', 'gapval', 'launch', 'gimbal', 'home', 'land']]
        testdf['abs_total'] = testdf_copy.sum(axis = 1)
        testdf['abs_mean'] = testdf_copy.mean(axis = 1)

        # Choose the gap value resulting in the lowest mean offset for all keypoints. If multiple gap values
        # have the lowest mean, take their mean. Store this value in the main data table
        my_value = np.mean(testdf[testdf['abs_mean'] == np.min(testdf.abs_mean)]['gapval'])
        df.loc[df['flight'] == i, 'offset'] = my_value

        # Create a dictionary with the absolute values of the time discrepancies using the chosen gap value (it
        # is necessary to recalculate these values in case the chosen gap value is a mean of multiple values in
        # the original table). Store the maximum and mean discrepancies, and the type of keypoint that had the
        # max discrepancy, in the main data table.
        discrepancies = {'launch': abs((test['launchtime_log'] + my_value)- test['launchtime_vid']),
                         'gimbal': abs((test['gimbaldown_log'] + my_value)- test['gimbaldown_vid']),
                         'home': abs((test['gohome_log'] + my_value)- test['gohome_vid']),
                         'land': abs((test['landed_log'] + my_value)- test['landed_vid'])}
        discreps = {k: discrepancies[k] for k in discrepancies if not np.isnan(discrepancies[k])}

        if len(discreps) > 0:
            mean_discrep = pd.Series([*discreps.values()]).mean()
            max_discrep = pd.Series([*discreps.values()]).max()
            max_discrep_point = max(discreps, key = discreps.get)
            df.loc[df['flight'] == i, 'max_discrep'] = max_discrep
            df.loc[df['flight'] == i, 'mean_discrep'] = mean_discrep
            df.loc[df['flight'] == i, 'max_discrep_point'] = max_discrep_point
        else:
            df.loc[df['flight'] == i, 'max_discrep'] = np.nan
            df.loc[df['flight'] == i, 'mean_discrep'] = np.nan
            df.loc[df['flight'] == i, 'max_discrep_point'] = np.nan
    return(df)

# This function gets the lat/lon/altitude of the launch point and the lat/lon/ascent_m of the drone at the start
# of each clip
def get_locations(vid_kp, align_df, log_directory):
    # get list of log files
    log_files = glob.glob(log_directory + '*.csv')

    # get clip start times from vid_kp
    #clip_df = pd.melt(vid_kp, id_vars = ['flight'], value_vars = ['first_clip_start', 'second_clip_start'],
                     #var_name = 'clip_type', value_name = 'clip_start_vid')
    clip_df = pd.melt(vid_kp, id_vars = ['flight'], value_vars = ['first_clip_start'],
                     var_name = 'clip_type', value_name = 'clip_start_vid')
    clip_df['clip_type'] = [i.split('_')[0] for i in clip_df['clip_type']]

    # convert clip start times from mm:ss to milliseconds
    clip_df['minutes'] = [i.split(':')[0] if i is not np.nan else np.nan for i in clip_df['clip_start_vid']]
    clip_df['minutes'] = pd.to_numeric(clip_df['minutes'], errors = 'coerce')
    clip_df['seconds'] = [i.split(':')[1] if i is not np.nan else np.nan for i in clip_df['clip_start_vid']]
    clip_df['seconds'] = pd.to_numeric(clip_df['seconds'], errors = 'coerce')
    clip_df['clip_start_vid'] = (clip_df['minutes'] * 60000) + (clip_df['seconds'] * 1000)
    clip_df.drop(['minutes', 'seconds'], axis = 1, inplace = True)

    # get the offset value for each flight and subtract it from the clip start time scored from the video to
    # get the clip start time in the drone log
    clip_df['offset'] = [align_df.loc[align_df['flight'] == i,'offset'].values[0] for i in clip_df['flight']]
    clip_df['clip_start_log'] = clip_df['clip_start_vid'] - clip_df['offset']

    # now go through each flight log and get the GPS coordinates of the drone at launch and clip start
    # create new columns in clip_df to store new values
    clip_df.loc[:, 'launch_lat'] = np.nan
    clip_df.loc[:, 'launch_lon'] = np.nan
    clip_df.loc[:, 'launch_alt'] = np.nan
    clip_df.loc[:, 'clip_start_lat'] = np.nan
    clip_df.loc[:, 'clip_start_lon'] = np.nan
    clip_df.loc[:, 'clip_start_ascent'] = np.nan
    for i in log_files:
        # read in the log
        log = pd.read_csv(i)

        # get the flight name
        name = i.split('ob')[1]
        name = name.split('_')[0]

        # Get the mean launch lat, lon & alt from the point in the log where the drone hasn't moved laterally
        # yet and coordinates are > 0 (sometimes before drone has satellite connection it assumes it is at
        # 0,0). Note that previously, instead of 'max_distance_m'== 0, I used 'max_ascent_m'==0, but there
        # were 3 flights where the drone didn't achieve satellite lock (and therefore did not establish its
        # position before taking off, so these 3 flights had null launch locations.)
        launch_lat = log[(log['max_distance_m'] == 0) & (log['latitude'] > 0)]['latitude'].mean()
        launch_lon = log[(log['max_distance_m'] == 0) & (log['longitude'] > 0)]['longitude'].mean()
        launch_alt = log[(log['max_distance_m'] == 0) & (log['latitude'] > 0)]['altitude_m'].mean()
        # store these values in clip_df
        clip_df.loc[clip_df['flight'] == name, 'launch_lat'] = launch_lat
        clip_df.loc[clip_df['flight'] == name, 'launch_lon'] = launch_lon
        clip_df.loc[clip_df['flight'] == name, 'launch_alt'] = launch_alt

        # Now get the lat, lon and alt from the point where the clip starts
        clip1_start = clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'first'), 'clip_start_log'].values[0]
        #clip2_start = clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'second'), 'clip_start_log'].values[0]
        # Check if the exact clip-start time is in the log time column; if so, take the lat and lon from that line
        if sum(log['time_millisecond'].isin([clip1_start])) > 0:
            clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'first'), 'clip_start_lat'] = log.loc[log['time_millisecond'] == clip1_start, 'latitude'].values[0]
            clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'first'), 'clip_start_lon'] = log.loc[log['time_millisecond'] == clip1_start, 'longitude'].values[0]
            clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'first'), 'clip_start_ascent'] = log.loc[log['time_millisecond'] == clip1_start, 'ascent_m'].values[0]
        # since some of the offsets are averaged, some start times are not in the drone log exactly. So take either the time just before or after the start point, whichever one is closer
        else:
            before = log.loc[log['time_millisecond'] < clip1_start, 'time_millisecond'].max()
            after = log.loc[log['time_millisecond'] > clip1_start, 'time_millisecond'].min()
            before_diff = abs(before - clip1_start)
            after_diff = abs(after - clip1_start)
            if (pd.notnull(after_diff) & (before_diff <= 1000)):  # this removes cases where the drone log cuts out early
                if before_diff <= after_diff: # if the time before the start time is closer to the start time, take the before time
                    clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'first'), 'clip_start_lat'] = log.loc[log['time_millisecond'] == before, 'latitude'].values[0]
                    clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'first'), 'clip_start_lon'] = log.loc[log['time_millisecond'] == before, 'longitude'].values[0]
                    clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'first'), 'clip_start_ascent'] = log.loc[log['time_millisecond'] == before, 'ascent_m'].values[0]
                else: # otherwise take the after time
                    clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'first'), 'clip_start_lat'] = log.loc[log['time_millisecond'] == after, 'latitude'].values[0]
                    clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'first'), 'clip_start_lon'] = log.loc[log['time_millisecond'] == after, 'longitude'].values[0]
                    clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'first'), 'clip_start_ascent'] = log.loc[log['time_millisecond'] == after, 'ascent_m'].values[0]
        # now repeat for second clip
        #if sum(log['time_millisecond'].isin([clip2_start])) > 0:
        #    clip_df.loc[(clip_df['flight'] == name) & ( clip_df['clip_type'] == 'second'), 'clip_start_lat'] = log.loc[log['time_millisecond'] == clip2_start, 'latitude'].values[0]
        #    clip_df.loc[(clip_df['flight'] == name) & ( clip_df['clip_type'] == 'second'), 'clip_start_lon'] = log.loc[log['time_millisecond'] == clip2_start, 'longitude'].values[0]
        #    clip_df.loc[(clip_df['flight'] == name) & ( clip_df['clip_type'] == 'second'), 'clip_start_ascent'] = log.loc[log['time_millisecond'] == clip2_start, 'ascent_m'].values[0]
        #else:
        #    before = log.loc[log['time_millisecond'] < clip2_start, 'time_millisecond'].max()
        #    after = log.loc[log['time_millisecond'] > clip2_start, 'time_millisecond'].min()
        #    before_diff = abs(before - clip2_start)
        #    after_diff = abs(after - clip2_start)
        #    if (pd.notnull(after_diff) & (before_diff <= 1000)):  # this removes cases where the drone log cuts out early
        #        if before_diff <= after_diff: # if the time before the start time is closer to the start time, take the before time
        #            clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'second'), 'clip_start_lat'] = log.loc[log['time_millisecond'] == before, 'latitude'].values[0]
        #            clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'second'), 'clip_start_lon'] = log.loc[log['time_millisecond'] == before, 'longitude'].values[0]
        #            clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'second'), 'clip_start_ascent'] = log.loc[log['time_millisecond'] == before, 'ascent_m'].values[0]
        #        else: # otherwise take the after time
        #            clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'second'), 'clip_start_lat'] = log.loc[log['time_millisecond'] == after, 'latitude'].values[0]
        #            clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'second'), 'clip_start_lon'] = log.loc[log['time_millisecond'] == after, 'longitude'].values[0]
        #            clip_df.loc[(clip_df['flight'] == name) & (clip_df['clip_type'] == 'second'), 'clip_start_ascent'] = log.loc[log['time_millisecond'] == after, 'ascent_m'].values[0]
    clip_df.drop(['offset'], axis = 1, inplace = True)
    return clip_df

# This function defines flight periods within drone logs: launch, approach, proximity (clip 1 + time during
# approach when drone is within 50m of group), clip 1 and clip 2. It saves the start and end times of those
# states (from the log, in milliseconds) in the flight_df database
def flight_stages(old_log_directory, flight_df, clip_df):
    files = glob.glob(old_log_directory +'*.csv')

    flight_df['clip1_start'] = np.nan
    flight_df['clip1_end'] = np.nan
    #flight_df['clip2_start'] = np.nan
    #flight_df['clip2_end'] = np.nan
    flight_df['ascent_start'] = np.nan
    flight_df['ascent_end'] = np.nan
    flight_df['approach_start'] = np.nan
    flight_df['approach_end'] = np.nan
    flight_df['proximity_start'] = np.nan
    flight_df['proximity_end'] = np.nan
    for i in files:
        log = pd.read_csv(i)
        flight_name = i.split('_')[1].split('b')[1]

        if flight_name in clip_df['flight'].values:

            # start with clip1 - this starts when the first clip starts and goes for 2 minutes (120000 milliseconds)
            clip1_start = clip_df[(clip_df['flight'] == flight_name) & (clip_df['clip_type'] == 'first')]['clip_start_log'].values[0]
            flight_df.loc[flight_df['flight'] == flight_name, 'clip1_start'] = clip1_start
            flight_df.loc[flight_df['flight'] == flight_name, 'clip1_end'] = clip1_start + 120000

            # then clip2 - this starts when the second clip starts and goes for 2 minutes (120000 milliseconds)
            #if ((clip_df['flight'] == flight_name) & (clip_df['clip_type'] == 'second')).any():
            #    clip2_start = clip_df[(clip_df['flight'] == flight_name) & (clip_df['clip_type'] == 'second')]['clip_start_log'].values[0]
            #    flight_df.loc[flight_df['flight'] == flight_name, 'clip2_start'] = clip2_start
            #    flight_df.loc[flight_df['flight'] == flight_name, 'clip2_end'] = clip2_start + 120000

            # ascent (this starts when ascent_m is over 0 and ends when the drone stops ascending (over 30m) and starts moving away from the launchpoint)
            log['ascent_shift'] = (log['ascent_m'].shift(-1) - log['ascent_m'])
            log['distance_shift'] = (log['distance_m'].shift(-1) - log['distance_m'])
            ascent_end = log[(log['ascent_shift'] < 0.2) & (log['distance_shift'] > 0.2) & (log['max_ascent_m'] > 30)].min()['time_millisecond']
            ascent_start = log[(log['time_millisecond'] < ascent_end) & (log['max_ascent_m'] > 0)]['time_millisecond'].min()
            flight_df.loc[flight_df['flight'] == flight_name, 'ascent_start'] = ascent_start
            flight_df.loc[flight_df['flight'] == flight_name, 'ascent_end'] = ascent_end

            # approach (the time between the launch and clip1 start)
            flight_df.loc[flight_df['flight'] == flight_name, 'approach_start'] = ascent_end
            flight_df.loc[flight_df['flight'] == flight_name, 'approach_end'] = clip1_start

            # proximity (clip 1, plus any time during the approach when the drone was within 50m of the group location (clip start location))
            clip_start_lat = clip_df[(clip_df['flight'] == flight_name) & (clip_df['clip_type'] == 'first')]['clip_start_lat'].values[0]
            clip_start_lon = clip_df[(clip_df['flight'] == flight_name) & (clip_df['clip_type'] == 'first')]['clip_start_lon'].values[0]
            log['dist_from_group'] = [haversine(clip_start_lat, clip_start_lon, c,d) * 1000 for c,d in zip(log['latitude'], log['longitude'])]
            prox_start = log[((log['time_millisecond'] > ascent_end) & (log['dist_from_group'] < 50))]['time_millisecond'].min()
            flight_df.loc[flight_df['flight'] == flight_name, 'proximity_start'] = prox_start
            flight_df.loc[flight_df['flight'] == flight_name, 'proximity_end'] = clip1_start + 120000

            log.drop(['ascent_shift', 'distance_shift', 'dist_from_group'], axis = 1, inplace = True)

    return flight_df
