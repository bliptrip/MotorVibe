#!/usr/bin/env python3
#
# Objective: read in accelerometer readings from ulg file, strip leading and trailing data, and calculate average, stdev, max, min

from argparse import ArgumentParser
import numpy as np
import os
import pandas as pd
from pathlib import Path
import re
import sys

re_pwm = re.compile(r'.*pwm_([0-9]+)_.*') #For determining the pwm value from the file path
#re_props = re.compile(r'.*\.props\..*') #For matching whether we are looking at motors with or without props
re_accelerometer = re.compile(r'vibe_([xyzXYZ])')

def parse_args():
    parser = ArgumentParser(description='Filter out accelerometer data between a pair of motors from ulg files, writing comparative output to csv file.')
    parser.add_argument('-o', '--output', dest='output', required=True, help='CSV filename to dump statistics to.')
    parser.add_argument('-m', '--map', dest='map',
            help="Dictionary containing a map of first motor's pwm to another equivalent pwm (to compare approximately the same RPM value)",
            default="{1300000:1163158,1400000:1252632,1500000:1342105,1600000:1431579,1700000:1521053,1800000:1610526,1900000:1700000}")
    parser.add_argument('--motor1', dest='motor1', help='Name of motor 1', default='sunnysky')
    parser.add_argument('--motor2', dest='motor2', help='Name of motor 2.', default='tmotor')
    parser.add_argument('-r', '--regexp', dest='regexp', help='Regular expression to use for matching correct paths to search for PX4 ulog files.', default=r'^((tmotor)|(sunnysky)).+')
    parser.add_argument('-i', '--ignore', dest='ignore', action='store_true',
                        help='Ignore string parsing exceptions', default=True)
    parsed = parser.parse_args()
    return(parsed)

if __name__ == "__main__":
    parsed  = parse_args()
    re_comp = re.compile(parsed.regexp)
    coldict = eval("{{'axis': None, 'stat': None, 'props': None, 'RPM_class': 'uint32', '{0}_pwm': 'uint32', '{1}_pwm': 'uint32', '{0}_acceleration': 'float64', '{1}_acceleration': 'float64'}}".format(parsed.motor1, parsed.motor2))
    vibration_columns = ["timestamp", "vibe_x", "vibe_y", "vibe_z", "clip_x", "clip_y", "clip_z"]
    pwm_map   = eval(parsed.map)
    m1_pwm_map = list(pwm_map.keys())
    m2_pwm_map = [pwm_map[k] for k in m1_pwm_map]
    #Create an uninitialized 
    prop_classes = ['n', 'y']
    rpm_classes = range(0, len(pwm_map.keys()))
    axes = ["x", "y", "z"]
    stat_classes = ["mean", "stdev", "max", "min"]
    num_indices = len(prop_classes) * len(rpm_classes) * len(axes) * len(stat_classes)
    props_list = [""] * num_indices
    rpm_class_list = [0] * num_indices
    axis_list = [""] * num_indices
    stat_list = [""] * num_indices
    m1_pwm_list = [0] * num_indices
    m2_pwm_list = [0] * num_indices
    m1_acceleration_list = [0.0] * num_indices
    m2_acceleration_list = [0.0] * num_indices
    index = 0
    for p in range(0, len(prop_classes)):
        for r in range(0, len(rpm_classes)):
            for a in range(0, len(axes)):
                for s in range(0, len(stat_classes)):
                    props_list[index] = prop_classes[p]
                    rpm_class_list[index] = r
                    axis_list[index] = axes[a]
                    stat_list[index] = stat_classes[s]
                    index = index + 1
    pdf     = pd.DataFrame(data=np.array([axis_list, stat_list, props_list, rpm_class_list, m1_pwm_list, m2_pwm_list, m1_acceleration_list, m2_acceleration_list]).transpose(), columns=coldict.keys()) #Generate a shell of a pandas dataframe for storing output statistics
    for (c,v) in coldict.items():
        if v is not None:
            pdf[c] = pdf[c].astype(v)
    vibration_logfiles = list(filter(lambda s: re_comp.match(s), map(lambda s: str(s), Path('./').glob('**/vibe.mavlink.csv'))))
    directories = map(lambda f: os.path.dirname(f), vibration_logfiles)
    basenames = map(lambda f: os.path.basename(f), vibration_logfiles)
    for (fp, d, f) in zip(vibration_logfiles, directories, basenames):
        #Extract pwm, motor name, whether this is a log with props attached or not, and derive RPM class.
        pwm_match = re_pwm.match(fp)
        pwm = int(pwm_match.group(1))
        props = 'y' if re.search('props',fp) else 'n'
        motor_name = parsed.motor1 if re.search(parsed.motor1, fp) else parsed.motor2
        try:
            if motor_name == parsed.motor1:
                rpm_class = m1_pwm_map.index(pwm)
            else:
                rpm_class = m2_pwm_map.index(pwm)
        except ValueError:
            continue
        raw_df = pd.read_csv(fp, header=None, names=vibration_columns)
        for k in ["vibe_x", "vibe_y", "vibe_z"]:
            axis = re_accelerometer.match(k).group(1)
            accel_fieldname = "{}_acceleration".format(motor_name)
            #Calculate stats
            raw_df[k] = np.absolute(raw_df[k])
            dmean = np.mean(raw_df[k]) #Don't d-mean me!
            pdf.loc[ (pdf['axis'] == axis) & (pdf['stat'] == 'mean') & (pdf['props'] == props) & (pdf['RPM_class'] == rpm_class), accel_fieldname ] = dmean
            pdf.loc[ (pdf['axis'] == axis) & (pdf['stat'] == 'mean') & (pdf['props'] == props) & (pdf['RPM_class'] == rpm_class), "{}_pwm".format(motor_name) ] = pwm
            dstdev = np.std(raw_df[k])
            pdf.loc[ (pdf['axis'] == axis) & (pdf['stat'] == 'stdev') & (pdf['props'] == props) & (pdf['RPM_class'] == rpm_class), accel_fieldname ] = dstdev
            pdf.loc[ (pdf['axis'] == axis) & (pdf['stat'] == 'stdev') & (pdf['props'] == props) & (pdf['RPM_class'] == rpm_class), "{}_pwm".format(motor_name) ] = pwm
            dmax = np.max(raw_df[k])
            pdf.loc[ (pdf['axis'] == axis) & (pdf['stat'] == 'max') & (pdf['props'] == props) & (pdf['RPM_class'] == rpm_class), accel_fieldname ] = dmax
            pdf.loc[ (pdf['axis'] == axis) & (pdf['stat'] == 'max') & (pdf['props'] == props) & (pdf['RPM_class'] == rpm_class), "{}_pwm".format(motor_name) ] = pwm
            dmin = np.min(raw_df[k])
            pdf.loc[ (pdf['axis'] == axis) & (pdf['stat'] == 'min') & (pdf['props'] == props) & (pdf['RPM_class'] == rpm_class), accel_fieldname ] = dmin
            pdf.loc[ (pdf['axis'] == axis) & (pdf['stat'] == 'min') & (pdf['props'] == props) & (pdf['RPM_class'] == rpm_class), "{}_pwm".format(motor_name) ] = pwm
    pdf.to_csv(parsed.output)
