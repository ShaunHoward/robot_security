__author__ = 'shaun'

import math
import numpy as np
import csv2dict as cd
MEAN = "mean"
MEDIAN = "median"
STD_DEV = "std_dev"
VAR = "variance"
STD_ERR = "std_err"
MSE = "mean_sqd_err"


def topics_by_pattern(odom_topics, patterns):
    pat_bins = {x: [] for x in patterns}
    for pat in patterns:
        for topic in odom_topics.keys():
            if pat in topic:
                pat_bins[pat].append(topic)
    return pat_bins


def rmse(predictions, targets):
    return math.sqrt((np.mean((predictions - targets) ** 2)))


def calc_window_size(curr_arr_len, desired_arr_len):
    return curr_arr_len / desired_arr_len


def get_stats(vals, targets):
    # do project 1 stats on value list
    stats_dict = dict()
    if vals is not None:
        if targets is not None:
            len_vals = len(vals)
            len_targs = len(targets)
            # make lengths equal
            if len_vals > len_targs:
                window = calc_window_size(len_vals, len_targs)
                vals = moving_window_avg(vals, window)
            else:
                window = calc_window_size(len_targs, len_vals)
                targets = moving_window_avg(targets, window)
            stats_dict[MSE] = rmse(vals, targets)
        else:
            print "no targets were provided"
        stats_dict[MEAN] = np.mean(vals)
        stats_dict[MEDIAN] = np.median(vals)
        stats_dict[STD_DEV] = np.std(vals)
        stats_dict[VAR] = np.var(vals)
        stats_dict[STD_ERR] = np.var(vals) / math.sqrt(len(vals))
    else:
        print "no values were provided"
    return stats_dict


def moving_window_avg(vals, target_window_size):
    new_vals = vals[:len(vals)/target_window_size*target_window_size].reshape(-1, target_window_size).mean(1)
    return new_vals


def get_stats_dict(target_topic="t2_odom"):
    odom_topics = cd.convert_csvs_to_dict()
    my_dict = topics_by_pattern(odom_topics, cd.sample_prefixes)
    stats_dict = dict((k, dict()) for k in my_dict.keys())
    for k in my_dict.keys():
        topic_list = my_dict[k]
        targ_vec = None
        for topic in topic_list:
            if target_topic in topic:
                targ_vec = np.array(odom_topics[topic])
        for topic in topic_list:
            if target_topic in topic:
                continue
            stats_dict[k][topic] = get_stats(np.array(odom_topics[topic]), targ_vec)
    return stats_dict

if __name__ == "__main__":
    get_stats_dict()
