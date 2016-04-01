__author__ = 'shaun'

import os
import math
import numpy as np
import matplotlib.pyplot as plt
import csv
import csv2dict as cd
import itertools
MEAN = 0
MEDIAN = 1
STD_DEV = 2
VAR = 3
STD_ERR = 4
MSE = 5

cwd = os.getcwd()
filter_csv_path = os.path.join(cwd, 'filtered_csvs')

sample_prefixes = ['ekf_.2speed', 'ekf_.4speed', 'ukf_.2speed', 'ukf_.4speed_1', 'ukf_.4speed_2']
target_topic = "t2_odom"
csv_header_out = ["topic_name", "mean", "median", "std_dev", "variance", "std error", "mean-squared error"]


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


def compress_list(vals, des_len):
    len_vals = len(vals)
    # try to make lengths equal
    if len_vals > des_len:
        window = calc_window_size(len_vals, des_len)
        vals = moving_window_avg(vals, window)
        len_vals = len(vals)
    else:
        print "vals list not of des_len size"
    # account for offset by removing last few values to make arrays equal in length
    if len_vals > des_len:
        diff = len_vals - des_len
        vals = vals[:len(vals) - diff]
    return vals


def get_stats_list(vals, targets=None):
    # do project 1 stats on value list
    stats_list = 6 * [0]
    if vals is not None:
        stats_list[MEAN] = np.mean(vals)
        stats_list[MEDIAN] = np.median(vals)
        stats_list[STD_DEV] = np.std(vals)
        stats_list[VAR] = np.var(vals)
        stats_list[STD_ERR] = np.var(vals) / math.sqrt(len(vals))
        stats_list[MSE] = 0
        if targets is not None:
            len_vals = len(vals)
            len_targs = len(targets)
            # try to make lengths equal
            if len_vals > len_targs:
                window = calc_window_size(len_vals, len_targs)
                vals = moving_window_avg(vals, window)
                len_vals = len(vals)
            else:
                window = calc_window_size(len_targs, len_vals)
                targets = moving_window_avg(targets, window)
                len_targs = len(targets)
            # account for offset by removing last few values to make arrays equal in length
            if len_vals > len_targs:
                diff = len_vals - len_targs
                vals = vals[:len(vals) - diff]
            elif len_targs > len_vals:
                diff = len_targs - len_vals
                targets = targets[:len(targets) - diff]
            stats_list[MSE] = rmse(vals, targets)
        else:
            print "no targets were provided"
    else:
        print "no values were provided"
    return stats_list


def moving_window_avg(vals, target_window_size):
    new_vals = vals[:len(vals)/target_window_size*target_window_size].reshape(-1, target_window_size).mean(1)
    return new_vals


def get_stats_dict():
    odom_topics = cd.convert_csvs_to_dict(filter_csv_path)
    my_dict = topics_by_pattern(odom_topics, sample_prefixes)
    stats_dict = dict((k, dict()) for k in my_dict.keys())
    for k in my_dict.keys():
        topic_list = my_dict[k]
        targ_vec = None
        for topic in topic_list:
            if target_topic in topic:
                targ_vec = np.array(odom_topics[topic]['x']).astype(np.float64)
                stats_dict[k][topic] = [get_stats_list(np.array(odom_topics[topic]['x'])), odom_topics[topic]['x'], odom_topics[topic]['std_dev_from_covar']]
        for topic in topic_list:
            if target_topic in topic:
                continue
            stats_dict[k][topic] = [get_stats_list(np.array(odom_topics[topic]['x']), targ_vec), odom_topics[topic]['x'], odom_topics[topic]['std_dev_from_covar']]
    return stats_dict


def export_all_stats_to_csvs(stats_dict, dir=cwd):
    if stats_dict is not None and type(stats_dict) is dict and len(stats_dict.keys()) > 0:
        for exp_pat in stats_dict.keys():
            graph_stats(stats_dict[exp_pat], exp_pat)
            export_stats_to_csv(stats_dict[exp_pat], os.path.join(dir, exp_pat + ".csv"))
    else:
        print "export stats fail: stats dictionary was not provided or is in an incorrect format"


def graph_stats(sub_stats_dict, pat):
    shortest_size = 10000000
    sim_odom_index = 0
    for topic in sub_stats_dict.keys():
        if "t2_odom" in topic:
           sim_odom_index = sub_stats_dict.keys().index(topic)
        x_vals = sub_stats_dict[topic][1]
        len_x = len(x_vals)
        if len_x < shortest_size:
            shortest_size = len_x
    i = 0
    odom_x_vals = compress_list(np.asarray(sub_stats_dict[sub_stats_dict.keys()[sim_odom_index]][1]).astype(np.float64), shortest_size)
    for topic in sub_stats_dict.keys():
        legend = []
        if i != sim_odom_index:
            legend.append(topic)
            # round stats data for nicer printing
            # sub_stats_list = [truncate(stat) for stat in sub_stats_dict[topic][0]]
            x_vals = compress_list(np.asarray(sub_stats_dict[topic][1]).astype(np.float64), shortest_size)
            std_dev_vals = compress_list(np.asarray(sub_stats_dict[topic][2]).astype(np.float64), shortest_size)
            if "dist" in topic:
                plt.errorbar(range(0, len(x_vals)), x_vals, std_dev_vals, fmt='g')
            else:
                plt.errorbar(range(0, len(x_vals)), x_vals, std_dev_vals, fmt='b')
            plt.plot(range(0, len(odom_x_vals)), odom_x_vals, 'r')
            legend.append("simulator_odometry")
            plt.title(topic + " x position vs time")
            plt.xlabel("Time (steps)")
            plt.ylabel("TurtleBot x position (m)")
            plt.legend(legend, loc='lower right')
            plt.savefig('_'.join([topic, 'pos_err_graph.png']), bbox_inches='tight')
            plt.clf()
        i += 1

def export_stats_to_csv(sub_stats_dict, file_path):
    with open(file_path, 'wb') as csvfile:
        w = csv.writer(csvfile, delimiter=',')
        # write column header
        w.writerow(csv_header_out)
        # write topic rows and vals
        for topic in sub_stats_dict.keys():
            # round stats data for nicer printing
            sub_stats_list = [truncate(stat) for stat in sub_stats_dict[topic][0]]
            #x_str = ', '.join([str(x) for x in sub_stats_dict[topic][1]])
            #std_dev_str = ', '.join([str(x) for x in sub_stats_dict[topic][2]])
            row = [topic] + sub_stats_list #list(itertools.chain([topic], sub_stats_list, [x_str], [std_dev_str]))
            w.writerow(row)


def truncate(f, n=8):
    '''Truncates/pads a float f to n decimal places without rounding'''
    s = '{}'.format(f)
    if 'e' in s or 'E' in s:
        return '{0:.{1}f}'.format(f, n)
    i, p, d = s.partition('.')
    return '.'.join([i, (d+'0'*n)[:n]])

if __name__ == "__main__":
    export_all_stats_to_csvs(get_stats_dict())
