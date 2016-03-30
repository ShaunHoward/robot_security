__author__ = 'shaun'

import csv2dict as cd


def topics_by_pattern(odom_topics, patterns):
    pat_bins = {x: [] for x in patterns}
    for pat in patterns:
        for topic in odom_topics.keys():
            if pat in topic:
                pat_bins[pat].append(topic)
    return pat_bins


def do_stats(vals):
    # do project 1 stats on value list
    return dict()


def run_stats():
    odom_topics = cd.convert_csvs_to_dict()
    my_dict = topics_by_pattern(odom_topics, cd.sample_prefixes)
    stats_dict = dict((k, dict()) for k in my_dict.keys())
    for k in my_dict.keys():
        topic_list = my_dict[k]
        for topic in topic_list:
            stats_dict[k][topic] = do_stats(odom_topics[topic])
