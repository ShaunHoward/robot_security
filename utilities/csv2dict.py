__author__ = 'shaun'
import csv
import os
import math


def get_file_paths_and_names(directory):
    """
    This function will generate the file names in a directory
    tree by walking the tree either top-down or bottom-up. For each
    directory in the tree rooted at directory top (including top itself),
    it yields a 3-tuple (dirpath, dirnames, filenames).
    """
    file_paths = []  # List which will store all of the full filepaths.
    file_names = []
    # Walk the tree.
    for root, directories, files in os.walk(directory):
        for filename in files:
            # Join the two strings in order to form the full filepath.
            filepath = os.path.join(root, filename)
            file_names.append('_'.join([root.split("/")[-1], filename]))
            file_paths.append(filepath)  # Add it to the list.

    return file_paths, file_names  # Self-explanatory.


def bagcsv2dict(fp, fn, topic_dict, col_names=["x", "covariance"], ext=".csv", chop_size=200):
    """
    This will export the values of the first column with the given
    name to a list and put it in the provided dict an return it.
    fp: the file path
    fn: the file name
    ext: the file extension
    col_name: the name of column to keep
    col_type: the type to cast the column values into
    dict: the existing dict to store col list in with fn - ext as key
    :return: the dict with the list added
    """
    key = 0
    my_dict = dict()
    for col_name in col_names:
        vals = []
        first = True
        reader = csv.reader(open(fp, "rb"))
        i_ = 1
        for rows in reader:
            #if i < chop_size:
            if first:
                first = False
                inds = [i for i in range(len(rows)) if col_name in rows[i]]
                if col_name == "covariance":
                    col_name = "std_dev_from_covar"
                key = inds[0]
                continue
            # don't forget to convert strings to the specified type!
            if col_name == "x":
                vals.append(float(rows[key]))
            elif col_name == "std_dev_from_covar":
                vals.append(math.sqrt(float(rows[key].split(",")[0].lstrip('['))) / math.sqrt(i_))
            i_ += 1
            #else:
            #    break
        my_dict[col_name] = vals
    fn = fn.rstrip(ext)
    topic_dict[fn] = my_dict
    return topic_dict


def convert_csvs_to_dict(csvs_path):
    (fps, fns) = get_file_paths_and_names(csvs_path)
    my_dict = dict()
    for i in range(len(fps)):
        my_dict = bagcsv2dict(fps[i], fns[i], my_dict)
    return my_dict


