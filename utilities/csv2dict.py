__author__ = 'shaun'
import csv
import os

cwd = os.getcwd()
filter_csv_path = os.path.join(cwd, 'filter_csvs')

sample_prefixes = ['ekf_.2speed', 'ekf_.6speed', 'ukf_.2speed', 'ukf_.4speed', 'ukf_.6or8speed']


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


def bagcsv2dict(fp, fn, dict, col_name="x", col_type=float, ext=".csv"):
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
    reader = csv.reader(open(fp, "rb"))
    first = True
    x_key = 0
    vals = []
    for rows in reader:
        if first:
            first = False
            inds = [i for i in range(len(rows)) if col_name in rows[i]]
            x_key = inds[0]
            continue
        # don't forget to convert strings to the specified type!
        vals.append(col_type(rows[x_key]))
    fn = fn.rstrip(ext)
    dict[fn] = vals
    return dict


def convert_csvs_to_dict():
    (fps, fns) = get_file_paths_and_names(filter_csv_path)
    my_dict = dict()
    for i in range(len(fps)):
        my_dict = bagcsv2dict(fps[i], fns[i], my_dict)
    return my_dict


if __name__ == "__main__":
    convert_csvs_to_dict()

