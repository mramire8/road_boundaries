__author__ = 'maru'

import os
import numpy as np


def load_data(datafile, separator=","):
    # latitude, longitude, elevation, intensity
    points = np.loadtxt(datafile, delimiter=separator, usecols=(2, 3, 4, 5), unpack=True)

    return points


def get_lidar_filenames(input_dir):
    names = []
    for path, subdirs, files in os.walk(input_dir):
        for filename in files:
            if ".txt" in filename and "index" not in filename:
                names.append(os.path.join(path, filename))
    return names


def load_lidar(path, skip=5, maxi=5, file_names=None):
    if file_names is None:
        file_names = get_lidar_filenames(path)
    total = len(file_names)
    print "Files: %s" % total
    data = []
    latitude = []
    longitude = []
    elevation = []
    intensity = []
    # skip = 5
    i = 0

    # for f in file_names[:max]:
    for idx in maxi:
        f = file_names[idx]
        print i, f
        lat, long, elev, inten = load_data(f)
        latitude = np.append(latitude, lat[range(0, lat.shape[0], skip)])
        longitude = np.append(longitude, long[range(0, lat.shape[0], skip)])
        elevation = np.append(elevation, elev[range(0, lat.shape[0], skip)])
        intensity = np.append(intensity, inten[range(0, lat.shape[0], skip)])

        i += 1

    return latitude, longitude, elevation, intensity