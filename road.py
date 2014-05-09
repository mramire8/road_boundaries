__author__ = 'maru'

import argparse
import os
import numpy as np
import cv2
import matplotlib
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pyproj
import load_data
from scipy import ndimage
from scipy import misc
from collections import defaultdict
from skimage import morphology

apfk = argparse.ArgumentParser(description=__doc__,
                               prog="road.py",
                               formatter_class=argparse.RawTextHelpFormatter)

apfk.add_argument('input',
                  metavar='INPUT_FOLDER',
                  type=str,
                  default="./input",
                  # default=41.8819,
                  help='Folder with input images')

apfk.add_argument('--max-iterations',
                  metavar='MAX_ITERATION',
                  type=int,
                  default=1,
                  help='Max number of 20 file batches to try')

apfk.add_argument('--showimage',
                  metavar='SHOW_IMAGE',
                  type=bool,
                  default=False,
                  help='Whether to show the matching image or not')

args = apfk.parse_args()
print args
print

#######################################################################################################################
## CONSTANTS
#######################################################################################################################

LAT = 0
LONG = 1
ELE = 2
INT = 3
LIDAR_HOME = '/path/to/your/lidar/data'
#######################################################################################################################


def filter_elevation_data(lidar):
    """
    Remove the cloud points that are higher than a thereshold given by the first quartile of elevation data
    @param lidar: lidar data
    """
    # T = lidar[ELE] < lidar[ELE].min() + ((lidar[ELE].max() -lidar[ELE].min()) /4)
    # T = lidar[ELE] < lidar[ELE].mean()
    T = lidar[ELE] < (np.median(lidar[ELE][lidar[ELE] < np.median(lidar[ELE])]) ) # Elevations in the first quartile of the data
    filtered_int = lidar[INT]
    filtered_long = lidar[LONG]
    filtered_lat = lidar[LAT]
    filtered_elev = lidar[ELE]

    filtered_int = filtered_int[T]
    filtered_long = filtered_long[T]
    filtered_lat = filtered_lat[T]
    filtered_elev = filtered_elev[T]

    return filtered_lat, filtered_long, filtered_elev, filtered_int


def find_intense_points(lidar, th=None):
    """
    remove the cloud points that are of lower intensity based on a threshold
    @param lidar: lidar data with latitude, longitude, elevation and intensity
    @param th: intensity threshold [0,255]
    @return: filtered lidar data
    """
    # th = lidar[INT] > np.median(lidar[INT][lidar[INT] > np.median(lidar[INT])])
    if th is None:
        th = 210

    T = lidar[INT] > th
    filtered_int = lidar[INT]
    filtered_long = lidar[LONG]
    filtered_lat = lidar[LAT]
    filtered_elev = lidar[ELE]

    filtered_int = filtered_int[T]
    filtered_long = filtered_long[T]
    filtered_lat = filtered_lat[T]
    filtered_elev = filtered_elev[T]

    return filtered_lat, filtered_long, filtered_elev, filtered_int


def get_boundary_intense(img, show=False):
    """
    find the lines formed by pixed in a binary raster image of lidar data
    @param img: 2D image
    @param show: if the progress is shown
    @return: list of starting and ending points of lines in img image
    """
    if show:
        plt.gray()

        plt.subplot(131)
        plt.title("Original")
        plt.imshow(img)

    minLineLength = 50
    maxLineGap = 10

    img3 = np.zeros(img.shape)
    non_zero = img > 0
    img3[non_zero] = 255

    x = misc.toimage(img3, cmin=0, cmax=255, mode='L')

    edges = cv2.Canny(np.asarray(x),50,150,apertureSize = 3)

    if show:
        plt.subplot(132)
        plt.title("Edges")
        plt.imshow(edges)

    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=minLineLength, maxLineGap=maxLineGap)
    if lines is None:
        return []

    for x1, y1, x2, y2 in lines[0]:
        cv2.line(img3, (x1, y1), (x2, y2), (255, 0, 0), 2)

    cv2.imwrite('houghlines_result.jpg', img3)

    if show:
        plt.subplot(133)
        plt.title("Identified Lines")
        plt.imshow(img3)
        plt.show()
    return lines


def plot_data(lidar, title=""):
    """
    Plot lidar point cloud data
    """
    fig = plt.figure()

    ax = fig.add_subplot(111, projection='3d')  #, axisbg='black')
    # ax.title(title)
    colors = (lidar[ELE] / lidar[ELE].max()) #* 255
    ax.scatter(lidar[LAT], lidar[LONG], zs=lidar[ELE], zdir='z', marker='+', c=colors, s=1)
    ax.set_xlabel('Latitude')
    ax.set_ylabel('Longitude')
    ax.set_zlabel('Elevation')


    # plt.savefig("demo.png")
    plt.show()


def convert_to_coordinates(lidar):
    """
    Convert GPS degree coordinates to meters
    @param lidar:
    @return: lidar data in meters
    """
    p = pyproj.Proj(proj='utm', zone=33, ellps='WGS84')
    x, y = p(lidar[LAT], lidar[LONG])
    return x, y, lidar[ELE], lidar[INT]



def print_stats(coord):
    """
    Print statistics of lidar data
    @param coord:
    @return:
    """
    print "Latitude: min:{}, max:{}, avg:{}, stdev:{}".format(coord[0].min(), coord[0].max(), coord[0].mean(),coord[0].std())
    print "Longitude: min:{}, max:{}, avg:{}, stdev:{}".format(coord[1].min(), coord[1].max(), coord[1].mean(),coord[1].std())
    print "Elevation: min:{}, max:{}, avg:{}, stdev:{}".format(coord[2].min(), coord[2].max(), coord[2].mean(),coord[2].std())
    print "Intensity: min:{}, max:{}, avg:{}, stdev:{}".format(coord[3].min(), coord[3].max(), coord[3].mean(),coord[3].std())


def create_grid2(cloud):
    """
    Create a raster image from 3d point cloud data by creating a grid
    @param cloud:
    @return:
    """
    minx, maxx = cloud[LAT].min(), cloud[LAT].max()
    miny, maxy = cloud[LONG].min(), cloud[LONG].max()
    minz, maxz = cloud[ELE].min(), cloud[ELE].max()
    im_range = (1+maxx - minx, 1+maxy - miny, 1+maxz-minz)
    cell_size = .10  # .1 meters = 10 cms

    # create blank images
    cells = np.zeros((np.around(im_range[0]/.1)+1, np.around(im_range[1]/.1)+1),dtype=np.float32)
    counts = np.zeros((np.around(im_range[0]/.1)+1, np.around(im_range[1]/.1)+1),dtype=np.float32)
    light = np.zeros((np.around(im_range[0]/.1)+1, np.around(im_range[1]/.1)+1),dtype=np.float32)

    mz = 0
    print ("Grid size: {} wide x {} tall ".format( im_range[0] / .1, im_range[1] / .1,im_range[2] / .1))
    ## loop through the data putting points in cells
    for x, y, z, i in zip(cloud[LAT], cloud[LONG], cloud[ELE], cloud[INT]):
        xc = int(np.round((x - minx) / cell_size))
        yc = int(np.round((y - miny) / cell_size))
        zc = int(np.round((z - minz) / cell_size))
        mz = max(mz, zc)
        # if zc <= ground: ## we are interested on only the ground level
        if i > 80: # and zc < 100:                ## we are interested on only the ground level
            cells[xc,yc] += z
            light[xc,yc] += i
            counts[xc,yc] += 1  ## to compute promedio
    counts[counts == 0] = 1  ## to avoid NaN
    print "maxz: %s " % mz
    return cells/counts, light/counts, minx, miny, minz


def cap(x,y,limits):
    return min(x,limits[0]-1), min(y, limits[1]-1)


def grid_to_map_lines(cells, offset, results):
    """
    Convert line coordinates from 2D image to 3D coordiantes in meters
    @param cells: grid image (rater)
    @param offset: offset when the image was created
    @param results: list of lines found by Hough transform
    @return: return list of coordinates in point cloud units
    """
    if len(results) == 0:
        print "No boundaries were detected."
        return []

    inter_results = []
    for x1, y1, x2, y2 in results[0]:
        xc1, yc1 = (y1 * .10) + offset[0], (x1 * .10) + offset[1]
        zc1 = cells[cap(x1, y1, cells.shape)] + offset[2]
        xc2, yc2 = (y2 * .10) + offset[0], (x2 * .10) + offset[1]
        zc2 = cells[cap(x2, y2, cells.shape)] + offset[2]
        # print [xc1, xc2], [yc1, yc2], [zc1, zc2]
        inter_results.append([[xc1, xc2], [yc1, yc2], [zc1, zc2]])
    return inter_results


def plot_data_lines(lidar, results=None, offset=None, cells=None, intensity=None):
    """
    Plot point cloud and boundaries
    @return: None
    """
    fig = plt.figure()
    plt.set_cmap('rainbow')
    plt.title("Final Product")
    ax = fig.add_subplot(111, projection='3d')  #, axisbg='black')
    # colors = (lidar[ELE] / lidar[ELE].max()) * 255
    colors = lidar[INT] #/255.
    ax.scatter(lidar[LAT], lidar[LONG], zs=lidar[ELE], zdir='z', marker='+', c=colors, s=1)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    m = np.std(lidar[ELE])

    print lidar[LAT].max(), lidar[LONG].max()

    inter_results = grid_to_map_lines(cells, offset, results)
    for line in inter_results:
        xc1, xc2, yc1, yc2, zc1, zc2 = line[0][0],line[0][1],line[1][0],line[1][1],line[2][0],line[2][1]
        ax.plot([xc1, xc2], [yc1, yc2], zs=[zc1+m, zc2+m], zdir='z')
    # plt.savefig("demo.png")

    plt.show()
    return inter_results


def main(batch_i=0):
    """
    Main loop
    @param batch_i: segment i of 20 files to read
    @return:
    """
    file_names = load_data.get_lidar_filenames(LIDAR_HOME)
    n = len(file_names)-1  ##
    batches = n / 50  ## loading every 50 files
    batch_size = 10
    first = n-(batch_i*batch_size)
    last = max(n-((batch_i+1)*batch_size), 0)
    # for b in xrange(batches,0,-)

    lidar = load_data.load_lidar(LIDAR_HOME, skip=50, maxi=range(first,last,-1))
    coord = convert_to_coordinates(lidar)
    coord_0 = coord
    # latitude, longitude, elevation, intensity
    print "Dimensions: {}".format(coord[0].shape)

    print_stats(coord)
    plot_data(coord, title="Converted Coordiantes")
    coord = filter_elevation_data(coord)
    coord = find_intense_points(coord, th=130)
    plot_data(coord, title="Filtered Elevation")
    cells, light, minx, miny, minz = create_grid2(coord)
    print "Dimensions %s x %s" % (len(cells[0]), len(cells))
    results = []
    results = get_boundary_intense((light + cells),show=True)  # find boundaries from the raster images : magic
    lines = plot_data_lines(coord_0, results=results, offset=(minx, miny, minz), cells=cells, intensity=light)

    return lines


def print_results(results):
    """
    Print consolidated results
    """
    print "-"*80
    print "id\tx1\ty1\tz1\tx2\ty2\tz2"
    print "-"*80
    i = 0
    for line in results[0]:
        print "{6}\t{0:.2f}\t{1:.2f}\t{4:.2f}\t{2:.2f}\t{3:.2f}\t{5:.2f}".format(line[0][0],line[0][1],line[1][0],line[1][1],line[2][0],line[2][1],i)
        i +=1
    print "-"*80
    print "Total of boundary segments %s" % len(results)


if __name__ == '__main__':
    if args.input is not None:
        LIDAR_HOME = args.input
    results = []
    for i in range(0,args.max_iterations):
        results.append(main(batch_i=i))

    print_results(results)
