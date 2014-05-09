#README

## REQUIREMENTS

This code requires: 
	- python 2.7 
	- scipy 
	- skimage
	- OpenCV
	- pyproj

## Directories: 

 <current_directory>

 |__ road_boudary

  |__road.py    main application
	


## TO RUN

To execute the road boundary detector: 

usage: 
'''
speed.py [-h] [--max-iterations MAX_ITERATIONS]
                INPUT_FOLDER
'''

positional arguments:

  INPUT_FOLDER          Folder with input images e.g. street view images


optional arguments:

  -h, --help                 show this help message and exit

  --max-test MAX_ITERATION   maximum number of batches of 20 files to read in this run


## Examples: 
 
 *Search for boundaries in only the first batch:*
 '''
 python road.py /path/to/street/view/images --max-iterations 1
 '''

 *Search for boundaries in first 10 batches (total of 200 files):*
 '''
 python road.py /path/to/street/view/images --max-iterations 10
 '''
 
### OUTPUT EXAMPLES:

Dimensions: (94363,)

Latitude: min:4533075.40174, max:4533159.03042, avg:4533137.23384, stdev:8.96149956075

Longitude: min:1151329.60837, max:1151469.83628, avg:1151401.27195, stdev:11.9720119705

Elevation: min:137.212167906, max:158.20857566, avg:146.493843224, stdev:3.0446982367

Intensity: min:1.0, max:255.0, avg:94.5099138433, stdev:52.7636390447

Grid size: 455.439057238 wide x 507.602812001 tall 


Dimensions 509 x 456

OFFSET: 4533159.03042 1151469.83628

-------------------------------------------------------------------------------

id	x1	y1 z1	x2	y2	z2	

-------------------------------------------------------------------------------

1	4533139.47,	1151408.2483195136,	4533136.98	1151414.35	142.37626611199346,	142.38

2	4533142.97,	1151380.8483195137,	4533138.08	1151392.85	142.37626611199346,	142.38

3	4533142.77,	1151399.4483195136,	4533138.08	1151410.95	142.37626611199346,	142.38

4	4533147.07,	1151389.0483195137,	4533142.98	1151399.05	142.37626611199346,	142.38

5	4533139.37,	1151389.3483195137,	4533132.58	1151406.15	142.37626611199346,	142.38

-------------------------------------------------------------------------------

