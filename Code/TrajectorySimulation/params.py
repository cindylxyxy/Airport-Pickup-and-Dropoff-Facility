import math
from math import ceil
import random

dirname = ''
suffix = ''
simType = 'exp'
debug = False
spmatch = False

side = 'single'
angle = 0
mode = 'long'
control = 'partial'

meanSERV = 60. # or 300 sec
SIM_HOUR = 20
SIM_UNIT = 3600 * SIM_HOUR
SIM_ITER = 30
LOT_TOTAL = 40

if angle == 90:
	# 90-degree configurations
	nUnit = 3
	dgap = 2
	meanPOUT = 4.6
	meanPLIN = 9.7 
	LOT_LENGTH = 10. # 8.25 previously
	LOT_WIDTH = 17.75
	LANE_WIDTH = 23. 

if angle == 0:
	# 0-degree configurations
	nUnit = 1
	dgap = 1
	meanPOUT = 5.6 # 5.6 seconds
	LOT_WIDTH = 10. # 8.25 previously
	LANE_WIDTH = 12.
	if mode == 'long':
		meanPLIN = 4.8 # 4.8 seconds 
		LOT_LENGTH = 32.5
	if mode == 'short':
		meanPLIN = 16.5 # new here 
		LOT_LENGTH = 25.0 # 23.0 previously

if angle == 45:
	# 45-degree configurations
	nUnit = 2 # ?
	dgap = 2 # ?
	meanPOUT = 10.3 # 10.3 seconds
	meanPLIN = 4.2 # 4.2 seconds 
	LOT_LENGTH = 13.0
	LOT_WIDTH = 17.5
	LANE_WIDTH = 12.0 

rateDRIV = 44./3. # 23.0 previously
CAR_LENGTH = LOT_LENGTH * nUnit
meanDRIV = CAR_LENGTH / rateDRIV # time to drive through one boarding spot 
m_out = int( round(meanPOUT / meanDRIV) )
m_in = int( round(meanPLIN / meanDRIV) ) 
# g_out = max(m_out, 3)
# if spmatch:
# meanPOUT = m_out * meanDRIV
# meanPLIN = m_in * meanDRIV
# SAFETY_DIST = g_out * CAR_LENGTH
# only use safety distance when pulling out and only check behind

rateSERV = 1. / meanSERV
ratePOUT = 1. / meanPOUT
ratePLIN = 1. / meanPLIN

if angle == 0:
	filename = '%s_%s_%s_%s_%s_%s_%s_%s_%s'%(angle, mode, side, control, meanSERV, m_out, m_in, simType, suffix)
elif angle == 90:
	filename ='%s_%s_%s_%s_%s_%s_%s_%s'%(angle, side, control, meanSERV, m_out, m_in, simType, suffix)

SEED_SERV = random.Random(2020).sample(range( int(1e12) ), LOT_TOTAL)
SEED_POUT = random.Random(9031).sample(range( int(1e12) ), LOT_TOTAL)
SEED_PLIN = random.Random(4654).sample(range( int(1e12) ), LOT_TOTAL)
SEED_DRIV = random.Random(1203).sample(range( int(1e12) ), LOT_TOTAL)

SMALL_INTERVAL = 1e-7

event_priority = {'leave_system':        1,
				  'finish_pulling_out':  2,
				  'prepare_pulling_out': 3,
				  'start_service':       4,
				  'start_second_enter':  5,
				  'prepare_first_exit':  6,
				  'start_pulling_in':    7,
				  'enter_system':        8
				  }
#############################################################################
#############################################################################

# the speed limit within parking area is 15 miles per hour i.e. 22 ft/sec
# we assume an approximate value of 23 ft/sec 
# so that for 0-degree (short/normal) configurations it takes 1 sec to drive through the length of one boarding spot

# the data source is the handbook unless noted otherwise
# in general it assumes the design vehicle has a width of 6'7'' and a length of 17'1''

# the data for 0-degree configurations is obtained from the parallel parking stall design
# the parallel stall length is 23'0'' 
# the width projection with minimum level of comfort is 8'3'' 
# with an aisle width of 12' for one-way traffic; or with an aisle width of 22'4'' for two-way traffic
# the width projection with generous level of comfort is 9'0'' 
# with an aisle width of 15'0'' for one-way traffic and 25'4'' for two-way traffic
# our model does not model two-way traffic within the pickup and dropoff facility

# now if we increase the length of each boarding spot,
# it is likely that time for enter and exit maneuvers are likely to decrease 
# thus we also define the parameters for 0-degree (long) configurations
# the data source is from Peter's writeup
# the long parallel stall length is 32'6''

# the data for 90-degree configurations is obtained from the 90-degree parking stall design
# the 90-degree vehicle projection is 17'9''
# and the width projection with minimum level of comfort is 8'3'' with an aisle width of 23' for one-way and two-way traffic
# and the width projection with generous level of comfort is 9'0'' with an aisle width of 26'0'' for one-way and two-way traffic