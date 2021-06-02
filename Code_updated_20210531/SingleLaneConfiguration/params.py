########################################################################################################################################################
# Last updated: May 31, 2021 by Xinyu Liu
# This file specifies the input data to all other files required for trajectory-based simulation, with:
# Configuration -- Single Lane Configuration 
# Access Control -- Partial 
# Input Distributions -- Exponential service times, and deterministic movement times 

########################################################################################################################################################
import math
from math import ceil
import random

dirname = ''
suffix = ''
meanSERV = 60. # sec
# meanSERV = 120. # sec
# meanSERV = 150. # sec
# meanSERV = 180. # sec
# meanSERV = 300. # sec
# simType = 'cav'
simType = 'exp'
# simType = 'unif'
SIM_HOUR = 20 # hr
SIM_ITER = 20

########################################################################################################################################################
# !!!
# The following section of the file should not be changed unless you know exactly what you are trying to do.
# WARNING: Unintended changes to this section can have unexpected effects to the simulation results.

debug = False
VEHICLE_IDX = None
control = 'partial' 
# control = 'full'
side = 'single'
# side = 'double'
angle = 0
# angle = 90
# mode = 'long'
# mode = 'short'
mode = 'singlelane'

WARMUP_HOUR = 2 # hr
WARMUP_UNIT = 3600 * WARMUP_HOUR # sec
SIM_UNIT = 3600 * SIM_HOUR # sec
if side == 'single':
	LOT_TOTAL = 40
else:
	assert side == 'double'
	LOT_TOTAL = 20

if mode == 'singlelane':
	LOT_TOTAL = 60

########################################################################################################################################################
# !!!
# The following section of the file should not be changed unless you know exactly what you are trying to do.
# WARNING: Unintended changes to this section can have unexpected effects to the simulation results.

if angle == 90:
	# 90-degree configurations
	nUnit = 3
	dgap = 2
	meanPOUT = 4.6 # sec
	meanPLIN = 9.7 # sec
	LOT_LENGTH = 10. # ft
	LOT_WIDTH = 17.75 # ft
	LANE_WIDTH = 23. # ft

if angle == 0:
	# 0-degree configurations
	nUnit = 1
	dgap = 1
	meanPOUT = 5.6 # 5.6 sec
	LOT_WIDTH = 10. # ft
	LANE_WIDTH = 12. # ft
	if mode == 'long':
		meanPLIN = 4.8 # sec
		LOT_LENGTH = 32.5 # ft
	if mode == 'short':
		meanPLIN = 16.5 # sec
		LOT_LENGTH = 25.0 # ft
	if mode == 'singlelane':
		meanPLIN = 1.
		LOT_LENGTH = 23.0 # ft

if angle == 45:
	# 45-degree configurations
	nUnit = 2 # ?
	dgap = 2 # ?
	meanPOUT = 10.3 # sec
	meanPLIN = 4.2 # sec
	LOT_LENGTH = 13.0 # ft
	LOT_WIDTH = 17.5 # ft
	LANE_WIDTH = 12.0 # ft

rateDRIV = 44./3. # ft/sec
CAR_LENGTH = LOT_LENGTH * nUnit # ft
meanDRIV = CAR_LENGTH / rateDRIV # time to drive through one boarding spot in sec
# meanPOUT += meanDRIV
m_out = int( round(meanPOUT / meanDRIV) )
m_in = int( round(meanPLIN / meanDRIV) ) 
g_out = max(m_out, 3)

rateSERV = 1. / meanSERV
ratePOUT = 1. / meanPOUT
ratePLIN = 1. / meanPLIN

filename = '%s_%s_%s_%s_%s_%s_%s'%(angle, mode, side, control, meanSERV, simType, suffix)

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
				  'enter_system':        8,
				  'add_stop_idx':        7.5
				  }
