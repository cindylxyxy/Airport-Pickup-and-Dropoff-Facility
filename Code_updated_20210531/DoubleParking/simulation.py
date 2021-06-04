########################################################################################################################################################
# Last updated: May 31, 2021 by Xinyu Liu
# This file initiates and runs the trajectory-based simulation for the specified configuration and the specified access control. 
# Configuration -- Double Parking Configuration; 
# Access Control -- Partial 
# Input Distributions -- Exponential service times, and deterministic movement times 

########################################################################################################################################################import sys
import csv
import pickle
import numpy as np
import pandas as pd

from params import *
assert side == 'single'
assert angle == 0
assert mode == 'long'
assert control == 'partial'

from utils import *
if version == 'v1':
	from eventDP1 import *
if version == 'v2':
	from eventDP2 import *
else:
	print ('Wrong version specificed!')

np.savetxt(dirname + 'started_dp_%s.csv'%filename, np.array([]), delimiter=",")
print ('printing params for double parking configuration...')
print ('version:', version)
print ('access control:', control)
print ('type of simulation:', simType)
print ('loading spot configuration:', 'Lot length is %s, lot width is %s, and lane width is %s'%(LOT_LENGTH, LOT_WIDTH, LANE_WIDTH) )
print ('number of hours in simulation:', SIM_HOUR)
print ('number of simulation iterations:', SIM_ITER) 
print ('input to distributions:', 'meanSERV is %s, speed is %s, meanPOUT is %s, and meanPLIN is %s'%(meanSERV, rateDRIV, meanPOUT, meanPLIN) ) 
sys.stdout.flush()


if (not debug):
	outfile = open(dirname + 'res_dp_%s.csv'%filename, 'w')
	writer = csv.writer(outfile, delimiter = ',')
	writer.writerow( ['half_N', 'N', 'max_idle_time', 'max_idle_veh', 'max_idle_pct', 'max_idle_pct_veh', 'avg_prod', 'avg_idle', 'avg_pct_idle'] + list(range(1, 1 + SIM_ITER)) )

for N in range(1, 1 + LOT_TOTAL):

	print (N)
	sys.stdout.flush()

	test = system(N, SEED_SERV[N-1], SEED_POUT[N-1], SEED_PLIN[N-1], SEED_DRIV[N-1])
	
	if version == 'v1':
	
		for j in range(1, test.N + 1):
			test.inCount += 1
			car = vehicle(test, getin = True, stop = j, lane = 0)
			test.inservice[0][j-1] = car
			test.add_event( event(car.serv_time, car, 'prepare_first_exit') )

		for j in range(1, test.N + 1):
			test.inCount += 1
			car = vehicle(test, getin = True, stop = j, lane = 1)
			test.inservice[1][j-1] = car
			test.add_event( event(car.serv_time, car, 'prepare_pulling_out') )

	elif version == 'v2':

		for j in range(1, test.N + 1):
			test.inCount += 1
			car = vehicle(test, getin = True, stop_idx = test.N + j)
			test.inservice[0][j-1] = car
			test.add_event( event(car.serv_time, car, 'prepare_first_exit') )

		for j in range(1, test.N + 1):
			test.inCount += 1
			car = vehicle(test, getin = True, stop_idx = j)
			test.inservice[1][j-1] = car
			test.add_event( event(car.serv_time, car, 'prepare_pulling_out') )

	assert car == test.inservice[1][test.N - 1]
	test.subhead = car
	for j in range(test.N - 1, 0, -1):
		car.subnex = test.inservice[1][j - 1]
		car.subnex.subprev = car
		car = car.subnex

	count = []
	for i in range(SIM_ITER):
		count.append( test.run() )

	total_out = count[-1]
	total_in = total_out
	count = [count[0]] + [count[i+1] - count[i] for i in range(SIM_ITER-1)]

	car = test.head
	while car is not None:
		total_in += 1
		car.update_loc()
		if car.status == 1:
			if not car.prod_time == 0.0:
				import pdb; pdb.set_trace()
			car.prod_time += (car.curr_loc / car.driv)

		elif car.status == 1.5:
			if not car.plin_end >= test.curr:
				import pdb; pdb.set_trace()
			car.prod_time -= (car.plin_end - test.curr)

		elif car.status == 5:
			if not car.pout_end >= test.curr:
				import pdb; pdb.set_trace()
			car.prod_time -= (car.pout_end - test.curr)

		else:
			assert car.status == 6
			if not car.curr_loc >= car.stop * LOT_LENGTH:
				import pdb; pdb.set_trace()
			car.prod_time += ((car.curr_loc - car.stop * LOT_LENGTH) / car.driv)

		test.idle_time_calc(car)
		car = car.nex

	subcar = test.subhead
	while subcar is not None:
		
		if subcar.status in [1, 2, 3, 4.25, 4.5, 6]:
			total_in += 1
			if subcar.status not in [3, 4.5]:
				subcar.update_loc()
		
		if subcar.status == 1:
			if not subcar.prod_time == 0.0:
				import pdb; pdb.set_trace()
			subcar.prod_time += (subcar.curr_loc / subcar.driv)

		elif subcar.status == 2:
			if not subcar.plin_end >= test.curr:
				import pdb; pdb.set_trace()
			subcar.prod_time -= (subcar.plin_end - test.curr)
		
		elif subcar.status == 3:
			if not subcar.serv_end >= test.curr:
				import pdb; pdb.set_trace()
			subcar.prod_time -= (subcar.serv_end - test.curr)

		elif subcar.status == 4.25:
			if not subcar.pout_end >= test.curr:
				import pdb; pdb.set_trace()
			subcar.prod_time -= (subcar.pout_end - test.curr)

		elif subcar.status in [1.5, 4.5, 5]:
			pass

		else:
			assert subcar.status == 6
			if not subcar.curr_loc >= subcar.stop * LOT_LENGTH:
				import pdb; pdb.set_trace()
			subcar.prod_time += ((subcar.curr_loc - subcar.stop * LOT_LENGTH) / subcar.driv)

		test.idle_time_calc(subcar)
		subcar = subcar.subnex

	for stop in range(1, test.N + 1):
		if test.inservice[0][stop - 1] is not None and test.inservice[0][stop - 1].status in [3, 4]:
			car = test.inservice[0][stop - 1]
			total_in += 1
			if car.status == 3:
				if not car.serv_end >= test.curr:
					import pdb; pdb.set_trace()
				car.prod_time -= (car.serv_end - test.curr)
			test.idle_time_calc(car)
				
	total = total_in

	if (not debug):
		writer.writerow([N, 2 * N, 
						test.max_idle, test.max_idle_idx, test.max_pct_idle, test.max_pct_idle_idx,
						np.sum(test.prod_time) / total, 
						np.sum(test.idle_time) / total,
						np.sum(test.pct_idle) / total ] + count )