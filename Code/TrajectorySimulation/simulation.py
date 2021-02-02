import sys
import csv
import pickle
import numpy as np

from params import *
if not debug:
	import pandas as pd

from event import *
from eventShort import *
from event90 import *
from utils import *

# output = []
# max_idle = []
# max_idle_veh = []
# max_pct_idle = []
# max_pct_idle_veh = []
# effec = []
# idle = []
# pct_idle = []

np.savetxt(dirname + 'started_%s.csv'%filename, np.array([]), delimiter=",")
print ('printing params for %s sided layout ...'%side)
print ('angle:', angle)
if angle == 0:
	print ('mode:', mode)
print ('access control:', control)
print ('type of simulation:', simType)
print ('loading spot configuration:', 'Lot length is %s, lot width is %s, and lane width is %s'%(LOT_LENGTH, LOT_WIDTH, LANE_WIDTH) )
print ('number of hours in simulation:', SIM_HOUR)
print ('number of simulation iterations:', SIM_ITER) 
print ('input to distributions:', 'meanSERV is %s, speed is %s, meanPOUT is %s, and meanPLIN is %s'%(meanSERV, rateDRIV, meanPOUT, meanPLIN) ) 
print ('sample path match?', spmatch)
sys.stdout.flush()

# if side == 'double':
# 	LOT_TOTAL = LOT_HALF
# else:
# 	assert side == 'single'

if (not debug):
	outfile = open(dirname + 'res_%s.csv'%filename, 'w')
	writer = csv.writer(outfile, delimiter = ',')
	if side == 'single':
		writer.writerow( ['N', 'max_idle_time', 'max_idle_veh', 'max_idle_pct', 'max_idle_pct_veh', 'avg_prod', 'avg_idle', 'avg_pct_idle'] + list(range(1, 1 + SIM_ITER)) )
	else:
		writer.writerow( ['half_N', 'N', 'max_idle_time', 'max_idle_veh', 'max_idle_pct', 'max_idle_pct_veh', 'avg_prod', 'avg_idle', 'avg_pct_idle'] + list(range(1, 1 + SIM_ITER)) )

# for N in range(19, 21):
for N in range(25, 1 + LOT_TOTAL):

	print (N)
	sys.stdout.flush()

	if angle == 0 and mode == 'long':
		test = system(N, SEED_SERV[N-1], SEED_POUT[N-1], SEED_PLIN[N-1], SEED_DRIV[N-1])
	elif angle == 0 and mode == 'short':
		test = systemShort(N, SEED_SERV[N-1], SEED_POUT[N-1], SEED_PLIN[N-1], SEED_DRIV[N-1])
	elif angle == 90:
		test = system90(N, SEED_SERV[N-1], SEED_POUT[N-1], SEED_PLIN[N-1], SEED_DRIV[N-1])
	else:
		print ('Configuration not specified!')
		import pdb; pdb.set_trace()

	for j in range(1, test.N + 1):
		test.inCount += 1
		car = vehicle(test, getin = True, stop_idx = j)
		test.inservice[j-1] = car
		# if spmatch:
		# 	test.add_event( event(car.serv_time, car, 'finish_service') )
		# else:
		test.add_event( event(car.serv_time, car, 'prepare_pulling_out') )

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
			if car.curr_loc == 0.0:
				total_in -= 1
				car = car.nex
				continue
			else:
				car.prod_time += (car.curr_loc / car.driv)

		elif car.status == 2:
			if not car.plin_end >= test.curr:
				import pdb; pdb.set_trace()
			car.prod_time -= (car.plin_end - test.curr)

		elif car.status == 5:
			if not car.pout_end >= test.curr:
				import pdb; pdb.set_trace()
			car.prod_time -= (car.pout_end - test.curr)

		else:
			assert car.status == 6
			if spmatch and angle == 90:
				start_x = car.lane_idx * CAR_LENGTH
			elif angle == 90:
				start_x = (car.stop - 1) * LOT_LENGTH + CAR_LENGTH 
			else:
				start_x = car.stop * LOT_LENGTH
			if not car.curr_loc >= start_x:
				import pdb; pdb.set_trace()
			car.prod_time += ((car.curr_loc - start_x) / car.driv)

		test.idle_time_calc(car)
		car = car.nex

	for stop_idx in range(1, test.N + 1):
		if test.inservice[stop_idx - 1] is not None and test.inservice[stop_idx - 1].status in [3, 4]:
			car = test.inservice[stop_idx - 1]
			total_in += 1
			if car.status == 3:
				if not car.serv_end >= test.curr:
					import pdb; pdb.set_trace()
				car.prod_time -= (car.serv_end - test.curr)
			test.idle_time_calc(car)

	# max_idle.append( test.max_idle )
	# max_idle_veh.append( test.max_idle_idx )
	# max_pct_idle.append( test.max_pct_idle )
	# max_pct_idle_veh.append( test.max_pct_idle_idx )

	# effec.append( np.sum(test.prod_time) / count[-1] )
	# idle.append( np.sum(test.idle_time) / count[-1] )
	# pct_idle.append( np.sum(test.pct_idle) / count[-1] )
	# output.append(count)

	total = total_in

	if (not debug):

		if side == 'single':
			writer.writerow([N, 
							 test.max_idle, test.max_idle_idx, test.max_pct_idle, test.max_pct_idle_idx,
							 np.sum(test.prod_time) / total, 
							 np.sum(test.idle_time) / total,
							 np.sum(test.pct_idle) / total ] + count )
		else:
			writer.writerow([N, 2 * N, 
							 test.max_idle, test.max_idle_idx, test.max_pct_idle, test.max_pct_idle_idx,
							 np.sum(test.prod_time) / total, 
							 np.sum(test.idle_time) / total,
							 np.sum(test.pct_idle) / total ] + count )

# for N in range(1, 21):
# # for N in range(21, 1 + LOT_TOTAL):

# 	print (N)
# 	sys.stdout.flush()

# 	test = system(N, SEED_SERV[N-1], SEED_POUT[N-1], SEED_PLIN[N-1], SEED_DRIV[N-1])

# 	for j in range(1, test.N + 1):
# 		test.inCount += 1
# 		car = vehicle(test, getin = True, stop_idx = j)
# 		test.inservice[j-1] = car
# 		if spmatch:
# 			test.add_event( event(car.serv_time, car, 'finish_service') )
# 		else:
# 			test.add_event( event(car.serv_time, car, 'prepare_pulling_out') )

# 	count = []
# 	for i in range(SIM_ITER):
# 		count.append( test.run() )

# 	max_idle.append( test.max_idle )
# 	max_idle_veh.append( test.max_idle_idx )
# 	max_pct_idle.append( test.max_pct_idle )
# 	max_pct_idle_veh.append( test.max_pct_idle_idx )

# 	effec.append( np.sum(test.prod_time) / count[-1] )
# 	idle.append( np.sum(test.idle_time) / count[-1] )
# 	pct_idle.append( np.sum(test.pct_idle) / count[-1] )

# 	count = [count[0]] + [count[i+1] - count[i] for i in range(SIM_ITER-1)]
# 	output.append(count)

# if (not debug) and (not spmatch):

# 	if side == 'single':
# 		df = pd.DataFrame( columns = ['N', 'max_idle_time', 'max_idle_veh', 'max_idle_pct', 'max_idle_pct_veh', 'avg_prod', 'avg_idle', 'avg_pct_idle'] + list(range(1, 1 + SIM_ITER)) )
# 		# df.N = list(range(1, 21))
# 		df.N = list(range(21, 1 + LOT_TOTAL))
# 		df.max_idle_time = max_idle
# 		df.max_idle_veh = max_idle_veh
# 		df.max_idle_pct = max_pct_idle
# 		df.max_idle_pct_veh = max_pct_idle_veh
# 		df.avg_prod = effec
# 		df.avg_idle = idle
# 		df.avg_pct_idle = pct_idle
# 		df[df.columns[8:]] = np.array(output)

# 	else:
# 		df = pd.DataFrame( columns = ['half_N', 'N', 'max_idle_time', 'max_idle_veh', 'max_idle_pct', 'max_idle_pct_veh', 'avg_prod', 'avg_idle', 'avg_pct_idle'] + list(range(1, 1 + SIM_ITER)) )
# 		df.half_N = list(range(1, 21))
# 		# df.half_N = list(range(21, 1 + LOT_TOTAL))
# 		df.N = 2 * df.half_N
# 		df.max_idle_time = max_idle
# 		df.max_idle_veh = max_idle_veh
# 		df.max_idle_pct = max_pct_idle
# 		df.max_idle_pct_veh = max_pct_idle_veh
# 		df.avg_prod = effec
# 		df.avg_idle = idle
# 		df.avg_pct_idle = pct_idle
# 		df[df.columns[9:]] = np.array(output)

# 	df.to_csv(dirname + 'res_%s.csv'%filename, encoding = 'utf-8', index = False)