import sys
from math import ceil, floor, sqrt
import numpy as np
from heapq import *
from utils import *
from params import *


VEHICLE_IDX = None

def idx2spot(idx):
	if side == 'double':
		return ceil(idx / 2)
	else:
		assert side == 'single'
		return idx

def spot2blk(stop):
	# returns the lane block next to boarding stop 
	# for 1 <= stop <= system.half_N when double_sided
	if nUnit == 1:
		return stop
	else:
		return ceil(stop / nUnit)

def in_range(stop_idx, N):

	if (side == 'double'):
		
		if (angle == 0) and (mode == 'long'):
			if (stop_idx % 2 == 1):
				return range( min(N, stop_idx + 2), min(N, stop_idx + 3) + 1 )
			return range( min(N, stop_idx + 1), min(N, stop_idx + 2) + 1 )
		
		if (angle == 0) and (mode == 'short'):
			if (stop_idx % 2 == 1):
				return range( max(1, stop_idx - 2), stop_idx + 2 )
			return range( max(1, stop_idx - 3), stop_idx )

		if (angle == 90):
			if (stop_idx % 2 == 1):
				return range( max(1, stop_idx - 4), min(N, stop_idx + 5) + 1 )
			return range( max(1, stop_idx - 5), min(N, stop_idx + 4) + 1 )

	assert (side == 'single')
	if (angle == 0) and (mode == 'long'):
		return range( min(N, stop_idx + 1), min(N, stop_idx + 1) + 1)
	if (angle == 0) and (mode == 'short'):
		return range( max(1, stop_idx - 1), stop_idx )
	assert (angle == 90)
	return range( max(1, stop_idx - 2), min(N, stop_idx + 2) + 1 )

def out_range(stop_idx, N):
	
	if (side == 'double'):

		if (angle == 0) and (mode == 'long'):
			if (stop_idx % 2 == 1):
				return range( max(1, stop_idx - 2), stop_idx + 2 )
			return range( max(1, stop_idx - 3), stop_idx )

		if (angle == 0) and (mode == 'short'):
			if (stop_idx % 2 == 1):
				return range( max(1, stop_idx - 2), stop_idx + 2 )
			return range( max(1, stop_idx - 3), stop_idx )

		if (angle == 90):
			if (stop_idx % 2 == 1):
				return range( max(1, stop_idx - 4), min(N, stop_idx + 5) + 1 )
			return range( max(1, stop_idx - 5), min(N, stop_idx + 4) + 1 )

	assert side == 'single'
	if (angle == 0):
		return range( max(1, stop_idx - 1), stop_idx )
	assert (angle == 90)
	return range( max(1, stop_idx - 2), min(N, stop_idx + 2) + 1 )

class system():

	def __init__(self, N, seedSERV = None, seedPOUT = None, seedPLIN = None, seedDRIV = None):

		if simType == 'cav':
			self.timeSERV = ParamGen(Expo(rateSERV, seed = seedSERV))
			self.timeDRIV = ParamGen(Cons(rateDRIV, seed = seedDRIV))
			self.timePOUT = ParamGen(Cons(meanPOUT, seed = seedPOUT))
			self.timePLIN = ParamGen(Cons(meanPLIN, seed = seedPLIN))

		elif simType == 'exp':
			self.timeSERV = ParamGen(Expo(rateSERV, seed = seedSERV))
			self.timeDRIV = ParamGen(Unif2(rateDRIV, seed = seedDRIV))
			self.timePOUT = ParamGen(Expo(ratePOUT, seed = seedPOUT))
			self.timePLIN = ParamGen(Expo(ratePLIN, seed = seedPLIN))

		elif simType == 'exp1':
			self.timeSERV = ParamGen(Cons(rateSERV, seed = seedSERV))
			self.timeDRIV = ParamGen(Unif2(rateDRIV, seed = seedDRIV))
			self.timePOUT = ParamGen(Expo(ratePOUT, seed = seedPOUT))
			self.timePLIN = ParamGen(Expo(ratePLIN, seed = seedPLIN))

		elif simType == 'unif':
			self.timeSERV = ParamGen(Unif2(meanSERV, seed = seedSERV))
			self.timeDRIV = ParamGen(Unif2(rateDRIV, seed = seedDRIV))
			self.timePOUT = ParamGen(Unif2(meanPOUT, seed = seedPOUT))
			self.timePLIN = ParamGen(Unif2(meanPLIN, seed = seedPLIN))

		elif simType == 'unif1':
			self.timeSERV = ParamGen(Cons(meanSERV, seed = seedSERV))
			self.timeDRIV = ParamGen(Unif2(rateDRIV, seed = seedDRIV))
			self.timePOUT = ParamGen(Unif2(meanPOUT, seed = seedPOUT))
			self.timePLIN = ParamGen(Unif2(meanPLIN, seed = seedPLIN))

		elif simType == 'unif2':
			self.timeSERV = ParamGen(Unif2(meanSERV, seed = seedSERV))
			self.timeDRIV = ParamGen(Cons(rateDRIV, seed = seedDRIV))
			self.timePOUT = ParamGen(Cons(meanPOUT, seed = seedPOUT))
			self.timePLIN = ParamGen(Cons(meanPLIN, seed = seedPLIN))

		else:
			self.timeDRIV = ParamGen(Disc(rateDRIV, seed = seedDRIV))
			self.timePOUT = ParamGen(Tria(ratePOUT, seed = seedPOUT))
			self.timePLIN = ParamGen(Tria(ratePLIN, seed = seedPLIN))

			if simType == 'triag1':
				self.timeSERV = ParamGen(Tri1(rateSERV, seed = seedSERV))

			if simType == 'triag0':
				self.timeSERV = ParamGen(Tri0(rateSERV, seed = seedSERV))

			if simType == 'triag':
				self.timeSERV = ParamGen(Tria(rateSERV, seed = seedSERV))

		if side == 'double':
			self.half_N = N
			self.N = 2 * N
		else:
			assert side == 'single'
			self.N = N

		if spmatch:
			self.n = spot2blk(N) * CAR_LENGTH 
		else:
			self.n = N * LOT_LENGTH

		self.curr = 0.0
		self.start_time = 0.0
		self.eventheap = []
		self.waiting = []
		self.inservice = [None for _ in range(self.N)]
		self.head = None
		self.inCount = 0
		self.outCount = 0	
		self.entry_blocked = self.curr
		self.entry_cleared = self.curr
		self.debug = debug
		self.debug_times = []
		self.debug_speed = []

		self.max_idle = 0.0
		self.max_idle_idx = None
		self.max_pct_idle = 0.0
		self.max_pct_idle_idx = None
		self.prod_time = []
		self.idle_time = []
		self.pct_idle = []	
		self.first_service = None
		self.last_departure = - meanDRIV
		
	def add_event(self, event):
		heappush( self.eventheap, event) 

	def state_print(self):

		print (self.curr, self.curr_vehicle.stop_idx, self.curr_typ)
		x = []
		if side == 'single':
			y = [0 for _ in range(spot2blk(self.N))]
		else:
			assert side == 'double'
			y = [0 for _ in range(spot2blk(self.half_N))]

		for car in self.inservice:
			if car is None:
				x.append(0)
			elif car.status == 3:
				x.append(1)
			elif car.status in [3.5, 4]:
				x.append(2)
			elif car.status == 2:
				x.append(m_in + m_out)
			else:
				assert car.status == 5
				x.append(3)

		car = self.head
		while car is not None:
			if car.status in [1, 6]:
				car.update_loc()
				if np.abs( car.curr_loc % CAR_LENGTH - CAR_LENGTH ) < 1:
					lane_block = ceil(car.curr_loc / CAR_LENGTH)
				else:
					lane_block = floor( car.curr_loc / CAR_LENGTH )
				if lane_block > len(y):
					print ('one vehicle is leaving ...')
				else:
					if y[lane_block - 1] != 0:
						import pdb; pdb.set_trace()
					if car.status == 1:
						y[lane_block - 1] = car.stop_idx
					else:
						y[lane_block - 1] = self.N + 1
			car = car.nex

		print (x)
		print (y)

	def debug_print(self):

		if side == 'single':
			print ('Boarding area ...')
			for car in self.inservice:
				if car is not None:
					print ('Vehicle %s is assigned the spot %s and its current status is %s.' %(car.idx, car.stop_idx, car.status))

		else: 
			assert side == 'double'
			print ('Left boarding area ...')
			for idx in range(self.half_N):
				if self.inservice[2 * idx] is not None:
					car = self.inservice[2 * idx]
					print ('Vehicle %s is assigned the spot %s and its current status is %s.' %(car.idx, car.stop_idx, car.status))

			print ('Right boarding area ...')
			for idx in range(self.half_N):
				if self.inservice[2 * idx + 1] is not None:
					car = self.inservice[2 * idx + 1]
					print ('Vehicle %s is assigned the spot %s and its current status is %s.' %(car.idx, car.stop_idx, car.status))

		print ('Through lane ...')
		car = self.head
		while car != None:
			car.update_loc()
			print ('Vehicle %s is assigned the spot %s; its current status is %s and its current location is %s.' %(car.idx, car.stop_idx, car.status, car.curr_loc) )
			car = car.nex			

	def idle_time_calc(self, curr_vehicle):

		total = self.curr - curr_vehicle.enter_time 
		if spmatch:
			total += meanDRIV
		if (total < curr_vehicle.prod_time) and np.abs(total - curr_vehicle.prod_time) > 3 * SMALL_INTERVAL:
			import pdb; pdb.set_trace()
		idle = max(0.0, total - curr_vehicle.prod_time)
		if idle > self.max_idle:
			self.max_idle = idle
			self.max_idle_idx = curr_vehicle.idx
		if idle + curr_vehicle.prod_time == 0.0:
			import pdb; pdb.set_trace()
		if idle / (idle + curr_vehicle.prod_time) > self.max_pct_idle:
			self.max_pct_idle = idle / (idle + curr_vehicle.prod_time)
			self.max_pct_idle_idx = curr_vehicle.idx
		self.prod_time.append(curr_vehicle.prod_time)
		self.idle_time.append(idle)
		self.pct_idle.append( idle / (idle + curr_vehicle.prod_time) )

	def run(self):

		while self.curr - self.start_time <= SIM_UNIT:
			
			curr_event = heappop(self.eventheap)
			curr_vehicle = curr_event.vehicle
			curr_typ  = curr_event.typ
			self.curr_vehicle = curr_vehicle

			self.curr = float(curr_event.time)	
			self.curr_typ = curr_typ			
			try:
				stop = curr_vehicle.stop
			except AttributeError:
				assert curr_typ == 'enter_system'

			# print (self.curr, curr_vehicle.idx, curr_typ)

			if VEHICLE_IDX is not None and curr_vehicle.idx == VEHICLE_IDX:
				import pdb; pdb.set_trace()

			################################### update system ###################################
			if curr_typ == 'leave_system':
				self.leave_system(curr_vehicle)

			elif curr_typ == 'start_pulling_in':
				self.start_pulling_in(curr_vehicle)

			elif curr_typ == 'start_service':
				self.start_service(curr_vehicle)
			
			elif curr_typ == 'prepare_pulling_out':
				self.prepare_pulling_out(curr_vehicle)

			elif curr_typ == 'finish_pulling_out':
				self.finish_pulling_out(curr_vehicle)

			else:
				assert curr_typ == 'enter_system'
				self.enter_system(curr_vehicle, debug_idx = VEHICLE_IDX)

			if VEHICLE_IDX is not None and curr_vehicle.idx == VEHICLE_IDX:
				import pdb; pdb.set_trace()

			if self.debug:
				self.debug_print()
				# self.state_print()

		self.start_time = self.curr
		return (self.outCount)

	def leave_system(self, curr_vehicle):

		curr_vehicle.update_loc()

		if np.abs( curr_vehicle.curr_loc - curr_vehicle.dest_to_stop ) > 1e-5:
			if curr_vehicle.end_time <= self.curr + SMALL_INTERVAL:
				import pdb; pdb.set_trace() 
			self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'leave_system') )
			return

		assert self.head == curr_vehicle
		assert curr_vehicle.prev == None					
		self.head = curr_vehicle.nex
		if curr_vehicle.nex != None:
			curr_vehicle.nex.prev = None
			curr_vehicle.nex = None

		self.outCount += 1
		if angle == 0:
			curr_vehicle.prod_time += ( (curr_vehicle.dest_to_stop - curr_vehicle.stop * LOT_LENGTH) / curr_vehicle.driv )
		elif spmatch and angle == 90:
			curr_vehicle.prod_time += ( (curr_vehicle.dest_to_stop - curr_vehicle.lane_idx * CAR_LENGTH) / curr_vehicle.driv )
		else:
			assert angle == 90
			curr_vehicle.prod_time += ( (curr_vehicle.dest_to_stop - ((curr_vehicle.stop - 1) * LOT_LENGTH + CAR_LENGTH)) / curr_vehicle.driv )
		self.idle_time_calc(curr_vehicle)
		self.last_departure = self.curr
		return

	def start_pulling_in(self, curr_vehicle):

		curr_vehicle.update_loc()

		if np.abs( curr_vehicle.curr_loc - curr_vehicle.dest_to_stop ) > 1e-5:
			if curr_vehicle.end_time <= self.curr + SMALL_INTERVAL:
				import pdb; pdb.set_trace() 
			self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_pulling_in') )
			return

		curr_vehicle.curr_loc = curr_vehicle.dest_to_stop
		delayed = False
		req_time = self.curr

		# if the vehicle has arrived at the desired destination while its enter maneuver blocked by vehicles on the through lane
		# i.e. 1) if its immediate proceeding vehicle is having an exit maneuver at the same spot from the middle lane to the through lane;
		if (curr_vehicle.prev is not None) and (curr_vehicle.prev.stop == curr_vehicle.stop) and (curr_vehicle.prev.status == 5):
			assert curr_vehicle.prev.pout_end > self.curr
			delayed = True 
			req_time = max( req_time, curr_vehicle.prev.pout_end )

		# or 2) if the immediate proceeding vehicle is stopped.
		if (curr_vehicle.prev is not None) and (curr_vehicle.prev.status != 2) and (curr_vehicle.prev.curr_loc <= curr_vehicle.dest_to_stop + 1.5 * CAR_LENGTH):
			cp = curr_vehicle.prev.traj.head
			if cp.data.t > self.curr:
				if cp.data.t <= self.curr + 2 * SMALL_INTERVAL:
					delayed = True
					req_time = max( req_time, cp.data.t )
				else:
					import pdb; pdb.set_trace()
			else:
				while (cp.nex is not None) and (cp.nex.data.t <= self.curr):
					cp = cp.nex
				assert cp is not None
				if (cp.data.v == 0.0):
					assert cp.nex.data.t > self.curr
					delayed = True
					req_time = max( req_time, cp.nex.data.t )

		if delayed:
			assert req_time > self.curr
			curr_vehicle.traj = DLinkedList()
			curr_vehicle.traj.addEnd( changePoint(curr_vehicle.dest_to_stop, self.curr, 0.0) )
			curr_vehicle.traj.addEnd( changePoint(curr_vehicle.dest_to_stop, req_time, 'D') )
			curr_vehicle.end_time = req_time
			self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_pulling_in') )
			car = curr_vehicle.nex
			while car is not None:
				car.update_loc()
				car.update_traj()
				car = car.nex
			return

		assert self.inservice[curr_vehicle.stop_idx - 1] is None
		self.inservice[curr_vehicle.stop_idx - 1] = curr_vehicle
		curr_vehicle.start_in()

		car = curr_vehicle.nex
		while car != None:
			car.update_loc()
			car.update_traj()
			car = car.nex

		assert curr_vehicle.end_time is not None
		self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_service') )
		return

	def start_service(self, curr_vehicle):

		curr_vehicle.start_service()

		if curr_vehicle.prev != None:
			curr_vehicle.prev.nex = curr_vehicle.nex
		else:
			assert self.head == curr_vehicle
			self.head = curr_vehicle.nex

		if curr_vehicle.nex != None:
			car = curr_vehicle.nex
			car.prev = curr_vehicle.prev
			while car != None:
				car.update_loc()
				car.update_traj()
				car = car.nex

		curr_vehicle.prev = None
		curr_vehicle.nex = None

		assert curr_vehicle.serv_end > self.curr
		self.add_event( event(curr_vehicle.serv_end, curr_vehicle, 'prepare_pulling_out') )
		return

	def prepare_pulling_out(self, curr_vehicle):

		stop = curr_vehicle.stop
		assert self.inservice[curr_vehicle.stop_idx - 1] == curr_vehicle

		if curr_vehicle.status == 3:
			curr_vehicle.status = 4
			first_attempt = True
		else:
			first_attempt = False
		assert curr_vehicle.status == 4

		if self.first_service is None:
			self.first_service = self.curr
			first_attempt = False

		if curr_vehicle.idx == VEHICLE_IDX:
			delay_reason = None
			delay_status = None
			delay_speed = None

		req_time = self.curr

		# Firstly, check if any vehicles in the neighborhood have started their exit maneuvers
		# or have not finished their enter maneuvers

		# for k in in_range(curr_vehicle.stop_idx, self.N):
		# 	if (self.inservice[k - 1] is not None) and (self.inservice[k - 1].status == 2):
		# 		if (angle == 0) and (mode == 'short'):
		# 			if curr_vehicle.idx == VEHICLE_IDX and self.inservice[k - 1].plin_end - min(meanDRIV, max(0, self.inservice[k - 1].plin_time - .5)) > req_time:
		# 				delay_reason = 1
		# 				delay_status = 2
		# 			req_time = max( req_time, self.inservice[k - 1].plin_end - min(meanDRIV, max(0, self.inservice[k - 1].plin_time - .5)) )
		# 			# req_time = max( req_time, self.inservice[k - 1].plin_end - min(1.0, max(0, self.inservice[k - 1].plin_time - .5)) )
		# 		else:
		# 			if curr_vehicle.idx == VEHICLE_IDX and self.inservice[k - 1].plin_end > req_time:
		# 				delay_reason = 1
		# 				delay_status = 2
		# 			req_time = max( req_time, self.inservice[k - 1].plin_end )

		for k in out_range(curr_vehicle.stop_idx, self.N):
			if (self.inservice[k - 1] is not None) and (self.inservice[k - 1].status == 5):
				# if not self.inservice[k - 1].pout_end > self.curr:
				# 	import pdb; pdb.set_trace()
				if curr_vehicle.idx == VEHICLE_IDX and self.inservice[k - 1].pout_end > req_time:
					delay_reason = 1
					delay_status = 5
				req_time = max(req_time, self.inservice[k - 1].pout_end )

		if req_time > self.curr:
			curr_vehicle.end_time = req_time
			self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'prepare_pulling_out') )
			if curr_vehicle.idx == VEHICLE_IDX:
				self.debug_times.append( tuple( (delay_reason, delay_status, req_time) ) )
				assert delay_reason != 5
			return

		# Lastly, check if any vehicles are traveling on the through lane, 
		# s.t. the exit maneuver of the current vehicle will be blocked or block the movements of the existing vehicles
		assert req_time == self.curr
		if curr_vehicle.idx == VEHICLE_IDX:
			delayed, req_time, prev, delay_reason, delay_status, delay_speed = self.check_lane_zero_long(curr_vehicle, first_attempt, delay_reason, delay_status, delay_speed)
		else:
			delayed, req_time, prev = self.check_lane_zero_long(curr_vehicle, first_attempt)

		if delayed:
			assert req_time > self.curr
			curr_vehicle.end_time = req_time
			self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'prepare_pulling_out') )
			if curr_vehicle.idx == VEHICLE_IDX:
				self.debug_times.append( tuple( (delay_reason, delay_status, req_time) ) )
				if delay_reason == 5:
					assert delay_speed is not None
					self.debug_speed.append( delay_speed )		
			return

		# Now, the current vehicle is ready to start the exit maneuver
		curr_vehicle.prev = prev
		if prev != None:
			curr_vehicle.nex = prev.nex
			try:
				prev.nex.prev = curr_vehicle
			except:
				pass
			prev.nex = curr_vehicle

		elif self.head != None:
			self.head.prev = curr_vehicle
			curr_vehicle.nex = self.head 
			self.head = curr_vehicle

		else:
			self.head = curr_vehicle

		curr_vehicle.start_out()
		if spmatch and first_attempt and meanDRIV - (self.curr - self.first_service) % meanDRIV > SMALL_INTERVAL:
			assert simType == 'cav'
			assert curr_vehicle.pout_time == meanPOUT
			curr_vehicle.pout_time -= (self.curr - self.first_service) % meanDRIV
			curr_vehicle.pout_end = curr_vehicle.pout_time + curr_vehicle.pout_start
			curr_vehicle.prod_time -= (self.curr - self.first_service) % meanDRIV
			curr_vehicle.update_traj()
		
		assert curr_vehicle.pout_end > self.curr
		self.add_event( event( curr_vehicle.pout_end, curr_vehicle, 'finish_pulling_out') )

		# and we update the trajectories of all upcoming vehicles
		car = curr_vehicle.nex
		while car != None:
			car.update_loc()
			car.update_traj()
			car = car.nex

		# lastly we schedule the replacement vehicles
		new_vehicle = vehicle(self)
		if spmatch:
			if first_attempt and meanDRIV - (self.curr - self.first_service) % meanDRIV > SMALL_INTERVAL:
				assert simType == 'cav'
				self.add_event( event(self.curr + meanDRIV - (self.curr - self.first_service) % meanDRIV, new_vehicle, 'enter_system') )
			else:
				self.add_event( event( self.curr + meanDRIV, new_vehicle, 'enter_system') )
		else:
			self.add_event( event( self.curr, new_vehicle, 'enter_system') )
		heappush( self.waiting, (- curr_vehicle.stop_idx) )
		return

	def finish_pulling_out(self, curr_vehicle):

		assert curr_vehicle.status == 5
		curr_vehicle.status = 6

		assert self.inservice[curr_vehicle.stop_idx - 1] is not None
		self.inservice[curr_vehicle.stop_idx - 1] = None
		if not (curr_vehicle.end_time is not None and curr_vehicle.end_time > self.curr):
			import pdb; pdb.set_trace()
		self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'leave_system') )				
		return 

	def enter_system(self, curr_vehicle, debug_idx = None):

		assert curr_vehicle.status == 0

		if self.entry_blocked == self.curr:
			self.add_event( event( self.entry_cleared, curr_vehicle, 'enter_system' ) )
			return

		# if there is no vehicle on the driving lane
		# the replacement vehicle is assigned to the spot with the largest index
		if self.head == None:
			self.inCount += 1
			self.head = curr_vehicle
			curr_vehicle.assign_spot( -heappop(self.waiting) )
			if spmatch:
				assert simType == 'cav'
				curr_vehicle.curr_loc = LOT_LENGTH
				if spot2blk(curr_vehicle.stop) == 1:
					curr_vehicle.status = 2
					curr_vehicle.plin_time = self.timePLIN.next()
					curr_vehicle.plin_start = max(0., self.curr - meanDRIV)
					curr_vehicle.plin_end = curr_vehicle.plin_time + curr_vehicle.plin_start
					curr_vehicle.prod_time += curr_vehicle.plin_time
					curr_vehicle.update_traj()
					assert curr_vehicle.end_time != None
					assert self.inservice[curr_vehicle.stop_idx - 1] is None
					self.inservice[curr_vehicle.stop_idx - 1] = curr_vehicle
					self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'start_service') )
					if debug:
						print (self.curr, curr_vehicle.stop_idx, 'enter_system')
						import pdb;pdb.set_trace()
					return
				else:
					assert curr_vehicle.dest_to_stop >= curr_vehicle.curr_loc
			curr_vehicle.update_traj()
			assert curr_vehicle.end_time != None and curr_vehicle.end_time >= self.curr
			self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'start_pulling_in') )
			if debug:
				print (self.curr, curr_vehicle.stop_idx, 'enter_system')
				import pdb; pdb.set_trace()
			return

		# if there are vehicles on the driving lane, find the last one
		req_time = self.curr
		car = self.head
		while car.nex is not None:
			car = car.nex	
		car.update_loc()

		# if the last one is within CAR_LENGTH (i.e. first lane block occupied)
		# the replacement vehicle cannot enter under either access control
		if car.curr_loc <= CAR_LENGTH + SMALL_INTERVAL:
			car_time = car.calc_time(CAR_LENGTH + SMALL_INTERVAL)
			if car_time == self.curr:
				cp = car.traj.head
				assert cp.data.t <= self.curr
				while cp.nex is not None and cp.nex.data.t <= self.curr:
					cp = cp.nex
				if cp.nex is None:
					import pdb; pdb.set_trace()
				else:
					assert cp.nex.data.t > self.curr
					if cp.data.v > 0.0:
						pass
					else:
						import pdb; pdb.set_trace()
			else:
				assert car_time > self.curr
				req_time = max( req_time, car_time)

		if spmatch and req_time <= self.curr:
			if car.curr_loc < LOT_LENGTH + CAR_LENGTH - SMALL_INTERVAL:
				if car.status not in [1, 6]:
					print ('!!!!!!!!!!!!!!!!')
					import pdb; pdb.set_trace()
				assert car.dest_to_stop >= LOT_LENGTH + CAR_LENGTH
				car_time = car.calc_time(LOT_LENGTH + CAR_LENGTH)
				req_time = max( req_time, car_time )

		# if self.inservice[0] is not None and self.inservice[0].status == 4:
		# 	req_time = max( req_time, self.inservice[0].end_time)
		# 	assert (req_time > self.curr)

		# if side == 'double' and self.inservice[1] is not None and self.inservice[1].status == 4:
		# 	req_time = max( req_time, self.inservice[1].end_time) 
		# 	assert (req_time > self.curr)

		if req_time > self.curr:
			self.entry_blocked = self.curr
			self.entry_cleared = req_time
			self.add_event( event( self.entry_cleared, curr_vehicle, 'enter_system') )
			return

		if debug_idx is not None and curr_vehicle.idx == debug_idx:
			import pdb; pdb.set_trace()

		if control == 'partial':
			self.inCount += 1
			curr_vehicle.prev = car
			car.nex = curr_vehicle
			curr_vehicle.assign_spot( -heappop(self.waiting) )
			if spmatch:
				assert simType == 'cav'
				assert curr_vehicle.prev.curr_loc >= LOT_LENGTH + CAR_LENGTH - SMALL_INTERVAL
				curr_vehicle.curr_loc = LOT_LENGTH
				if spot2blk(curr_vehicle.stop) == 1:
					curr_vehicle.status = 2
					curr_vehicle.plin_time = self.timePLIN.next()
					curr_vehicle.plin_start = max(0., self.curr - meanDRIV)
					curr_vehicle.plin_end = curr_vehicle.plin_time + curr_vehicle.plin_start
					curr_vehicle.prod_time += curr_vehicle.plin_time
					curr_vehicle.update_traj()
					assert curr_vehicle.end_time != None
					assert self.inservice[curr_vehicle.stop_idx - 1] is None
					self.inservice[curr_vehicle.stop_idx - 1] = curr_vehicle
					self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'start_service') )
					if debug:
						print (self.curr, curr_vehicle.stop_idx, 'enter_system')
						import pdb; pdb.set_trace()
					return
				else:
					assert curr_vehicle.dest_to_stop >= curr_vehicle.curr_loc
			curr_vehicle.update_traj()
			assert curr_vehicle.end_time != None and curr_vehicle.end_time >= self.curr
			self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'start_pulling_in') )
			if debug:
				print (self.curr, curr_vehicle.stop_idx, 'enter_system')
				import pdb; pdb.set_trace()
			return

		assert control == 'full'
		# car is the last vehicle on the driving lane 
		# and also the prev for the replacement vehicle if the latter can enter
		j_new = []
		last = car

		# j_new will include the spots that the replacement vehicle can head to 
		# without being blocked or delayed by the last vehicle on the lane in expectation
		for j in sorted(self.waiting, reverse = True):
			j = - j
			assert j > 0
			if side == 'double':
				J = idx2spot(j)
			else:
				assert side == 'single'
				J = j

			if (J < car.stop) or (car.status == 6):
				j_new.append(j)

			elif car.status == 2:
				# K_in with K = car.j and J = idx2spot(j)
				assert car.stop_idx != j
				assert car.plin_start <= self.curr
				assert not J == car.stop == 1
				assert car.stop >= 3
				if ( (car.stop - 2) * LOT_LENGTH / rateDRIV >= max(0.0, meanPLIN - (self.curr - car.plin_start)) ):
					j_new.append(j)

			elif car.status == 5:
				# K_out with K = car.j and J = idx2spot(j)
				if ( (car.stop - 1) * LOT_LENGTH / rateDRIV >= max(0.0, meanPOUT - (self.curr - car.pout_start)) ):
					j_new.append(j)

			else:
				assert car.status == 1
				# I_in with K = car.j and J = idx2spot(j)
				assert car.stop_idx != j
				assert car.stop >= 3
				if ( (car.curr_loc - LOT_LENGTH) / rateDRIV >= meanPLIN ):
					j_new.append(j)

		if j_new != []:
			assert j_new[-1] == max(j_new)
			car_time = 0.0

		else:
			j = - sorted(self.waiting, reverse = True)[0]
			if side == 'double':
				assert (idx2spot(j) >= car.stop)
			else:
				assert side == 'single'
				assert (j >= car.stop)

			if car.status == 2:
				assert car.stop_idx != j
				assert car.plin_start <= self.curr
				assert ( (car.stop - 2) * LOT_LENGTH / rateDRIV < meanPLIN - (self.curr - car.plin_start) )
				car_time = meanPLIN - (self.curr - car.plin_start) - (car.stop - 2) * LOT_LENGTH / rateDRIV

			elif car.status == 5:
				assert car.pout_start <= self.curr
				assert ( (car.stop - 1) * LOT_LENGTH / rateDRIV < meanPOUT - (self.curr - car.pout_start)  )
				car_time = meanPOUT - (self.curr - car.pout_start) - (car.stop - 1) * LOT_LENGTH / rateDRIV

			else:
				assert car.status == 1
				assert car.stop_idx != j
				assert ( (car.curr_loc - LOT_LENGTH) / rateDRIV < meanPLIN )
				car_time = meanPLIN - (car.curr_loc - LOT_LENGTH) / rateDRIV

		while (j_new != []) and (car.prev is not None):

			car = car.prev
			car.update_loc()
			for idx in range(len(j_new)):
				j = j_new[idx]
				if side == 'double':
					J = idx2spot(j)
				else:
					assert side == 'single'
					J = j

				if (J < car.stop) or (car.status == 6):
					pass

				elif car.status == 2:
					# K_in with K = car.j and J = idx2spot(j)
					assert car.stop_idx != j
					assert car.plin_start <= self.curr
					if ( (car.stop - 2) * LOT_LENGTH / rateDRIV < max(0.0, meanPLIN - (self.curr - car.plin_start)) ):
						j_new = j_new[:idx]
						if idx == 0:
							car_time = max(car_time, meanPLIN - (self.curr - car.plin_start) - (car.stop - 2) * LOT_LENGTH / rateDRIV)
						break

				elif car.status == 5:
					# K_out with K = car.j and J = idx2spot(j)
					assert car.pout_start <= self.curr
					if ( (car.stop - 1) * LOT_LENGTH / rateDRIV < max(0.0, meanPOUT - (self.curr - car.pout_start)) ):
						j_new = j_new[:idx]
						if idx == 0:
							car_time = max(car_time, meanPOUT - (self.curr - car.pout_start) - (car.stop - 1) * LOT_LENGTH / rateDRIV)
						break

				else:
					assert car.status == 1
					# I_in with K = car.j and J = idx2spot(j)
					assert car.stop_idx != j
					if ( (car.curr_loc - LOT_LENGTH) / rateDRIV < meanPLIN ):
						j_new = j_new[:idx]
						if (idx == 0):
							car_time = max(car_time, meanPLIN - (car.curr_loc - LOT_LENGTH) / rateDRIV)
						break

		if j_new == []:		
			car_time = self.curr + car_time
			if car_time == self.curr:
				car_time += SMALL_INTERVAL
			if car_time <= self.curr:
				import pdb; pdb.set_trace()
			self.add_event( event( car_time, curr_vehicle, 'enter_system') )
			return

		# car.prev is None
		# i.e. there is at least one spot where a replacement vehicle can head to 
		# without being blocked or delayed by any vehicle already on the lane
		# if there are multiple, choose the largest 
		self.inCount += 1
		assert j_new[-1] == max(j_new)
		curr_vehicle.assign_spot( j_new[-1] )
		assert (- j_new[-1]) in self.waiting
		self.waiting.remove( - j_new[-1] )
		curr_vehicle.prev = last
		last.nex = curr_vehicle
		if spmatch:
			assert simType == 'cav'
			assert curr_vehicle.prev.curr_loc >= LOT_LENGTH + CAR_LENGTH - SMALL_INTERVAL
			curr_vehicle.curr_loc = LOT_LENGTH
			if spot2blk(curr_vehicle.stop) == 1:
				curr_vehicle.status = 2
				curr_vehicle.plin_time = self.timePLIN.next()
				curr_vehicle.plin_start = max(0., self.curr - meanDRIV)
				curr_vehicle.plin_end = curr_vehicle.plin_time + curr_vehicle.plin_start
				curr_vehicle.prod_time += curr_vehicle.plin_time
				curr_vehicle.update_traj()
				assert curr_vehicle.end_time != None
				assert self.inservice[curr_vehicle.stop_idx - 1] is None
				self.inservice[curr_vehicle.stop_idx - 1] = curr_vehicle
				self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'start_service') )
				if debug:
					print (self.curr, curr_vehicle.stop_idx, 'enter_system')
					import pdb; pdb.set_trace()
				return
			else:
				assert curr_vehicle.dest_to_stop >= curr_vehicle.curr_loc
		curr_vehicle.update_traj()
		assert curr_vehicle.end_time != None and curr_vehicle.end_time >= self.curr
		self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'start_pulling_in') )
		if debug:
			print (self.curr, curr_vehicle.stop_idx, 'enter_system')
			import pdb; pdb.set_trace()
		return

	def check_lane_zero_long(self, curr_vehicle, first_attempt, delay_reason = None, delay_status = None, delay_speed = None):

		assert angle == 0 and mode == 'long'

		delayed = False
		req_time = self.curr

		car = self.head
		prev = None
		stopped = False

		stop = curr_vehicle.stop
		idx = curr_vehicle.idx

		while car != None:
			car.update_loc()

			if car.curr_loc >= stop * LOT_LENGTH + CAR_LENGTH - SMALL_INTERVAL:
				prev = car

			# changes added on Jan 22, 2021
			elif car.status == 2 and car.stop == stop + 1:
				assert (side == 'single' and stop <= self.N - 1) or (side == 'double' and stop <= self.half_N - 1) 
				assert (self.inservice[car.stop_idx - 1] == car)
				assert car.plin_end >= self.curr > car.plin_start
				if spmatch:
					if meanPLIN + car.plin_start > self.curr:
						delayed = True
						car_time = car.plin_start + meanPLIN
						assert car_time > self.curr
						if idx == VEHICLE_IDX and car_time > req_time:
							delay_reason = 2
							delay_status = 2
						req_time = max( req_time, car_time)
					else:
						prev = car
				else:
					if meanPLIN + car.plin_start - meanDRIV > self.curr:
						delayed = True
						car_time = car.plin_start + meanPLIN - meanDRIV
						assert car_time > self.curr
						if idx == VEHICLE_IDX and car_time > req_time:
							delay_reason = 2
							delay_status = 2
						req_time = max( req_time, car_time)
					else:
						prev = car

			elif stopped and car.status != 5:
				pass
			
			elif car.status == 5: 
				if car.stop >= stop and car.pout_end != self.curr:
					import pdb; pdb.set_trace()
				assert car.dest_to_stop >= stop * LOT_LENGTH + CAR_LENGTH

				if spmatch:
					car_time = car.calc_time((stop - 1) * LOT_LENGTH) - self.curr
					if (first_attempt and car_time < meanPOUT - (self.curr - self.first_service) % meanDRIV - SMALL_INTERVAL) or (not first_attempt and car_time < meanPOUT - SMALL_INTERVAL):
						delayed = True
						car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH )
						assert car_time > self.curr
						if idx == VEHICLE_IDX and car_time > req_time:
							delay_reason = 3
							delay_status = 5
						req_time = max( req_time, car_time )

				else:
					car_time = max(0, meanPOUT - (self.curr - car.pout_start)) + ((stop - 1) * LOT_LENGTH - car.curr_loc) / rateDRIV
					if car_time < meanPOUT:
						delayed = True
						car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH )
						assert car_time > self.curr
						if idx == VEHICLE_IDX and car_time > req_time:
							delay_reason = 3
							delay_status = 5
						req_time = max( req_time, car_time )

			elif car.status == 2:
				assert (car.stop < stop) or (car.stop == stop and side == 'double')
				if car.stop == stop:
					break
				stopped = True

			elif (car.stop == stop) and (car.status == 1):
				assert (car.curr_loc <= car.dest_to_stop == (stop - 1) * LOT_LENGTH)
				break			 

			elif (car.stop < stop) and (car.status == 1):
				if stop < 2:
					import pdb; pdb.set_trace()
				assert (car.curr_loc <= car.dest_to_stop <= (stop - 2) * LOT_LENGTH)
				stopped = True

			elif (car.stop == stop + 1) and (car.status == 1):
				assert car.dest_to_stop == stop * LOT_LENGTH
				car_time = ((stop - 1) * LOT_LENGTH - car.curr_loc) / rateDRIV
				if car_time < meanPOUT:
					delayed = True
					if not car.end_time > self.curr - SMALL_INTERVAL:
						import pdb; pdb.set_trace()
					if car.end_time <= self.curr:
						car_time = car.end_time + SMALL_INTERVAL
					else:
						car_time = car.end_time
					if idx == VEHICLE_IDX and car_time > req_time:
						delay_reason = 4
						delay_status = 1
					req_time = max( req_time, car_time )

			else:
				assert (car.status in [1, 6])
				assert car.dest_to_stop >= stop * LOT_LENGTH + CAR_LENGTH
				car_time = ((stop - 1) * LOT_LENGTH - car.curr_loc) / rateDRIV
				if car_time < meanPOUT:
					delayed = True
					car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH )
					assert car_time > self.curr
					if idx == VEHICLE_IDX and car_time > req_time:
						delay_reason = 5
						delay_status = car.status
						cp = car.traj.head
						if cp.data.t > self.curr:
							import pdb; pdb.set_trace()
						while cp.nex is not None and cp.nex.data.t <= self.curr:
							cp = cp.nex
						if cp.nex is None:
							import pdb; pdb.set_trace()
						assert cp.nex.data.t > self.curr >= cp.data.t 
						delay_speed = cp.data.v
					req_time = max( req_time, car_time )						

			car = car.nex

		if idx == VEHICLE_IDX: 
			return (tuple( (delayed, req_time, prev, delay_reason, delay_status, delay_speed) ))
		else:
			return (tuple( (delayed, req_time, prev) ))

class vehicle():
	
	def __init__(self, sys, getin = False, stop_idx = None):

		self.sys = sys
		self.driv = self.sys.timeDRIV.next() 
		self.status = 0

		self.idx = None
		self.stop_idx = None
		self.stop = None
		self.lane_idx = None
		self.curr_loc = None
		self.dest_to_stop = None
		self.enter_time = None
		self.end_time = None
		self.prod_time = 0.0	

		self.prev = None
		self.nex = None
		self.traj = None

		if getin:
			assert stop_idx is not None
			self.assign_spot(stop_idx)
			self.status = 2
			self.start_service()
		else:
			assert stop_idx is None
			self.curr_loc = 0.0

	def assign_spot(self, stop_idx):
		assert self.status == 0
		self.status = 1
		self.idx = self.sys.inCount
		self.stop_idx = stop_idx
		self.enter_time = self.sys.curr
		self.stop = idx2spot(stop_idx)
		self.lane_idx = self.stop
		if (angle == 0) and (mode == 'long'):
			self.dest_to_stop = (self.stop - 1) * LOT_LENGTH
		elif (angle == 0):
			assert (mode == 'short')
			self.dest_to_stop = self.stop * LOT_LENGTH
		else:
			assert (angle == 90)
			self.lane_idx = spot2blk(self.stop)
			if spmatch:
				self.dest_to_stop = self.lane_idx * CAR_LENGTH
			else:
				self.dest_to_stop = (self.stop - 1) * LOT_LENGTH + CAR_LENGTH

	def start_in(self):
		assert self.curr_loc == self.dest_to_stop
		assert (self.status == 1)
		self.status = 2
		self.plin_time = self.sys.timePLIN.next()
		self.plin_end = self.plin_time + self.sys.curr
		self.plin_start = self.sys.curr
		self.prod_time += (self.dest_to_stop / self.driv + self.plin_time)
		self.update_traj()

	def start_service(self):
		assert (self.status == 2)
		self.status = 3
		self.serv_time = self.sys.timeSERV.next()
		self.serv_end = self.serv_time + self.sys.curr
		self.serv_start = self.sys.curr
		self.prod_time += self.serv_time
		self.traj = None

	def start_out(self):
		assert self.status == 4
		self.status = 5
		if spmatch and angle == 90:
			self.curr_loc = self.lane_idx * CAR_LENGTH
		elif angle == 90:
			self.curr_loc = (self.stop - 1) * LOT_LENGTH + CAR_LENGTH 
		else:
			self.curr_loc = self.stop * LOT_LENGTH
		self.dest_to_stop = self.sys.n + CAR_LENGTH
		self.pout_time = self.sys.timePOUT.next()
		self.pout_end = self.pout_time + self.sys.curr
		self.pout_start = self.sys.curr
		self.prod_time += self.pout_time
		if not spmatch and angle == 0 and mode == 'long':
			if self.prev is not None and self.prev.status == 2 and self.prev.stop == self.stop + 1 and self.prev.plin_end > self.pout_end:
				self.pout_end = self.prev.plin_end
		self.update_traj()

	def calc_time(self, loc):

		if self.curr_loc > loc:
			if control == 'full':
				pass
			else:
				import pdb; pdb.set_trace()

		cp = self.traj.head

		# if the curr_loc is the target loc,
		# then find the last time that it stays here
		# i.e. the first time that it starts to have nonzero speed.
		# NOTE: nonzero speed can be either positive or 'D'.
		if self.curr_loc == loc:

			if cp.data.t > self.sys.curr:
				if np.abs(cp.data.t - self.sys.curr) > SMALL_INTERVAL:
					import pdb; pdb.set_trace()
				elif cp.nex is not None:
					import pdb; pdb.set_trace()
				else:
					return cp.data. t

			while cp.nex is not None and cp.nex.data.t <= self.sys.curr:
				cp = cp.nex
			assert cp.data.t <= self.sys.curr

			if cp.nex is None:
				return self.sys.curr
			else:
				assert cp.nex.data.t > self.sys.curr
				if cp.data.v > 0.0:
					return self.sys.curr
				while cp.data.v == 0.0:
					cp = cp.nex
				return cp.data.t

		# since all trajectories end with speed 'D' i.e. nonzero
		# we can expect that all vehicles with curr_loc == loc already considered
		# except for those starting with cp.data.v > 0.0,
		# which means the vehicle is arrived at the loc
		while cp.nex != None:
			if cp.nex.data.x <= loc:
				cp = cp.nex
			else:
				break
		# assert cp.data.v != 0.0
		if cp.data.v == 0.0:
			import pdb;pdb.set_trace()

		# now we start with the changepoint s.t.
		# cp.data.x <= loc < cp.nex.data.x
		# Note: if cp.data.v == 0.0, then cp.data.x = cp.nex.data.x
		# given that cp.data.x <= loc
		# we can expect cp.nex.data.x <= loc thus the iteration will go on 
		# which explains the strict inequality between loc and cp.nex.data.x
		if cp.nex == None:
			assert cp.data.v == 'D'
			return cp.data.t

		return cp.data.t + (loc - cp.data.x) / cp.data.v
	
	def update_loc(self):

		assert (self.status != 3) and (self.status != 4)

		if (self.status == 2):
			if spmatch and spot2blk(self.stop) == 1:
				assert self.curr_loc == LOT_LENGTH 
			else:
				assert self.curr_loc == self.dest_to_stop
			return

		if (self.status == 5):
			if spmatch and angle == 90:
				assert self.curr_loc == self.lane_idx * CAR_LENGTH
			elif angle == 90:
				assert self.curr_loc == (self.stop - 1) * LOT_LENGTH + CAR_LENGTH 
			else:
				assert self.curr_loc == self.stop * LOT_LENGTH
			return

		cp = self.traj.head

		if cp.nex == None:
			if not np.abs(cp.data.t - self.sys.curr) <= 1e-05:
				import pdb; pdb.set_trace()
			if not cp.data.x == self.curr_loc:
				if np.abs(cp.data.x - self.curr_loc) <= SMALL_INTERVAL:
					self.curr_loc = cp.data.x
				else:
					import pdb; pdb.set_trace()
			return 

		if cp.data.t > self.sys.curr:
			print ('Line 474: current time not accounted for in the trajectory!!!')
			import pdb; pdb.set_trace()

		if self.end_time < self.sys.curr - SMALL_INTERVAL:
			print ('Line 478: self.end_time, self.sys.curr', self.end_time, self.sys.curr)
			import pdb; pdb.set_trace()

		while cp.nex.data.t <= self.sys.curr:
			cp = cp.nex
			if cp.nex == None:
				if np.abs(self.end_time - self.sys.curr) > SMALL_INTERVAL:
					print ('Line 486: it should be small', np.abs(self.end_time - self.sys.curr))
					import pdb; pdb.set_trace()
				self.curr_loc = cp.data.x
				return

		assert cp.data.t <= self.sys.curr < cp.nex.data.t
		assert cp.data.v != 'D'
		self.curr_loc = cp.data.x + cp.data.v * (self.sys.curr - cp.data.t)
		if self.curr_loc > self.dest_to_stop:
			if not (self.curr_loc - self.dest_to_stop < 1e-05):
				import pdb; pdb.set_trace()
			self.curr_loc = self.dest_to_stop
		return

	def update_traj(self):

		traj = self.traj
		end_time = self.end_time
		assert self.status in [1, 2, 5, 6]

		self.traj = DLinkedList()

		if self.status == 2:
			# i.e. the vehicle has started pulling in but has not started service
			if spmatch and spot2blk(self.stop) == 1:
				assert self.curr_loc == LOT_LENGTH
			else:
				assert (self.curr_loc == self.dest_to_stop)
			if self.plin_end > self.sys.curr:
				self.end_time = self.plin_end
			else:
				assert self.end_time == self.plin_end
			if end_time is not None:
				assert self.end_time >= end_time
			assert self.end_time >= self.sys.curr
			self.traj.addEnd( changePoint(self.curr_loc, self.sys.curr, 0.0) )
			self.traj.addEnd( changePoint(self.curr_loc, self.end_time, 'D') )
			return

		if self.curr_loc == self.dest_to_stop and self.status == 6:
			assert self.end_time is not None
			assert (np.abs(self.end_time - self.sys.curr) < SMALL_INTERVAL)
			if end_time is not None:
				assert self.end_time >= end_time
			assert self.end_time >= self.sys.curr
			self.traj.addEnd( changePoint(self.curr_loc, self.sys.curr, 0.0))
			self.traj.addEnd( changePoint(self.curr_loc, self.end_time, 'D') )
			return

		if self.curr_loc == self.dest_to_stop and self.status == 1:
			if self.end_time is None:
				assert (self.stop == 1) or (spmatch and self.stop == 2)
				self.end_time = self.sys.curr
			if end_time is not None:
				assert self.end_time >= end_time
			if not self.end_time >= self.sys.curr:
				if not np.abs(self.end_time - self.sys.curr) < SMALL_INTERVAL:
					import pdb; pdb.set_trace()
				self.end_time = self.sys.curr
			self.traj.addEnd( changePoint(self.curr_loc, self.sys.curr, 0.0))
			self.traj.addEnd( changePoint(self.curr_loc, self.end_time, 'D') )
			return

		start_t = self.sys.curr

		if self.status == 5:
			# i.e. the vehicle has started pulling out
			self.update_loc()
			assert self.traj.head == None
			if self.pout_end > self.sys.curr:
				self.traj.addEnd( changePoint(self.curr_loc, self.sys.curr, 0.0) )
				start_t = self.pout_end
			
		if self.prev == None:
			try:
				assert self.sys.head == self
			except:
				print ('Line 541: this car does not have a prev while self.sys.head is not itself!!!')
				import pdb; pdb.set_trace()

			self.traj.addEnd( changePoint(self.curr_loc, start_t, self.driv) )
			self.end_time = start_t + (self.dest_to_stop - self.curr_loc) / self.driv
			if (end_time is not None) and (self.end_time < end_time):
				assert (self.end_time >= end_time - SMALL_INTERVAL)
				self.end_time = end_time
			self.traj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D') )
			return

		if self.prev.end_time <= start_t + 10 * SMALL_INTERVAL:
			# e.g. if prev finishes the exit maneuver before self
			self.traj.addEnd( changePoint(self.curr_loc, start_t, self.driv) )
			self.end_time = start_t + (self.dest_to_stop - self.curr_loc) / self.driv
			if (end_time is not None) and (self.end_time < end_time):
				if not (self.end_time >= end_time - 9 * SMALL_INTERVAL):
					import pdb; pdb.set_trace()
				self.end_time = end_time
			self.traj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D') )
			return

		# if self.prev.status == 2 and self.dest_to_stop >= self.prev.curr_loc:
		# 	# assert simType == 'cav'
		# 	if self.curr_loc < self.prev.curr_loc - CAR_LENGTH:
		# 		self.traj.addEnd( changePoint(self.curr_loc, start_t, self.driv) )
		# 		car_time = start_t + (self.prev.curr_loc - CAR_LENGTH - self.curr_loc) / self.driv
		# 		if car_time < self.prev.plin_end - meanDRIV:
		# 			self.traj.addEnd( changePoint(self.prev.curr_loc - CAR_LENGTH, car_time, 0.0) )
		# 			self.traj.addEnd( changePoint(self.prev.curr_loc - CAR_LENGTH, self.prev.plin_end - meanDRIV, self.driv))
		# 			self.end_time = self.prev.plin_end - meanDRIV + (self.dest_to_stop - self.prev.curr_loc + CAR_LENGTH) / self.driv
		# 		else:
		# 			self.end_time = start_t + (self.dest_to_stop - self.curr_loc) / self.driv
		# 		self.traj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D'))
		# 	else:
		# 		assert self.curr_loc < self.prev.stop * LOT_LENGTH
		# 		assert traj is not None
		# 		import pdb; pdb.set_trace()
		# 		self.traj = traj				
		# 	return					

		#####################################################################################
		cp = self.prev.traj.head
		if cp.nex is None:
			import pdb; pdb.set_trace()

		# the assertion above should be fine since cp.nex == None iff either one of the following holds:
		# i) the vehicle is pulling in and thus self.getin is True
		# ii) the vehicle is the head onlane thus self.prev is None
		# it implies that cp.data.v != 'D'
		assert (cp.data.t <= start_t)
		while (cp.nex is not None) and (cp.nex.data.t <= start_t):
			assert cp.data.v != 'D'
			cp = cp.nex

		# the while-loop cannot end with (cp.nex is None)
		# remember that self.prev should already has an accurate trajectory w. a valid end_time
		# if (cp.nex is None), then (self.prev.end_time == cp.nex.data.t)
		# since (self.prev.end_time <= start_t) is already accounted for
		# it is only possible to have (cp.nex.data.t > start_t) and also (cp.nex.data.v = 'D')
		if (cp.nex is None):
			print ('line 575: testing if an option is possible')
			import pdb; pdb.set_trace()
			assert (self.prev.end_time == cp.nex.data.t > start_t)
			assert cp.data.v == 'D'
			# ??? assert self.prev.status == 5
			self.traj.addEnd( changePoint(self.curr_loc, start_t, self.driv) )
			self.end_time = start_t + (self.dest_to_stop - self.curr_loc) / self.driv
			if end_time is not None:
				assert self.end_time >= end_time
			self.traj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D') )
			return
		
		# some sanity check before proceeding
		assert (cp.data.v != 'D') and (cp.nex != None)
		assert (cp.data.t <= start_t < cp.nex.data.t)

		# let us define some counters to keep track of time and location (in the projected traj)
		iter_t = start_t
		if (start_t == self.sys.curr):
			self.prev.update_loc()
			prev_loc = self.prev.curr_loc
		else:
			prev_loc = cp.data.x + (start_t - cp.data.t) * cp.data.v

		# the RHS below is the location of prev at start_t
		# check that the distance between the prev and the curr at start_t is at least CAR_LENGTH
		if (self.status != 5) and np.abs(self.curr_loc + CAR_LENGTH - prev_loc) < 1e-5:
			self.curr_loc = max( 0.0, prev_loc - CAR_LENGTH )
			if (prev_loc < CAR_LENGTH):
				self.traj.addEnd( changePoint(0.0, start_t, 0.0) )
				start_t = self.prev.calc_time(CAR_LENGTH + SMALL_INTERVAL)
				assert start_t > self.sys.curr
		start_x = self.curr_loc
		iter_x = self.curr_loc

		# check that the distance between the prev and the curr at start_t is at least CAR_LENGTH
		if self.curr_loc + CAR_LENGTH > prev_loc:
			import pdb; pdb.set_trace()

		# if the distance == CAR_LENGTH AND cp.data.v == self.driv
		# the curr has to travel at cp.data.v == self.driv for some time
		# i.e. the curr already catches up with the its prev
		#      and the curr has a free-flow rate == cp.data.v s.t the headway distance is maintained 
		if (start_x + CAR_LENGTH == prev_loc) and (cp.data.v == self.driv):
			
			while (cp.nex is not None):
				cp = cp.nex
				if (cp.data.v == 'D') or (cp.data.x >= self.dest_to_stop + CAR_LENGTH):
					self.traj.addEnd( changePoint(start_x, start_t, self.driv) )
					self.end_time = start_t + (self.dest_to_stop - start_x) / self.driv
					if (end_time is not None) and (self.end_time < end_time):
						if not (self.end_time >= end_time - 4 * SMALL_INTERVAL):
							import pdb; pdb.set_trace()
						self.end_time = end_time
					self.traj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D'))
					return
				elif (cp.data.v < self.driv):
					self.traj.addEnd( changePoint(start_x, start_t, self.driv) )
					break
				elif (cp.data.v > self.driv):
					break
				else:
					assert (cp.data.v == self.driv)

			assert (cp is not None) and (cp.data.v != self.driv)
			if not (start_x + (cp.data.t - start_t) * self.driv + CAR_LENGTH == cp.data.x):
				if np.abs(start_x + (cp.data.t - start_t) * self.driv + CAR_LENGTH - cp.data.x) > 2 * SMALL_INTERVAL:
					import pdb; pdb.set_trace()
			iter_t = cp.data.t
			prev_loc = cp.data.x
			iter_x = prev_loc - CAR_LENGTH
		
		while True:

			# if the distance == CAR_LENGTH AND cp.data.v < self.driv
			# the curr has to travel at cp.data.v <= self.driv for some time
			# i.e. the curr already catches up with the its prev
			#      and the curr has a free-flow rate geq cp.data.v s.t the headway distance is maintained 
			# the curr starts to travel at cp.data.v (not constant, but always the same as the prev)

			if (iter_x + CAR_LENGTH == prev_loc) and (cp.data.v < self.driv):

				while (cp.nex is not None):

					assert cp.data.v != 'D' 
					if not (iter_x + CAR_LENGTH == cp.data.x + (iter_t - cp.data.t) * cp.data.v):
						import pdb; pdb.set_trace()

					if cp.data.v <= self.driv:

						if cp.data.v == 0 and np.abs(iter_x - self.dest_to_stop) < SMALL_INTERVAL:
							self.end_time = iter_t
							if (end_time is not None) and (self.end_time < end_time):
								if not (self.end_time >= end_time - 10 * SMALL_INTERVAL):
									import pdb; pdb.set_trace()
								self.end_time = end_time						
							self.traj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D') )
							return
						else: 
							self.traj.addEnd( changePoint(iter_x, iter_t, cp.data.v) )

						if cp.nex.data.x - CAR_LENGTH >= self.dest_to_stop:
							if not (cp.data.v > 0.0):
								if ( np.abs(cp.nex.data.x - cp.data.x) < SMALL_INTERVAL ):
									self.end_time = cp.nex.data.t
									if (end_time is not None) and (self.end_time < end_time):
										if not (self.end_time >= end_time - SMALL_INTERVAL):
											import pdb; pdb.set_trace()
										self.end_time = end_time
									self.traj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D'))
								else:
									import pdb; pdb.set_trace()
							else:
								self.end_time = iter_t + (self.dest_to_stop - iter_x) / cp.data.v
								if (end_time is not None) and (self.end_time < end_time):
									if not (self.end_time >= end_time - 46 * SMALL_INTERVAL):
										import pdb; pdb.set_trace()
									self.end_time = end_time
								self.traj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D'))
							return

						if np.abs(iter_x + (cp.nex.data.t - iter_t) * cp.data.v + CAR_LENGTH - cp.nex.data.x) > 4e-05:
							import pdb; pdb.set_trace()
						cp = cp.nex 
						iter_x = cp.data.x - CAR_LENGTH
						iter_t = cp.data.t
					
					else:
						assert cp.data.v > self.driv
						break	

				if cp.nex is None:
					assert cp.data.v == 'D'
					assert (iter_x + CAR_LENGTH == cp.data.x)
					self.traj.addEnd( changePoint(iter_x, iter_t, self.driv) )
					self.end_time = iter_t + (self.dest_to_stop - iter_x) / self.driv
					if (end_time is not None) and (self.end_time < end_time):
						if not (self.end_time >= end_time - 2 * SMALL_INTERVAL):
							import pdb; pdb.set_trace()
						self.end_time = end_time
					self.traj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D'))
					return

				assert cp.nex is not None
				assert (cp.data.v != 'D') and (cp.data.v > self.driv)
				assert (iter_x + CAR_LENGTH == cp.data.x)
				assert (iter_x < self.dest_to_stop)
				start_t = iter_t
				start_x = iter_x
				prev_loc = cp.data.x

			# if the distance is > CAR_LENGTH 
			# OR if the distance == CAR_LENGTH while self.driv < cp.data.v
			# the curr can drive at its own free-flow rate for some time
			if not ( (iter_x + CAR_LENGTH < prev_loc) | (cp.data.v > self.driv) ):
				import pdb; pdb.set_trace()
			assert (iter_x + CAR_LENGTH <= prev_loc)

			# the curr starts to drive at self.driv
			self.traj.addEnd( changePoint(start_x, start_t, self.driv) )

			while (cp.nex is not None):

				if (iter_x >= self.dest_to_stop - SMALL_INTERVAL):
					break

				# as long as self.driv <= cp.data.v,
				# the curr continues to travel at self.driv
				# the inequality is not strict 
				# because the distance between two vehicles (> CAR_LENGTH) will be maintained if both travels at the same rate
				if (self.driv <= cp.data.v):

					if (self.driv == cp.data.v) and (iter_x + CAR_LENGTH == prev_loc):
						cp = cp.nex
						prev_loc = cp.data.x
						iter_x = prev_loc - CAR_LENGTH
						iter_t = cp.data.t
						if (cp.data.v != 'D') and (cp.data.v < self.driv):
							break
					else:
						cp = cp.nex
						prev_loc = cp.data.x
						iter_x = start_x + (cp.data.t - start_t) * self.driv
						iter_t = cp.data.t
						if not (iter_x + CAR_LENGTH <= prev_loc):
							if np.abs(iter_x + CAR_LENGTH - prev_loc) < 1e-05:
								iter_x = prev_loc - CAR_LENGTH
							else:
								import pdb; pdb.set_trace()
					continue

				assert (cp.data.v < self.driv)
				if (iter_x + CAR_LENGTH == prev_loc):
					break

				if np.abs(iter_x + (cp.nex.data.t - iter_t) * self.driv + CAR_LENGTH - cp.nex.data.x) < SMALL_INTERVAL:
					cp = cp.nex
					iter_x = cp.data.x - CAR_LENGTH
					iter_t = cp.data.t
					prev_loc = cp.data.x
					if (cp.data.v == 'D') or (cp.data.v < self.driv):
						break

				elif (iter_x + (cp.nex.data.t - iter_t) * self.driv + CAR_LENGTH < cp.nex.data.x):
					cp = cp.nex
					iter_x = start_x + (cp.data.t - start_t) * self.driv
					iter_t = cp.data.t
					prev_loc = cp.data.x
					assert (iter_x + CAR_LENGTH < cp.data.x)

				else:
					change_t = (prev_loc - CAR_LENGTH - iter_x) / (self.driv - cp.data.v)
					if not (0.0 < change_t < cp.nex.data.t - cp.data.t):
						import pdb; pdb.set_trace()
					if cp.data.v == 0.0:
						iter_x = prev_loc - CAR_LENGTH
						iter_t += change_t
					else:
						iter_t += change_t
						prev_loc = cp.data.x + (iter_t - cp.data.t) * cp.data.v
						iter_x  = prev_loc - CAR_LENGTH
						
					break

			# if the prev ends before the curr catches up with the prev
			# i.e. the curr can travel at self.driv until its own destination (from the info available up to now)
			# OR if the curr catches up with the prev after the destination of the curr
			if (cp.nex is None) | (iter_x >= self.dest_to_stop - SMALL_INTERVAL): 
				self.end_time = start_t + (self.dest_to_stop - start_x) / self.driv
				if (end_time is not None) and (self.end_time < end_time):
					if not (self.end_time >= end_time - 48 * SMALL_INTERVAL):
						import pdb; pdb.set_trace()
					self.end_time = end_time
				self.traj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D'))
				return

			assert (cp.data.v != 'D')
			assert (cp.data.v < self.driv)
			start_x = iter_x
			start_t = iter_t
			if not (prev_loc == start_x + CAR_LENGTH):
				import pdb; pdb.set_trace()

		return
