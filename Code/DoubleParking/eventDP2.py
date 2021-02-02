import sys
from math import ceil, floor, sqrt
import numpy as np
from heapq import *
from utils import *
from params import *

VEHICLE_IDX = None

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

		elif simType == 'unif':
			self.timeSERV = ParamGen(Unif2(meanSERV, seed = seedSERV))
			self.timeDRIV = ParamGen(Unif2(rateDRIV, seed = seedDRIV))
			self.timePOUT = ParamGen(Unif2(meanPOUT, seed = seedPOUT))
			self.timePLIN = ParamGen(Unif2(meanPLIN, seed = seedPLIN))

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

		self.N = N
		self.n = N * LOT_LENGTH 
		self.curr = 0.0
		self.start_time = 0.0
		self.eventheap = []
		self.inservice = [ [None for _ in range(self.N)] , [None for _ in range(self.N)] ]
		self.waiting = []
		self.head = None
		self.subhead = None
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

		############ alternative 2 specific ############
		self.reassigned = set()
		################################################

	def add_event(self, event):
		heappush( self.eventheap, event) 

	def debug_print(self):
		
		print ('Curb side lane ...')
		for stop in range(1, self.N + 1):
			if self.inservice[0][stop - 1] is not None:
				car = self.inservice[0][stop - 1]
				print ('Vehicle %s is assigned the spot %s and its current status is %s.' %(car.idx, car.stop, car.status) )

		print ('Middle lane ... Part 1')
		for stop in range(1, self.N + 1):
			if self.inservice[1][stop - 1] is not None:
				car = self.inservice[1][stop - 1]
				print ('Vehicle %s is assigned the spot %s and its current status is %s.' %(car.idx, car.stop, car.status) )

		print ('Middle lane ... Part 2')
		car = self.subhead
		while car != None:
			if car.status == 3 or car.status == 4.5:
				print ('Vehicle %s is assigned the spot %s and its current status is %s.' %(car.idx, car.stop, car.status) )

			else:
				car.update_loc()
				print ('Vehicle %s is assigned the spot %s; its current status is %s and its current location is %s.' %(car.idx, car.stop, car.status, car.curr_loc) )
			car = car.subnex

		print ('Through lane ...')
		car = self.head
		while car != None:
			car.update_loc()
			print ('Vehicle %s is assigned the spot %s; its current status is %s and its current location is %s.' %(car.idx, car.stop, car.status, car.curr_loc) )
			car = car.nex

	def idle_time_calc(self, curr_vehicle):
		
		total = self.curr - curr_vehicle.enter_time
		if (total < curr_vehicle.prod_time) and np.abs(total - curr_vehicle.prod_time) > 3 * SMALL_INTERVAL:
			import pdb; pdb.set_trace()
		idle = max(0.0, total - curr_vehicle.prod_time)
		if idle > self.max_idle:
			self.max_idle = idle
			self.max_idle_idx = curr_vehicle.idx
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
			curr_typ = curr_event.typ
			self.curr_vehicle = curr_vehicle
			self.curr = float(curr_event.time)
			self.curr_typ = curr_typ
			try:
				stop = curr_vehicle.stop
			except AttributeError:
				assert curr_typ == 'enter_system'

			if VEHICLE_IDX is not None and curr_vehicle.idx == VEHICLE_IDX:
				import pdb; pdb.set_trace()

			################################### update system ###################################
			if curr_typ == 'leave_system':
				self.leave_system(curr_vehicle)

			elif curr_typ == 'start_pulling_in':
				# if curr_vehicle.idx == VEHICLE_IDX:
				# 	import pdb; pdb.set_trace()
				self.start_pulling_in(curr_vehicle)
				# if curr_vehicle.idx == VEHICLE_IDX:
				# 	import pdb; pdb.set_trace()

			elif curr_typ == 'start_second_enter':
				self.start_second_enter(curr_vehicle)

			elif curr_typ == 'start_service':
				# if curr_vehicle.idx == VEHICLE_IDX:
				# 	import pdb; pdb.set_trace()
				self.start_service(curr_vehicle)
				# if curr_vehicle.idx == VEHICLE_IDX:
				# 	import pdb; pdb.set_trace()

			elif curr_typ == 'prepare_pulling_out':
				# if curr_vehicle.idx == VEHICLE_IDX:
				#  	import pdb; pdb.set_trace()
				self.prepare_pulling_out(curr_vehicle)
				# if curr_vehicle.idx == VEHICLE_IDX:
				#  	import pdb; pdb.set_trace()

			elif curr_typ == 'prepare_first_exit':
				# if curr_vehicle.idx == VEHICLE_IDX:
				#  	import pdb; pdb.set_trace()
				self.prepare_first_exit(curr_vehicle)
				# if curr_vehicle.idx == VEHICLE_IDX:
				#  	import pdb; pdb.set_trace()

			elif curr_typ == 'finish_pulling_out':
				# if curr_vehicle.idx == VEHICLE_IDX:
				# 	import pdb; pdb.set_trace()
				self.finish_pulling_out(curr_vehicle)
				# if curr_vehicle.idx == VEHICLE_IDX:
				# 	import pdb; pdb.set_trace()

			else:
				assert curr_typ == 'enter_system'
				# self.enter_system(curr_vehicle)
				self.enter_system(curr_vehicle, debug_idx = VEHICLE_IDX)

			if VEHICLE_IDX is not None and curr_vehicle.idx == VEHICLE_IDX:
				import pdb; pdb.set_trace()

			if self.debug:
				self.debug_print()
				import pdb; pdb.set_trace()

		self.start_time = self.curr
		return (self.outCount)

	def leave_system(self, curr_vehicle):

		curr_vehicle.update_loc()

		if np.abs( curr_vehicle.curr_loc - curr_vehicle.dest_to_stop ) > 1e-5:
			if curr_vehicle.end_time <= self.curr + SMALL_INTERVAL:
				import pdb; pdb.set_trace()
			self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'leave_system') )
			return

		assert curr_vehicle.subprev is None
		assert curr_vehicle.prev is None

		if self.subhead == curr_vehicle:
			assert curr_vehicle.nex is None
			self.subhead = curr_vehicle.subnex
			if curr_vehicle.subnex is not None:
				curr_vehicle.subnex.subprev = None
				curr_vehicle.subnex = None
		else:
			assert self.head == curr_vehicle
			assert curr_vehicle.subnex is None			
			self.head = curr_vehicle.nex
			if curr_vehicle.nex != None:
				curr_vehicle.nex.prev = None
				curr_vehicle.nex = None

		self.outCount += 1
		curr_vehicle.prod_time += ( (curr_vehicle.dest_to_stop - curr_vehicle.stop * LOT_LENGTH) / curr_vehicle.driv )
		self.idle_time_calc(curr_vehicle)
		return

	def start_pulling_in(self, curr_vehicle):

		curr_vehicle.update_loc()

		# if this event has been delayed s.t. the vehicle has not traveled to the desired destination by now
		if np.abs( curr_vehicle.curr_loc - curr_vehicle.dest_to_stop ) > 1e-5:
			if curr_vehicle.end_time <= self.curr + SMALL_INTERVAL:
				import pdb; pdb.set_trace()
			self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_pulling_in') )
			return

		curr_vehicle.curr_loc = curr_vehicle.dest_to_stop
		stop = curr_vehicle.stop
		delayed = False
		req_time = self.curr

		# if the vehicle has arrived at the desired destination while its enter maneuver blocked by vehicles on the through lane
		# i.e. 1) if its immediate proceeding vehicle is having an exit maneuver at the same spot from the middle lane to the through lane;
		if (curr_vehicle.prev is not None) and (curr_vehicle.prev.stop == stop) and (curr_vehicle.prev.status == 5):
			assert curr_vehicle.prev.pout_end > self.curr
			delayed = True 
			req_time = max( req_time, curr_vehicle.prev.pout_end )

		# or 2) if the immediate proceeding vehicle is stopped.
		if (curr_vehicle.prev is not None) and (curr_vehicle.prev.status != 1.5) and (curr_vehicle.prev.curr_loc <= curr_vehicle.dest_to_stop + 1.5 * CAR_LENGTH):
			assert curr_vehicle.prev.status != 2
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
			car = None
			return

		# Now let us consider the vehicles on the middle lane.
		# First, if the middle lane is empty, curr_vehicle can start its enter maneuver right now.
		if self.subhead is None:
			assert not delayed
			assert self.inservice[1][stop - 1] is None
			# not applicable for alternative 2
			# assert curr_vehicle.lane is None
			self.subhead = curr_vehicle
			curr_vehicle.start_in()
			car = curr_vehicle.nex
			while car != None:
				car.update_loc()
				car.update_traj()
				car = car.nex
			car = None
			
			self.inservice[1][stop - 1] = curr_vehicle
			assert curr_vehicle.end_time is not None

			# if self.inservice[0][stop - 1] is None:
			# 	curr_vehicle.lane = 0
			# 	self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_second_enter'))
			# else:
			# 	curr_vehicle.lane = 1
			# 	self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_service') )
			# return

			############ alternative 2 specific ############
			if self.inservice[0][stop - 1] is None:
				if curr_vehicle.lane == 1:
					new_stop_idx = curr_vehicle.stop_idx + self.N
					if new_stop_idx in self.waiting:
						self.waiting.remove(new_stop_idx)
						heappush( self.waiting, curr_vehicle.stop_idx )
					else:
						self.reassigned.add(curr_vehicle.stop)
					curr_vehicle.lane = 0
					curr_vehicle.stop_idx = new_stop_idx
				self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_second_enter'))
			else:
				if not (curr_vehicle.lane == 1):
					assert (curr_vehicle.lane == 0 and stop in self.reassigned)
					self.reassigned.remove(stop)
					curr_vehicle.lane = 1
					curr_vehicle.stop_idx = stop
				self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_service') )
			return
			################################################

		# Otherwise it is possible that the vehicles on the second lane would block the current vehicle from its enter maneuver
		# 1) if a vehicle has an enter or exit maneuver between the middle lane to the curb side lane at the same spot
		if self.inservice[0][stop - 1] is not None:
			car = self.inservice[0][stop - 1]
			if car.status == 2:
				assert car.plin_end > self.curr 
				delayed = True
				req_time = max( req_time, car.plin_end )
			elif car.status == 4.25:
				assert car.pout_end > self.curr
				delayed = True
				req_time = max( req_time, car.pout_end )
			else:
				assert car.status in [3, 4]

		# # 2) if a vehicle has an exit maneuver between the middle lane to the curb side lane at the spot behind
		# if (stop >= 2) and (self.inservice[0][stop - 2] is not None) and (self.inservice[0][stop - 2].status == 4.25):
		# 	car = self.inservice[0][stop - 2]
		# 	assert car.pout_end > self.curr
		# 	delayed = True
		# 	req_time = max( req_time, car.pout_end )

		# 3) the spot on the middle lane should not have a vehicle in service
		if self.inservice[1][stop - 1] is not None and self.inservice[1][stop - 1].status == 4.5:
			assert  self.inservice[1][stop - 1].end_time > self.curr
			delayed = True
			req_time = max( req_time,  self.inservice[1][stop - 1].end_time )

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
			car = None
			return

		# 4) if a vehicle is traveling on the middle lane and is projected to block the current vehicle's enter maneuver
		#    a vehicle already moving on the middle lane can have a status of 1 or 6.
		#    the scenario of 'blocking the current vehicle' is defined such that,
		#    a) if the upcoming vehicle has a destination spot with a larger or same index (larger means further away from the entrance);
		#    b) AND the upcoming vehicle 
		#    we calculate the estimated time that this upcoming vehicle will arrive  

		car = self.subhead
		prev = None

		while car != None:

			assert car.status != 4

			if car.status not in [3, 4.5]:
				car.update_loc()
	
			if car.curr_loc >= stop * LOT_LENGTH + CAR_LENGTH - SMALL_INTERVAL:
				prev = car

			elif (car.status == 1.5) and (car.stop == stop + 1):
				prev = car

			# changes added on Jan 22, 2021
			elif (car.status == 4.25) and (car.stop == stop + 1):
				pass
			# end changes

			elif car.status in [3, 4.5]:
				assert car.stop < stop
				break

			# elif car.curr_loc >= max( 0, stop * LOT_LENGTH - CAR_LENGTH - PLIN_DIST):
			# 	assert (car.stop == stop + 1 and car.status == 4.25) or (car.stop < stop) or (car.status in [1, 6])

			# 	if car.status in [1.5, 2, 4.25, 5]:
			# 		pass

			# 	else:
			# 		assert car.status in [1, 6]
			# 		car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH, sub = True )
			# 		assert car_time > self.curr
			# 		delayed = True
			# 		req_time = max( req_time, car_time )

			# else:
			# 	break

			# changes made on Jan 22, 2021
			elif car.status in [1.5, 2, 4.25]:
				assert car.stop < stop
				break

			else:
				if not ( (car.status == 5 and car.stop < stop) or (car.status in [1, 6]) ):
					import pdb; pdb.set_trace()

				if (car.status == 1 and car.stop < stop):
					# pass
					break

				elif car.status == 5:
					assert car.stop < stop
					break
					# car_time = max(0, meanPOUT - (self.curr - car.pout_start)) + (stop - 1 - car.stop) * LOT_LENGTH / rateDRIV
					# if car_time < meanPLIN:
					# 	delayed = True
					# 	car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH, sub = True )
					# 	assert car_time > self.curr
					# 	req_time = max( req_time, car_time )

				else:
					assert car.status in [1, 6]
					car_time = ((stop - 1) * LOT_LENGTH - car.curr_loc) / rateDRIV
					if car_time < meanPLIN:
						delayed = True
						if car.status == 1 and car.stop == stop:
							assert self.inservice[0][stop - 1] is None
							car_time = car.end_time
						else:
							car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH, sub = True )
						assert car_time > self.curr
						req_time = max( req_time, car_time )
			# end changes

			car = car.subnex

		if req_time > self.curr:
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
			car = None
			return
	
		# Now the vehicle can start the first step of enter maneuver
		assert curr_vehicle.subprev is None
		assert curr_vehicle.subnex is None
	
		if prev is None:	
			curr_vehicle.subnex = self.subhead
			if self.subhead is not None:
				curr_vehicle.subnex.subprev = curr_vehicle
			self.subhead = curr_vehicle
		else:
			curr_vehicle.subprev = prev
			curr_vehicle.subnex = prev.subnex
			if prev.subnex is not None:
				curr_vehicle.subnex.subprev = curr_vehicle
			prev.subnex = curr_vehicle

		assert self.inservice[1][stop - 1] is None
		self.inservice[1][stop - 1] = curr_vehicle
		curr_vehicle.start_in()

		car = curr_vehicle.nex
		while car != None:
			car.update_loc()
			car.update_traj()
			car = car.nex

		car = curr_vehicle.subnex
		while car != None and car.status not in [3, 4.5]:
			car.update_loc()
			car.update_subtraj()
			car = car.subnex

		assert curr_vehicle.end_time is not None

		# if self.inservice[0][stop - 1] is None:
		# 	curr_vehicle.lane = 0
		# 	self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_second_enter'))
		# else:
		# 	curr_vehicle.lane = 1
		# 	self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_service') )
		# return

		############ alternative 2 specific ############
		if self.inservice[0][stop - 1] is None:
			if curr_vehicle.lane == 1:
				new_stop_idx = curr_vehicle.stop_idx + self.N
				if new_stop_idx in self.waiting:
					self.waiting.remove(new_stop_idx)
					heappush( self.waiting, curr_vehicle.stop_idx )
				else:
					self.reassigned.add(curr_vehicle.stop)
				curr_vehicle.lane = 0
				curr_vehicle.stop_idx = new_stop_idx
			self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_second_enter'))
		else:
			if not (curr_vehicle.lane == 1):
				assert (curr_vehicle.lane == 0 and stop in self.reassigned)
				self.reassigned.remove(stop)
				curr_vehicle.lane = 1
				curr_vehicle.stop_idx = stop
			self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_service') )
		return
		################################################

	def start_second_enter(self, curr_vehicle):

		stop = curr_vehicle.stop

		# if curr_vehicle entered the system through the middle lane s.t. it has not been assigned a lane to stop
		if curr_vehicle.status == 1:
			# assert curr_vehicle.entry == 1 and curr_vehicle.lane is None
			# the latter is not applicable to alternative 2
			assert curr_vehicle.entry == 1
			assert (curr_vehicle.subprev is not None) or (self.subhead == curr_vehicle)
			curr_vehicle.update_loc()

			# if this event has been delayed s.t. the vehicle has not traveled to the desired destination by now
			if np.abs( curr_vehicle.curr_loc - curr_vehicle.dest_to_stop ) > 1e-5:
				if curr_vehicle.end_time <= self.curr + SMALL_INTERVAL:
					import pdb; pdb.set_trace()
				self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_second_enter') )
				return

			curr_vehicle.curr_loc = curr_vehicle.dest_to_stop
			assert self.inservice[1][stop - 1] is None
			self.inservice[1][stop - 1] = curr_vehicle

			# if self.inservice[0][stop - 1] is not None:
			# 	curr_vehicle.lane = 1
			# 	curr_vehicle.status = 1.5
			# 	self.add_event( event(self.curr, curr_vehicle, 'start_service') )
			# 	return
			# else:
			# 	curr_vehicle.lane = 0

		############ alternative 2 specific ############
		if self.inservice[0][stop - 1] is not None:
			if not (curr_vehicle.lane == 1):
				assert (curr_vehicle.lane == 0 and stop in self.reassigned)
				self.reassigned.remove(stop)
				curr_vehicle.lane = 1
				curr_vehicle.stop_idx = stop
			curr_vehicle.status = 1.5
			self.add_event( event(self.curr, curr_vehicle, 'start_service') )
			return

		if self.inservice[0][stop - 1] is None and curr_vehicle.lane == 1:
			new_stop_idx = curr_vehicle.stop_idx + self.N
			if new_stop_idx in self.waiting:
				self.waiting.remove(new_stop_idx)
				heappush( self.waiting, curr_vehicle.stop_idx )
			else:
				self.reassigned.add(curr_vehicle.stop)
			curr_vehicle.lane = 0
			curr_vehicle.stop_idx = new_stop_idx
		################################################

		curr_vehicle.curr_loc = stop * LOT_LENGTH
		curr_vehicle.dest_to_stop = stop * LOT_LENGTH
		
		# self.inservice[0] and self.inservice[1] include all vehicles that are entering, serving, exiting or waiting to exit
		assert curr_vehicle.lane == 0
		assert self.inservice[0][stop - 1] is None
		assert self.inservice[1][stop - 1] == curr_vehicle
		self.inservice[0][stop - 1] = curr_vehicle
		self.inservice[1][stop - 1] = None

		curr_vehicle.second_in()
		
		# remove curr_vehicle from the through lane
		if curr_vehicle.prev is not None:
			curr_vehicle.prev.nex = curr_vehicle.nex
		elif self.head == curr_vehicle:
			self.head = curr_vehicle.nex
		else:
			assert curr_vehicle.entry == 1

		if curr_vehicle.nex != None:
			curr_vehicle.nex.prev = curr_vehicle.prev
			car = curr_vehicle.nex
			while car != None:		
				car.update_loc()
				car.update_traj()
				car = car.nex
			car = None

		curr_vehicle.prev = None
		curr_vehicle.nex = None

		# update the trajectories of the following vehicles on the middle lane
		car = curr_vehicle.subnex
		while car != None and car.status not in [3, 4.5]:
			car.update_loc()
			car.update_subtraj()
			car = car.subnex
		car = None

		assert (curr_vehicle.end_time == curr_vehicle.plin_end != None)
		self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'start_service') )
		return		

	def start_service(self, curr_vehicle):

		assert curr_vehicle.lane is not None
		assert curr_vehicle.entry is not None
		curr_vehicle.start_service()

		if curr_vehicle.entry == 1:
			curr_vehicle.prod_time += (curr_vehicle.stop * LOT_LENGTH / curr_vehicle.driv)
		else:
			assert curr_vehicle.entry == 2
			curr_vehicle.prod_time += ((curr_vehicle.stop - 1) * LOT_LENGTH / curr_vehicle.driv)

		if curr_vehicle.lane == 0:
			assert curr_vehicle.prev is None
			assert curr_vehicle.nex is None		
			if curr_vehicle.subprev != None:
				curr_vehicle.subprev.subnex = curr_vehicle.subnex
			else:
				assert self.subhead == curr_vehicle
				self.subhead = curr_vehicle.subnex

			if curr_vehicle.subnex != None:
				car = curr_vehicle.subnex
				car.subprev = curr_vehicle.subprev
				while car != None and car.status not in [3, 4.5]:	
					car.update_loc()
					car.update_subtraj()
					car = car.subnex
				car = None 

			curr_vehicle.subprev = None
			curr_vehicle.subnex = None

			assert curr_vehicle.serv_end > self.curr
			self.add_event( event(curr_vehicle.serv_end, curr_vehicle, 'prepare_first_exit') )
			
		else:
			assert curr_vehicle.lane == 1
			if curr_vehicle.prev != None:
				curr_vehicle.prev.nex = curr_vehicle.nex
			elif self.head == curr_vehicle:
				self.head = curr_vehicle.nex
			else:
				assert curr_vehicle.entry == 1

			if curr_vehicle.nex != None:
				car = curr_vehicle.nex
				car.prev = curr_vehicle.prev				
				while car != None:		
					car.update_loc()
					car.update_traj()
					car = car.nex
				car = None

			curr_vehicle.prev = None
			curr_vehicle.nex = None

			car = curr_vehicle.subnex
			while car != None and car.status not in [3, 4.5]:
				car.update_loc()
				car.update_subtraj()
				car = car.subnex
			car = None

			assert curr_vehicle.serv_end > self.curr
			self.add_event( event(curr_vehicle.serv_end, curr_vehicle, 'prepare_pulling_out') )

		return

	def prepare_first_exit(self, curr_vehicle):

		stop = curr_vehicle.stop
		assert curr_vehicle.lane == 0
		assert self.inservice[0][stop - 1] == curr_vehicle

		if curr_vehicle.status == 3:
			curr_vehicle.status = 4

		# 1) check if any vehicle is occupying the spot beside the current one
		if self.inservice[1][stop - 1] is not None:
			assert self.inservice[1][stop - 1] not in [1, 2, 4, 4.25, 6]
			if (self.inservice[1][stop - 1].status == 5):
				assert self.inservice[1][stop - 1].pout_end is not None
				curr_vehicle.end_time = self.inservice[1][stop - 1].pout_end
				self.add_event( event(curr_vehicle.end_time, curr_vehicle, 'prepare_first_exit') )
			else:
				assert self.inservice[1][stop - 1].status in [1.5, 3, 4.5]
				assert curr_vehicle.blocked == False
				curr_vehicle.blocked = True
			return

		# Now check if any vehicles are traveling on the through lane, close to the current vehicle
		# the vehicle only starts the exit maneuver when there is no projected blocking caused by vehicles already on the lane.
		car = self.subhead
		prev = None
		req_time = self.curr

		while car != None:

			assert car.status != 4

			if car.status not in [3, 4.5]:
				car.update_loc()
			
			if car.curr_loc >= stop * LOT_LENGTH + CAR_LENGTH - SMALL_INTERVAL:
				prev = car

			elif (car.status == 1.5) and (car.stop == stop + 1):
				# assert (car.plin_end > self.curr)
				# req_time = max( req_time, car.plin_end )
				# changes on Jan 22, 2021
				prev = car
				# end changes

			# elif car.curr_loc >= max( 0, stop * LOT_LENGTH - CAR_LENGTH - POUT_DIST ) and car.status in [1, 6]:
			# 	if np.abs( car.curr_loc + CAR_LENGTH - stop * LOT_LENGTH ) < SMALL_INTERVAL:
			# 		cp = car.subtraj.head
			# 		if cp.data.t > self.curr:
			# 			import pdb; pdb.set_trace()
			# 		while cp.nex is not None and cp.nex.data.t <= self.curr:
			# 			cp = cp.nex
			# 		if cp.nex is None:
			# 			import pdb; pdb.set_trace()
			# 		assert cp.nex.data.t > self.curr >= cp.data.t
			# 		if cp.data.v == 0.0:
			# 			break
			# 	car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH, sub = True )
			# 	assert car_time > self.curr
			# 	req_time = max( req_time, car_time )

			# else:
			# 	break

			# changes added on Jan 22, 2021
			elif (car.status == 4.25) and (car.stop == stop + 1):
				print ('unexpected!')
		
			elif car.status in [3, 4.5]:
				assert car.stop < stop
				break

			elif car.status in [1.5, 2, 4.25]:
				assert car.stop < stop
				break

			else:
				if not ( (car.status == 5 and car.stop < stop) or (car.status in [1, 6]) ):
					import pdb; pdb.set_trace()

				if car.status in [1, 6] and np.abs( car.curr_loc + CAR_LENGTH - stop * LOT_LENGTH ) < SMALL_INTERVAL:
					cp = car.subtraj.head
					if cp.data.t > self.curr:
						import pdb; pdb.set_trace()
					while cp.nex is not None and cp.nex.data.t <= self.curr:
						cp = cp.nex
					if cp.nex is None:
						import pdb; pdb.set_trace()
					assert cp.nex.data.t > self.curr >= cp.data.t
					if cp.data.v == 0.0:
						break
					elif car.status == 1 and car.stop == stop - 1:
						break 
					elif car.dest_to_stop >= stop * LOT_LENGTH + CAR_LENGTH:
						car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH, sub = True )
						assert car_time > self.curr
						req_time = max( req_time, car_time )
					else:
						if not (car.dest_to_stop == stop * LOT_LENGTH and car.status == 1):
							import pdb; pdb.set_trace()
						car_time = car.end_time
						assert car_time > self.curr
						req_time = max( req_time, car_time )

				elif (car.status == 5) or (car.status == 1 and car.stop < stop):
					pass

				else:
					assert car.status in [1, 6]
					car_time = ((stop - 1) * LOT_LENGTH - car.curr_loc) / rateDRIV
					if car_time < meanPOUT:
						if car.status == 1 and car.stop == stop:
							car_time = car.end_time
						else:
							assert car.dest_to_stop >= stop * LOT_LENGTH + CAR_LENGTH
							car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH, sub = True )
						assert car_time > self.curr
						req_time = max( req_time, car_time )
			# end changes

			car = car.subnex

		if req_time > self.curr:
			curr_vehicle.end_time = req_time
			self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'prepare_first_exit') )
			car = None
			prev = None
			return

		# Now, the current vehicle is ready to start the exit maneuver
		curr_vehicle.subprev = prev
		if prev != None:
			curr_vehicle.subnex = prev.subnex
			if prev.subnex is not None:
				prev.subnex.subprev = curr_vehicle
			prev.subnex = curr_vehicle
		elif self.subhead != None:
			self.subhead.subprev = curr_vehicle
			curr_vehicle.subnex = self.subhead 
			self.subhead = curr_vehicle
		else:
			self.subhead = curr_vehicle

		curr_vehicle.first_out()
		assert curr_vehicle.pout_end is not None
		self.add_event( event( curr_vehicle.pout_end, curr_vehicle, 'prepare_pulling_out') )

		# and the trajectories of all upcoming vehicles on the sub lane
		car = curr_vehicle.subnex
		if car is not None and car.status in [1, 6]:
			car.update_loc()
			if car.curr_loc + CAR_LENGTH - stop * LOT_LENGTH > SMALL_INTERVAL:
				car.curr_loc = stop * LOT_LENGTH - CAR_LENGTH
			car.update_subtraj()
			car = car.subnex
		elif car is not None and car.status not in [3, 4.5]:
			car.update_loc()
			car.update_subtraj()
			car = car.subnex

		while (car != None) and (car.status not in [3, 4.5]):
			car.update_loc()
			car.update_subtraj()
			car = car.subnex

		return

	def prepare_pulling_out(self, curr_vehicle):

		stop = curr_vehicle.stop
		
		if curr_vehicle.status != 4.5:
			assert (curr_vehicle.status == 3 and curr_vehicle.lane == 1) or (curr_vehicle.status == 4.25 and curr_vehicle.lane == 0)
			curr_vehicle.status = 4.5
			if curr_vehicle.lane == 0:
				assert self.inservice[0][stop - 1] == curr_vehicle
				assert self.inservice[1][stop - 1] is None
				self.inservice[1][stop - 1] = curr_vehicle
				self.inservice[0][stop - 1] = None

		assert self.inservice[1][stop - 1] == curr_vehicle

		# First, check if the current vehicle can leave the system through the second lane
		if self.subhead == curr_vehicle:
			assert curr_vehicle.curr_loc == stop * LOT_LENGTH
			assert curr_vehicle.subprev is None
			curr_vehicle.status = 6
			curr_vehicle.dest_to_stop = self.n + CAR_LENGTH
			curr_vehicle.update_subtraj()
			self.inservice[1][stop - 1] = None
			assert curr_vehicle.end_time is not None
			self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'leave_system') )

			car = curr_vehicle.subnex
			while car is not None and car.status not in [3, 4.5]:
				car.update_loc()
				car.update_subtraj()
				car = car.subnex

			if (self.inservice[0][stop - 1] is not None) and (self.inservice[0][stop - 1].blocked):
				assert self.inservice[0][stop - 1].status == 4
				self.inservice[0][stop - 1].blocked = False
				req_time = curr_vehicle.calc_time( (stop + 1) * LOT_LENGTH, sub = True )
				assert req_time > self.curr
				self.inservice[0][stop - 1].end_time = req_time
				self.add_event( event(req_time, self.inservice[0][stop - 1], 'prepare_first_exit') ) 

			# lastly we schedule the replacement vehicle by checking two things
			# 1) if the entrance is cleared
			# 2) if the replacement vehicle will only arrive after the current vehicle finishes the exit maneuver
			# Note: 2) is already satisfied for partial control
			# new_vehicle = vehicle(self)
			# req_time = self.curr
			# if self.head != None:
			# 	car = self.head
			# 	while car.nex is not None:
			# 		car = car.nex
			# 	car.update_loc()
			# 	if car.curr_loc <= CAR_LENGTH:
			# 		req_time = max( req_time, car.calc_time(CAR_LENGTH) )
			# 	car = None 

			# self.add_event( event( req_time, new_vehicle, 'enter_system') )
			# heappush( self.waiting, (- stop) )

			# changes added on Jan 22, 2021
			new_vehicle = vehicle(self)
			self.add_event( event(self.curr, new_vehicle, 'enter_system') )
			############ alternative 2 specific ############
			heappush( self.waiting, (- curr_vehicle.stop_idx) )
			################################################
			return

		if curr_vehicle.idx == VEHICLE_IDX:
			delay_reason = None
			delay_status = None

		req_time = self.curr
		delayed = False

		# Now check if any vehicles in the neighborhood have started but not finished their exit or enter maneuvers
		# between the middle lane and the through lane
		if (stop >= 2) and (self.inservice[1][stop - 2] is not None) and (self.inservice[1][stop - 2].status == 5):
			car = self.inservice[1][stop - 2]
			assert car.pout_end > self.curr
			delayed = True
			if curr_vehicle.idx == VEHICLE_IDX and car.pout_end > req_time:
				delay_reason = 1
				delay_status = 5
			req_time = max(req_time, car.pout_end)

		# if (stop <= self.N - 1) and (self.inservice[1][stop] is not None) and (self.inservice[1][stop].status == 1.5):
		# 	if curr_vehicle.idx == VEHICLE_IDX and self.inservice[1][stop].plin_end - min(meanDRIV, max(0, self.inservice[1][stop].plin_time - .5)) > req_time:
		# 		delay_reason = 2
		# 		delay_status = 1.5
		# 	req_time = max( req_time, self.inservice[1][stop].plin_end - min(meanDRIV, max(0, self.inservice[1][stop].plin_time - .5)) )

		# if req_time > self.curr:
		# 	curr_vehicle.end_time = req_time
		# 	curr_vehicle.update_subtraj()
		# 	self.add_event( event( req_time, curr_vehicle, 'prepare_pulling_out') )
		# 	if curr_vehicle.idx == VEHICLE_IDX:
		# 		self.debug_times.append( tuple( (delay_reason, delay_status, req_time) ) )
		# 		assert delay_reason != 5
		# 	car = curr_vehicle.subnex
		# 	while car is not None and car.status not in [3, 4.5]:
		# 		car.update_loc()
		# 		car.update_subtraj()
		# 		car = car.subnex
		# 	car = None
		# 	return

		# Lastly, check if any vehicles are traveling on the through lane, 
		# s.t. the exit maneuver of the current vehicle will be blocked or block the movements of the existing vehicles
		# the vehicles already on the through lane can have a status of [1, 1.5, 5, 6]
		
		car = self.head
		prev = None
		stopped = False

		while car != None:
			car.update_loc()

			if car.curr_loc >= stop * LOT_LENGTH + CAR_LENGTH - SMALL_INTERVAL:
				prev = car

			# changes added on Jan 22, 2021
			elif car.status == 1.5 and car.stop == stop + 1:
				assert (stop <= self.N - 1) and (self.inservice[1][stop] == car)
				assert car.plin_end >= self.curr > car.plin_start
				if meanPLIN + car.plin_start - meanDRIV > self.curr:
					delayed = True
					car_time = car.plin_start + meanPLIN - meanDRIV
					assert car_time > self.curr
					if curr_vehicle.idx == VEHICLE_IDX and car_time > req_time:
						delay_reason = 2
						delay_status = 1.5
					req_time = max( req_time, car_time)
				else:
					prev = car

			elif stopped and car.status != 5:
				pass

			elif car.status == 5:
				assert car.stop < stop
				assert car.dest_to_stop >= stop * LOT_LENGTH + CAR_LENGTH
				car_time = max(0, meanPOUT - (self.curr - car.pout_start)) + ((stop - 1) * LOT_LENGTH - car.curr_loc) / rateDRIV
				if car_time < meanPOUT:
					delayed = True
					car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH )
					assert car_time > self.curr
					if curr_vehicle.idx == VEHICLE_IDX and car_time > req_time:
						delay_reason = 3
						delay_status = 5
					req_time = max( req_time, car_time )

			elif car.status == 1.5:
				assert car.stop < stop
				# pass
				# change made on Jan 25, 2021
				stopped = True

			elif (car.stop == stop) and (car.status == 1):
				assert (car.curr_loc <= car.dest_to_stop == (stop - 1) * LOT_LENGTH)
				break

			elif (car.stop < stop) and (car.status == 1):
				assert (car.curr_loc <= car.dest_to_stop <= (stop - 2) * LOT_LENGTH)
				stopped = True

			elif (car.stop == stop + 1) and (car.status == 1):
				assert car.dest_to_stop == stop * LOT_LENGTH
				car_time = ((stop - 1) * LOT_LENGTH - car.curr_loc) / rateDRIV
				if car_time < meanPOUT:
					delayed = True
					# car_time = car.end_time + meanPLIN
					assert car.end_time >= self.curr
					if car.end_time == self.curr:
						car_time = car.end_time + SMALL_INTERVAL
					else:
						car_time = car.end_time
					if curr_vehicle.idx == VEHICLE_IDX and car_time > req_time:
						delay_reason = 4
						delay_status = 1
					req_time = max( req_time, car_time )

			else:
				assert car.status in [1, 6]
				assert car.dest_to_stop >= stop * LOT_LENGTH + CAR_LENGTH
				car_time = ((stop - 1) * LOT_LENGTH - car.curr_loc) / rateDRIV
				if car_time < meanPOUT:
					delayed = True
					car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH )
					assert car_time > self.curr
					if curr_vehicle.idx == VEHICLE_IDX and car_time > req_time:
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

			'''
			elif car.curr_loc >= max(0, stop * LOT_LENGTH - CAR_LENGTH - POUT_DIST):
				assert (car.status != 2) and (car.status != 4.25)
				
				if (car.stop == stop) and (car.status == 1):
					assert (car.curr_loc <= car.dest_to_stop == (stop - 1) * LOT_LENGTH)
					break

				elif (car.stop < stop) and (car.status == 1):
					assert (car.curr_loc <= car.dest_to_stop <= (stop - 2) * LOT_LENGTH)
					break
				
				elif car.status == 1.5 and car.stop == stop + 1:
					car_time = car.plin_end
					if car_time == self.curr:
						car_time += SMALL_INTERVAL
					assert car_time > self.curr
					delayed = True
					if curr_vehicle.idx == VEHICLE_IDX and req_time > car_time:
						delay_reason = 3
						delay_status = 1.5
					req_time = max( req_time, car_time )

				elif car.status == 1.5:
					assert car.stop < stop
					break

				elif car.status == 5:
					assert car.stop < stop - 1

				else:
					assert (car.status == 6) or (car.status == 1 and car.stop > stop)
					assert car.dest_to_stop > stop * LOT_LENGTH - SMALL_INTERVAL
					car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH )
					assert car_time >= self.curr
					if car_time == self.curr:
						assert car.status == 1
						assert car.stop == stop + 1
						delayed = True
						if curr_vehicle.idx == VEHICLE_IDX and self.curr + meanPLIN > req_time:
							delay_reason = 4
							delay_status = 1
						req_time = max( req_time, self.curr + meanPLIN )
					else:
						assert car_time > self.curr
						delayed = True
						if curr_vehicle.idx == VEHICLE_IDX and car_time > req_time:
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
			else:
				break
			car = car.nex
			'''

		if delayed:
			assert req_time > self.curr
			curr_vehicle.end_time = req_time
			curr_vehicle.update_subtraj()
			self.add_event( event( req_time, curr_vehicle, 'prepare_pulling_out') )
			if curr_vehicle.idx == VEHICLE_IDX:
				self.debug_times.append( tuple( (delay_reason, delay_status, req_time) ) )
				if delay_reason == 5:
					self.debug_speed.append( delay_speed )
			car = curr_vehicle.subnex
			while car is not None and car.status not in [3, 4.5]:
				car.update_loc()
				car.update_subtraj()
				car = car.subnex
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
		self.add_event( event( curr_vehicle.pout_end, curr_vehicle, 'finish_pulling_out') )

		# and we update the trajectories of all upcoming vehicles
		car = curr_vehicle.nex
		while car != None:
			car.update_loc()
			car.update_traj()
			car = car.nex

		# and the trajectories of all upcoming vehicles on the sub lane
		car = curr_vehicle.subnex 	
		while car is not None and car.status not in [3, 4.5]:
			car.update_loc()
			car.update_subtraj()
			car = car.subnex
		
		if (self.inservice[0][stop - 1] is not None) and (self.inservice[0][stop - 1].blocked):
			assert self.inservice[0][stop - 1].status == 4
			self.inservice[0][stop - 1].blocked = False
			assert curr_vehicle.pout_end > self.curr
			self.inservice[0][stop - 1].end_time = curr_vehicle.pout_end
			self.add_event( event(curr_vehicle.pout_end, self.inservice[0][stop - 1], 'prepare_first_exit') ) 

		# lastly we schedule the replacement vehicle by checking two things
		# 1) if the entrance is cleared
		# 2) if the replacement vehicle will only arrive after the current vehicle finishes the exit maneuver
		# Note: 2) is already satisfied for partial control
		# req_time = self.curr
		# new_vehicle = vehicle(self)
		# if self.head != None:
		# 	car = self.head
		# 	while car.nex is not None:
		# 		car = car.nex
		# 	car.update_loc()
		# 	if car.curr_loc <= CAR_LENGTH:
		# 		if car.status == 1 and car.stop <= 2 and car.dest_to_stop == car.curr_loc:
		# 			req_time = max( req_time, car.end_time )
		# 		elif car.status == 1:
		# 			req_time = max( req_time, car.calc_time(CAR_LENGTH) )
		# 	car = None

		# self.add_event( event( req_time, new_vehicle, 'enter_system') )
		# # heappush( self.waiting, (- stop) )

		# changes added on Jan 22, 2021
		new_vehicle = vehicle(self)
		self.add_event( event(self.curr, new_vehicle, 'enter_system') )
		############ alternative 2 specific ############
		heappush( self.waiting, (- curr_vehicle.stop_idx) )
		################################################
		return

	def finish_pulling_out(self, curr_vehicle):

		assert curr_vehicle.status == 5
		curr_vehicle.status = 6

		if curr_vehicle.subprev is not None:
			curr_vehicle.subprev.subnex = curr_vehicle.subnex
		else:
			assert curr_vehicle == self.subhead
			self.subhead = curr_vehicle.subnex

		if curr_vehicle.subnex is not None:
			car = curr_vehicle.subnex
			car.subprev = curr_vehicle.subprev
			while car is not None and car.status not in [3, 4.5]:
				car.update_loc()
				car.update_subtraj()
				car = car.subnex
			car = None

		curr_vehicle.subprev = None
		curr_vehicle.subnex = None
		curr_vehicle.subtraj = None

		assert self.inservice[1][curr_vehicle.stop - 1] is not None
		self.inservice[1][curr_vehicle.stop - 1] = None
		assert curr_vehicle.end_time is not None
		self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'leave_system') )
		return 			

	def enter_system(self, curr_vehicle, debug_idx = None):

		length = len(self.waiting)
		assert curr_vehicle.status == 0

		if self.entry_blocked == self.curr:
			self.add_event( event( self.entry_cleared, curr_vehicle, 'enter_system' ) )
			if len(self.waiting) != length:
				import pdb; pdb.set_trace()
			return

		# first, we decide if a vehicle cannot enter either lane
		# i.e. if and only if entry to both lanes is blocked by other vehicles
		enter_r = True
		enter_m = True
		delay_r = self.curr
		delay_m = self.curr

		if self.head is not None:
			car = self.head
			while car.nex is not None:
				car = car.nex
			car.update_loc()
			
			# if the last one is within CAR_LENGTH (i.e. first lane block occupied)
			# the replacement vehicle cannot enter under either access control	
			if car.curr_loc <= CAR_LENGTH + SMALL_INTERVAL:
				car_time = car.calc_time( CAR_LENGTH + SMALL_INTERVAL )
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
					if not car_time > self.curr:
						import pdb; pdb.set_trace()
					enter_r = False
					delay_r = car_time

		subcar = None
		if self.subhead is not None:
			subcar = self.subhead
			while subcar.subnex is not None:
				subcar = subcar.subnex
			if subcar.status not in [3, 4.5]:
				subcar.update_loc()

			# if the last one is within CAR_LENGTH (i.e. first lane block occupied)
			# the replacement vehicle cannot enter under either access control
			if subcar.curr_loc <= CAR_LENGTH + SMALL_INTERVAL:
				car_time = subcar.calc_time( CAR_LENGTH + SMALL_INTERVAL, sub = True )
				if car_time <= self.curr:
					cp = subcar.subtraj.head
					assert cp.data.t <= self.curr
					while cp.nex is not None and cp.nex.data.t <= self.curr:
						cp = cp.nex
					if (cp.nex is None):
						# if (subcar.curr_loc == subcar.dest_to_stop == CAR_LENGTH):
						# 	assert subcar.stop == 1
						# 	enter_m = False
						# 	delay_m = car_time + SMALL_INTERVAL
						# else:
						import pdb; pdb.set_trace() 
					else:
						assert cp.nex.data.t > self.curr
						if cp.data.v > 0.0:
							pass
						else:
							import pdb; pdb.set_trace()	
				else:
					if not car_time > self.curr:
						import pdb; pdb.set_trace()
					enter_m = False
					delay_m = car_time

		############ alternative 2 specific ############
		stop_idx = - heappop(self.waiting)
		stop = stop_idx
		if stop > self.N:
			stop -= self.N
		################################################

		if self.inservice[1][0] is not None and self.inservice[1][0].status == 4.5:
			enter_r = False
			delay_r = max( delay_r, self.inservice[1][0].end_time)
			enter_m = False
			delay_m = max( delay_m, self.inservice[1][0].end_time)

		if self.inservice[0][0] is not None and self.inservice[0][0].status == 4:
			if (self.inservice[0][0].blocked and enter_m):
				import pdb; pdb.set_trace()
			if not self.inservice[0][0].blocked:
				assert self.inservice[0][0] is not None
				enter_m = False
				delay_m = max( delay_m, self.inservice[0][0].end_time )
				if stop == 1:
					enter_r = False
					delay_r = max( delay_r, self.inservice[0][0].end_time )

		if (not enter_r) and (not enter_m):
			assert delay_r > self.curr
			self.entry_blocked = self.curr
			if (delay_m > self.curr):
				self.entry_cleared = min(delay_r, delay_m)
			else:
				self.entry_cleared = delay_r
			self.add_event( event( self.entry_cleared, curr_vehicle, 'enter_system' ) )
			############ alternative 2 specific ############
			heappush(self.waiting, (-stop_idx) )
			################################################
			if len(self.waiting) != length:
				import pdb; pdb.set_trace()
			return

		# stop = - heappop(self.waiting)
		# if (not enter_m) and (stop == 1):
		# 	assert (enter_r)
		# 	assert delay_m > self.curr
		# 	self.entry_blocked = self.curr
		# 	self.entry_cleared = delay_m
		# 	self.add_event( event( self.entry_cleared, curr_vehicle, 'enter_system' ) )
		# 	heappush(self.waiting, (-stop) )
		# 	if len(self.waiting) != length:
		# 		import pdb; pdb.set_trace()
		# 	return

		############ alternative 2 specific ############
		if (not enter_m) and (stop == self.N + 1):
			assert (enter_r)
			assert delay_m > self.curr
			self.entry_blocked = self.curr
			self.entry_cleared = delay_m
			self.add_event( event( self.entry_cleared, curr_vehicle, 'enter_system' ) )
			heappush(self.waiting, (-stop_idx) )
			if len(self.waiting) != length:
				import pdb; pdb.set_trace()
			return
		################################################	

		if (not enter_r) and (subcar is not None):
			assert enter_m
			assert self.subhead is not None
			enter_m = False
			visited = []
			while self.waiting != []:
				prev = subcar
				while (prev is not None) and (prev.curr_loc < stop * LOT_LENGTH + CAR_LENGTH - SMALL_INTERVAL):
					if (prev.status in [1.5, 3, 4.25, 4.5]) or (prev.status == 1 and prev.dest_to_stop <= stop * LOT_LENGTH):
						visited.append(stop_idx)
						stop_idx = - heappop(self.waiting)
						stop = stop_idx 
						if stop > self.N:
							stop -= self.N
						break
					prev = prev.subprev
				if (prev is None) or (prev.curr_loc >= stop * LOT_LENGTH + CAR_LENGTH - SMALL_INTERVAL):
					enter_m = True
					break

			if not enter_m:
				assert self.waiting == []
				prev = subcar
				while (prev is not None) and (prev.curr_loc < stop * LOT_LENGTH + CAR_LENGTH - SMALL_INTERVAL):
					if (prev.status in [1.5, 3, 4.25, 4.5]) or (prev.status == 1 and prev.dest_to_stop <= stop * LOT_LENGTH):
						visited.append(stop_idx)
						delay_m = max( delay_m, prev.calc_time( stop * LOT_LENGTH + CAR_LENGTH, sub = True ) )
						break
					prev = prev.subprev
				if (prev is None) or (prev.curr_loc >= stop * LOT_LENGTH + CAR_LENGTH - SMALL_INTERVAL):
					enter_m = True

			for element in visited:
				heappush( self.waiting, (- element) )

			if (not enter_m):
				stop_idx = None
				stop = None
				assert delay_r > self.curr
				self.entry_blocked = self.curr
				if (delay_m > self.curr):
					self.entry_cleared = min(delay_r, delay_m)
				else:
					self.entry_cleared = delay_r
				self.add_event( event( self.entry_cleared, curr_vehicle, 'enter_system' ) )
				if len(self.waiting) != length:
					import pdb; pdb.set_trace()
				return

		assert stop_idx is not None
		assert stop is not None
		assert (stop_idx == stop) or (stop_idx == stop + self.N)
		if len(self.waiting) != (length - 1):
			import pdb; pdb.set_trace()

		# otherwise, the replacement vehicle can enter at least 1 of the middle lane and the through lane immediately
		self.inCount += 1
		curr_vehicle.assign_spot(stop_idx)
		if debug_idx is not None and curr_vehicle.idx == debug_idx:
			import pdb; pdb.set_trace()

		# if the right lane is blocked right now, or if the middle lane has no vehicle
		# then the replacement vehicle will undoubtedly enter the middle lane 
		if (not enter_r) or (self.subhead is None):
			assert enter_m
			curr_vehicle.entry = 1
			if self.subhead is None:
				self.subhead = curr_vehicle
			else:
				curr_vehicle.subprev = subcar
				subcar.subnex = curr_vehicle
			curr_vehicle.dest_to_stop = curr_vehicle.stop * LOT_LENGTH
			curr_vehicle.update_subtraj()
			assert curr_vehicle.end_time is not None 
			self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'start_second_enter' ) )
			return

		# if the middle lane is blocked right now, the replacement vehicle enters the right lane immediately
		# notably even if the right lane has no vehicle, the estimated time to arrival through right lane is not nessarily shorter 
		# if (not enter_m) or (stop >= self.N * 0.5):
		if (not enter_m):
			assert enter_r
			curr_vehicle.entry = 2
			if self.head is None:
				self.head = curr_vehicle
			else:
				curr_vehicle.prev = car
				car.nex = curr_vehicle
			curr_vehicle.dest_to_stop = (stop - 1) * LOT_LENGTH
			curr_vehicle.update_traj()
			assert curr_vehicle.end_time is not None
			self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'start_pulling_in' ) )
			return

		# if neither lane is blocked
		assert (enter_r and enter_m)
		assert (self.subhead is not None)

		car = self.head
		prev = None
		eta_r = self.curr + (stop - 1) * LOT_LENGTH / rateDRIV + meanPLIN
		
		while car is not None:
			car.update_loc()
			assert car.status != 2 and car.status != 4.25

			if (stop < car.stop) or (car.status == 6):
				pass

			elif (car.status == 1.5):
				assert car.plin_start <= self.curr
				if ( (car.dest_to_stop - CAR_LENGTH) / rateDRIV < max(0.0, meanPLIN - (self.curr - car.plin_start)) ):
					eta_r = max( eta_r, meanPLIN + car.plin_start + ( (stop - 1) * LOT_LENGTH - (car.dest_to_stop - CAR_LENGTH) ) / rateDRIV + meanPLIN )

			elif (car.status == 5):
				assert car.pout_start <= self.curr
				if ( (car.stop - 1) * LOT_LENGTH / rateDRIV < max(0.0, meanPOUT - (self.curr - car.pout_start)) ):
					eta_r = max( eta_r, meanPOUT + car.pout_start + (stop - car.stop) * LOT_LENGTH / rateDRIV + meanPLIN )

			else:
				assert car.status == 1
				assert car.dest_to_stop >= car.curr_loc
				if ( (car.dest_to_stop - CAR_LENGTH) / rateDRIV < (car.dest_to_stop - car.curr_loc) / rateDRIV + meanPLIN ):
					eta_r = max( eta_r, self.curr + (car.dest_to_stop - car.curr_loc) / rateDRIV + meanPLIN + ((stop - 1) * LOT_LENGTH - (car.dest_to_stop - CAR_LENGTH)) / rateDRIV + meanPLIN )

			prev = car
			car = car.nex

		subcar = self.subhead
		subprev = None
		eta_m = self.curr + stop * LOT_LENGTH / rateDRIV
		entry = 1
		
		while subcar is not None:

			if subcar.status in [3, 4.25, 4.5]:
				entry = 2
				break

			subcar.update_loc()

			if (subcar.status == 6) or (stop < subcar.stop):
				pass

			elif (subcar.status == 1.5) and (subcar.lane == 0):
				assert subcar.plin_start <= self.curr
				if (subcar.dest_to_stop - CAR_LENGTH) / rateDRIV < meanPLIN + max(0.0, meanPLIN - (self.curr - subcar.plin_start)):
					eta_m = max( eta_m, 2 * meanPLIN + subcar.plin_start + (stop * LOT_LENGTH - (subcar.dest_to_stop - CAR_LENGTH)) / rateDRIV )

			elif (subcar.status == 2):
				assert subcar.lane == 0
				assert subcar.plin_start <= self.curr
				if (subcar.dest_to_stop - CAR_LENGTH) / rateDRIV < max(0.0, meanPLIN - (self.curr - subcar.plin_start)):
					eta_m = max( eta_m, meanPLIN + subcar.plin_start + (stop * LOT_LENGTH - (subcar.dest_to_stop - CAR_LENGTH)) / rateDRIV )

			elif (subcar.status == 1.5) and (subcar.lane == 1):
				assert (stop > subcar.stop)
				entry = 2
				break

			elif (subcar.status == 5):
				assert subcar.pout_start <= self.curr
				if (subcar.dest_to_stop - CAR_LENGTH) / rateDRIV < max(0.0, meanPOUT - (self.curr - subcar.pout_start)):
					eta_m = max( eta_m, meanPOUT + subcar.pout_start + (stop * LOT_LENGTH - (subcar.dest_to_stop - CAR_LENGTH)) / rateDRIV )

			else:
				assert subcar.status == 1
				assert subcar.dest_to_stop >= subcar.curr_loc
				if ( (subcar.dest_to_stop - CAR_LENGTH) / rateDRIV < (subcar.dest_to_stop - subcar.curr_loc) / rateDRIV + meanPLIN ):
					eta_m = max( eta_m, self.curr + (subcar.dest_to_stop - subcar.curr_loc) / rateDRIV + meanPLIN + (stop * LOT_LENGTH - (subcar.dest_to_stop - CAR_LENGTH)) / rateDRIV + meanPLIN )

			subprev = subcar
			subcar = subcar.subnex

		curr_vehicle.entry = entry

		if (entry == 2) or (eta_r <= eta_m):
			curr_vehicle.entry = 2
			curr_vehicle.prev = prev
			if prev is not None:
				assert prev.nex is None
				prev.nex = curr_vehicle
			else:
				assert self.head is None
				self.head = curr_vehicle
			curr_vehicle.dest_to_stop = (curr_vehicle.stop - 1) * LOT_LENGTH
			curr_vehicle.update_traj()
			assert curr_vehicle.end_time is not None
			self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'start_pulling_in' ) )
			return

		assert (entry == 1) and (eta_r > eta_m)
		curr_vehicle.entry = 1
		curr_vehicle.subprev = subprev
		subprev.subnex = curr_vehicle
		curr_vehicle.dest_to_stop = curr_vehicle.stop * LOT_LENGTH
		curr_vehicle.update_subtraj()
		assert curr_vehicle.end_time is not None
		self.add_event( event( curr_vehicle.end_time, curr_vehicle, 'start_second_enter' ) )
		return


class vehicle():

	# def __init__(self, sys, getin = False, stop = None, lane = None):
	
	############ alternative 2 specific ############
	def __init__(self, sys, getin = False, stop_idx = None):
	################################################

		self.sys = sys
		self.driv = self.sys.timeDRIV.next()
		self.status = 0
		self.blocked = False

		self.idx = None
		self.stop = None
		self.lane = None
		self.entry = None
		self.curr_loc = None
		self.dest_to_stop = None
		self.enter_time = None
		self.end_time = None
		self.prod_time = 0.0

		self.prev = None
		self.nex = None
		self.subprev = None
		self.subnex = None
		self.traj = None
		self.subtraj = None 		

		# if getin:
		# 	assert stop is not None
		# 	assert lane is not None
		# 	self.assign_spot(stop)
		# 	self.status = 2 - 0.5 * lane 
		# 	self.lane = lane
		# 	self.start_service()
		# else:
		# 	assert stop is None
		# 	assert lane is None
		# 	self.curr_loc = 0.0

		############ alternative 2 specific ############
		self.stop_idx = None
		if getin:
			assert stop_idx is not None
			self.assign_spot(stop_idx)
			self.status = 2 - 0.5 * self.lane 
			self.start_service()
		else:
			assert stop_idx is None
			self.curr_loc = 0.0
		################################################

	# def assign_spot(self, stop):
	# 	assert self.status == 0
	# 	self.status = 1
	# 	self.idx = self.sys.inCount
	# 	self.stop = stop
	# 	self.enter_time = self.sys.curr

	############ alternative 2 specific ############
	def assign_spot(self, stop_idx):

		assert self.status == 0
		self.status = 1
		self.idx = self.sys.inCount
		self.enter_time = self.sys.curr
		self.stop_idx = stop_idx
		if stop_idx <= self.sys.N:
			self.stop = stop_idx
			self.lane = 1
		else:
			assert stop_idx <= 2 * self.sys.N
			self.stop = stop_idx - self.sys.N
			self.lane = 0	
	################################################

	def start_in(self):
		assert self.curr_loc == self.dest_to_stop
		# not applicable for alternative 2
		# assert self.lane is None
		assert self.status == 1
		self.status = 1.5
		self.plin_time = self.sys.timePLIN.next()
		self.plin_end = self.plin_time + self.sys.curr
		self.plin_start = self.sys.curr
		self.prod_time += self.plin_time
		self.update_subtraj()
		self.update_traj()

	def second_in(self):
		assert self.curr_loc == self.dest_to_stop
		assert self.lane == 0
		assert (self.status == 1.5) or (self.entry == 1 and self.status == 1)
		self.status = 2
		self.traj = None
		self.plin_time = self.sys.timePLIN.next()
		self.plin_end = self.plin_time + self.sys.curr
		self.plin_start = self.sys.curr
		self.prod_time += self.plin_time
		self.update_subtraj()

	def start_service(self):
		if self.lane == 0:
			assert self.status == 2
		else:
			assert self.lane == 1
			assert self.status == 1.5
		self.status = 3 
		self.curr_loc = self.stop * LOT_LENGTH
		self.dest_to_stop = self.sys.n + CAR_LENGTH
		self.serv_time = self.sys.timeSERV.next()
		self.serv_end = self.serv_time + self.sys.curr
		self.serv_start = self.sys.curr
		self.prod_time += self.serv_time
		self.traj = None
		self.subtraj = None
		if self.lane == 1:
			self.end_time = self.serv_end
			self.update_subtraj()

	def first_out(self):
		assert self.lane == 0
		assert self.status == 4
		self.status = 4.25
		self.pout_time = self.sys.timePOUT.next()
		self.pout_end = self.pout_time + self.sys.curr
		self.pout_start = self.sys.curr
		self.prod_time += self.pout_time
		assert (self.curr_loc - self.stop * LOT_LENGTH) < SMALL_INTERVAL
		self.curr_loc = self.stop * LOT_LENGTH
		self.update_subtraj()

	def start_out(self):
		assert self.status == 4.5
		self.status = 5
		self.pout_time = self.sys.timePOUT.next()
		self.pout_end = self.pout_time + self.sys.curr
		self.pout_start = self.sys.curr
		self.prod_time += self.pout_time
		# changes made on Jan 22, 2021
		if self.prev is not None and self.prev.status == 1.5 and self.prev.stop == self.stop + 1 and self.prev.plin_end > self.pout_end:
			self.pout_end = self.prev.plin_end
		# end changes 
		self.update_subtraj()
		self.update_traj()

	# check
	def calc_time(self, loc, sub = False):

		if self.curr_loc > loc:
			import pdb; pdb.set_trace()

		if sub:
			cp = self.subtraj.head
		else:
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

	# check	
	def update_loc(self):

		assert (self.status != 3) and (self.status != 4.5) and (self.status != 4)

		if (self.status == 1.5) or (self.status == 2):
			assert self.curr_loc == self.dest_to_stop 
			return 

		if (self.status == 4.25) or (self.status == 5):
			assert self.curr_loc == self.stop * LOT_LENGTH
			return

		if self.traj is not None:
			assert self.subtraj is None
			cp = self.traj.head
		else:
			assert (self.subtraj is not None)
			cp = self.subtraj.head

		if cp.nex is None:
			if not np.abs(cp.data.t - self.sys.curr) <= 3 * SMALL_INTERVAL:
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

	# check
	def update_traj(self):

		traj = self.traj
		end_time = self.end_time
		assert self.status in [1, 1.5, 5, 6]

		self.traj = DLinkedList()

		if self.status == 1.5:
			# i.e. the vehicle has started pulling in but has not started service
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
			if not (np.abs(self.end_time - self.sys.curr) < SMALL_INTERVAL):
				import pdb; pdb.set_trace()
			if end_time is not None:
				assert self.end_time >= end_time
			assert self.end_time >= self.sys.curr
			self.traj.addEnd( changePoint(self.curr_loc, self.sys.curr, 0.0) )
			self.traj.addEnd( changePoint(self.curr_loc, self.end_time, 'D') )
			return

		if self.curr_loc == self.dest_to_stop and self.status == 1:
			if self.end_time is None:
				assert (self.stop == 1)
				self.end_time = self.sys.curr
			if not (np.abs(self.end_time - self.sys.curr) < 4 * SMALL_INTERVAL):
				if traj is None:
					import pdb; pdb.set_trace()
				elif traj.head.data.v != 0.0:
					import pdb; pdb.set_trace()
				self.traj = traj
				return
			if end_time is not None:
				assert self.end_time >= end_time 
			assert self.end_time >= self.sys.curr
			self.traj.addEnd( changePoint(self.curr_loc, self.sys.curr, 0.0) )
			self.traj.addEnd( changePoint(self.curr_loc, self.end_time, 'D') )
			return

		start_t = self.sys.curr

		if self.status == 5:
			# i.e. the vehicle has started pulling out 
			assert self.curr_loc == self.stop * LOT_LENGTH
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
				if not (self.end_time >= end_time - 5 * SMALL_INTERVAL):
					import pdb; pdb.set_trace()
				self.end_time = end_time
			self.traj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D') )
			return

		if self.prev.end_time <= start_t + 4 * SMALL_INTERVAL:
			# e.g. if prev finishes the exit maneuver before self
			self.traj.addEnd( changePoint(self.curr_loc, start_t, self.driv) )
			self.end_time = start_t + (self.dest_to_stop - self.curr_loc) / self.driv
			if (end_time is not None) and (self.end_time < end_time):
				if not (self.end_time >= end_time - 11 * SMALL_INTERVAL):
					import pdb; pdb.set_trace()
				self.end_time = end_time
			self.traj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D') )
			return

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
		# check if the distance between prev and curr is CAR_LENGTH
		if self.status != 5 and np.abs(self.curr_loc + CAR_LENGTH - prev_loc) < 1e-05:
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
						if not (self.end_time >= end_time - 5 * SMALL_INTERVAL):
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
								if not (self.end_time >= end_time - 6 * SMALL_INTERVAL):
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
									if not (self.end_time >= end_time - 13 * SMALL_INTERVAL):
										import pdb; pdb.set_trace()
									self.end_time = end_time
								self.traj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D'))
							return

						if np.abs(iter_x + (cp.nex.data.t - iter_t) * cp.data.v + CAR_LENGTH - cp.nex.data.x) > 3.7e-5:
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
					if not (self.end_time >= end_time - 1e-05):
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

	# check
	def update_subtraj(self):

		traj = self.subtraj
		end_time = self.end_time
		self.subtraj = DLinkedList()

		if self.status in [3, 4.5]:
			assert self.end_time > self.sys.curr
			assert self.curr_loc == self.stop * LOT_LENGTH
			self.subtraj.addEnd( changePoint(self.curr_loc, self.sys.curr, 0.0) )
			self.subtraj.addEnd( changePoint(self.curr_loc, self.end_time, 'D') )
			return

		if (self.status == 1.5) or (self.status == 2):
			# i.e. the vehicle has started the second enter maneuver but has not started service
			assert (self.curr_loc == self.dest_to_stop)
			assert self.plin_end >= self.sys.curr
			self.subtraj.addEnd( changePoint(self.curr_loc, self.sys.curr, 0.0) )
			self.end_time = self.plin_end
			if end_time is not None and self.end_time < end_time:
				import pdb; pdb.set_trace()
			self.subtraj.addEnd( changePoint(self.curr_loc, self.end_time, 'D') )
			return

		if (self.status == 4.25) or (self.status == 5):
			# i.e. the vehicle has started pulling out 
			assert (self.curr_loc == self.stop * LOT_LENGTH)
			assert self.subtraj.head == None
			assert self.pout_end >= self.sys.curr
			if self.end_time is None:
				assert self.pout_start == self.sys.curr
			if self.status == 4.25:
				self.end_time = self.pout_end
			self.subtraj.addEnd( changePoint(self.curr_loc, self.sys.curr, 0.0) )
			self.subtraj.addEnd( changePoint(self.curr_loc, self.pout_end, 'D'))
			return

		if self.curr_loc == self.dest_to_stop:
			assert ( (self.status == 6) | (self.status == 1) )
			if self.end_time is None:
				self.end_time = self.sys.curr
			assert self.end_time >= self.sys.curr
			if self.status == 6:
				assert (np.abs(self.end_time - self.sys.curr) < SMALL_INTERVAL)
			self.subtraj.addEnd( changePoint(self.curr_loc, self.sys.curr, 0.0) )
			self.subtraj.addEnd( changePoint(self.curr_loc, self.end_time, 'D') )
			return

		start_t = self.sys.curr

		if (self.subprev is None) or (self.subprev.curr_loc >= CAR_LENGTH + self.dest_to_stop) or (self.subprev.status in [1.5, 2] and self.subprev.stop * LOT_LENGTH >= self.dest_to_stop + CAR_LENGTH):
			if self.subprev is None:
				assert self.sys.subhead == self
			self.subtraj.addEnd( changePoint(self.curr_loc, start_t, self.driv) )
			self.end_time = start_t + (self.dest_to_stop - self.curr_loc) / self.driv
			if (end_time is not None) and (self.end_time < end_time):
				assert (self.end_time >= end_time - SMALL_INTERVAL)
				self.end_time = end_time
			self.subtraj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D') )
			return

		if (self.subprev.end_time <= start_t + 2 * SMALL_INTERVAL) or (self.subprev.status == 5 and self.subprev.pout_end <= start_t + SMALL_INTERVAL):
			# e.g. if prev has finished the enter maneuver before the current time
			# or if prev has left the system by now (but unlikely because leave_system has the highest event priority)
			self.subtraj.addEnd( changePoint(self.curr_loc, start_t, self.driv) )
			self.end_time = start_t + (self.dest_to_stop - self.curr_loc) / self.driv
			if (end_time is not None) and (self.end_time < end_time):
				assert (self.end_time >= end_time - SMALL_INTERVAL)
				self.end_time = end_time
			self.subtraj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D') )
			return

		#####################################################################################
		cp = self.subprev.subtraj.head
		if not (cp.nex is not None):
			import pdb; pdb.set_trace()
		# the assertion above should be fine since cp.nex == None iff either one of the following holds:
		# i) the vehicle is pulling in and thus self.getin is True
		# ii) the vehicle is the head onlane thus self.prev is None
		# it implies that cp.data.v != 'D'
		if not (cp.data.t <= start_t):
			import pdb; pdb.set_trace()

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
			assert (self.subprev.end_time == cp.data.t)
			assert cp.data.v == 'D'
			if not (self.subprev.status == 4.5 and self.subprev.end_time == start_t):
				import pdb; pdb.set_trace()
			self.subtraj.addEnd( changePoint(self.curr_loc, start_t, self.driv) )
			self.end_time = start_t + (self.dest_to_stop - self.curr_loc) / self.driv
			if end_time is not None:
				assert self.end_time >= end_time
			self.subtraj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D') )
			return

		# some sanity check before proceeding
		assert (cp.data.v != 'D') and (cp.nex != None)
		assert (cp.data.t <= start_t < cp.nex.data.t)

		# let us define some counters to keep track of time and location (in the projected traj)
		iter_t = start_t
		if (start_t == self.sys.curr):
			if self.subprev.status not in [3, 4.5]:
				self.subprev.update_loc()
			prev_loc = self.subprev.curr_loc
		else:
			prev_loc = cp.data.x + (start_t - cp.data.t) * cp.data.v
	
		# the RHS below is the location of prev at start_t
		# check if the distance between prev and curr is CAR_LENGTH
		if self.status != 5 and np.abs(self.curr_loc + CAR_LENGTH - prev_loc) < 1e-05:
			self.curr_loc = max( 0.0, prev_loc - CAR_LENGTH )
			if (prev_loc < CAR_LENGTH):
				self.traj.addEnd( changePoint(0.0, start_t, 0.0) )
				start_t = self.subprev.calc_time(CAR_LENGTH + SMALL_INTERVAL, sub = True)
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
					self.subtraj.addEnd( changePoint(start_x, start_t, self.driv) )
					self.end_time = start_t + (self.dest_to_stop - start_x) / self.driv
					if (end_time is not None) and (self.end_time < end_time):
						if not (self.end_time >= end_time - SMALL_INTERVAL):
							if (self.subprev.subprev is not None) and (self.subprev.subprev.status in [3, 4.5]):
								pass
							else:
								car = self.subprev
								possible = False
								while car is not None and car.status not in [3, 4.5]:
									if car == self.sys.curr_vehicle and car.status in [1.5, 4.25]:
										if (self.sys.curr_typ == 'prepare_first_exit' and car.status == 4.25) or (self.sys.curr_typ == 'start_pulling_in' and car.status == 1.5):
											possible = True
									car = car.subprev
								if car is not None and car.status in [3, 4.5] and possible:
									pass
								elif self.end_time - end_time < - 1000 * SMALL_INTERVAL:
									pass
								else:
									import pdb; pdb.set_trace()
						else:
							self.end_time = end_time
					self.subtraj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D'))
					return
				elif (cp.data.v < self.driv):
					self.subtraj.addEnd( changePoint(start_x, start_t, self.driv) )
					break
				elif (cp.data.v > self.driv):
					break
				else:
					assert (cp.data.v == self.driv)

			assert (cp is not None) and (cp.data.v != self.driv)
			if not (start_x + (cp.data.t - start_t) * self.driv + CAR_LENGTH == cp.data.x):
				if np.abs(start_x + (cp.data.t - start_t) * self.driv + CAR_LENGTH - cp.data.x) > SMALL_INTERVAL:
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
							# print ('hey!')
							# self.traj.addEnd( changePoint(dest_to_stop, iter_t, cp.data.v) )
							self.end_time = iter_t
							if (end_time is not None) and (self.end_time < end_time):
								if not (self.end_time >= end_time - SMALL_INTERVAL):
									if (self.subprev.subprev is not None) and (self.subprev.subprev.status in [3, 4.5]):
										pass
									else:
										car = self.subprev
										possible = False
										while car is not None and car.status not in [3, 4.5]:
											if car == self.sys.curr_vehicle and car.status in [1.5, 4.25]:
												if (self.sys.curr_typ == 'prepare_first_exit' and car.status == 4.25) or (self.sys.curr_typ == 'start_pulling_in' and car.status == 1.5):
													possible = True
											car = car.subprev
										if car is not None and car.status in [3, 4.5] and possible:
											pass
										elif self.end_time - end_time < - 1000 * SMALL_INTERVAL:
											pass
										else:
											import pdb; pdb.set_trace()
								else:
									self.end_time = end_time						
							self.subtraj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D') )
							# import pdb; pdb.set_trace()
							return
						else: 
							self.subtraj.addEnd( changePoint(iter_x, iter_t, cp.data.v) )

						if cp.nex.data.x - CAR_LENGTH >= self.dest_to_stop:
							if not (cp.data.v > 0.0):
								if ( np.abs(cp.nex.data.x - cp.data.x) < SMALL_INTERVAL ):
									self.end_time = cp.nex.data.t
									if (end_time is not None) and (self.end_time < end_time):
										if not (self.end_time >= end_time - SMALL_INTERVAL):
											if (self.subprev.subprev is not None) and (self.subprev.subprev.status in [3, 4.5]):
												pass
											else:
												car = self.subprev
												possible = False
												while car is not None and car.status not in [3, 4.5]:
													if car == self.sys.curr_vehicle and car.status in [1.5, 4.25]:
														if (self.sys.curr_typ == 'prepare_first_exit' and car.status == 4.25) or (self.sys.curr_typ == 'start_pulling_in' and car.status == 1.5):
															possible = True
													car = car.subprev
												if car is not None and car.status in [3, 4.5] and possible:
													pass
												elif self.end_time - end_time < - 1000 * SMALL_INTERVAL:
													pass
												else:
													import pdb; pdb.set_trace()
										else:
											self.end_time = end_time
									self.subtraj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D'))
								else:
									import pdb; pdb.set_trace()
							else:
								self.end_time = iter_t + (self.dest_to_stop - iter_x) / cp.data.v
								if (end_time is not None) and (self.end_time < end_time):
									if not (self.end_time >= end_time - SMALL_INTERVAL):
										if (self.subprev.subprev is not None) and (self.subprev.subprev.status in [3, 4.5]):
											pass
										else:
											car = self.subprev
											possible = False
											while car is not None and car.status not in [3, 4.5]:
												if car == self.sys.curr_vehicle and car.status in [1.5, 4.25]:
													if (self.sys.curr_typ == 'prepare_first_exit' and car.status == 4.25) or (self.sys.curr_typ == 'start_pulling_in' and car.status == 1.5):
														possible = True
												car = car.subprev
											if car is not None and car.status in [3, 4.5] and possible:
												pass
											elif self.end_time - end_time < - 1000 * SMALL_INTERVAL:
												pass
											else:
												import pdb; pdb.set_trace()
									else:
										self.end_time = end_time
								self.subtraj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D'))
							return

						if np.abs(iter_x + (cp.nex.data.t - iter_t) * cp.data.v + CAR_LENGTH - cp.nex.data.x) > SMALL_INTERVAL:
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
					self.subtraj.addEnd( changePoint(iter_x, iter_t, self.driv) )
					self.end_time = iter_t + (self.dest_to_stop - iter_x) / self.driv
					if (end_time is not None) and (self.end_time < end_time):
						if not (self.end_time >= end_time - SMALL_INTERVAL):
							if (self.subprev.subprev is not None) and (self.subprev.subprev.status in [3, 4.5]):
								pass
							else:
								car = self.subprev
								possible = False
								while car is not None and car.status not in [3, 4.5]:
									if car == self.sys.curr_vehicle and car.status in [1.5, 4.25]:
										if (self.sys.curr_typ == 'prepare_first_exit' and car.status == 4.25) or (self.sys.curr_typ == 'start_pulling_in' and car.status == 1.5):
											possible = True
									car = car.subprev
								if car is not None and car.status in [3, 4.5] and possible:
									pass
								elif self.end_time - end_time < - 1000 * SMALL_INTERVAL:
									pass
								else:
									import pdb; pdb.set_trace()
						else:
							self.end_time = end_time
					self.subtraj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D'))
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
			self.subtraj.addEnd( changePoint(start_x, start_t, self.driv) )

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
							if np.abs(iter_x + CAR_LENGTH - prev_loc) < SMALL_INTERVAL:
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
					if not (self.end_time >= end_time - SMALL_INTERVAL):
						if (self.subprev.subprev is not None) and (self.subprev.subprev.status in [3, 4.5]):
							pass
						else:
							car = self.subprev
							possible = False
							while car is not None and car.status not in [3, 4.5]:
								if car == self.sys.curr_vehicle and car.status in [1.5, 4.25]:
									if (self.sys.curr_typ == 'prepare_first_exit' and car.status == 4.25) or (self.sys.curr_typ == 'start_pulling_in' and car.status == 1.5):
										possible = True
								car = car.subprev
							if car is not None and car.status in [3, 4.5] and possible:
								pass
							elif self.end_time - end_time < - 100 * SMALL_INTERVAL:
								pass
							else:
								import pdb; pdb.set_trace()
					else:
						self.end_time = end_time
				self.subtraj.addEnd( changePoint(self.dest_to_stop, self.end_time, 'D'))
				return

			assert (cp.data.v != 'D')
			assert (cp.data.v < self.driv)
			start_x = iter_x
			start_t = iter_t
			if not (prev_loc == start_x + CAR_LENGTH):
				import pdb; pdb.set_trace()

		return
