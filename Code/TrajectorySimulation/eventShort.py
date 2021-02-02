import sys
from math import ceil, floor, sqrt
import numpy as np
from heapq import *
from utils import *
from params import *
from event import *


VEHICLE_IDX = None

class systemShort(system):

	def __init__(self, N, seedSERV = None, seedPOUT = None, seedPLIN = None, seedDRIV = None):
		super(systemShort, self).__init__(N, seedSERV, seedPOUT, seedPLIN, seedDRIV)

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

		################################################# 0-degree Short Specific #################################################
		# if the vehicle has arrived at the desired destination while its enter maneuver blocked by vehicles on the through lane
		# i.e. 1) if its immediate proceeding vehicle is having an exit maneuver at the spot in front
		# if (curr_vehicle.prev is not None) and (curr_vehicle.prev.stop == curr_vehicle.stop + 1) and (curr_vehicle.prev.status == 5):
		# 	assert curr_vehicle.prev.pout_end > self.curr
		# 	delayed = True 
		# 	req_time = max( req_time, curr_vehicle.prev.pout_end )
		
		# or 2) if the immediate proceeding vehicle is stopped, or having an enter maneuver into the spot in front.
		# if (curr_vehicle.prev is not None) and (curr_vehicle.status not in [2, 5]) and (curr_vehicle.prev.curr_loc <= curr_vehicle.dest_to_stop + 1.5 * CAR_LENGTH):
		# 	cp = curr_vehicle.prev.traj.head
		# 	if cp.data.t > self.curr:
		# 		if cp.data.t <= self.curr + 2 * SMALL_INTERVAL:
		# 			delayed = True
		# 			req_time = max( req_time, cp.data.t )
		# 		else:
		# 			import pdb; pdb.set_trace()
		# 	else:
		# 		while (cp.nex is not None) and (cp.nex.data.t <= self.curr):
		# 			cp = cp.nex
		# 		assert cp is not None
		# 		if (cp.data.v == 0.0):
		# 			assert cp.nex.data.t > self.curr
		# 			delayed = True
		# 			req_time = max( req_time, cp.nex.data.t )
		# 		else:
		# 			assert curr_vehicle.prev.status != 2
		###########################################################################################################################
		
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

		for k in in_range(curr_vehicle.stop_idx, self.N):
			if (self.inservice[k - 1] is not None) and (self.inservice[k - 1].status == 2):
				# if curr_vehicle.idx == VEHICLE_IDX and self.inservice[k - 1].plin_end - min(meanDRIV, max(0, self.inservice[k - 1].plin_time - .5)) > req_time:
				# 	delay_reason = 1
				# 	delay_status = 2
				# req_time = max( req_time, self.inservice[k - 1].plin_end - min(meanDRIV, max(0, self.inservice[k - 1].plin_time - .5)) )
				if curr_vehicle.idx == VEHICLE_IDX and self.inservice[k - 1].plin_end > req_time:
					delay_reason = 1
					delay_status = 2
				req_time = max( req_time, self.inservice[k - 1].plin_end )


		for k in out_range(curr_vehicle.stop_idx, self.N):
			if (self.inservice[k - 1] is not None) and (self.inservice[k - 1].status == 5):
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
			delayed, req_time, prev, delay_reason, delay_status, delay_speed = self.check_lane_short(curr_vehicle, first_attempt, delay_reason, delay_status, delay_speed)
		else:
			delayed, req_time, prev = self.check_lane_short(curr_vehicle, first_attempt)

		if delayed:
			if not req_time > self.curr:
				import pdb; pdb.set_trace()
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
				assert car.stop >= 2
				if ( (car.stop - 1) * LOT_LENGTH / rateDRIV >= max(0.0, meanPLIN - (self.curr - car.plin_start)) ):
					j_new.append(j)

			elif car.status == 5:
				# K_out with K = car.j and J = idx2spot(j)
				if ( (car.stop - 1) * LOT_LENGTH / rateDRIV >= max(0.0, meanPOUT - (self.curr - car.pout_start)) ):
					j_new.append(j)

			else:
				assert car.status == 1
				# I_in with K = car.j and J = idx2spot(j)
				assert car.stop_idx != j
				assert car.stop >= 2
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
				assert ( (car.stop - 1) * LOT_LENGTH / rateDRIV < meanPLIN - (self.curr - car.plin_start) )
				car_time = meanPLIN - (self.curr - car.plin_start) - (car.stop - 1) * LOT_LENGTH / rateDRIV

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
					if ( (car.stop - 1) * LOT_LENGTH / rateDRIV < max(0.0, meanPLIN - (self.curr - car.plin_start)) ):
						j_new = j_new[:idx]
						if idx == 0:
							car_time = max(car_time, meanPLIN - (self.curr - car.plin_start) - (car.stop - 1) * LOT_LENGTH / rateDRIV)
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

	def check_lane_short(self, curr_vehicle, first_attempt, delay_reason = None, delay_status = None, delay_speed = None):

		assert angle == 0 and mode == 'short'

		delayed = False
		req_time = self.curr

		car = self.head
		prev = None
		stopped = False

		stop = curr_vehicle.stop
		idx = curr_vehicle.idx

		while car != None:
			car.update_loc()

			if car.stop > stop and car.status == 2:
				assert car.plin_start <= self.curr
				if meanPOUT + (car.stop - 1 - stop) * LOT_LENGTH / rateDRIV + SMALL_INTERVAL < max(0., meanPLIN - (self.curr - car.plin_start)):
					assert meanPLIN - (self.curr - car.plin_start) > 0.
					assert meanPLIN - (self.curr - car.plin_start) > meanPOUT + (car.stop - 1 - stop) * LOT_LENGTH / rateDRIV
					delayed = True
					req_time = max( req_time, self.curr + meanPLIN - (self.curr - car.plin_start) - (meanPOUT + (car.stop - 1 - stop) * LOT_LENGTH / rateDRIV))
				else:
					prev = car

			elif car.status == 5 and car.stop == stop + 1:
				if self.curr < meanDRIV + car.pout_start:
					assert car.pout_end > self.curr
					delayed = True
					req_time = max( req_time, min(car.pout_start + meanDRIV, car.pout_end) )
				else:
					prev = car

			elif car.curr_loc >= stop * LOT_LENGTH + CAR_LENGTH - SMALL_INTERVAL:
				prev = car

			# elif car.curr_loc >= stop * LOT_LENGTH + CAR_LENGTH - SMALL_INTERVAL:
			# 	assert car.status in [1, 6]
			# 	cp = car.traj.head
			# 	if cp.data.t > self.curr:
			# 		import pdb; pdb.set_trace()
			# 	while cp.nex is not None and cp.nex.data.t <= self.curr:
			# 		cp = cp.nex
			# 	if cp.nex is None:
			# 		if car.status == 1 and car.curr_loc == car.dest_to_stop and np.abs(cp.data.t - self.curr) < SMALL_INTERVAL:
			# 			delayed = True
			# 			req_time = max( req_time, self.curr + SMALL_INTERVAL )
			# 		else:
			# 			import pdb; pdb.set_trace()
			# 	else:
			# 		assert cp.nex.data.t > self.curr >= cp.data.t
			# 		if cp.data.v > 0.0:
			# 			prev = car
			# 		else:
			# 			assert cp.nex.data.t > self.curr
			# 			delayed = True
			# 			req_time = max(req_time, cp.nex.data.t)

			# elif car.status == 2 and car.stop in [stop, stop - 1]:
			# 	assert car.stop_idx != curr_vehicle.stop_idx
			# 	req_time = max( req_time, car.plin_end )
			# 	if req_time > self.curr:
			# 		delayed = True
			# 	else:
			# 		break

			elif stopped and car.status != 5:
				pass

			elif car.status == 5: 
				if car.stop >= stop - 1 and car.pout_end != self.curr:
					import pdb; pdb.set_trace()
				assert car.dest_to_stop >= stop * LOT_LENGTH + CAR_LENGTH
				assert car.pout_start <= self.curr
				car_time = max(0, meanPOUT - (self.curr - car.pout_start)) + ((stop - 1) * LOT_LENGTH - car.curr_loc) / rateDRIV
				if car_time < meanPOUT or car.stop >= stop - 1:
					delayed = True
					car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH )
					assert car_time > self.curr
					if idx == VEHICLE_IDX and car_time > req_time:
						delay_reason = 3
						delay_status = 5
					req_time = max( req_time, car_time )

			elif car.status == 2:
				assert car.stop < stop - dgap or car.plin_end == self.curr
				if car.plin_end > self.curr:
					stopped = True

			elif car.status == 1 and car.stop_idx == curr_vehicle.stop_idx:
				assert (car.curr_loc < car.dest_to_stop) and (car.curr_loc < (stop - 1) * LOT_LENGTH + SMALL_INTERVAL)
				break

			elif car.status == 1 and car.stop < stop:
				assert (car.curr_loc <= car.dest_to_stop <= (stop - 1) * LOT_LENGTH)
				stopped = True

			else:
				assert (car.status == 6) or (car.status == 1 and car.stop >= stop and car.stop_idx != curr_vehicle.stop_idx)
				assert car.dest_to_stop >= stop * LOT_LENGTH + CAR_LENGTH or car.stop == stop
				car_time = ((stop - 1) * LOT_LENGTH - car.curr_loc) / rateDRIV
				if car_time < meanPOUT:
					delayed = True
					car_time = car.calc_time( stop * LOT_LENGTH + CAR_LENGTH )
					if car_time == self.curr:
						assert car.stop == stop
						car_time += SMALL_INTERVAL
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