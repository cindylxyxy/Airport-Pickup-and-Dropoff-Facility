This file documents the simulation details for configurations that allow double parking.

To run the simulation for double parking configurations, in your computer or server, first make sure that you are under the correct directory '../PickupDropoffFacility/Code_updated_20210531/DoubleParking/', then use the command 'python simulation.py' to run the simulation. 

The input parameters are initiated in the file 'params.py'. For a full list of input to the simulation model, please refer to our paper. We will only include here a list of input that is easy to change and experiment for the general audience who hopes to see the simulation results under a spectrum of settings. Changes to the other items in the input list will require thorough understanding towards the model, careful calibration of the input data, and very likely but not necessarily some debugging. Such items include but not are not limited to the follows: The simulation for double parking assumes that each boarding spot is configured similarly to a 0-degree long spot, in terms of the orientation, width, and length of each spot. Only partial access control is considered for double parking mode. For a hassle-free start, you can work with the following input parameters. Note that the first alternative for each item is the default as written in the 'params.py' file.
  
  1. version = 'v1' or 'v2' for spot assignment rule, with 'v1' for Alternative 1 (as explained below) and 'v2' for Alternative 2. 
	
  2. meanSERV = 60 for mean service time, other values as mentioned in our paper include 120s, 150s, 180s, and 300s.
	
  3. simType = 'exp' for service time distribution being Exponential, vehicle desired speed distribution being Continuous Uniform, and enter/exit maneuver time distribution being Exponential. Other alternatives are 'cav' for service time distribution being Exponential, vehicle desired speed distribution being Constant, and enter/exit maneuver time distribution being Constant; and 'unif' for service time distribution being Continuous Uniform, vehicle desired speed distribution being Continuous Uniform, and enter/exit maneuver time distribution being Continuous Uniform.
	
  4. SIM_HOUR = 20 for duration of simulation in each iteration being 20 hrs. You may change to any integer of choice. Please make sure that your input is in integral form.
	
  5. SIM_ITER = 20 for the number of iterations in the simulation being 20. You may change to any integer of choice. Please make sure that your input is in integral form.
	
  6. dirname = '' for any desired directory to save the output. You may change to any directory of choice. 
	
  7. suffix = '' for any desired suffix to differentiate the output files of multiple simulation runs with different input combinations. You may change to any suffix of choice. 

The 'eventDP1.py' file specifies the event definitions, tracks the vehicle status, and updates the vehicle trajectories under spot assignment rule - Alternative 1. The 'eventDP2.py' file is a variation under Alternative 2. The 'utils.py' file stores the helper functions and objects.

The detailed dynamics of the trajectory-based simulation for Double Parking simulation are recorded below. 

	1. Lanes.
	The simulation assumes that there are 3 lanes in the facility: A curbside lane for vehicles to stop and load/unload passengers, a maneuvering lane in the middle of the three (also referred to as the second lane), and a through lane for vehicles to access the facility. Notably, vehicles can double park on the middle lane only when the curb side lane is fully occupied. The middle lane is also a through lane which allows vehicles to enter and exit the facility; however, for the purpose of clarity, this document refers to the through lane exclusively as the side lane on which no vehicle stops to load/unload passengers. The choice over the middle and the through lanes of any vehicle when entering and exiting the facility will be elaborated below.

	2. Boarding spots.
	The replacement vehicles enter the system with an assigned destination boarding spot. The assigned stop for each vehicle is stored in the 'stop' attribute of the 'vehicle' object. The assignment rule chooses a stop with at least one vacancy on the curb or on the middle lane, and the assignment of a stop does not specify in which lane a vehicle should stop and load/unload passengers. Rather, the vehicle would drive forward on either the through lane or the middle lane until a location where it can start maneuvering. If the curb side lane has an available spot with the given index, the vehicle primarily maneuver into the curb side lane. Otherwise it stops in the middle lane and 'parks' beside a vehicle on the curb. So far, the descriptions here apply to version 1 i.e. for spot assignment rule - Alternative 1. In Alternative 2, each spot owns a unique spot index which differentiates itself from a parallel one on the curbside lane (if the spot itself is on the middle lane) or on the middle lane (if the spot itself is on the curb side). The assignment is done such that, when the curb is not fully occupied, an available spot that is closest to the exit on the curb is chosen; and when the curb is no long available, an available spot that is closest to the exit on the middle lane is chosen. Note that even in Alternative 2, vehicles assigned to the middle lane will still maneuver in to the curbside lane should the spot next to its assigned on the curbside lane becomes available after the initial assignment and before the service starts on the middle lane.

	3. Vehicle Status.
	The vehicle status takes a value from
		=> 0 if the replacement vehicle is created;
		=> 1 if the vehicle has entered the system and been assigned a spot
		=> for vehicles assigned to the curbside lane:
				1.5 if the vehicle has started the first enter maneuver out of the 2
				2   if the vehicle has started the second enter maneuver out of the 2
		=> for vehicles assigned to the middle lane:
				2   if the vehicle has started the only enter maneuver 
		=> 3 if the vehicle has finished its enter maneuver(s) and started service
		=> 4 if the vehicle has finished service
		=> for vehicles assigned to the curbside lane:
				4.5 if the vehicle has started the first exit maneuver out of the 2
				5   if the vehicle has started the second exit maneuver out of the 2
		=> for vehicles assigned to the middle lane:
				4.5   if the vehicle has started the only exit maneuver 
		=> 6 if the vehicle has finished its exit maneuver(s) and is driving towards the system exit

	3.1. Vehicle attributes on the curbside lane.
	The vehicles on the curbside lane can possibly have 4 types of status: last step of enter maneuver (2), in service (3), service completed but blocked from exit maneuver (4), and first step of exit maneuver (4.5). 
	The objects for vehicles on the curbside lane can be accessed by calling
		self.inservice[0][stop - 1]
	where #stop# is a variable that stores the stop of the vehicle as defined earlier. 
	All such vehicles should have their #lane# as 0. 
	While vehicles have a status of 3 or 4, it should not have #prev#, #nex#, #subprev#, #subnex#, #traj#, and #subtraj#.
	While vehicles have a status of 2 or 4.5, it should only have #subprev#, #subnex#, and #subtraj#.

	3.2. Vehicle attributes on the middle lane.
	The vehicles on the middle lane can have all types of status: entering the system (1), first step of enter maneuver (1.5), second/last step of enter maneuver (2), in service (3), service completed but blocked from exit maneuver (4), first step of exit maneuver from the curb side lane (4.5), last step of exit maneuver (5: either between the curb side and the middle lanes then #lane# is 0, or between the middle and the through lanes), and leaving the system (6). 	
	The objects for vehicles on the middle lane can be accessed by calling
		self.inservice[1][stop - 1]
	where #stop# is a variable that stores the stop of the vehicle as defined earlier. 
	All such vehicles should have their #lane# as 1. 
	While vehicles have a status of 3 or 4, it should not have #prev#, #nex#, #subprev#, #subnex#, #traj#, and #subtraj#.
	While vehicles have a status of 2 or 4.5, it should only have #subprev#, #subnex#, and #subtraj#.
	While vehicles have a status of 1.5 or 5, it should only have #prev#, #nex#, and #traj#.

	4. System Events
	The system has 8 different event types as described below. Each event is associated with a specific vehicle, and the vehicle has been assigned a stop except for the event 'enter_system'. There is also a priority order among the events, i.e. when events of different types are scheduled at the same time, some types of events will be processed with priority. Notably, the event priority can be given as an input to the program but it is advised that you should not change the priority order. The event decriptions below is organized according to decreasing priorty of events, which is implemented currently.
	5.1. 'leave_system': 
	At this event, the associated vehicle (a.k.a. the current vehicle or curr_vehicle) can be removed from the system if it has arrived at the system exit point (i.e. the end of either through lane). If the vehicle was delayed after this event had been scheduled s.t. it has not arrived at the exit point, the event will be pushed to a future time calculated based on the updated trajectory of this vehicle. Otherwise, the vehicle leaving the system must be a leading vehicle either on the through lane (i.e. curr_vehicle == self.head) or the middle lane (i.e. curr_vehicle == self.subhead). Then the following operations will happen: 1) The current vehicle is removed from the system and the leading vehicle on its lane is changed to its immediate follower (if there exists one). 2) The departures from the system is augmented by 1 (i.e. self.outCount += 1). 3) The total, productive and idle system times as well as the percentage idle time of the current vehicle are calculated and recorded.   
	5.2. 'finish_pulling_out':
	At this event, the associated vehicle finishes the exit maneuver from the middle lane to the through lane. This event might be scheduled for vehicles servicing on either the middle or the curb side lane. This event should never be rescheduled as exit maneuvers should not be delayed once started. The following operations will happen: 1) The current vehicle will change its status from 5 to 6, marking the end of its exit maneuver. (Notably, it is not necessary to update its trajectory since t)
	5.3. 'prepare_pulling_out':
	5.4. 'start_service'
	5.5. 'start_second_enter'
	5.6. 'prepare_first_exit'
	5.7. 'start_pulling_in'
	5.8. 'enter_system'
