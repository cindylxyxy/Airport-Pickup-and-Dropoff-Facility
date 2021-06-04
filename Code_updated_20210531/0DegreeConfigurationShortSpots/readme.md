This file documents the simulation details for 0-degree configurations with short spots.

To run the simulation, in your computer or server, first make sure that you are under the correct direcotry '../PickupDropoffFacility/Code_updated_20210531/0DegreeConfiguratioShortSpots/', then use the command 'python run simulation.py' to start the simulation.

The input parameters are initiated in the file 'params.py'. For a full list of input to the simulation model, please refer to our paper. We will only include here a list of input that is easy to change and experiment for the general audience who hopes to see the simulation results under a spectrum of settings. Changes to the other items in the input list will requre thorough understanding towards the model, careful calibration of the input data, and very likely but not necessarily some debugging. For a hassle-free start, you can work with the following input parameters. Note that the first alternative for each item is the default as written in the 'params.py' file.

  1. side = 'single' for Single-Sided 0-degree Configuration with short spots, and you may change to Double-Sided 0-degree Configuration by replacing the value with 'double'.
  2. control = 'partial' for partial access control, and you may change to full access control by replacing the value with 'full'. 
  3. meanSERV = 60 for mean service time, other values as mentioned in our paper include 120s, 150s, 180s, and 300s.
  4. simType = 'exp' for service time distribution being Exponential, vehicle desired speed distribution being Continuous Uniform, and enter/exit maneuver time distribution being Exponential. Other alternatives are 'cav' for service time distribution being Exponential, vehicle desired speed distribution being Constant, and enter/exit maneuver time distribution being Constant; and 'unif' for service time distribution being Continuous Uniform, vehicle desired speed distribution being Continuous Uniform, and enter/exit maneuver time distribution being Continuous Uniform.
  5. SIM_HOUR = 20 for duration of simulation in each iteration being 20 hrs. You may change to any integer of choice. Please make sure your input is in integral form.
  6. SIM_ITER = 20 for the number of iterations in the simulation being 20. You may change to any integer of choice. Please make sure your input is in integral form.
  7. dirname = '' for any desired directory to save the output. You may change to any directory of choice.
  8. suffix = '' for any desired suffix to differentiate the output files of multiple simulation runs with different input combinations. You may change to any suffix of choice.

The 'event.py' and 'eventShort.py' files jointly specify all the event definitions, track the vehicle status, and update the vehicle trajectories. The 'utils.py' file stores helper functions and objects.
