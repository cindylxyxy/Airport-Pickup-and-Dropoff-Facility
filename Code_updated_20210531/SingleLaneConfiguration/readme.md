This file documents the simulation details of Single Lane Configuration.

To run the simulation for Single Lane Configuration, in your computer or server, first make sure that you are under the correct directory '../PickupDropoffFacility/Code_updated_20210531/SingleLaneConfiguration/', then use the command 'python simulation.py' to run the simulation.

The input parameters are initiated in the file 'params.py'. For a full list of input to the simulation model, please refer to our paper. We will only include here a list of input that is easy to change and experiment for the general audience who hopes to see the simulation results under a spectrum of settings. Changes to the other items in the input list will require thorough understanding towards the model, careful calibration of the input data, and very likely but not necessarily some debugging. Such items include but not are not limited to the follows: The simulation for Single Lane assumes that each boarding spot has 0-degree orientation, with length and width recorded in our paper. For a hassle-free start, you can work with the following input parameters. Note that the first alternative for each item is the default as written in the 'params.py' file.

  1. meanSERV = 60 (seconds) for mean service time, other values as mentioned in our paper include 120s, 150s, 180s, and 300s.
  2. simType = 'exp' for service time distribution being Exponential, vehicle desired speed distribution being Continuous Uniform, and enter/exit maneuver time distribution being Exponential. Other alternatives are 'cav' for service time distribution being Exponential, vehicle desired speed distribution being Constant, and enter/exit maneuver time distribution being Constant; and 'unif' for service time distribution being Continuous Uniform, vehicle desired speed distribution being Continuous Uniform, and enter/exit maneuver time distribution being Continuous Uniform.
  3. SIM_HOUR = 20 for duration of simulation in each iteration being 20 hrs. You may change to any integer of choice. Please make sure that your input is in integral form.
  4. SIM_ITER = 20 for the number of iterations in the simulation being 20. You may change to any integer of choice. Please make sure that your input is in integral form.
  5. dirname = '' for any desired directory to save the output. You may change to any directory of choice.
  6. suffix = '' for any desired suffix to differentiate the output files of multiple simulation runs with different input combinations. You may change to any suffix of choice.

The 'eventSingleNew.py' file specifies the event definitions, tracks the vehicle status, and updates the vehicle trajectories. The 'utils.py' file stores the helper functions and objects.
