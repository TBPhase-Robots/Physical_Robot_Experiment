# Simulation Machine Code User Guide


# runSimulation

The central hub for all the code for running the robot control simulation. Works alongside all other files in the 'Experiments/Python Model Event Driven' directory.
All agents are simulated in this script. Agents are added and removed with commands from the robot server via ROS. Real world robots are avatars of these agents.

The runSimulation agent control framework is a state machine. The user is able to change the state of the state machine via the StateController.py user interface,
or by sending the appropriate commands to the appropriate ROS topics.

runSimulation loads a default json configuration file to describe the parameters for each experiment trial. The config file is loaded upon initialisation and
defines a behaviours such as agent repulsion forces, simulation space dimensions, and whether a local or real world simulation should be run.


TO USE:

Run 'runSimulation.py' by command line with the python interpreter on a linux system.


    'python3 runSimulation.py'

Now launch 'StateController.py' from another terminal.


    'python3 StateController.py'

# StateController

The User Interface for setting the state machine's state in runSimulation, as well as sending commands.

In StateController, the user is able to iterate through all trial setup steps - sending agents (and by extension physical robots)
to their startup positions defined in the current configuration file.

The user is able to set all active agents to standby, as well as change the current configuration file.


TO USE:

Run 'runSimulation.py' by command line with the python interpreter on a linux system (if it is not already running).
eg 'python3 runSimulation.py'

Now launch 'StateController.py' from another terminal.
'python3 StateController.py'

WARNING: Only click the 'add agent' button if the simulation is running locally, denoted with the configuration file flags:
'event_driven_lock_movements' and 'event_driven' being set to false.

For local simulations:

    Click the 'add agent' button until the desired number of agents has been added.
    Click 'setup start and sheep' to assign agent roles based on configuration file. Sheep will make their way to destination with pathfinding.
    Once sheep are in position, click 'dog setup loop'.
    Once dogs are in position, click 'pig setup loop'. Pigs are defined as any excess agents that are not needed for the current experiment.
    Once pigs are in position, click 'standby setup loop' for all standby agents (potential dogs) to be sent to their positions.
    Once all are in position, click the 'experiment' button.

    To change the configuration file, you must first click 'set all agents to standby', optionally followed by the standby setup loop.
    Type the name of the new configuration file (minus the .json extension) to the text field and click 'set new JSON'.

For physical robot experiments:

    Wait until all physical robots are registered and have appeared on screen.
    Click the 'add agent' button until the desired number of agents has been added.
    Click 'setup start and sheep' to assign agent roles based on configuration file. Sheep will make their way to destination with pathfinding.
    Once sheep are in position, click 'dog setup loop'.
    Once dogs are in position, click 'pig setup loop'. Pigs are defined as any excess agents that are not needed for the current experiment.
    Once pigs are in position, click 'standby setup loop' for all standby agents (potential dogs) to be sent to their positions.
    Once all are in position, click the 'experiment' button.

    To change the configuration file, you must first click 'set all agents to standby', optionally followed by the standby setup loop.
    Type the name of the new configuration file (minus the .json extension) to the text field and click 'set new JSON'.

