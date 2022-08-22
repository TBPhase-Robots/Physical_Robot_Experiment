# Configuration File User Guide

# Example Config File:

The config file is used to set the many parameters that may be defined during a robot experiement trial.

It affects what happens in on-screen simulation space.

This file may be changed during runtime via the StateController, although stability is not always guaranteed.

Currently, not all the values in this file are the most up to date. Those that are not up do date or are currently unused will
have a warning note attached in the following description.






            "world_width" : 1280,
            "world_height" : 720,

world_width and world_height are the dimensions of the window used in the simulation. Currently it is in a 16:9 aspect ration.
In the simulation, scales and all movement vector forces are calculated in the 2d pixel space. In the rest of the code, this may
also be defined as 'screen space'. As such, although these dimensions *may* be changed, it is not advised to do so as there is no
standard calculation scaling implemented as of yet. Agents will be treated as closer together at lower resolutions and farther apart
at higher resolutions. Presently, parameters are calibrated for 1280x720.

            "play_area_x" : 25,
            "play_area_y": 25,
            "play_area_width" : 1100,
            "play_area_height" : 690, 

NOTE: Play area is in the process of being replaced by arena markers.

Play area defines an area of screen space in which it is desirable for agents (particularly sheep) to stay within. 
Any agents that leave the area experience an overhwelming force to return inside.
The area is a rectangle defined from a top left coordinate in screen space with width and height extending from this point.

            "time_limit" : 5000,

Time limit in seconds until the trial ends itself.

            "path_finding_width" : 16,
            "path_finding_height" : 9,

path_finding_width and path_finding_height are used in the PathfindingManager class. Pathfinding is used during the setup phases
to create a path for agents to move to their start positions with obstacle avoidance.

Screen space is divided into an x by y grid with dimensions defined by path_finding_width and path_finding_height.
A pathfinding matrix of this dimension is created. Any screen tile with a stationary agent inside it is defined as a negative value
in the matrix.

When routes are plotted using pathfinding, agents travel to the midpoint of each tile.


            "show_empowerment" : true,

Show empowerment colours on dog agents in screen simulation space. Colours range from red to green. 

            "cam_width" : 2.2,
            "cam_height" : 1.5,

cam_width and cam_height define physical bounds that the (real world) camera is able to see measured in metres.
Incoming pose data is described in metres. This must be converted to screen space pixels. Alongside world_width and world_height,
cam_width and cam_height  allow for the conversion between metres and screen space.

            "event_driven" : true,
            "event_driven_lock_movements" : true,

event_driven and event_driven_lock_movements must be set to true if the simulation is going to be run networked with robots.
Setting both flags to false allows for local simulation.

event_driven halts the execution of the simulation until any robot pose update has been received via ROS. The simulation calculates new force vectors for all agents
and publishes to each robot.

event_driven_lock_movements prevents agents moving themselves using their local behaviour within the simulation. With this flag set to true, no agent positions may be updated by any method aside from a robot pose update.



            "debug_dog_forces" : false,
            "debug_sheep_forces" : false,

Draws debug lines for the forces on dog and sheep agents during the experiment state. This includes the multiple vector components required for dog movements, such as
the driving forces and orbital forces.

            "debug_steering_points" : false,

Draws the steering points for each dog agent during the experiment state. Steering points are the point to which dog agents attempt to move towards after summing all their component vectors.

            "debug_sub_flocks" : false,

Each dog may be responsible for a sub-flock of sheep. Draw these sub-flocks to screen.

            "debug_sheep_states" : false,
            "debug_dog_states" : false,

Sheep may be grazing or in the process of being herded. Dogs may be driving or collecting flocks. Colour code these agents.


            
            "empowerment_type" : 1,

Select the empowerment approximation type to be used in the experiment.

            "initial_sheep_positions" : [[400, 500], [300, 500], [240, 350]],
            "initial_dog_positions" : [[440, 350], [340, 350]],
            "standby_positions" : [[150, 100], [350, 100], [550, 100], [750, 100], [950, 100], [150, 175], [350, 175], [550, 175], [750, 175], [950, 175]],
            "pigsty_positions" : [[50, 50], [50, 200], [50, 350], [50, 500], [75, 650]], 

Define the initial positions agents must go to at the start of each trial. These are defined in screen space coordinates measured from top-left.

The maximum number of sheep in the simulation is defined by the length of initial_sheep_positions.

The number of dogs in play at the start of the trial is defined by the length of initial_dog_positions.

standby_positions defines the number of spaces available for dogs not in play to wait until they are dispatched. The maximum number of dogs and standby agents
cannot exceed 5 in the experiment phase.

pigsty_positions defines where pig agents should go during the experiment trial. Pig agents are defined as agents that are redundant during the experiment trial.
These agents will never be dispatched. 


            "sheep_resulsion_from_dogs" : 5000,
            "sheep_repulsion_from_sheep" : 2000,
            "sheep_attraction_to_sheep" : 1,

            "lambda_D" : 0.1,   
            "lambda_S" : 0.6,
            "lambda_G" : -1,

Values used to describe the coefficients and exponential falloffs used in the attraction and repulsion calculations between agents during the experiment.
The lambda values describe the negative exponent used in calculating their respective forces, wheras the other values are flat force multipliers.

lambda_S and sheep_repulsion_from_sheep help describe the sheep-sheep repulsion mechanics.

lambda_G and sheep_attraction_to_sheep help describe the sheep-sheep attraction mechanics

lambda_D and sheep_resulsion_from_dogs help describe the repulsion force of all dogs in the simulation on each sheep agent.


            "no_of_sheep_in_social_group" : 3,

Size of sheep social groups. Used for group behaviour dynamics. 

            "sheep_vision_range" : 110,

Range during experiment that sheep agents are able to detect, and therefore react to dog agents. Measured in screen space. 

            "grazing_movement_chance" : 0.3,

Chance per config property [ticks_per_choice] that a sheep will choose another random direction to move while grazing. 

            "dog_forces_with_flock" : 1,

dog_forces_with_flock is the multiplier to the force at which dog agents drive towards the flock.

            "dog_repulsion_from_dogs" : 2,

dog_repulsion_from_dogs is the multiplier to the force at which dog agents push away from each other.

            "dog_repulsion_from_sheep" : 1,

dog_repulsion_from_sheep is the multiplier to the force dogs experience when moving close to sheep.

            "dog_attraction_to_steering_point" : 4,

dog_attraction_to_steering_point is the multiplier to the force dogs experience pulling them towards their steering point.

            "dog_orbital_around_flock" : 8,

dog_orbital_around_flock is the multiplier to the force dogs experience orthogonal to the flock to aid in collecting behaviour.


            "agent_repulsion_from_agents" : 0.5,

NOTE: This value is not currently enabled in the most recent build (as of 22/08/2022). Feature may be removed to create a more robost avoidance system in conjunction with pathfinding that moves blocking agents out the way,
instead of agents trying to somewhat avoid them themself.

agent_repulsion_from_agents is a multiplier coefficient used to create a self-repulsive force for rudimentary obstacle avoidance in the Agent class' MoveToPoint method.


            "move_to_point_repel_distance" : 90,

NOTE: This value is not currently enabled in the most recent build (as of 22/08/2022). Feature may be removed to create a more robost avoidance system in conjunction with pathfinding that moves blocking agents out the way,
instead of agents trying to somewhat avoid them themself.
Distance between agent centres before a repulsive force can be created for rudimentary obstacle avoidance in the Agent class' MoveToPoint method.



            "sequential_pathfinding" : true,

The sequential_pathfinding flag defines if, in the setup phase where agents are moving to their initial positions, agents should move simultaneously or sequentially. Simultaneously is faster but more prone to collisions.


            "agent_radius" : 50,

Radius of agents in screen space pixels on screen.
            
            "ticks_per_choice" : 5,

Simulation update ticks required before sheep agent may choose to graze in a difference direction.
            
            "collection_radius" : 70,

Collection radius around dog agent during its collecting behaviour 

            "max_number_of_dogs" : 5,

Maximum number of dog agents allowed at any one time in trial. Includes standby agents.

            "driving_distance_from_flock_radius" : 20,

Distance at which dog agents should attempt to remain behind the flock that they are driving. Measured in screen space.

            "collection_distance_from_target_sheep" : 10,

Used to calculate the distance of a dog agent's steering point from the farthest sheep's position during collecting behaviour.

            "realistic_agent_movement_markers" : true,
            "realistic_agent_movement" : true,

realistic_agent_movement_markers and realistic_agent_movement are only applicable when simulating locally - ie when event_driven and event_driven_lock_movement are both set to false. 

realistic_agent_movement_markers debugs the agent's current rotation and target direction vector with blue and black lines respectively.

realistic_agent_movement forces the agents to move with an approximation of differential drive instead of omni-drive movement.

            "robot_move_to_point_braking_distance" : 150,

robot_move_to_point_braking_distance describes the distance in screen space that an agent should start slowing down during the Agent.MoveToPoint method as it approaches its target location.

            "robot_dog_max_speed" : 0.75,
            "robot_sheep_max_speed" : 0.5,
            "robot_pig_max_speed" : 0.4,
            "robot_standby_max_speed" : 1.0

            These final values are not used because Jed was doing a lot and then we left. They were supposed to be used for setting robot speed limits. The code is fully implemented simulation end, but not on the robots. These values would be sent via ROS to rbots when they are assigned a new role, acting as multipliers to motor PWM outputs.
           



