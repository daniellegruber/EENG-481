# Setup
To set up the environment, open `import_robot.m` and run the script. This will 1) generate the `rigidBodyTree` objects that are required for other files, 2) add certain folders to your working directory, and 3) generate certain variables that are required by other files.

# Overview of folder structure

## URDF

This folder contains the URDF files from which the robotic hand models are derived. These files were downloaded from [this Github repo](https://github.com/dexsuite/dex-urdf/blob/main/robots/hands/shadow_hand) but originate from [this Github repo](https://github.com/shadow-robot/sr_common/tree/noetic-devel) which is directly from the Shadow Robot company. I used the former repo instead of the latter because the former seems to have modified the URDF file to a format that is coincidentally compatible with Matlab/Simulink (it doesn’t have substitution arguments, i.e., stuff like $(...) ).

I modified the original URDF files in the following ways:
1. Replaced obj file references with equivalent stl file references since Matlab/Simulink doesn't use obj files
2. Replaced fixed world_joint with a revolute joint ARMJ1 to allow the specification of palm orientation (e.g., palm facing away from observer for ASL numbers 1-5 and  palm facing toward observer for ASL numbers 6-9)
3. Changed upper bound of THJ2 from 40 deg to 120 deg to better accommodate signs such as the letter e
4. Added prismatic joint ARMJ2 to allow sliding left/right for repeated letters

Finally, note that the hands specified by the original URDF files correspond to [this datasheet](https://www.shadowrobot.com/wp-content/uploads/2022/03/shadow_dexterous_hand_e_technical_specification.pdf) which is from the [Shadow Robot company's website](https://shadow-robot-company-dexterous-hand.readthedocs-hosted.com/en/latest/index.html). 

## meshes

This folder contains the visuals needed to visualize the robotic hands. These files were also downloaded from [this Github repo](https://github.com/dexsuite/dex-urdf/blob/main/robots/hands/shadow_hand). As mentioned above, I had to convert obj files in the meshes folder to stl files using [this site](https://www.makexyz.com/convert/obj-to-stl) because Simulink doesn’t use obj files. Prior to conversion, I got a bunch of warning messages like “Warning: Mesh file 'wrist_E3M5.obj' in link 'wrist' has an unsupported file extension and cannot be visualized. Valid file extensions include STL, STP, and STEP.”

## Base models

This folder contains the Simulink models that directly result from calling `smimport` on the URDF files contained in the URDF folder, e.g., `smimport("URDF/shadow_hand_left_26df.urdf", "ModelName","shl26df");`. You probably won't need to access these models.

Note that the prefixes "shl26df" and "shr26df" in model names correspond to the shadow hand left and shadow hand right models, respectively, modified to have 26 joints instead of the original 24. In other words, models with these prefixes have robots derived from "URDF/shadow_hand_left_26df.urdf" and "URDF/shadow_hand_right_256f.urdf", respectively.

## Quick edit models

These are very similar to the files in the Base models folder, but these are the modifications made:
1. Added input and output ports (and Simulink-PS and PS-Simulink converters, respectively) to the robot
2. Configured the input ports to route desired joint angles to the joints (desired y-distance in the case of ARMJ2) so that you can directly specify desired configurations 
3. Configured the output ports to output the actual signals in case of any mismatch with the input
4. Wired the input ports to a subsystem where you can manually edit joint angles one by one

Note that modifications 1-3 are present in all subsequently described Simulink models.

To see the result of your changes, simply run the simulation. A Mechanics Explorer window should pop up that shows the hand in the updated configuration.

Finally, note that you can run the script `get_joint_values.m` to save the joint values you've manually defined in one of the models of this folder.

## Signals

If you want to visualize a sequence of joint configurations, you can use the helper function `jointValuesToInputSignals` ("Helper functions" folder). Calling this function with the appropriate inputs will generate a mat file containing a Simulink dataset in the Signals folder. This mat file can then be loaded by a model in the "Test signal models" folder, as described below.  

## User input 

Here, the input ports of the Robot system are wired to model-level input ports that can be configured to supply external input. The functions `supplyInputToUserInputMdlByDs` and `supplyInputToUserInputMdlByMat` in the "Helper functions" folder will help you programmatically supply this input via Simulink dataset and mat file, respectively. See `userInputTrajectoryDemo.m` for an example of using these functions.

## Inverse kinematics

The code in this folder generates configurations corresponding to ASL signs programmatically, in particular, using inverse kinematics. See [this doc](https://docs.google.com/document/d/1UxFYyjYZJJsubn2o0A_L4ytDzZd5ZUhHJyOhD95Q6RQ/edit?usp=sharing) for an explanation.

## Trajectory generation

The code in this folder helps generate trajectories between configurations, i.e., trajectories for sequences of signs.

# Trajectory generation

The main function used for trajectory generation (generating trajectories for sequences of signs) is `genConfigTrajectoryFromInput.m` in this folder. The MATLAB `pchip` function is used for interpolation between different signs (sets of joint angles), henceforth referred to as waypoints. As described in the MATLAB documentation, "`p = pchip(x,y,xq)` returns a vector of interpolated values p corresponding to the query points in xq. The values of p are determined by shape-preserving piecewise cubic interpolation of x and y." 

There were four primary considerations in designing this function:
1. When interpolating between two joint angles, use the "shortest-distance angular path." This is the purpose of lines 135-141 in `genConfigTrajectoryFromInput.m`. For example, if the start value of a joint angle is -130 deg and the end value is 180 deg, then instead of interpolating from -130 -> 180, interpolate from 230 -> 180 (since -130 deg is equivalent to 230 deg). This way, the joint rotates through 50 deg rather than 310 deg in the generated trajectory.
2. Signs with movement (e.g., letters j and z) are stored in the "Configs" folder as n x w arrays, where n is the number of joints and w is the number of waypoints. These waypoints should all appear in the overall waypoint array and should correspond to custom time points indicating how fast to proceed through them. This is the purpose of the "setup info" in lines 13-15 of `genConfigTrajectoryFromInput.m`.
3. For letters repeated twice in a row, adjust ARMJ2 of the second sign's waypoint(s) so that interpolation yields a sliding motion. This is achieved by lines 105-109 of `genConfigTrajectoryFromInput.m`.
4. When simply interpolating between the "official waypoints" for signs (the joint angles corresponding to signs), sometimes the trajectories of fingers will interfere with one another. To avoid this, insert special intermediate waypoints to avoid fingers crossing when going from one sign to the next. The script `createTransitionTbl.m` defines a transition table `transitionTbl` that specifies which letter -> letter transitions need intermediate waypoints inserted and at what proportion of the way from the start -> end sign they should "be hit." (Note that the code in the "Inverse kinematics/Generate letter transitions" folder is used  to create these intermediate waypoints.) The `transitionTbl` generated by `createTransitionTbl.m` is used in `genConfigTrajectoryFromInput.m`. For a visual explanation of the approach to interference avoidance in trajectory generation, see [these slides.](https://docs.google.com/presentation/d/1qhWpDE0XMEH9NNLhkwlcIzZ_LFKR7924oTQJtnU-RWQ/edit?usp=sharing)  
