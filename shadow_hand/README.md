# Setup
To set up the environment, open import_robot.m and run the script. This will 1) generate the `rigidBodyTree` objects that are required for other files, 2) add certain folders to your working directory, and 3) generate certain variables that are required by other files.

# Overview of folder structure

## URDF

This folder contains the URDF files from which the robotic hand models are derived. These files were downloaded from [this Github repo](https://github.com/dexsuite/dex-urdf/blob/main/robots/hands/shadow_hand) but originate from [this Github repo](https://github.com/shadow-robot/sr_common/tree/noetic-devel) which is directly from the Shadow Robot company. I used the former repo instead of the latter because the former seems to have modified the URDF file to a format that is coincidentally compatible with Matlab/Simulink (it doesn’t have substitution arguments, i.e., stuff like $(...) ).

The URDF files are slightly modified to use stl files instead of obj files since Matlab/Simulink doesn't use obj files.

Finally, note that the hands specified by the URDF files correspond to [this datasheet](https://www.shadowrobot.com/wp-content/uploads/2022/03/shadow_dexterous_hand_e_technical_specification.pdf) which is from the [Shadow Robot company's website](https://shadow-robot-company-dexterous-hand.readthedocs-hosted.com/en/latest/index.html). 

## meshes

This folder contains the visuals needed to visualize the robotic hands. These files were also downloaded from [this Github repo](https://github.com/dexsuite/dex-urdf/blob/main/robots/hands/shadow_hand). As mentioned above, I had to convert obj files in the meshes folder to stl files using [this site](https://www.makexyz.com/convert/obj-to-stl) because Simulink doesn’t use obj files. Prior to conversion, I got a bunch of warning messages like “Warning: Mesh file 'wrist_E3M5.obj' in link 'wrist' has an unsupported file extension and cannot be visualized. Valid file extensions include STL, STP, and STEP.”

## Base models

This folder contains the Simulink models that directly result from calling `smimport` on the URDF files contained in the URDF folder, e.g., `smimport("shadow_hand_left.urdf", "ModelName","shadow_hand_left");`. You probably won't need to access these models.

## Quick edit models

These are very similar to the files in the Base models folder, but these are the modifications made:
1. Added input and output ports (and Simulink-PS and PS-Simulink converters, respectively) to the robot
2. Configured the input ports to route desired joint angles to the joints (e.g., as opposed to torques) so that you can directly specify desired configurations
3. Configured the output ports to output the actual signals in case of any mismatch with the input
4. Wired the input ports to a subsystem where you can manually edit joint angles one by one
