%% Import urdf files into Simulink
% Urdf files downloaded from here:
% https://github.com/dexsuite/dex-urdf/blob/main/robots/hands/shadow_hand
% Urdf files slightly modified to use stl files instead of obj files

smimport("shadow_hand_left.urdf", "ModelName","shadow_hand_left");
smimport("shadow_hand_right.urdf", "ModelName","shadow_hand_right");

%% Get rigidBodyTree objects required for some Simscape Multibody blocks
% shadow_hand_left_rbt = importrobot("shadow_hand_left.slx");
% shadow_hand_right_rbt = importrobot("shadow_hand_right.slx");

% Import directly from urdf, not simulink file, because this method
% auto assigns the bodies their associated names which is convenient for
% selecting certain bodies to serve as end-effectors
shadow_hand_left_rbt = importrobot("shadow_hand_left.urdf");
shadow_hand_right_rbt = importrobot("shadow_hand_right.urdf");

%% Make sure to run this section before simulating
% Meshes folder required for visuals
addpath("meshes");

%% Joint names
% Names of joints corresponding to bounds extracted via constraintJointBounds
% I figured out the matching by comparing the default bounds to those in the urdf file
% This ordering now aligns to the ordering of input/output ports of robot subsystems in Simulink files 
jointNames = {'WRJ2', 'WRJ1', 'FFJ4', 'FFJ3', 'FFJ2', 'FFJ1', 'LFJ5', 'LFJ4', 'LFJ3', 'LFJ2', 'LFJ1', ...
    'MFJ4', 'MFJ3', 'MFJ2', 'MFJ1', 'RFJ4', 'RFJ3', 'RFJ2', 'RFJ1', 'THJ5', 'THJ4', 'THJ3', 'THJ2', 'THJ1'};

nJoints = 24;
