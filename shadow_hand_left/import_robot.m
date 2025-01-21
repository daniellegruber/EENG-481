% Downloaded the following folders
% https://github.com/dexsuite/dex-urdf/blob/main/robots/hands/shadow_hand

% using https://download-directory.github.io/

% Converted obj files in meshes folder -> stl files using https://www.makexyz.com/convert/obj-to-stl
%[shadow_hand_left, shadow_hand_left_info] = importrobot("shadow_hand_left.urdf");
smimport("shadow_hand_left.urdf", "ModelName","shadow_hand_left");
smimport("shadow_hand_right.urdf", "ModelName","shadow_hand_right");
%%
[shadow_hand_left_rbt, shadow_hand_left_info] = importrobot("shadow_hand_left.slx");
[shadow_hand_right_rbt, shadow_hand_right_info] = importrobot("shadow_hand_right.slx");
%%
addpath("meshes");

%% Create dataset for Signal Editor block containing desired joint trajectories

% Get joint names (names of input ports to Robot block)
h=getSimulinkBlockHandle('test_asl_poses/Robot');
handles = find_system(h, 'LookUnderMasks', 'on', 'FollowLinks', 'on', 'SearchDepth', 1, 'BlockType', 'Inport');
portInfo = [get_param(handles, 'Name'), get_param(handles, 'Port')];

% Create dataset and save
Ts = 0.001;
T = 2;
ds = Simulink.SimulationData.Dataset;
for i=1:24
    trajectory = timeseries(zeros(1, length(0:Ts:T)), 0:Ts:T);
    ds = addElement(ds, trajectory, portInfo{i,1});
end
save('signals.mat', 'ds');

%%
% WRJ2 and WRJ1: determine "uprightness" of hand, should probably be set to 0 most
% of time
% THJ5: joint at "base" of thumb, determines thumb orientation