%% Create dataset for Signal Editor block containing desired joint trajectories

% Get joint names (names of input ports to Robot block)
load_system('left_test_asl_poses');
h=getSimulinkBlockHandle('left_test_asl_poses/Robot');
handles = find_system(h, 'LookUnderMasks', 'on', 'FollowLinks', 'on', 'SearchDepth', 1, 'BlockType', 'Inport');
portInfo = [get_param(handles, 'Name'), get_param(handles, 'Port')];
load("number_1.mat", "jointValues");

% Create dataset and save
Ts = 0.001;
T = 2;
time_samples = 0:Ts:T;
ds = Simulink.SimulationData.Dataset;
for i=1:24
    %trajectory = timeseries(zeros(1, length(0:Ts:T)), 0:Ts:T);
    trajectory = timeseries(jointValues(i) * ones(1, length(time_samples)), time_samples);
    ds = addElement(ds, trajectory, portInfo{i,1});
end
save('signals.mat', 'ds');

%%
trvec(se3(getTransform(shadow_hand_left_rbt,homeConfiguration(shadow_hand_left_rbt),"wrist","palm")))
trvec(se3(shadow_hand_left_rbt.Bodies{3}.Joint.JointToParentTransform))
%%
box_dims = [
    0.062 0.007 0.090;
    0.036 0.017 0.090;
    0.026 0.017 0.010;
    0.026 0.014 0.018;
    0.021 0.027 0.016;
    0.022 0.005 0.032;
    0.018 0.024 0.004;
    0.020 0.024 0.032
];
palm_to_box_trvecs = [
    -0.011 0.0085 0.042;
    0.002 -0.0035 0.042;
    -0.029 -0.0035 0.082;
    -0.0265 -0.001 0.070;
    -0.0315 -0.0085 0.001;
    -0.0125 -0.015 0.009;
    -0.011 0 0.089;
    0.03 0 0.009
];
palm_to_box_rotvecs = [
    0 0 0;
    0 0 0;
    0 0 0;
    0.2 0 -0.25;
    0 0 0;
    0 0 0.48;
    0 0 0;
    0 0 0;
];
figure;
show(shadow_hand_left_rbt,Visuals="on",Collisions="off");
for box_idx=1:8
    box_dim = [0.062 0.007 0.090];
    world_to_palm = se3(getTransform(shadow_hand_left_rbt,homeConfiguration(shadow_hand_left_rbt),"palm","world"));
    palm_to_box = se3(eul2rotm(palm_to_box_rotvecs(box_idx, :),"XYZ"), palm_to_box_trvecs(box_idx,:));
    box_pose = world_to_palm * palm_to_box;
    box = collisionBox(box_dim(1),box_dim(2),box_dim(3), Pose=box_pose);
    hold on;
    [~,patchObj] = show(box);
    patchObj.FaceColor = [0 1 1];
    patchObj.EdgeColor = 'none';
end
%%
figure;
show(shadow_hand_left_rbt,Visuals="on",Collisions="off");
%lftip_trvec = trvec(se3(getTransform(shadow_hand_left_rbt,homeConfiguration(shadow_hand_left_rbt),"world","lftip")));
lftip_trvec = trvec(se3(getTransform(shadow_hand_left_rbt,homeConfiguration(shadow_hand_left_rbt),"lftip","world")));
point_xyz = [0.02 lftip_trvec(2) 0.3];
point_pose = se3(eye(3), point_xyz);
%point_pose = se3(getTransform(shadow_hand_left_rbt,homeConfiguration(shadow_hand_left_rbt),"lftip","world"));
box = collisionBox(0.01,0.01,0.01,Pose=point_pose);
hold on;
[~,patchObj] = show(box);
patchObj.FaceColor = [0 1 1];
patchObj.EdgeColor = 'none';

%%
figure;
show(shadow_hand_left_rbt,Visuals="on",Collisions="off");
%lftip_trvec = trvec(se3(getTransform(shadow_hand_left_rbt,homeConfiguration(shadow_hand_left_rbt),"world","lftip")));
lftip_trvec = trvec(se3(getTransform(shadow_hand_left_rbt,homeConfiguration(shadow_hand_left_rbt),"lftip","world")));
point_xyz = [0.02 0.033 0.3];
%point_pose = se3(eye(3), point_xyz);
point_pose = se3(getTransform(shadow_hand_left_rbt,homeConfiguration(shadow_hand_left_rbt),"lftip","world"));
box = collisionBox(0.03,0.01,0.01,Pose=point_pose);
hold on;
[~,patchObj] = show(box);
patchObj.FaceColor = [0 1 1];
patchObj.EdgeColor = 'none';

%%
% Create solver
gik = generalizedInverseKinematics('RigidBodyTree', shadow_hand_left_rbt, ...
    'ConstraintInputs', {'pose','joint'});
% gik = generalizedInverseKinematics('RigidBodyTree', shadow_hand_left_rbt, ...
%     'ConstraintInputs', {'cartesian','position','aiming','orientation','joint'});

% Joint constraints -- only want little finger lf to move
jointLimits = constraintJointBounds(shadow_hand_left_rbt);
% oldBounds = jointLimits.Bounds;
% upperBounds = oldBounds(:,2);
% lowerBounds = oldBounds(:,1);
% upperBounds(1:2) = 0;
% lowerBounds(1:2) = 0;
% jointLimits.Bounds = [upperBounds, lowerBounds];


% oldBounds = jointLimits.Bounds;
% upperBounds = oldBounds(:,2);
% lowerBounds = oldBounds(:,1);
% upperBounds([1:6, 12:24]) = 0;
% lowerBounds([1:6, 12:24]) = 0;
% jointLimits.Bounds = [upperBounds, lowerBounds];

% End effector pose contraints
lftip_to_world = se3(getTransform(shadow_hand_left_rbt,homeConfiguration(shadow_hand_left_rbt),"lftip","world"));
R1 = se3(rotm(lftip_to_world));
R2 = se3([0, deg2rad(270), 0],"eul","XYZ");
T1 = se3([0.02 0.033 0.3], "trvec");
%target_pose = T1 * R1 * R2;
target_pose = T1 * R1;
%target_pose = lftip_to_world;

%lftip_pos = constraintCartesianBounds('lftip', 'ReferenceBody', 'world');
lftip_pos = constraintPoseTarget('lftip', 'ReferenceBody', 'world');
%target_pose = se3(eul2rotm([0, deg2rad(270), 0],"XYZ"), [0.02 0.033 0.3]);
lftip_pos.TargetTransform = tform(target_pose);
lftip_pos.OrientationTolerance = deg2rad(60);
lftip_pos.PositionTolerance = 0;
lftip_pos.Weights = [1, 1];
%lftip_pos.Bounds = [0.01, 0.03; 0.03, 0.034; 0.25, 0.35];

% Initial config
q0=homeConfiguration(shadow_hand_left_rbt);

%[qWaypoints(2,:),solutionInfo] = gik(q0, lftip_pos, jointLimits);
% [qWaypoints,solutionInfo] = gik(q0, lftip_pos, jointLimits);

qConst = gik(q0, lftip_pos, jointLimits);
figure;
show(shadow_hand_left_rbt,qConst)

%%
rbt = importrobot('shadow_hand_left_only_move_lf.urdf');
jointLimits = constraintJointBounds(rbt);
find(jointLimits.Bounds(:,1) == -1)