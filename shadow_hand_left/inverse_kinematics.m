%% Create target pose of little finger tip (lftip)
% Get a "curled in" little finger by trying to get the little finger tip
% (lftip) as close to palm as possible, and orienting it so that its z-axis
% points into the palm
%% Set parameters
% Proportion of y-distance from palm ref frame to lftip ref frame to
% have y value of lftip
yprop = 0.8;

% Proportion of z-distance from palm ref frame to lfknuckle ref frame to
% have z value of lftip
zprop = 0.3;

%% Set up target pose of lftip

q0 = homeConfiguration(shadow_hand_right_rbt);

% End effector pose contraints
lftip_to_world = se3(getTransform(shadow_hand_left_rbt,homeConfiguration(shadow_hand_left_rbt),"lftip","world"));
lfknuckle_to_world = se3(getTransform(shadow_hand_left_rbt,homeConfiguration(shadow_hand_left_rbt),"lfknuckle","world"));
palm_to_world = se3(getTransform(shadow_hand_left_rbt,homeConfiguration(shadow_hand_left_rbt),"palm","world"));

%T1 = se3(tform(palm_to_world)); % get translation vector for palm relative to world
trvec_palm = trvec(palm_to_world); 
trvec_lftip = trvec(lftip_to_world); 
trvec_lfknuckle = trvec(lftip_to_world); 
trvec_new = trvec_palm;
trvec_new(2) = trvec_palm(2) + yprop*(trvec_lftip(2) - trvec_palm(2)); 
trvec_new(3) = trvec_palm(3) + zprop*(trvec_lfknuckle(3) - trvec_palm(3));
T1 = se3(trvec_new, "trvec");
R1 = se3(rotm(lftip_to_world)); % get home config orientation of lftip relative to world
% by inspection, we see that we need to rotate around by x axis by 270 deg to get z axis of lftip pointing in -x direction of world frame
% make a little less than 270 so that it's more realistic
R2 = se3([deg2rad(250), 0, 0],"eul","XYZ"); 
targetPose = T1 * R1 * R2;

%% Create solver
gik = generalizedInverseKinematics('RigidBodyTree', shadow_hand_right_rbt, ...
    'ConstraintInputs', {'pose','joint'});

% gik = generalizedInverseKinematics('RigidBodyTree', shadow_hand_right_rbt, ...
%     'ConstraintInputs', {'pose','joint'}, 'SolverAlgorithm','LevenbergMarquardt');

% gik = generalizedInverseKinematics('RigidBodyTree', shadow_hand_left_rbt, ...
%     'ConstraintInputs', {'cartesian','position','aiming','orientation','joint'});

% Solver parameters
% gik.SolverParameters.MaxIterations = 1500;
gik.SolverParameters.MaxTime = 2;

% Joint constraints -- only want little finger lf to move
jointLimits = constraintJointBounds(shadow_hand_right_rbt);
oldBounds = jointLimits.Bounds;
upperBounds = oldBounds(:,2);
lowerBounds = oldBounds(:,1);
% Fix non-lf joints
upperBounds([1:6, 12:24]) = 0; 
lowerBounds([1:6, 12:24]) = 0;
% limit LFJ5 and LFJ4 bounds so that lftip doesn't go too far into middle of palm
% upperBounds(7) = deg2rad(10); 
% lowerBounds(8) = deg2rad(-10); 
jointLimits.Bounds = [lowerBounds, upperBounds];
jointLimits.Weights = 10 * ones(1, 24);

% End effector pose contraints
%lftip_pos = constraintCartesianBounds('lftip', 'ReferenceBody', 'world');
lftip_pos = constraintPoseTarget('lftip', 'ReferenceBody', 'world');
lftip_pos.TargetTransform = tform(targetPose);
lftip_pos.OrientationTolerance = deg2rad(10);
lftip_pos.PositionTolerance = 0;
lftip_pos.Weights = [1, 1]; % PositionTolerance and OrientationTolerance
%lftip_pos.Bounds = [0.01, 0.03; 0.03, 0.034; 0.25, 0.35];

% Initial config
q0=homeConfiguration(shadow_hand_right_rbt);

%% Run solver
[qSol, solutionInfo] = gik(q0, lftip_pos, jointLimits);
% qSol = gik(q0, lftip_pos, jointLimits);
% figure;
% show(shadow_hand_left_rbt,qSol) % for some reason this doesn't show the right joint values

%% Create signals to provide to right_test_asl_poses.slx
solJointValues = vertcat(qSol.JointPosition);
solJointValues(abs(solJointValues) < 1e-3)=0;
jointValuesToInputSignals(solJointValues, jointNames, 0.001, 2, ...
    ['signals ', char(datetime('now', 'Format', 'd-MMM-y HH-mm-ss'))]);
