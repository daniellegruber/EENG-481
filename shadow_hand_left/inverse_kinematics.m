%% Create target pose of little finger tip (lftip)
% This is just for testing. Obviously for the real thing we wouldn't get
% the target pose from a specified configuration -- that defeats the whole
% purpose. This is to see whether the configuration achieved by the solver
% matches the configuration (targetConfig) from which targetPose is derived.

load("fist.mat","jointValues");

targetConfig = repmat(struct('JointName','', 'JointPosition', 0), 1, nJoints);
for i = 1:24
    targetConfig(i) = struct('JointName', jointNames{i}, 'JointPosition', jointValues(i));
end

targetPose = se3(getTransform(shadow_hand_right_rbt,targetConfig,"lftip","world"));

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
upperBounds([1:6, 12:24]) = 1e-6; 
lowerBounds([1:6, 12:24]) = -1 * 1e-6;
jointLimits.Bounds = [lowerBounds, upperBounds];
jointLimits.Weights = 10 * ones(1, 24);

% End effector pose contraints
%lftip_pos = constraintCartesianBounds('lftip', 'ReferenceBody', 'world');
lftip_pos = constraintPoseTarget('lftip', 'ReferenceBody', 'world');
lftip_pos.TargetTransform = tform(targetPose);
lftip_pos.OrientationTolerance = deg2rad(0);
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
