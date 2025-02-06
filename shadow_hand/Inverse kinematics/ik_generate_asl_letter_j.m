
%% Solve inverse kinematics for each finger tip, one by one

% Reminder: se3(trvec(tip_to_world), 'trvec') * se3(rotm(tip_to_world)) == tip_to_world

rbt = shr26df_rbt;
sign_name = 'letter_j';

% Start position is letter i
load('Configs\letter_i.mat');
startValues = jointValues;
q0 = jointValuesToConfigObj(startValues, jointNames);

% End position is letter i, but ARMJ1=180 deg
endValues = startValues;
endValues(ismember(jointNames, 'ARMJ1')) = deg2rad(180);
jointValues = endValues;
save(['Configs', filesep, sign_name, '_stage_2.mat'], "jointValues");


lftip_to_world_q0 = se3(getTransform(rbt,q0,'lftip',"world"));
palm_to_world_q0 = se3(getTransform(rbt,q0,"palm","world"));
trvec_lftip_q0 = trvec(lftip_to_world_q0); 
trvec_palm_q0 = trvec(palm_to_world_q0); 

xprop = 1;
yprop = -0.1;
zprop = 1;

endValues = startValues;
endValues(1) = deg2rad(180);
qf = jointValuesToConfigObj(endValues, jointNames);

afterAdjustments = {};

%% Set up target pose of tip
% lftip_to_world = se3(getTransform(rbt,q0,'lftip',"world"));

distanceConstraint = constraintPositionTarget('lftip');
distanceConstraint.ReferenceBody = 'world';

trvec_target = zeros(1,3);
trvec_target(1) = trvec_lftip_q0(1) + xprop * (trvec_lftip_q0(3) - trvec_palm_q0(3));
trvec_target(2) = trvec_lftip_q0(2) + yprop * (trvec_palm_q0(2) - trvec_lftip_q0(2) + trvec_palm_q0(3) - trvec_lftip_q0(3));
trvec_target(3) = trvec_lftip_q0(3) + zprop * (trvec_palm_q0(3) - trvec_lftip_q0(3));

distanceConstraint.TargetPosition = trvec_target;
distanceConstraint.PositionTolerance = 0;
positionOrPose = 0;


%% Create solver
if positionOrPose==0
    gik = generalizedInverseKinematics('RigidBodyTree', rbt, ...
    'ConstraintInputs', {'position','joint'});
else
    gik = generalizedInverseKinematics('RigidBodyTree', rbt, ...
        'ConstraintInputs', {'pose','joint'});
end

% Solver parameters
% gik.SolverParameters.MaxIterations = 1500;
gik.SolverParameters.MaxTime = 2;


if positionOrPose == 1
    % End effector pose contraints
    tip_pos = constraintPoseTarget(tip_frame, 'ReferenceBody', 'world');
    tip_pos.TargetTransform = tform(targetPose);
    tip_pos.OrientationTolerance = deg2rad(30); % allow more leeway for orientation
    tip_pos.PositionTolerance = 0;
    tip_pos.Weights = [1, 1]; % PositionTolerance and OrientationTolerance
end

% Joint constraints -- only want ARMJ1, WRJ2, and WRJ1 to move
jointLimits = constraintJointBounds(rbt);
oldBounds = jointLimits.Bounds;
upperBounds = oldBounds(:,2);
lowerBounds = oldBounds(:,1);
constrainIdx = ~ismember(jointNames, {'ARMJ1', 'WRJ2', 'WRJ1'});
upperBounds(constrainIdx) = startValues(constrainIdx); 
lowerBounds(constrainIdx) = startValues(constrainIdx); 
jointLimits.Bounds = [lowerBounds, upperBounds];
jointLimits.Weights = 20 * ones(1, nJoints);

%% Run solver
if positionOrPose == 1
    [qSol, solutionInfo] = gik(q0, tip_pos, jointLimits);
else
    [qSol, solutionInfo] = gik(q0, distanceConstraint, jointLimits);
end
solJointValues = vertcat(qSol.JointPosition);
solJointValues(abs(solJointValues) < 1e-3)=0;

jointValues = solJointValues;
jointValuesToInputSignals(solJointValues, jointNames, 0.001, 2, sign_name);
save(['Configs', filesep, sign_name, '_stage_1.mat'], "jointValues");

% valuesPrev = solJointValues;
qCurrent = jointValuesToConfigObj(solJointValues, jointNames); % Initial config for next iteration

%% Create signals to provide to right_test_asl_poses.slx
jointValuesToInputSignals(solJointValues, jointNames, 0.001, 2, ...
     'signals_after_solving_stage_1');

mdl = "User input models/shr26df_user_input.slx";
%mdl = "User input models/shl26df_user_input.slx";

% Show robotic hand
supplyInputToUserInputMdlByMat(mdl, 'Signals/signals_after_solving_stage_1.mat');

%% For debugging
lftip_to_world = se3(getTransform(rbt,qCurrent,'lftip',"world"));
disp('lftip offsets:\n')
disp(trvec(lftip_to_world));
disp(trvec_target);


%% Generate trajectory between signs
signSeq = {'letter_i', 'letter_j_stage_1', 'letter_j_stage_2'};
prevConfig = startValues;
[ds, lastConfig] = genConfigTrajectoryFromInput(signSeq, prevConfig, jointNames);

% Show robotic hand
supplyInputToUserInputMdlByDs(mdl, ds);

qWaypoints = zeros(length(signSeq), nJoints);
for i = 1:length(signSeq)
    load(['Configs', filesep, signSeq{i}, '.mat'], 'jointValues');
    qWaypoints(i, :) = jointValues;
end
jointValues = qWaypoints;
save(['Configs', filesep, sign_name, '.mat'], "jointValues");
%%

