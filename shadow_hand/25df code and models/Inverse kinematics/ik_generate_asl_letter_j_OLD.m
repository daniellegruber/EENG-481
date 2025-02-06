
% mdl = "User input models/shr25df_user_input.slx";
% %mdl = "User input models/shl25df_user_input.slx";
% 
% % Show robotic hand
% supplyInputToUserInputMdlByMat(mdl, 'Signals/letter_i.mat');
%% Solve inverse kinematics for each finger tip, one by one

% Reminder: se3(trvec(tip_to_world), 'trvec') * se3(rotm(tip_to_world)) == tip_to_world

rbt = shr25df_rbt;

pose_name = 'letter_j';
load('Configs\letter_i.mat');
startValues = jointValues;
q0 = jointValuesToConfigObj(startValues, jointNames);

% Desired offsets of lftip from original i pose to stages in the j
% trajectory
% xoffset = [0.8, 0.8];
% yoffset = [0.05, 0.1];
% zoffset = [-0.15, -0.1];

lftip_to_world_q0 = se3(getTransform(rbt,q0,'lftip',"world"));
palm_to_world_q0 = se3(getTransform(rbt,q0,"palm","world"));
trvec_lftip_q0 = trvec(lftip_to_world_q0); 
trvec_palm_q0 = trvec(palm_to_world_q0); 

% xoffset = [-0.02, 0, 0];
% yoffset = [-0.035, -0.03, -0.01];
% zoffset = [0.08, 0.02, 0.01];

% xprop = [0, 0.02, 0.1];
% yprop = [-0.2, -1.5, -2.5];
% zprop = [0, 0.08, 0.2];

xprop = [0, 0.08, 0.2, 0.6, 0.9, 1, 0.6, 0];
yprop = [0.1, 0.3, 0.5, 0.9, 0.3, -0.1, -0.9, -1.3];
zprop = [0, 0.08, 0.2, 0.7, 0.9, 1, 0.6, 0];

endValues = startValues;
endValues(1) = deg2rad(180);
qf = jointValuesToConfigObj(endValues, jointNames);

afterAdjustments = {};
for stage = 7:8
%% Set up target pose of tip
% lftip_to_world = se3(getTransform(rbt,q0,'lftip',"world"));

distanceConstraint = constraintPositionTarget('lftip');
distanceConstraint.ReferenceBody = 'world';

% % Get transforms of certain frames relative to world in home config
% palm_to_world = se3(getTransform(rbt,q0,"palm","world"));
% 
% % Create target translation
% % trvec_tip_q0 = trvec(tip_to_world_q0); 
% trvec_tip = trvec(lftip_to_world); 
% trvec_palm = trvec(palm_to_world); 

% trvec_target = trvec_tip;
% trvec_target(1) = trvec_target(1) + xoffset(stage);
% trvec_target(2) = trvec_target(2) + yoffset(stage);
% trvec_target(3) = trvec_target(3) + zoffset(stage);


% trvec_target = trvec_palm;
% trvec_target(1) = trvec_target(1) + (trvec_tip(3) - trvec_palm(3)) + xoffset(stage);
% trvec_target(2) = trvec_target(2) + yoffset(stage);
% trvec_target(3) = trvec_target(3) + zoffset(stage);

% trvec_target = trvec_palm_q0;
% trvec_target(1) = trvec_target(1) + (trvec_tip(3) - trvec_palm(3)) + xoffset(stage);
% trvec_target(2) = trvec_target(2) + yoffset(stage);
% trvec_target(3) = trvec_target(3) + zoffset(stage);


% trvec_target = zeros(1,3);
% trvec_target(1) = trvec_lftip_q0(1) + xprop(stage) * (trvec_lftip_q0(3) - trvec_palm_q0(3));
% trvec_target(2) = trvec_lftip_q0(2) + yprop(stage) * (trvec_palm_q0(2) - trvec_lftip_q0(2));
% trvec_target(3) = trvec_lftip_q0(3) + zprop(stage) * (trvec_palm_q0(3) - trvec_lftip_q0(3));


trvec_target = zeros(1,3);
trvec_target(1) = trvec_lftip_q0(1) + xprop(stage) * (trvec_lftip_q0(3) - trvec_palm_q0(3));
trvec_target(2) = trvec_lftip_q0(2) + yprop(stage) * (trvec_palm_q0(2) - trvec_lftip_q0(2) + trvec_palm_q0(3) - trvec_lftip_q0(3));
trvec_target(3) = trvec_lftip_q0(3) + zprop(stage) * (trvec_palm_q0(3) - trvec_lftip_q0(3));

distanceConstraint.TargetPosition = trvec_target;
distanceConstraint.PositionTolerance = 0;%1e-3;
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
upperBounds(4:end) = startValues(4:end); 
lowerBounds(4:end) = startValues(4:end); 
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
jointValuesToInputSignals(solJointValues, jointNames, 0.001, 2, pose_name);
save(['Configs', filesep, pose_name, '_stage_', num2str(stage), '.mat'], "jointValues");

% valuesPrev = solJointValues;
qCurrent = jointValuesToConfigObj(solJointValues, jointNames); % Initial config for next iteration

% qSol = gik(q0, lftip_pos, jointLimits);
% figure;
% show(shadow_hand_left_rbt,qSol) % for some reason this doesn't show the right joint values

%% Create signals to provide to right_test_asl_poses.slx
jointValuesToInputSignals(solJointValues, jointNames, 0.001, 2, ...
     ['signals_after_solving_', fingerNames{fingerIdx}]);

% jointValuesToInputSignals(solJointValues, jointNames, 0.001, 2, ...
%     ['signals ', char(datetime('now', 'Format', 'd-MMM-y HH-mm-ss'))]);


mdl = "User input models/shr25df_user_input.slx";
%mdl = "User input models/shl25df_user_input.slx";

% Show robotic hand
supplyInputToUserInputMdlByMat(mdl, 'Signals/signals_after_solving_TH.mat');

%% For debugging
lftip_to_world_q0 = se3(getTransform(rbt,q0,'lftip',"world"));
lftip_to_world = se3(getTransform(rbt,qCurrent,'lftip',"world"));
palm_to_world_q0 = se3(getTransform(rbt,q0,'palm',"world"));
trvec_lftip = trvec(lftip_to_world);
trvec_lftip_q0 = trvec(lftip_to_world_q0);
trvec_palm_q0 = trvec(palm_to_world_q0);


disp('lftip offsets:\n')
% lftip_offset = trvec(lftip_to_world) - trvec(lftip_to_world_q0);
% lftip_offset = trvec(lftip_to_world) - trvec(palm_to_world_q0);
% lftip_offset(1) = lftip_offset(1) - (trvec_lftip_q0(3) - trvec_palm_q0(3));
%disp(lftip_offset)
disp(trvec(lftip_to_world));
disp(trvec_target);
end

%% Generate trajectory between signs
% signSeq = {'letter_j_stage_1', 'letter_j_stage_2', 'letter_j_stage_3', 'letter_j_stage_4', 'letter_j_stage_5'};
% signSeq = {'letter_i', 'letter_j_stage_3', 'letter_j_stage_6', 'letter_j_stage_7', 'letter_j_stage_9'};
% signSeq = {'letter_i', 'letter_j_stage_3', 'letter_j_stage_6', 'letter_j_stage_9'};
signSeq = {'letter_i', 'letter_j_stage_6', 'letter_j_stage_9'};
prevConfig = startValues;
[ds, lastConfig] = genConfigTrajectoryFromInput(signSeq, prevConfig, jointNames);

% Show robotic hand
supplyInputToUserInputMdlByDs(mdl, ds);

%%
endValues = startValues;
endValues(1) = deg2rad(180);
jointValues = endValues;
save(['Configs', filesep, pose_name, '_stage_9.mat'], "jointValues");
