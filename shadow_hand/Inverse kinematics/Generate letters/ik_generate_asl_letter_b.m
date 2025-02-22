%% Solve inverse kinematics for each finger tip, one by one

% Reminder: se3(trvec(tip_to_world), 'trvec') * se3(rotm(tip_to_world)) == tip_to_world

rbt = shr25df_rbt;

pose_name = 'letter_b';
q0 = homeConfiguration(rbt);
valuesPrev = zeros(1,nJoints);
afterAdjustments = {};

for fingerIdx = 1:5
%% Set up target pose of tip
tip_frame = [lower(fingerNames{fingerIdx}),'tip'];
tip_to_world = se3(getTransform(rbt,q0,tip_frame,"world"));
if fingerIdx == 5 % Thumb
    % % Get transforms of certain frames relative to world in home config
    % lfknuckle_to_world = se3(getTransform(rbt,q0,'lfknuckle',"world"));
    % 
    % % Create target translation
    % trvec_lfknuckle = trvec(lfknuckle_to_world); 
    % trvec_target = trvec_lfknuckle;
    % trvec_target(1) = trvec_target(1) + 0; 
    % trvec_target(2) = trvec_target(2) + 0.03; 
    % trvec_target(3) = trvec_target(3) - 0.2;
    % T1 = se3(trvec_target, "trvec");
    % 
    % % Get target rotation
    % 
    % % R1 = se3(rotm(tip_to_world)); 
    % % Now define angles relative to world frame
    % %R2 = se3([0, 0, 0],"eul","XYZ"); 
    % 
    % R1 = se3([deg2rad(90), 0, 0],"eul","XYZ"); 
    % 
    % % Create target pose
    % targetPose = T1 * R1; %* R1 * R2;
    % positionOrPose = 1;

    distanceConstraint = constraintPositionTarget(tip_frame);
    distanceConstraint.ReferenceBody = 'world';

    % Get transforms of certain frames relative to world in home config
    lfknuckle_to_world = se3(getTransform(rbt,q0,'lfknuckle',"world"));

    % Create target translation
    trvec_lfknuckle = trvec(lfknuckle_to_world); 
    trvec_target = trvec_lfknuckle;
    trvec_target(1) = trvec_target(1) + 0.02; 
    trvec_target(2) = trvec_target(2) + 0.04; 
    trvec_target(3) = trvec_target(3) - 0.02;

    distanceConstraint.TargetPosition = trvec_target;
    distanceConstraint.PositionTolerance = 0;%1e-3;
    positionOrPose = 0;

else % Other four fingers
    targetPose = tip_to_world;
    positionOrPose = 1;
end

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
    tip_pos.PositionTolerance = 1e-2;
    tip_pos.Weights = [1, 1]; % PositionTolerance and OrientationTolerance
end

% Joint constraints -- only want little finger lf to move
jointLimits = constraintJointBounds(rbt);
oldBounds = jointLimits.Bounds;
upperBounds = oldBounds(:,2);
lowerBounds = oldBounds(:,1);
% Fix non-finger joints to values obtained from previous iteration
nonFingerIdx = ~startsWith(jointNames,fingerNames{fingerIdx});
upperBounds(nonFingerIdx) = valuesPrev(nonFingerIdx); 
lowerBounds(nonFingerIdx) = valuesPrev(nonFingerIdx); 
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

if fingerIdx == 5
    if ~isempty(afterAdjustments)
        for i = 1:2:length(afterAdjustments)
            jointIdx = contains(jointNames, afterAdjustments{i});
            jointValue = afterAdjustments{i+1};
            solJointValues(jointIdx) = jointValue;
        end
    end
    jointValues = solJointValues;
    jointValuesToInputSignals(solJointValues, jointNames, 0.001, 2, pose_name);
    save(['Configs', filesep, pose_name, '.mat'], "jointValues");
end
valuesPrev = solJointValues;
q0 = jointValuesToConfigObj(solJointValues, jointNames); % Initial config for next iteration

% qSol = gik(q0, lftip_pos, jointLimits);
% figure;
% show(shadow_hand_left_rbt,qSol) % for some reason this doesn't show the right joint values

%% Create signals to provide to right_test_asl_poses.slx
jointValuesToInputSignals(solJointValues, jointNames, 0.001, 2, ...
     ['signals_after_solving_', fingerNames{fingerIdx}]);

% jointValuesToInputSignals(solJointValues, jointNames, 0.001, 2, ...
%     ['signals ', char(datetime('now', 'Format', 'd-MMM-y HH-mm-ss'))]);
end

mdl = "User input models/shr25df_user_input.slx";
%mdl = "User input models/shl25df_user_input.slx";

% Show robotic hand
supplyInputToUserInputMdlByMat(mdl, 'Signals/signals_after_solving_TH.mat');

%% For debugging
thtip_to_world = se3(getTransform(rbt,q0,'thtip',"world"));
lfknuckle_to_world = se3(getTransform(rbt,q0,'lfknuckle',"world"));
disp(trvec(thtip_to_world) - trvec(lfknuckle_to_world))
disp(rad2deg(rotm2eul(rotm(thtip_to_world), 'XYZ')))