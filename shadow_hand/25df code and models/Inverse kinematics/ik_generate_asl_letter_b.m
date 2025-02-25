%% Solve inverse kinematics for each finger tip, one by one

% Reminder: se3(trvec(tip_to_world), 'trvec') * se3(rotm(tip_to_world)) == tip_to_world

mdl = "User input models/shr26df_user_input.slx";
%mdl = "User input models/shl26df_user_input.slx";
rbt = shr26df_rbt;

pose_name = 'letter_b';
q0 = homeConfiguration(rbt);
valuesPrev = zeros(1,nJoints);
afterAdjustments = {};

for fingerIdx = 1:5
%% Set up target pose of tip
tip_frame = [lower(fingerNames{fingerIdx}),'tip'];
tip_to_world = se3(getTransform(rbt,q0,tip_frame,"world"));
if fingerIdx == 5 % Thumb
    % Get transforms of certain frames relative to world in home config
    lfknuckle_to_world = se3(getTransform(rbt,q0,'lfknuckle',"world"));

    % Create target translation
    trvec_lfknuckle = trvec(lfknuckle_to_world); 
    trvec_target = trvec_lfknuckle;
    trvec_target(1) = trvec_target(1) + 0.02; 
    trvec_target(2) = trvec_target(2) + 0.04; 
    trvec_target(3) = trvec_target(3) - 0.02;

    % Create distance constraint
    distanceConstraint = constraintPositionTarget(tip_frame);
    distanceConstraint.ReferenceBody = 'world';
    distanceConstraint.TargetPosition = trvec_target;
    distanceConstraint.PositionTolerance = 0;%1e-3;
    
    % Run solver
    jointValues = runGikSolver(rbt, fingerIdx, ...
            valuesPrev, distanceConstraint, []);

else % Other four fingers
    targetPose = tip_to_world;
    
    % Create pose constraint
    poseConstraint = constraintPoseTarget(tip_frame, 'ReferenceBody', 'world');
    poseConstraint.TargetTransform = tform(targetPose);
    poseConstraint.OrientationTolerance = deg2rad(30); % allow more leeway for orientation
    poseConstraint.PositionTolerance = 0;
    poseConstraint.Weights = [1, 1]; % PositionTolerance and OrientationTolerance

    % Run solver
    jointValues = runGikSolver(rbt, fingerIdx, ...
            valuesPrev, [], poseConstraint);
end

valuesPrev = jointValues;

if fingerIdx == 5
    jointValues = applyAfterAdjustments(jointValues, jointNames, afterAdjustments);
    jointValuesToInputSignals(jointValues, jointNames, 0.001, 2, pose_name);
    save(['Configs', filesep, pose_name, '.mat'], "jointValues");
end

q0 = jointValuesToConfigObj(jointValues, jointNames); % Initial config for next iteration

%% Create signals to provide to right_test_asl_poses.slx
jointValuesToInputSignals(jointValues, jointNames, 0.001, 2, ...
     ['signals_after_solving_', fingerNames{fingerIdx}]);

end

% Show robotic hand
supplyInputToUserInputMdlByMat(mdl, 'Signals/signals_after_solving_TH.mat');

%% For debugging
thtip_to_world = se3(getTransform(rbt,q0,'thtip',"world"));
lfknuckle_to_world = se3(getTransform(rbt,q0,'lfknuckle',"world"));
disp(trvec(thtip_to_world) - trvec(lfknuckle_to_world))
disp(rad2deg(rotm2eul(rotm(thtip_to_world), 'XYZ')))