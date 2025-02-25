%% Solve inverse kinematics for each finger tip, one by one

% Reminder: se3(trvec(tip_to_world), 'trvec') * se3(rotm(tip_to_world)) == tip_to_world

mdl = "User input models/shr26df_user_input.slx";
%mdl = "User input models/shl26df_user_input.slx";
rbt = shr26df_rbt;

pose_name = 'letter_s';
q0 = homeConfiguration(rbt);
valuesPrev = zeros(1,nJoints);

xoffset_from_palm = zeros(1,4);
yoffset_from_knuckle = [0.012, 0.01 0, -0.01];
zoffset_from_palm = [0.04, 0.04, 0.04, 0.04];
%zoffset_from_palm = [0.04, 0.04, 0.04, 0.04]+0.01; % try this out later

afterAdjustments = {};

for fingerIdx = 1:5
%% Set up target pose of tip
tip_frame = [lower(fingerNames{fingerIdx}),'tip'];
if fingerIdx == 5 % Thumb
    % Get transforms of certain frames relative to world in home config
    mfmiddle_to_world = se3(getTransform(rbt,q0,'mfmiddle',"world"));

    % Create target translation
    trvec_mfmiddle = trvec(mfmiddle_to_world); 
    trvec_target = trvec_mfmiddle;
     
    trvec_target(1) = trvec_target(1) + 0.04; 
    trvec_target(2) = trvec_target(2) - 0.01; 
    trvec_target(3) = trvec_target(3) - 0.02;
    T1 = se3(trvec_target, "trvec");

    % Get target rotation
    R1 = se3([deg2rad(90), 0, 0],"eul","XYZ"); 

    % Create target pose
    targetPose = T1 * R1;
    
    % Create pose constraint
    poseConstraint = constraintPoseTarget(tip_frame, 'ReferenceBody', 'world');
    poseConstraint.TargetTransform = tform(targetPose);
    poseConstraint.OrientationTolerance = deg2rad(60); % allow more leeway for orientation
    poseConstraint.PositionTolerance = 0;
    poseConstraint.Weights = [1, 1]; % PositionTolerance and OrientationTolerance

    % Run solver
    jointValues = runGikSolver(rbt, fingerIdx, ...
            valuesPrev, [], poseConstraint);

else % Other four fingers
    knuckle_frame = [lower(fingerNames{fingerIdx}),'knuckle'];
    
    % Get transforms of certain frames relative to world in home config
    knuckle_to_world = se3(getTransform(rbt,q0,knuckle_frame,"world"));
    palm_to_world = se3(getTransform(rbt,q0,"palm","world"));
    
    % Create target translation
    trvec_palm = trvec(palm_to_world); 
    trvec_knuckle = trvec(knuckle_to_world); 
    
    trvec_target = trvec_palm;
    trvec_target(1) = trvec_palm(1) + xoffset_from_palm(fingerIdx);
    trvec_target(2) = trvec_knuckle(2)+ yoffset_from_knuckle(fingerIdx);
    trvec_target(3) = trvec_palm(3) + zoffset_from_palm(fingerIdx);

    % Create distance constraint
    distanceConstraint = constraintPositionTarget(tip_frame);
    distanceConstraint.ReferenceBody = 'world';
    distanceConstraint.TargetPosition = trvec_target;
    distanceConstraint.PositionTolerance = 0;%1e-3;
    
    % Run solver
    jointValues = runGikSolver(rbt, fingerIdx, ...
            valuesPrev, distanceConstraint, []);
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
disp(trvec(thtip_to_world) - trvec_mfmiddle)
%disp(rad2deg(rotm2eul(rotm(thtip_to_world), 'XYZ')))