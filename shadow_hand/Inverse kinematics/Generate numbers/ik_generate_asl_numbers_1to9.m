%% Solve inverse kinematics for each finger tip, one by one

% Reminder: se3(trvec(tip_to_world), 'trvec') * se3(rotm(tip_to_world)) == tip_to_world

rbt = shr25df_rbt;

for numberIdx = 1:9
q0 = homeConfiguration(rbt);
valuesPrev = zeros(1,nJoints);

pose_name = number_pose_params(numberIdx).name;
curledInFingers = number_pose_params(numberIdx).curledInFingers;
thumbRestingFinger = number_pose_params(numberIdx).thumbRestingFinger;
thumbTouchesFinger = number_pose_params(numberIdx).thumbTouchesFinger;
afterAdjustments = number_pose_params(numberIdx).afterAdjustments;
disp(['Running inverse kinematics for ', pose_name])

for fingerIdx = 1:5
%% Set parameters
% Params for curledInFingers
% Proportion of y-distance from palm ref frame to lftip ref frame to
% have y value of lftip
yprop = 0.8;

% Proportion of z-distance from palm ref frame to lfknuckle ref frame to
% have z value of lftip
zprop = 0.5;

% Params for thumbTouchesFinger
yoffset_from_palm = [0.01, -0.01, 0.01, 0.04];
zoffset_from_knuckle = [0, 0, 0, 0];
zrot = deg2rad([30, 30, 0, 10]);
yrot = deg2rad([0, 30, 0, 0]);

%% Set up target pose of tip
tip_frame = [lower(fingerNames{fingerIdx}),'tip'];
tip_to_world = se3(getTransform(rbt,q0,tip_frame,"world"));
if fingerIdx == 5 % Thumb
    if ~isempty(thumbRestingFinger) % If thumb rests on a finger
        restingIdx = contains(fingerNames,thumbRestingFinger{1});
        resting_frame = [lower(fingerNames{restingIdx}),'middle']; %thumb rests upon middle link of this finger
        distanceFromRestingFinger = constraintPositionTarget(tip_frame);
        distanceFromRestingFinger.ReferenceBody = resting_frame;
        distanceFromRestingFinger.PositionTolerance = 1e-2;

    elseif ~isempty(thumbTouchesFinger) % If thumb touches a finer
        touching_frame = [lower(thumbTouchesFinger{1}),'tip']; %thumb touches tip of this finger
        distanceFromTouchingFinger = constraintPositionTarget(tip_frame);
        distanceFromTouchingFinger.ReferenceBody = touching_frame;
        distanceFromTouchingFinger.PositionTolerance = 1e-3;
    end
else % Other four fingers
    if any(contains(curledInFingers, fingerNames{fingerIdx}))
        
        % Get transforms of certain frames relative to world in home config
        knuckle_frame = [lower(fingerNames{fingerIdx}),'knuckle'];
        knuckle_to_world = se3(getTransform(rbt,q0,knuckle_frame,"world"));
        palm_to_world = se3(getTransform(rbt,q0,"palm","world"));
        
        % Create target translation
        trvec_palm = trvec(palm_to_world); 
        trvec_tip = trvec(tip_to_world); 
        trvec_knuckle = trvec(knuckle_to_world); 
        trvec_target = trvec_palm;
        trvec_target(2) = trvec_palm(2) + yprop*(trvec_tip(2) - trvec_palm(2)); 
        trvec_target(3) = trvec_palm(3) + zprop*(trvec_knuckle(3) - trvec_palm(3));
        T1 = se3(trvec_target, "trvec");
        
        % Get target rotation
        % Get home config orientation of tip relative to world
        R1 = se3(rotm(tip_to_world)); 
        % By inspection, we see that we need to rotate around by x axis by 270 deg to get z axis of tip pointing in -x direction of world frame
        % Make a little less than 270 so that it's more realistic
        R2 = se3([deg2rad(250), 0, 0],"eul","XYZ"); 
        
        % Create target pose
        targetPose = T1 * R1 * R2;
    elseif any(contains(thumbTouchesFinger, fingerNames{fingerIdx}))
        knuckle_frame = [lower(fingerNames{fingerIdx}),'knuckle'];
        
        % Get transforms of certain frames relative to world in home config
        knuckle_to_world = se3(getTransform(rbt,q0,knuckle_frame,"world"));
        palm_to_world = se3(getTransform(rbt,q0,"palm","world"));
        
        % Create target translation
        trvec_palm = trvec(palm_to_world); 
        trvec_tip = trvec(tip_to_world); 
        trvec_knuckle = trvec(knuckle_to_world); 
        trvec_target = trvec_palm;
        trvec_target(1) = trvec_palm(1) + 0.08; 
        trvec_target(2) = trvec_palm(2)+ yoffset_from_palm(fingerIdx);
        trvec_target(3) = trvec_knuckle(3) + zoffset_from_knuckle(fingerIdx);
        T1 = se3(trvec_target, "trvec");
        
        % Get target rotation
        % Get home config orientation of tip relative to world
        R1 = se3(rotm(tip_to_world)); 
        % By inspection, we see that we need to rotate around by x axis by 90 deg to get z axis of tip pointing in +x direction of world frame
        R2 = se3([deg2rad(90), yrot(fingerIdx), zrot(fingerIdx)],"eul","XYZ"); 
        
        % Create target pose
        targetPose = T1 * R1 * R2;
    else
        targetPose = tip_to_world;
    end

    % End effector pose contraints
    tip_pos = constraintPoseTarget(tip_frame, 'ReferenceBody', 'world');
    tip_pos.TargetTransform = tform(targetPose);
    tip_pos.OrientationTolerance = deg2rad(30); % allow more leeway for orientation
    tip_pos.PositionTolerance = 0;
    tip_pos.Weights = [20, 1]; % PositionTolerance and OrientationTolerance

end

%% Create solver
if fingerIdx == 5
    gik = generalizedInverseKinematics('RigidBodyTree', rbt, ...
    'ConstraintInputs', {'position','joint'});
else
    gik = generalizedInverseKinematics('RigidBodyTree', rbt, ...
        'ConstraintInputs', {'pose','joint'});
end

% Solver parameters
% gik.SolverParameters.MaxIterations = 1500;
gik.SolverParameters.MaxTime = 2;

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
if fingerIdx < 5
    [qSol, solutionInfo] = gik(q0, tip_pos, jointLimits);
else
    if ~isempty(thumbRestingFinger)
        [qSol, solutionInfo] = gik(q0, distanceFromRestingFinger, jointLimits);
    elseif ~isempty(thumbTouchesFinger)
        [qSol, solutionInfo] = gik(q0, distanceFromTouchingFinger, jointLimits);
    end
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
end