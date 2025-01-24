%% Create target pose of each finger tip, one by one
% For each of the four non-thumb fingers, get a "curled in" finger by 
% trying to get the finger tip (lftip, rftip, mftip, fftip) as close to the
% palm as possible, and orienting it so that its z-axis roughly points into 
% the palm

% Reminder: se3(trvec(tip_to_world), 'trvec') * se3(rotm(tip_to_world)) == tip_to_world

fingerNames = {'LF', 'RF', 'MF', 'FF', 'TH'};
rbt = shadow_hand_right_rbt;
q0 = homeConfiguration(rbt);
valuesPrev = zeros(1,24);

curledInFingers = {'LF', 'RF', 'MF'};
thumbRestingFinger = {'LF'}; % which finger thumb rests upon, e.g., MF in number 1 and RF in number 2
thumbTouchesFinger = {}; % which finger thumb touches, e.g., RF in number 7

for fingerIdx = 1:5
%% Set parameters
% Proportion of y-distance from palm ref frame to lftip ref frame to
% have y value of lftip
yprop = 0.8;

% Proportion of z-distance from palm ref frame to lfknuckle ref frame to
% have z value of lftip
zprop = 0.5;

%% Set up target pose of tip
tip_frame = [lower(fingerNames{fingerIdx}),'tip'];
tip_to_world = se3(getTransform(rbt,q0,tip_frame,"world"));
if fingerIdx == 5 % Thumb
    if ~isempty(thumbRestingFinger)
        restingIdx = contains(fingerNames,thumbRestingFinger{1});
        resting_frame = [lower(fingerNames{restingIdx}),'middle']; %thumb rests upon middle link of this finger
        %T1 = se3(getTransform(rbt,q0,resting_frame,"world"));
        
        % trvec_target = trvec(se3(getTransform(rbt,q0,resting_frame,"world")));
        % trvec_target(1) = trvec_target(1) + 0.1;

        trvec_target = trvec(se3(getTransform(rbt,q0,"lftip","world")));
        rvec_target(1) = trvec_target(1) + 0.3;
        
        
        % trvec_target(2) = trvec_target(2) - 0.1;
        % trvec_target = trvec(se3(getTransform(rbt,q0,"rfmiddle","world")));
        % trvec_target(1) = trvec_target(1) + 0.05;
        T1 = se3(trvec_target, 'trvec');

        % Get target rotation
        % Get home config orientation of tip relative to world
        R1 = se3(rotm(tip_to_world)); 
        % By inspection, we see that we need to rotate around by x axis by -135 deg (225 deg) to get z axis of thtip pointing in -y direction of world frame
        % Make a little less than 225 so that it's more realistic
        R2 = se3([deg2rad(180), 0, 0],"eul","XYZ");
        % Then rotate around y-axis by a few degrees so z axis is point a
        % bit out of the page
        R3 = se3([0, deg2rad(5), 0],"eul","XYZ");

        targetPose = T1 * R1 * R2 * R3;
    elseif ~isempty(thumbTouchesFinger)
    end
else % Other four fingers
    if any(contains(curledInFingers, fingerNames{fingerIdx}))
        knuckle_frame = [lower(fingerNames{fingerIdx}),'knuckle'];
        
        % Get transforms of certain frames relative to world in home config
        knuckle_to_world = se3(getTransform(rbt,q0,knuckle_frame,"world"));
        palm_to_world = se3(getTransform(rbt,q0,"palm","world"));
        
        % Create target translation
        trvec_palm = trvec(palm_to_world); 
        trvec_tip = trvec(tip_to_world); 
        trvec_knuckle = trvec(knuckle_to_world); 
        trvec_new = trvec_palm;
        trvec_new(2) = trvec_palm(2) + yprop*(trvec_tip(2) - trvec_palm(2)); 
        trvec_new(3) = trvec_palm(3) + zprop*(trvec_knuckle(3) - trvec_palm(3));
        T1 = se3(trvec_new, "trvec");
        
        % Get target rotation
        % Get home config orientation of tip relative to world
        R1 = se3(rotm(tip_to_world)); 
        % By inspection, we see that we need to rotate around by x axis by 270 deg to get z axis of tip pointing in -x direction of world frame
        % Make a little less than 270 so that it's more realistic
        R2 = se3([deg2rad(250), 0, 0],"eul","XYZ"); 
        
        % Create target pose
        targetPose = T1 * R1 * R2;
    else
        targetPose = tip_to_world;
    end

end

%% Create solver
gik = generalizedInverseKinematics('RigidBodyTree', rbt, ...
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
% Fix non-finger joints to values obtained from previous iteration
nonFingerIdx = ~startsWith(jointNames,fingerNames{fingerIdx});
upperBounds(nonFingerIdx) = valuesPrev(nonFingerIdx); 
lowerBounds(nonFingerIdx) = valuesPrev(nonFingerIdx); 
jointLimits.Bounds = [lowerBounds, upperBounds];
jointLimits.Weights = 20 * ones(1, 24);

% End effector pose contraints
%lftip_pos = constraintCartesianBounds('lftip', 'ReferenceBody', 'world');
tip_pos = constraintPoseTarget(tip_frame, 'ReferenceBody', 'world');
tip_pos.TargetTransform = tform(targetPose);
tip_pos.OrientationTolerance = deg2rad(180); % allow more leeway for orientation
tip_pos.PositionTolerance = 0;
tip_pos.Weights = [20, 1]; % PositionTolerance and OrientationTolerance
%lftip_pos.Bounds = [0.01, 0.03; 0.03, 0.034; 0.25, 0.35];

%% Run solver
[qSol, solutionInfo] = gik(q0, tip_pos, jointLimits);
solJointValues = vertcat(qSol.JointPosition);
solJointValues(abs(solJointValues) < 1e-3)=0;
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


