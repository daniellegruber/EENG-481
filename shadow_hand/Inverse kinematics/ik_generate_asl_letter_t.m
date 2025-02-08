%% Solve inverse kinematics for each finger tip, one by one

% Reminder: se3(trvec(tip_to_world), 'trvec') * se3(rotm(tip_to_world)) == tip_to_world

rbt = shr26df_rbt;

pose_name = 'letter_t';
q0 = homeConfiguration(rbt);
valuesPrev = zeros(1,nJoints);

% xoffset_from_palm = [0 0 0 0.04];
% yoffset_from_knuckle = [0.012, 0.01 0, -0.005];
% zoffset_from_palm = [0.04, 0.04, 0.04, 0.06];
% xoffset_from_palm = [0 0 0 0.05];
% yoffset_from_knuckle = [0.012, 0.01 0, 0.01];
% zoffset_from_palm = [0.04, 0.04, 0.04, 0.065];
xoffset_from_palm = [0 0 0 0.046];
yoffset_from_knuckle = [0.012, 0.01 0, 0.01];
zoffset_from_palm = [0.04, 0.04, 0.04, 0.06];

%afterAdjustments = {'FFJ4', deg2rad(-20)};
afterAdjustments = {};

for fingerIdx = [1 2 3 5 4] % do thtip after mftip
%% Set up target pose of tip
tip_frame = [lower(fingerNames{fingerIdx}),'tip'];
tip_to_world = se3(getTransform(rbt,q0,tip_frame,"world"));
if fingerIdx == 5 % Thumb
    % distanceConstraint = constraintPositionTarget(tip_frame);
    % distanceConstraint.ReferenceBody = 'world';

    % Get transforms of certain frames relative to world in home config
    mfknuckle_to_world = se3(getTransform(rbt,q0,"mfknuckle","world"));

    % Create target translation
    trvec_mfknuckle = trvec(mfknuckle_to_world); 
    trvec_tip = trvec(tip_to_world); 

    trvec_target = trvec_mfknuckle;
    trvec_target(1) = trvec_target(1) + 0.03;
    trvec_target(2) = trvec_target(2) + 0.01;
    trvec_target(3) = trvec_target(3) + 0.02;

    % distanceConstraint.TargetPosition = trvec_target;
    % distanceConstraint.PositionTolerance = 0;%1e-3;
    % positionOrPose = 0;

    T1 = se3(trvec_target, "trvec");
    % 
    % Get target rotation
    R1 = se3([deg2rad(0), 0, 0],"eul","XYZ"); 

    % Create target pose
    targetPose = T1 * R1;
    positionOrPose = 1;
else 
    distanceConstraint = constraintPositionTarget(tip_frame);
    distanceConstraint.ReferenceBody = 'world';

    knuckle_frame = [lower(fingerNames{fingerIdx}),'knuckle'];
    
    % Get transforms of certain frames relative to world in home config
    knuckle_to_world = se3(getTransform(rbt,q0,knuckle_frame,"world"));
    palm_to_world = se3(getTransform(rbt,q0,"palm","world"));
    
    % Create target translation
    trvec_palm = trvec(palm_to_world); 
    trvec_tip = trvec(tip_to_world); 
    trvec_knuckle = trvec(knuckle_to_world); 
    
    trvec_target = trvec_palm;
    trvec_target(1) = trvec_palm(1) + xoffset_from_palm(fingerIdx);
    trvec_target(2) = trvec_knuckle(2)+ yoffset_from_knuckle(fingerIdx);
    trvec_target(3) = trvec_palm(3) + zoffset_from_palm(fingerIdx);

    distanceConstraint.TargetPosition = trvec_target;
    distanceConstraint.PositionTolerance = 0;%1e-3;
    positionOrPose = 0;
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
    tip_pos.OrientationTolerance = deg2rad(50); % allow more leeway for orientation
    tip_pos.PositionTolerance = 0;
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
if fingerIdx == 4
    upperBounds(ismember(jointNames,'FFJ4')) = deg2rad(-40);
    lowerBounds(ismember(jointNames,'FFJ4')) = deg2rad(-40);
end
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

if fingerIdx == 4 
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

mdl = "User input models/shr26df_user_input.slx";
%mdl = "User input models/shl26df_user_input.slx";

% Show robotic hand
%supplyInputToUserInputMdlByMat(mdl, 'Signals/signals_after_solving_TH.mat');
supplyInputToUserInputMdlByMat(mdl, 'Signals/signals_after_solving_FF.mat');

%% For debugging
thtip_to_world = se3(getTransform(rbt,q0,'thtip',"world"));
fftip_to_world = se3(getTransform(rbt,q0,'fftip',"world"));
mftip_to_world = se3(getTransform(rbt,q0,'mftip',"world"));
rftip_to_world = se3(getTransform(rbt,q0,'rftip',"world"));
lftip_to_world = se3(getTransform(rbt,q0,'lftip',"world"));
palm_to_world = se3(getTransform(rbt,homeConfiguration(rbt),'palm',"world"));

% disp('thtip offsets:\n')
% thtip_offset = trvec(thtip_to_world) - trvec_mfknuckle;
% disp(thtip_offset)
% disp(rad2deg(rotm2eul(rotm(thtip_to_world), 'XYZ')))

disp('fftip offsets:\n')
fftip_palm_offset = trvec(fftip_to_world) - trvec(palm_to_world);
fftip_base_offset = trvec(fftip_to_world) - trvec(se3(getTransform(rbt,homeConfiguration(rbt),'ffknuckle',"world")));
fftip_offset = fftip_palm_offset;
fftip_offset(2) = fftip_base_offset(2);
disp(fftip_offset)

%% Generate intermediate qWaypoint in which MF, FF lifted 
% This allow non-interfering transition from letter t -> other signs, i.e.,
% makes sure paths of fingers don't cross
xoffset_from_palm = [0 0 0.08 0.08 0.06 0.05];
yoffset_from_knuckle = [0 0 0 0 0.02 0.02];
zoffset_from_palm = [0, 0 0.12 0.12 0.08 0.09];

for toOverOrUnderIdx = 1:2
load(['Configs', filesep, 'letter_t.mat'], "jointValues");
valuesPrev = jointValues;
q0 = jointValuesToConfigObj(jointValues, jointNames); % Initial config for next iteration

for fingerIdx = [3 4 5]
    tip_frame = [lower(fingerNames{fingerIdx}),'tip'];
    tip_to_world = se3(getTransform(rbt,q0,tip_frame,"world"));

    distanceConstraint = constraintPositionTarget(tip_frame);
    distanceConstraint.ReferenceBody = 'world';

    if fingerIdx == 5
        knuckle_frame = [lower(fingerNames{fingerIdx}),'base'];
    else
        knuckle_frame = [lower(fingerNames{fingerIdx}),'knuckle'];
    end

    % Get transforms of certain frames relative to world in home config
    knuckle_to_world = se3(getTransform(rbt,homeConfiguration(rbt),knuckle_frame,"world"));
    palm_to_world = se3(getTransform(rbt,homeConfiguration(rbt),"palm","world"));

    % Create target translation
    trvec_palm = trvec(palm_to_world); 
    trvec_tip = trvec(tip_to_world); 
    trvec_knuckle = trvec(knuckle_to_world); 

    trvec_target = trvec_palm;
    if fingerIdx == 5 && toOverOrUnderIdx == 2
        trvec_target(1) = trvec_palm(1) + xoffset_from_palm(6);
        trvec_target(2) = trvec_knuckle(2)+ yoffset_from_knuckle(6);
        trvec_target(3) = trvec_palm(3) + zoffset_from_palm(6);
    else
        trvec_target(1) = trvec_palm(1) + xoffset_from_palm(fingerIdx);
        trvec_target(2) = trvec_knuckle(2)+ yoffset_from_knuckle(fingerIdx);
        trvec_target(3) = trvec_palm(3) + zoffset_from_palm(fingerIdx);
    end

    distanceConstraint.TargetPosition = trvec_target;
    distanceConstraint.PositionTolerance = 0;%1e-3;
    positionOrPose = 0;

    % Create solver
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
        tip_pos.OrientationTolerance = deg2rad(50); % allow more leeway for orientation
        tip_pos.PositionTolerance = 0;
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

    % Run solver
    if positionOrPose == 1
        [qSol, solutionInfo] = gik(q0, tip_pos, jointLimits);
    else
        [qSol, solutionInfo] = gik(q0, distanceConstraint, jointLimits);
    end
    solJointValues = vertcat(qSol.JointPosition);
    solJointValues(abs(solJointValues) < 1e-3)=0;

    if fingerIdx == 5 
        jointValues = solJointValues;
        if toOverOrUnderIdx == 1
            % For transitions to signs where the thumb is over another
            % finger
            save(['Configs', filesep, 'letter_t_to_over.mat'], "jointValues");
        else
            % For transitions to signs where the thumb is under another
            % finger
            save(['Configs', filesep, 'letter_t_to_under.mat'], "jointValues");
        end
    end
    valuesPrev = solJointValues;
    q0 = jointValuesToConfigObj(solJointValues, jointNames); % Initial config for next iteration

    % Create signals to provide to right_test_asl_poses.slx
    jointValuesToInputSignals(solJointValues, jointNames, 0.001, 2, ...
         ['signals_after_solving_', fingerNames{fingerIdx}]);

end
supplyInputToUserInputMdlByMat(mdl, 'Signals/signals_after_solving_TH.mat');

% For debugging
thtip_to_world = se3(getTransform(rbt,q0,'thtip',"world"));
fftip_to_world = se3(getTransform(rbt,q0,'fftip',"world"));
mftip_to_world = se3(getTransform(rbt,q0,'mftip',"world"));
palm_to_world = se3(getTransform(rbt,homeConfiguration(rbt),'palm',"world"));
thbase_to_world = se3(getTransform(rbt,homeConfiguration(rbt),'thbase',"world"));

% disp('fftip offsets:\n')
% fftip_palm_offset = trvec(fftip_to_world) - trvec(palm_to_world);
% fftip_base_offset = trvec(fftip_to_world) - trvec(se3(getTransform(rbt,homeConfiguration(rbt),'ffknuckle',"world")));
% fftip_offset = fftip_palm_offset;
% fftip_offset(2) = fftip_base_offset(2);
% disp(fftip_offset)

disp('thtip offsets:\n')
thtip_offset = trvec(thtip_to_world) - trvec(palm_to_world);
thtip_thbase_offset = trvec(thtip_to_world) - trvec(thbase_to_world);
thtip_offset(2) = thtip_thbase_offset(2);
disp(thtip_offset)
end