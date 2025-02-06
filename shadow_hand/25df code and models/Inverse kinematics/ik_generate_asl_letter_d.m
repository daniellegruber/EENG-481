%% Solve inverse kinematics for each finger tip, one by one

% Reminder: se3(trvec(tip_to_world), 'trvec') * se3(rotm(tip_to_world)) == tip_to_world

rbt = shr25df_rbt;

pose_name = 'letter_d';
q0 = homeConfiguration(rbt);
valuesPrev = zeros(1,nJoints);

xoffset_from_palm = [0.08, 0.077, 0.077];
yoffset_from_knuckle = [0.012, 0.01 0];
zoffset_from_palm = [0.077, 0.077, 0.08];

afterAdjustments = {'ARMJ1', deg2rad(90)};

for fingerIdx = 1:5
%% Set up target pose of tip
tip_frame = [lower(fingerNames{fingerIdx}),'tip'];
tip_to_world = se3(getTransform(rbt,q0,tip_frame,"world"));
if fingerIdx == 5 % Thumb
    distanceConstraint = constraintPositionTarget(tip_frame);
    distanceConstraint.ReferenceBody = 'world';

    % Get transforms of certain frames relative to world in home config
    mftip_to_world = se3(getTransform(rbt,q0,'mftip',"world"));
    palm_to_world = se3(getTransform(rbt,q0,"palm","world"));
    
    % Create target translation
    trvec_palm = trvec(palm_to_world); 
    trvec_tip = trvec(tip_to_world); 
    trvec_mftip = trvec(mftip_to_world); 
    
    trvec_target = trvec_mftip;
    trvec_target(3) = trvec_target(3) - 0.004;

    distanceConstraint.TargetPosition = trvec_target;
    distanceConstraint.PositionTolerance = 0;%1e-3;
    positionOrPose = 0;
elseif fingerIdx == 4 
    targetPose = tip_to_world;
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
fftip_to_world = se3(getTransform(rbt,q0,'fftip',"world"));
mftip_to_world = se3(getTransform(rbt,q0,'mftip',"world"));
rftip_to_world = se3(getTransform(rbt,q0,'rftip',"world"));
lftip_to_world = se3(getTransform(rbt,q0,'lftip',"world"));
palm_to_world = se3(getTransform(rbt,q0,'palm',"world"));

disp('thtip offsets:\n')
thtip_palm_offset = trvec(thtip_to_world) - trvec(palm_to_world);
thtip_base_offset = trvec(thtip_to_world) - trvec(se3(getTransform(rbt,q0,'thbase',"world")));
thtip_offset = thtip_palm_offset;
thtip_offset(2) = thtip_base_offset(2);
disp(thtip_offset)

% disp('fftip offsets:\n')
% fftip_palm_offset = trvec(fftip_to_world) - trvec(palm_to_world);
% fftip_base_offset = trvec(fftip_to_world) - trvec(se3(getTransform(rbt,q0,'ffknuckle',"world")));
% fftip_offset = fftip_palm_offset;
% fftip_offset(2) = fftip_base_offset(2);
% disp(fftip_offset)
% 
% disp('mftip offsets:\n')
% mftip_palm_offset = trvec(mftip_to_world) - trvec(palm_to_world);
% mftip_base_offset = trvec(mftip_to_world) - trvec(se3(getTransform(rbt,q0,'mfknuckle',"world")));
% mftip_offset = mftip_palm_offset;
% mftip_offset(2) = mftip_base_offset(2);
% disp(mftip_offset)
% 
% disp('rftip offsets:\n')
% rftip_palm_offset = trvec(rftip_to_world) - trvec(palm_to_world);
% rftip_base_offset = trvec(rftip_to_world) - trvec(se3(getTransform(rbt,q0,'rfknuckle',"world")));
% rftip_offset = rftip_palm_offset;
% rftip_offset(2) = rftip_base_offset(2);
% disp(rftip_offset)
% 
% disp('lftip offsets:\n')
% lftip_palm_offset = trvec(lftip_to_world) - trvec(palm_to_world);
% lftip_base_offset = trvec(lftip_to_world) - trvec(se3(getTransform(rbt,q0,'lfknuckle',"world")));
% lftip_offset = lftip_palm_offset;
% lftip_offset(2) = lftip_base_offset(2);
% disp(lftip_offset)