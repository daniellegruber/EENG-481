% thumbOverSigns = addLetterPrefix({'a', 'b', 'h', 'i', 'j', 'k', 'r','s', 'u', 'v', 'w', 'z'});
% thumbUnderSigns = addLetterPrefix({'m', 'n', 'p', 't'});
thumbOverSigns = addLetterPrefix({'a', 'i', 'j', 'k', 'r','s', 'u', 'v', 'w', 'z'});
thumbUnderSigns = addLetterPrefix({'b', 'h', 'm', 'n', 'p', 't'});
%thumbUnderSignLiftedFingers = [3 2 3 2 2 2];
thumbUnderSignLiftedFingers = {...
    {'LF', 'RF', 'MF', 'FF'}, ...
    {'MF', 'FF'}, ...
    {'RF', 'MF', 'FF'}, ...
    {'MF', 'FF'}, ...
    {'MF', 'FF'}, ...
    {'MF', 'FF'} ...
    };
%% Thumb over signs: generate intermediate qWaypoint in which thumb out of way
% This allow non-interfering transition from letter t -> other signs, i.e.,
% makes sure paths of fingers don't cross
% xoffset_from_palm = 0.06;
% yoffset_from_knuckle = 0.02;
% zoffset_from_palm = 0.08;
xoffset_from_palm = 0.07;
yoffset_from_knuckle = 0.02;
zoffset_from_palm = 0.07;

% Get target thtip position
thumbJointsIdx = startsWith(jointNames,'TH');
tip_frame = 'thtip';
tip_to_world = se3(getTransform(rbt,q0,tip_frame,"world"));

distanceConstraint = constraintPositionTarget(tip_frame);
distanceConstraint.ReferenceBody = 'world';

knuckle_frame = 'thbase';

% Get transforms of certain frames relative to world in home config
knuckle_to_world = se3(getTransform(rbt,homeConfiguration(rbt),knuckle_frame,"world"));
palm_to_world = se3(getTransform(rbt,homeConfiguration(rbt),"palm","world"));

% Create target translation
trvec_palm = trvec(palm_to_world); 
trvec_tip = trvec(tip_to_world); 
trvec_knuckle = trvec(knuckle_to_world); 

trvec_target = trvec_palm;
trvec_target(1) = trvec_palm(1) + xoffset_from_palm;
trvec_target(2) = trvec_knuckle(2)+ yoffset_from_knuckle;
trvec_target(3) = trvec_palm(3) + zoffset_from_palm;

distanceConstraint.TargetPosition = trvec_target;
distanceConstraint.PositionTolerance = 0;%1e-3;
positionOrPose = 0;
 
% Solve once for thumb angles, then apply to all thumbOver signs
q0 = homeConfiguration(rbt);

gik = generalizedInverseKinematics('RigidBodyTree', rbt, ...
'ConstraintInputs', {'position','joint'});
   
% Solver parameters
% gik.SolverParameters.MaxIterations = 1500;
gik.SolverParameters.MaxTime = 2;

% Joint constraints -- only want little finger lf to move
jointLimits = constraintJointBounds(rbt);
oldBounds = jointLimits.Bounds;
upperBounds = oldBounds(:,2);
lowerBounds = oldBounds(:,1);
% Fix non-finger joints to values obtained from previous iteration
nonFingerIdx = ~thumbJointsIdx;
upperBounds(nonFingerIdx) = valuesPrev(nonFingerIdx); 
lowerBounds(nonFingerIdx) = valuesPrev(nonFingerIdx); 
jointLimits.Bounds = [lowerBounds, upperBounds];
jointLimits.Weights = 20 * ones(1, nJoints);

% Run solver
[qSol, solutionInfo] = gik(q0, distanceConstraint, jointLimits);
solJointValues = vertcat(qSol.JointPosition);
solJointValues(abs(solJointValues) < 1e-3)=0;

thumbValues = solJointValues(thumbJointsIdx);

% Iterate over signs
for i = 1:length(thumbOverSigns)
    sign_name = thumbOverSigns{i};
    load(['Configs', filesep, sign_name, '.mat'], "jointValues");
    jointDim = find(size(jointValues) == length(jointNames));
    if jointDim ~= 2
        jointValues = jointValues';
    end
    jointValues(end, thumbJointsIdx) = thumbValues;
    jointValues = jointValues(end, :);
    save(['Configs', filesep, sign_name, '_to_over.mat'], "jointValues");
end

%% Thumb under signs: generate intermediate qWaypoint in which RF, MF, FF lifted 
% This allow non-interfering transition from letter t -> other signs, i.e.,
% makes sure paths of fingers don't cross
xoffset_from_palm = [0.08 0.08 0.08 0.08 0.06 0.05];
yoffset_from_knuckle = [0 0 0 0 0.02 0.02];
zoffset_from_palm = [0.12 0.12 0.12 0.12 0.08 0.09];
% xoffset_from_palm = [0.08 0.08 0.08 0.08 0.07 0.05];
% yoffset_from_knuckle = [0 0 0 0 0.02 0.02];
% zoffset_from_palm = [0.12 0.12 0.12 0.12 0.07 0.07];


for toOverOrUnderIdx = 1:2
    q0 = homeConfiguration(rbt);
    valuesPrev = zeros(1, nJoints);
    
    for fingerIdx = 1:5
        tip_frame = [lower(fingerNames{fingerIdx}),'tip'];
    
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
            [qSol, ~] = gik(q0, tip_pos, jointLimits);
        else
            [qSol, ~] = gik(q0, distanceConstraint, jointLimits);
        end
        solJointValues = vertcat(qSol.JointPosition);
        solJointValues(abs(solJointValues) < 1e-3)=0;
    
        if fingerIdx == 5 
            jointValues = solJointValues;
            if toOverOrUnderIdx == 1
                % For transitions to signs where the thumb is over another
                % finger
                save(['Configs', filesep, 'under_to_over.mat'], "jointValues");
            else
                % For transitions to signs where the thumb is under another
                % finger
                save(['Configs', filesep, 'under_to_under.mat'], "jointValues");
            end
            jointValuesToInputSignals(solJointValues, jointNames, 0.001, 2, ...
                ['signals_after_solving_', fingerNames{fingerIdx}]);
        end
        valuesPrev = solJointValues;
        q0 = jointValuesToConfigObj(solJointValues, jointNames); % Initial config for next iteration
    end
    %supplyInputToUserInputMdlByMat(mdl, 'Signals/signals_after_solving_TH.mat');
end


load(['Configs', filesep, 'under_to_over.mat'], "jointValues");
underToOverJointValues = correctJointValueDims(jointValues, nJoints);
load(['Configs', filesep, 'under_to_under.mat'], "jointValues");
underToUnderJointValues = correctJointValueDims(jointValues, nJoints);

% Iterate over signs
for i = 1:length(thumbUnderSigns)
    sign_name = thumbUnderSigns{i};
    load(['Configs', filesep, sign_name, '.mat'], "jointValues");
    originalJointValues = correctJointValueDims(jointValues, nJoints);

    jointValues = originalJointValues(end,:);
    for fingerName = thumbUnderSignLiftedFingers{i}
        fingerJointsIdx = startsWith(jointNames,fingerName);
        jointValues(fingerJointsIdx) = underToOverJointValues(fingerJointsIdx);
    end

    thumbJointsIdx = startsWith(jointNames,"TH");
    jointValues(thumbJointsIdx) = underToOverJointValues(thumbJointsIdx);
    save(['Configs', filesep, sign_name, '_to_over.mat'], "jointValues");
    jointValues(thumbJointsIdx) = underToUnderJointValues(thumbJointsIdx);
    save(['Configs', filesep, sign_name, '_to_under.mat'], "jointValues");
end

%% Letter k: generate intermediate waypoints for transitions to u, v
% This allow non-interfering transition from letter t -> other signs, i.e.,
% makes sure paths of fingers don't cross
xoffset_from_palm = 0.06;
yoffset_from_knuckle = -0.01;
% yoffset_from_knuckle = 0;
zoffset_from_palm = 0.08;

% Get target thtip position
thumbJointsIdx = startsWith(jointNames,'TH');
tip_frame = 'thtip';
tip_to_world = se3(getTransform(rbt,q0,tip_frame,"world"));

distanceConstraint = constraintPositionTarget(tip_frame);
distanceConstraint.ReferenceBody = 'world';

knuckle_frame = 'thbase';

% Get transforms of certain frames relative to world in home config
knuckle_to_world = se3(getTransform(rbt,homeConfiguration(rbt),knuckle_frame,"world"));
palm_to_world = se3(getTransform(rbt,homeConfiguration(rbt),"palm","world"));

% Create target translation
trvec_palm = trvec(palm_to_world); 
trvec_tip = trvec(tip_to_world); 
trvec_knuckle = trvec(knuckle_to_world); 

trvec_target = trvec_palm;
trvec_target(1) = trvec_palm(1) + xoffset_from_palm;
trvec_target(2) = trvec_knuckle(2)+ yoffset_from_knuckle;
trvec_target(3) = trvec_palm(3) + zoffset_from_palm;

distanceConstraint.TargetPosition = trvec_target;
distanceConstraint.PositionTolerance = 0;%1e-3;
positionOrPose = 0;
 
% Solve once for thumb angles, then apply to all thumbOver signs
q0 = homeConfiguration(rbt);

gik = generalizedInverseKinematics('RigidBodyTree', rbt, ...
'ConstraintInputs', {'position','joint'});
   
% Solver parameters
% gik.SolverParameters.MaxIterations = 1500;
gik.SolverParameters.MaxTime = 2;

% Joint constraints -- only want little finger lf to move
jointLimits = constraintJointBounds(rbt);
oldBounds = jointLimits.Bounds;
upperBounds = oldBounds(:,2);
lowerBounds = oldBounds(:,1);
% Fix non-finger joints to values obtained from previous iteration
nonFingerIdx = ~thumbJointsIdx;
upperBounds(nonFingerIdx) = valuesPrev(nonFingerIdx); 
lowerBounds(nonFingerIdx) = valuesPrev(nonFingerIdx); 
jointLimits.Bounds = [lowerBounds, upperBounds];
jointLimits.Weights = 20 * ones(1, nJoints);

% Run solver
[qSol, solutionInfo] = gik(q0, distanceConstraint, jointLimits);
solJointValues = vertcat(qSol.JointPosition);
solJointValues(abs(solJointValues) < 1e-3)=0;

thumbValues = solJointValues(thumbJointsIdx);

sign_name = 'letter_k';
load(['Configs', filesep, sign_name, '.mat'], "jointValues");
jointDim = find(size(jointValues) == length(jointNames));
if jointDim ~= 2
    jointValues = jointValues';
end
jointValues(end, thumbJointsIdx) = thumbValues;
jointValues = jointValues(end, :);
save(['Configs', filesep, sign_name, '_to_uv.mat'], "jointValues");

jointValuesToInputSignals(jointValues', jointNames, 0.001, 2, ...
                    'signals_after_solving_TH');
mdl = "User input models/shr26df_user_input.slx";
%mdl = "User input models/shl26df_user_input.slx";

% Show robotic hand
%supplyInputToUserInputMdlByMat(mdl, 'Signals/signals_after_solving_TH.mat');

% For debugging
q0 = jointValuesToConfigObj(jointValues', jointNames);
thtip_to_world = se3(getTransform(rbt,q0,'thtip',"world"));
palm_to_world = se3(getTransform(rbt,homeConfiguration(rbt),'palm',"world"));
thbase_to_world = se3(getTransform(rbt,homeConfiguration(rbt),'thbase',"world"));

disp('thtip offsets:\n')
thtip_offset = trvec(thtip_to_world) - trvec(palm_to_world);
thtip_thbase_offset = trvec(thtip_to_world) - trvec(thbase_to_world);
thtip_offset(2) = thtip_thbase_offset(2);
disp(thtip_offset)

%% Letter e: generate intermediate waypoints for transitions to u, v
% This allow non-interfering transition from letter t -> other signs, i.e.,
% makes sure paths of fingers don't cross
% xoffset_from_palm = 0.07;
% yoffset_from_knuckle = 0.02;
% zoffset_from_palm = 0.06;

xoffset_from_palm = 0.07;
yoffset_from_knuckle = 0.01;
zoffset_from_palm = 0.05;

% Get target thtip position
thumbJointsIdx = startsWith(jointNames,'TH');
tip_frame = 'thtip';
tip_to_world = se3(getTransform(rbt,q0,tip_frame,"world"));

distanceConstraint = constraintPositionTarget(tip_frame);
distanceConstraint.ReferenceBody = 'world';

knuckle_frame = 'thbase';

% Get transforms of certain frames relative to world in home config
knuckle_to_world = se3(getTransform(rbt,homeConfiguration(rbt),knuckle_frame,"world"));
palm_to_world = se3(getTransform(rbt,homeConfiguration(rbt),"palm","world"));

% Create target translation
trvec_palm = trvec(palm_to_world); 
trvec_tip = trvec(tip_to_world); 
trvec_knuckle = trvec(knuckle_to_world); 

trvec_target = trvec_palm;
trvec_target(1) = trvec_palm(1) + xoffset_from_palm;
trvec_target(2) = trvec_knuckle(2)+ yoffset_from_knuckle;
trvec_target(3) = trvec_palm(3) + zoffset_from_palm;

distanceConstraint.TargetPosition = trvec_target;
distanceConstraint.PositionTolerance = 0;%1e-3;
positionOrPose = 0;
 
% Solve once for thumb angles, then apply to all thumbOver signs
q0 = homeConfiguration(rbt);

gik = generalizedInverseKinematics('RigidBodyTree', rbt, ...
'ConstraintInputs', {'position','joint'});
   
% Solver parameters
% gik.SolverParameters.MaxIterations = 1500;
gik.SolverParameters.MaxTime = 2;

% Joint constraints -- only want little finger lf to move
jointLimits = constraintJointBounds(rbt);
oldBounds = jointLimits.Bounds;
upperBounds = oldBounds(:,2);
lowerBounds = oldBounds(:,1);
% Fix non-finger joints to values obtained from previous iteration
nonFingerIdx = ~thumbJointsIdx;
upperBounds(nonFingerIdx) = valuesPrev(nonFingerIdx); 
lowerBounds(nonFingerIdx) = valuesPrev(nonFingerIdx); 
jointLimits.Bounds = [lowerBounds, upperBounds];
jointLimits.Weights = 20 * ones(1, nJoints);

% Run solver
[qSol, solutionInfo] = gik(q0, distanceConstraint, jointLimits);
solJointValues = vertcat(qSol.JointPosition);
solJointValues(abs(solJointValues) < 1e-3)=0;

thumbValues = solJointValues(thumbJointsIdx);

sign_name = 'letter_e';
load(['Configs', filesep, sign_name, '.mat'], "jointValues");
jointDim = find(size(jointValues) == length(jointNames));
if jointDim ~= 2
    jointValues = jointValues';
end
jointValues(end, thumbJointsIdx) = thumbValues;
jointValues = jointValues(end, :);
save(['Configs', filesep, sign_name, '_intermediate.mat'], "jointValues");

jointValuesToInputSignals(jointValues', jointNames, 0.001, 2, ...
                    'signals_after_solving_TH');
mdl = "User input models/shr26df_user_input.slx";
%mdl = "User input models/shl26df_user_input.slx";

% % Show robotic hand
% supplyInputToUserInputMdlByMat(mdl, 'Signals/signals_after_solving_TH.mat');
% 
% % For debugging
% q0 = jointValuesToConfigObj(jointValues', jointNames);
% thtip_to_world = se3(getTransform(rbt,q0,'thtip',"world"));
% palm_to_world = se3(getTransform(rbt,homeConfiguration(rbt),'palm',"world"));
% thbase_to_world = se3(getTransform(rbt,homeConfiguration(rbt),'thbase',"world"));
% 
% disp('thtip offsets:\n')
% thtip_offset = trvec(thtip_to_world) - trvec(palm_to_world);
% thtip_thbase_offset = trvec(thtip_to_world) - trvec(thbase_to_world);
% thtip_offset(2) = thtip_thbase_offset(2);
% disp(thtip_offset)
% 
% % Generate trajectory between signs
% signSeq = {'letter_s', 'letter_e_to_intermediate', 'letter_e', 'letter_e_to_intermediate', 'letter_s'};
% [ds, lastConfig] = genConfigTrajectoryFromInput(signSeq, jointNames);
% 
% % Show robotic hand
% supplyInputToUserInputMdlByDs(mdl, ds);
%% Helper functions
function signSeq = addLetterPrefix(letterCell)
    signSeq = cellfun(@(x) ['letter_', x], letterCell, 'UniformOutput', false);
end

function jointValues = correctJointValueDims(jointValues, nJoints)
    jointDim = find(size(jointValues) == nJoints);
    if jointDim ~= 2
        jointValues = jointValues';
    end
end