thumbOverSigns = addLetterPrefix({'b','i', 'j', 'r','s', 'u', 'v', 'w', 'z'});

%% Generate intermediate qWaypoint in which thumb out of way
% This allow non-interfering transition from letter t -> other signs, i.e.,
% makes sure paths of fingers don't cross
xoffset_from_palm = 0.06;
yoffset_from_knuckle = 0.02;
zoffset_from_palm = 0.08;

%% Get target thtip position
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

%% Iterate over signs
for i = 1:length(thumbOverSigns)
    sign_name = thumbOverSigns{i};
    load(['Configs', filesep, sign_name, '.mat'], "jointValues");
    valuesPrev = jointValues;
    q0 = jointValuesToConfigObj(jointValues, jointNames); % Initial config for next iteration

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

    jointValues = solJointValues;
    save(['Configs', filesep, sign_name, '_to_over.mat'], "jointValues");
end


function signSeq = addLetterPrefix(letterCell)
    signSeq = cellfun(@(x) ['letter_', x], letterCell, 'UniformOutput', false);
end