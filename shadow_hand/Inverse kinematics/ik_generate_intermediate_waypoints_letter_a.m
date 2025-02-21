%% Setup
mdl = "User input models/shr26df_user_input.slx";
rbt = shr26df_rbt;

%% Letter a -> something
toLetter = 'z';
% [ds1, qInterp] = genConfigTrajectoryNoInterferenceAvodiance({'letter_a', ['letter_', toLetter]}, jointNames);
[ds1, qInterp] = genConfigTrajectoryNoInterferenceAvodiance({'letter_a', 'letter_z_stage_1'}, jointNames);
supplyInputToUserInputMdlByDs(mdl, ds1);
%%
intermediatePointProp = 0.3;
intermediatePointIdx = floor(size(qInterp,2) * intermediatePointProp);
jointValuesIntermediate = qInterp(:,intermediatePointIdx);
qIntermediate = jointValuesToConfigObj(jointValuesIntermediate, jointNames);

ds = jointValuesToInputSignals(jointValuesIntermediate, jointNames, 0.001, 2, '');
supplyInputToUserInputMdlByDs(mdl, ds);

%% Letter a -> something adjustments

% to letter b
% x_offset = 0.02;
% y_offset = 0.02;
% z_offset = 0;

% to letter c
% x_offset = 0.01;
% y_offset = 0.01;
% z_offset = 0;

% to letter d
% x_offset = 0;
% y_offset = 0.015;
% z_offset = 0;

% to letter e
% x_offset = 0.04;
% y_offset = 0;
% z_offset = -0.01;

% to letter h
% x_offset = -0.04;
% y_offset = 0.04;
% z_offset = 0.01;

% to letter k
% x_offset = 0;
% y_offset = 0.01;
% z_offset = 0;

% to letter p
% x_offset = 0;
% y_offset = 0.03;
% z_offset = -0.04;

% to letter r
% x_offset = 0;
% y_offset = 0.02;
% z_offset = 0;

% to letter s
% x_offset = 0.01;
% y_offset = 0.01;
% z_offset = 0;

% to letter u
% x_offset = 0;
% y_offset = 0.02;
% z_offset = 0;

% to letter w
% x_offset = 0;
% y_offset = 0.02;
% z_offset = 0;

% to letter x
% x_offset = -0.02;
% y_offset = 0.02;
% z_offset = 0;

% to letter z
x_offset = 0;
y_offset = 0.02;
z_offset = 0;

fingerIdx = 5;
tip_frame = [lower(fingerNames{fingerIdx}),'tip'];

% Get transforms of certain frames relative to world in home config
tip_to_world_q0 = se3(getTransform(rbt,qIntermediate, tip_frame,"world"));


% Create target translation
trvec_target = trvec(tip_to_world_q0);
trvec_target(1) = trvec_target(1) + x_offset;
trvec_target(2) = trvec_target(2) + y_offset;
trvec_target(3) = trvec_target(3) + z_offset;

% Create distance constraint on tip frame
distanceConstraint = constraintPositionTarget(tip_frame);
distanceConstraint.ReferenceBody = 'world';
distanceConstraint.TargetPosition = trvec_target;
distanceConstraint.PositionTolerance = 0;%1e-3;

jointValues = runGikSolver(rbt, fingerIdx, ...
    jointValuesIntermediate, distanceConstraint, []);
%save(['Configs', filesep, 'transition_a_to_b.mat'], "jointValues");
save(['Configs', filesep, 'transition_a_to_', toLetter, '.mat'], "jointValues");

disp('thtip offsets:\n')
trvec_q0 = trvec(tip_to_world_q0);
qCurr = jointValuesToConfigObj(jointValues, jointNames);
trvec_qCurr = trvec(se3(getTransform(rbt,qCurr,'thtip',"world")));
disp(trvec_qCurr - trvec_q0)

ds = jointValuesToInputSignals(jointValues, jointNames, 0.001, 2, '');
supplyInputToUserInputMdlByDs(mdl, ds);

%% Compare trajectories

[ds2, ~] = genConfigTrajectoryNoInterferenceAvodiance({'letter_a', ['transition_a_to_', toLetter], ['letter_', toLetter]}, jointNames);

supplyInputToUserInputMdlByDs(mdl, ds2);
pause(8);

%% Letter a -> something (multiple fingers problematic)

toLetter = 't';
[ds1, qInterp] = genConfigTrajectoryNoInterferenceAvodiance({'letter_a', ['letter_', toLetter]}, jointNames);
%supplyInputToUserInputMdlByDs(mdl, ds1);

intermediatePointProp = 0.3;
intermediatePointIdx = floor(size(qInterp,2) * intermediatePointProp);
jointValuesIntermediate = qInterp(:,intermediatePointIdx);
qIntermediate = jointValuesToConfigObj(jointValuesIntermediate, jointNames);

ds = jointValuesToInputSignals(jointValuesIntermediate, jointNames, 0.001, 2, '');
supplyInputToUserInputMdlByDs(mdl, ds);
%%
% to letter m
% x_offset = [0 0.01 0.01 0.01 0];
% y_offset = [0 0 0 0 0.02];
% z_offset = [0 0.01 0.01 0.01 -0.02];

% to letter t
x_offset = [0 0 0 0.03 0];
y_offset = [0 0 0 0.01 0.02];
z_offset = [0 0 0 0.04 -0.04];

valuesPrev = jointValuesIntermediate;
for fingerIdx = 1:5
tip_frame = [lower(fingerNames{fingerIdx}),'tip'];

% Get transforms of certain frames relative to world in home config
tip_to_world_q0 = se3(getTransform(rbt,qIntermediate, tip_frame,"world"));


% Create target translation
trvec_target = trvec(tip_to_world_q0);
trvec_target(1) = trvec_target(1) + x_offset(fingerIdx);
trvec_target(2) = trvec_target(2) + y_offset(fingerIdx);
trvec_target(3) = trvec_target(3) + z_offset(fingerIdx);

% Create distance constraint on tip frame
distanceConstraint = constraintPositionTarget(tip_frame);
distanceConstraint.ReferenceBody = 'world';
distanceConstraint.TargetPosition = trvec_target;
distanceConstraint.PositionTolerance = 0;%1e-3;

jointValues = runGikSolver(rbt, fingerIdx, ...
    valuesPrev, distanceConstraint, []);
valuesPrev = jointValues;

disp([tip_frame, ' offsets:\n'])
trvec_q0 = trvec(tip_to_world_q0);
qCurr = jointValuesToConfigObj(jointValues, jointNames);
trvec_qCurr = trvec(se3(getTransform(rbt,qCurr,tip_frame,"world")));
disp(trvec_qCurr - trvec_q0)

end
save(['Configs', filesep, 'transition_a_to_', toLetter, '.mat'], "jointValues");
ds = jointValuesToInputSignals(jointValues, jointNames, 0.001, 2, '');
supplyInputToUserInputMdlByDs(mdl, ds);

[ds2, ~] = genConfigTrajectoryNoInterferenceAvodiance({'letter_a', ['transition_a_to_', toLetter], ['letter_', toLetter]}, jointNames);

supplyInputToUserInputMdlByDs(mdl, ds2);
pause(8);

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