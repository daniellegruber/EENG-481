%% Setup
mdl = "User input models/shr26df_user_input.slx";
rbt = shr26df_rbt;

%% Letter b -> something
toLetter = 's';
[ds1, qInterp] = genConfigTrajectoryNoInterferenceAvoidance({'letter_b', ['letter_', toLetter]}, jointNames);
%[ds1, qInterp] = genConfigTrajectoryNoInterferenceAvoidance({'letter_b', 'letter_z_stage_1'}, jointNames);
supplyInputToUserInputMdlByDs(mdl, ds1);
%%
intermediatePointProp = 0.5;
intermediatePointIdx = floor(size(qInterp,2) * intermediatePointProp);
jointValuesIntermediate = qInterp(:,intermediatePointIdx);
qIntermediate = jointValuesToConfigObj(jointValuesIntermediate, jointNames);

ds = jointValuesToInputSignals(jointValuesIntermediate, jointNames, 0.001, 2, '');
supplyInputToUserInputMdlByDs(mdl, ds);

%% Letter b -> something adjustments

% to letter e
% x_offset = 0.04;
% y_offset = 0.03;
% z_offset = 0;

% to letter i
% x_offset = 0.04;
% y_offset = 0.03;
% z_offset = 0;

% to letter r
% x_offset = 0.01;
% y_offset = 0.01;
% z_offset = 0;

% to letter s
x_offset = 0.02;
y_offset = 0.04;
z_offset = 0;

% to letter u
% x_offset = 0.01;
% y_offset = 0.01;
% z_offset = 0;

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
save(['Configs', filesep, 'transition_b_to_', toLetter, '.mat'], "jointValues");

disp('thtip offsets:\n')
trvec_q0 = trvec(tip_to_world_q0);
qCurr = jointValuesToConfigObj(jointValues, jointNames);
trvec_qCurr = trvec(se3(getTransform(rbt,qCurr,'thtip',"world")));
disp(trvec_qCurr - trvec_q0)

ds = jointValuesToInputSignals(jointValues, jointNames, 0.001, 2, '');
supplyInputToUserInputMdlByDs(mdl, ds);

%% Compare trajectories

[ds2, ~] = genConfigTrajectoryNoInterferenceAvoidance({'letter_b', ['transition_b_to_', toLetter], ['letter_', toLetter]}, jointNames);

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