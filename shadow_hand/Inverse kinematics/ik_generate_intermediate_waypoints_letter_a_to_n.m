%% Letter a -> letter n requires multiple points
[ds1, qInterp] = genConfigTrajectoryNoInterferenceAvodiance({'letter_a', 'letter_n'}, jointNames);
%supplyInputToUserInputMdlByDs(mdl, ds1);
intermediatePointProp = [0.5, 0.7];

x_offset = {[0 0 0.05 0.05 0], [0 0 0.05 0.05 0]};
y_offset = {[0 0 0 0 0.04], [0 0 0 0 0]};
z_offset = {[0 0 0.02 0.02 -0.04], [0 0 0.02 0.02 -0.02]};

for i = 1:length(intermediatePointProp)
    intermediatePointIdx = floor(size(qInterp,2) * intermediatePointProp(i));
    jointValuesIntermediate = qInterp(:,intermediatePointIdx);
    qIntermediate = jointValuesToConfigObj(jointValuesIntermediate, jointNames);
    % ds = jointValuesToInputSignals(jointValuesIntermediate, jointNames, 0.001, 2, '');
    % supplyInputToUserInputMdlByDs(mdl, ds);
    
    valuesPrev = jointValuesIntermediate;
    for fingerIdx = 1:5
        tip_frame = [lower(fingerNames{fingerIdx}),'tip'];
        
        % Get transforms of certain frames relative to world in home config
        tip_to_world_q0 = se3(getTransform(rbt,qIntermediate, tip_frame,"world"));
        
        
        % Create target translation
        trvec_target = trvec(tip_to_world_q0);
        trvec_target(1) = trvec_target(1) + x_offset{i}(fingerIdx);
        trvec_target(2) = trvec_target(2) + y_offset{i}(fingerIdx);
        trvec_target(3) = trvec_target(3) + z_offset{i}(fingerIdx);
        
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
    save(['Configs', filesep, 'transition_a_to_n_', num2str(i), '.mat'], "jointValues");
    ds = jointValuesToInputSignals(jointValues, jointNames, 0.001, 2, '');
    supplyInputToUserInputMdlByDs(mdl, ds);
end
% Compare trajectories
[ds2, ~] = genConfigTrajectoryNoInterferenceAvodiance({'letter_a', 'transition_a_to_n_1','transition_a_to_n_2', 'letter_n'}, jointNames);
supplyInputToUserInputMdlByDs(mdl, ds2);
pause(8);
