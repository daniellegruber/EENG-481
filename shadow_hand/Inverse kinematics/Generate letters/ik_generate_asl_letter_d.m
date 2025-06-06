%% Solve inverse kinematics for each finger tip, one by one

% Reminder: se3(trvec(tip_to_world), 'trvec') * se3(rotm(tip_to_world)) == tip_to_world

mdl = "User input models/shr26df_user_input.slx";
%mdl = "User input models/shl26df_user_input.slx";
rbt = shr26df_rbt;

pose_name = 'letter_d';
q0 = homeConfiguration(rbt);
valuesPrev = zeros(1,nJoints);

xoffset_from_palm = [0.08, 0.077, 0.077];
yoffset_from_knuckle = [0.012, 0.01 0];
zoffset_from_palm = [0.077, 0.077, 0.08];

afterAdjustments = {'ARMJ1', deg2rad(90)};

for fingerIdx = [1 2 3 5] % keep first finger straight
%% Set up target pose of tip
tip_frame = [lower(fingerNames{fingerIdx}),'tip'];
tip_to_world = se3(getTransform(rbt,q0,tip_frame,"world"));
if fingerIdx == 5 % Thumb
    % Get transforms of certain frames relative to world in home config
    mftip_to_world = se3(getTransform(rbt,q0,'mftip',"world"));
    palm_to_world = se3(getTransform(rbt,q0,"palm","world"));
    
    % Create target translation
    trvec_palm = trvec(palm_to_world); 
    trvec_tip = trvec(tip_to_world); 
    trvec_mftip = trvec(mftip_to_world); 
    
    trvec_target = trvec_mftip;
    trvec_target(3) = trvec_target(3) - 0.004;

    % Create distance constraint
    distanceConstraint = constraintPositionTarget(tip_frame);
    distanceConstraint.ReferenceBody = 'world';
    distanceConstraint.TargetPosition = trvec_target;
    distanceConstraint.PositionTolerance = 0;%1e-3;
    
    % Run solver
    jointValues = runGikSolver(rbt, fingerIdx, ...
            valuesPrev, distanceConstraint, []);
else
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