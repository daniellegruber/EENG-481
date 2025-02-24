%% Solve inverse kinematics for each finger tip, one by one

% Reminder: se3(trvec(tip_to_world), 'trvec') * se3(rotm(tip_to_world)) == tip_to_world

rbt = shr26df_rbt;

pose_name = 'letter_q';
q0 = homeConfiguration(rbt);
valuesPrev = zeros(1,nJoints);

% xoffset_from_palm = [0 0 0 0];
% yoffset_from_knuckle = [0.012, 0.01 0, -0.01];
% zoffset_from_palm = [0.04, 0.02, 0.02, 0.02];
xoffset_from_palm = [0 0 0 0.096];
yoffset_from_knuckle = [0.012, 0.01 0, 0];
zoffset_from_palm = [0.04, 0.04, 0.04, 0.1];

afterAdjustments = {'ARMJ1', deg2rad(90), 'WRJ1', deg2rad(60)};
% afterAdjustments = {};

for fingerIdx = 1:5
%% Set up target pose of tip
tip_frame = [lower(fingerNames{fingerIdx}),'tip'];
tip_to_world = se3(getTransform(rbt,q0,tip_frame,"world"));
if fingerIdx == 5 % Thumb
    distanceConstraint = constraintPositionTarget(tip_frame);
    distanceConstraint.ReferenceBody = 'world';

    % Get transforms of certain frames relative to world in home config
    fftip_to_world = se3(getTransform(rbt,q0,"fftip","world"));

    % Create target translation
    trvec_fftip = trvec(fftip_to_world); 
    trvec_tip = trvec(tip_to_world); 

    trvec_target = trvec_fftip;
    trvec_target(1) = trvec_target(1) + 0.005;
    %trvec_target(2) = trvec_target(2) + 0.01;
    trvec_target(3) = trvec_target(3) - 0.04;

    distanceConstraint.TargetPosition = trvec_target;
    distanceConstraint.PositionTolerance = 0;%1e-3;
    positionOrPose = 0;
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

% Run solver
jointValues = runGikSolver(rbt, fingerIdx, ...
            valuesPrev, distanceConstraint, []);
valuesPrev = jointValues;

% Perform after-adjustments
if fingerIdx == 5 
    if ~isempty(afterAdjustments)
        for i = 1:2:length(afterAdjustments)
            jointIdx = contains(jointNames, afterAdjustments{i});
            jointValue = afterAdjustments{i+1};
            jointValues(jointIdx) = jointValue;
        end
    end
    jointValuesToInputSignals(jointValues, jointNames, 0.001, 2, pose_name);
    save(['Configs', filesep, pose_name, '.mat'], "jointValues");
end

q0 = jointValuesToConfigObj(jointValues, jointNames); % Initial config for next iteration

%% Create signals to provide to right_test_asl_poses.slx
jointValuesToInputSignals(jointValues, jointNames, 0.001, 2, ...
     ['signals_after_solving_', fingerNames{fingerIdx}]);

% jointValuesToInputSignals(solJointValues, jointNames, 0.001, 2, ...
%     ['signals ', char(datetime('now', 'Format', 'd-MMM-y HH-mm-ss'))]);
end

mdl = "User input models/shr26df_user_input.slx";
%mdl = "User input models/shl26df_user_input.slx";

% Show robotic hand
supplyInputToUserInputMdlByMat(mdl, 'Signals/signals_after_solving_TH.mat');

%% For debugging
thtip_to_world = se3(getTransform(rbt,q0,'thtip',"world"));
fftip_to_world = se3(getTransform(rbt,q0,'fftip',"world"));
mftip_to_world = se3(getTransform(rbt,q0,'mftip',"world"));
rftip_to_world = se3(getTransform(rbt,q0,'rftip',"world"));
lftip_to_world = se3(getTransform(rbt,q0,'lftip',"world"));
palm_to_world = se3(getTransform(rbt,homeConfiguration(rbt),'palm',"world"));

disp('thtip offsets:\n')
thtip_offset = trvec(thtip_to_world) - trvec_fftip;
disp(thtip_offset)
%disp(rad2deg(rotm2eul(rotm(thtip_to_world), 'XYZ')))

% disp('rftip offsets:\n')
% rftip_palm_offset = trvec(rftip_to_world) - trvec(palm_to_world);
% rftip_base_offset = trvec(rftip_to_world) - trvec(se3(getTransform(rbt,homeConfiguration(rbt),'rfknuckle',"world")));
% rftip_offset = rftip_palm_offset;
% rftip_offset(2) = rftip_base_offset(2);
% disp(rftip_offset)

% disp('mftip offsets:\n')
% mftip_palm_offset = trvec(mftip_to_world) - trvec(palm_to_world);
% mftip_base_offset = trvec(mftip_to_world) - trvec(se3(getTransform(rbt,homeConfiguration(rbt),'mfknuckle',"world")));
% mftip_offset = mftip_palm_offset;
% mftip_offset(2) = mftip_base_offset(2);
% disp(mftip_offset)

disp('fftip offsets:\n')
fftip_palm_offset = trvec(fftip_to_world) - trvec(palm_to_world);
fftip_base_offset = trvec(fftip_to_world) - trvec(se3(getTransform(rbt,q0,'ffknuckle',"world")));
fftip_offset = fftip_palm_offset;
fftip_offset(2) = fftip_base_offset(2);
disp(fftip_offset)