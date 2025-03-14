%% Solve inverse kinematics for each finger tip, one by one

% Reminder: se3(trvec(tip_to_world), 'trvec') * se3(rotm(tip_to_world)) == tip_to_world

mdl = "User input models/shr26df_user_input.slx";
%mdl = "User input models/shl26df_user_input.slx";
rbt = shr26df_rbt;

pose_name = 'letter_double_z';
q0 = homeConfiguration(rbt);
valuesPrev = zeros(1,nJoints);

xoffset_from_palm = zeros(1,4);
yoffset_from_knuckle = [0.012, 0.01 0, -0.01];
zoffset_from_palm = [0.04, 0.04, 0.04, 0.04];
%zoffset_from_palm = [0.04, 0.04, 0.04, 0.04]+0.01; % try this out later

afterAdjustments = {};

for fingerIdx = 1:5
%% Set up target pose of tip
tip_frame = [lower(fingerNames{fingerIdx}),'tip'];
if fingerIdx == 5 % Thumb
    % Get transforms of certain frames relative to world in home config
    rfmiddle_to_world = se3(getTransform(rbt,q0,"rfmiddle","world"));

    % Create target translation
    trvec_rfmiddle = trvec(rfmiddle_to_world); 

    trvec_target = trvec_rfmiddle;
    trvec_target(1) = trvec_target(1) + 0.01;
elseif fingerIdx == 3 || fingerIdx == 4 
    % Get transforms of certain frames relative to world in home config
    tiphome_to_world = se3(getTransform(rbt,homeConfiguration(rbt),tip_frame,"world"));
    
    % Create target translation
    trvec_tiphome = trvec(tiphome_to_world); 
    
    trvec_target = trvec_tiphome;
    trvec_target(1) = trvec_target(1) + 0.05;
    %trvec_target(2) = trvec_knuckle(2)+ yoffset_from_knuckle(fingerIdx);
    trvec_target(3) = trvec_target(3) - 0.06;
else 
    knuckle_frame = [lower(fingerNames{fingerIdx}),'knuckle'];
    
    % Get transforms of certain frames relative to world in home config
    knuckle_to_world = se3(getTransform(rbt,q0,knuckle_frame,"world"));
    palm_to_world = se3(getTransform(rbt,q0,"palm","world"));
    
    % Create target translation
    trvec_palm = trvec(palm_to_world);  
    trvec_knuckle = trvec(knuckle_to_world); 
    
    trvec_target = trvec_palm;
    trvec_target(1) = trvec_palm(1) + xoffset_from_palm(fingerIdx);
    trvec_target(2) = trvec_knuckle(2)+ yoffset_from_knuckle(fingerIdx);
    trvec_target(3) = trvec_palm(3) + zoffset_from_palm(fingerIdx);
end

% Create distance constraint
distanceConstraint = constraintPositionTarget(tip_frame);
distanceConstraint.ReferenceBody = 'world';
distanceConstraint.TargetPosition = trvec_target;
distanceConstraint.PositionTolerance = 0;%1e-3;

% Run solver
jointValues = runGikSolver(rbt, fingerIdx, ...
        valuesPrev, distanceConstraint, []);

valuesPrev = jointValues;

if fingerIdx == 5
    jointValues = applyAfterAdjustments(jointValues, jointNames, afterAdjustments);
    jointValuesToInputSignals(jointValues, jointNames, 0.001, 2, [pose_name, '_ref']);
    save(['Configs', filesep, pose_name, '_ref.mat'], "jointValues");
end

q0 = jointValuesToConfigObj(jointValues, jointNames); % Initial config for next iteration

%% Create signals to provide to right_test_asl_poses.slx
jointValuesToInputSignals(jointValues, jointNames, 0.001, 2, ...
     ['signals_after_solving_', fingerNames{fingerIdx}]);

end

% Show robotic hand
supplyInputToUserInputMdlByMat(mdl, 'Signals/signals_after_solving_TH.mat');

%% Now get sequence of 4 stages
afterAdjustments = {...
    {'ARMJ2', 0, 'WRJ1', deg2rad(10)}, ...
    {'ARMJ2', -0.15, 'WRJ1', deg2rad(10)}, ...
    {'ARMJ2', 0, 'WRJ1', deg2rad(80)}, ...
    {'ARMJ2', -0.15, 'WRJ1', deg2rad(80)} ...
    };

% Start position is letter_z_ref
load('Configs\letter_double_z_ref.mat');
startValues = jointValues;
q0 = jointValuesToConfigObj(startValues, jointNames);
trvec_fftip_q0 = trvec(se3(getTransform(rbt,q0,"fftip","world")));

for stage = 1:4
    jointValues = startValues;
    for i = 1:2:length(afterAdjustments{stage})
        jointIdx = contains(jointNames, afterAdjustments{stage}{i});
        jointValue = afterAdjustments{stage}{i+1};
        jointValues(jointIdx) = jointValue;
    end

    % Save
    jointValuesToInputSignals(jointValues, jointNames, 0.001, 2, [pose_name, '_stage_', num2str(stage)]);
    save(['Configs', filesep, pose_name, '_stage_', num2str(stage), '.mat'], "jointValues");

    % Show robotic hand
    % supplyInputToUserInputMdlByMat(mdl, ['Signals/', pose_name, '_stage_', num2str(stage), '.mat']);
    % 
    % qCurr = jointValuesToConfigObj(jointValues, jointNames); % Initial config for next iteration
    % disp([pose_name, '_stage_', num2str(stage)]);
    % disp(trvec(se3(getTransform(rbt,qCurr,"fftip","world")))-trvec_fftip_q0);
end

%% Generate trajectory between signs
signSeq = {'letter_double_z_stage_1', 'letter_double_z_stage_2', 'letter_double_z_stage_3', 'letter_double_z_stage_4'};
[ds, lastConfig] = genConfigTrajectoryNoInterferenceAvoidance(signSeq, jointNames);

% Show robotic hand
supplyInputToUserInputMdlByDs(mdl, ds);

qWaypoints = zeros(length(signSeq), nJoints);
for i = 1:length(signSeq)
    load(['Configs', filesep, signSeq{i}, '.mat'], 'jointValues');
    qWaypoints(i, :) = jointValues;
end
jointValues = qWaypoints;
save(['Configs', filesep, pose_name, '.mat'], "jointValues");