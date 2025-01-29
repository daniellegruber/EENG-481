%% For two numbers
jointValues1 = load('Configs/number_1.mat', 'jointValues');
jointValues1 = jointValues1.jointValues;
jointValues2 = load('Configs/number_2.mat', 'jointValues');
jointValues2 = jointValues2.jointValues;

qWaypoints = [jointValues1'; jointValues2'];

Ts = 0.001;
tFinal = 2;
tWaypoints = [0, tFinal];
qInterp = pchip(tWaypoints,qWaypoints',0:Ts:tFinal)';


ds = jointValuesToInputSignals(qInterp', jointNames, Ts, tFinal, 'number_1_to_2');

%% For numbers 1-9
qWaypoints = zeros(length(number_pose_params), nJoints);

for i = 1:length(number_pose_params)
    pose_name = number_pose_params(i).name;
    load(['Configs', filesep, pose_name, '.mat'], 'jointValues');
    qWaypoints(i,:) = jointValues;
end


Ts = 0.001; % sample time
tFinal = 10;
tWaypoints = linspace(0, tFinal, length(number_pose_params));
qInterp = pchip(tWaypoints,qWaypoints',0:Ts:tFinal)';


ds = jointValuesToInputSignals(qInterp', jointNames, Ts, tFinal, 'numbers_1_to_9');

%%
%mdl = "User input models/shr25df_user_input.slx";
mdl = "User input models/shl25df_user_input.slx";
% open_system(mdl)
% simIn = Simulink.SimulationInput(mdl);
% load('Signals\numbers_1_to_9.mat');
% simIn = setExternalInput(simIn,ds);
% out = sim(simIn);
supplyInputToUserInputMdlByMat(mdl, 'Signals\numbers_1_to_9.mat');