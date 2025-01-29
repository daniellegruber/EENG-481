function [ds, lastConfig] = genConfigTrajectoryFromInput(signSeq, prevConfig, jointNames)
nSigns = length(signSeq);
nJoints = length(jointNames);
qWaypoints = zeros(nSigns + 1, nJoints);

qWaypoints(1, :) = prevConfig; % first config is starting position
for i = 1:length(signSeq)
    sign_name = signSeq{i};
    load(['Configs', filesep, sign_name, '.mat'], 'jointValues');
    qWaypoints(i+1,:) = jointValues;
end
lastConfig = jointValues;


Ts = 0.001; % sample time
timePerSign = 1.25;
tFinal = nSigns*timePerSign;
tWaypoints = 0:timePerSign:tFinal;
qInterp = pchip(tWaypoints,qWaypoints',0:Ts:tFinal)';

ds = jointValuesToInputSignals(qInterp', jointNames, Ts, tFinal, '');
end