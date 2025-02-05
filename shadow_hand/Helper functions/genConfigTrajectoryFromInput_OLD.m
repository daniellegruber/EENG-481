function [ds, lastConfig] = genConfigTrajectoryFromInput_OLD(signSeq, prevConfig, jointNames)
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

diff1 = abs(diff(qWaypoints, 1));
diff2 = abs(diff(wrapTo2Pi(qWaypoints), 1));
wrapTo2PiIdx = round(diff2, 4) < round(diff1, 4);
qWaypoints2 = qWaypoints(2:end, :);
qWaypoints2(wrapTo2PiIdx) = wrapTo2Pi(qWaypoints2(wrapTo2PiIdx));
qWaypoints2 = [qWaypoints(1,:); qWaypoints2];

qInterp = pchip(tWaypoints,qWaypoints2',0:Ts:tFinal);

ds = jointValuesToInputSignals(qInterp, jointNames, Ts, tFinal, '');
end