function [ds, qInterp] = genConfigTrajectoryFromInput(signSeq, prevConfig, jointNames)
nSigns = length(signSeq);
nJoints = length(jointNames);

% Init qWaypoints
nWaypoints = 0;
movingSigns = {'letter_j', 'letter_z'}; 
%movingSignNWaypoints = [3, 4];
movingSignTWaypoints = {[0 0.5 1], [0 0.5 1 1.5]};
for i = 1:length(signSeq)
    idx = find(ismember(movingSigns, signSeq{i}));
    if isempty(idx)
        nWaypoints = nWaypoints + 1;
    else
        %nWaypoints = nWaypoints + movingSignNWaypoints(idx);
        nWaypoints = nWaypoints + length(movingSignTWaypoints{idx});
    end
end
qWaypoints = zeros(nWaypoints + 1, nJoints);
tWaypoints = zeros(nWaypoints + 1, 1);
qWaypoints(1, :) = prevConfig; % first config is starting position

timeBetweenSigns = 1;
waypointEndIdx = 1;
for i = 1:length(signSeq)
    sign_name = signSeq{i};
    load(['Configs', filesep, sign_name, '.mat'], 'jointValues');
    jointDim = find(size(qWaypoints) == length(jointNames));
    if jointDim ~= 2
        jointValues = jointValues';
    end
    
    % If jointValues consists of a trajectory of joint values, this extracts
    % the first set of joint values
    waypointStartIdx = waypointEndIdx  + 1;
    idx = find(ismember(movingSigns, signSeq{i}));
    if isempty(idx)
        waypointEndIdx = waypointStartIdx;
        tWaypoints(waypointStartIdx:waypointEndIdx) = tWaypoints(waypointStartIdx-1) + timeBetweenSigns;
    else
        waypointEndIdx = waypointStartIdx + length(movingSignTWaypoints{idx}) - 1;
        tWaypoints(waypointStartIdx:waypointEndIdx) = tWaypoints(waypointStartIdx-1) + timeBetweenSigns + movingSignTWaypoints{idx};
    end

    qWaypoints(waypointStartIdx:waypointEndIdx,:) = jointValues; 
end


Ts = 0.001; % sample time
tFinal = tWaypoints(end);

diff1 = abs(diff(qWaypoints, 1));
diff2 = abs(diff(wrapTo2Pi(qWaypoints), 1));
wrapTo2PiIdx = round(diff2, 4) < round(diff1, 4);
qWaypoints2 = qWaypoints(2:end, :);
qWaypoints2(wrapTo2PiIdx) = wrapTo2Pi(qWaypoints2(wrapTo2PiIdx));
qWaypoints2 = [qWaypoints(1,:); qWaypoints2];

qInterp = pchip(tWaypoints,qWaypoints2',0:Ts:tFinal);

ds = jointValuesToInputSignals(qInterp, jointNames, Ts, tFinal, '');
end