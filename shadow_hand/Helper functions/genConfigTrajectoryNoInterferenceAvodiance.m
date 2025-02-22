function [ds, qInterp] = genConfigTrajectoryNoInterferenceAvodiance(signSeq, jointNames)
nJoints = length(jointNames);

% Init qWaypoints, tWaypoints
movingSigns = {'letter_j', 'letter_z'}; 
movingSignTWaypoints = {[0 0.5 1], [0 0.5 1 1.5]};
nMax = 100;
qWaypoints = zeros(nMax, nJoints);
tWaypoints = zeros(nMax, 1);

% Iterate over sign sequence
timeBetweenSigns = 1;
waypointEndIdx = 0;
for i = 1:length(signSeq)
    sign_name = signSeq{i};
    load(['Configs', filesep, sign_name, '.mat'], 'jointValues');
    jointValues = correctJointValueDims(jointValues, nJoints);
    
    % Update tWaypoints
    waypointStartIdx = waypointEndIdx  + 1;
   
    movingIdx = find(ismember(movingSigns, signSeq{i}));

    % Moving sign
    if ~isempty(movingIdx) 
        tPoints = movingSignTWaypoints{movingIdx};
    else 
        tPoints = 0;
    end

    waypointEndIdx = waypointStartIdx + length(tPoints) - 1;
    if i == 1
        tWaypoints(waypointStartIdx:waypointEndIdx) = tPoints;
    else
        tWaypoints(waypointStartIdx:waypointEndIdx) = tWaypoints(waypointStartIdx-1) + timeBetweenSigns + tPoints;
    end

    % If repeated letter, slide to right or left depending on hand
    if i > 1 && strcmp(signSeq{i}, signSeq{i-1})
        slideJointIdx = ismember(jointNames, 'ARMJ2');
        jointValues(:, slideJointIdx) = 0.1;
    end

    % Update qWaypoints
    qWaypoints(waypointStartIdx:waypointEndIdx,:) = jointValues; 
end

qWaypoints = qWaypoints(1:waypointEndIdx, :);
tWaypoints = tWaypoints(1:waypointEndIdx, :);

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

    
function signSeq = addLetterPrefix(letterCell)
    signSeq = cellfun(@(x) ['letter_', x], letterCell, 'UniformOutput', false);
end

function jointValues = correctJointValueDims(jointValues, nJoints)
    jointDim = find(size(jointValues) == nJoints);
    if jointDim ~= 2
        jointValues = jointValues';
    end
end

end