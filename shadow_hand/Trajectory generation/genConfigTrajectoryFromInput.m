function [ds, qInterp] = genConfigTrajectoryFromInput(signSeq, jointNames, ...
    transitionTbl, leftOrRight)

nJoints = length(jointNames);

% Setup info about letter names (corresponding to indices of transitionTbl)
letterNames = cell(1,27);
letterStr = 'a':'z';
for i = 1:26
    letterNames{i} = letterStr(i);
end
letterNames{end} = 'double_z';
letterNames = addLetterPrefix(letterNames);

% Setup info about moving signs
movingSigns = {'letter_j', 'letter_z', 'letter_double_z'}; 
movingSignTWaypoints = {[0 0.5 1], [0 0.5 1 1.5], [0 0.5 1 1.5]};

% Init qWaypoints, tWaypoints
nMax = 100; % create buffer with nMax rows, then eliminate unneccessary rows after
qWaypoints = zeros(nMax, nJoints);
tWaypoints = zeros(nMax, 1);

% Iterate over sign sequence
timeBetweenSigns = 1.5;
timeHoldSign = 0.5;

waypointEndIdx = 0;
tEndOfPrevSign = 0;
for i = 1:length(signSeq)

    firstZOfDoubleZFlag = 0; % becomes 1 for first z of repeated z, e.g., the first z of "pizza"
    secondZOfDoubleZFlag = 0; % becomes 1 for second z of repeated z, e.g., the second z of "pizza"
    if i < length(signSeq) && strcmp(signSeq{i}, 'letter_z') && strcmp(signSeq{i+1}, 'letter_z')
        firstZOfDoubleZFlag = 1;
    end
    if i > 1 && strcmp(signSeq{i}, 'letter_z') && strcmp(signSeq{i-1}, 'letter_z')
        secondZOfDoubleZFlag = 1;
    end

    if ~secondZOfDoubleZFlag
        if firstZOfDoubleZFlag
            sign_name = 'letter_double_z';
        else
            sign_name = signSeq{i};
        end
        load(['Configs', filesep, sign_name, '.mat'], 'jointValues');
        jointValues = correctJointValueDims(jointValues, nJoints);
        
        % Start idx of where to insert waypoints associated with current
        % sign (and potentially intermediate waypoints associated with the
        % current sign -> next sign transition)
        waypointStartIdx = waypointEndIdx  + 1;
       
        movingIdx = find(ismember(movingSigns, sign_name)); %  determine if this is a sign with movement, e.g., letter_j
    
        transitionFlag = 0; % becomes 1 if intermediate waypoint(s) need to be inserted between two signs
        transitionNames = {}; % name of intermediate waypoint(s) to load
        transitionProps = 0; % proportion of way between two signs to insert waypoint
        nTransitionPoints = 0;
        if i < length(signSeq)
            if i < (length(signSeq)-1) && firstZOfDoubleZFlag
                transitionTblIdx = [find(ismember(letterNames, sign_name)), find(ismember(letterNames, signSeq{i+2}))];
            else
                transitionTblIdx = [find(ismember(letterNames, sign_name)), find(ismember(letterNames, signSeq{i+1}))];
            end
            if ~isempty(transitionTbl{transitionTblIdx(1), transitionTblIdx(2)})
                transitionFlag = 1;
                transitionNames = transitionTbl{transitionTblIdx(1), transitionTblIdx(2)}{1};
                transitionProps = transitionTbl{transitionTblIdx(1), transitionTblIdx(2)}{2};
                nTransitionPoints = length(transitionProps);
            end
        end
    
        % Determine times of when to "hit" waypoints (these will be
        % inserted into the tWaypoints array)
        if ~isempty(movingIdx) % moving sign
            tPoints = movingSignTWaypoints{movingIdx};
            tPoints = [tPoints, tPoints(end) + timeHoldSign];
            tEndOfCurrSign = tEndOfPrevSign + timeBetweenSigns + tPoints(end);
            if transitionFlag
                tPoints = [tPoints, tPoints(end) + transitionProps * timeBetweenSigns];
            end
            jointValues = [jointValues; jointValues(end, :)];
        elseif transitionFlag % intermediate waypoint(s) need to be inserted, as indicated by transitionTbl
            tEndOfCurrSign = tEndOfPrevSign + timeBetweenSigns + timeHoldSign;
            tPoints = [0, timeHoldSign, timeHoldSign + transitionProps * timeBetweenSigns];
            jointValues = [jointValues; jointValues];
        else 
            tEndOfCurrSign = tEndOfPrevSign + timeBetweenSigns + timeHoldSign;
            tPoints = [0, timeHoldSign];
            jointValues = [jointValues; jointValues];
        end
    
        % End idx of where to insert waypoints associated with current
        % sign (and potentially intermediate waypoints associated with the
        % current sign -> next sign transition)
        waypointEndIdx = waypointStartIdx + length(tPoints) - 1;

        % Update tWaypoints
        if i == 1
            tWaypoints(waypointStartIdx:waypointEndIdx) = tPoints;
        else
            tWaypoints(waypointStartIdx:waypointEndIdx) = tEndOfPrevSign + tPoints;
        end
    
        % If transitionTbl indiates that intermediate waypoint(s) need to be
        % inserted, load these waypoints and insert into qWaypoints
        if transitionFlag
            for j = 1:nTransitionPoints
            jointValues2 = jointValues;
            load(['Configs', filesep, transitionNames{j}, '.mat'], 'jointValues');
            jointValues3 = correctJointValueDims(jointValues, nJoints);
            jointValues = [jointValues2; jointValues3];
            end
        end

        % If repeated letter, slide to right or left depending on hand
        if i > 1 && ~firstZOfDoubleZFlag && strcmp(signSeq{i}, signSeq{i-1})
            slideJointIdx = ismember(jointNames, 'ARMJ2');
            jointValues(:, slideJointIdx) = -0.1;
        end
    
        % Update qWaypoints
        qWaypoints(waypointStartIdx:waypointEndIdx,:) = jointValues; 
        tEndOfPrevSign = tEndOfCurrSign;
    end
end

% Remove unneccessary buffer rows
qWaypoints = qWaypoints(1:waypointEndIdx, :);
tWaypoints = tWaypoints(1:waypointEndIdx, :);

% Modify joint values if left hand
if ~leftOrRight %left hand
    leftNegateIdx = contains(jointNames, {'ARMJ1','ARMJ2'});
    qWaypoints(:, leftNegateIdx) = -qWaypoints(:, leftNegateIdx);
end

Ts = 0.001; % sample time
tFinal = tWaypoints(end);

%  When interpolating between two joint angles, use the "shortest-distance angular path"
diff1 = abs(diff(qWaypoints, 1));
diff2 = abs(diff(wrapTo2Pi(qWaypoints), 1));
wrapTo2PiIdx = round(diff2, 4) < round(diff1, 4);
qWaypoints2 = qWaypoints(2:end, :);
qWaypoints2(wrapTo2PiIdx) = wrapTo2Pi(qWaypoints2(wrapTo2PiIdx));
qWaypoints2 = [qWaypoints(1,:); qWaypoints2];

% Interpolate
qInterp = pchip(tWaypoints,qWaypoints2',0:Ts:tFinal);

% Generate simulink ds from qInterp
ds = jointValuesToInputSignals(qInterp, jointNames, Ts, tFinal, '');

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

end