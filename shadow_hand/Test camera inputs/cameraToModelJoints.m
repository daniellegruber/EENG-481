%% Define camera joint names and mapping to model joints
cameraJointNames = {'Thumb CMC', 'Thumb MCP', 'Thumb SIP'};
prefixes = {'Index', 'Middle', 'Ring', 'Pinky'};
unprefixedJoints = {'MCP', 'PIP', 'DIP'};

for prefix = prefixes
    cameraJointNames = [cameraJointNames, addPrefix(unprefixedJoints, [prefix{1}, ' '])];
end

% Thumb CMC doesn't seem to map to any of the TH joints, I might try to
% figure out if it roughly maps to a "combination" of TH joints
modelJointNames = {'ZZZ', 'THJ2', 'THJ1', 'FFJ3', 'FFJ2', 'FFJ1', ...
    'MFJ3', 'MFJ2', 'MFJ1', 'RFJ3', 'RFJ2', 'RFJ1', 'LFJ3', 'LFJ2', 'LFJ1'};

%% Load pair
pairName = 'Pair 6';
csvName = ['Test camera inputs', filesep, 'Test Poses', filesep, pairName, filesep, 'output.csv'];
cameraJointTbl = readtable(csvName);

%% Make sure that joints are in correct order
if ~all(cellfun(@isequal, cameraJointTbl.Category, cameraJointNames'))
    error('Unexpected order of joints in table')
end

%% Show model with camera inputs
cameraJointValues = deg2rad(180 - cameraJointTbl.Value);
modelJointValues = zeros(1, nJoints);

for i = 1:length(cameraJointValues)
    modelJointName = modelJointNames{i};
    modelJointValues(startsWith(jointNames,modelJointName)) = cameraJointValues(i);
end

jointValuesToInputSignals(modelJointValues', jointNames, 0.001, 2, 'camera_input');
supplyInputToUserInputMdlByMat(mdl, 'Signals/camera_input.mat');

function prefixedCharCell = addPrefix(charCell, prefix)
    prefixedCharCell = cellfun(@(x) [prefix, x], charCell, 'UniformOutput', false);
end