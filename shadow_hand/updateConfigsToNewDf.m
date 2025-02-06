% Specify where new joint is as index of old joint array where it is to be
% inserted before, e.g.,
%   ARMJ1, WRJ2, ... -> ARMJ2, ARMJ1, WRJ2, ...
%   insertBeforeIdx = 1

newJointIdx = 1; 
oldNJoints = 25;
configs = dir('Configs/*.mat');
for i = 1:length(configs)
    load(['Configs', filesep, configs(i).name], 'jointValues');
    jointDim = find(size(jointValues) == oldNJoints);
    if isempty(jointDim)
        warning(['Config ', configs(i).name(1:end-4) ' has unexpected number of joints']);
    else
        if jointDim == 2
            jointValues = jointValues';
        end
        nWaypoints = size(jointValues, 2);
        jointValues = [jointValues(1:newJointIdx-1,:); zeros(1, nWaypoints); jointValues(newJointIdx:end,:)];
        save(['Configs', filesep, configs(i).name], "jointValues");
    end
end