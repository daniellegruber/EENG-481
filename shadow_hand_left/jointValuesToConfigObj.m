function config = jointValuesToConfigObj(jointValues, jointNames)
    config = repmat(struct('JointName','', 'JointPosition', 0), 1, length(jointValues));
    for i = 1:length(jointValues)
        config(i) = struct('JointName', jointNames{i}, 'JointPosition', jointValues(i));
    end
end