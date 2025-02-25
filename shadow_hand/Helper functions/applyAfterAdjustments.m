function jointValues = applyAfterAdjustments(jointValues, jointNames, afterAdjustments)
    if ~isempty(afterAdjustments)
        for i = 1:2:length(afterAdjustments)
            jointIdx = contains(jointNames, afterAdjustments{i});
            jointValue = afterAdjustments{i+1};
            jointValues(jointIdx) = jointValue;
        end
    end
end