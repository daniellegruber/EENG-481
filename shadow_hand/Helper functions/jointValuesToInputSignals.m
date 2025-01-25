function jointValuesToInputSignals(jointValues, jointNames, sampleTime, maxTime, fileName)
% jointValues is n x m array, n corresponds to number of joints and m
% corresponds to number of time samples

% Create dataset and save
time_samples = 0:sampleTime:maxTime;
ds = Simulink.SimulationData.Dataset;
for i=1:size(jointValues, 1)
    trajectory = timeseries(jointValues(i, :), time_samples);
    ds = addElement(ds, trajectory, jointNames{i});
end
save(['Signals', filesep, fileName, '.mat'], 'ds');

end