function jointValuesToConstInputSignals(jointValues, jointNames, sampleTime, maxTime, fileName)

% Create dataset and save
time_samples = 0:sampleTime:maxTime;
ds = Simulink.SimulationData.Dataset;
for i=1:length(jointValues)
    trajectory = timeseries(jointValues(i) * ones(1, length(time_samples)), time_samples);
    ds = addElement(ds, trajectory, jointNames{i});
end
save(['Signals', filesep, fileName, '.mat'], 'ds');

end