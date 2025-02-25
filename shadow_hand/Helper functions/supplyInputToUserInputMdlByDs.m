function out = supplyInputToUserInputMdlByDs(mdl, ds)
simIn = Simulink.SimulationInput(mdl);
simIn = setExternalInput(simIn,ds);
simIn = setModelParameter(simIn,StopTime=num2str(ds{1}.Time(end)));
out = sim(simIn);
end