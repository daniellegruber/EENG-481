function out = supplyInputToUserInputMdlByDs(mdl, ds)
open_system(mdl)
simIn = Simulink.SimulationInput(mdl);
simIn = setExternalInput(simIn,ds);
simIn = setModelParameter(simIn,StopTime=num2str(ds{1}.Time(end)));
out = sim(simIn);
end