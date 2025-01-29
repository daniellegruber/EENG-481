function out = supplyInputToUserInputMdlByMat(mdl, matFile)
open_system(mdl)
simIn = Simulink.SimulationInput(mdl);
load(matFile, 'ds');
simIn = setExternalInput(simIn,ds);
out = sim(simIn);
end