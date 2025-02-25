function out = supplyInputToUserInputMdlByMat(mdl, matFile)
simIn = Simulink.SimulationInput(mdl);
load(matFile, 'ds');
simIn = setExternalInput(simIn,ds);
out = sim(simIn);
end