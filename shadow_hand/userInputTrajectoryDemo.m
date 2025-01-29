mdl = "User input models/shr25df_user_input.slx";
%mdl = "User input models/shl25df_user_input.slx";

numberSeq = input('Please enter a sequence of numbers to sign:\n', "s");
prevConfig = zeros(1, nJoints); % start in home config
while ~strcmp(numberSeq, 'stop')
    % Parse supplied sequence
    numberSeqParsed = strsplit(numberSeq, ' ');
    signSeq = cellfun(@(x) ['number_', x], numberSeqParsed, 'UniformOutput', false);
    
    % Generate trajectory between signs
    [ds, lastConfig] = genConfigTrajectoryFromInput(signSeq, prevConfig, jointNames);
    
    % Show robotic hand
    supplyInputToUserInputMdlByDs(mdl, ds);
    pause(length(signSeq)+1); % wait for Mechanics Explorer to show

    % Set starting config of next trajectory to last config of this
    % trajectory
    prevConfig = lastConfig;

    % Get next number sequence
    numberSeq = input('Please enter a sequence of numbers to sign:\n', "s");
end