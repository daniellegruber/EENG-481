mdl = "User input models/shr25df_user_input.slx";
%mdl = "User input models/shl25df_user_input.slx";

%% Number demo
numberSeq = input('Please enter a sequence of numbers to sign:\n', "s");
prevConfig = zeros(1, nJoints); % start in home config
while ~strcmp(numberSeq, 'stop')
    % Parse supplied sequence
    numberSeqParsed = strsplit(numberSeq, ' ');
    signSeq = cellfun(@(x) ['number_', x], numberSeqParsed, 'UniformOutput', false);
    
    % Generate trajectory between signs
    [ds, qInterp] = genConfigTrajectoryFromInput(signSeq, prevConfig, jointNames);
    
    % Show robotic hand
    supplyInputToUserInputMdlByDs(mdl, ds);
    pause(length(signSeq)+1); % wait for Mechanics Explorer to show

    % Set starting config of next trajectory to last config of this
    % trajectory
    prevConfig = qInterp(:, end);

    % Get next number sequence
    numberSeq = input('Please enter a sequence of numbers to sign:\n', "s");
end

%% Letter demo

mdl = "User input models/shr26df_user_input.slx";
%mdl = "User input models/shl26df_user_input.slx";

letterSeq = input('Please enter a sequence of letters to sign:\n', "s");
prevConfig = 'home';
while ~strcmp(letterSeq, 'stop')
    % Parse supplied sequence
    letterSeqParsed = strsplit(letterSeq, ' ');
    signSeq = cellfun(@(x) ['letter_', x], letterSeqParsed, 'UniformOutput', false);
    signSeq = [{prevConfig}, signSeq];
    
    % Generate trajectory between signs
    [ds, qInterp] = genConfigTrajectoryFromInput(signSeq, jointNames);
    
    % Show robotic hand
    supplyInputToUserInputMdlByDs(mdl, ds);
    pause(length(signSeq)+1); % wait for Mechanics Explorer to show

    % Set starting config of next trajectory to last config of this
    % trajectory
    prevConfig = signSeq{end};

    % Get next number sequence
    letterSeq = input('Please enter a sequence of letters to sign:\n', "s");
end

% NOTE: need to fix home -> letter trajectories, add special case for b
%% Letter demo: with vs without interferance avoidance

mdl = "User input models/shr26df_user_input.slx";
%mdl = "User input models/shl26df_user_input.slx";

letterSeq = input('Please enter a sequence of letters to sign:\n', "s");
prevConfig = 'home';
while ~strcmp(letterSeq, 'stop')
    % Parse supplied sequence
    letterSeqParsed = strsplit(letterSeq, ' ');
    signSeq = cellfun(@(x) ['letter_', x], letterSeqParsed, 'UniformOutput', false);
    signSeq = [{prevConfig}, signSeq];
    
    % Generate trajectory between signs with interference avoidance
    [ds1, ~] = genConfigTrajectoryFromInput(signSeq, jointNames);
    
    % Show robotic hand
    supplyInputToUserInputMdlByDs(mdl, ds1);
    pause(length(signSeq)+1); % wait for Mechanics Explorer to show

    % Generate trajectory between signs without interference avoidance
    [ds2, ~] = genConfigTrajectoryNoInterferenceAvodiance(signSeq, jointNames);
    
    % Show robotic hand
    supplyInputToUserInputMdlByDs(mdl, ds2);
    pause(length(signSeq)+1); % wait for Mechanics Explorer to show

    % Set starting config of next trajectory to last config of this
    % trajectory
    % prevConfig = signSeq{end};

    % Get next number sequence
    letterSeq = input('Please enter a sequence of letters to sign:\n', "s");
end

% NOTE: need to fix home -> letter trajectories, add special case for b