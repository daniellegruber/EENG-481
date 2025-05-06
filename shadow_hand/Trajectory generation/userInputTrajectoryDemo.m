%% Number demo
% numberSeq = input('Please enter a sequence of numbers to sign, separated by spaces:\n', "s");
% 
% while ~strcmp(numberSeq, 'stop')
%     % Parse supplied sequence
%     signSeq = getNumberSignSeqFromChars(numberSeq);
% 
%     % Generate trajectory between signs
%     [ds, ~] = genConfigTrajectoryNoInterferenceAvoidance(signSeq, jointNames);
% 
%     % Show robotic hand
%     supplyInputToUserInputMdlByDs(mdl, ds);
%     pause(length(signSeq)+1); % wait for Mechanics Explorer to show
% 
%     % Get next number sequence
%     numberSeq = input('Please enter a sequence of numbers to sign, separated by spaces:\n', "s");
% end

%% Letter demo
load(['Trajectory generation', filesep, 'transitionTbl.mat'], "transitionTbl");

mdl = "User input models/shr26df_user_input.slx";
leftOrRight = 1;

% mdl = "User input models/shl26df_user_input.slx";
% leftOrRight = 0;

letterSeq = input('Please enter a sequence of letters to sign:\n', "s");

while ~strcmp(letterSeq, 'stop')
    % Parse supplied sequence
    signSeq = getLetterSignSeqFromChars(letterSeq);

    % Generate trajectory between signs
    [ds, qInterp] = genConfigTrajectoryFromInput(signSeq, jointNames, transitionTbl, leftOrRight);
    
    % Show robotic hand
    supplyInputToUserInputMdlByDs(mdl, ds);
    pause(length(signSeq)+1); % wait for Mechanics Explorer to show

    % Get next letter sequence
    letterSeq = input('Please enter a sequence of letters to sign:\n', "s");
end

%% Letter demo: with vs without interferance avoidance
load(['Trajectory generation', filesep, 'transitionTbl.mat'], "transitionTbl");

mdl = "User input models/shr26df_user_input.slx";
%mdl = "User input models/shl26df_user_input.slx";
leftOrRight = 1;

letterSeq = input('Please enter a sequence of letters to sign:\n', "s");

while ~strcmp(letterSeq, 'stop')
    % Parse supplied sequence
    signSeq = getLetterSignSeqFromChars(letterSeq);

    % Generate trajectory between signs without interference avoidance
    disp("Now showing trajectory between signs without interference avoidance")
    [ds1, ~] = genConfigTrajectoryNoInterferenceAvoidance(signSeq, jointNames);
    
    % Show robotic hand
    supplyInputToUserInputMdlByDs(mdl, ds1);
    pause(length(signSeq)+5); % wait for Mechanics Explorer to show

    % Generate trajectory between signs with interference avoidance
    disp("Now showing trajectory between signs with interference avoidance")
    [ds2, ~] = genConfigTrajectoryFromInput(signSeq, jointNames, transitionTbl, leftOrRight);
    
    % Show robotic hand
    supplyInputToUserInputMdlByDs(mdl, ds2);
    pause(length(signSeq)+1); % wait for Mechanics Explorer to show

    % Get next number sequence
    letterSeq = input('Please enter a sequence of letters to sign:\n', "s");
end

%% Only generate trajectories without interference avoidance

mdl = "User input models/shr26df_user_input.slx";
%mdl = "User input models/shl26df_user_input.slx";

letterSeq = input('Please enter a sequence of letters to sign:\n', "s");

while ~strcmp(letterSeq, 'stop')
    % Parse supplied sequence
    signSeq = getLetterSignSeqFromChars(letterSeq);

    % Generate trajectory between signs without interference avoidance
    [ds2, ~] = genConfigTrajectoryNoInterferenceAvoidance(signSeq, jointNames);
    
    % Show robotic hand
    supplyInputToUserInputMdlByDs(mdl, ds2);
    pause(length(signSeq)+5); % wait for Mechanics Explorer to show

    % Get next number sequence
    letterSeq = input('Please enter a sequence of letters to sign:\n', "s");
end

%% Helper functions

function signSeq = getNumberSignSeqFromChars(numberSeq)
    numberSeqParsed = strsplit(numberSeq, ' ');
    signSeq = cellfun(@(x) ['number_', x], numberSeqParsed, 'UniformOutput', false);
end

function signSeq = getLetterSignSeqFromChars(letterSeq)
    letterSeqParsed = cell(1, length(letterSeq));
    for i = 1:length(letterSeq)
        letterSeqParsed{i} = letterSeq(i);
    end
    signSeq = cellfun(@(x) ['letter_', x], letterSeqParsed, 'UniformOutput', false);
end