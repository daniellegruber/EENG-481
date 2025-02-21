% rows are "from", cols are "to"
transitionTbl = cell(26); 
letterNames = cell(1,26);
letterStr = 'a':'z';
for i = 1:26
    letterNames{i} = letterStr(i);
end
%% Fill in transitions that need intermediate waypoint

% Each entry has the name of the intermediate waypoint(s) to insert as well as the
% proportion of the transition time between the two signs when they should
% be inserted
% E.g., when going from a->b, insert the waypoint associated with
% 'transition_a_to_b.mat' at 0.5 of the total transition time between a and b

transitionTbl{createIdxFromLetterNames('a', 'b')} = {{'transition_a_to_b'}, 0.5};
transitionTbl{createIdxFromLetterNames('a', 'c')} = {{'transition_a_to_c'}, 0.4};
transitionTbl{createIdxFromLetterNames('a', 'd')} = {{'transition_a_to_d'}, 0.3};
transitionTbl{createIdxFromLetterNames('a', 'e')} = {{'transition_a_to_e'}, 0.8};
transitionTbl{createIdxFromLetterNames('a', 'h')} = {{'transition_a_to_h'}, 0.8};
transitionTbl{createIdxFromLetterNames('a', 'k')} = {{'transition_a_to_k'}, 0.3};
transitionTbl{createIdxFromLetterNames('a', 'm')} = {{'transition_a_to_m'}, 0.3};
transitionTbl{createIdxFromLetterNames('a', 'n')} = {{'transition_a_to_n_1','transition_a_to_n_2'}, [0.4, 0.8]};

%transitionTbl{createIdxFromLetterNames('m', 'n')} = 'letter_m_to_under';

%% Show trajectory
%signSeq = {'letter_m', 'letter_n', 'letter_m', 'letter_a', 'letter_b', 'letter_a', 'letter_c', 'letter_j', 'letter_z'};
% signSeq = addLetterPrefix({'a', 'b', 'a', 'c', 'a', 'd', 'a', 'e', 'a', 'h', 'a', 'm'});
signSeq = addLetterPrefix({'a', 'h', 'a', 'm', 'a', 'n'});
[ds, ~] = genConfigTrajectoryFromInput(signSeq, jointNames, transitionTbl);
    
% Show robotic hand
supplyInputToUserInputMdlByDs(mdl, ds);

%% Helper functions
function idx = createIdxFromLetterNames(l1, l2)
letterNames = cell(1,26);
letterStr = 'a':'z';
for i = 1:26
    letterNames{i} = letterStr(i);
end
row = find(ismember(letterNames, l1));
col = find(ismember(letterNames, l2));
sz = [26, 26];
idx = sub2ind(sz,row,col);
end

function signSeq = addLetterPrefix(letterCell)
    signSeq = cellfun(@(x) ['letter_', x], letterCell, 'UniformOutput', false);
end