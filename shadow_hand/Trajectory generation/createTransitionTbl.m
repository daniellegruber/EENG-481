% rows are "from", cols are "to"
transitionTbl = cell(27); 
letterNames = cell(1,27);
letterStr = 'a':'z';
for i = 1:26
    letterNames{i} = letterStr(i);
end
letterNames{end} = 'double_z';
%% Fill in transitions that need intermediate waypoint

% TO DO: 
% * make letters c and o more distinct
% * add transitions for letter z -> others

% Each entry has the name of the intermediate waypoint(s) to insert as well as the
% proportion of the transition time between the two signs when they should
% be inserted
% E.g., when going from a->b, insert the waypoint associated with
% 'transition_a_to_b.mat' at 0.5 of the total transition time between a and b

% letter a <-> something
transitionTbl{createIdxFromLetterNames('a', 'b')} = {{'transition_a_to_b'}, 0.5};
transitionTbl{createIdxFromLetterNames('a', 'c')} = {{'transition_a_to_c'}, 0.4};
transitionTbl{createIdxFromLetterNames('a', 'd')} = {{'transition_a_to_d'}, 0.3};
transitionTbl{createIdxFromLetterNames('a', 'e')} = {{'transition_a_to_e'}, 0.8};
transitionTbl{createIdxFromLetterNames('a', 'h')} = {{'transition_a_to_h'}, 0.8};
transitionTbl{createIdxFromLetterNames('a', 'i')} = {{'transition_a_to_i'}, 0.5};
transitionTbl{createIdxFromLetterNames('a', 'j')} = {{'transition_a_to_i'}, 0.5};
transitionTbl{createIdxFromLetterNames('a', 'k')} = {{'transition_a_to_k'}, 0.3};
transitionTbl{createIdxFromLetterNames('a', 'm')} = {{'transition_a_to_m'}, 0.3};
transitionTbl{createIdxFromLetterNames('a', 'n')} = {{'transition_a_to_n'}, 0.5};
transitionTbl{createIdxFromLetterNames('a', 'p')} = {{'transition_a_to_p'}, 0.5};
transitionTbl{createIdxFromLetterNames('a', 'r')} = {{'transition_a_to_r'}, 0.3};
transitionTbl{createIdxFromLetterNames('a', 's')} = {{'transition_a_to_s'}, 0.3};
transitionTbl{createIdxFromLetterNames('a', 't')} = {{'transition_a_to_t'}, 0.5};
transitionTbl{createIdxFromLetterNames('a', 'u')} = {{'transition_a_to_u'}, 0.3};
transitionTbl{createIdxFromLetterNames('a', 'v')} = {{'transition_a_to_u'}, 0.3};
transitionTbl{createIdxFromLetterNames('a', 'w')} = {{'transition_a_to_w'}, 0.3};
transitionTbl{createIdxFromLetterNames('a', 'x')} = {{'transition_a_to_x'}, 0.5};
transitionTbl{createIdxFromLetterNames('a', 'z')} = {{'transition_a_to_z'}, 0.3};

% letter b <-> something
transitionTbl{createIdxFromLetterNames('b', 'e')} = {{'transition_b_to_e'}, 0.5};
transitionTbl{createIdxFromLetterNames('b', 'i')} = {{'transition_b_to_i'}, 0.5};
transitionTbl{createIdxFromLetterNames('b', 'r')} = {{'transition_b_to_r'}, 0.8};
transitionTbl{createIdxFromLetterNames('b', 's')} = {{'transition_b_to_s'}, 0.5};
transitionTbl{createIdxFromLetterNames('b', 'u')} = {{'transition_b_to_u'}, 0.8};

% letter c <-> something
transitionTbl{createIdxFromLetterNames('c', 'e')} = {{'transition_c_to_e'}, 0.5};
transitionTbl{createIdxFromLetterNames('c', 'n')} = {{'transition_c_to_n'}, 0.7};

% letter d <-> something
transitionTbl{createIdxFromLetterNames('d', 'e')} = {{'transition_d_to_e'}, 0.5};
transitionTbl{createIdxFromLetterNames('d', 'i')} = {{'transition_d_to_i'}, 0.5};
transitionTbl{createIdxFromLetterNames('d', 'n')} = {{'transition_d_to_n'}, 0.5};
transitionTbl{createIdxFromLetterNames('d', 's')} = {{'transition_d_to_s'}, 0.7};

% letter e <-> something
transitionTbl{createIdxFromLetterNames('e', 'f')} = {{'transition_e_to_f'}, 0.5};
transitionTbl{createIdxFromLetterNames('e', 'g')} = {{'transition_e_to_g'}, 0.5};
transitionTbl{createIdxFromLetterNames('e', 'h')} = {{'transition_e_to_h'}, 0.3}; % still slight interference but okay for now
transitionTbl{createIdxFromLetterNames('e', 'i')} = {{'transition_e_to_i'}, 0.5};
transitionTbl{createIdxFromLetterNames('e', 'j')} = {{'transition_e_to_i'}, 0.5};
transitionTbl{createIdxFromLetterNames('e', 'k')} = {{'transition_e_to_k'}, 0.3};
transitionTbl{createIdxFromLetterNames('e', 'l')} = {{'transition_e_to_l'}, 0.3};
transitionTbl{createIdxFromLetterNames('e', 'm')} = {{'transition_e_to_m'}, 0.5};
transitionTbl{createIdxFromLetterNames('e', 'n')} = {{'transition_e_to_n'}, 0.5};
transitionTbl{createIdxFromLetterNames('e', 'o')} = {{'transition_e_to_o'}, 0.5};
transitionTbl{createIdxFromLetterNames('e', 'q')} = {{'transition_e_to_q'}, 0.5};
transitionTbl{createIdxFromLetterNames('e', 'r')} = {{'transition_e_to_r'}, 0.3};
transitionTbl{createIdxFromLetterNames('e', 's')} = {{'transition_e_to_s'}, 0.5};
transitionTbl{createIdxFromLetterNames('e', 't')} = {{'transition_e_to_t'}, 0.5};
transitionTbl{createIdxFromLetterNames('e', 'u')} = {{'transition_e_to_u'}, 0.3};
transitionTbl{createIdxFromLetterNames('e', 'v')} = {{'transition_e_to_u'}, 0.3};
transitionTbl{createIdxFromLetterNames('e', 'w')} = {{'transition_e_to_w'}, 0.3};
transitionTbl{createIdxFromLetterNames('e', 'x')} = {{'transition_e_to_x'}, 0.5};
transitionTbl{createIdxFromLetterNames('e', 'y')} = {{'transition_e_to_y'}, 0.5};
transitionTbl{createIdxFromLetterNames('e', 'z')} = {{'transition_e_to_z'}, 0.5};

% letter g <-> something
transitionTbl{createIdxFromLetterNames('g', 'a')} = {{'transition_g_to_a'}, 0.5};
transitionTbl{createIdxFromLetterNames('g', 'h')} = {{'transition_g_to_h'}, 0.5};
transitionTbl{createIdxFromLetterNames('g', 'i')} = {{'transition_g_to_i'}, 0.5};
transitionTbl{createIdxFromLetterNames('g', 'n')} = {{'transition_g_to_n'}, 0.5}; % not great but works for now
transitionTbl{createIdxFromLetterNames('g', 's')} = {{'transition_g_to_s'}, 0.5};

% letter h <-> something
transitionTbl{createIdxFromLetterNames('h', 'i')} = {{'transition_h_to_i'}, 0.5};
transitionTbl{createIdxFromLetterNames('h', 's')} = {{'transition_h_to_s'}, 0.5};
transitionTbl{createIdxFromLetterNames('h', 't')} = {{'transition_h_to_t'}, 0.5};
transitionTbl{createIdxFromLetterNames('h', 'w')} = {{'transition_h_to_w'}, 0.5};

% letter i <-> something
transitionTbl{createIdxFromLetterNames('i', 'k')} = {{'transition_i_to_k'}, 0.5};
transitionTbl{createIdxFromLetterNames('i', 'm')} = {{'transition_i_to_m'}, 0.5};
transitionTbl{createIdxFromLetterNames('i', 'n')} = {{'transition_i_to_n'}, 0.5}; % not great but works for now
transitionTbl{createIdxFromLetterNames('i', 'p')} = {{'transition_i_to_p'}, 0.5};
transitionTbl{createIdxFromLetterNames('i', 'r')} = {{'transition_i_to_r'}, 0.5};
transitionTbl{createIdxFromLetterNames('i', 't')} = {{'transition_i_to_t'}, 0.5};
transitionTbl{createIdxFromLetterNames('i', 'u')} = {{'transition_i_to_u'}, 0.5};
transitionTbl{createIdxFromLetterNames('i', 'v')} = {{'transition_i_to_u'}, 0.5};
transitionTbl{createIdxFromLetterNames('i', 'w')} = {{'transition_i_to_w'}, 0.5};
transitionTbl{createIdxFromLetterNames('i', 'z')} = {{'transition_i_to_z'}, 0.5};

% letter k <-> something
transitionTbl{createIdxFromLetterNames('k', 'n')} = {{'transition_k_to_n'}, 0.5};
transitionTbl{createIdxFromLetterNames('k', 'r')} = {{'transition_k_to_r'}, 0.5};

% letter m <-> something
transitionTbl{createIdxFromLetterNames('m', 's')} = {{'transition_m_to_s'}, 0.5};
transitionTbl{createIdxFromLetterNames('m', 'y')} = {{'transition_m_to_y'}, 0.5};

% letter n <-> something
transitionTbl{createIdxFromLetterNames('n', 'o')} = {{'transition_n_to_o'}, 0.5};
transitionTbl{createIdxFromLetterNames('n', 'p')} = {{'transition_n_to_p'}, 0.5}; % not great but works for now
transitionTbl{createIdxFromLetterNames('n', 'r')} = {{'transition_n_to_r'}, 0.5};
transitionTbl{createIdxFromLetterNames('n', 's')} = {{'transition_n_to_s_1','transition_n_to_s_2'}, [0.3, 0.5]};
transitionTbl{createIdxFromLetterNames('n', 't')} = {{'transition_n_to_t'}, 0.5};
transitionTbl{createIdxFromLetterNames('n', 'u')} = {{'transition_n_to_u'}, 0.5};
transitionTbl{createIdxFromLetterNames('n', 'v')} = {{'transition_n_to_u'}, 0.5};
transitionTbl{createIdxFromLetterNames('n', 'x')} = {{'transition_n_to_x_1','transition_n_to_x_2'}, [0.3, 0.5]};
transitionTbl{createIdxFromLetterNames('n', 'y')} = {{'transition_n_to_y'}, 0.5};
transitionTbl{createIdxFromLetterNames('n', 'z')} = {{'transition_n_to_z_1','transition_n_to_z_2'}, [0.3, 0.5]};

% letter o <-> something
transitionTbl{createIdxFromLetterNames('o', 't')} = {{'transition_o_to_t'}, 0.5};

% letter p <-> something
transitionTbl{createIdxFromLetterNames('p', 's')} = {{'transition_p_to_s'}, 0.5}; % not great but works for now
transitionTbl{createIdxFromLetterNames('p', 'y')} = {{'transition_p_to_y'}, 0.2}; 

% letter r <-> something
transitionTbl{createIdxFromLetterNames('r', 's')} = {{'transition_r_to_s'}, 0.5}; 
transitionTbl{createIdxFromLetterNames('r', 'x')} = {{'transition_r_to_x'}, 0.5}; 
transitionTbl{createIdxFromLetterNames('r', 'z')} = {{'transition_r_to_z'}, 0.5}; 

% letter s <-> something
transitionTbl{createIdxFromLetterNames('s', 't')} = {{'transition_s_to_t'}, 0.5}; 
transitionTbl{createIdxFromLetterNames('s', 'u')} = {{'transition_s_to_u'}, 0.5}; 
transitionTbl{createIdxFromLetterNames('s', 'v')} = {{'transition_s_to_u'}, 0.5}; 
transitionTbl{createIdxFromLetterNames('s', 'w')} = {{'transition_s_to_w'}, 0.5}; 

% letter u <-> something
transitionTbl{createIdxFromLetterNames('u', 'w')} = {{'transition_u_to_w'}, 0.5}; 
transitionTbl{createIdxFromLetterNames('u', 'x')} = {{'transition_u_to_x'}, 0.5}; 
transitionTbl{createIdxFromLetterNames('u', 'z')} = {{'transition_u_to_z'}, 0.5}; 

% letter y <-> something
transitionTbl{createIdxFromLetterNames('y', 't')} = {{'transition_y_to_t'}, 0.5}; 

% letter z <-> something
transitionTbl{createIdxFromLetterNames('z', 'a')} = {{'transition_z_to_a'}, 0.5}; 
transitionTbl{createIdxFromLetterNames('z', 'e')} = {{'transition_z_to_e'}, 0.5}; 
transitionTbl{createIdxFromLetterNames('z', 'i')} = {{'transition_z_to_i'}, 0.5}; 
transitionTbl{createIdxFromLetterNames('z', 'u')} = {{'transition_z_to_u'}, 0.5}; 

% letter double z <-> something
transitionTbl{createIdxFromLetterNames('double_z', 'a')} = {{'transition_double_z_to_a'}, 0.5}; 
transitionTbl{createIdxFromLetterNames('double_z', 'e')} = {{'transition_double_z_to_e'}, 0.5}; 
transitionTbl{createIdxFromLetterNames('double_z', 'i')} = {{'transition_double_z_to_i'}, 0.5}; 

% Add reverse transitions for existing entries 
% e.g., if a -> n transition is defined, define n -> a transition using
% a -> n entry 

% Exception is for moving signs, i.e., j, z and double z, since going to 
% one of these letters (e.g., from a -> j_stage1) is different from
% coming from one of these letters (e.g., from j_stage3 -> a)

movingIdx = find(ismember(letterNames, {'j','z','double_z'}));
[r, c] = find(~cellfun(@isempty, transitionTbl));
notFromOrToMovingIdx = (~ismember(r,movingIdx)  & ~ismember(c,movingIdx));
r = r(notFromOrToMovingIdx);
c = c(notFromOrToMovingIdx);
originalEntries = transitionTbl(sub2ind([27, 27], r,c));
reverseOrderEntries = cellfun(@(x) {flip(x{1}), 1-flip(x{2})}, originalEntries, 'UniformOutput',false);
transitionTbl(sub2ind([27, 27], c,r)) = reverseOrderEntries;

save(['Trajectory generation', filesep, 'transitionTbl.mat'], "transitionTbl");

%% Helper functions
function idx = createIdxFromLetterNames(l1, l2)
letterNames = cell(1,27);
letterStr = 'a':'z';
for i = 1:26
    letterNames{i} = letterStr(i);
end
letterNames{end} = 'double_z';
row = find(ismember(letterNames, l1));
col = find(ismember(letterNames, l2));
sz = [27, 27];
idx = sub2ind(sz,row,col);
end

function signSeq = addLetterPrefix(letterCell)
    signSeq = cellfun(@(x) ['letter_', x], letterCell, 'UniformOutput', false);
end