%% Get joint values manually set in test_asl_poses_quick_edit.slx files
%out = sim('left_test_asl_poses_quick_edit.slx');
out = sim('right_test_asl_poses_quick_edit.slx');
jointValues = out.jointValues{1}.Values.Data(1,:);

% Save joint values to mat files -- make sure to change mat file name
% depending on which ASL sign you've recreated!
fileName = 'number_3';
save(['Configs', filesep, fileName, '.mat'], "jointValues");
jointValuesToConstInputSignals(jointValues, jointNames, 0.001, 2, fileName)