%% Get joint values manually set in test_asl_poses_quick_edit.slx files
%out = sim('left_test_asl_poses_quick_edit.slx');
out = sim('right_test_asl_poses_quick_edit.slx');
jointValues = out.jointValues{1}.Values.Data(1,:);

% Save joint values to mat files -- make sure to change mat file name
% depending on which ASL sign you've recreated!
%save('number_1.mat', "jointValues");
save('fist.mat', "jointValues");