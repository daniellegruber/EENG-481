% curledInFingers: contains all the (non-thumb) fingers curled into the palm
% thumbRestingFinger: if the thumb rests on a curled in finger, describes
% which finger the thumb rests upon, e.g., RF in numbers 1 and 2
% thumbTouchesFinger: if the thumb touches a finer, describes which finger
% the thumb touches, e.g. RF in number 7
number_pose_params = repmat(struct(...
    'name', '', ...
    'curledInFingers',{}, ...
    'thumbRestingFinger', {}, ...
    'thumbTouchesFinger', {}, ...
    'afterAdjustments', {}), ...
    1, 10);

%% Number 1
number_pose_params(1).name = 'number_1';
number_pose_params(1).curledInFingers = {'LF', 'RF', 'MF'};
number_pose_params(1).thumbRestingFinger = {'RF'};
number_pose_params(1).thumbTouchesFinger = {};
number_pose_params(1).afterAdjustments = {};

%% Number 2
number_pose_params(2).name = 'number_2';
number_pose_params(2).curledInFingers = {'LF', 'RF'};
number_pose_params(2).thumbRestingFinger = {'RF'};
number_pose_params(2).thumbTouchesFinger = {};
number_pose_params(2).afterAdjustments = {'MFJ4',deg2rad(4),'FFJ4',deg2rad(-4)};

%% Number 3
number_pose_params(3).name = 'number_3';
number_pose_params(3).curledInFingers = {'LF', 'RF'};
number_pose_params(3).thumbRestingFinger = {};
number_pose_params(3).thumbTouchesFinger = {};
number_pose_params(3).afterAdjustments = {'MFJ4',deg2rad(4),'FFJ4',deg2rad(-4)};

%% Number 4
number_pose_params(4).name = 'number_4';
number_pose_params(4).curledInFingers = {};
number_pose_params(4).thumbRestingFinger = {'LF'};
number_pose_params(4).thumbTouchesFinger = {};
number_pose_params(4).afterAdjustments = {'LFJ4',deg2rad(-8),'RFJ4',deg2rad(-4), 'MFJ4',deg2rad(0),'FFJ4',deg2rad(-4)};

%% Number 5
number_pose_params(5).name = 'number_5';
number_pose_params(5).curledInFingers = {};
number_pose_params(5).thumbRestingFinger = {};
number_pose_params(5).thumbTouchesFinger = {};
number_pose_params(5).afterAdjustments = {'LFJ4',deg2rad(-8),'RFJ4',deg2rad(-4), 'MFJ4',deg2rad(0),'FFJ4',deg2rad(-4)};

%% Number 6
number_pose_params(6).name = 'number_6';
number_pose_params(6).curledInFingers = {};
number_pose_params(6).thumbRestingFinger = {};
number_pose_params(6).thumbTouchesFinger = {'LF'};
number_pose_params(6).afterAdjustments = {'RFJ4',deg2rad(-4), 'FFJ4',deg2rad(-4)};

%% Number 7
number_pose_params(7).name = 'number_7';
number_pose_params(7).curledInFingers = {};
number_pose_params(7).thumbRestingFinger = {};
number_pose_params(7).thumbTouchesFinger = {'RF'};
number_pose_params(7).afterAdjustments = {'LFJ4',deg2rad(-8), 'FFJ4',deg2rad(-6)};

%% Number 8
number_pose_params(8).name = 'number_8';
number_pose_params(8).curledInFingers = {};
number_pose_params(8).thumbRestingFinger = {};
number_pose_params(8).thumbTouchesFinger = {'MF'};
number_pose_params(8).afterAdjustments = {'LFJ4',deg2rad(-8), 'RFJ4',deg2rad(-4), 'FFJ4',deg2rad(-6)};

%% Number 9
number_pose_params(9).name = 'number_9';
number_pose_params(9).curledInFingers = {};
number_pose_params(9).thumbRestingFinger = {};
number_pose_params(9).thumbTouchesFinger = {'FF'};
number_pose_params(9).afterAdjustments = {'LFJ4',deg2rad(-8), 'RFJ4',deg2rad(-4), 'MFJ4',deg2rad(1)};