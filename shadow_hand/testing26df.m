shr26df_rbt = importrobot(['URDF', filesep, 'shadow_hand_right_26df.urdf']);
jointNames = {'ARMJ2','ARMJ1','WRJ2', 'WRJ1', 'FFJ4', 'FFJ3', 'FFJ2', 'FFJ1', 'LFJ5', 'LFJ4', 'LFJ3', 'LFJ2', 'LFJ1', ...
    'MFJ4', 'MFJ3', 'MFJ2', 'MFJ1', 'RFJ4', 'RFJ3', 'RFJ2', 'RFJ1', 'THJ5', 'THJ4', 'THJ3', 'THJ2', 'THJ1'};

nJoints = 26;

Ts = 0.001; % sample time
qWaypoints = zeros(2, nJoints);
qWaypoints(2, 1) = -0.1;
tWaypoints = [0 1];
tFinal = tWaypoints(end);

diff1 = abs(diff(qWaypoints, 1));
diff2 = abs(diff(wrapTo2Pi(qWaypoints), 1));
wrapTo2PiIdx = round(diff2, 4) < round(diff1, 4);
qWaypoints2 = qWaypoints(2:end, :);
qWaypoints2(wrapTo2PiIdx) = wrapTo2Pi(qWaypoints2(wrapTo2PiIdx));
qWaypoints2 = [qWaypoints(1,:); qWaypoints2];

qInterp = pchip(tWaypoints,qWaypoints2',0:Ts:tFinal);

ds = jointValuesToInputSignals(qInterp, jointNames, Ts, tFinal, '');

mdl = "User input models/shr26df_user_input.slx";
supplyInputToUserInputMdlByDs(mdl, ds);