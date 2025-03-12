tol = 1e-10;
jointValues(1) = 0.1;
q0 = jointValuesToConfigObj(jointValues, jointNames); 
%%
T1 = getTransform(rbt,q0,'forearm','world');
T2 = getTransform(rbt,q0,'wrist','forearm');
T3 = getTransform(rbt,q0,'palm','wrist');

T4 = getTransform(rbt,q0,'palm','world');

disp(norm(T1*T2*T3 - T4) < tol);

%%
T0 = getTransform(rbt,q0,'world','world2');
T1 = getTransform(rbt,q0,'forearm','world');
T2 = getTransform(rbt,q0,'wrist','forearm');
T3 = getTransform(rbt,q0,'palm','wrist');

T4 = getTransform(rbt,q0,'palm','world2');

disp(norm(T0*T1*T2*T3 - T4) < tol);
%%
T1 = getTransform(rbt,q0,'palm','world');
T2 = getTransform(rbt,q0,'ffknuckle','palm');
T3 = getTransform(rbt,q0,'ffproximal','ffknuckle');
T4 = getTransform(rbt,q0,'ffmiddle','ffproximal');
T5 = getTransform(rbt,q0,'ffdistal','ffmiddle');
T6 = getTransform(rbt,q0,'fftip','ffdistal');

T7 = getTransform(rbt,q0,'fftip','world');

disp(norm(T1*T2*T3*T4*T5*T6 - T7) < tol);
%%
T1 = getTransform(rbt,q0,'palm','world');
T2 = getTransform(rbt,q0,'thbase','palm');
T3 = getTransform(rbt,q0,'thproximal','thbase');
T4 = getTransform(rbt,q0,'thhub','thproximal');
T5 = getTransform(rbt,q0,'thmiddle','thhub');
T6 = getTransform(rbt,q0,'thdistal','thmiddle');
T7 = getTransform(rbt,q0,'thtip','thdistal');

T8 = getTransform(rbt,q0,'thtip','world');

disp(norm(T1*T2*T3*T4*T5*T6*T7 - T8) < tol);