dt = 0.02;

accel_t1 = 0.5;
accel_t2 = 1.0;
accel_t3 = 2.0;

decel_t1 = 0.1;
decel_t2 = 0.5;
decel_t3 = 1.0;
decel_t4 = 2.0;
decel_t5 = 5.0;

% profile_05_01 = [zeros(1,50) linspace(0,0.4,accel_t1/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t1/dt+1)];
profile_05_05 = [zeros(1,200) linspace(0,0.4,accel_t1/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t2/dt+1)];
% profile_05_10 = [zeros(1,50) linspace(0,0.4,accel_t1/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t3/dt+1)];
% profile_05_20 = [zeros(1,50) linspace(0,0.4,accel_t1/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t4/dt+1)];
% profile_05_50 = [zeros(1,50) linspace(0,0.4,accel_t1/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t5/dt+1)];

% profile_10_01 = [zeros(1,50) linspace(0,0.4,accel_t2/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t1/dt+1)];
% profile_10_05 = [zeros(1,50) linspace(0,0.4,accel_t2/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t2/dt+1)];
% profile_10_10 = [zeros(1,50) linspace(0,0.4,accel_t2/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t3/dt+1)];
% profile_10_20 = [zeros(1,50) linspace(0,0.4,accel_t2/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t4/dt+1)];
% profile_10_50 = [zeros(1,50) linspace(0,0.4,accel_t2/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t5/dt+1)];
% 
% profile_20_01 = [zeros(1,50) linspace(0,0.4,accel_t3/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t1/dt+1)];
% profile_20_05 = [zeros(1,50) linspace(0,0.4,accel_t3/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t2/dt+1)];
% profile_20_10 = [zeros(1,50) linspace(0,0.4,accel_t3/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t3/dt+1)];
% profile_20_20 = [zeros(1,50) linspace(0,0.4,accel_t3/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t4/dt+1)];
% profile_20_50 = [zeros(1,50) linspace(0,0.4,accel_t3/dt+1) 0.4*ones(1,3/dt) linspace(0.4,0,decel_t5/dt+1)];

% profile_05_01 = [profile_05_01 profile_05_01 profile_05_01 zeros(1,50)];
profile_05_05 = [profile_05_05 profile_05_05 zeros(1,200)];
% profile_05_10 = [profile_05_10 profile_05_10 profile_05_10 zeros(1,50)];
% profile_05_20 = [profile_05_20 profile_05_20 profile_05_20 zeros(1,50)];
% profile_05_50 = [profile_05_50 profile_05_50 profile_05_50 zeros(1,50)];
% 
% profile_10_01 = [profile_10_01 profile_10_01 profile_10_01 zeros(1,50)];
% profile_10_05 = [profile_10_05 profile_10_05 profile_10_05 zeros(1,50)];
% profile_10_10 = [profile_10_10 profile_10_10 profile_10_10 zeros(1,50)];
% profile_10_20 = [profile_10_20 profile_10_20 profile_10_20 zeros(1,50)];
% profile_10_50 = [profile_10_50 profile_10_50 profile_10_50 zeros(1,50)];
% 
% profile_20_01 = [profile_20_01 profile_20_01 profile_20_01 zeros(1,50)];
% profile_20_05 = [profile_20_05 profile_20_05 profile_20_05 zeros(1,50)];
% profile_20_10 = [profile_20_10 profile_20_10 profile_20_10 zeros(1,50)];
% profile_20_20 = [profile_20_20 profile_20_20 profile_20_20 zeros(1,50)];
% profile_20_50 = [profile_20_50 profile_20_50 profile_20_50 zeros(1,50)];


% textfilesave('r_05_01.txt',profile_05_01)
textfilesave('r_05_05.txt',profile_05_05)
% textfilesave('r_05_10.txt',profile_05_10)
% textfilesave('r_05_20.txt',profile_05_20)
% textfilesave('r_05_50.txt',profile_05_50)
% textfilesave('r_10_01.txt',profile_10_01)
% textfilesave('r_10_05.txt',profile_10_05)
% textfilesave('r_10_10.txt',profile_10_10)
% textfilesave('r_10_20.txt',profile_10_20)
% textfilesave('r_10_50.txt',profile_10_50)
% textfilesave('r_20_01.txt',profile_20_01)
% textfilesave('r_20_05.txt',profile_20_05)
% textfilesave('r_20_10.txt',profile_20_10)
% textfilesave('r_20_20.txt',profile_20_20)
% textfilesave('r_20_50.txt',profile_20_50)

% figure;plot(profile_05_01)
% hold on
% plot(profile_05_05)
% plot(profile_05_10)
% plot(profile_05_20)
% plot(profile_05_50)
% 
% plot(profile_10_05)
% plot(profile_10_10)
% plot(profile_10_20)
% plot(profile_10_50)
% plot(profile_10_50)
% 
% plot(profile_20_05)
% plot(profile_20_10)
% plot(profile_20_20)
% plot(profile_20_50)
% plot(profile_20_50)
