clear all
close all
clc

load logdata1.txt
data = logdata1;

% (time.time(), V_ref[ref_count], target_x_vel, LaccX_fb, LaccZ_fb, LvelX_fb, LposX_fb, AvelY_fb,  AthetaY_fb)

time = data(:,1) - data(1,1);
V_ref = data(:,2);
targetVx = data(:,3);
LaccX_fb = data(:,4);
LaccZ_fb = data(:,5);
LvelX_fb = data(:,6);
LposX_fb = data(:,7);
AvelY_fb = data(:,8);
AthetaY_fb = data(:,9);

figure;
hold on
plot(time, V_ref, 'r')
hold on                                                                                                                                                                                                                                               
plot(time, LvelX_fb, 'b')
plot(time, LvelX_fb - V_ref, '--k')
% plot(time, targetVx, 'k')
grid on
legend('joystic command', 'odometry Vx', 'error')
title('Vx')

figure;plot(time,cumsum(V_ref)*0.02, 'r');title('LposX fb');
hold on;plot(time,LposX_fb, 'b'); grid on;

figure;plot(time,AvelY_fb, 'r');title('AvelY fb');
hold on;plot(time,LvelX_fb*0.05, 'b'); grid on;

figure;plot(time,AthetaY_fb, 'r');title('AthetaY fb');
hold on;plot(time,LvelX_fb*0.2, 'b'); grid on;
