clear all
close all
clc

load logdata_op.txt
data_op = logdata_op;
load logdata_cl1.txt
data_cl = logdata_cl1;

% (time.time(), V_ref[ref_count], target_x_vel, LaccX_fb, LaccZ_fb, LvelX_fb, LposX_fb, AvelY_fb,  AthetaY_fb)

time_op = data_op(:,1) - data_op(1,1);
V_ref_op = data_op(:,2);
targetVx_op = data_op(:,3);
LaccX_fb_op = data_op(:,4);
LaccZ_fb_op = data_op(:,5);
LvelX_fb_op = data_op(:,6);
LposX_fb_op = data_op(:,7);
AvelY_fb_op = data_op(:,8);
AthetaY_fb_op = data_op(:,9);

time_cl = data_cl(:,1) - data_cl(1,1);
V_ref_cl = data_cl(:,2);
targetVx_cl = data_cl(:,3);
LaccX_fb_cl = data_cl(:,4);
LaccZ_fb_cl = data_cl(:,5);
LvelX_fb_cl = data_cl(:,6);
LposX_fb_cl = data_cl(:,7);
AvelY_fb_cl = data_cl(:,8);
AthetaY_fb_cl = data_cl(:,9);

% figure;
% plot(time_op, V_ref_op, 'r','LineWidth',1.5)
% hold on                                                                                                                                                                                                                                               
% plot(time_op, LvelX_fb_op, '--b','LineWidth',1.5)
% plot(time_op, LvelX_fb_op - V_ref_op, '--k','LineWidth',1.5)
% grid on
% legend('command', 'odometry Vx', 'error')
% title('Vx')
% set(gca,'Fontsize',16)
% xlabel('t(sec)')
% xlim([3 18])
% 
% figure;
% plot(time_cl, V_ref_cl, 'r','LineWidth',1.5)
% hold on                                                                                                                                                                                                                                               
% plot(time_cl, LvelX_fb_cl, '--b','LineWidth',1.5)
% plot(time_cl, LvelX_fb_cl - V_ref_cl, '--k','LineWidth',1.5)
% grid on
% legend('command', 'odometry Vx', 'error')
% title('Vx')
% set(gca,'Fontsize',16)
% xlabel('t(sec)')
% xlim([3 18])

figure;
plot(time_cl, V_ref_cl, 'k','LineWidth',1)
hold on                                                                                                                                                                                                                                               
plot(time_op, LvelX_fb_op, 'b','LineWidth',1)
plot(time_cl, LvelX_fb_cl, 'r','LineWidth',1)
grid on
legend('command', 'openloop', 'closedloop')
title('Vx')
set(gca,'Fontsize',16)
xlabel('t(sec)')
xlim([3 18])

figure;
plot(time_op, AthetaY_fb_op, 'b','LineWidth',1.5)
hold on                                                                                                                                                                                                                                               
plot(time_cl, AthetaY_fb_cl, '--r','LineWidth',1.5)
plot(time_cl, V_ref_cl*0.06/0.4, '--k','LineWidth',1)
grid on
legend('openloop', 'closedloop', 'Vx reference')
title('pitch angle')
set(gca,'Fontsize',16)
xlabel('t(sec)')
xlim([3 18])
ylim([-0.09 0.09])

figure;
plot(time_op, AvelY_fb_op, 'b','LineWidth',1)
hold on                                                                                                                                                                                                                                               
plot(time_cl, AvelY_fb_cl, '--r','LineWidth',1)
plot(time_cl, V_ref_cl*0.15/0.4, '--k','LineWidth',1)
grid on
legend('openloop', 'closedloop', 'Vx reference')
title('pitch velocity')
set(gca,'Fontsize',16)
xlabel('t(sec)')
xlim([3 18])





% 
% 
% 
% 
% figure;plot(time_op,vel_op,'r')
% % hold on
% figure;plot(time_cl,vel_cl,'--b')
% 
% 
% figure;plot(time_op,theta_op,'r')
% hold on
% plot(time_cl,theta_cl,'b')
% 
% 
% 
% figure;
% plot(tout,input_wo.signals.values(:,1),'b', 'LineWidth', 1.5)
% hold on
% plot(tout,input_w.signals.values(:,1),'--r', 'LineWidth', 1.5)
% legend('without suspension', 'with suspension')
% grid on
% title('real front input')
% xlabel('t(sec)'); ylabel('force(N)')
% set(gca,'Fontsize',16)