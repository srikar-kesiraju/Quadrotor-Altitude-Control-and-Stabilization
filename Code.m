%% 
% Input equation

syms w z theta q T Tau m Iyy
x=[z;w;theta;q];
A=[0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
B=[0 0;1/0.5 0;0 0;0 1/0.1];
U=[T;Tau];%this determines whether its siso or mimo
dx=A*x+B*U;
%% 
% Output equation

C=eye(4);
D=zeros(4,2);
dy=C*x+D*U;
%% 
% state space model

sys=ss(A,B,C,D);
%% 
% controllability and Observability

P=ctrb(A,B);
r=rank(P);
if r == size(A, 1)
    disp('The system is controllable.');
else
    disp('The system is not controllable.');
end
Ob = obsv(A,C);

if rank(Ob) == 4
    disp('System is Observable.');
else
    disp('System is NOT Observable.');
end
%% 
% Open Loop Stability

[wn,eta,p]=damp(A);%since the Open Loop equation is marginally stable. We use a gain matrix by place command to make it asymptotically stable
%% 
% Control Gain Matrix 
% 
% The design requirements are the following.
%% 
% * Overshoot less than 10%
% * Rise time less than 2 seconds
% * Settling time less than 10 seconds
% * Steady-state error less than 2%== Kp>49 Kp=52
% * natural frequency is 3
% * damping ratio is 0.85

A1 = [0 1; 0 0];
B1 = [0; 1/0.5];
poles1 = [-2.83+2.1j, -2.83-2.1j];
K1 = place(A1, B1, poles1);

A2 = [0 1; 0 0];
B2 = [0; 1/0.1];
poles2 = [-4.0, -4.1];
K2 = place(A2, B2, poles2);

K_decoupled = [K1(1), K1(2),    0,       0;
               0,      0,      K2(1), K2(2)];
disp(K_decoupled)
%% 
% Designing the Controller.
% 
% Since we need to move it to another place and stabilize it 
% 
% its a Tracking Problem
% 
% U=k(xd-xo)-kd*xd

A_cl=A-B*K_decoupled;
sys_cl = ss(A_cl, B, C, D);
full=dcgain(sys_cl);
kd=inv(full([1,3],:));
disp(kd)
sys_tracking = ss(A_cl, B*kd, C, D);
damp(A_cl)
xd=[1;0;0.1;0];
xd_red=[xd(1);xd(3)];
xo=[0;0;0;0];
U=-K_decoupled*(xo-xd)+kd*xd_red;
disp(U);
%% 
% Simulating it 
% 
% Part A 
% 
% Taking it to a specific orientation and making sure it goes there with out 
% exceeding the design parameters

t=0:0.01:20;
[y_out,t_delayed]=lsim(sys_tracking,repmat(xd_red',length(t),1),t,xo);
figure('Name', 'Quadrotor MIMO Tracking Results');
subplot(2,2,1);
plot(t_delayed, y_out(:,1), 'LineWidth', 2); hold on;
yline(xd_red(1), '--r', 'Target'); 
grid on; title('Altitude (z)'); ylabel('Meters (m)');
legend('Response', 'Target');

subplot(2,2,2);
plot(t_delayed, y_out(:,2), 'LineWidth', 1.5);
grid on; title('Vertical Velocity (w)'); ylabel('m/s');

subplot(2,2,3);
plot(t_delayed, y_out(:,3), 'LineWidth', 2); hold on;
yline(xd_red(2), '--r', 'Target'); 
grid on; title('Pitch Angle (\theta)'); ylabel('Radians (rad)');
xlabel('Time (s)');

subplot(2,2,4);
plot(t_delayed, y_out(:,4), 'LineWidth', 1.5);
grid on; title('Pitch Rate (q)'); ylabel('rad/s');
xlabel('Time (s)');

subplot(2,2,1)
xlim([-1.9 18.1])
ylim([0.52 1.53])
info_z_ideal = stepinfo(y_out(:,1), t_delayed, xd_red(1));
info_th_ideal = stepinfo(y_out(:,3), t_delayed, xd_red(2));
Ideal_Results = table({'Altitude (z)'; 'Pitch (theta)'}, ...
    [info_z_ideal.Overshoot; info_th_ideal.Overshoot], ...
    [info_z_ideal.RiseTime; info_th_ideal.RiseTime], ...
    [info_z_ideal.Peak; info_th_ideal.Peak], ...
    'VariableNames', {'State', 'Overshoot_Pct', 'RiseTime_s', 'Peak_Value'});

fprintf('\n--- Baseline MIMO Tracking Performance (Ideal Case) ---\n');
disp(Ideal_Results);
%% 
% Part B
% 
% Testing wheter it will sustain if a disturbance occurs.


dist_accel = zeros(length(t), 1);
dist_accel(t >= 4 & t <= 5) = 1.0;
disturbed_input = repmat(xd_red', length(t), 1) + dist_accel;
[y_disturbed, t_disturbed] = lsim(sys_tracking, disturbed_input, t, xo);
figure('Name', 'Disturbed Quadrotor MIMO Tracking Results');
subplot(2,2,1);
plot(t_disturbed, y_disturbed(:,1), 'LineWidth', 1.5); hold on;
yline(xd_red(1), '--r', 'Target'); 
grid on; title('Altitude (z) with Disturbance'); ylabel('Meters (m)');
legend('Response', 'Target');
%% 
% Checking whether the design requirements are satified or not

sys_z = ss(A_cl(1:2,1:2), B(1:2,1)*kd(1,1), C(1:2,1:2), D(1:2,1));
info_z = stepinfo(sys_z);

sys_th = ss(A_cl(3:4,3:4), B(3:4,2)*kd(2,2), eye(2),zeros(2,1));
info_th = stepinfo(sys_th);
Disturbed_Results=table({'Altitude (z)';'Pitch (theta)'},...
    [info_z.Overshoot;info_th.Overshoot],...
    [info_z.RiseTime;info_th.RiseTime],...
    [info_z.Peak;info_th.Peak],...
    'VariableNames',{'State','Overshoot_Pct','RiseTime_s','PeakValue'});
fprintf('\n--- Baseline MIMO Disturbed Input Performance (Ideal Case) ---\n')
disp(Disturbed_Results);
%% 
% Effect of Sensor Delay on Target reaching

delays=0:0.2:1;
colors = lines(length(delays));
required_info=zeros(length(delays),4);
figure('Name', 'Sensor Delay Sensitivity Study');
ax1 = subplot(2,2,1); hold on; grid on; title('Altitude (z)'); ylabel('m');
ax2 = subplot(2,2,2); hold on; grid on; title('Vertical Velocity (w)'); ylabel('m/s');
ax3 = subplot(2,2,3); hold on; grid on; title('Pitch (\theta)'); ylabel('rad'); xlabel('Time (s)');
ax4 = subplot(2,2,4); hold on; grid on; title('Pitch Rate (q)'); ylabel('rad/s'); xlabel('Time (s)');
for i=1:length(delays)
    currentdelay=delays(i);
    if currentdelay==0
        [ysim,t_sim]=lsim(sys_tracking,repmat(xd_red',length(t),1),t,xo);
        step_info=stepinfo(ysim(:,1),t_sim,xd_red(1));
        required_info(1,:)=[0,step_info.Overshoot,step_info.RiseTime,step_info.Peak];
    else
        [num,den]=pade(currentdelay,2);
        delay_tf=tf(num,den);
        delay_mimo=delay_tf*eye(4);
        sys_delayed_loop=delay_mimo*sys_tracking;
        [y_delayed,t_delayed]=lsim(sys_delayed_loop,repmat(xd_red',length(t),1),t,[xo;zeros(8,1)]);
        s_info = stepinfo(y_delayed(:,1), t_delayed, xd_red(1));
        required_info(i,:)=[currentdelay,s_info.Overshoot,s_info.RiseTime,s_info.Peak];
    end
    plot(ax1,t_delayed,y_delayed(:,1))
    plot(ax2,t_delayed,y_delayed(:,2))
    plot(ax3,t_delayed,y_delayed(:,3))
    plot(ax4,t_delayed,y_delayed(:,4))
end
T_results = array2table(required_info, ...
    'VariableNames', {'Delay_Seconds', 'Overshoot_Percent', 'RiseTime_Seconds','Peak_Value'});
fprintf('\n--- UAV Sensor Delay:-Target Analysis ---\n');
disp(T_results);
yline(ax1, xd_red(1), '--r', 'Target');
yline(ax3, xd_red(2), '--r', 'Target');
legend(ax1, string(delays) + "s delay");
%% 
% Effect of Sensor delay on Disturbance Stability

dist_results=zeros(length(delays),4);
figure('Name', 'Disturbance Rejection with Sensor Delay');
ax_dist = subplot(1,1,1); hold on; grid on;
title('Altitude (z) Recovery from 2s Disturbance');
ylabel('Meters (m)'); xlabel('Time (s)');
for i=1:length(delays)
    currentdelay=delays(i);
    if currentdelay==0
        sys_loop=sys_tracking;
        xo_loop=xo;
    else
        [num,den]=pade(currentdelay,2);
        sys_loop = tf(num,den) * eye(4) * sys_tracking;
        xo_loop=[xo;zeros(8,1)];
    end
    disturbed_input=repmat(xd_red',length(t),1)+[dist_accel,zeros(length(t),1)];
    [y_dist,t_dist]=lsim(sys_loop,disturbed_input,t,xo_loop);
    s_info_dist      = stepinfo(y_dist(:,1), t_dist, xd_red(1));
    dist_results(i,:) = [currentdelay, ...
                          s_info_dist.Overshoot, ...
                          s_info_dist.RiseTime,...
                          s_info_dist.Peak];
    plot(ax_dist, t_dist, y_dist(:,1), 'Color', colors(i,:), 'LineWidth', 1.5);
end
disturbed_result=array2table(dist_results,...
    'VariableNames', {'Delay_Seconds', 'Overshoot_Percent', 'RiseTime_Seconds','Peak_Value'});
fprintf('\n--- UAV Sensor Delay:- Disturbed input Analysis ---\n')
disp(disturbed_result);
yline(ax_dist, xd_red(1), '--r', 'Target');
legend(ax_dist, string(delays) + "s delay");
ylim(ax_dist, [0.8 2.5]);