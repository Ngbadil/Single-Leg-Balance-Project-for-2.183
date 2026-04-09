clear all; close all; clc;

% setpath    % add AutoDerived, Modeling, Visualization folders to Matlab path


%% Gather information from participants

participants = readtable('table_1_data_deidentified.csv'); %Import participant data using struct

modifiedParticipants = participants; %Create copy of table

modifiedParticipants.Height_in_ = modifiedParticipants.Height_in_*0.0254; %Convert inches to meters
modifiedParticipants = renamevars(modifiedParticipants,'Height_in_','Height_m'); %Rename table column name

modifiedParticipants.Weight_lb_ = modifiedParticipants.Weight_lb_*0.453592; %Convert lb to kg
modifiedParticipants = renamevars(modifiedParticipants,'Weight_lb_','Mass_kg_'); %Rename table column name


%% Parameters and Initial Conditions
p   = parameters();                  % DIP parameters (no joint_lim or ground needed)

%Test parameters
for i = 1:1%size(modifiedParticipants,1)
    % totalMass_kg, totalHeight_m, gender, plane, theta_rad
    test = getLumpedParams_DIP_Ypose(modifiedParticipants.Mass_kg_(i),modifiedParticipants.Height_m(i),modifiedParticipants.Gender_1_M_2_F_(i),'frt',pi/4);

    p(1) = test.L1; p(2)=test.L2;p(3) = test.c1; p(4)=test.c2; p(5)=test.m1; p(6)=test.m2; p(7)=test.j1; p(8)=test.j2;
end

z_eq = [0; 0; 0; 0];                % upright equilibrium [th1, th2, dth1, dth2]
u_eq = [0; 0];
z0   = [0.1; 0.05; 0; 0];           % small perturbation from upright, at rest

tf   = 2;                            % simulation duration (s)


%% Compute LQR Gain
[A_num, B_num] = linearization_DIP(z_eq, u_eq, p);

% Sanity check — should see two eigenvalues with positive real parts
disp('Open-loop eigenvalues:')
disp(eig(A_num))

% Bryson's rule starting point:
% Q(i,i) = 1/max_acceptable_error^2, R(j,j) = 1/max_torque^2
Q_lqr = diag([1/0.17^2, 1/0.17^2, 1/1^2, 1/1^2]);  % ~10 deg tilt, 1 rad/s
R_lqr = diag([1/50^2, 1/50^2]);                      % 50 Nm max torque

K = lqr(A_num, B_num, Q_lqr, R_lqr);
disp('LQR gain K:')
disp(K)

%% Simulate
[tout, zout, uout] = simulate_DIP(z0, K, z_eq, p, [0 tf], [-2*pi/3 2*pi/3]);

%% Plot joint angles
figure(1)
subplot(2,1,1)
plot(tout, rad2deg(zout(1,:)), 'b', ...
     tout, rad2deg(zout(2,:)), 'r')
legend('\theta_1 (ankle)', '\theta_2 (hip)')
xlabel('time (s)')
ylabel('angle (deg)')
title('Joint Angles')

subplot(2,1,2)
plot(tout, rad2deg(zout(3,:)), 'b', ...
     tout, rad2deg(zout(4,:)), 'r')
legend('\omega_1 (ankle)', '\omega_2 (hip)')
xlabel('time (s)')
ylabel('angular velocity (deg/s)')
title('Joint Velocities')

%% Plot control torques
figure(2)
plot(tout, uout(1,:), 'b', ...
     tout, uout(2,:), 'r')
legend('\tau_1 (ankle)', '\tau_2 (hip)')
xlabel('time (s)')
ylabel('torque (Nm)')
title('Control Torques')

%% Plot CoM height
num_steps = size(zout, 2);
rB = zeros(2, num_steps);
for i = 1:num_steps
    keys = r_DIP(zout(:,i), p);
    rB(:,i) = keys(5:6);
end

figure(3)
plot(tout, rB(2,:))
xlabel('time (s)')
ylabel('height (m)')
title('Head height')

%% Animate
figure(4)
clf
speed = 0.5;
animateSol(tout, zout, p)