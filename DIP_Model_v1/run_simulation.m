clear all; close all; clc;

% setpath    % add AutoDerived, Modeling, Visualization folders to Matlab path

%% Parameters and Initial Conditions
p   = parameters();                 % DIP parameters (no joint_lim or ground needed)
z_eq = [0; 0; 0; 0];                % upright equilibrium [th1, th2, dth1, dth2]
u_eq = [0; 0];
z0   = [0.1; 0.05; 0; 0];           % small perturbation from upright, at rest

tf   = 50;                          % simulation duration (s)

noise.motorNoiseLvL = 100;            % noise magnitude
noise.motorNoiseRatio = 1.6;        % ankle:hip noise ratio
noise.noise_type = 'l';             % 'w' for gaussian white noise, 'l' for low pass (more noticeable)


controller.Q = diag([1/0.17^2, 1/0.17^2, 1/1^2, 1/1^2]);  % ~10 deg tilt, 1 rad/s
controller.R = diag([1/50^2, 1/50^2]);                      % 50 Nm max torque

%% Create Input Struct
% Inputs:
% input_struct = struct of simulation parameters, including:
%   'lumped_params'     : struct of lumped parameters for DIP
%     'l_OA'  : distance from ankle joint to hip joint
%     'l_AB'  : distance from hip joint to head
%     'l_Om1' : distance from ankle joint to center of mass of link 1
%     'l_Am2' : distance from hip joint to center of mass of link 2
%     'm1'    : mass of leg segment
%     'm2'    : mass of torso segment
%     'I1'    : moment of inertia of link 1 about its center of mass
%     'I2'    : moment of inertia of link 2 about its center of mass
%     'g'     : gravity
%   'z0'                : initial position
%   'simFreq_Hz'        : simulation frequency (Hz), default 1000 Hz
%   'simDuration_s'     : simulation duration (s), default 60 s
%   'joint;             : joint limit for hip joint
%   'noise'             : struct of noise parameters
%     'motorNoiseLvL_Nm'  : standard deviation of actuator noise (Nm),
%                         default 10 Nm
%     'motorNoiseRatio'   : ankle actuator noise / hip actuator noise,
%                         default 1.6
%     'noise_type'        : type of noise 'w' for gaussian white noise,
%                         'l' for low pass (more noticeable)
%   'controller'        : struct of LQR cost function weighting matrices
%     'Q'  : 4x4 state penalty matrix, default ones(4)
%     'R'  : 2x2 control-input penalty matrix, default 1e6*[0.3 0; 0 1/0.3]

input_struct.lumped_params = p;
input_struct.z0 = z0;
input_struct.simFreq_Hz = 1000;
input_struct.simDuration_s = tf;
input_struct.joint = [-2*pi/3 2*pi/3];
input_struct.noise = noise;
input_struct.controller = controller;

%% Simulate
% [tout, zout, uout] = simulate_DIP(z0, p, [0 tf], [-2*pi/3 2*pi/3], noise, controller, 1000);
[tout, zout, uout] = simulate_DIP(input_struct);

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

%% Plot Foot Force
num_steps = size(zout, 2);
F_O = F_O_DIP(zout, uout, p);
%F_O = zeros(2, num_steps);
%for i = 1:num_steps
%    F_O(:,i) = F_O_DIP(zout(:,i), uout(:,i), p);
%end

figure(4)
subplot(2,1,1)
plot(tout, F_O(1,:))
xlabel('time (s)')
ylabel('Force in x-dir (N)')
title('Foot Force (x)')

subplot(2,1,2)
plot(tout, F_O(2,:))
xlabel('time (s)')
ylabel('Force in y-dir (N)')
title('Foot Force (y)')


%% Animate
speed = 0.5;
animateSol(tout, zout, p, speed)