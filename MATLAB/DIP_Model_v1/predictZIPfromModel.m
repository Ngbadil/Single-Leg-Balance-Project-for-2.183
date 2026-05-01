function [f, zIP_ratio, H] = predictZIPfromModel(input_struct)
% PREDICTZIPFROMMODEL Compute closed-form zIP estimate from model
%
% Compute the closed-form estimate of the intersection-point height (zIP)
% over frequency from a double-inverted-pendulum model of bipedal stance
% based on the cross-spectral-density (CPSD) matrix between 
% center of pressure (COP) and foot-force orientation (theta_F)
%
% Inputs:
% input_struct = struct of simulation parameters, including:
%   'lumped_params'     : struct of lumped parameters for DIP
%     'm1' : mass of link 1 (ankle to hip)
%     'm2' : mass of link 2 (above hip)
%     'c1' : distance from ankle joint to center of mass of link 1
%     'c2' : distance from ankle joint to center of mass of link 2
%     'j1' : moment of inertia of link 1 about its center of mass
%     'j2' : moment of inertia of link 2 about its center of mass
%     'L1' : length of link 1
%     'L_COM': height of net center of mass 
%   'controller_params' : struct of LQR cost function weighting matrices
%     'Q'  : 4x4 state penalty matrix, default ones(4)
%     'R'  : 2x2 control-input penalty matrix, default 1e6*[0.3 0; 0 1/0.3]
%   'motorNoiseRatio' : ankle actuator noise / hip actuator noise,
%                        default 1.6
%   'f'  : frequencies at which to evaluate zIP
%
% Outputs:
% f         = frequencies at which zIP is evaluated
% zIP_ratio = zIP/zCOM; intersection-point height normalized by COM height
% H         = closed-loop transfer function fo system
%
% Rika Sugimoto-Dimitrova (rikasd@mit.edu)
% 2024-02-06
% Last updated: 2024-06-18

%% Load params

if nargin < 1
    % Default parameters
    alpha = 1e6; beta = 0.2;

    lumped_params = ... % Lumped model parameters from Shiozawa et al. 2021
        struct('m1',26.30,'m2',42.88,...
               'c1',0.589,'c2',0.332,...
               'j1',1.400,'j2',2.227,...
               'L1',0.867,'L_COM',0.967);
    controller_params.Q = eye(4);
    controller_params.R = alpha*[beta 0; 0 1/beta];
    motorNoiseRatio     = 1.6;

    f = 0.1:0.09765625:50; % same bins as 60s data sampled at 100 Hz

else
    % Input parameters
    lumped_params     = input_struct.lumped_params;
    controller_params = input_struct.controller_params;
    motorNoiseRatio   = input_struct.motorNoiseRatio;
    f                 = input_struct.f;
end

w = 2*pi*f;

%% Generate model dynamics
% linearize dynamics
[A_CL,B_CL,C_CL,D_CL] = getModel_linDIP(lumped_params,controller_params);

s = tf('s');
H = C_CL * inv(s*eye(size(A_CL)) - A_CL) * B_CL + D_CL;

Gww = [motorNoiseRatio^2 0; 0 1];

%% Get frequency response

[H_mag,H_phase,~] = bode(H,w);
[HT_mag,HT_phase,~] = bode(Gww*(H.'),w);
HT_frf = HT_mag.*exp(1j*pi/180*HT_phase);
HC_frf = H_mag.*exp(-1j*pi/180*H_phase);

%% Compute zIP
zIP_ratio = f*0;
for i = 1:length(w)
    Gcpsd_frf(:,:,i) = HC_frf(:,:,i)*HT_frf(:,:,i);
    [V,D] = eig(real(Gcpsd_frf(:,:,i)));
    [~,i_max] = max(diag(abs(D)));
    zIP_ratio(i) = V(2,i_max)/V(1,i_max) / lumped_params.L_COM;
end

end