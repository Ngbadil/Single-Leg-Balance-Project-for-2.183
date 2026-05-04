function [A_cl,B_cl,C_cl,D_cl] = getModel_linDIP(lumped_params,controller_params)
%GETMODEL_LINDIP Get full-state-feedback closed-loop (cl) state-space model 
%               of linearized double-inverted pendulum (DIP) with 
%               linear quadratic regulator (LQR) controller 
%
% Inputs:
% 'lumped_params' = struct of lumped parameters for DIP
%   'l_OA'  : distance from ankle joint to hip joint
%   'l_AB'  : distance from hip joint to head
%   'l_Om1' : distance from ankle joint to center of mass of link 1
%   'l_Am2' : distance from hip joint to center of mass of link 2
%   'm1'    : mass of leg segment
%   'm2'    : mass of torso segment
%   'I1'    : moment of inertia of link 1 about its center of mass
%   'I2'    : moment of inertia of link 2 about its center of mass
%   'g'     : gravity
% controller_params = struct containing controller parameters for the LQR
%   'Q'  : 4x4 state penalty matrix
%   'R'  : 2x2 control-input penalty matrix
%
% Outputs:
% State-space model matrices A_cl, B_cl, C_cl, D_cl
%   A = dynamics matrix
%   B = input matrix
%   C = output matrix
%   D = direct feedthrough matrix
%
% Rika Sugimoto Dimitrova
% 2024-02-10
% Reference: Shiozawa et al. 2021, Appendices 2-4
% Last updated: 2024-06-18

p = [lumped_params.L1; lumped_params.L2; lumped_params.c1; lumped_params.c2; lumped_params.m1; lumped_params.m2; lumped_params.j1; lumped_params.j2;];
m1 = lumped_params.m1;
m2 = lumped_params.m2;

q_eq = [0;0]; Dq_eq = [0;0];
z = [q_eq;Dq_eq];
J_CoM = J_CoM_DIP(z,p);
DJ_CoM = DJ_CoM_DIP(z,p);

% open-loop system state-space matrices
% A_ol = [zeros(2) eye(2); -M\J_G zeros(2)];
% B_ol = [zeros(2); M\eye(2)];
A_ol = Alin_DIP(p);
B_ol = Blin_DIP(p);

C_2 = [0 0 0 0];
D_2 = [1 0];

J_CoM_x = J_CoM(1,:);
DJ_CoM_x = DJ_CoM(1,:);
J_2 = -(m1+m2)*[DJ_CoM_x J_CoM_x];
C_1 = J_2*A_ol;
D_1 = J_2*B_ol;
C_ol = [C_1; C_2];
D_ol = [D_1; D_2];

% LQR controller gains
K_x = lqr(A_ol,B_ol,controller_params.Q,controller_params.R); 

A_cl = A_ol - B_ol*K_x;
B_cl = B_ol;
C_cl = C_ol - D_ol*K_x;
D_cl = D_ol;

end

