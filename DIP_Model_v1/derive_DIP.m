clear
name = 'DIP';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t th1 dth1 ddth1 th2 dth2 ddth2 I1 I2 l_OA l_AB l_Om1 l_Am2 m1 m2 tau1 tau2 Fx Fy g real; 

% Group them
q   = [th1 th2]';      % generalized coordinates
dq  = [dth1 dth2]';    % first time derivatives (angular velocity)
ddq = [ddth1 ddth2]';  % second time derivatives (angular acceleration)
u   = [tau1 tau2]';     % controls
p   = [ l_OA; l_AB; l_Om1; l_Am2; m1; m2; I1; I2; g;];        % parameters 

% Generate Vectors and Derivatives
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);
e1hat = sin(th1)*ihat + cos(th1)*jhat;                            % vector of first linkage
e2hat = sin(th1 + th2)*ihat + cos(th1 + th2)*jhat;     % vector of second linkage

rO= [0;0;0];                % foot position
rA = l_OA*e1hat;            % hip position
rB = rA + l_AB*e2hat;       % head position
r_m1= l_Om1*e1hat;          % leg CoM position
r_m2= rA + l_Am2*e2hat;     % torso CoM position

ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; % a handy anonymous function for taking time derivatives

drO = ddt(rO);      % foot velocity (should be zero vector)
drA = ddt(rA);      % hip velocity
drB = ddt(rB);      % head velocity
dr_m1= ddt(r_m1);   % leg CoM velocity
dr_m2 = ddt(r_m2);  % torso CoM velocity

% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));    % force contributions to generalized forces
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));   % moment contributions to generalized forces

% Find angular velocity of each link
omega0 = 0;
omega1 = dth1;
omega2 = dth1 + dth2;


T1 = (1/2)*m1 * dot(dr_m1,dr_m1) + (1/2) * I1 * omega1^2;   % Kinetic Energy of Link 1
T2 = (1/2)*m2 * dot(dr_m2,dr_m2) + (1/2) * I2 * omega2^2;   % Kinetic Energy of Link 2

Vg1 = m1*g*dot(r_m1, jhat);   % Potential Energy  due to Gravity of Link 1
Vg2 = m2*g*dot(r_m2, jhat);   % Potential Energy  due to Gravity of Link 2

T = simplify(T1 + T2);  % Kinetic Energy of System
Vg = Vg1 + Vg2;         % Potential Energy due to Gravity of System
V = Vg;                 % Potential Energy of System

Q_tau1 = M2Q(tau1*khat,dth1*khat);  % Moment contribution of tau1
Q_tau2 = M2Q(tau2*khat,dth2*khat);  % Moment contribution of tau2
Q = Q_tau1 + Q_tau2;

% Calculate rcm, the location of the center of mass
rcm = (m1*r_m1 + m2*r_m2)/(m1+m2);

% Assemble the array of cartesian coordinates of the key points
r = [rO(1:2); rA(1:2); rB(1:2)];
dr = [drO(1:2); drA(1:2); drB(1:2)];

%% All the work is done!  Just turn the crank...
% Derive Energy Function and Equations of Motion
E = T+V;
L = T-V;
eom = ddt(jacobian(L,dq).') - jacobian(L,q).' - Q;

% Rearrange Equations of Motion
A = simplify(jacobian(eom,ddq));
b = simplify(A*ddq - eom); % FOR SOME REASON I HAD TO ADD SIMPLIFY

% Write Energy Function and Equations of Motion
z  = [q ; dq];

%% Linearize about upright equilibrium z* = [0; 0; 0; 0]
z_eq = [0; 0; 0; 0];   % [th1, th2, dth1, dth2] all zero
u_eq = [0; 0];          % no torque needed at equilibrium

% Substitute equilibrium into A and b
A_eq = simplify(subs(A,   [q; dq; tau1; tau2], [z_eq; u_eq]));
b_eq = simplify(subs(b,   [q; dq; tau1; tau2], [z_eq; u_eq]));

% Jacobians of b with respect to state and control
db_dz  = simplify(subs(jacobian(b, [q; dq]),   [q; dq; tau1; tau2], [z_eq; u_eq]));
db_du  = simplify(subs(jacobian(b, [tau1; tau2]), [q; dq; tau1; tau2], [z_eq; u_eq]));

% Linearized dynamics matrices
% dz/dt = A_lin*z + B_lin*u
% top half: d(dq)/dz = [0 I]
% bottom half: A_eq^-1 * db_dz and A_eq^-1 * db_du
A_lin_sym = simplify([zeros(2),   eye(2);
                       A_eq \ db_dz]);

B_lin_sym = simplify([zeros(2,2);
                       A_eq \ db_du]);

%% Compute Jacobians for contact with ground
J_rA = jacobian(rA(1:2),q);
J_rB = jacobian(rB(1:2),q);

%% Turn into functions (faster computations)
matlabFunction(A,'file',['A_' name],'vars',{z p});
matlabFunction(b,'file',['b_' name],'vars',{z u p});
matlabFunction(A_lin_sym, 'file', ['Alin_' name], 'vars', {p});
matlabFunction(B_lin_sym, 'file', ['Blin_' name], 'vars', {p});
matlabFunction(E,'file',['E_' name],'vars',{z p});
matlabFunction(r,'file',['r_' name],'vars',{z p});
matlabFunction(dr,'file',['dr_' name],'vars',{z p});
matlabFunction(J_rA,  'file', ['J_rA_'  name], 'vars', {z, p});
matlabFunction(J_rB,  'file', ['J_rB_'  name], 'vars', {z, p});



