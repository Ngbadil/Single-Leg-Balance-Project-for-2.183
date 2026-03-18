function [tout, zout, uout] = simulate_DIP(z0,K,z_eq,p,tspan, joint_lim) 
    %% Perform Dynamic simulation
    t0 = tspan(1); tend = tspan(end);
    dt = 0.0001;
    num_step = floor(tend/dt);
    tout = linspace(t0, tend, num_step);
    zout = zeros(4,num_step);
    zout(:,1) = z0;
    uout = zeros(2,num_step);
    
    for i=1:num_step-1
        t = tout(i);
        [dz,u] = dynamics(t, zout(:,i), p, K, z_eq, joint_lim);
        zout(:,i+1) = zout(:,i) + dz*dt;
        zout(3:4, i+1) = hip_discrete_contact(zout(:,i+1), p);
        zout(3:4, i+1) = head_discrete_contact(zout(:,i+1), p);
        uout(:,i) = u; 
    end
end

%% Control %%
function u = lqr_control(z, K, z_eq)
    u = -K * (z - z_eq);
end

function u = hip_joint_constraint(z, joint_lim)
    Cth = z(2);
    Cthdot = z(4);
    K_hip = 50000;
    D_hip = 100;

    u = [0;0];
    if Cth > joint_lim(2)                            % past upper limit
        u(2) = -K_hip*(Cth - joint_lim(2)) - D_hip*Cthdot;
    elseif Cth < joint_lim(1)                        % past lower limit
        u(2) = -K_hip*(Cth - joint_lim(1)) - D_hip*Cthdot;
    end
end

%% Dynamics %%
function [dz,u] = dynamics(t,z,p,K,z_eq, joint_lim)
    % Get mass matrix
    A = A_DIP(z,p);
    
    % Compute Controls
    u = lqr_control(z, K, z_eq);
    u = [0; 0];

    % Hip Joint Limit Constraint
    u = u + hip_joint_constraint(z,joint_lim);
    
    % Get b = Q - V(q,qd) - G(q)
    b = b_DIP(z,u,p);

    
    % Solve for qdd.
    qdd = A\(b);
    dz = 0*z;
    
    % Form dz
    dz(1:2) = z(3:4);
    dz(3:4) = qdd;
end

%% Discrete Contact %%
function qdot = hip_discrete_contact(z,p)
    qdot = z(3:4);
    r = r_DIP(z, p);
    dr = dr_DIP(z,p);
    J_rA = J_rA_DIP(z,p);
    rA = r(3:4);
    drA = dr(3:4);

    M = A_DIP(z,p);
    Jc = J_rA(2,:);    % just the y row
    Lambda = inv(Jc * inv(M) * Jc');
    Cy = rA(2);
    Cydot = drA(2);

    if Cydot < 0 && Cy < 0
        Fc = Lambda*(-Jc*z(3:4));
        qdot = qdot + inv(M)*Jc'*Fc;
    end
end

function qdot = head_discrete_contact(z,p)
    qdot = z(3:4);
    r = r_DIP(z, p);
    dr = dr_DIP(z,p);
    J_rB = J_rB_DIP(z,p);
    rB = r(5:6);
    drB = dr(5:6);

    M = A_DIP(z,p);
    Jc = J_rB(2,:);    % just the y row
    Lambda = inv(Jc * inv(M) * Jc');
    Cy = rB(2);
    Cydot = drB(2);

    if Cydot < 0 && Cy < 0
        Fc = Lambda*(-Jc*z(3:4));
        qdot = qdot + inv(M)*Jc'*Fc;
    end
end