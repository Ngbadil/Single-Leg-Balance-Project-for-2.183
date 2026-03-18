function [A_lin, B_lin] = linearize_DIP(z_eq, u_eq, p)
    eps = 1e-6;
    n = length(z_eq);
    m = length(u_eq);
    
    % Evaluate f = A\b at equilibrium
    f0 = f_DIP(z_eq, u_eq, p);
    
    % Linearize w.r.t. state (A_lin)
    A_lin = zeros(n, n);
    for i = 1:n
        z_pert = z_eq;
        z_pert(i) = z_pert(i) + eps;
        A_lin(:,i) = (f_DIP(z_pert, u_eq, p) - f0) / eps;
    end
    
    % Linearize w.r.t. control (B_lin)
    B_lin = zeros(n, m);
    for i = 1:m
        u_pert = u_eq;
        u_pert(i) = u_pert(i) + eps;
        B_lin(:,i) = (f_DIP(z_eq, u_pert, p) - f0) / eps;
    end
end

function dz = f_DIP(z, u, p)
    % Full nonlinear state derivative
    M   = A_DIP(z, p);
    bvec = b_DIP(z, u, p);
    qdd = M \ bvec;
    dz  = [z(3:4); qdd];
end