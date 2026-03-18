function [p, joint_lim, ground] = parameters() 
    % Human DIP Parameters
    
    % Link lengths (m)
    l_OA  = 0.9;       % ankle to hip (leg length)
    l_AB  = 0.6;       % hip to shoulder (torso length)
    
    % CoM locations along each link (m)
    l_Om1 = 0.45;      % leg CoM from ankle (halfway)
    l_Am2 = 0.30;      % torso CoM from hip (halfway)
    
    % Masses (kg)
    m1 = 15;           % leg segment
    m2 = 55;           % torso segment
    
    % Moments of inertia (kg*m^2)
    I1 = (1/12) * m1 * l_OA^2;    % 1.0125
    I2 = (1/12) * m2 * l_AB^2;    % 1.65
    
    % Gravity (m/s^2)
    g = 9.81;
    
    % Pack in same order as derive_DIP.m
    p = [l_OA; l_AB; l_Om1; l_Am2; m1; m2; I1; I2; g];
end
