function [zout, uout, p] = simulate_DIP_CoM(MASS_RATIO, JOINT_LIM, BASE_PARTICIPANT_DATA, PLOT_GRAPHS, z_start, R)
    
    % setpath    % add AutoDerived, Modeling, Visualization folders to Matlab path
    
    
    %% Gather information from participants
    
    participants = readtable(BASE_PARTICIPANT_DATA); %Import participant data using struct
    
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
        
        % MASS RATIO ADJUSTMENT %
        if MASS_RATIO ~= 0
            m = test.m1 + test.m2;
            test.m1 = m*          1/(1+MASS_RATIO);
            test.m2 = m* MASS_RATIO/(1+MASS_RATIO);
        end
        p(1) = test.L1; p(2)=test.L2;p(3) = test.c1; p(4)=test.c2; p(5)=test.m1; p(6)=test.m2; p(7)=test.j1; p(8)=test.j2;
    end
    
    z_eq = [0; 0; 0; 0];                % upright equilibrium [th1, th2, dth1, dth2]
    u_eq = [0; 0];
    % z0   = [0; 0; 0; 0];                % start upright
    % z0   = [0.1; 0.05; 0; 0];           % small perturbation from upright, at rest
    z0 = z_start;
    
    tf   = 50;                          % simulation duration (s)
    
    noise.motorNoiseLvL = 1;            % noise magnitude
    noise.motorNoiseRatio = 1.6;        % ankle:hip noise ratio
    noise.noise_type = 'l';             % 'w' for gaussian white noise, 'l' for low pass (more noticeable)
    
    %% Compute LQR Gain
    
    % Sanity check — should see two eigenvalues with positive real parts
    % disp('Open-loop eigenvalues:')
    % disp(eig(A_num))
    
    % controller.Q = diag([1/0.17^2, 1/0.17^2, 1/1^2, 1/1^2]);  % ~10 deg tilt, 1 rad/s
    % controller.R = diag([1/50^2, 1/50^2]);                      % 50 Nm max torque

    if nargin < 6  
        fit_results = fitLQRzIPModel("processed_zIP_results_sagittal_0.5_BW_square_VAF5.mat");
        controller.R = fit_results.group.R_best;
    else
        controller.R = R;
    end
    controller.Q = eye(4);
    
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
    % input_struct.joint = [-2*pi/3 2*pi/3];
    input_struct.joint = JOINT_LIM;
    input_struct.noise = noise;
    input_struct.controller = controller;
    
    %% Simulate
    % [tout, zout, uout] = simulate_DIP(z0, p, [0 tf], [-2*pi/3 2*pi/3], noise, controller, 1000);
    [tout, zout, uout] = simulate_DIP(input_struct);
    
    %% Plot joint angles
    if PLOT_GRAPHS
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
    end
end