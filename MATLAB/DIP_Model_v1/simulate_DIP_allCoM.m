clear all; close all; clc;
warning('off', 'MATLAB:singularMatrix');
warning('off', 'MATLAB:table:ModifiedAndSavedVarnames');

%% USER INPUT %%

% CoM_RATIO_RANGE = 0.1:0.1:5.0;
n_points = 50;
CoM_RATIO_RANGE = logspace(-1,1,n_points);
JOINT_LIM_RANGE = [linspace(-60*pi/180, 0, n_points); fliplr(linspace(1*pi/180, 250*pi/180, n_points))]';
% JOINT_LIM_RANGE = [linspace(-45*pi/180, 0, n_points); 125*pi/180*ones(1,npoints))]';

TEST_COM = true;
TEST_JOINTLIM = false;

% That's all. Run code for results. %

%% PRECOMPUTE R %%
fit_results = fitLQRzIPModel("processed_zIP_results_sagittal_0.5_BW_square_VAF5.mat");
R = fit_results.group.R_best;
% R = diag([1/50^2, 1/50^2]); % for speed

%% STANDING CoM HEIGHT PARAMETER SWEEP %%
if TEST_COM == true
    disp('Collecting data...');
    
    % Initialize data
    lowest_head_height = zeros(size(CoM_RATIO_RANGE));
    lowest_CoM_height  = zeros(size(CoM_RATIO_RANGE));
    
    for i=1:length(CoM_RATIO_RANGE)
        disp(['SIM NUM ', num2str(i),'/',num2str(length(CoM_RATIO_RANGE)),': Currently simulating ', num2str(CoM_RATIO_RANGE(i)), ' lower-to-upper body mass ratio...'])
        
        n = 10;
        hh = zeros(size(n));
        ch = zeros(size(n));
        for j=1:n
            % Simulate mass ratio
            [z,u,p] = simulate_DIP_CoM(CoM_RATIO_RANGE(i), [-20*pi/180, 125*pi/180], 'table_1_data_deidentified.csv',false, [0,0,0,0], R);
        
            % Get head and CoM position
            head_pos = head_pos_DIP(z,p);
            CoM_pos  =  CoM_pos_DIP(z,p);
        
            % Get min height
            hh(j) = (min(head_pos(2,:))-max(head_pos(2,:)))/max(head_pos(2,:));
            ch(j) = (min(CoM_pos(2,:))-max(CoM_pos(2,:)))/max(CoM_pos(2,:));
        end
        lowest_head_height(i) = mean(hh);
        lowest_CoM_height(i) = mean(ch);
    
    end
    
    disp('Data collection completed.');
    
    
    %% PLOT RESULTS %%
    disp('Plotting results...');
    
    figure (1); clf
    semilogx(CoM_RATIO_RANGE,lowest_head_height, LineWidth=3)
    xlabel('Upper-to-Lower Body Ratio (-)')
    ylabel('Change in Head Height (rel. to standing, normalized) (m)')
    % improvePlot;
    
    figure (2); clf
    semilogx(CoM_RATIO_RANGE,lowest_CoM_height, LineWidth=3)
    xlabel('Upper-to-Lower Body Ratio (-)')
    ylabel('Change in CoM Height (rel. to standing, normalized) (m)')
    % improvePlot;
    
    disp('Results plotted. This is such a Friendship Moment.');
end


%% JOINT_LIM PARAMETER SWEEP %%
if TEST_JOINTLIM == true
    disp('Collecting data...');
    
    % Initialize data
    lowest_head_height = zeros(size(CoM_RATIO_RANGE));
    lowest_CoM_height  = zeros(size(CoM_RATIO_RANGE));
    num_falls = zeros(size(CoM_RATIO_RANGE));
    mean_ankle_torque = zeros(size(CoM_RATIO_RANGE));
    mean_hip_torque   = zeros(size(CoM_RATIO_RANGE));
    
    for i=1:length(CoM_RATIO_RANGE)
        disp(['SIM NUM ', num2str(i),'/',num2str(length(CoM_RATIO_RANGE)),': Currently simulating ', num2str(JOINT_LIM_RANGE(i,1)), ' to ', num2str(JOINT_LIM_RANGE(i,2)), ' joint limits (range of ',  num2str(JOINT_LIM_RANGE(i,2)-JOINT_LIM_RANGE(i,1)), ')...'])
        
        n = 5;
        hh = zeros(size(n));
        ch = zeros(size(n));
        ankle_rms = zeros(size(n));
        hip_rms = zeros(size(n));
        for j=1:n
            % Simulate mass ratio
            [z,u,p] = simulate_DIP_CoM(0, JOINT_LIM_RANGE(i,:), 'table_1_data_deidentified.csv',false, [0,0,0,0], R);
        
            % Get head and CoM position
            head_pos = head_pos_DIP(z,p);
            CoM_pos  =  CoM_pos_DIP(z,p);
        
            % Get min height
            if min(CoM_pos(2,:))>max(CoM_pos(2,:))*0.1
                hh(j) = (min(head_pos(2,:))-max(head_pos(2,:)))/max(head_pos(2,:));
                ch(j) = (min(CoM_pos(2,:))-max(CoM_pos(2,:)))/max(CoM_pos(2,:));
                ankle_rms(j) = rms(u(1,:));
                hip_rms(j)   = rms(u(2,:));
            else
                num_falls(i) = num_falls(i)+1;
            end
        end
        lowest_head_height(i) = mean(hh);
        lowest_CoM_height(i) = mean(ch);
        mean_ankle_torque(i) = mean(ankle_rms(ankle_rms > 0)); 
        mean_hip_torque(i)   = mean(hip_rms(hip_rms > 0));
    
    end
    
    disp('Data collection completed.');
    
    
    %% PLOT RESULTS %%
    disp('Plotting results...');
    
    figure (1); clf
    plot(JOINT_LIM_RANGE(:,2)-JOINT_LIM_RANGE(:,1),lowest_head_height, LineWidth=3)
    xlabel('Joint Limit Range (rad)')
    ylabel('Change in Head Height (rel. to standing, normalized) (m)')
    % improvePlot;
    
    figure (2); clf
    plot(JOINT_LIM_RANGE(:,2)-JOINT_LIM_RANGE(:,1),lowest_CoM_height, LineWidth=3)
    xlabel('Joint Limit Range (rad)')
    ylabel('Change in CoM Height (rel. to standing, normalized) (m)')
    % improvePlot;
    
    figure (3); clf
    plot(JOINT_LIM_RANGE(:,2)-JOINT_LIM_RANGE(:,1),num_falls, LineWidth=3, Marker='o')
    xlabel('Joint Limit Range (rad)')
    ylabel('Number of Falls (-)')
    % improvePlot;

    figure(4); clf; hold on;
    plot(JOINT_LIM_RANGE(:,2)-JOINT_LIM_RANGE(:,1), mean_ankle_torque, 'LineWidth', 3, 'DisplayName', 'Ankle Torque (RMS)')
    plot(JOINT_LIM_RANGE(:,2)-JOINT_LIM_RANGE(:,1), mean_hip_torque, 'LineWidth', 3, 'DisplayName', 'Hip Torque (RMS)')
    
    xlabel('Joint Limit Range (rad)')
    ylabel('Average Control Effort (Nm)')
    title('Control Effort vs. Available Joint Range')
    legend('Location', 'best')
    grid on;
    % improvePlot;
    
    disp('Results plotted. This is such a Friendship Moment.');
end