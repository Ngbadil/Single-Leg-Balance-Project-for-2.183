clear all; close all; clc;
warning('off', 'MATLAB:singularMatrix');
warning('off', 'MATLAB:table:ModifiedAndSavedVarnames');

%% USER INPUT %%

% CoM_RATIO_RANGE = 0.1:0.1:5.0;
n_points = 100;
n_repeats = 50;
CoM_RATIO_RANGE   = logspace(-1,1,n_points);
START_ANGLE_RANGE = linspace(0,0.5,51);
JOINT_LIM_RANGE   = [linspace(-60*pi/180, 0, n_points); fliplr(linspace(1*pi/180, 250*pi/180, n_points))]';
% JOINT_LIM_RANGE = [linspace(-45*pi/180, 0, n_points); 125*pi/180*ones(1,npoints))]';

TEST_COM_headheight = true;
TEST_COM_fallrecovery = false;
TEST_JOINTLIM = false;

GROUP_NAMES = ["Young", "Old Low", "Old High"];
R_PRECOMPUTED = true;
% That's all. Run code for results. %

%% PRECOMPUTE R %%
if R_PRECOMPUTED == true
    % load('R_vals.mat', 'R');
    R = zeros(2,2,length(GROUP_NAMES));
    R(:,:,1) = 1e6 * diag([0.21, 1/0.21]);
    R(:,:,2) = 1e6 * diag([0.22, 1/0.22]);
    R(:,:,3) = 1e6 * diag([0.26, 1/0.26]);
else
    R = zeros(2,2,length(GROUP_NAMES));
    for g = 1:length(GROUP_NAMES)
        settings.group_name = GROUP_NAMES(g);
        fit_results = fitLQRzIPModel("processed_zIP_results_sagittal_0.5_BW_square_VAF5.mat", settings);
        R(:,:,g) = fit_results.group.R_best;
    end
    save("R_vals.mat", 'R');
end
% R = diag([1/50^2, 1/50^2]); % for speed

%% STANDING CoM HEIGHT PARAMETER SWEEP %%
if TEST_COM_headheight == true
    disp('Collecting data...');
    
    % Initialize data
    lowest_head_height = zeros(3, length(CoM_RATIO_RANGE));
    lowest_CoM_height  = zeros(3, length(CoM_RATIO_RANGE));
    foot_force_range_x = zeros(3, length(CoM_RATIO_RANGE));
    foot_force_range_y = zeros(3, length(CoM_RATIO_RANGE));
    
    for g=1:length(GROUP_NAMES)
        disp(['Simulating ', GROUP_NAMES(g), ' group...'])
        for i=1:length(CoM_RATIO_RANGE)
            disp(['SIM NUM ', num2str(i),'/',num2str(length(CoM_RATIO_RANGE)),': Currently simulating ', num2str(CoM_RATIO_RANGE(i)), ' lower-to-upper body mass ratio...'])
            
            hh = zeros(size(n_repeats));
            ch = zeros(size(n_repeats));
            ffx = zeros(size(n_repeats));
            ffy = zeros(size(n_repeats));
            for j=1:n_repeats
                % Simulate mass ratio
                [z,u,p] = simulate_DIP_CoM(CoM_RATIO_RANGE(i), [-20*pi/180, 125*pi/180], 'table_1_data_deidentified.csv',false, [0,0,0,0], R(:,:,g));
            
                % Get head and CoM position
                head_pos = head_pos_DIP(z,p);
                CoM_pos  =  CoM_pos_DIP(z,p);
            
                % Get min height
                hh(j) = (min(head_pos(2,:))-max(head_pos(2,:)))/max(head_pos(2,:));
                ch(j) = (min(CoM_pos(2,:))-max(CoM_pos(2,:)))/max(CoM_pos(2,:));
    
                % Get foot force 
                foot_force = F_O_DIP(z,u,p);
                foot_force_x = foot_force(1,:);
                foot_force_y = foot_force(2,:);
    
                % Get range of foot force in x and y direction
                ffx(j) = (max(foot_force_x)-min(foot_force_x));
                ffy(j) = (max(foot_force_y)-min(foot_force_y));
    
            end
            lowest_head_height(g,i) = mean(hh);
            lowest_CoM_height(g,i)  = mean(ch);
            foot_force_range_x(g,i) = mean(ffx);
            foot_force_range_y(g,i) = mean(ffy);
        end
    
    end
    
    disp('Data collection completed.');
    
    
    %% PLOT RESULTS %%
    disp('Plotting results...');
    
    figure (1); clf; hold on;
    for g = 1:length(GROUP_NAMES)
        semilogx(CoM_RATIO_RANGE, ...
            lowest_head_height(g,:), ...
            'LineWidth',3);
    end
    xlabel('Upper-to-Lower Body Ratio (-)')
    ylabel('Change in Head Height (rel. to standing, normalized) (m)')
    legend(GROUP_NAMES)
    xscale log
    savefig('CoMSweep_hh_nonoise.fig');
    saveas(gcf, 'CoMSweep_hh.png');
    % improvePlot;
    
    figure (2); clf; hold on;
    for g = 1:length(GROUP_NAMES)
        semilogx(CoM_RATIO_RANGE, ...
            lowest_CoM_height(g,:), ...
            'LineWidth',3);
    end
    xlabel('Upper-to-Lower Body Ratio (-)')
    ylabel('Change in CoM Height (rel. to standing, normalized) (m)')
    legend(GROUP_NAMES)
    xscale log
    savefig('CoMSweep_ch_nonoise.fig');
    saveas(gcf, 'CoMSweep_ch.png');
    % improvePlot;
    
    figure (3); clf; hold on;
    for g = 1:length(GROUP_NAMES)
        semilogx(CoM_RATIO_RANGE, ...
            foot_force_range_x(g,:), ...
            'LineWidth',3);
    end
    xlabel('Upper-to-Lower Body Ratio (-)')
    ylabel('Force Range in x (N)')
    legend(GROUP_NAMES)
    xscale log
    savefig('CoMSweep_ffx_nonoise.fig');
    saveas(gcf, 'CoMSweep_ffx.png');
    % improvePlot;
    
    figure (4); clf; hold on;
    for g = 1:length(GROUP_NAMES)
        semilogx(CoM_RATIO_RANGE, ...
            foot_force_range_y(g,:), ...
            'LineWidth',3);
    end
    xlabel('Upper-to-Lower Body Ratio (-)')
    ylabel('Force Range in y (N)')
    legend(GROUP_NAMES)
    xscale log
    savefig('CoMSweep_ffy_nonoise.fig');
    saveas(gcf, 'CoMSweep_ffy.png');
    % improvePlot;
    
    disp('Results plotted. This is such a Friendship Moment.');
end

%% STANDING CoM HEIGHT PARAMETER SWEEP %%
if TEST_COM_fallrecovery == true
    disp('Collecting data...');
    
    % Initialize data
    fall_point = zeros(3,length(CoM_RATIO_RANGE));
    
    for g=1:length(GROUP_NAMES)
        disp(['Simulating ', GROUP_NAMES(g), ' group...'])
        for i=1:length(CoM_RATIO_RANGE)
            disp(['SIM NUM ', num2str(i),'/',num2str(length(CoM_RATIO_RANGE)),': Currently simulating ', num2str(CoM_RATIO_RANGE(i)), ' lower-to-upper body mass ratio...'])
            fp = zeros(size(START_ANGLE_RANGE));
            for j=1:length(START_ANGLE_RANGE)
                disp(['     Simulating ', num2str(START_ANGLE_RANGE(j)),' start angle...'])
                for k = 1:5
                    % Simulate mass ratio and start angle
                    [z,u,p] = simulate_DIP_CoM(CoM_RATIO_RANGE(i), [-20*pi/180, 125*pi/180], 'table_1_data_deidentified.csv',false, [START_ANGLE_RANGE(j),0,0,0], R(:,:,g));
                    
                    hip_pos = hip_pos_DIP(z,p);
                    if min(hip_pos(2,:)) < 0.2*hip_pos_DIP([0,0,0,0],p)
                        fp(j) = fp(j) + 1;
                    end
                end
                if fp(j) > 5*0.8
                    fall_point(g,i) = START_ANGLE_RANGE(j);
                    break
                end
            end
        end
    end
    
    disp('Data collection completed.');
    
    
    %% PLOT RESULTS %%
    disp('Plotting results...');
    
    figure (5); clf; hold on;
    for g = 1:length(GROUP_NAMES)
        semilogx(CoM_RATIO_RANGE, ...
            fall_point(g,:), ...
            'LineWidth',3);
    end
    xlabel('Upper-to-Lower Body Ratio (-)')
    ylabel('Fall Recovery Failure Angle (rad)')
    legend(GROUP_NAMES)
    xscale log
    savefig('CoMSweep_frfa.fig');
    saveas(gcf, 'CoMSweep_frfa.png');
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