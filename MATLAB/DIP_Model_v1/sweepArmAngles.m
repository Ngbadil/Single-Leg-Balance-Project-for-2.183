function sweep_results = sweepArmAngles(results_file, settings)
% SWEEPARMANGLES  Sweep arm elevation angle (theta) through [45, 60, 75, 90] deg
%                 and compare the effect on best-fit LQR parameters and zIP curves.
%
% The arm angle (theta_rad) appears in getLumpedParams_DIP_Ypose and controls
% how high the arms are raised during the Y-pose:
%   theta = 0        ->  T-pose  (arms horizontal)
%   theta = pi/4     ->  45 deg  (current default, ~Y-pose)
%   theta = pi/3     ->  60 deg
%   theta = 5*pi/12  ->  75 deg
%   theta = pi/2     ->  90 deg  (arms fully raised, I-pose top)
%
% Raising the arms shifts the upper-body CoM upward and changes the moment
% of inertia of link 2.  This modifies the lumped parameters fed to
% predictZIPfromModel and therefore changes what (beta, sigma) pair
% minimises RMSE against the experimental zIP data.
%
% USAGE
%   sweep_results = sweepArmAngles('processed_zIP_results_sagittal_0.5_BW_square_VAF5.mat')
%   sweep_results = sweepArmAngles(results_file, settings)   % pass custom settings
%
% The function passes all settings through to fitLQRzIPModel.
% Plotting is suppressed inside fitLQRzIPModel; this function produces its
% own comparison figures.
%
% OUTPUT  sweep_results  struct with fields:
%   .theta_deg        [1 x N] angles swept (degrees)
%   .theta_rad        [1 x N] angles swept (radians)
%   .group            struct array, one entry per angle:
%       .beta_best    best-fit beta
%       .sigma_best   best-fit sigma_r
%       .rmse_best    minimum group RMSE
%   .participant      cell array of per-participant tables (one per angle)
%   .freq_bins        frequency axis from the data file
%   .zip_exp          experimental group-average zIP curve
%   .zip_model        [N_freq x N_angles] model zIP curves at group-best params
%


%% ── Default settings ────────────────────────────────────────────────────────

if nargin < 2
    settings = struct();
end

% Angles to sweep  (degrees -> radians inside loop)
if ~isfield(settings, 'theta_deg_vec')
    settings.theta_deg_vec = [30, 45, 90];
end

% Pass-through defaults that fitLQRzIPModel would set anyway,
% but we override plot_results = false so we can make our own figures.
base_settings = struct();
base_settings.run_group_fit        = true;
base_settings.run_participant_fit  = true;
base_settings.group_name           = 'Old High';
base_settings.trial_type           = 'Long';
base_settings.selected_leg         = 'both';
base_settings.apply_VAF_filter     = true;
base_settings.beta_vec             = 0.1:0.1:5;
base_settings.sigma_vec            = 0.001:0.05:0.5;
base_settings.plot_results         = false;   % <-- suppressed; we plot below
base_settings.save_results         = true;
base_settings.print_results        = true;
base_settings.save_folder          = pwd;
base_settings.number_old_groups    = 2;

% Merge user-supplied settings over base defaults
user_fields = fieldnames(settings);
for k = 1:numel(user_fields)
    f = user_fields{k};
    if ~strcmp(f, 'theta_deg_vec')   % theta handled separately below
        base_settings.(f) = settings.(f);
    end
end

theta_deg_vec = settings.theta_deg_vec;
N_angles      = numel(theta_deg_vec);

%% ── Pre-allocate output ─────────────────────────────────────────────────────

sweep_results.theta_deg = theta_deg_vec;
sweep_results.theta_rad = deg2rad(theta_deg_vec);

group_beta  = nan(1, N_angles);
group_sigma = nan(1, N_angles);
group_rmse  = nan(1, N_angles);
pid_tables  = cell(1, N_angles);

zip_model_all = [];   % filled after first successful run

%% ── Main sweep loop ─────────────────────────────────────────────────────────

fprintf('\n========================================\n');
fprintf('  ARM ANGLE SWEEP\n');
fprintf('  Angles: %s degrees\n', num2str(theta_deg_vec));
fprintf('========================================\n\n');

for i = 1:N_angles

    theta_deg = theta_deg_vec(i);
    theta_rad = deg2rad(theta_deg);

    fprintf('\n--- Running theta = %g deg (%.4f rad) ---\n', theta_deg, theta_rad);

    % Build settings for this angle
    angle_settings          = base_settings;
    angle_settings.theta_rad = theta_rad;

    % Run the full group + participant fit
    fit = fitLQRzIPModel(results_file, angle_settings);

    % Store group-level results
    group_beta(i)  = fit.group.beta_best;
    group_sigma(i) = fit.group.sigma_best;
    group_rmse(i)  = fit.group.rmse_best;

    % Store participant-level results
    if isfield(fit, 'participant')
        pid_tables{i} = fit.participant.table;
    end

    % Compute model zIP curve at group-best params for this angle
    alpha = 1e6;
    ctrl.Q = eye(4);
    ctrl.R = alpha * diag([group_beta(i), 1/group_beta(i)]);

    f_axis = fit.freq_bins(:)';
    group_ids = fit.group_pids_used;

    model_curves_angle = [];
    for k = 1:numel(group_ids)
        pid_str = sprintf('P%02d', group_ids(k));
        if ~isfield(fit.participant_lumped_params, pid_str)
            continue
        end
        lp = fit.participant_lumped_params.(pid_str);

        inp.lumped_params     = lp;
        inp.controller_params = ctrl;
        inp.motorNoiseRatio   = group_sigma(i);
        inp.f                 = f_axis;

        [~, zip_m] = predictZIPfromModel(inp);
        model_curves_angle = [model_curves_angle, zip_m(:)];
    end

    zip_model_all(:, i) = mean(model_curves_angle, 2, 'omitnan'); %#ok<AGROW>

    % Store experimental curve from first run (same across all angles)
    if i == 1
        sweep_results.freq_bins = fit.freq_bins;
        zip_exp_raw = fit.group_trials;
        sweep_results.zip_exp = mean(zip_exp_raw, 2, 'omitnan');
    end

    fprintf('  -> beta = %.4f  |  sigma = %.4f  |  RMSE = %.6f\n', ...
        group_beta(i), group_sigma(i), group_rmse(i));
end

sweep_results.group.beta_best  = group_beta;
sweep_results.group.sigma_best = group_sigma;
sweep_results.group.rmse_best  = group_rmse;
sweep_results.participant      = pid_tables;
sweep_results.zip_model        = zip_model_all;

fprintf('\n========================================\n');
fprintf('  SWEEP COMPLETE\n');
fprintf('========================================\n\n');

%% Print summary table
fprintf('%-10s  %-10s  %-10s  %-10s\n', 'Theta(deg)', 'Beta_best', 'Sigma_best', 'RMSE_best');
fprintf('%s\n', repmat('-',1,46));
for i = 1:N_angles
    fprintf('%-10g  %-10.4f  %-10.4f  %-10.6f\n', ...
        theta_deg_vec(i), group_beta(i), group_sigma(i), group_rmse(i));
end
fprintf('\n');

%% ── Figures ─────────────────────────────────────────────────────────────────

f_axis   = sweep_results.freq_bins(:);
zip_exp  = sweep_results.zip_exp(:);
valid    = ~isnan(f_axis) & ~isnan(zip_exp);
f_plot   = f_axis(valid);
z_exp_pl = zip_exp(valid);

% Colour map: one colour per angle
cmap = lines(N_angles);

% ── Figure 1: Overlaid zIP curves ──────────────────────────────────────────
figure('Name','Arm Angle Sweep — zIP Curves','NumberTitle','off');
hold on;

% Experimental data
plot(f_plot, z_exp_pl, 'k-o', 'LineWidth', 2, 'MarkerSize', 4, ...
    'DisplayName', 'Experimental (group mean)');

% Model curves for each angle
for i = 1:N_angles
    z_m = zip_model_all(valid, i);
    plot(f_plot, z_m, '--', 'Color', cmap(i,:), 'LineWidth', 2, ...
        'DisplayName', sprintf('\\theta = %g° (\\beta=%.3f, \\sigma=%.3f)', ...
        theta_deg_vec(i), group_beta(i), group_sigma(i)));
end

yline(1, 'k:', 'LineWidth', 1.2);
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('z_{IP} / z_{CoM}', 'FontSize', 12);
title(sprintf('Effect of Arm Elevation on zIP Curve\n(%s — %s trials)', ...
    base_settings.group_name, base_settings.trial_type), 'FontSize', 13);
legend('Location', 'best', 'FontSize', 9);
grid on;
xlim([0 8]);
set(gca, 'FontSize', 11);

% ── Figure 2: Best-fit parameters vs angle ─────────────────────────────────
figure('Name','Arm Angle Sweep — Best-Fit Parameters','NumberTitle','off');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
bar(theta_deg_vec, group_beta, 'FaceColor', [0.25 0.55 0.87]);
xlabel('Arm Elevation \theta (degrees)', 'FontSize', 11);
ylabel('\beta_{best}', 'FontSize', 12);
title('Best-Fit \beta (ankle:hip torque weighting)', 'FontSize', 11);
xticks(theta_deg_vec);
xticklabels(arrayfun(@(d) sprintf('%g°', d), theta_deg_vec, 'UniformOutput', false));
grid on;

nexttile;
bar(theta_deg_vec, group_sigma, 'FaceColor', [0.93 0.53 0.18]);
xlabel('Arm Elevation \theta (degrees)', 'FontSize', 11);
ylabel('\sigma_{r,best}', 'FontSize', 12);
title('Best-Fit \sigma_r (motor noise ratio)', 'FontSize', 11);
xticks(theta_deg_vec);
xticklabels(arrayfun(@(d) sprintf('%g°', d), theta_deg_vec, 'UniformOutput', false));
grid on;

nexttile;
bar(theta_deg_vec, group_rmse, 'FaceColor', [0.47 0.67 0.19]);
xlabel('Arm Elevation \theta (degrees)', 'FontSize', 11);
ylabel('RMSE_{best}', 'FontSize', 12);
title('Best-Fit Group RMSE', 'FontSize', 11);
xticks(theta_deg_vec);
xticklabels(arrayfun(@(d) sprintf('%g°', d), theta_deg_vec, 'UniformOutput', false));
grid on;

sgtitle(sprintf('Group-Level Fit vs Arm Angle  (%s — %s)', ...
    base_settings.group_name, base_settings.trial_type), 'FontSize', 13);

% ── Figure 3: Participant-level beta scatter across angles ──────────────────
if ~isempty(pid_tables{1})
    figure('Name','Arm Angle Sweep — Participant \beta','NumberTitle','off');
    hold on;

    % Collect all participant IDs that appear in at least one run
    all_pids = [];
    for i = 1:N_angles
        if ~isempty(pid_tables{i})
            all_pids = [all_pids; pid_tables{i}.PID]; %#ok<AGROW>
        end
    end
    all_pids = unique(all_pids);
    pid_cmap = lines(numel(all_pids));

    for p = 1:numel(all_pids)
        pid = all_pids(p);
        betas_pid  = nan(1, N_angles);
        for i = 1:N_angles
            if isempty(pid_tables{i}), continue; end
            row = pid_tables{i}.PID == pid;
            if any(row)
                betas_pid(i) = pid_tables{i}.Beta_Best(row);
            end
        end
        plot(theta_deg_vec, betas_pid, 'o-', 'Color', pid_cmap(p,:), ...
            'LineWidth', 1.2, 'MarkerSize', 5, ...
            'DisplayName', sprintf('PID %02d', pid));
    end

    % Highlight group mean
    plot(theta_deg_vec, group_beta, 'k-^', 'LineWidth', 2.5, ...
        'MarkerSize', 8, 'DisplayName', 'Group mean');

    xlabel('Arm Elevation \theta (degrees)', 'FontSize', 11);
    ylabel('\beta_{best}', 'FontSize', 12);
    title(sprintf('Per-Participant \\beta vs Arm Angle\n(%s — %s)', ...
        base_settings.group_name, base_settings.trial_type), 'FontSize', 12);
    xticks(theta_deg_vec);
    xticklabels(arrayfun(@(d) sprintf('%g°', d), theta_deg_vec, 'UniformOutput', false));
    legend('Location', 'eastoutside', 'FontSize', 8);
    grid on;
    set(gca, 'FontSize', 11);

    % ── Figure 4: Participant-level RMSE scatter ───────────────────────────
    figure('Name','Arm Angle Sweep — Participant RMSE','NumberTitle','off');
    hold on;

    for p = 1:numel(all_pids)
        pid = all_pids(p);
        rmse_pid = nan(1, N_angles);
        for i = 1:N_angles
            if isempty(pid_tables{i}), continue; end
            row = pid_tables{i}.PID == pid;
            if any(row)
                rmse_pid(i) = pid_tables{i}.RMSE_Best(row);
            end
        end
        plot(theta_deg_vec, rmse_pid, 'o-', 'Color', pid_cmap(p,:), ...
            'LineWidth', 1.2, 'MarkerSize', 5, ...
            'DisplayName', sprintf('PID %02d', pid));
    end

    plot(theta_deg_vec, group_rmse, 'k-^', 'LineWidth', 2.5, ...
        'MarkerSize', 8, 'DisplayName', 'Group mean');

    xlabel('Arm Elevation \theta (degrees)', 'FontSize', 11);
    ylabel('RMSE_{best}', 'FontSize', 12);
    title(sprintf('Per-Participant RMSE vs Arm Angle\n(%s — %s)', ...
        base_settings.group_name, base_settings.trial_type), 'FontSize', 12);
    xticks(theta_deg_vec);
    xticklabels(arrayfun(@(d) sprintf('%g°', d), theta_deg_vec, 'UniformOutput', false));
    legend('Location', 'eastoutside', 'FontSize', 8);
    grid on;
    set(gca, 'FontSize', 11);
end

%% ── RMSE heatmap comparison ──────────────────────────────────────────────

% Load RMSE grids if available (they're stored inside fit_results from each
% call — re-run with a small custom call to retrieve them, or note that
% users can call fitLQRzIPModel manually and extract RMSE_grid).
fprintf('Tip: set settings.plot_results=true inside fitLQRzIPModel calls\n');
fprintf('     to also view the individual RMSE surfaces per angle.\n\n');

end
