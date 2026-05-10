function all_results = runAllGroups_coarse(results_file)
% RUNALLGROUPS_COARSE  Run sweepArmAngles at GROUP LEVEL ONLY for every
%                      group, then overlay the results for cross-group
%                      comparison.
%
% Settings are aligned with the working struct that produced valid fits:
%   trial_type   = 'Long'
%   beta_vec     = 0.1:0.1:5
%   sigma_vec    = 0.001:0.05:0.5
%
% USAGE
%   all_results = runAllGroups_coarse()
%   all_results = runAllGroups_coarse('processed_zIP_results_sagittal_0.5_BW_square_VAF5.mat')
%
% OUTPUT  all_results  struct with one field per group, each containing
%                      the full sweep_results from sweepArmAngles.

if nargin < 1
    results_file = 'processed_zIP_results_sagittal_0.5_BW_square_VAF5.mat';
end

%% ── Configuration ──────────────────────────────────────────────────────────

groups        = {'Young', 'Old Low', 'Old High'};
trial_types   = {'Short', 'Long',    'Long'    };   % per-group trial type
theta_deg_vec = [30, 45, 60, 75, 90];

% Grids — exactly matching your working struct
sweep_beta_vec  = 0.1:0.1:5;       % 50 points
sweep_sigma_vec = 0.001:0.05:0.5;  % 11 points

fprintf('\n========================================\n');
fprintf('  GROUP-LEVEL SWEEP — ALL GROUPS\n');
fprintf('  Groups: %s\n', strjoin(groups, ', '));
fprintf('  Angles: %s deg\n', num2str(theta_deg_vec));
fprintf('  Beta:  %d points  |  Sigma: %d points\n', ...
    numel(sweep_beta_vec), numel(sweep_sigma_vec));
fprintf('  Trial type: Long\n');
fprintf('========================================\n');

all_results = struct();

%% ── Loop over groups ───────────────────────────────────────────────────────

for g = 1:numel(groups)

    group_name = groups{g};
    fprintf('\n\n##### Group %d/%d: %s #####\n', g, numel(groups), group_name);

    % Build settings struct — fields and values mirror the working struct
    s = struct();
    s.run_group_fit       = true;
    s.run_participant_fit = false;            % group-level only (speed)
    s.group_name          = group_name;
    s.trial_type          = trial_types{g};
    s.selected_leg        = 'both';
    s.apply_VAF_filter    = true;
    s.theta_rad           = pi/4;             % overwritten per angle inside sweep
    s.beta_vec            = sweep_beta_vec;
    s.sigma_vec           = sweep_sigma_vec;
    s.plot_results        = false;
    s.save_results        = false;
    s.print_results       = false;
    s.save_folder         = pwd;
    s.number_old_groups   = 2;
    s.theta_deg_vec       = theta_deg_vec;    % consumed by sweepArmAngles

    sweep = sweepArmAngles(results_file, s);

    % Save under a valid struct field name (no spaces)
    field = matlab.lang.makeValidName(group_name);
    all_results.(field) = sweep;

    % Brief summary print
    fprintf('\n  %s summary:\n', group_name);
    fprintf('    %-10s  %-10s  %-10s  %-10s\n', 'theta', 'beta', 'sigma', 'RMSE');
    for i = 1:numel(theta_deg_vec)
        fprintf('    %-10g  %-10.4f  %-10.4f  %-10.6f\n', ...
            theta_deg_vec(i), ...
            sweep.group.beta_best(i), ...
            sweep.group.sigma_best(i), ...
            sweep.group.rmse_best(i));
    end
end

fprintf('\n\n========================================\n');
fprintf('  ALL GROUPS COMPLETE — generating comparison plots\n');
fprintf('========================================\n\n');

%% ── Cross-group comparison plots ──────────────────────────────────────────

cmap    = lines(numel(groups));
markers = {'o', 's', '^', 'd'};

% ── Figure 1: Beta vs angle, all groups overlaid ──────────────────────────
figure('Name','All Groups: \beta vs Arm Angle','NumberTitle','off');
hold on;
for g = 1:numel(groups)
    field = matlab.lang.makeValidName(groups{g});
    plot(theta_deg_vec, all_results.(field).group.beta_best, ...
        'Color', cmap(g,:), 'Marker', markers{g}, ...
        'LineStyle', '-', 'LineWidth', 2.5, 'MarkerSize', 9, ...
        'MarkerFaceColor', cmap(g,:), ...
        'DisplayName', groups{g});
end
xlabel('Arm Elevation \theta (degrees)', 'FontSize', 12);
ylabel('\beta_{best}', 'FontSize', 12);
title('Group-Level \beta vs Arm Angle (All Groups)', 'FontSize', 13);
xticks(theta_deg_vec);
xticklabels(arrayfun(@(d) sprintf('%g°', d), theta_deg_vec, 'UniformOutput', false));
legend('Location', 'best', 'FontSize', 11);
grid on;
set(gca, 'FontSize', 11);

% ── Figure 2: Sigma vs angle, all groups overlaid ─────────────────────────
figure('Name','All Groups: \sigma_r vs Arm Angle','NumberTitle','off');
hold on;
for g = 1:numel(groups)
    field = matlab.lang.makeValidName(groups{g});
    plot(theta_deg_vec, all_results.(field).group.sigma_best, ...
        'Color', cmap(g,:), 'Marker', markers{g}, ...
        'LineStyle', '-', 'LineWidth', 2.5, 'MarkerSize', 9, ...
        'MarkerFaceColor', cmap(g,:), ...
        'DisplayName', groups{g});
end
xlabel('Arm Elevation \theta (degrees)', 'FontSize', 12);
ylabel('\sigma_{r,best}', 'FontSize', 12);
title('Group-Level \sigma_r vs Arm Angle (All Groups)', 'FontSize', 13);
xticks(theta_deg_vec);
xticklabels(arrayfun(@(d) sprintf('%g°', d), theta_deg_vec, 'UniformOutput', false));
legend('Location', 'best', 'FontSize', 11);
grid on;
set(gca, 'FontSize', 11);

% ── Figure 3: RMSE vs angle, all groups overlaid ──────────────────────────
figure('Name','All Groups: RMSE vs Arm Angle','NumberTitle','off');
hold on;
for g = 1:numel(groups)
    field = matlab.lang.makeValidName(groups{g});
    plot(theta_deg_vec, all_results.(field).group.rmse_best, ...
        'Color', cmap(g,:), 'Marker', markers{g}, ...
        'LineStyle', '-', 'LineWidth', 2.5, 'MarkerSize', 9, ...
        'MarkerFaceColor', cmap(g,:), ...
        'DisplayName', groups{g});
end
xlabel('Arm Elevation \theta (degrees)', 'FontSize', 12);
ylabel('RMSE_{best}', 'FontSize', 12);
title('Group-Level RMSE vs Arm Angle (All Groups)', 'FontSize', 13);
xticks(theta_deg_vec);
xticklabels(arrayfun(@(d) sprintf('%g°', d), theta_deg_vec, 'UniformOutput', false));
legend('Location', 'best', 'FontSize', 11);
grid on;
set(gca, 'FontSize', 11);

% ── Figure 4: zIP curves at theta = 45 deg, all groups ────────────────────
figure('Name','All Groups: zIP curves at \theta = 45°','NumberTitle','off');
hold on;

% Use 45 deg if present, else the closest angle
[~, idx_45] = min(abs(theta_deg_vec - 45));

for g = 1:numel(groups)
    field = matlab.lang.makeValidName(groups{g});
    sweep = all_results.(field);

    f_axis  = sweep.freq_bins(:);
    z_exp   = sweep.zip_exp(:);
    z_model = sweep.zip_model(:, idx_45);

    valid = ~isnan(f_axis) & ~isnan(z_exp);
    f_pl  = f_axis(valid);

    % Experimental: solid line
    plot(f_pl, z_exp(valid), '-', 'Color', cmap(g,:), 'LineWidth', 2, ...
        'Marker', markers{g}, 'MarkerSize', 5, 'MarkerFaceColor', cmap(g,:), ...
        'DisplayName', sprintf('%s — exp', groups{g}));

    % Model: dashed line
    plot(f_pl, z_model(valid), '--', 'Color', cmap(g,:), 'LineWidth', 1.8, ...
        'DisplayName', sprintf('%s — model (\\beta=%.2f)', ...
        groups{g}, sweep.group.beta_best(idx_45)));
end

yline(1, 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('z_{IP} / z_{CoM}', 'FontSize', 12);
title(sprintf('zIP Curves Across Groups at \\theta = %g°', ...
    theta_deg_vec(idx_45)), 'FontSize', 13);
legend('Location', 'best', 'FontSize', 9);
grid on;
xlim([0 8]);
set(gca, 'FontSize', 11);

% ── Figure 5: Combined summary panel ──────────────────────────────────────
figure('Name','All Groups: Summary Dashboard','NumberTitle','off', ...
    'Position', [100, 100, 1200, 400]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on;
for g = 1:numel(groups)
    field = matlab.lang.makeValidName(groups{g});
    plot(theta_deg_vec, all_results.(field).group.beta_best, ...
        'Color', cmap(g,:), 'Marker', markers{g}, 'LineWidth', 2, ...
        'MarkerSize', 7, 'MarkerFaceColor', cmap(g,:), ...
        'DisplayName', groups{g});
end
xlabel('\theta (deg)'); ylabel('\beta_{best}'); title('\beta vs Arm Angle');
xticks(theta_deg_vec); grid on; legend('Location','best','FontSize',8);

nexttile;
hold on;
for g = 1:numel(groups)
    field = matlab.lang.makeValidName(groups{g});
    plot(theta_deg_vec, all_results.(field).group.sigma_best, ...
        'Color', cmap(g,:), 'Marker', markers{g}, 'LineWidth', 2, ...
        'MarkerSize', 7, 'MarkerFaceColor', cmap(g,:), ...
        'DisplayName', groups{g});
end
xlabel('\theta (deg)'); ylabel('\sigma_{r,best}'); title('\sigma_r vs Arm Angle');
xticks(theta_deg_vec); grid on;

nexttile;
hold on;
for g = 1:numel(groups)
    field = matlab.lang.makeValidName(groups{g});
    plot(theta_deg_vec, all_results.(field).group.rmse_best, ...
        'Color', cmap(g,:), 'Marker', markers{g}, 'LineWidth', 2, ...
        'MarkerSize', 7, 'MarkerFaceColor', cmap(g,:), ...
        'DisplayName', groups{g});
end
xlabel('\theta (deg)'); ylabel('RMSE_{best}'); title('RMSE vs Arm Angle');
xticks(theta_deg_vec); grid on;

sgtitle('Cross-Group Comparison Summary', 'FontSize', 13);

%% ── Print final comparison table ──────────────────────────────────────────

fprintf('\n========================================\n');
fprintf('  CROSS-GROUP COMPARISON TABLE\n');
fprintf('========================================\n\n');

fprintf('%-12s', 'Group');
for i = 1:numel(theta_deg_vec)
    fprintf(' | %-7s', sprintf('%g°', theta_deg_vec(i)));
end
fprintf('\n%s\n', repmat('-', 1, 12 + 10*numel(theta_deg_vec)));

fprintf('Beta_best:\n');
for g = 1:numel(groups)
    field = matlab.lang.makeValidName(groups{g});
    fprintf('  %-10s', groups{g});
    for i = 1:numel(theta_deg_vec)
        fprintf(' | %-7.3f', all_results.(field).group.beta_best(i));
    end
    fprintf('\n');
end

fprintf('\nSigma_best:\n');
for g = 1:numel(groups)
    field = matlab.lang.makeValidName(groups{g});
    fprintf('  %-10s', groups{g});
    for i = 1:numel(theta_deg_vec)
        fprintf(' | %-7.4f', all_results.(field).group.sigma_best(i));
    end
    fprintf('\n');
end

fprintf('\nRMSE_best:\n');
for g = 1:numel(groups)
    field = matlab.lang.makeValidName(groups{g});
    fprintf('  %-10s', groups{g});
    for i = 1:numel(theta_deg_vec)
        fprintf(' | %-7.4f', all_results.(field).group.rmse_best(i));
    end
    fprintf('\n');
end

fprintf('\n');

end