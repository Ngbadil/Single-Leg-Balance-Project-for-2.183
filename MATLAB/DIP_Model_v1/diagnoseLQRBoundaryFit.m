function diag = diagnoseLQRBoundaryFit(results_file, settings)
%DIAGNOSELQRBOUNDARYFIT Diagnose boundary optima in fitLQRzIPModel.
%
% This intentionally does not change the fitter. It reuses the same data
% selection and scoring structure, then compares the normal model zIP
% against a sign-flipped model zIP to expose sign-convention problems.

if nargin < 1 || isempty(results_file)
    results_file = 'processed_zIP_results_sagittal_0.5_BW_square_VAF5.mat';
end

if nargin < 2
    settings = struct();
end

defaults.group_name = 'Old High';
defaults.trial_type = 'Long';
defaults.selected_leg = 'both';
defaults.apply_VAF_filter = true;
defaults.theta_rad = pi/2;
defaults.beta_vec = 0.1:0.1:5;
defaults.sigma_vec = 0.001:0.05:0.5;
defaults.number_old_groups = 2;
defaults.outlier_abs_threshold = 5;
defaults.noise_mode = 'normal'; % normal, swapped, or inverse_first

settings = mergeDefaults(settings, defaults);

loaded = load(results_file);
results = loaded.results;
freq_bins = loaded.freq_bins(:);
plane = loaded.plane;

if isfield(loaded, 'VAF_min_threshold')
    VAF_min_threshold = loaded.VAF_min_threshold;
else
    VAF_min_threshold = 80;
end

participant_table = readtable('table_1_data_deidentified.csv', ...
    'VariableNamingRule', 'preserve');

if strcmpi(plane, 'sagittal')
    plane_DIP = 'sgt';
elseif strcmpi(plane, 'frontal')
    plane_DIP = 'frt';
else
    error('Plane must be sagittal or frontal.');
end

participant_lumped_params = buildParticipantParams( ...
    participant_table, plane_DIP, settings.theta_rad);

group_ids = getGroupIds(settings.group_name, settings.number_old_groups);

[group_trials, group_pids_used, participant_trials] = collectTrials( ...
    results, group_ids, settings.trial_type, settings.selected_leg, ...
    settings.apply_VAF_filter, VAF_min_threshold);

zip_exp_mean = mean(group_trials, 2, 'omitnan');
zip_exp_median = median(group_trials, 2, 'omitnan');

fprintf('\n=== LQR Boundary Diagnostic ===\n');
fprintf('File: %s\n', results_file);
fprintf('Group/trial/leg: %s / %s / %s\n', ...
    settings.group_name, settings.trial_type, settings.selected_leg);
fprintf('theta_rad = %.6g | VAF threshold = %.3g\n', ...
    settings.theta_rad, VAF_min_threshold);
fprintf('beta range = %.4g..%.4g (%d values)\n', ...
    settings.beta_vec(1), settings.beta_vec(end), numel(settings.beta_vec));
fprintf('sigma range = %.4g..%.4g (%d values)\n', ...
    settings.sigma_vec(1), settings.sigma_vec(end), numel(settings.sigma_vec));
fprintf('noise_mode = %s\n', settings.noise_mode);
fprintf('Collected %d trials from %d participants.\n', ...
    size(group_trials, 2), numel(unique(group_pids_used)));
fprintf('Valid zIP points after VAF filter: %d / %d (%.1f%%)\n', ...
    sum(~isnan(group_trials(:))), numel(group_trials), ...
    100 * sum(~isnan(group_trials(:))) / numel(group_trials));
fprintf('Group mean zIP first/mid/last = %.5g / %.5g / %.5g\n', ...
    zip_exp_mean(1), zip_exp_mean(round(end/2)), zip_exp_mean(end));
fprintf('Group median zIP first/mid/last = %.5g / %.5g / %.5g\n', ...
    zip_exp_median(1), zip_exp_median(round(end/2)), zip_exp_median(end));

printOutliers(results, group_ids, settings, VAF_min_threshold, freq_bins);

diag = struct();
diag.settings = settings;
diag.freq_bins = freq_bins;
diag.group_trials = group_trials;
diag.group_pids_used = group_pids_used;
diag.zip_exp_mean = zip_exp_mean;
diag.zip_exp_median = zip_exp_median;

fprintf('\nScoring normal model sign...\n');
diag.normal = scoreGrid(1, results, group_ids, participant_lumped_params, ...
    participant_trials, freq_bins, settings);
printGridSummary('normal sign', diag.normal, settings);

fprintf('\nScoring sign-flipped model zIP...\n');
diag.flipped = scoreGrid(-1, results, group_ids, participant_lumped_params, ...
    participant_trials, freq_bins, settings);
printGridSummary('flipped sign', diag.flipped, settings);

end

function settings = mergeDefaults(settings, defaults)
fields = fieldnames(defaults);
for i = 1:numel(fields)
    f = fields{i};
    if ~isfield(settings, f)
        settings.(f) = defaults.(f);
    end
end
end

function group_ids = getGroupIds(group_name, number_old_groups)
young_low = 1:29;
old = 30:60;
if number_old_groups == 2
    old_low = [45, 49, 50, 31, 54, 30, 47, 32, 53];
    old_med = [];
    old_high = [52, 56, 55, 36, 51, 43, 35, 39];
else
    old_low = [45, 49, 50, 31, 54, 30, 47];
    old_med = [32, 53, 52, 56];
    old_high = [55, 36, 51, 43, 35, 39];
end

switch group_name
    case 'Young'
        group_ids = young_low;
    case 'Old'
        group_ids = old;
    case 'Old Low'
        group_ids = old_low;
    case 'Old Med'
        group_ids = old_med;
    case 'Old High'
        group_ids = old_high;
    otherwise
        error('Unknown group name: %s', group_name);
end
end

function participant_lumped_params = buildParticipantParams(participant_table, plane_DIP, theta_rad)
participant_lumped_params = struct();
for i = 1:height(participant_table)
    pid = participant_table.Participant_ID(i);
    totalHeight_m = participant_table{i, 'Height (in)'} * 0.0254;
    totalMass_kg = participant_table{i, 'Weight (lb)'} * 0.453592;
    gender_num = participant_table{i, 'Gender (1=M,2=F)'};

    if isnan(pid) || isnan(totalHeight_m) || isnan(totalMass_kg) || isnan(gender_num)
        continue
    end

    lp = getLumpedParams_DIP_Ypose(totalMass_kg, totalHeight_m, ...
        gender_num, plane_DIP, theta_rad);
    lp.L_COM = (lp.m1 * lp.c1 + lp.m2 * (lp.L1 + lp.c2)) / (lp.m1 + lp.m2);

    participant_lumped_params.(sprintf('P%02d', pid)) = lp;
end
end

function [group_trials, group_pids_used, participant_trials] = collectTrials( ...
    results, group_ids, trial_type, selected_leg, apply_VAF_filter, VAF_min_threshold)

group_trials = [];
group_pids_used = [];
participant_trials = struct();

for p = 1:numel(results)
    if isempty(results(p).Participant_ID)
        continue
    end

    pid = results(p).Participant_ID;
    if ~ismember(pid, group_ids)
        continue
    end

    [trials_pid, VAFs_pid] = collectParticipantTrials(results(p), trial_type, selected_leg);
    if isempty(trials_pid)
        continue
    end

    if apply_VAF_filter && ~isempty(VAFs_pid)
        trials_pid(VAFs_pid < VAF_min_threshold) = NaN;
    end

    group_trials = [group_trials, trials_pid]; %#ok<AGROW>
    group_pids_used(end + 1) = pid; %#ok<AGROW>
    participant_trials.(sprintf('P%02d', pid)) = trials_pid;
end
end

function [trials, VAFs] = collectParticipantTrials(result_pid, trial_type, selected_leg)
trials = [];
VAFs = [];
sides = {};
if strcmpi(selected_leg, 'L') || strcmpi(selected_leg, 'both')
    sides{end + 1} = 'Left'; %#ok<AGROW>
end
if strcmpi(selected_leg, 'R') || strcmpi(selected_leg, 'both')
    sides{end + 1} = 'Right'; %#ok<AGROW>
end

for i = 1:numel(sides)
    zip_field = [sides{i} '_' trial_type];
    vaf_field = ['VAF_' sides{i} '_' trial_type];
    if isfield(result_pid, zip_field) && isfield(result_pid, vaf_field)
        n = min(size(result_pid.(zip_field), 2), size(result_pid.(vaf_field), 2));
        if n > 0
            trials = [trials, result_pid.(zip_field)(:, 1:n)]; %#ok<AGROW>
            VAFs = [VAFs, result_pid.(vaf_field)(:, 1:n)]; %#ok<AGROW>
        end
    end
end
end

function printOutliers(results, group_ids, settings, VAF_min_threshold, freq_bins)
fprintf('\nSurviving zIP flags after VAF filter:\n');
found = false;
for p = 1:numel(results)
    if isempty(results(p).Participant_ID) || ~ismember(results(p).Participant_ID, group_ids)
        continue
    end

    sides = {'Left', 'Right'};
    for i = 1:numel(sides)
        zip_field = [sides{i} '_' settings.trial_type];
        vaf_field = ['VAF_' sides{i} '_' settings.trial_type];
        if ~isfield(results(p), zip_field) || ~isfield(results(p), vaf_field)
            continue
        end

        n = min(size(results(p).(zip_field), 2), size(results(p).(vaf_field), 2));
        Z = results(p).(zip_field)(:, 1:n);
        V = results(p).(vaf_field)(:, 1:n);
        mask = V >= VAF_min_threshold & ~isnan(Z) & ...
            (abs(Z) > settings.outlier_abs_threshold | Z < 0);
        [rows, cols] = find(mask);
        for k = 1:numel(rows)
            fprintf('PID %02d %-5s trial %2d f=%4.1f zIP=%9.4f VAF=%7.2f\n', ...
                results(p).Participant_ID, sides{i}, cols(k), freq_bins(rows(k)), ...
                Z(rows(k), cols(k)), V(rows(k), cols(k)));
            found = true;
        end
    end
end

if ~found
    fprintf('None found using abs(zIP) > %.3g or zIP < 0.\n', ...
        settings.outlier_abs_threshold);
end
end

function out = scoreGrid(model_sign, results, group_ids, participant_lumped_params, ...
    participant_trials, freq_bins, settings)

alpha = 1e6;
controller_params.Q = eye(4);
RMSE_grid = nan(numel(settings.beta_vec), numel(settings.sigma_vec));

for iB = 1:numel(settings.beta_vec)
    beta = settings.beta_vec(iB);
    controller_params.R = alpha * diag([beta, 1 / beta]);

    for iS = 1:numel(settings.sigma_vec)
        sigma_r = settings.sigma_vec(iS);
        sq_err_all = [];

        for p = 1:numel(results)
            if isempty(results(p).Participant_ID)
                continue
            end

            pid = results(p).Participant_ID;
            if ~ismember(pid, group_ids)
                continue
            end

            pid_str = sprintf('P%02d', pid);
            if ~isfield(participant_lumped_params, pid_str) || ...
                    ~isfield(participant_trials, pid_str)
                continue
            end

            input_struct.lumped_params = participant_lumped_params.(pid_str);
            input_struct.controller_params = controller_params;
            input_struct.motorNoiseRatio = sigma_r;
            input_struct.f = freq_bins(:)';

            if strcmpi(settings.noise_mode, 'normal')
                [~, zip_model_pid] = predictZIPfromModel(input_struct);
            else
                [~, zip_model_pid] = predictZIPForDiagnostic(input_struct, settings.noise_mode);
            end
            zip_model_pid = model_sign * zip_model_pid(:);
            trials_pid = participant_trials.(pid_str);

            for k = 1:size(trials_pid, 2)
                zip_trial = trials_pid(:, k);
                valid_idx = ~isnan(zip_trial) & ~isnan(zip_model_pid);
                if sum(valid_idx) < 3
                    continue
                end
                sq_err_all = [sq_err_all; ...
                    (zip_trial(valid_idx) - zip_model_pid(valid_idx)).^2]; %#ok<AGROW>
            end
        end

        if ~isempty(sq_err_all)
            RMSE_grid(iB, iS) = sqrt(mean(sq_err_all, 'omitnan'));
        end
    end
end

function [f, zIP_ratio] = predictZIPForDiagnostic(input_struct, noise_mode)
lumped_params = input_struct.lumped_params;
controller_params = input_struct.controller_params;
motorNoiseRatio = input_struct.motorNoiseRatio;
f = input_struct.f;
w = 2 * pi * f;

[A_CL, B_CL, C_CL, D_CL] = getModel_linDIP(lumped_params, controller_params);

s = tf('s');
H = C_CL * inv(s * eye(size(A_CL)) - A_CL) * B_CL + D_CL;

switch lower(noise_mode)
    case 'normal'
        Gww = [motorNoiseRatio^2 0; 0 1];
    case 'swapped'
        Gww = [1 0; 0 motorNoiseRatio^2];
    case 'inverse_first'
        Gww = [1 / motorNoiseRatio^2 0; 0 1];
    otherwise
        error('Unknown noise_mode: %s', noise_mode);
end

[H_mag, H_phase] = bode(H, w);
[HT_mag, HT_phase] = bode(Gww * (H.'), w);
HT_frf = HT_mag .* exp(1j * pi / 180 * HT_phase);
HC_frf = H_mag .* exp(-1j * pi / 180 * H_phase);

zIP_ratio = f * 0;
for i = 1:length(w)
    Gcpsd_frf = HC_frf(:, :, i) * HT_frf(:, :, i);
    [V, D] = eig(real(Gcpsd_frf));
    [~, i_max] = max(diag(abs(D)));
    zIP_ratio(i) = V(2, i_max) / V(1, i_max) / lumped_params.L_COM;
end
end

[rmse_best, idx_best] = min(RMSE_grid(:), [], 'omitnan');
[iB_best, iS_best] = ind2sub(size(RMSE_grid), idx_best);

out.RMSE_grid = RMSE_grid;
out.beta_best = settings.beta_vec(iB_best);
out.sigma_best = settings.sigma_vec(iS_best);
out.rmse_best = rmse_best;
out.iB_best = iB_best;
out.iS_best = iS_best;
end

function printGridSummary(label, out, settings)
fprintf('Best %s: beta=%.4g sigma=%.4g RMSE=%.6g idx=(%d,%d)\n', ...
    label, out.beta_best, out.sigma_best, out.rmse_best, ...
    out.iB_best, out.iS_best);
fprintf('  beta at edge? %d | sigma at low edge? %d | sigma at high edge? %d\n', ...
    out.iB_best == numel(settings.beta_vec), out.iS_best == 1, ...
    out.iS_best == numel(settings.sigma_vec));

col_min = min(out.RMSE_grid, [], 1, 'omitnan');
fprintf('  sigma column minima:');
for i = 1:numel(settings.sigma_vec)
    fprintf(' %.4g:%.4g', settings.sigma_vec(i), col_min(i));
end
fprintf('\n');
end
