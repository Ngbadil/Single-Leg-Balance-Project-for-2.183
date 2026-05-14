function fit_results = fitLQRzIPModel(results_file, settings)
% Group-level model fit from saved results file
% Evan Linton
% Original code: 10 April 2026 (saved as "getBestFitParamsEvan.m")
% Updated to make function: 01 May 2026
% Updated to allow for new data processing: 05 May 2026

%% === DEFAULT SETTINGS ===

if nargin < 2
    settings = struct();
end

defaults = struct();

% ---- Fit toggles ----
defaults.run_group_fit = true;
defaults.run_participant_fit = true;

% ---- Group selection ----
defaults.group_name = 'Old High'; %You can also have All for all groups, Old High, Old Low, Young 
defaults.trial_type = 'Short';
defaults.selected_leg = 'both';
defaults.apply_VAF_filter = true;

% ---- Model parameters ----
defaults.theta_rad = pi/4;

% ---- Search ranges ----
defaults.beta_vec = 0.05:0.01:0.5;
defaults.sigma_vec = 0.05:0.05:1.0;

% ---- Behavior toggles ----
defaults.plot_results = true;
defaults.save_results = true;
defaults.print_results = true;

% ---- Saving ----
defaults.save_folder = pwd;

% ---- Group structure ----
defaults.number_old_groups = 2;


%% === MERGE USER SETTINGS WITH DEFAULTS ===

fields = fieldnames(defaults);

for i = 1:length(fields)

    field = fields{i};

    if ~isfield(settings, field)

        settings.(field) = defaults.(field);

    end

end


%% === Unpack settings ===

run_group_fit = settings.run_group_fit;
run_participant_fit = settings.run_participant_fit;

group_name = settings.group_name;
trial_type = settings.trial_type;
selected_leg = settings.selected_leg;
apply_VAF_filter = settings.apply_VAF_filter;

theta_rad = settings.theta_rad;

beta_vec = settings.beta_vec;
sigma_vec = settings.sigma_vec;

plot_results = settings.plot_results;
save_results = settings.save_results;
print_results = settings.print_results;

number_old_groups = settings.number_old_groups;
save_folder = settings.save_folder;


%% === Load and merge data ===
if ischar(results_file) || isstring(results_file)
    results_files = cellstr(results_file);
elseif iscell(results_file)
    results_files = results_file;
else
    error('results_file must be a string or a cell array of strings.');
end

results = struct();
participant_table = table();
freq_bins = [];
plane = '';
VAF_min_threshold = [];

for rf = 1:length(results_files)

    loaded = load(results_files{rf});

    if ~isfield(loaded, 'results')
        error('File does not contain results: %s', results_files{rf});
    end

    % First file defines freq_bins and plane
    if isempty(freq_bins)
        freq_bins = loaded.freq_bins;
    else
        if length(freq_bins) ~= length(loaded.freq_bins) || any(freq_bins ~= loaded.freq_bins)
            error('Frequency bins do not match across files.');
        end
    end

    if isempty(plane)
        plane = loaded.plane;
    else
        if ~strcmpi(plane, loaded.plane)
            error('Plane does not match across files.');
        end
    end

    if isempty(VAF_min_threshold)
        if isfield(loaded, 'VAF_min_threshold')
            VAF_min_threshold = loaded.VAF_min_threshold;
        else
            VAF_min_threshold = 80;
        end
    end

    % Merge results by participant index
    results_this = loaded.results;

    for p = 1:length(results_this)

        if ~isfield(results_this(p), 'Participant_ID') || isempty(results_this(p).Participant_ID)
            continue
        end

        pid = results_this(p).Participant_ID;

        % Initialize if needed
        if length(results) < pid || ~isfield(results(pid), 'Participant_ID') || isempty(results(pid).Participant_ID)

            results(pid).Participant_ID = pid;

            results(pid).Left_Long = [];
            results(pid).Left_Short = [];
            results(pid).Right_Long = [];
            results(pid).Right_Short = [];

            results(pid).VAF_Left_Long = [];
            results(pid).VAF_Left_Short = [];
            results(pid).VAF_Right_Long = [];
            results(pid).VAF_Right_Short = [];
        end

        % Append fields
        trial_fields = {
            'Left_Long'
            'Left_Short'
            'Right_Long'
            'Right_Short'
            'VAF_Left_Long'
            'VAF_Left_Short'
            'VAF_Right_Long'
            'VAF_Right_Short'
        };

        for ff = 1:length(trial_fields)
            field = trial_fields{ff};

            if isfield(results_this(p), field) && ~isempty(results_this(p).(field))
                results(pid).(field) = [results(pid).(field), results_this(p).(field)];
            end
        end
    end

    % Collect participant metadata if saved in file
    if isfield(loaded, 'participant_table')
        participant_table = [participant_table; loaded.participant_table];
    end
end


%% === Add original dataset participant metadata ===
original_metadata_file = '';

candidate_metadata_files = {
    fullfile(pwd, 'table_1_data_deidentified.csv')
    fullfile(fileparts(results_files{1}), 'table_1_data_deidentified.csv')
    '/Users/evan/Documents/Research/Dan/table_1_data_deidentified.csv'
    '/Users/evan/Documents/MATLAB/Evan Yoga Data/table_1_data_deidentified.csv'
};

for c = 1:length(candidate_metadata_files)
    if isfile(candidate_metadata_files{c})
        original_metadata_file = candidate_metadata_files{c};
        break
    end
end

if isempty(original_metadata_file)
    [meta_file, meta_path] = uigetfile('*.csv', ...
        'Select original dataset metadata file');

    if isequal(meta_file,0)
        error('No original dataset metadata file selected.');
    end

    original_metadata_file = fullfile(meta_path, meta_file);
end

original_participant_table = readtable(original_metadata_file, ...
    'VariableNamingRule', 'preserve');

needed_vars = {'Participant_ID', 'Height (in)', 'Weight (lb)', 'Gender (1=M,2=F)'};

if ~isempty(participant_table)
    missing_project = setdiff(needed_vars, participant_table.Properties.VariableNames);
    if ~isempty(missing_project)
        error('Project metadata missing columns: %s', strjoin(missing_project, ', '));
    end
    participant_table = participant_table(:, needed_vars);
end

missing_original = setdiff(needed_vars, original_participant_table.Properties.VariableNames);
if ~isempty(missing_original)
    error('Original metadata missing columns: %s', strjoin(missing_original, ', '));
end

original_participant_table = original_participant_table(:, needed_vars);

participant_table = [participant_table; original_participant_table];
participant_table = unique(participant_table, 'rows');

if print_results
    fprintf('Loaded original metadata from:\n%s\n', original_metadata_file);
end


%% === Convert project participant IDs if needed ===

participant_id_map = containers.Map( ...
    {'FB','NB','AL','EL'}, ...
    [101, 102, 103, 104]);

if ~isnumeric(participant_table.Participant_ID)

    pid_num = nan(height(participant_table), 1);

    for i = 1:height(participant_table)
        label = char(string(participant_table.Participant_ID{i}));

        if isKey(participant_id_map, label)
            pid_num(i) = participant_id_map(label);
        else
            pid_num(i) = str2double(label);
        end
    end

    participant_table.Participant_ID = pid_num;
end


%% === Build participant-specific DIP lumped parameters ===

% Convert plane name to getLumpedParams_DIP_Ypose convention
if strcmpi(plane, 'sagittal')
    plane_DIP = 'sgt';
elseif strcmpi(plane, 'frontal')
    plane_DIP = 'frt';
else
    error('Plane must be sagittal or frontal.');
end

% Initialize storage
participant_lumped_params = struct();

for i = 1:height(participant_table)

    pid = participant_table.Participant_ID(i);

    % Pull participant metadata
    totalHeight_m = participant_table{i, 'Height (in)'} * 0.0254;
    totalMass_kg  = participant_table{i, 'Weight (lb)'} * 0.453592;
    gender_num    = participant_table{i, 'Gender (1=M,2=F)'};

    % Skip if missing
    if isnan(pid) || isnan(totalHeight_m) || isnan(totalMass_kg) || isnan(gender_num)
        continue
    end

    % Compute participant-specific lumped params
    lumped_params_pid = getLumpedParams_DIP_Ypose( ...
        totalMass_kg, totalHeight_m, gender_num, plane_DIP, theta_rad);

    % Add net COM height expected by predictZIPfromModel
    lumped_params_pid.L_COM = ...
        (lumped_params_pid.m1*lumped_params_pid.c1 + ...
         lumped_params_pid.m2*(lumped_params_pid.L1 + lumped_params_pid.c2)) / ...
        (lumped_params_pid.m1 + lumped_params_pid.m2);

    % Store as P01, P02, etc.
    pid_str = sprintf('P%02d', pid);
    participant_lumped_params.(pid_str) = lumped_params_pid;
end


%% Group definitions
young_original = 1:29;

% Project participants
FB = 101;  % young hypermobility
NB = 102;  % young healthy
AL = 103;  % young healthy
EL = 104;  % young healthy, if eventually collected

project_young_healthy = [NB, AL, EL];
project_young_hyper   = FB;

% Include project young participants in Young group
young_low = [young_original, project_young_healthy, project_young_hyper];

old = 30:60;

% Splits fall risk groups
if number_old_groups == 2
    old_low = [45, 49, 50, 31, 54, 30, 47, 32, 53];
    old_med = [];
    old_high = [52, 56, 55, 36, 51, 43, 35, 39];
else
    old_low = [45, 49, 50, 31, 54, 30, 47];
    old_med = [32, 53, 52, 56];
    old_high = [55, 36, 51, 43, 35, 39];
end


%% === Get participant IDs for selected group ===
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
    case 'All'
        group_ids = unique([young_low, old_low, old_med, old_high]);
    otherwise
        error('Unknown group name.');
end


%% === Debug: check which group participants have results and lumped params ===
if print_results
    fprintf('\n=== Group membership check: %s ===\n', group_name);

    for kk = 1:length(group_ids)
        pid = group_ids(kk);
        pid_str = sprintf('P%02d', pid);

        has_results = pid <= length(results) && ...
            isfield(results(pid), 'Participant_ID') && ...
            ~isempty(results(pid).Participant_ID);

        has_lumped = isfield(participant_lumped_params, pid_str);

        fprintf('PID %02d | has results = %d | has lumped params = %d\n', ...
            pid, has_results, has_lumped);
    end
end


%% Collect all trials for the chosen group
group_trials = [];
group_pids_used = [];

for p = 1:length(results)
    if isempty(results(p).Participant_ID)
        continue;
    end

    pid = results(p).Participant_ID;

    if ~ismember(pid, group_ids)
        continue;
    end

    trials = [];
    VAFs = [];

    if strcmpi(selected_leg, 'L') || strcmpi(selected_leg, 'both')
        zip_field = ['Left_' trial_type];
        vaf_field = ['VAF_Left_' trial_type];
        if isfield(results(p), zip_field) && isfield(results(p), vaf_field)
            n = min(size(results(p).(zip_field),2), size(results(p).(vaf_field),2));
            if n > 0
                trials = [trials, results(p).(zip_field)(:,1:n)];
                VAFs   = [VAFs,   results(p).(vaf_field)(:,1:n)];
            end
        end
    end

    if strcmpi(selected_leg, 'R') || strcmpi(selected_leg, 'both')
        zip_field = ['Right_' trial_type];
        vaf_field = ['VAF_Right_' trial_type];
        if isfield(results(p), zip_field) && isfield(results(p), vaf_field)
            n = min(size(results(p).(zip_field),2), size(results(p).(vaf_field),2));
            if n > 0
                trials = [trials, results(p).(zip_field)(:,1:n)];
                VAFs   = [VAFs,   results(p).(vaf_field)(:,1:n)];
            end
        end
    end

    if isempty(trials)
        continue;
    end

    if apply_VAF_filter && ~isempty(VAFs)
        trials(VAFs < VAF_min_threshold) = NaN;
    end

    group_trials = [group_trials, trials];
    group_pids_used(end+1) = pid;
end

fprintf('\nGroup %s diagnostics:\n', group_name);
fprintf('Total trials collected: %d\n', size(group_trials,2));
fprintf('Total zIP points: %d\n', numel(group_trials));
fprintf('Non-NaN zIP points: %d\n', sum(~isnan(group_trials(:))));
fprintf('Percent valid: %.1f%%\n', 100*sum(~isnan(group_trials(:))) / numel(group_trials));

if print_results
    fprintf('Collected %d trials from %d participants for %s %s.\n', ...
        size(group_trials,2), numel(unique(group_pids_used)), group_name, trial_type);
end

if isempty(group_trials)
    error('No trials collected. Check group_name, trial_type, selected_leg, and results fields.');
end


% %% === Build group-average lumped parameters ===
% 
% group_pids_used = unique(group_pids_used);
% 
% group_lumped_params = struct();
% 
% fields = {'m1','m2','c1','c2','j1','j2','L1','L2','L_COM'};
% 
% for f = 1:length(fields)
%     vals = [];
% 
%     for k = 1:length(group_pids_used)
%         pid_str = sprintf('P%02d', group_pids_used(k));
% 
%         if isfield(participant_lumped_params, pid_str)
%             vals(end+1) = participant_lumped_params.(pid_str).(fields{f});
%         end
%     end
% 
%     group_lumped_params.(fields{f}) = mean(vals, 'omitnan');
% end


%% Experimental curve to fit
zip_exp = nanmean(group_trials, 2);
f_exp = freq_bins(:);

valid_idx = ~isnan(f_exp) & ~isnan(zip_exp);
f_fit = f_exp(valid_idx);
zip_fit = zip_exp(valid_idx);


%% Fixed controller assumptions
alpha = 1e6; % fixed alpha
controller_params.Q = eye(4); % Q is just indentity matrix

% NOTE:
% Alpha and Q are intentionally fixed in the current framework
% However, they could be changed to a user input in the future


%% === GROUP-LEVEL FIT using participant-specific lumped params ===

if run_group_fit

    RMSE_group = nan(length(beta_vec), length(sigma_vec));

    % ---- Progress bar setup ----
    total_steps = length(beta_vec) * length(sigma_vec);
    step_counter = 0;
    h_wait = waitbar(0, 'Running group-level parameter search...');
    
    for iB = 1:length(beta_vec)
        beta = beta_vec(iB);

        for iS = 1:length(sigma_vec)
            sigma_r = sigma_vec(iS);

            controller_params.R = alpha * diag([beta, 1/beta]);

            sq_err_all = [];

            for p = 1:length(results)

                if isempty(results(p).Participant_ID)
                    continue
                end

                pid = results(p).Participant_ID;

                if ~ismember(pid, group_ids)
                    continue
                end

                pid_str = sprintf('P%02d', pid);

                if ~isfield(participant_lumped_params, pid_str)
                    continue
                end

                lumped_params_pid = participant_lumped_params.(pid_str);

                % ---- collect this participant's trials ----
                trials_pid = [];
                VAFs_pid = [];

                if strcmpi(selected_leg, 'L') || strcmpi(selected_leg, 'both')
                    zip_field = ['Left_' trial_type];
                    vaf_field = ['VAF_Left_' trial_type];

                    if isfield(results(p), zip_field) && isfield(results(p), vaf_field)
                        n = min(size(results(p).(zip_field),2), size(results(p).(vaf_field),2));
                        if n > 0
                            trials_pid = [trials_pid, results(p).(zip_field)(:,1:n)];
                            VAFs_pid   = [VAFs_pid,   results(p).(vaf_field)(:,1:n)];
                        end
                    end
                end

                if strcmpi(selected_leg, 'R') || strcmpi(selected_leg, 'both')
                    zip_field = ['Right_' trial_type];
                    vaf_field = ['VAF_Right_' trial_type];

                    if isfield(results(p), zip_field) && isfield(results(p), vaf_field)
                        n = min(size(results(p).(zip_field),2), size(results(p).(vaf_field),2));
                        if n > 0
                            trials_pid = [trials_pid, results(p).(zip_field)(:,1:n)];
                            VAFs_pid   = [VAFs_pid,   results(p).(vaf_field)(:,1:n)];
                        end
                    end
                end

                if isempty(trials_pid)
                    continue
                end

                if apply_VAF_filter && ~isempty(VAFs_pid)
                    trials_pid(VAFs_pid < VAF_min_threshold) = NaN;
                end

                % ---- model prediction using this participant's body params ----
                input_struct.lumped_params = lumped_params_pid;
                input_struct.controller_params = controller_params;
                input_struct.motorNoiseRatio = sigma_r;
                input_struct.f = freq_bins(:)';

                [~, zip_model_pid] = predictZIPfromModel(input_struct);
                zip_model_pid = zip_model_pid(:);

                % ---- compare model to each trial ----
                for k = 1:size(trials_pid,2)
                    zip_trial = trials_pid(:,k);

                    valid_idx = ~isnan(zip_trial) & ~isnan(zip_model_pid);

                    if sum(valid_idx) < 3
                        continue
                    end

                    sq_err_all = [sq_err_all; ...
                        (zip_trial(valid_idx) - zip_model_pid(valid_idx)).^2];
                end
            end

            if isempty(sq_err_all) || all(isnan(sq_err_all))
                RMSE_group(iB,iS) = NaN;
            else
                RMSE_group(iB,iS) = sqrt(mean(sq_err_all, 'omitnan'));
            end

            % ---- Update progress bar ----
            step_counter = step_counter + 1;
            waitbar(step_counter / total_steps, h_wait, ...
                sprintf('Group fit progress: %d / %d', ...
                step_counter, total_steps));
        end
    end

    % Best group fit
    [group_min_rmse, idx_best] = min(RMSE_group(:), [], 'omitnan');
    
    if isempty(group_min_rmse) || isnan(group_min_rmse)
        warning('No valid RMSE values found for group: %s', group_name);
        beta_group_best = NaN;
        sigma_group_best = NaN;
    else
        [iB_best, iS_best] = ind2sub(size(RMSE_group), idx_best);
        beta_group_best = beta_vec(iB_best);
        sigma_group_best = sigma_vec(iS_best);
    end

    if print_results
        fprintf('\n=== GROUP-LEVEL FIT ===\n');
        fprintf('Group: %s\n', group_name);
        fprintf('Best beta = %.4f\n', beta_group_best);
        fprintf('Best sigma_r = %.4f\n', sigma_group_best);
        fprintf('Best RMSE = %.6f\n', group_min_rmse);
    end

    close(h_wait);

end


%% === PARTICIPANT-LEVEL FITS ===

if run_participant_fit

    participant_fit_results = table();
    row_count = 0;

    % Participants that should be fit
    fit_pids = [];
    for p = 1:length(results)
        if ~isempty(results(p).Participant_ID) && ismember(results(p).Participant_ID, group_ids)
            fit_pids(end+1) = results(p).Participant_ID;
        end
    end
    fit_pids = unique(fit_pids);

    % Progress bar
    h_wait_pid = waitbar(0, 'Running participant-level fits...');
    pid_counter = 0;
    total_pids = length(fit_pids);

    for p = 1:length(results)

        if isempty(results(p).Participant_ID)
            continue
        end

        pid = results(p).Participant_ID;

        if ~ismember(pid, group_ids)
            continue
        end

        pid_counter = pid_counter + 1;
        waitbar(pid_counter / total_pids, h_wait_pid, ...
            sprintf('Fitting PID %02d (%d / %d)', pid, pid_counter, total_pids));

        pid_str = sprintf('P%02d', pid);

        if ~isfield(participant_lumped_params, pid_str)
            continue
        end

        lumped_params_pid = participant_lumped_params.(pid_str);

        % ---- collect this participant's trials ----
        trials_pid = [];
        VAFs_pid = [];

        if strcmpi(selected_leg, 'L') || strcmpi(selected_leg, 'both')
            zip_field = ['Left_' trial_type];
            vaf_field = ['VAF_Left_' trial_type];

            if isfield(results(p), zip_field) && isfield(results(p), vaf_field)
                n = min(size(results(p).(zip_field),2), size(results(p).(vaf_field),2));
                if n > 0
                    trials_pid = [trials_pid, results(p).(zip_field)(:,1:n)];
                    VAFs_pid   = [VAFs_pid,   results(p).(vaf_field)(:,1:n)];
                end
            end
        end

        if strcmpi(selected_leg, 'R') || strcmpi(selected_leg, 'both')
            zip_field = ['Right_' trial_type];
            vaf_field = ['VAF_Right_' trial_type];

            if isfield(results(p), zip_field) && isfield(results(p), vaf_field)
                n = min(size(results(p).(zip_field),2), size(results(p).(vaf_field),2));
                if n > 0
                    trials_pid = [trials_pid, results(p).(zip_field)(:,1:n)];
                    VAFs_pid   = [VAFs_pid,   results(p).(vaf_field)(:,1:n)];
                end
            end
        end

        if isempty(trials_pid)
            continue
        end

        if apply_VAF_filter && ~isempty(VAFs_pid)
            trials_pid(VAFs_pid < VAF_min_threshold) = NaN;
        end

        % Initializes RSME_pid for each participant
        RMSE_pid = nan(length(beta_vec), length(sigma_vec));

        % ---- participant-specific grid search ----
        for iB = 1:length(beta_vec)
            beta = beta_vec(iB);

            for iS = 1:length(sigma_vec)
                sigma_r = sigma_vec(iS);

                controller_params.R = alpha * diag([beta, 1/beta]);

                input_struct.lumped_params = lumped_params_pid;
                input_struct.controller_params = controller_params;
                input_struct.motorNoiseRatio = sigma_r;
                input_struct.f = freq_bins(:)';

                [~, zip_model_pid] = predictZIPfromModel(input_struct);
                zip_model_pid = zip_model_pid(:);

                if all(isnan(zip_model_pid))
                    fprintf('PID %02d model is all NaN for beta %.3f, sigma %.3f\n', pid, beta, sigma_r);
                end

                sq_err_all = [];

                for k = 1:size(trials_pid,2)
                    zip_trial = trials_pid(:,k);

                    valid_idx = ~isnan(zip_trial) & ~isnan(zip_model_pid);

                    if sum(valid_idx) < 3
                        continue
                    end

                    sq_err_all = [sq_err_all; ...
                        (zip_trial(valid_idx) - zip_model_pid(valid_idx)).^2];
                end

                if isempty(sq_err_all) || all(isnan(sq_err_all))
                    RMSE_pid(iB,iS) = NaN;
                else
                    RMSE_pid(iB,iS) = sqrt(mean(sq_err_all, 'omitnan'));
                end
            end
        end

        [pid_min_rmse, idx_best] = min(RMSE_pid(:), [], 'omitnan');
        if isempty(pid_min_rmse) || isnan(pid_min_rmse)
            warning('No valid participant fit for PID %02d', pid);
            continue
        end
        [iB_best, iS_best] = ind2sub(size(RMSE_pid), idx_best);

        beta_pid_best = beta_vec(iB_best);
        sigma_pid_best = sigma_vec(iS_best);

        if ismember(pid, young_low)
            participant_group = "Young";
        elseif ismember(pid, old_low)
            participant_group = "Old Low";
        elseif ismember(pid, old_med)
            participant_group = "Old Med";
        elseif ismember(pid, old_high)
            participant_group = "Old High";
        else
            participant_group = "Unknown";
        end
        
        if pid == 101
            participant_subgroup = "2.183 Dataset - Young Hypermobile";
        elseif ismember(pid, [102 103 104])
            participant_subgroup = "2.183 Dataset - Young Healthy";
        else
            participant_subgroup = "Original Dataset";
        end

        row_count = row_count + 1;

        participant_fit_results.PID(row_count,1) = pid;
        participant_fit_results.Beta_Best(row_count,1) = beta_pid_best;
        participant_fit_results.SigmaR_Best(row_count,1) = sigma_pid_best;
        participant_fit_results.RMSE_Best(row_count,1) = pid_min_rmse;
        participant_fit_results.N_Trials(row_count,1) = size(trials_pid,2);
        participant_fit_results.R_Best{row_count,1} = alpha * diag([beta_pid_best, 1/beta_pid_best]);
        participant_fit_results.Group(row_count,1) = participant_group;
        participant_fit_results.Subgroup(row_count,1) = participant_subgroup;

        if print_results
            fprintf('\nPID %02d\n', pid);
            fprintf('Best beta = %.4f\n', beta_pid_best);
            fprintf('Best sigma_r = %.4f\n', sigma_pid_best);
            fprintf('Best RMSE = %.6f\n', pid_min_rmse);
        end
    end

    close(h_wait_pid);

    if print_results && run_participant_fit
        disp(participant_fit_results)
    end

end


%% === Pack outputs ===

fit_results = struct();

fit_results.settings = settings;
fit_results.plane = plane;
fit_results.group_name = group_name;
fit_results.trial_type = trial_type;
fit_results.selected_leg = selected_leg;
fit_results.freq_bins = freq_bins;
fit_results.group_pids_used = group_pids_used;
fit_results.group_trials = group_trials;
fit_results.participant_lumped_params = participant_lumped_params;
fit_results.results = results;
fit_results.VAF_min_threshold = VAF_min_threshold;

if run_group_fit
    fit_results.group.beta_best = beta_group_best;
    fit_results.group.sigma_best = sigma_group_best;
    fit_results.group.rmse_best = group_min_rmse;
    fit_results.group.R_best = alpha * diag([beta_group_best, 1/beta_group_best]);
    fit_results.group.RMSE_grid = RMSE_group;
end

if run_participant_fit
    fit_results.participant.table = participant_fit_results;
end


%% === SAVE MODELING RESULTS ===

if save_results

    if ~exist(save_folder, 'dir')
        mkdir(save_folder)
    end

    date_str = char(datetime('now','Format','yyyy-MM-dd_HH-mm'));

    save_filename = sprintf( ...
        'LQR_fit_results_%s_%s_%s_%s.mat', ...
        plane, group_name, trial_type, date_str);

    save_path = fullfile(save_folder, save_filename);

    save(save_path, 'fit_results');

    if print_results
        fprintf('\nSaved modeling results to:\n%s\n', save_path);
    end
end


%% === Plot participant-level fits ===
if plot_results && run_participant_fit && ~isempty(participant_fit_results)

    for r = 1:height(participant_fit_results)

        pid = participant_fit_results.PID(r);
        pid_str = sprintf('P%02d', pid);

        beta_pid_best  = participant_fit_results.Beta_Best(r);
        sigma_pid_best = participant_fit_results.SigmaR_Best(r);

        lumped_params_pid = participant_lumped_params.(pid_str);

        % Re-collect this participant's trials
        p_idx = find(arrayfun(@(s) ~isempty(s.Participant_ID) && s.Participant_ID == pid, results), 1);

        trials_pid = [];
        VAFs_pid = [];

        if strcmpi(selected_leg, 'L') || strcmpi(selected_leg, 'both')
            zip_field = ['Left_' trial_type];
            vaf_field = ['VAF_Left_' trial_type];

            n = min(size(results(p_idx).(zip_field),2), size(results(p_idx).(vaf_field),2));
            if n > 0
                trials_pid = [trials_pid, results(p_idx).(zip_field)(:,1:n)];
                VAFs_pid   = [VAFs_pid,   results(p_idx).(vaf_field)(:,1:n)];
            end
        end

        if strcmpi(selected_leg, 'R') || strcmpi(selected_leg, 'both')
            zip_field = ['Right_' trial_type];
            vaf_field = ['VAF_Right_' trial_type];

            n = min(size(results(p_idx).(zip_field),2), size(results(p_idx).(vaf_field),2));
            if n > 0
                trials_pid = [trials_pid, results(p_idx).(zip_field)(:,1:n)];
                VAFs_pid   = [VAFs_pid,   results(p_idx).(vaf_field)(:,1:n)];
            end
        end

        if apply_VAF_filter && ~isempty(VAFs_pid)
            trials_pid(VAFs_pid < VAF_min_threshold) = NaN;
        end

        zip_pid_mean = nanmean(trials_pid, 2);

        f_exp = freq_bins(:);
        valid_idx = ~isnan(f_exp) & ~isnan(zip_pid_mean);
        f_fit_pid = f_exp(valid_idx);
        zip_fit_pid = zip_pid_mean(valid_idx);

        % Recompute model using participant best-fit params
        controller_params.R = alpha * diag([beta_pid_best, 1/beta_pid_best]);

        input_struct.lumped_params = lumped_params_pid;
        input_struct.controller_params = controller_params;
        input_struct.motorNoiseRatio = sigma_pid_best;
        input_struct.f = f_fit_pid';

        [~, zip_model_pid] = predictZIPfromModel(input_struct);
        zip_model_pid = zip_model_pid(:);

        % Plot
        figure; hold on;
        plot(f_fit_pid, zip_fit_pid, 'ko-', 'LineWidth', 1.5, ...
            'DisplayName', sprintf('PID %02d experimental', pid));

        plot(f_fit_pid, zip_model_pid, 'r--', 'LineWidth', 2, ...
            'DisplayName', sprintf('\\beta = %.3f, \\sigma_r = %.3f', ...
            beta_pid_best, sigma_pid_best));

        yline(1, 'k:');
        xlabel('Frequency (Hz)');
        ylabel('z_{IP}/z_{CoM}');
        title(sprintf('Participant-level fit: PID %02d (%s %s)', ...
            pid, group_name, trial_type));
        legend('Location', 'best');
        grid on;
        xlim([0 8]);
    end
end

if print_results && run_participant_fit
    disp(participant_fit_results)
end


%% === Plot group-level fit ===

if plot_results && run_group_fit

    % Experimental group mean curve
    zip_exp = nanmean(group_trials, 2);
    f_exp = freq_bins(:);

    valid_idx = ~isnan(f_exp) & ~isnan(zip_exp);
    f_fit = f_exp(valid_idx);
    zip_fit = zip_exp(valid_idx);

    % Compute model curve for each participant using group-best controller
    model_curves = [];

    controller_params.R = alpha * diag([beta_group_best, 1/beta_group_best]);

    for k = 1:length(group_ids)

        pid = group_ids(k);
        pid_str = sprintf('P%02d', pid);

        if ~isfield(participant_lumped_params, pid_str)
            fprintf('Missing lumped params for PID %02d\n', pid);
            continue
        end

        input_struct.lumped_params = participant_lumped_params.(pid_str);
        input_struct.controller_params = controller_params;
        input_struct.motorNoiseRatio = sigma_group_best;
        input_struct.f = f_fit';

        [~, zip_model_pid] = predictZIPfromModel(input_struct);

        model_curves = [model_curves, zip_model_pid(:)];
    end

    zip_model_group_mean = mean(model_curves, 2, 'omitnan');

    figure; hold on;
    plot(f_fit, zip_fit, 'ko-', 'LineWidth', 1.5, ...
        'DisplayName', [group_name ' experimental']);
    plot(f_fit, zip_model_group_mean, 'r--', 'LineWidth', 2, ...
        'DisplayName', sprintf('Model fit: \\beta = %.3f, \\sigma_r = %.3f', ...
        beta_group_best, sigma_group_best));

    yline(1, 'k:');
    xlabel('Frequency (Hz)');
    ylabel('z_{IP}/z_{CoM}');
    title(['Group-level fit: ' group_name]);
    legend('Location', 'best');
    grid on;
    xlim([0 8]);

end


%% Plot RMSE surface
if plot_results && run_group_fit
    figure;
    imagesc(sigma_vec, beta_vec, RMSE_group);
    set(gca, 'YDir', 'normal');
    xlabel('\sigma_r');
    ylabel('\beta');
    title(['Group RMSE Surface: ' group_name]);
    colorbar;
    hold on;
    plot(sigma_group_best, beta_group_best, 'wo', ...
        'MarkerFaceColor', 'w', 'MarkerSize', 8);
end


%% === Bubble violin plots of participant best-fit beta and sigma ==
if plot_results && run_participant_fit && ~isempty(participant_fit_results)
    plotBestFitParamViolins(participant_fit_results, young_low, old_low, old_high);
end
end


%% Helper function for violin plotting
function plotBestFitParamViolins(participant_fit_results, young_low, old_low, old_high)

groups = {'Young','Old Low','Old High'};
group_ids = {young_low, old_low, old_high};
group_colors = {[0 0 1], [0 0.7 0], [1 0 0]};

project_young = [102 103 104];
project_hyper = 101;

figure; tiledlayout(1,2);

% ---- beta ----
nexttile; hold on; grid on;
plotOneViolinPanel(participant_fit_results, ...
    'Beta_Best', '\beta', groups, group_ids, group_colors, project_young, project_hyper);

% ---- sigma ----
nexttile; hold on; grid on;
plotOneViolinPanel(participant_fit_results, ...
    'SigmaR_Best', '\sigma_r', groups, group_ids, group_colors, project_young, project_hyper);

end


function plotOneViolinPanel(T, value_field, y_label, groups, group_ids, group_colors, project_young, project_hyper)

violin_width = 0.28;
base_marker_size = 70;
size_scale = 45;

for g = 1:numel(groups)

    idx = ismember(T.PID, group_ids{g});
    vals = T.(value_field)(idx);
    pids = T.PID(idx);

    vals = vals(~isnan(vals));

    if isempty(vals)
        continue
    end

    % ---- Violin shape ----
    if numel(unique(vals)) >= 2
        [density, y_grid] = ksdensity(vals);
        density = density ./ max(density) * violin_width;

        x_left  = g - density;
        x_right = g + density;

        patch([x_left, fliplr(x_right)], ...
              [y_grid, fliplr(y_grid)], ...
              group_colors{g}, ...
              'FaceAlpha', 0.15, ...
              'EdgeColor', group_colors{g}, ...
              'LineWidth', 1.2, ...
              'HandleVisibility', 'off');
    else
        % If only one unique value, draw a thin vertical line
        plot([g g], [vals(1)-0.005 vals(1)+0.005], ...
            'Color', group_colors{g}, ...
            'LineWidth', 2, ...
            'HandleVisibility', 'off');
    end

    % ---- Median line ----
    med_val = median(vals, 'omitnan');
    plot([g-violin_width, g+violin_width], [med_val, med_val], ...
        'Color', group_colors{g}, ...
        'LineWidth', 2, ...
        'HandleVisibility', 'off');

    % ---- Bubble points: one bubble per unique parameter value ----
    unique_vals = unique(vals);

    for u = 1:numel(unique_vals)

        this_val = unique_vals(u);
        same_idx = idx & T.(value_field) == this_val;
        same_pids = T.PID(same_idx);

        n_same = sum(same_idx);

        x = g;

        c = group_colors{g};
        marker = 'o';

        % Special coloring if this bubble includes project participants
        if any(ismember(same_pids, project_young))
            c = 0.45 * group_colors{g};
        end

        if any(ismember(same_pids, project_hyper))
            c = [0 0 0.35];
            marker = 'd';
        end

        scatter(x, this_val, ...
            base_marker_size + size_scale*n_same, ...
            c, marker, ...
            'filled', ...
            'MarkerFaceAlpha', 0.55, ...
            'MarkerEdgeColor', c, ...
            'LineWidth', 1.2, ...
            'HandleVisibility', 'off');
    end
end

xlim([0.5, numel(groups)+0.5]);
xticks(1:numel(groups));
xticklabels(groups);
ylabel(y_label);
title(y_label);
set(gca, 'FontSize', 12);

% Dummy legend entries
scatter(NaN, NaN, 100, [0 0 1], 'o', 'filled', ...
    'MarkerFaceAlpha', 0.55, 'DisplayName', 'Young');
scatter(NaN, NaN, 100, [0 0.7 0], 'o', 'filled', ...
    'MarkerFaceAlpha', 0.55, 'DisplayName', 'Old Low');
scatter(NaN, NaN, 100, [1 0 0], 'o', 'filled', ...
    'MarkerFaceAlpha', 0.55, 'DisplayName', 'Old High');
scatter(NaN, NaN, 100, [0 0 0.35], 'd', 'filled', ...
    'MarkerFaceAlpha', 0.55, 'DisplayName', 'FB / Hypermobile');

legend('Location','best');

end