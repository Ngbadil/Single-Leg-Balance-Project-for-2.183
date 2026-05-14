participant_table = readtable('table_1_data_deidentified.csv', ...
    'VariableNamingRule', 'preserve');
 
% Participant IDs (not row indices) for each group
young_low = 1:29;
old_low   = [45, 49, 50, 31, 54, 30, 47, 32, 53];
old_med   = [];   % define explicitly; leave empty if no participants
old_high  = [52, 56, 55, 36, 51, 43, 35, 39];
 
% Use a containers.Map keyed by participant ID so we don't conflate
% row index with participant number.
CoM_height = containers.Map('KeyType', 'double', 'ValueType', 'double');
 
fprintf('Rows in participant_table: %d\n', height(participant_table));
 
for i = 1:height(participant_table)
    pid_raw = participant_table.Participant_ID(i);
    % Handle either numeric or string IDs ("P30" -> 30)
    if iscell(pid_raw) || isstring(pid_raw)
        pid = str2double(regexprep(char(string(pid_raw)), '[^0-9]', ''));
    else
        pid = double(pid_raw);
    end
 
    totalHeight_m = participant_table{i, 'Height (in)'} * 0.0254
    head_pos(2)
    totalMass_kg  = participant_table{i, 'Weight (lb)'} * 0.453592;
    gender_num    = participant_table{i, 'Gender (1=M,2=F)'};
 
    % Skip if missing
    if isnan(pid) || isnan(totalHeight_m) || isnan(totalMass_kg) || isnan(gender_num)
        continue
    end
 
    p = parameters();
 
    % Compute participant-specific lumped params
    lumped_params_pid = getLumpedParams_DIP_Ypose( ...
        totalMass_kg, totalHeight_m, gender_num, 'sgt', pi/4);
 
    % Add net COM height expected by predictZIPfromModel
    lumped_params_pid.L_COM = ...
        (lumped_params_pid.m1 * lumped_params_pid.c1 + ...
         lumped_params_pid.m2 * (lumped_params_pid.L1 + lumped_params_pid.c2)) / ...
        (lumped_params_pid.m1 + lumped_params_pid.m2);
 
    p(1) = lumped_params_pid.L1;
    p(2) = lumped_params_pid.L2;
    p(3) = lumped_params_pid.c1;
    p(4) = lumped_params_pid.c2;
    p(5) = lumped_params_pid.m1;
    p(6) = lumped_params_pid.m2;
    p(7) = lumped_params_pid.j1;
    p(8) = lumped_params_pid.j2;
 
    head_pos = head_pos_DIP([0;0;0;0], p);
    CoM_pos  = CoM_pos_DIP ([0;0;0;0], p);
 
    CoM_height(pid) = CoM_pos(2) / head_pos(2);
end
 
% Helper to pull values for a group's IDs, skipping any not in the map
getVals = @(ids) cell2mat(arrayfun(@(k) CoM_height(k), ...
    ids(arrayfun(@(k) isKey(CoM_height, k), ids)), 'UniformOutput', false));
 
young_vals    = getVals(young_low);
old_low_vals  = getVals(old_low);
old_med_vals  = getVals(old_med);
old_high_vals = getVals(old_high);
 
fprintf('Group sizes: young=%d, old_low=%d, old_med=%d, old_high=%d\n', ...
    numel(young_vals), numel(old_low_vals), numel(old_med_vals), numel(old_high_vals));
 
figure;
hold on;
 
x1 = ones(size(young_vals))    * 1;
x2 = ones(size(old_low_vals))  * 2;
x3 = ones(size(old_med_vals))  * 3;
x4 = ones(size(old_high_vals)) * 4;
 
jit = 0.08;
scatter(x1 + (rand(size(x1))-0.5)*jit, young_vals,    70, 'b', 'filled');
scatter(x2 + (rand(size(x2))-0.5)*jit, old_low_vals,  70, 'g', 'filled');
if ~isempty(old_med_vals)
    scatter(x3 + (rand(size(x3))-0.5)*jit, old_med_vals, 70, [1 0.5 0], 'filled');
end
scatter(x4 + (rand(size(x4))-0.5)*jit, old_high_vals, 70, 'r', 'filled');
 
xticks([1 2 3 4]);
if isempty(old_med_vals)
    xticklabels({'Young','Old Low','','Old High'});
else
    xticklabels({'Young','Old Low','Old Med','Old High'});
end
 
ylabel('CoM Height / Head Height (ratio)')   % unitless ratio, not meters
title('Center-of-Mass Height (normalized) by Participant Group')
grid on
box on
legend({'Young','Old Low','Old Med','Old High'}, 'Location', 'best');
hold off