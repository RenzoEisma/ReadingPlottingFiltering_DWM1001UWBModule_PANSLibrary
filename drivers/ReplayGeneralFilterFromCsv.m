% =========================================================================
% ReplayGeneralFilterFromCsv.m
% Author: Renzo Eisma
% Date: 06/2026
%
% Purpose:
% Replay a raw UWB CSV through GeneralFilter.m as if the data arrived live.
% This lets you test filter settings from GeneralFilter.m without taking a
% new measurement every time.
%
% Important:
% - This script does NOT contain filter settings.
% - Change filter settings inside GeneralFilter.m.
% - This script only selects a raw UWB CSV, feeds each row to GeneralFilter,
%   and lets GeneralFilter write the normal live-format CSV.
%
% Output filename:
%   [Log]_uwb_general_filter_Session_YYYYMMDD_HHMMSS.csv
%
% Output format:
%   Time,POSX,POSY,POSZ
% =========================================================================

clear;
clc;

%% ========================================================================
%  SELECT INPUT CSV
% =========================================================================

[input_file, input_folder] = uigetfile({'*.csv', 'CSV files (*.csv)'}, 'Select raw UWB CSV');

if isequal(input_file, 0)
    disp('[REPLAY GENERAL FILTER] No file selected. Script stopped.');
    return;
end

input_path = fullfile(input_folder, input_file);
[~, input_name, ~] = fileparts(input_path);

output_file = makeOutputFileName(input_name, input_folder);
output_path = fullfile(input_folder, output_file);

fprintf('[REPLAY GENERAL FILTER] Input CSV:\n%s\n', input_path);
fprintf('[REPLAY GENERAL FILTER] Output CSV:\n%s\n', output_path);

T = readtable(input_path, 'VariableNamingRule', 'preserve');

if height(T) == 0
    error('[REPLAY GENERAL FILTER] Input CSV is empty.');
end

%% ========================================================================
%  FIND COLUMNS
% =========================================================================

var_names = string(T.Properties.VariableNames);
lower_names = lower(strrep(var_names, ' ', '_'));

idx_time = findFirstColumn(lower_names, [ ...
    "timestamp", "time", "pc_timestamp", "pc_time", "pc", "t"]);

idx_x = findFirstColumn(lower_names, [ ...
    "x", "posx", "pos_x", "position_x", "uwb_x", "raw_x"]);

idx_y = findFirstColumn(lower_names, [ ...
    "y", "posy", "pos_y", "position_y", "uwb_y", "raw_y"]);

idx_z = findFirstColumn(lower_names, [ ...
    "z", "posz", "pos_z", "position_z", "uwb_z", "raw_z"]);

idx_quality = findFirstColumn(lower_names, [ ...
    "quality", "q", "accuracy", "acc"]);

if isempty(idx_time)
    error('[REPLAY GENERAL FILTER] Could not find a time column. Expected timestamp, Time, or PC_Timestamp.');
end

if isempty(idx_x) || isempty(idx_y) || isempty(idx_z)
    error('[REPLAY GENERAL FILTER] Could not find X/Y/Z columns. Expected x,y,z or POSX,POSY,POSZ.');
end

fprintf('[REPLAY GENERAL FILTER] Using columns:\n');
fprintf('  Time:    %s\n', var_names(idx_time));
fprintf('  X:       %s\n', var_names(idx_x));
fprintf('  Y:       %s\n', var_names(idx_y));
fprintf('  Z:       %s\n', var_names(idx_z));

if isempty(idx_quality)
    fprintf('  Quality: not found, using NaN\n');
else
    fprintf('  Quality: %s\n', var_names(idx_quality));
end

%% ========================================================================
%  PREPARE DATA
% =========================================================================

time_values = toNumericColumn(T{:, idx_time});
x_values = toNumericColumn(T{:, idx_x});
y_values = toNumericColumn(T{:, idx_y});
z_values = toNumericColumn(T{:, idx_z});

if isempty(idx_quality)
    quality_values = NaN(height(T), 1);
else
    quality_values = toNumericColumn(T{:, idx_quality});
end

valid_rows = isfinite(time_values) & isfinite(x_values) & isfinite(y_values) & isfinite(z_values);

if ~any(valid_rows)
    error('[REPLAY GENERAL FILTER] No valid rows found with finite Time/X/Y/Z.');
end

%% ========================================================================
%  CREATE FILTER AND WRITE NORMAL LIVE-FORMAT OUTPUT
% =========================================================================

% No configuration is passed here on purpose.
% Change filter settings inside GeneralFilter.m itself.
filter = GeneralFilter();

% This is the exact output file. No "Offline" and no error/debug file.
filter.setLogFile(output_path);

fprintf('[REPLAY GENERAL FILTER] Replaying %d rows through GeneralFilter...\n', height(T));

for i = 1:height(T)
    uwb_sample = struct();
    uwb_sample.position = [x_values(i), y_values(i), z_values(i)];
    uwb_sample.quality = quality_values(i);
    uwb_sample.timestamp = time_values(i);
    uwb_sample.source = 'python_udp_replay';
    uwb_sample.valid = valid_rows(i);

    filter.processUwbSample(uwb_sample);
end

filter.closeLog();

fprintf('[REPLAY GENERAL FILTER] Done.\n');
fprintf('[REPLAY GENERAL FILTER] Accepted: %d, Outliers: %d, Invalid: %d\n', ...
    filter.AcceptedCount, filter.OutlierCount, filter.InvalidCount);
fprintf('[REPLAY GENERAL FILTER] Output written to:\n%s\n', output_path);

%% ========================================================================
%  LOCAL HELPER FUNCTIONS
% =========================================================================

function output_file = makeOutputFileName(input_name, input_folder)
    % The required output name is:
    %   [Log]_uwb_general_filter_Session_YYYYMMDD_HHMMSS.csv

    input_name = char(string(input_name));
    input_folder = char(string(input_folder));

    % Extract the session name from the selected file or from its folder path.
    combined_text = [input_folder, filesep, input_name];
    session_name = regexp(combined_text, 'Session_\d{8}_\d{6}', 'match', 'once');

    if isempty(session_name)
        date_time = regexp(combined_text, '\d{8}_\d{6}', 'match', 'once');

        if isempty(date_time)
            warning('[REPLAY GENERAL FILTER] No session name found. Using current time for output filename.');
            session_name = ['Session_', datestr(now, 'yyyymmdd_HHMMSS')];
        else
            session_name = ['Session_', date_time];
        end
    end

    % Extract only the original log name.
    % Examples:
    %   [Log]_uwb_listener1_Session_20260603_204050 -> [Log]
    %   [Log]_uwb_listener2_Session_20260603_204050 -> [Log]
    %   [Log]_uwb_general_filter_Session_20260603_204050 -> [Log]
    %   [Log]_optitrack_Session_20260603_204050 -> [Log]
    log_name = input_name;

    known_suffixes = { ...
        '_uwb_listener', ...
        '_uwb_general_filter', ...
        '_uwb_filtered', ...
        '_uwb', ...
        '_optitrack', ...
        '_gps_rtk', ...
        '_measurement_window'};

    for k = 1:numel(known_suffixes)
        suffix = known_suffixes{k};
        idx = strfind(log_name, suffix);

        if ~isempty(idx)
            log_name = log_name(1:idx(1)-1);
            break;
        end
    end

    if isempty(strtrim(log_name))
        log_name = '[Log]';
    end

    output_file = sprintf('%s_uwb_general_filter_%s.csv', log_name, session_name);
end

function idx = findFirstColumn(lower_names, possible_names)
    idx = [];

    for k = 1:numel(possible_names)
        wanted = lower(string(possible_names(k)));
        match = find(lower_names == wanted, 1, 'first');

        if ~isempty(match)
            idx = match;
            return;
        end
    end
end

function values = toNumericColumn(col)
    if isnumeric(col)
        values = double(col);
        return;
    end

    if iscell(col)
        values = str2double(string(col));
        return;
    end

    if isstring(col) || ischar(col) || iscategorical(col)
        values = str2double(string(col));
        return;
    end

    try
        values = double(col);
    catch
        values = str2double(string(col));
    end
end