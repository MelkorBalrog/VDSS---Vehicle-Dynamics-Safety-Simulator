%--------------------------------------------------------------------------
% This file is part of VDSS - Vehicle Dynamics Safety Simulator.
%
% VDSS is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% VDSS is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <https://www.gnu.org/licenses/>.
%--------------------------------------------------------------------------
%{
% @file plotVehicleMotionResults.m
% @brief Utility script to visualize logged vehicle motion data.
%        Calculates steering rates and exports all signals.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}

% MATLAB Script to Calculate and Plot Steering Rate from Steering Angle using Net Method
% Additionally, calculates Steering Wheel Angle and Steering Wheel Rate.
% Extracts and plots Orientation, VelocityU, LateralVelocityV, YawRateR,
% RollAngle, RollRate, AccelerationLongitudinal, AccelerationLateral, Clutch,
% CurrentGear, Throttle, Horsepower,
% CommandedAcceleration, and CommandedSteerAngles.
% Converts velocities from meters per second (m/s) to kilometers per hour (kph).
% Plots all signals in a single figure using tiledlayout.
% Exports all processed data to a CSV file with appropriate column headers.

% ============================
% 1. Initialization
% ============================

% Clear workspace, command window, and close all figures
clear;
clc;
close all;

% ============================
% 2. Load and Inspect Data
% ============================

% Define the data file name
data_file = 'simulation_Vehicle1.mat'; % Update with your actual filename

% Check if the data file exists
if exist(data_file, 'file') == 2
    data = load(data_file);
    disp(['Data loaded successfully from ', data_file]);
else
    error(['Error: Data file ', data_file, ' not found. Please check the filename and path.']);
end

% Display the field names to verify correct field access
disp('Available Fields in the Data:');
disp(fieldnames(data));

% ============================
% 3. Define Steering Ratio and Calculate Steering Wheel Angle
% ============================

% Define the Steering Ratio
% This value depends on the vehicle. Common values range from 14:1 to 20:1.
% Adjust this value based on your vehicle's specifications.
steering_ratio = 15; % Example: 15:1
disp(['Using Steering Ratio: ', num2str(steering_ratio), ':1']);

% ============================
% 4. Extract and Prepare Steering Angle Data
% ============================

% Check if 'SteeringAngle' exists in the loaded data
if isfield(data, 'SteeringAngle')
    SteeringAngle = data.SteeringAngle;
    disp('SteeringAngle data extracted.');
else
    error('Error: The field ''SteeringAngle'' is missing in the loaded data.');
end

% Extract Time
if isfield(data, 'Time')
    t = data.Time; % Time [s]
    disp('Time data extracted.');
else
    error('Error: The field ''Time'' is missing in the loaded data.');
end

% Convert steering angle from radians to degrees if necessary
% Assumption: If the maximum absolute value of SteeringAngle is <= 2*pi (~6.283), it's in radians; otherwise, in degrees
if max(abs(SteeringAngle)) <= 2*pi
    % Convert from radians to degrees
    st_angle_deg = SteeringAngle * (180/pi);
    disp('SteeringAngle converted from radians to degrees.');
else
    % Assuming already in degrees
    st_angle_deg = SteeringAngle;
    disp('SteeringAngle is assumed to be in degrees.');
end

% Ensure that t and st_angle_deg are column vectors
t = t(:);                         % Converts to column vector
st_angle_deg = st_angle_deg(:);   % Converts to column vector

% Verify that Time and Steering Angle vectors are of the same length
if length(t) ~= length(st_angle_deg)
    error('Error: Time and Steering Angle vectors must be of the same length.');
end

% ============================
% 5. Calculate Steering Rate Using Net Method
% ============================

% Compute the steering angle rate using the net method
dt = diff(t);  % Time differences
st_angle_rate = diff(st_angle_deg) ./ dt;  % Rate of change [°/s]

% Pad the steering angle rate with NaN to match the length of the original data
st_angle_rate_padded = [st_angle_rate; NaN];

% ============================
% 6. Calculate Steering Wheel Angle and Steering Wheel Rate
% ============================

% Calculate Steering Wheel Angle
stw_angle_deg = st_angle_deg * steering_ratio; % Steering Wheel Angle [°]
disp('Steering Wheel Angle calculated.');

% Calculate Steering Wheel Rate using the net method
stw_angle_rate = diff(stw_angle_deg) ./ dt; % Steering Wheel Rate [°/s]

% Pad the steering wheel angle rate with NaN to match the length of the original data
stw_angle_rate_padded = [stw_angle_rate; NaN];

% ============================
% 7. Extract Additional Signals
% ============================

% Updated list of additional signals to extract
additional_fields = {
    'Orientation', ...
    'VelocityU', ...
    'LateralVelocityV', ...
    'YawRateR', ...
    'RollAngle', ...
    'RollRate', ...
    'AccelerationLongitudinal', ...
    'AccelerationLateral', ...
    'Clutch', ...
    'CurrentGear', ...
    'Throttle', ...
    'Horsepower', ...
    'JerkLongitudinal', ...
    'JerkLateral', ...
    'CommandedAcceleration', ...
    'CommandedSteerAngles'
};

% Keep a separate list for any force-related fields discovered later
force_field_names = {};

% Initialize a structure to hold the extracted signals
extracted_signals = struct();

% Loop through each additional field and extract if present
for i = 1:length(additional_fields)
    field = additional_fields{i};
    if isfield(data, field)
        temp = data.(field);
        % Check if the data is a vector
        if isvector(temp)
            temp = temp(:); % Ensure column vector
        else
            % If multi-dimensional, take the first column
            temp = temp(:,1);
            warning(['Field ''', field, ''' is multi-dimensional. Taking the first column for export.']);
        end
        % Check length and pad with NaN if necessary
        if length(temp) < length(t)
            temp = [temp; NaN(length(t)-length(temp),1)];
            warning(['Field ''', field, ''' is shorter than Time. Padded with NaN.']);
        elseif length(temp) > length(t)
            temp = temp(1:length(t)); % Trim to match length
            warning(['Field ''', field, ''' is longer than Time. Trimmed to match Time length.']);
        end
        extracted_signals.(field) = temp;
        disp([field, ' data extracted and processed.']);
    else
        warning(['Warning: Field ''', field, ''' is missing in the loaded data. Filling with NaNs.']);
        extracted_signals.(field) = NaN(length(t),1); % Fill with NaNs if missing
    end
end

% ==============================================
% 7b. Extract Force Fields from calculatedForces
% ==============================================

% Some simulation files store force information inside a struct called
% 'calculatedForces' (lower or upper case). Extract all numeric vectors
% within this struct and append them to the exported signals.
force_struct = [];
if isfield(data, 'calculatedForces')
    force_struct = data.calculatedForces;
elseif isfield(data, 'CalculatedForces')
    force_struct = data.CalculatedForces;
end

if ~isempty(force_struct)
    cf_names = fieldnames(force_struct);
    for i = 1:length(cf_names)
        f_name = cf_names{i};
        temp = force_struct.(f_name);
        if isvector(temp)
            temp = temp(:); % ensure column
        else
            temp = temp(:,1);
            warning(['Force field ''', f_name, ''' is multi-dimensional. Taking the first column.']);
        end
        if length(temp) < length(t)
            temp = [temp; NaN(length(t)-length(temp),1)];
        elseif length(temp) > length(t)
            temp = temp(1:length(t));
        end
        extracted_signals.(f_name) = temp;
        force_field_names{end+1} = f_name; %#ok<AGROW>
        disp(['Force field ', f_name, ' extracted.']);
    end
else
    disp('No calculated forces found in the data.');
end

% ============================
% 8. Convert Velocities from m/s to kph
% ============================

% Conversion factor from m/s to kph
ms_to_kph = 3.6;

% Convert VelocityU and LateralVelocityV if they are not NaN
if ~all(isnan(extracted_signals.VelocityU))
    extracted_signals.VelocityU_kph = extracted_signals.VelocityU * ms_to_kph;
    disp('VelocityU converted from m/s to kph.');
else
    extracted_signals.VelocityU_kph = NaN(length(t),1);
    disp('VelocityU is NaN. Conversion skipped.');
end

if ~all(isnan(extracted_signals.LateralVelocityV))
    extracted_signals.LateralVelocityV_kph = extracted_signals.LateralVelocityV * ms_to_kph;
    disp('LateralVelocityV converted from m/s to kph.');
else
    extracted_signals.LateralVelocityV_kph = NaN(length(t),1);
    disp('LateralVelocityV is NaN. Conversion skipped.');
end

% ============================
% 9. Plot All Signals in a Single Figure Using TiledLayout
% ============================

% Updated list to include all signals
all_fields = {
    'Steering Angle (°)', ...
    'Steering Rate (°/s)', ...
    'Steering Wheel Angle (°)', ...
    'Steering Wheel Rate (°/s)', ...
    'Orientation (°)', ...
    'VelocityU (kph)', ...
    'LateralVelocityV (kph)', ...
    'YawRateR (°/s)', ...
    'RollAngle (°)', ...
    'RollRate (°/s)', ...
    'AccelerationLongitudinal (m/s²)', ...
    'AccelerationLateral (m/s²)', ...
    'Clutch (%)', ...
    'CurrentGear', ...
    'Throttle (%)', ...
    'Horsepower (hp)', ...
    'CommandedAcceleration', ...
    'CommandedSteerAngles (°)'
};

% Initialize a figure
figure('Name', 'All Vehicle Signals over Time', 'NumberTitle', 'off');

% Define tiled layout with appropriate number of tiles
num_signals = length(all_fields);
rows = ceil(num_signals / 2); % Adjust columns as needed
cols = 2; % Number of columns

% Create tiled layout
tl = tiledlayout(rows, cols, 'TileSpacing', 'compact', 'Padding', 'compact');

% Plot each signal in its respective tile
for i = 1:num_signals
    nexttile;
    switch all_fields{i}
        case 'Steering Angle (°)'
            plot(t, st_angle_deg, '-b', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'Steering Angle (°)'}, 'Location', 'best');
        case 'Steering Rate (°/s)'
            plot(t, st_angle_rate_padded, '-r', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'Steering Rate (°/s)'}, 'Location', 'best');
        case 'Steering Wheel Angle (°)'
            plot(t, stw_angle_deg, '-g', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'Steering Wheel Angle (°)'}, 'Location', 'best');
        case 'Steering Wheel Rate (°/s)'
            plot(t, stw_angle_rate_padded, '-m', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'Steering Wheel Rate (°/s)'}, 'Location', 'best');
        case 'Orientation (°)'
            plot(t, extracted_signals.Orientation, '-m', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'Orientation (°)'}, 'Location', 'best');
        case 'VelocityU (kph)'
            plot(t, extracted_signals.VelocityU_kph, '-b', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'VelocityU (kph)'}, 'Location', 'best');
        case 'LateralVelocityV (kph)'
            plot(t, extracted_signals.LateralVelocityV_kph, '-g', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'LateralVelocityV (kph)'}, 'Location', 'best');
        case 'YawRateR (°/s)'
            plot(t, extracted_signals.YawRateR, '-c', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'YawRateR (°/s)'}, 'Location', 'best');
        case 'RollAngle (°)'
            plot(t, extracted_signals.RollAngle, '-b', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'RollAngle (°)'}, 'Location', 'best');
        case 'RollRate (°/s)'
            plot(t, extracted_signals.RollRate, '-r', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'RollRate (°/s)'}, 'Location', 'best');
        case 'AccelerationLongitudinal (m/s²)'
            plot(t, extracted_signals.AccelerationLongitudinal, '-k', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'AccelerationLongitudinal (m/s²)'}, 'Location', 'best');
        case 'AccelerationLateral (m/s²)'
            plot(t, extracted_signals.AccelerationLateral, '-y', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'AccelerationLateral (m/s²)'}, 'Location', 'best');
        case 'Clutch (%)'
            plot(t, extracted_signals.Clutch, '-r', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'Clutch (%)'}, 'Location', 'best');
        case 'CurrentGear'
            stairs(t, extracted_signals.CurrentGear, '-m', 'LineWidth', 1.2); % Use stairs for discrete steps
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'CurrentGear'}, 'Location', 'best');
        case 'Throttle (%)'
            plot(t, extracted_signals.Throttle, '-c', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'Throttle (%)'}, 'Location', 'best');
        case 'Horsepower (hp)'
            plot(t, extracted_signals.Horsepower, '-b', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'Horsepower (hp)'}, 'Location', 'best');
        case 'CommandedAcceleration'
            plot(t, extracted_signals.CommandedAcceleration, '-c', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'CommandedAcceleration'}, 'Location', 'best');
        case 'CommandedSteerAngles (°)'
            plot(t, extracted_signals.CommandedSteerAngles, '-y', 'LineWidth', 1.2);
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'CommandedSteerAngles (°)'}, 'Location', 'best');
        otherwise
            % Default case (should not occur)
            plot(t, NaN(length(t),1), '-k');
            ylabel(all_fields{i});
            title(all_fields{i});
            legend({'N/A'}, 'Location', 'best');
    end
    grid on;
end

% Adjust layout for better visibility
tight_layout(); % Adjusts subplot positions

% ============================
% 9b. Plot Jerk Signals in Separate Figure
% ============================

figure('Name', 'Vehicle Jerk Signals', 'NumberTitle', 'off');
tl_jerk = tiledlayout(2,1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, extracted_signals.JerkLongitudinal, '-b', 'LineWidth', 1.2);
ylabel('Jerk Longitudinal (m/s^3)');
title('Jerk Longitudinal');
grid on;

nexttile;
plot(t, extracted_signals.JerkLateral, '-r', 'LineWidth', 1.2);
ylabel('Jerk Lateral (m/s^3)');
title('Jerk Lateral');
xlabel('Time (s)');
grid on;

tight_layout();

% ============================
% 9c. Plot Force Signals by Category
% ============================

if ~isempty(force_field_names)
    categories = struct('General',{{}}, 'Tire',{{}}, 'Trailer',{{}}, 'Moment',{{}}, 'Hitch',{{}});
    for i = 1:numel(force_field_names)
        fname = force_field_names{i};
        lname = lower(fname);
        if contains(lname,'tire') || contains(lname,'traction') || contains(lname,'f_y')
            categories.Tire{end+1} = fname;
        elseif contains(lname,'trailer')
            categories.Trailer{end+1} = fname;
        elseif contains(lname,'moment') || strcmpi(fname,'M_z')
            categories.Moment{end+1} = fname;
        elseif contains(lname,'hitch')
            categories.Hitch{end+1} = fname;
        else
            categories.General{end+1} = fname;
        end
    end

    cat_names = fieldnames(categories);
    for c = 1:length(cat_names)
        flds = categories.(cat_names{c});
        if isempty(flds)
            continue;
        end
        figure('Name', ['Vehicle Forces - ' cat_names{c}], 'NumberTitle', 'off');
        rows = ceil(numel(flds)/2);
        cols = 2;
        tl_force = tiledlayout(rows, cols, 'TileSpacing','compact','Padding','compact'); %#ok<NASGU>
        for k = 1:numel(flds)
            nexttile;
            plot(t, extracted_signals.(flds{k}), 'LineWidth', 1.2);
            title(strrep(flds{k}, '_', ' '), 'Interpreter','none');
            ylabel(flds{k});
            grid on;
        end
        tight_layout();
    end
end

% ============================
% 10. Save Results (With Column Names)
% ============================

% Updated column names
column_names = {
    'Time (s)', ...
    'Steering Angle (deg)', ...
    'Steering Rate (deg/s)', ...
    'Steering Wheel Angle (deg)', ...
    'Steering Wheel Rate (deg/s)', ...
    'Orientation (deg)', ...
    'VelocityU (kph)', ...
    'LateralVelocityV (kph)', ...
    'YawRateR (deg/s)', ...
    'RollAngle (deg)', ...
    'RollRate (deg/s)', ...
    'AccelerationLongitudinal (m/s²)', ...
    'AccelerationLateral (m/s²)', ...
    'JerkLongitudinal (m/s^3)', ...
    'JerkLateral (m/s^3)', ...
    'Clutch (%)', ...
    'CurrentGear', ...
    'Throttle (%)', ...
    'Horsepower (hp)', ...
    'CommandedAcceleration', ...
    'CommandedSteerAngles (deg)'
};

% Append names of extracted force fields
for i = 1:numel(force_field_names)
    column_names{end+1} = force_field_names{i}; %#ok<AGROW>
end

% Prepare data for export
export_data = [t, ...
               st_angle_deg, st_angle_rate_padded, ...
               stw_angle_deg, stw_angle_rate_padded, ...
               extracted_signals.Orientation, ...
               extracted_signals.VelocityU_kph, ...
               extracted_signals.LateralVelocityV_kph, ...
               extracted_signals.YawRateR, ...
               extracted_signals.RollAngle, ...
               extracted_signals.RollRate, ...
               extracted_signals.AccelerationLongitudinal, ...
               extracted_signals.AccelerationLateral, ...
               extracted_signals.JerkLongitudinal, ...
               extracted_signals.JerkLateral, ...
               extracted_signals.Clutch, ...
               extracted_signals.CurrentGear, ...
               extracted_signals.Throttle, ...
               extracted_signals.Horsepower, ...
               extracted_signals.CommandedAcceleration, ...

               extracted_signals.CommandedSteerAngles];

% Append force data to export
for i = 1:numel(force_field_names)
    export_data = [export_data, extracted_signals.(force_field_names{i})]; %#ok<AGROW>
end

% Verify all columns have the same number of rows
expected_length = length(t);
for col = 1:size(export_data,2)
    if length(export_data(:,col)) ~= expected_length
        error(['Error: Column ', num2str(col), ' has length ', num2str(length(export_data(:,col))), ...
               ' which does not match expected length of ', num2str(expected_length), '.']);
    end
end

% Create a cell array with column names and data
export_cell = [column_names; num2cell(export_data)];

% Define output CSV file name
output_file = 'SteeringRateData.csv';

% Write data to CSV file with column names
try
    writecell(export_cell, output_file);
    disp(['Data has been successfully saved to ', output_file]);
catch ME
    warning(['Warning: Failed to write data to ', output_file, '. Error: ', ME.message]);
end

% ============================
% End of Script
% ============================

% ============================
% Helper Function: Tight Layout
% ============================
% Note: MATLAB does not have a built-in tight_layout function like Python's matplotlib.
% The following custom function adjusts subplot positions to minimize white space.

function tight_layout()
    % Adjust the subplots to minimize white space
    % Get all subplot handles
    h = get(gcf, 'Children');
    % Loop through each subplot and adjust position
    for i = 1:length(h)
        pos = get(h(i), 'Position');
        pos(1) = pos(1) + 0.02; % Shift right by 2%
        pos(3) = pos(3) - 0.04; % Decrease width by 4%
        pos(2) = pos(2) + 0.02; % Shift up by 2%
        pos(4) = pos(4) - 0.04; % Decrease height by 4%
        set(h(i), 'Position', pos);
    end
end
