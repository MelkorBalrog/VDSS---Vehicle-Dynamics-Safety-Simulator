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
% @file checkMaxDistanceAtMaxSpeed.m
% @brief Compares maximum travel distances of two vehicles in a simulation.
%        Loads logs from two runs and prints the difference.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}

% MATLAB Script: checkMaxDistanceAtMaxSpeed.m
% Description: Compares the maximum distances traveled by two vehicles over the complete simulation.

% Clear workspace and command window
clear; clc;

% -------------------------- User Configuration --------------------------
% Specify the paths to the two .mat files
file1 = 'simulation_Vehicle1.mat';  % Replace with your first .mat file name
file2 = 'simulation_Vehicle2.mat';  % Replace with your second .mat file name
% ------------------------------------------------------------------------

% --------------------------- Load Data -----------------------------------
% Load the first vehicle's data
data1 = load(file1);
data2 = load(file2);
% ------------------------------------------------------------------------

% ----------------------- Define Vehicle Data ----------------------------
% Function to extract necessary fields from loaded data
extractFields = @(data) struct(...
    'PositionX', data.PositionX, ...
    'PositionY', data.PositionY);

% Extract fields for both vehicles
vehicle1 = extractFields(data1);
vehicle2 = extractFields(data2);
% ------------------------------------------------------------------------

% ---------------------- Calculate Displacements -------------------------
% Calculate displacement from the starting point for each vehicle
displacement1 = sqrt((vehicle1.PositionX - vehicle1.PositionX(1)).^2 + ...
                     (vehicle1.PositionY - vehicle1.PositionY(1)).^2);
displacement2 = sqrt((vehicle2.PositionX - vehicle2.PositionX(1)).^2 + ...
                     (vehicle2.PositionY - vehicle2.PositionY(1)).^2);
% ------------------------------------------------------------------------

% ---------------------- Find Maximum Distances --------------------------
% Find the maximum displacement for each vehicle
maxDistance1 = max(displacement1);
maxDistance2 = max(displacement2);

% Calculate the difference in maximum distances
distanceDifference = abs(maxDistance1 - maxDistance2);
% ------------------------------------------------------------------------

% ----------------------- Display Results --------------------------------
fprintf('--- Maximum Distance Comparison ---\n\n');

% Vehicle 1 Results
fprintf('Vehicle 1:\n');
fprintf('  Maximum Distance Traveled: %.2f meters\n\n', maxDistance1);

% Vehicle 2 Results
fprintf('Vehicle 2:\n');
fprintf('  Maximum Distance Traveled: %.2f meters\n\n', maxDistance2);

% Distance Difference
fprintf('Distance Difference: %.2f meters\n', distanceDifference);
% ------------------------------------------------------------------------

% ----------------------- Visualization -----------------------------------
% Create a bar chart comparing the maximum distances
figure('Name', 'Maximum Distance Comparison', 'NumberTitle', 'off');

% Data for bar chart
distances = [maxDistance1, maxDistance2];
labels = {'Vehicle 1', 'Vehicle 2'};

% Create bar chart
barHandle = bar(distances, 'FaceColor', [0.2 0.6 0.8]);
hold on;

% Annotate bars with distance values
for j = 1:length(distances)
    text(j, distances(j), sprintf('%.2f m', distances(j)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 12);
end

% Labels and Title
set(gca, 'XTickLabel', labels, 'FontSize', 12);
ylabel('Maximum Distance (meters)', 'FontSize', 12);
title('Comparison of Maximum Distances Traveled by Vehicles', 'FontSize', 14);
grid on;
ylim([0, max(distances)*1.2]);

% Add annotation for the difference
annotation('textbox', [0.5, 0.8, 0.3, 0.1], 'String', ...
    sprintf('Difference: %.2f m', distanceDifference), ...
    'EdgeColor', 'none', 'FontSize', 12, 'HorizontalAlignment', 'center');

hold off;
% ------------------------------------------------------------------------

% ------------------------ End of Script ----------------------------------
