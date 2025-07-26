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
% @file DataManager.m
% @brief Handles data loading and exporting for the simulation.
%        Provides persistence utilities for the UI.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}
%/**
% * @class DataManager
% * @brief Handles all data-related operations for the simulation.
% *
% * The DataManager class manages data loading, saving, and exporting functionalities within the simulation.
% * It interacts with UI components and simulation configuration objects to ensure that all relevant data is
% * accurately processed and persisted. This includes loading simulation parameters from Excel files, saving
% * manual control data, exporting plots as images, and saving comprehensive simulation results.
% *
% * @version 2.2
% * @date 2024-10-18
% */
classdef DataManager < handle
    properties
        %/**
        % * @property dt
        % * @brief Time step in seconds.
        % *
        % * Determines the temporal resolution of the simulation data.
        % *
        % * @default 0.1
        % */
        dt = 0.1; % Time step in seconds

        %/**
        % * @property globalVehicle1Data
        % * @brief Structure to store Vehicle 1 simulation data.
        % *
        % * Contains arrays for X and Y positions, orientation (Theta), speed, and steering angle over time.
        % */
        globalVehicle1Data = struct('X', [], 'Y', [], 'Theta', [], 'Speed', [], 'SteeringAngle', []);

        %/**
        % * @property globalTrailer1Data
        % * @brief Structure to store Trailer 1 simulation data.
        % *
        % * Contains arrays for X and Y positions, orientation (Theta), speed, and steering angle over time.
        % */
        globalTrailer1Data = struct('X', [], 'Y', [], 'Theta', [], 'Speed', [], ...
                                    'SteeringAngle', [], 'Boxes', [], ...
                                    'XBoxes', [], 'YBoxes', [], 'ThetaBoxes', []);

        %/**
        % * @property globalVehicle2Data
        % * @brief Structure to store Vehicle 2 simulation data.
        % *
        % * Contains arrays for X and Y positions, orientation (Theta), speed, and steering angle over time.
        % */
        globalVehicle2Data = struct('X', [], 'Y', [], 'Theta', [], 'Speed', [], 'SteeringAngle', []);

        %/**
        % * @property globalTrailer2Data
        % * @brief Structure to store Trailer 2 simulation data.
        % *
        % * Contains arrays for X and Y positions, orientation (Theta), speed, and steering angle over time.
        % */
        globalTrailer2Data = struct('X', [], 'Y', [], 'Theta', [], 'Speed', [], ...
                                    'SteeringAngle', [], 'Boxes', [], ...
                                    'XBoxes', [], 'YBoxes', [], 'ThetaBoxes', []);

        %/**
        % * @property manualControlData
        % * @brief Structure to store Manual Control data.
        % *
        % * Contains arrays for Time and Control Inputs, allowing for the recording and saving of manual interventions.
        % */
        manualControlData = struct('Time', [], 'ControlInputs', []); % Adjust fields as necessary

        %/**
        % * @property collisionData
        % * @brief Structure to store Collision data.
        % *
        % * Contains arrays for X and Y coordinates of collision points at each simulation step.
        % */
        collisionData = struct('X', [], 'Y', []); % Added collisionData property

        %/**
        % * @property TrailerLength1
        % * @brief Length of Trailer 1 in meters.
        % */
        TrailerLength1 = 0; % Length of Trailer 1

        %/**
        % * @property TrailerLength2
        % * @brief Length of Trailer 2 in meters.
        % */
        TrailerLength2 = 0; % Length of Trailer 2
    end

    properties (Access = private)
        %/**
        % * @property uiManager
        % * @brief Reference to the UI Manager.
        % *
        % * Provides access to various UI components for interaction and feedback.
        % */
        uiManager
    end

    methods
        %/**
        % * @brief Constructor to initialize the DataManager with UIManager reference.
        % *
        % * Initializes a new instance of the DataManager class by setting up references to the UI Manager.
        % *
        % * @param uiManager Reference to the UI Manager object.
        % *
        % * @throws Error if the uiManager reference is not provided.
        % */
        function obj = DataManager(uiManager)
            % Constructor initializes the DataManager with UIManager reference.
            if nargin < 1
                error('DataManager requires a uiManager reference.');
            end
            obj.uiManager = uiManager;
        end

        %/**
        % * @brief Loads Vehicle 1 simulation data from an Excel file.
        % *
        % * Interacts with the Vehicle Simulation Configuration object to load simulation parameters from
        % * an Excel file and applies them to the Vehicle Simulation object.
        % *
        % * @param vehicleSimConfig1 Reference to the Vehicle 1 Simulation Configuration object.
        % * @param vehicleSim1 Reference to the Vehicle 1 Simulation object where parameters will be applied.
        % *
        % * @return obj The updated DataManager object.
        % *
        % * @throws Error if loading the Excel file fails.
        % */
        function obj = loadVehicle1Excel(obj, vehicleSimConfig1, vehicleSim1)
            % Load Excel data into Vehicle 1 Simulation
            try
                vehicleSimConfig1.loadExcelFile();    % Load configuration if needed
                vehicleSim1.simParams = vehicleSimConfig1.getSimulationParameters();

                % Ensure 'includeTrailer' field exists
                if ~isfield(vehicleSim1.simParams, 'includeTrailer')
                    vehicleSim1.simParams.includeTrailer = false; % Default value
                    warning('Field "includeTrailer" not found in vehicleSim1.simParams. Defaulting to false.');
                end

                uialert(obj.uiManager.fig, 'Vehicle 1 simulation data loaded successfully.', 'Success');
            catch ME
                uialert(obj.uiManager.fig, ['Failed to load Vehicle 1 simulation data: ' ME.message], 'Load Error');
                rethrow(ME);
            end
        end

        %/**
        % * @brief Loads Vehicle 2 simulation data from an Excel file.
        % *
        % * Interacts with the Vehicle Simulation Configuration object to load simulation parameters from
        % * an Excel file and applies them to the Vehicle Simulation object.
        % *
        % * @param vehicleSimConfig2 Reference to the Vehicle 2 Simulation Configuration object.
        % * @param vehicleSim2 Reference to the Vehicle 2 Simulation object where parameters will be applied.
        % *
        % * @return obj The updated DataManager object.
        % *
        % * @throws Error if loading the Excel file fails.
        % */
        function obj = loadVehicle2Excel(obj, vehicleSimConfig2, vehicleSim2)
            % Load Excel data into Vehicle 2 Simulation
            try
                vehicleSimConfig2.loadExcelFile();    % Load configuration if needed
                vehicleSim2.simParams = vehicleSimConfig2.getSimulationParameters();

                % Ensure 'includeTrailer' field exists
                if ~isfield(vehicleSim2.simParams, 'includeTrailer')
                    vehicleSim2.simParams.includeTrailer = false; % Default value
                    warning('Field "includeTrailer" not found in vehicleSim2.simParams. Defaulting to false.');
                end

                uialert(obj.uiManager.fig, 'Vehicle 2 simulation data loaded successfully.', 'Success');
            catch ME
                uialert(obj.uiManager.fig, ['Failed to load Vehicle 2 simulation data: ' ME.message], 'Load Error');
                rethrow(ME);
            end
        end

        %/**
        % * @brief Creates a structure with trailer parameters for plotting.
        % *
        % * \param simParams Simulation parameters for the trailer.
        % * \param wheelHeight Wheel height obtained from simParams.
        % * \param wheelWidth Wheel width obtained from simParams.
        % * \param trailerNumber Integer indicating which trailer (1 or 2) is being set.
        % * \return trailerParams Structured parameters for the trailer.
        % */
        function trailerParams = createTrailerParams(obj, simParams, wheelHeight, wheelWidth, trailerNumber)
            % Validate trailerNumber
            if trailerNumber ~= 1 && trailerNumber ~= 2
                error('Invalid trailerNumber. It must be 1 or 2.');
            end

            % Check for existence of required fields in simParams
            requiredFields = {'trailerLength', 'trailerWidth', 'trailerHeight', 'trailerCoGHeight', ...
                              'trailerWheelbase', 'trailerTrackWidth', 'trailerNumAxles', ...
                              'trailerAxleSpacing', 'trailerHitchDistance'};

            for i = 1:length(requiredFields)
                if ~isfield(simParams, requiredFields{i})
                    error('simParams is missing required field: %s', requiredFields{i});
                end
            end

            % Use configured tires per axle for trailer
            numTiresPerAxle = simParams.numTiresPerAxleTrailer;
            boxNumAxles = simParams.trailerAxlesPerBox;
            numAxles = sum(boxNumAxles);

            % Compute trailer mass from weight distributions when available
            if isfield(simParams, 'baseTrailerMass')
                massVal = simParams.baseTrailerMass;
            else
                massVal = simParams.trailerMass;
            end
            boxMasses = [];
            if isfield(simParams,'trailerBoxWeightDistributions') && ~isempty(simParams.trailerBoxWeightDistributions)
                boxMasses = cellfun(@(ld) sum(ld(:,4))/9.81, simParams.trailerBoxWeightDistributions);
                massVal = sum(boxMasses);
            elseif isfield(simParams,'trailerNumBoxes') && simParams.trailerNumBoxes > 1
                massVal = massVal * simParams.trailerNumBoxes;
            end

            trailerParams = struct(...
                'isTractor', false, ...
                'length', simParams.trailerLength, ...
                'width', simParams.trailerWidth, ...
                'height', simParams.trailerHeight, ...
                'CoGHeight', simParams.trailerCoGHeight, ...
                'wheelbase', simParams.trailerWheelbase, ...
                'trackWidth', simParams.trailerTrackWidth, ...
                'numAxles', numAxles, ...
                'boxNumAxles', boxNumAxles, ...
                'axleSpacing', simParams.trailerAxleSpacing, ...
                'mass', massVal, ...
                'boxMasses', boxMasses, ...
                'wheelWidth', wheelWidth, ...    % Retrieved from simParams
                'wheelHeight', wheelHeight, ...  % Retrieved from simParams
                'numTiresPerAxle', numTiresPerAxle, ...
                'HitchDistance', simParams.trailerHitchDistance, ... % Distance from hitch to first axle
                'W_Total', massVal, ...
                'includeTrailer', simParams.includeTrailer ...
            );

            % Set the appropriate TrailerLength property
            if trailerNumber == 1
                obj.TrailerLength1 = simParams.trailerLength;
            elseif trailerNumber == 2
                obj.TrailerLength2 = simParams.trailerLength;
            end
        end

        %/**
        % * @brief Sets the length of Trailer 1.
        % *
        % * \param length Positive numeric value representing the length of Trailer 1 in meters.
        % */
        function setTrailerLength1(obj, length)
            if isnumeric(length) && length > 0
                obj.TrailerLength1 = length;
            else
                error('Invalid TrailerLength1. It must be a positive numeric value.');
            end
        end

        %/**
        % * @brief Sets the length of Trailer 2.
        % *
        % * \param length Positive numeric value representing the length of Trailer 2 in meters.
        % */
        function setTrailerLength2(obj, length)
            if isnumeric(length) && length > 0
                obj.TrailerLength2 = length;
            else
                error('Invalid TrailerLength2. It must be a positive numeric value.');
            end
        end

        %/**
        % * @brief Loads simulation parameters and sets trailer lengths based on vehicle simulations.
        % *
        % * \param vehicleSim1 Reference to Vehicle Simulation 1 object.
        % * \param vehicleSim2 Reference to Vehicle Simulation 2 object.
        % */
        function loadSimulationParameters(obj, vehicleSim1, vehicleSim2)
            % Set Trailer Lengths based on simulation parameters

            % Vehicle 1
            if isfield(vehicleSim1.simParams, 'trailerLength') && ...
               isfield(vehicleSim1.simParams, 'includeTrailer') && vehicleSim1.simParams.includeTrailer
                obj.TrailerLength1 = vehicleSim1.simParams.trailerLength;
            else
                obj.TrailerLength1 = 0; % Default or handle accordingly
                warning('VehicleSim1 does not include a trailer or trailerLength is not defined.');
            end

            % Vehicle 2
            if isfield(vehicleSim2.simParams, 'trailerLength') && ...
               isfield(vehicleSim2.simParams, 'includeTrailer') && vehicleSim2.simParams.includeTrailer
                obj.TrailerLength2 = vehicleSim2.simParams.trailerLength;
            else
                obj.TrailerLength2 = 0; % Default or handle accordingly
                warning('VehicleSim2 does not include a trailer or trailerLength is not defined.');
            end
        end

        %/**
        % * @brief Saves Manual Control data to an Excel file.
        % *
        % * If manual control data exists, prompts the user to select a save location and writes the data to a `.xlsx` file.
        % *
        % * @return None
        % *
        % * @warning If there is no manual control data to save, the method notifies the user.
        % */
        function saveManualControlData(obj)
            if ~isempty(obj.manualControlData.Time)
                [file, path] = uiputfile('*.xlsx', 'Save Manual Control Data');
                if isequal(file,0) || isequal(path,0)
                    disp('User canceled Manual Control Data save.');
                else
                    try
                        manualTable = struct2table(obj.manualControlData);
                        writetable(manualTable, fullfile(path, file));
                        uialert(obj.uiManager.fig, 'Manual control data saved successfully.', 'Save Success');
                        fprintf('Manual control data saved to %s.\n', fullfile(path, file));
                    catch ME
                        uialert(obj.uiManager.fig, ['Failed to save Manual control data: ' ME.message], 'Save Error');
                    end
                end
            else
                disp('No manual control data to save.');
                uialert(obj.uiManager.fig, 'No manual control data available to save.', 'No Data');
            end
        end

        %/**
        % * @brief Saves the current plot as a PNG image.
        % *
        % * Prompts the user to select a save location and exports the provided axes handle as a high-resolution `.png` image.
        % *
        % * \param sharedAx Handle to the axes containing the plot to be saved.
        % *
        % * @return None
        % *
        % * @warning If the user cancels the save operation, the method exits without saving.
        % */
        function savePlotAsPNG(obj, sharedAx)
            [file, path] = uiputfile('*.png', 'Save Plot Image');
            if isequal(file,0) || isequal(path,0)
                disp('User canceled PNG save.');
            else
                try
                    exportgraphics(sharedAx, fullfile(path, file), 'Resolution', 300);
                    uialert(obj.uiManager.fig, 'Plot image saved successfully.', 'Save Success');
                    fprintf('Plot image saved to %s.\n', fullfile(path, file));
                catch ME
                    uialert(obj.uiManager.fig, ['Failed to save plot image: ' ME.message], 'Save Error');
                end
            end
        end

        %/**
        % * @brief Saves comprehensive simulation results to an Excel file.
        % *
        % * Compiles simulation data from both Vehicles and their Trailers into a structured table and writes it to a `.xlsx` file.
        % *
        % * @return None
        % *
        % * @warning If saving fails due to file permissions or data inconsistencies, the method notifies the user.
        % */
        function saveSimulationResults(obj)
            [file, path] = uiputfile('*.xlsx', 'Save Simulation Results');
            if isequal(file,0) || isequal(path,0)
                disp('User canceled Excel save.');
            else
                try
                    % Compile simulation data into a table using global data structures
                    tractor1X = obj.globalVehicle1Data.X(:);
                    tractor1Y = obj.globalVehicle1Data.Y(:);
                    tractor1Theta = obj.globalVehicle1Data.Theta(:);
                    tractor1Speed = obj.globalVehicle1Data.Speed(:);
                    tractor1Steering = obj.globalVehicle1Data.SteeringAngle(:);

                    tractor2X = obj.globalVehicle2Data.X(:);
                    tractor2Y = obj.globalVehicle2Data.Y(:);
                    tractor2Theta = obj.globalVehicle2Data.Theta(:);
                    tractor2Speed = obj.globalVehicle2Data.Speed(:);
                    tractor2Steering = obj.globalVehicle2Data.SteeringAngle(:);

                    % Initialize trailer data arrays
                    trailer1X = [];
                    trailer1Y = [];
                    trailer1Theta = [];
                    trailer1Speed = [];
                    trailer1Steering = [];

                    trailer2X = [];
                    trailer2Y = [];
                    trailer2Theta = [];
                    trailer2Speed = [];
                    trailer2Steering = [];

                    % Check if trailer data exists
                    if ~isempty(obj.globalTrailer1Data.X)
                        trailer1X = obj.globalTrailer1Data.X(:);
                        trailer1Y = obj.globalTrailer1Data.Y(:);
                        trailer1Theta = obj.globalTrailer1Data.Theta(:);
                        trailer1Speed = obj.globalTrailer1Data.Speed(:);
                        trailer1Steering = obj.globalTrailer1Data.SteeringAngle(:);
                    end

                    if ~isempty(obj.globalTrailer2Data.X)
                        trailer2X = obj.globalTrailer2Data.X(:);
                        trailer2Y = obj.globalTrailer2Data.Y(:);
                        trailer2Theta = obj.globalTrailer2Data.Theta(:);
                        trailer2Speed = obj.globalTrailer2Data.Speed(:);
                        trailer2Steering = obj.globalTrailer2Data.SteeringAngle(:);
                    end

                    % Determine the number of data points
                    numDataPoints = max([length(tractor1X), length(tractor1Y), length(trailer1X), ...
                                         length(trailer1Y), length(tractor2X), length(tractor2Y), ...
                                         length(trailer2X), length(trailer2Y), ...
                                         length(tractor1Speed), length(trailer1Speed), ...
                                         length(tractor2Speed), length(trailer2Speed), ...
                                         length(tractor1Steering), length(trailer1Steering), ...
                                         length(tractor2Steering), length(trailer2Steering)]);

                    % Function to pad arrays with NaN
                    padWithNaN = @(data, targetLength) [data; NaN(targetLength - length(data), 1)];

                    % Pad each array to have the same number of data points
                    tractor1X = padWithNaN(tractor1X, numDataPoints);
                    tractor1Y = padWithNaN(tractor1Y, numDataPoints);
                    tractor1Theta = padWithNaN(tractor1Theta, numDataPoints);
                    tractor1Speed = padWithNaN(tractor1Speed, numDataPoints);
                    tractor1Steering = padWithNaN(tractor1Steering, numDataPoints);

                    tractor2X = padWithNaN(tractor2X, numDataPoints);
                    tractor2Y = padWithNaN(tractor2Y, numDataPoints);
                    tractor2Theta = padWithNaN(tractor2Theta, numDataPoints);
                    tractor2Speed = padWithNaN(tractor2Speed, numDataPoints);
                    tractor2Steering = padWithNaN(tractor2Steering, numDataPoints);

                    trailer1X = padWithNaN(trailer1X, numDataPoints);
                    trailer1Y = padWithNaN(trailer1Y, numDataPoints);
                    trailer1Theta = padWithNaN(trailer1Theta, numDataPoints);
                    trailer1Speed = padWithNaN(trailer1Speed, numDataPoints);
                    trailer1Steering = padWithNaN(trailer1Steering, numDataPoints);

                    trailer2X = padWithNaN(trailer2X, numDataPoints);
                    trailer2Y = padWithNaN(trailer2Y, numDataPoints);
                    trailer2Theta = padWithNaN(trailer2Theta, numDataPoints);
                    trailer2Speed = padWithNaN(trailer2Speed, numDataPoints);
                    trailer2Steering = padWithNaN(trailer2Steering, numDataPoints);

                    % Compute the Time vector
                    Time = (0:numDataPoints-1)' * obj.dt; % Starts at 0

                    % Create a table with all the required columns
                    results = table(Time, ...
                                   tractor1X, tractor1Y, tractor1Theta, tractor1Speed, tractor1Steering, ...
                                   trailer1X, trailer1Y, trailer1Theta, trailer1Speed, trailer1Steering, ...
                                   tractor2X, tractor2Y, tractor2Theta, tractor2Speed, tractor2Steering, ...
                                   trailer2X, trailer2Y, trailer2Theta, trailer2Speed, trailer2Steering, ...
                                   'VariableNames', {'Time_s', ...
                                                     'Tractor1_X_m', 'Tractor1_Y_m', 'Tractor1_Theta_rad', 'Tractor1_Speed_m_s', 'Tractor1_SteeringAngle_rad', ...
                                                     'Trailer1_X_m', 'Trailer1_Y_m', 'Trailer1_Theta_rad', 'Trailer1_Speed_m_s', 'Trailer1_SteeringAngle_rad', ...
                                                     'Tractor2_X_m', 'Tractor2_Y_m', 'Tractor2_Theta_rad', 'Tractor2_Speed_m_s', 'Tractor2_SteeringAngle_rad', ...
                                                     'Trailer2_X_m', 'Trailer2_Y_m', 'Trailer2_Theta_rad', 'Trailer2_Speed_m_s', 'Trailer2_SteeringAngle_rad'});

                    % Write the table to an Excel file
                    writetable(results, fullfile(path, file));

                    % Notify the user of successful save
                    uialert(obj.uiManager.fig, 'Simulation results saved successfully.', 'Save Success');
                    fprintf('Simulation results saved to %s.\n', fullfile(path, file));
                catch ME
                    uialert(obj.uiManager.fig, ['Failed to save simulation results: ' ME.message], 'Save Error');
                end
            end
        end
    end
end
