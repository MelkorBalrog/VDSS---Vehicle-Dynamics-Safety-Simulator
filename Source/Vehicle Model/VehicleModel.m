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
%/**
% * @file VehicleModel.m
% * @brief Simulates a Vehicle Model with integrated road conditions such as slope and friction.
% */
% * @author Miguel Marina <karel.capek.robotics@gmail.com>
classdef VehicleModel < handle
    % VehicleModel Simulates a Vehicle Model with integrated road conditions.
    %
    % This class includes functionality to load simulation parameters from a
    % GUI or an Excel file, run the simulation, and manage various controllers
    % and models associated with the vehicle's dynamics.

    properties
        hasUI
        guiManager
        filename
        simParams
        pid_SpeedController      % Instance of pid_SpeedController
        limiter_LateralControl   % Instance of limiter_LateralControl
        limiter_LongitudinalControl  % Instance of limiter_LongitudinalControl
        jerkController              % Instance of jerk_Controller for jerk limiting
        accController               % Instance of acc_Controller for ACC
        localizer                  % Instance of VehicleLocalizer for map-based localization
        curveSpeedLimiter           % Instance of curveSpeed_Limiter
        simulationName
        uiManager
    end

    methods (Access = public)
        %% Constructor
        function obj = VehicleModel(parent, ~, createUI, simulationName, uiManager)
            % VehicleModel Constructor for VehicleModel class
            %
            % Parameters:
            %   parent    - Parent UI component
            %   ~         - Placeholder for future parameters
            %   createUI  - Boolean flag to create UI (default: true)

            if nargin < 3
                createUI = true;
            end

            obj.hasUI = createUI;
            obj.simParams = struct();
            obj.simParams.excelData = []; % Initialize excelData field
            obj.simulationName = simulationName;

            if nargin == 5
                obj.uiManager = uiManager;
            end

            if createUI
                obj.guiManager = VehicleGUIManager(parent, obj);
            else
                obj.initializeDefaultParameters();
            end
        end
        
        %% Initialize Default Parameters
        function obj = initializeDefaultParameters(obj)
            % initializeDefaultParameters Initializes default simulation parameters
            
            % --- Basic Configuration ---
            obj.simParams.includeTrailer = true; % Include trailer by default
            obj.simParams.tractorMass = 8000; % kg (empty tractor mass)
            obj.simParams.trailerMass = 7000; % kg
            obj.simParams.baseTrailerMass = obj.simParams.trailerMass; % store unscaled mass
            obj.simParams.trailerMassScaled = false; % flag for scaling when no weight distribution
            obj.simParams.initialVelocity = 10; % m/s
            obj.simParams.I_trailerMultiplier = 1; % Multiplier for inertia
            obj.simParams.maxDeltaDeg = 70; % degrees
            obj.simParams.dtMultiplier = 0.5; % Time step multiplier
            obj.simParams.windowSize = 10; % For moving average
            
            % --- Tractor Parameters ---
            obj.simParams.tractorLength = 6.5; % meters
            obj.simParams.tractorWidth = 2.5; % meters
            obj.simParams.tractorHeight = 3.8; % meters
            obj.simParams.tractorCoGHeight = 1.5; % meters
            obj.simParams.tractorWheelbase = 4.0; % meters
            obj.simParams.tractorTrackWidth = 2.1; % meters
            obj.simParams.tractorNumAxles = 3;
            obj.simParams.tractorAxleSpacing = 1.310; % meters
            
            % **New Parameter for Tractor**
            obj.simParams.numTiresPerAxleTractor = 2; % Default to 2 tires per axle for tractor
            
            % --- Trailer Parameters ---
            obj.simParams.trailerLength = 12.0; % meters
            obj.simParams.trailerWidth = 2.5; % meters
            obj.simParams.trailerHeight = 4.0; % meters
            obj.simParams.trailerCoGHeight = 1.5; % meters
            obj.simParams.trailerWheelbase = 8.0; % meters
            obj.simParams.trailerTrackWidth = 2.1; % meters
            obj.simParams.trailerAxlesPerBox = [2];
            obj.simParams.trailerNumAxles = sum(obj.simParams.trailerAxlesPerBox);
            obj.simParams.trailerNumBoxes = numel(obj.simParams.trailerAxlesPerBox);
            obj.simParams.trailerAxleSpacing = 1.310; % meters
            obj.simParams.trailerHitchDistance = 1.310; % meters
            obj.simParams.tractorHitchDistance = 4.5; % meters
            obj.simParams.numTiresPerAxleTrailer = 4;
            
            % --- Control Limits Parameters ---
            obj.simParams.maxSteeringAngleAtZeroSpeed = 30; % degrees
            % obj.simParams.minSteeringAngleAtMaxSpeed = 10;  % degrees
            obj.simParams.steeringCurveFilePath = "SteeringCurve.xlsx";
            obj.simParams.maxSteeringSpeed = 30;            % m/s
            
            % --- Acceleration Limiter Parameters ---
            % obj.simParams.maxAccelAtZeroSpeed = 3.0;   % m/s^2
            obj.simParams.minAccelAtMaxSpeed = 1.0;    % m/s^2
            % obj.simParams.maxDecelAtZeroSpeed = -3.0;  % m/s^2
            obj.simParams.minDecelAtMaxSpeed = -1.0;   % m/s^2
            obj.simParams.accelCurveFilePath = "AccelCurve.xlsx";
            obj.simParams.decelCurveFilePath = "DecelCurve.xlsx";
            obj.simParams.maxSpeedForAccelLimiting = 30.0; % m/s
            % --- End of Acceleration Limiter Parameters ---
            
            % --- PID Speed Controller Parameters ---
            obj.simParams.Kp = 1.0;  % Proportional gain
            obj.simParams.Ki = 0.5;  % Integral gain
            obj.simParams.Kd = 0.1;  % Derivative gain
            obj.simParams.lambda1Accel = 1.0; % Levant differentiator lambda1 for error derivative
            obj.simParams.lambda2Accel = 1.0; % Levant differentiator lambda2 for error derivative
            obj.simParams.lambda1Vel = 1.0; % Levant differentiator lambda1 for velocity
            obj.simParams.lambda2Vel = 1.0; % Levant differentiator lambda2 for velocity
            obj.simParams.lambda1Jerk = 1.0; % Levant differentiator lambda1 for jerk
            obj.simParams.lambda2Jerk = 1.0; % Levant differentiator lambda2 for jerk
            obj.simParams.enableSpeedController = true;
            % --- End of PID Speed Controller Parameters ---
            
            % --- Speed Limiting Parameter ---
            obj.simParams.maxSpeed = 25.0; % m/s (Average speed limit for vehicles)
            % --- End of Speed Limiting Parameter ---
            
            % --- Tires Configuration Parameters ---
            obj.simParams.tractorTireHeight = 0.5; % meters
            obj.simParams.tractorTireWidth = 0.2;  % meters
            obj.simParams.trailerTireHeight = 0.5; % meters
            obj.simParams.trailerTireWidth = 0.2;  % meters
            % --- End of Tires Configuration Parameters ---
            
            % --- Aerodynamics Parameters ---
            obj.simParams.airDensity = 1.225;     % kg/m³; standard air density at sea level
            obj.simParams.dragCoeff = 0.8;        % Example drag coefficient; adjust based on vehicle's aerodynamics
            % --- End of Aerodynamics Parameters ---
            
            % --- Road Conditions Parameters ---
            obj.simParams.slopeAngle = 0;             % degrees
            obj.simParams.roadFrictionCoefficient = 0.9;  % μ (default value)
            obj.simParams.roadSurfaceType = 'Dry Asphalt';
            obj.simParams.roadRoughness = 0;          % 0 (smooth) to 1 (very rough)
            % --- End of Road Conditions Parameters ---
            
            % --- Pressure Matrices Initialization ---
            obj.initializePressureMatrices();
            % --- End of Pressure Matrices Initialization ---
            
            % --- Suspension Model Parameters ---
            obj.simParams.K_spring = 300000;   % Spring stiffness (N/m)
            obj.simParams.C_damping = 15000;   % Damping coefficient (N·s/m)
            obj.simParams.restLength = 0.5;    % Rest length (m)
            % --- End of Suspension Model Parameters ---
            
            % --- Wind Parameters ---
            obj.simParams.windSpeed = 5;         % m/s
            obj.simParams.windAngleDeg = 45;     % degrees
            % --- End of Wind Parameters ---
            
            % --- Brake Configuration Parameters (Updated) ---
            obj.simParams.brakingForce = 50000;    % N
            obj.simParams.brakeEfficiency = 85;     % %
            obj.simParams.brakeBias = 50;           % % (Front/Rear)
            obj.simParams.brakeType = 'Disk';       % ** New Parameter: Default Brake Type **
            obj.simParams.maxBrakingForce = 60000;
            % --- End of Brake Configuration Parameters ---
            
            % --- Transmission Parameters ---
            obj.simParams.maxGear = 12; % Number of gears
            obj.simParams.gearRatios = [14.94, 11.21, 8.31, 6.26, 4.63, 3.47, 2.54, 1.84, 1.34, 1.00, 0.78, 0.64]; % Gear ratios
            obj.simParams.finalDriveRatio = 3.42; % Final drive ratio
            obj.simParams.shiftUpSpeed = [5, 8, 11, 15, 20, 25, 30, 35, 40, 45, 50, 55]; % Upshift speeds
            obj.simParams.shiftDownSpeed = [3, 6, 9, 13, 18, 23, 28, 33, 38, 43, 48, 53]; % Downshift speeds
            obj.simParams.engineBrakeTorque = 1500; % Engine braking torque in Nm
            obj.simParams.shiftDelay = 0.75; % Shift delay in seconds
            % --- End of Transmission Parameters ---
            
            % --- Flat Tire Indices ---
            obj.simParams.flatTireIndices = []; % e.g., [1, 3] to indicate tires 1 and 3 are flat
            % --- End of Flat Tire Indices ---
            
            % --- Vehicle Type Parameter (New) ---
            obj.simParams.vehicleType = 'Passenger Vehicle'; % ** New Parameter: Default Vehicle Type **
            % --- End of Vehicle Type Parameter ---
            % --- *** New Command Parameters *** ---
            % Steering Commands
            obj.simParams.steeringCommands = '';         % Steering commands (default empty)
            
            % Acceleration Commands
            obj.simParams.accelerationCommands = '';     % Acceleration commands (default empty)

            obj.simParams.tirePressureCommands = '';
            % --- *** End of New Command Parameters *** ---
            obj.simParams.torqueFileName = "torque_curve.xlsx"; % default configuration file

            obj.simParams.maxClutchTorque = 3500;           % Maximum torque the clutch can handle (Nm), typical for heavy-duty trucks
            obj.simParams.engagementSpeed = 1.0;            % Time to engage the clutch (seconds), assuming gradual engagement
            obj.simParams.disengagementSpeed = 0.5;         % Time to disengage the clutch (seconds), assuming faster disengagement
            
            obj.simParams.mapCommands = 'straight(100,200,300,200)|curve(300,250,50,270,90,ccw)|straight(300,300,100,300)|curve(100,250,50,90,270,ccw)';
            obj.simParams.waypoints = [
                100, 200; 
                118, 200; 
                144, 200; 
                166, 200; 
                188, 200; 
                211, 200; 
                233, 200; 
                255, 200; 
                277, 200; 
                300, 200; 
                325, 206; 
                343, 225; 
                350, 250; 
                343, 275; 
                325, 293; 
                300, 300; 
                277, 300; 
                255, 300; 
                233, 300; 
                211, 300; 
                188, 300; 
                166, 300; 
                144, 300; 
                122, 300; 
                100, 300; 
                100, 200; 
                75, 206; 
                56, 225; 
                50, 250; 
                56, 275; 
                75, 293; 
                100, 300
            ];

            obj.simParams.waypointsX = 0;
            obj.simParams.waypointsY = 0;
        end


        % %% Load Excel File
        % function loadExcelFile(obj)
        %     % loadExcelFile Loads simulation data from an Excel file
        % 
        %     if ~obj.hasUI
        %         error('GUI is disabled. Cannot load Excel file.');
        %     end
        %     [file, path] = uigetfile('*.xlsx', 'Select Excel File');
        %     if isequal(file, 0)
        %         disp('User selected Cancel');
        %     else
        %         obj.filename = fullfile(path, file);
        %         disp(['User selected ', obj.filename]);
        %         try
        %             data = readtable(obj.filename, 'VariableNamingRule', 'preserve');
        %         catch ME
        %             uialert(obj.guiManager.tablePanel.Parent, ['Error reading Excel file: ' ME.message], 'Read Error');
        %             return;
        %         end
        %         cellData = table2cell(data);
        %         columnNames = data.Properties.VariableNames;
        %         set(obj.guiManager.excelTable, 'Data', cellData, 'ColumnName', columnNames);
        %         obj.simParams.excelData = data;
        %     end
        % end
                
        %% Set Simulation Parameters
        function setSimulationParameters(obj, simParams)
            % setSimulationParameters Sets the simulation parameters for the vehicle model
            %
            % Parameters:
            %   simParams - Structure containing simulation parameters to set
            %
            % This method updates the 'simParams' property of the VehicleModel instance with the provided parameters.
            % If the GUI is enabled, it also updates the GUI fields to reflect the new parameters.
        
            % Preserve existing excelData if it's not part of the new simParams
            if isfield(obj.simParams, 'excelData') && ~isfield(simParams, 'excelData')
                simParams.excelData = obj.simParams.excelData;
            end
        
            % Ensure baseTrailerMass and scaling flag exist
            if isfield(simParams, 'baseTrailerMass')
                baseMass = simParams.baseTrailerMass;
            else
                if isfield(simParams, 'trailerMassScaled') && simParams.trailerMassScaled && isfield(simParams,'trailerNumBoxes') && simParams.trailerNumBoxes > 1
                    baseMass = simParams.trailerMass / simParams.trailerNumBoxes;
                else
                    baseMass = simParams.trailerMass;
                end
                simParams.baseTrailerMass = baseMass;
            end
            if ~isfield(simParams, 'trailerMassScaled')
                simParams.trailerMassScaled = false;
            end
            obj.simParams = simParams;
        
            % If GUI is enabled, update the GUI fields with the new parameters
            if obj.hasUI && ~isempty(obj.guiManager)
                %% --- Basic Configuration ---
                obj.guiManager.includeTrailerCheckbox.Value = simParams.includeTrailer;
                obj.guiManager.tractorMassField.Value = simParams.tractorMass;
                obj.guiManager.velocityField.Value = simParams.initialVelocity;

                % If simParams contains Nx2 numeric waypoints, populate UITABLE using a loop
                if isfield(simParams, 'waypoints') && ~isempty(simParams.waypoints)
                    W = simParams.waypoints; % Nx2 numeric array
                    [numRows, numCols] = size(W);
                    cellData = cell(numRows, numCols);
                
                    % Populate cellData using a loop
                    for r = 1:numRows
                        for c = 1:numCols
                            cellData{r, c} = W(r, c);
                        end
                    end
                
                    obj.guiManager.waypointsTable.Data = cellData;
                end
        
                %% --- Advanced Configuration ---
                obj.guiManager.I_trailerMultiplierField.Value = simParams.I_trailerMultiplier;
                obj.guiManager.maxDeltaField.Value = simParams.maxDeltaDeg;
                obj.guiManager.dtMultiplierField.Value = simParams.dtMultiplier;
                obj.guiManager.windowSizeField.Value = simParams.windowSize;
        
                %% --- Tractor Parameters ---
                obj.guiManager.tractorLengthField.Value = simParams.tractorLength;
                obj.guiManager.tractorWidthField.Value = simParams.tractorWidth;
                obj.guiManager.tractorHeightField.Value = simParams.tractorHeight;
                obj.guiManager.tractorCoGHeightField.Value = simParams.tractorCoGHeight;
                obj.guiManager.tractorWheelbaseField.Value = simParams.tractorWheelbase;
                obj.guiManager.tractorTrackWidthField.Value = simParams.tractorTrackWidth;
                obj.guiManager.tractorNumAxlesDropdown.Value = num2str(simParams.tractorNumAxles);
                obj.guiManager.tractorAxleSpacingField.Value = simParams.tractorAxleSpacing;
                obj.guiManager.numTiresPerAxleTractorDropDown.Value = num2str(simParams.numTiresPerAxleTractor);
        
                %% --- Trailer Parameters ---
                obj.guiManager.trailerLengthField.Value = simParams.trailerLength;
                obj.guiManager.trailerWidthField.Value = simParams.trailerWidth;
                obj.guiManager.trailerHeightField.Value = simParams.trailerHeight;
                obj.guiManager.trailerCoGHeightField.Value = simParams.trailerCoGHeight;
                obj.guiManager.trailerWheelbaseField.Value = simParams.trailerWheelbase;
                obj.guiManager.trailerTrackWidthField.Value = simParams.trailerTrackWidth;
                if isprop(obj.guiManager, 'trailerNumAxlesDropdown')
                    obj.guiManager.trailerNumAxlesDropdown.Value = num2str(simParams.trailerNumAxles);
                end
                obj.guiManager.trailerAxleSpacingField.Value = simParams.trailerAxleSpacing;
                obj.guiManager.trailerHitchDistanceField.Value = simParams.trailerHitchDistance;
                obj.guiManager.tractorHitchDistanceField.Value = simParams.tractorHitchDistance;
                obj.guiManager.numTiresPerAxleTrailerDropDown.Value = num2str(simParams.numTiresPerAxleTrailer);

                obj.guiManager.maxClutchTorqueField.Value = simParams.maxClutchTorque;
                obj.guiManager.engagementSpeedField.Value = simParams.engagementSpeed;
                obj.guiManager.disengagementSpeedField.Value = simParams.disengagementSpeed;

                obj.guiManager.torqueFileNameField.Value = simParams.torqueFileName;
        
                %% --- Control Limits Parameters ---
                if isprop(obj.guiManager, 'steeringCurveFilePathField') && ...
                   isprop(obj.guiManager, 'maxSteeringSpeedField')
                    obj.guiManager.maxSteeringAngleAtZeroSpeedField.Value = simParams.maxSteeringAngleAtZeroSpeed;
                    % obj.guiManager.minSteeringAngleAtMaxSpeedField.Value = simParams.minSteeringAngleAtMaxSpeed;
                    obj.guiManager.steeringCurveFilePathField.Value = simParams.steeringCurveFilePath;
                    obj.guiManager.maxSteeringSpeedField.Value = simParams.maxSteeringSpeed;
                end
        
                % obj.guiManager.maxAccelAtZeroSpeedField.Value = simParams.maxAccelAtZeroSpeed;
                obj.guiManager.minAccelAtMaxSpeedField.Value = simParams.minAccelAtMaxSpeed;
                % obj.guiManager.maxDecelAtZeroSpeedField.Value = simParams.maxDecelAtZeroSpeed;
                obj.guiManager.minDecelAtMaxSpeedField.Value = simParams.minDecelAtMaxSpeed;
                obj.guiManager.accelCurveFilePathField.Value = simParams.accelCurveFilePath;
                obj.guiManager.decelCurveFilePathField.Value = simParams.decelCurveFilePath;
                obj.guiManager.maxSpeedForAccelLimitingField.Value = simParams.maxSpeedForAccelLimiting;
        
                %% --- Maximum Speed Parameter ---
                if isprop(obj.guiManager, 'maxSpeedField')
                    obj.guiManager.maxSpeedField.Value = simParams.maxSpeed;
                end
        
                %% --- PID Speed Controller Parameters ---
                if isprop(obj.guiManager, 'KpField') && isprop(obj.guiManager, 'KiField') && isprop(obj.guiManager, 'KdField') && isprop(obj.guiManager, 'enableSpeedControllerCheckbox')
                    obj.guiManager.KpField.Value = simParams.Kp;
                    obj.guiManager.KiField.Value = simParams.Ki;
                    obj.guiManager.KdField.Value = simParams.Kd;
                    if isprop(obj.guiManager, 'lambda1AccelField') && isprop(obj.guiManager, 'lambda2AccelField')
                        obj.guiManager.lambda1AccelField.Value = simParams.lambda1Accel;
                        obj.guiManager.lambda2AccelField.Value = simParams.lambda2Accel;
                    end
                    if isprop(obj.guiManager, 'lambda1VelField') && isprop(obj.guiManager, 'lambda2VelField')
                        obj.guiManager.lambda1VelField.Value = simParams.lambda1Vel;
                        obj.guiManager.lambda2VelField.Value = simParams.lambda2Vel;
                    end
                    if isprop(obj.guiManager, 'lambda1JerkField') && isprop(obj.guiManager, 'lambda2JerkField')
                        obj.guiManager.lambda1JerkField.Value = simParams.lambda1Jerk;
                        obj.guiManager.lambda2JerkField.Value = simParams.lambda2Jerk;
                    end
                    obj.guiManager.enableSpeedControllerCheckbox.Value = simParams.enableSpeedController;
                end
        
                %% --- Tires Configuration Parameters ---
                if isprop(obj.guiManager, 'tractorTireHeightField') && ...
                   isprop(obj.guiManager, 'tractorTireWidthField') && ...
                   isprop(obj.guiManager, 'trailerTireHeightField') && ...
                   isprop(obj.guiManager, 'trailerTireWidthField')
                    obj.guiManager.tractorTireHeightField.Value = simParams.tractorTireHeight;
                    obj.guiManager.tractorTireWidthField.Value = simParams.tractorTireWidth;
                    obj.guiManager.trailerTireHeightField.Value = simParams.trailerTireHeight;
                    obj.guiManager.trailerTireWidthField.Value = simParams.trailerTireWidth;
                end

                %% --- Commands Parameters ---  % *** New Section ***
                if isprop(obj.guiManager, 'steeringCommandsBox') && ...
                   isprop(obj.guiManager, 'accelerationCommandsBox')&& ...
                   isprop(obj.guiManager, 'tirePressureCommandsBox')
                    obj.guiManager.steeringCommandsBox.Value = simParams.steeringCommands;
                    obj.guiManager.accelerationCommandsBox.Value = simParams.accelerationCommands;
                    obj.guiManager.tirePressureCommandsBox.Value = simParams.tirePressureCommands;
                end
                % *** End of Commands Parameters ***
        
                %% --- Stiffness & Damping Parameters ---
                if isprop(obj.guiManager, 'stiffnessXField') && ...
                   isprop(obj.guiManager, 'stiffnessYField') && ...
                   isprop(obj.guiManager, 'stiffnessZField') && ...
                   isprop(obj.guiManager, 'stiffnessRollField') && ...
                   isprop(obj.guiManager, 'stiffnessPitchField') && ...
                   isprop(obj.guiManager, 'stiffnessYawField') && ...
                   isprop(obj.guiManager, 'dampingXField') && ...
                   isprop(obj.guiManager, 'dampingYField') && ...
                   isprop(obj.guiManager, 'dampingZField') && ...
                   isprop(obj.guiManager, 'dampingRollField') && ...
                   isprop(obj.guiManager, 'dampingPitchField') && ...
                   isprop(obj.guiManager, 'dampingYawField')
                    obj.guiManager.stiffnessXField.Value = simParams.stiffnessX;
                    obj.guiManager.stiffnessYField.Value = simParams.stiffnessY;
                    obj.guiManager.stiffnessZField.Value = simParams.stiffnessZ;
                    obj.guiManager.stiffnessRollField.Value = simParams.stiffnessRoll;
                    obj.guiManager.stiffnessPitchField.Value = simParams.stiffnessPitch;
                    obj.guiManager.stiffnessYawField.Value = simParams.stiffnessYaw;
        
                    obj.guiManager.dampingXField.Value = simParams.dampingX;
                    obj.guiManager.dampingYField.Value = simParams.dampingY;
                    obj.guiManager.dampingZField.Value = simParams.dampingZ;
                    obj.guiManager.dampingRollField.Value = simParams.dampingRoll;
                    obj.guiManager.dampingPitchField.Value = simParams.dampingPitch;
                    obj.guiManager.dampingYawField.Value = simParams.dampingYaw;
                end
        
                %% --- Aerodynamics Parameters ---
                if isprop(obj.guiManager, 'airDensityField') && isprop(obj.guiManager, 'dragCoeffField') && ...
                   isprop(obj.guiManager, 'windSpeedField') && isprop(obj.guiManager, 'windAngleDegField')
                    obj.guiManager.airDensityField.Value = simParams.airDensity;
                    obj.guiManager.dragCoeffField.Value = simParams.dragCoeff;
                    obj.guiManager.windSpeedField.Value = simParams.windSpeed;
                    obj.guiManager.windAngleDegField.Value = simParams.windAngleDeg;
                    obj.guiManager.windAngleRad = deg2rad(simParams.windAngleDeg);  % *** New: Wind Angle in Radians ***
                end
        
                %% --- Road Conditions Parameters ---
                if isprop(obj.guiManager, 'slopeAngleField') && ...
                   isprop(obj.guiManager, 'roadFrictionCoefficientField') && ...
                   isprop(obj.guiManager, 'roadSurfaceTypeDropdown') && ...
                   isprop(obj.guiManager, 'roadRoughnessField')
                    obj.guiManager.slopeAngleField.Value = simParams.slopeAngle;
                    obj.guiManager.roadFrictionCoefficientField.Value = simParams.roadFrictionCoefficient;
                    obj.guiManager.roadSurfaceTypeDropdown.Value = simParams.roadSurfaceType;
                    obj.guiManager.roadRoughnessField.Value = simParams.roadRoughness;
                end
        
                %% --- Suspension Model Parameters ---
                if isprop(obj.guiManager, 'K_springField') && ...
                   isprop(obj.guiManager, 'C_dampingField') && ...
                   isprop(obj.guiManager, 'restLengthField')
                    obj.guiManager.K_springField.Value = simParams.K_spring;
                    obj.guiManager.C_dampingField.Value = simParams.C_damping;
                    obj.guiManager.restLengthField.Value = simParams.restLength;
                end
        
                %% --- Brake Configuration Parameters ---  % *** New Section ***
                if isprop(obj.guiManager, 'brakingForceField') && ...
                   isprop(obj.guiManager, 'brakeEfficiencyField') && ...
                   isprop(obj.guiManager, 'brakeBiasField') && ...
                   isprop(obj.guiManager, 'brakeTypeDropdown') && ...
                   isprop(obj.guiManager, 'maxBrakingForceField')
                    obj.guiManager.brakingForceField.Value = simParams.brakingForce;
                    obj.guiManager.brakeEfficiencyField.Value = simParams.brakeEfficiency;
                    obj.guiManager.brakeBiasField.Value = simParams.brakeBias;
                    obj.guiManager.brakeTypeDropdown.Value = simParams.brakeType;
                    obj.guiManager.maxBrakingForceField.Value = simParams.maxBrakingForce;
                    %% --- End of Consistency Check ---
                end
                % *** End of Brake Configuration Parameters ***
        
                %% --- Transmission Configuration Parameters ---  % *** New Section ***
                if isprop(obj.guiManager, 'maxGearField') && ...
                   isprop(obj.guiManager, 'gearRatiosTable') && ...
                   isprop(obj.guiManager, 'finalDriveRatioField') && ...
                   isprop(obj.guiManager, 'shiftUpSpeedField') && ...
                   isprop(obj.guiManager, 'shiftDownSpeedField') && ...
                   isprop(obj.guiManager, 'engineBrakeTorqueField') && ...
                   isprop(obj.guiManager, 'shiftDelayField')
                    obj.guiManager.maxGearField.Value = simParams.maxGear;
                    obj.guiManager.gearRatiosTable.Data = simParams.gearRatios;
                    obj.guiManager.gearRatiosTable.RowName = arrayfun(@(x) sprintf('Gear %d', x), 1:length(simParams.gearRatios.Gear), 'UniformOutput', false);
                    obj.guiManager.finalDriveRatioField.Value = simParams.finalDriveRatio;
                    obj.guiManager.shiftUpSpeedField.Value = mat2str(simParams.shiftUpSpeed);
                    obj.guiManager.shiftDownSpeedField.Value = mat2str(simParams.shiftDownSpeed);
                    obj.guiManager.engineBrakeTorqueField.Value = simParams.engineBrakeTorque;
                    obj.guiManager.shiftDelayField.Value = simParams.shiftDelay;
        
                    %% --- Engine Gear Mapping ---
                    if isfield(simParams, 'engineGearMapping') && ~isempty(simParams.engineGearMapping)
                        obj.guiManager.engineGearMappingTable.Data = simParams.engineGearMapping;
                    end
                end
                % *** End of Transmission Configuration Parameters ***
        
                %% --- Engine Configuration Parameters ---
                if isprop(obj.guiManager, 'maxEngineTorqueField') && ...
                   isprop(obj.guiManager, 'maxPowerField') && ...
                   isprop(obj.guiManager, 'idleRPMField') && ...
                   isprop(obj.guiManager, 'redlineRPMField') && ...
                   isprop(obj.guiManager, 'fuelConsumptionRateField')
                    obj.guiManager.maxEngineTorqueField.Value = simParams.maxEngineTorque;
                    obj.guiManager.maxPowerField.Value = simParams.maxPower;
                    obj.guiManager.idleRPMField.Value = simParams.idleRPM;
                    obj.guiManager.redlineRPMField.Value = simParams.redlineRPM;
                    obj.guiManager.fuelConsumptionRateField.Value = simParams.fuelConsumptionRate;
                end
        
                %% --- Pressure Matrices Parameters ---
                if isprop(obj.guiManager, 'pressureMatricesTab') && isprop(obj.guiManager, 'pressureMatrixPanels')
                    obj.guiManager.setPressureMatrices(simParams.pressureMatrices);
                end
        
                %% --- Trailer Parameters Handling ---
                % **This is the critical section to handle the numTiresPerAxleTrailerDropDown**
                if isprop(obj.guiManager, 'includeTrailerCheckbox') && ...
                   isprop(obj.guiManager, 'numTiresPerAxleTrailerDropDown')
                    if simParams.includeTrailer
                        % **Include Trailer: Enable Dropdown and Set Items to '2', '4'**
                        obj.guiManager.numTiresPerAxleTrailerDropDown.Items = {'2', '4'};
                        trailerTiresStr = num2str(simParams.numTiresPerAxleTrailer);
                        if ismember(trailerTiresStr, obj.guiManager.numTiresPerAxleTrailerDropDown.Items)
                            obj.guiManager.numTiresPerAxleTrailerDropDown.Value = trailerTiresStr;
                        else
                            warning('numTiresPerAxleTrailer (%s) is not an allowed item. Setting to default ("2").', trailerTiresStr);
                            obj.guiManager.numTiresPerAxleTrailerDropDown.Value = '2';
                            simParams.numTiresPerAxleTrailer = 2; % Optionally update simParams
                        end
                        obj.guiManager.numTiresPerAxleTrailerDropDown.Enable = 'on';
                    else
                        % **Exclude Trailer: Set Dropdown to '0' and Disable It**
                        obj.guiManager.numTiresPerAxleTrailerDropDown.Items = {'0'};
                        obj.guiManager.numTiresPerAxleTrailerDropDown.Value = '0';
                        obj.guiManager.numTiresPerAxleTrailerDropDown.Enable = 'off';
                    end
                end
        
                %% --- Trailer Tab Visibility ---
                % Optionally, update the visibility of trailer-related tabs or panels based on includeTrailer
                if isprop(obj.guiManager, 'trailerParamsTab')
                    obj.guiManager.setTrailerTabVisibility(simParams.includeTrailer);
                end

                obj.guiManager.mapCommandsBox.Value = simParams.mapCommands;
                obj.guiManager.generateWaypoints();
        
                %% --- Transmission Tab Visibility (Optional) ---
                % If there are conditions to show/hide Transmission Configuration Tab
                % Implement similar visibility handling if needed
            end
        end

        %% Get Simulation Parameters from UI
        function simParams = getSimulationParameters(obj)
            % getSimulationParameters Retrieves simulation parameters from the UI
        
            simParams = struct();

            simParams.guiManager = obj.guiManager;
            obj.guiManager.generateWaypoints();
            simParams.mapCommands = obj.guiManager.mapCommandsBox.Value;
        
            % --- Basic Configuration ---
            simParams.includeTrailer = obj.guiManager.includeTrailerCheckbox.Value;
            if isprop(simParams,'trailerNumBoxes')
                if simParams.trailerNumBoxes <= 0
                    simParams.includeTrailer = false;
                end
            end
            simParams.tractorMass = obj.guiManager.tractorMassField.Value;
            simParams.initialVelocity = obj.guiManager.velocityField.Value;
            
            % Update simParams.waypoints to the final Nx2 numeric array
            simParams.waypoints = obj.guiManager.waypointsTable.Data;

            simParams.maxClutchTorque = obj.guiManager.maxClutchTorqueField.Value;
            simParams.engagementSpeed = obj.guiManager.engagementSpeedField.Value;
            simParams.disengagementSpeed = obj.guiManager.disengagementSpeedField.Value;
            
            simParams.torqueFileName = obj.guiManager.torqueFileNameField.Value;
        
            % --- Advanced Configuration ---
            simParams.I_trailerMultiplier = obj.guiManager.I_trailerMultiplierField.Value;
            simParams.maxDeltaDeg = obj.guiManager.maxDeltaField.Value;
            simParams.dtMultiplier = obj.guiManager.dtMultiplierField.Value;
            simParams.windowSize = obj.guiManager.windowSizeField.Value;
        
            % --- Tractor Parameters ---
            simParams.tractorLength = obj.guiManager.tractorLengthField.Value;
            simParams.tractorWidth = obj.guiManager.tractorWidthField.Value;
            simParams.tractorHeight = obj.guiManager.tractorHeightField.Value;
            simParams.tractorCoGHeight = obj.guiManager.tractorCoGHeightField.Value;
            simParams.tractorWheelbase = obj.guiManager.tractorWheelbaseField.Value;
            simParams.tractorTrackWidth = obj.guiManager.tractorTrackWidthField.Value;
            simParams.tractorNumAxles = str2double(obj.guiManager.tractorNumAxlesDropdown.Value);
            simParams.tractorAxleSpacing = obj.guiManager.tractorAxleSpacingField.Value;
            simParams.numTiresPerAxleTrailer = str2double(obj.guiManager.numTiresPerAxleTrailerDropDown.Value);
            simParams.numTiresPerAxleTractor = str2double(obj.guiManager.numTiresPerAxleTractorDropDown.Value);
        
            % --- Trailer Parameters ---
            simParams.trailerLength = obj.guiManager.trailerLengthField.Value;
            simParams.trailerWidth = obj.guiManager.trailerWidthField.Value;
            simParams.trailerHeight = obj.guiManager.trailerHeightField.Value;
            simParams.trailerCoGHeight = obj.guiManager.trailerCoGHeightField.Value;
            simParams.trailerWheelbase = obj.guiManager.trailerWheelbaseField.Value;
            simParams.trailerTrackWidth = obj.guiManager.trailerTrackWidthField.Value;
            if isprop(obj.guiManager, 'trailerNumAxlesDropdown')
                simParams.trailerNumAxles = str2double(obj.guiManager.trailerNumAxlesDropdown.Value);
            else
                simParams.trailerNumAxles = 0;
            end
            simParams.trailerAxleSpacing = obj.guiManager.trailerAxleSpacingField.Value;
            simParams.trailerHitchDistance = obj.guiManager.trailerHitchDistanceField.Value;
            simParams.tractorHitchDistance = obj.guiManager.tractorHitchDistanceField.Value;
            % --- Multi-Trailer Configuration ---
            simParams.trailerNumBoxes = obj.guiManager.trailerNumBoxesField.Value;
            simParams.trailerAxlesPerBox = str2num(obj.guiManager.trailerAxlesPerBoxField.Value);
            simParams.trailerBoxSpacing = obj.guiManager.trailerBoxSpacingField.Value;

            % If no trailer boxes are configured, disable the trailer entirely
            if simParams.trailerNumBoxes <= 0
                simParams.includeTrailer = false;
                simParams.trailerMass = 0;
                simParams.trailerAxlesPerBox = [];
                simParams.trailerNumAxles = 0;
            end
            % If multiple trailer boxes are configured, compute the total number of
            % axles as the sum across boxes. This ensures tire pressure and load
            % calculations account for every box.
            if ~isempty(simParams.trailerAxlesPerBox)
                simParams.trailerNumAxles = sum(simParams.trailerAxlesPerBox);
            end
            % Gather per-box weight distributions only when boxes are present
            if simParams.trailerNumBoxes > 0 && ...
               isprop(obj.guiManager, 'trailerBoxWeightFields') && ...
               ~isempty(obj.guiManager.trailerBoxWeightFields) && ...
                simParams.includeTrailer
                nBoxes = size(obj.guiManager.trailerBoxWeightFields,1);
                simParams.trailerBoxWeightDistributions = cell(1,nBoxes);
                track = simParams.trailerTrackWidth;
                wheelbase = simParams.trailerWheelbase;
                for b = 1:nBoxes
                    weightsKg = zeros(4,1);
                    for j = 1:4
                        weightsKg(j) = obj.guiManager.trailerBoxWeightFields{b,j}.Value;
                    end
                    extraKgPerCorner = 6000/4;
                    weightsKgWithExtra = weightsKg + extraKgPerCorner;
                    positions = [ ...
                        -wheelbase/2, -track/2, simParams.trailerCoGHeight; ...
                        -wheelbase/2,  track/2, simParams.trailerCoGHeight; ...
                         wheelbase/2, -track/2, simParams.trailerCoGHeight; ...
                         wheelbase/2,  track/2, simParams.trailerCoGHeight];
                    simParams.trailerBoxWeightDistributions{b} = [positions, weightsKgWithExtra*9.81];
                    if b == 1
                        totalMass = 0;
                    end
                    boxMass = sum(weightsKgWithExtra);
                    totalMass = totalMass + boxMass;
                end

                simParams.trailerMass = totalMass;
                simParams.baseTrailerMass = simParams.trailerMass; % store unscaled mass
                simParams.trailerMassScaled = false;
                fprintf('Total vehicle mass updated: %.2f kg\n', simParams.tractorMass + totalMass);
            elseif simParams.trailerNumBoxes > 1
                % No individual weight distributions provided, so scale the
                % configured trailer mass by the number of boxes only once.
                % Use the previously stored trailer mass values if available so
                % repeated calls don't keep multiplying the mass.
                if ~isfield(simParams, 'baseTrailerMass') || isempty(simParams.baseTrailerMass)
                    if isfield(obj.simParams, 'baseTrailerMass')
                        simParams.baseTrailerMass = obj.simParams.baseTrailerMass;
                    else
                        simParams.baseTrailerMass = obj.simParams.trailerMass;
                    end
                end
                if ~isfield(simParams, 'trailerMass') || isempty(simParams.trailerMass)
                    simParams.trailerMass = obj.simParams.trailerMass;
                end
                simParams.trailerMass = simParams.baseTrailerMass * simParams.trailerNumBoxes;
                simParams.trailerMassScaled = true;
                fprintf('Total vehicle mass updated: %.2f kg\n', simParams.tractorMass + simParams.trailerMass);
            end

            % --- Spinner Configuration Parameters ---
            nSpinners = max(simParams.trailerNumBoxes - 1, 0);
            simParams.spinnerConfigs = cell(1, nSpinners);
            if isprop(obj.guiManager, 'spinnerConfig') && nSpinners > 0
                for iSpinner = 1:nSpinners
                    stiffCfg = struct(...
                        'x', obj.guiManager.spinnerConfig(iSpinner).stiffnessXField.Value, ...
                        'y', obj.guiManager.spinnerConfig(iSpinner).stiffnessYField.Value, ...
                        'z', obj.guiManager.spinnerConfig(iSpinner).stiffnessZField.Value, ...
                        'roll', obj.guiManager.spinnerConfig(iSpinner).stiffnessRollField.Value, ...
                        'pitch', obj.guiManager.spinnerConfig(iSpinner).stiffnessPitchField.Value, ...
                        'yaw', obj.guiManager.spinnerConfig(iSpinner).stiffnessYawField.Value ...
                    );
                    dampCfg = struct(...
                        'x', obj.guiManager.spinnerConfig(iSpinner).dampingXField.Value, ...
                        'y', obj.guiManager.spinnerConfig(iSpinner).dampingYField.Value, ...
                        'z', obj.guiManager.spinnerConfig(iSpinner).dampingZField.Value, ...
                        'roll', obj.guiManager.spinnerConfig(iSpinner).dampingRollField.Value, ...
                        'pitch', obj.guiManager.spinnerConfig(iSpinner).dampingPitchField.Value, ...
                        'yaw', obj.guiManager.spinnerConfig(iSpinner).dampingYawField.Value ...
                    );
                    simParams.spinnerConfigs{iSpinner} = struct('stiffness', stiffCfg, 'damping', dampCfg);
                end
            end

            % --- Commands Parameters ---  % *** New Section ***
            if isprop(obj.guiManager, 'steeringCommandsBox') && ...
               isprop(obj.guiManager, 'accelerationCommandsBox') && ...
               isprop(obj.guiManager, 'tirePressureCommandsBox')
                simParams.steeringCommands = obj.guiManager.steeringCommandsBox.Value;
                simParams.accelerationCommands = obj.guiManager.accelerationCommandsBox.Value;
                simParams.tirePressureCommands = obj.guiManager.tirePressureCommandsBox.Value;
            else
                % Use default command parameters
                simParams.steeringCommands = obj.simParams.steeringCommands;
                simParams.accelerationCommands = obj.simParams.accelerationCommands;
                simParams.tirePressureCommands = obj.simParams.tirePressureCommands;
                warning('Commands GUI fields not found. Using default command parameters.');
            end
            % --- End of Commands Parameters ---

            % --- Engine Configuration Parameters ---  % *** Added Engine Configuration ***
            if isprop(obj.guiManager, 'maxEngineTorqueField') && ...
               isprop(obj.guiManager, 'maxPowerField') && ...
               isprop(obj.guiManager, 'idleRPMField') && ...
               isprop(obj.guiManager, 'redlineRPMField') && ...
               isprop(obj.guiManager, 'engineBrakeTorqueField') && ...
               isprop(obj.guiManager, 'fuelConsumptionRateField')
                simParams.maxEngineTorque = obj.guiManager.maxEngineTorqueField.Value;
                simParams.maxPower = obj.guiManager.maxPowerField.Value;
                simParams.idleRPM = obj.guiManager.idleRPMField.Value;
                simParams.redlineRPM = obj.guiManager.redlineRPMField.Value;
                simParams.engineBrakeTorque = obj.guiManager.engineBrakeTorqueField.Value;
                simParams.fuelConsumptionRate = obj.guiManager.fuelConsumptionRateField.Value;
            else
                % Use default engine configuration parameters
                simParams.maxEngineTorque = obj.simParams.maxEngineTorque;
                simParams.maxPower = obj.simParams.maxPower;
                simParams.idleRPM = obj.simParams.idleRPM;
                simParams.redlineRPM = obj.simParams.redlineRPM;
                simParams.engineBrakeTorque = obj.simParams.engineBrakeTorque;
                simParams.fuelConsumptionRate = obj.simParams.fuelConsumptionRate;
                warning('Engine Configuration GUI fields not found. Using default engine configuration parameters.');
            end
        
            % --- Control Limits Parameters ---
            if isprop(obj.guiManager, 'steeringCurveFilePathField') && ...
               isprop(obj.guiManager, 'maxSteeringSpeedField') && ...
               isprop(obj.guiManager, 'maxSteeringAngleAtZeroSpeedField')
                if isa(obj.guiManager.maxSteeringAngleAtZeroSpeedField, 'matlab.ui.control.NumericEditField')
                    simParams.maxSteeringAngleAtZeroSpeed = obj.guiManager.maxSteeringAngleAtZeroSpeedField.Value;
                else
                    simParams.maxSteeringAngleAtZeroSpeed = obj.guiManager.maxSteeringAngleAtZeroSpeedField;
                end
                % simParams.minSteeringAngleAtMaxSpeed = obj.guiManager.minSteeringAngleAtMaxSpeedField.Value;
                simParams.steeringCurveFilePath = obj.guiManager.steeringCurveFilePathField.Value;
                simParams.maxSteeringSpeed = obj.guiManager.maxSteeringSpeedField.Value;
            else
                % Use default steering controller parameters
                simParams.maxSteeringAngleAtZeroSpeed = obj.simParams.maxSteeringAngleAtZeroSpeed;
                % simParams.minSteeringAngleAtMaxSpeed = obj.simParams.minSteeringAngleAtMaxSpeed;
                simParams.steeringCurveFilePath = obj.simParams.steeringCurveFilePath;
                simParams.maxSteeringSpeed = obj.simParams.maxSteeringSpeed;
                warning('Steering Controller GUI fields not found. Using default steering parameters.');
            end
        
            % --- Brake Configuration Parameters ---
            if isprop(obj.guiManager, 'brakingForceField') && ...
               isprop(obj.guiManager, 'brakeEfficiencyField') && ...
               isprop(obj.guiManager, 'brakeBiasField') && ...
               isprop(obj.guiManager, 'brakeTypeDropdown') && ...
               isprop(obj.guiManager, 'maxBrakingForceField') % Added check for brakeTypeDropdown
                simParams.brakingForce = obj.guiManager.brakingForceField.Value;
                simParams.brakeEfficiency = obj.guiManager.brakeEfficiencyField.Value;
                simParams.brakeBias = obj.guiManager.brakeBiasField.Value;
                simParams.brakeType = obj.guiManager.brakeTypeDropdown.Value;  % ** New Parameter **
                simParams.maxBrakingForce = obj.guiManager.maxBrakingForceField.Value;
            else
                % Use default brake system parameters
                simParams.brakingForce = obj.simParams.brakingForce;
                simParams.brakeEfficiency = obj.simParams.brakeEfficiency;
                simParams.brakeBias = obj.simParams.brakeBias;
                simParams.brakeType = obj.simParams.brakeType;  % ** New Parameter **
                warning('Brake Configuration GUI fields not found. Using default brake configuration parameters.');
            end
            % --- End of Brake Configuration Parameters ---
        
            % --- Transmission Parameters ---
            if isprop(obj.guiManager, 'maxGearField') && ...
               isprop(obj.guiManager, 'gearRatiosTable') && ...
               isprop(obj.guiManager, 'finalDriveRatioField') && ...
               isprop(obj.guiManager, 'shiftUpSpeedField') && ...
               isprop(obj.guiManager, 'shiftDownSpeedField') && ...
               isprop(obj.guiManager, 'engineBrakeTorqueField') && ...
               isprop(obj.guiManager, 'shiftDelayField')
                
                simParams.maxGear = obj.guiManager.maxGearField.Value;
                
                % Retrieve gear ratios from the table
                if iscell(obj.guiManager.gearRatiosTable.Data)
                    gearData = obj.guiManager.gearRatiosTable.Data;
                    simParams.gearRatios = cell2mat(gearData(:,2)); % Assuming second column contains ratios
                else
                    gearData = obj.guiManager.gearRatiosTable.Data;
                    simParams.gearRatios = gearData.Ratio; % Assuming second column contains ratios
                end
                
                simParams.finalDriveRatio = obj.guiManager.finalDriveRatioField.Value;
                
                % Retrieve shift up speeds from the text field
                shiftUpStr = obj.guiManager.shiftUpSpeedField.Value;
                simParams.shiftUpSpeed = str2num(shiftUpStr); %#ok<ST2NM>
                
                % Retrieve shift down speeds from the text field
                shiftDownStr = obj.guiManager.shiftDownSpeedField.Value;
                simParams.shiftDownSpeed = str2num(shiftDownStr); %#ok<ST2NM>
                
                simParams.engineBrakeTorque = obj.guiManager.engineBrakeTorqueField.Value;
                simParams.shiftDelay = obj.guiManager.shiftDelayField.Value;
            else
                % Use default transmission parameters
                simParams.maxGear = obj.simParams.maxGear;
                simParams.gearRatios = obj.simParams.gearRatios;
                simParams.finalDriveRatio = obj.simParams.finalDriveRatio;
                simParams.shiftUpSpeed = obj.simParams.shiftUpSpeed;
                simParams.shiftDownSpeed = obj.simParams.shiftDownSpeed;
                simParams.engineBrakeTorque = obj.simParams.engineBrakeTorque;
                simParams.shiftDelay = obj.simParams.shiftDelay;
                warning('Transmission Configuration GUI fields not found. Using default transmission parameters.');
            end
            % --- End of Transmission Parameters ---
        
            % --- Acceleration Limiter Parameters ---
            % simParams.maxAccelAtZeroSpeed = obj.guiManager.maxAccelAtZeroSpeedField.Value;
            simParams.minAccelAtMaxSpeed = obj.guiManager.minAccelAtMaxSpeedField.Value;
            % simParams.maxDecelAtZeroSpeed = obj.guiManager.maxDecelAtZeroSpeedField.Value;
            simParams.minDecelAtMaxSpeed = obj.guiManager.minDecelAtMaxSpeedField.Value;
            simParams.accelCurveFilePath = obj.guiManager.accelCurveFilePathField.Value;
            simParams.decelCurveFilePath = obj.guiManager.decelCurveFilePathField.Value;
            simParams.maxSpeedForAccelLimiting = obj.guiManager.maxSpeedForAccelLimitingField.Value;
            % --- End of Acceleration Limiter Parameters ---
        
            % --- PID Speed Controller Parameters ---
            if isprop(obj.guiManager, 'KpField') && isprop(obj.guiManager, 'KiField') && isprop(obj.guiManager, 'KdField') && isprop(obj.guiManager, 'enableSpeedControllerCheckbox')
                simParams.Kp = obj.guiManager.KpField.Value;
                simParams.Ki = obj.guiManager.KiField.Value;
                simParams.Kd = obj.guiManager.KdField.Value;
                if isprop(obj.guiManager, 'lambda1AccelField') && isprop(obj.guiManager, 'lambda2AccelField')
                    simParams.lambda1Accel = obj.guiManager.lambda1AccelField.Value;
                    simParams.lambda2Accel = obj.guiManager.lambda2AccelField.Value;
                else
                    simParams.lambda1Accel = obj.simParams.lambda1Accel;
                    simParams.lambda2Accel = obj.simParams.lambda2Accel;
                end
                if isprop(obj.guiManager, 'lambda1VelField') && isprop(obj.guiManager, 'lambda2VelField')
                    simParams.lambda1Vel = obj.guiManager.lambda1VelField.Value;
                    simParams.lambda2Vel = obj.guiManager.lambda2VelField.Value;
                else
                    simParams.lambda1Vel = obj.simParams.lambda1Vel;
                    simParams.lambda2Vel = obj.simParams.lambda2Vel;
                end
                if isprop(obj.guiManager, 'lambda1JerkField') && isprop(obj.guiManager, 'lambda2JerkField')
                    simParams.lambda1Jerk = obj.guiManager.lambda1JerkField.Value;
                    simParams.lambda2Jerk = obj.guiManager.lambda2JerkField.Value;
                else
                    simParams.lambda1Jerk = obj.simParams.lambda1Jerk;
                    simParams.lambda2Jerk = obj.simParams.lambda2Jerk;
                end
                simParams.enableSpeedController = obj.guiManager.enableSpeedControllerCheckbox.Value;
            else
                % Use default PID parameters
                simParams.Kp = obj.simParams.Kp;
                simParams.Ki = obj.simParams.Ki;
                simParams.Kd = obj.simParams.Kd;
                simParams.lambda1Accel = obj.simParams.lambda1Accel;
                simParams.lambda2Accel = obj.simParams.lambda2Accel;
                simParams.lambda1Vel = obj.simParams.lambda1Vel;
                simParams.lambda2Vel = obj.simParams.lambda2Vel;
                simParams.lambda1Jerk = obj.simParams.lambda1Jerk;
                simParams.lambda2Jerk = obj.simParams.lambda2Jerk;
                simParams.enableSpeedController = obj.simParams.enableSpeedController;
                warning('PID Controller GUI fields not found. Using default PID parameters.');
            end
            % --- End of PID Speed Controller Parameters ---
        
            % --- Maximum Speed Parameter ---
            if isprop(obj.guiManager, 'maxSpeedField')
                simParams.maxSpeed = obj.guiManager.maxSpeedField.Value; % m/s
            else
                simParams.maxSpeed = obj.simParams.maxSpeed; % Default value
                warning('Max Speed GUI field not found. Using default maxSpeed.');
            end
            % --- End of Max Speed Parameter ---
        
            % --- Tires Configuration Parameters ---
            if isprop(obj.guiManager, 'tractorTireHeightField') && ...
               isprop(obj.guiManager, 'tractorTireWidthField') && ...
               isprop(obj.guiManager, 'trailerTireHeightField') && ...
               isprop(obj.guiManager, 'trailerTireWidthField')
                simParams.tractorTireHeight = obj.guiManager.tractorTireHeightField.Value;
                simParams.tractorTireWidth = obj.guiManager.tractorTireWidthField.Value;
                simParams.trailerTireHeight = obj.guiManager.trailerTireHeightField.Value;
                simParams.trailerTireWidth = obj.guiManager.trailerTireWidthField.Value;
            else
                % Use default tire parameters
                simParams.tractorTireHeight = obj.simParams.tractorTireHeight;
                simParams.tractorTireWidth = obj.simParams.tractorTireWidth;
                simParams.trailerTireHeight = obj.simParams.trailerTireHeight;
                simParams.trailerTireWidth = obj.simParams.trailerTireWidth;
                warning('Tires Configuration GUI fields not found. Using default tire parameters.');
            end
            % --- End of Tires Configuration Parameters ---
        
            % --- Aerodynamics Parameters ---
            if isprop(obj.guiManager, 'airDensityField') && isprop(obj.guiManager, 'dragCoeffField') && ...
               isprop(obj.guiManager, 'windSpeedField') && isprop(obj.guiManager, 'windAngleDegField')
                simParams.airDensity = obj.guiManager.airDensityField.Value;
                simParams.dragCoeff = obj.guiManager.dragCoeffField.Value;
                simParams.windSpeed = obj.guiManager.windSpeedField.Value;
                simParams.windAngleDeg = obj.guiManager.windAngleDegField.Value;
            else
                % Use default aerodynamics parameters
                simParams.airDensity = obj.simParams.airDensity;
                simParams.dragCoeff = obj.simParams.dragCoeff;
                simParams.windSpeed = obj.simParams.windSpeed;
                simParams.windAngleDeg = obj.simParams.windAngleDeg;
                warning('Aerodynamics GUI fields not found. Using default aerodynamics parameters.');
            end
            % --- End of Aerodynamics Parameters ---
        
            % --- Road Conditions Parameters ---
            if isprop(obj.guiManager, 'slopeAngleField') && ...
               isprop(obj.guiManager, 'roadFrictionCoefficientField') && ...
               isprop(obj.guiManager, 'roadSurfaceTypeDropdown') && ...
               isprop(obj.guiManager, 'roadRoughnessField')
                simParams.slopeAngle = obj.guiManager.slopeAngleField.Value;  % In degrees
                simParams.roadFrictionCoefficient = obj.guiManager.roadFrictionCoefficientField.Value;
                simParams.roadSurfaceType = obj.guiManager.roadSurfaceTypeDropdown.Value;
                simParams.roadRoughness = obj.guiManager.roadRoughnessField.Value;
            else
                % Use default road conditions parameters
                simParams.slopeAngle = obj.simParams.slopeAngle;
                simParams.roadFrictionCoefficient = obj.simParams.roadFrictionCoefficient;
                simParams.roadSurfaceType = obj.simParams.roadSurfaceType;
                simParams.roadRoughness = obj.simParams.roadRoughness;
                warning('Road Conditions GUI fields not found. Using default road conditions parameters.');
            end
            % --- End of Road Conditions Parameters ---
        
            % --- Stiffness & Damping Parameters ---
            if isprop(obj.guiManager, 'stiffnessXField') && ...
               isprop(obj.guiManager, 'stiffnessYField') && ...
               isprop(obj.guiManager, 'stiffnessZField') && ...
               isprop(obj.guiManager, 'stiffnessRollField') && ...
               isprop(obj.guiManager, 'stiffnessPitchField') && ...
               isprop(obj.guiManager, 'stiffnessYawField') && ...
               isprop(obj.guiManager, 'dampingXField') && ...
               isprop(obj.guiManager, 'dampingYField') && ...
               isprop(obj.guiManager, 'dampingZField') && ...
               isprop(obj.guiManager, 'dampingRollField') && ...
               isprop(obj.guiManager, 'dampingPitchField') && ...
               isprop(obj.guiManager, 'dampingYawField')
                simParams.stiffnessX = obj.guiManager.stiffnessXField.Value;
                simParams.stiffnessY = obj.guiManager.stiffnessYField.Value;
                simParams.stiffnessZ = obj.guiManager.stiffnessZField.Value;
                simParams.stiffnessRoll = obj.guiManager.stiffnessRollField.Value;
                simParams.stiffnessPitch = obj.guiManager.stiffnessPitchField.Value;
                simParams.stiffnessYaw = obj.guiManager.stiffnessYawField.Value;
        
                simParams.dampingX = obj.guiManager.dampingXField.Value;
                simParams.dampingY = obj.guiManager.dampingYField.Value;
                simParams.dampingZ = obj.guiManager.dampingZField.Value;
                simParams.dampingRoll = obj.guiManager.dampingRollField.Value;
                simParams.dampingPitch = obj.guiManager.dampingPitchField.Value;
                simParams.dampingYaw = obj.guiManager.dampingYawField.Value;
            else
                % Use default stiffness & damping parameters
                simParams.stiffnessX = obj.simParams.stiffnessX;
                simParams.stiffnessY = obj.simParams.stiffnessY;
                simParams.stiffnessZ = obj.simParams.stiffnessZ;
                simParams.stiffnessRoll = obj.simParams.stiffnessRoll;
                simParams.stiffnessPitch = obj.simParams.stiffnessPitch;
                simParams.stiffnessYaw = obj.simParams.stiffnessYaw;
        
                simParams.dampingX = obj.simParams.dampingX;
                simParams.dampingY = obj.simParams.dampingY;
                simParams.dampingZ = obj.simParams.dampingZ;
                simParams.dampingRoll = obj.simParams.dampingRoll;
                simParams.dampingPitch = obj.simParams.dampingPitch;
                simParams.dampingYaw = obj.simParams.dampingYaw;
                warning('Stiffness & Damping GUI fields not found. Using default stiffness & damping parameters.');
            end
            % --- End of Stiffness & Damping Parameters ---
        
            % --- Suspension Model Parameters ---
            if isprop(obj.guiManager, 'K_springField') && ...
               isprop(obj.guiManager, 'C_dampingField') && ...
               isprop(obj.guiManager, 'restLengthField')
                simParams.K_spring = obj.guiManager.K_springField.Value;
                simParams.C_damping = obj.guiManager.C_dampingField.Value;
                simParams.restLength = obj.guiManager.restLengthField.Value;
            else
                % Use default suspension parameters
                simParams.K_spring = obj.simParams.K_spring;
                simParams.C_damping = obj.simParams.C_damping;
                simParams.restLength = obj.simParams.restLength;
                warning('Suspension Model GUI fields not found. Using default suspension parameters.');
            end
            % --- End of Suspension Model Parameters ---
        
            % --- Wind Parameters ---
            if isprop(obj.guiManager, 'windSpeedField') && ...
               isprop(obj.guiManager, 'windAngleDegField')
                simParams.windSpeed = obj.guiManager.windSpeedField.Value;
                simParams.windAngleDeg = obj.guiManager.windAngleDegField.Value;
            else
                simParams.windSpeed = obj.simParams.windSpeed;
                simParams.windAngleDeg = obj.simParams.windAngleDeg;
                warning('Wind GUI fields not found. Using default wind parameters.');
            end
            % --- End of Wind Parameters ---
        
            % --- X and Y Coefficients ---
            if isprop(obj.guiManager, 'pCx1Field') && isprop(obj.guiManager, 'pCy1Field')
                simParams.pCx1 = obj.guiManager.pCx1Field.Value;
                simParams.pDx1 = obj.guiManager.pDx1Field.Value;
                simParams.pDx2 = obj.guiManager.pDx2Field.Value;
                simParams.pEx1 = obj.guiManager.pEx1Field.Value;
                simParams.pEx2 = obj.guiManager.pEx2Field.Value;
                simParams.pEx3 = obj.guiManager.pEx3Field.Value;
                simParams.pEx4 = obj.guiManager.pEx4Field.Value;
                simParams.pKx1 = obj.guiManager.pKx1Field.Value;
                simParams.pKx2 = obj.guiManager.pKx2Field.Value;
                simParams.pKx3 = obj.guiManager.pKx3Field.Value;
        
                simParams.pCy1 = obj.guiManager.pCy1Field.Value;
                simParams.pDy1 = obj.guiManager.pDy1Field.Value;
                simParams.pDy2 = obj.guiManager.pDy2Field.Value;
                simParams.pEy1 = obj.guiManager.pEy1Field.Value;
                simParams.pEy2 = obj.guiManager.pEy2Field.Value;
                simParams.pEy3 = obj.guiManager.pEy3Field.Value;
                simParams.pEy4 = obj.guiManager.pEy4Field.Value;
                simParams.pKy1 = obj.guiManager.pKy1Field.Value;
                simParams.pKy2 = obj.guiManager.pKy2Field.Value;
                simParams.pKy3 = obj.guiManager.pKy3Field.Value;
            else
                % Use default coefficients
                simParams.pCx1 = obj.simParams.pCx1;
                simParams.pDx1 = obj.simParams.pDx1;
                simParams.pDx2 = obj.simParams.pDx2;
                simParams.pEx1 = obj.simParams.pEx1;
                simParams.pEx2 = obj.simParams.pEx2;
                simParams.pEx3 = obj.simParams.pEx3;
                simParams.pEx4 = obj.simParams.pEx4;
                simParams.pKx1 = obj.simParams.pKx1;
                simParams.pKx2 = obj.simParams.pKx2;
                simParams.pKx3 = obj.simParams.pKx3;
        
                simParams.pCy1 = obj.simParams.pCy1;
                simParams.pDy1 = obj.simParams.pDy1;
                simParams.pDy2 = obj.simParams.pDy2;
                simParams.pEy1 = obj.simParams.pEy1;
                simParams.pEy2 = obj.simParams.pEy2;
                simParams.pEy3 = obj.simParams.pEy3;
                simParams.pEy4 = obj.simParams.pEy4;
                simParams.pKy1 = obj.simParams.pKy1;
                simParams.pKy2 = obj.simParams.pKy2;
                simParams.pKy3 = obj.simParams.pKy3;
                warning('X and Y Coefficients GUI fields not found. Using default coefficients.');
            end
            % --- End of X and Y Coefficients ---
        
            % --- Retrieve and Set Pressure Matrices ---
            if isprop(obj.guiManager, 'pressureMatricesTab') && isprop(obj.guiManager, 'pressureMatrixPanels')
                simParams.pressureMatrices = obj.guiManager.getPressureMatrices();
            else
                % Use default pressure matrices
                simParams.pressureMatrices = obj.simParams.pressureMatrices;
                warning('Pressure Matrices GUI components not found. Using default pressure matrices.');
            end
            % --- End of Pressure Matrices ---
        
            % --- Retrieve Excel Data ---
            simParams.excelData = obj.simParams.excelData; % Assuming excelData is managed elsewhere
        
            % --- Final Step: Update Trailer Tab Visibility ---
            simParams.includeTrailer = obj.guiManager.includeTrailerCheckbox.Value;
        end

        %% Save Logs Method
        function [txtFilename, csvFilename] = saveLogs(obj, logMessages)
            % saveLogs Saves log messages to a text and CSV file without overwriting existing files

            % Generate unique filenames for logs
            txtFilename = obj.getUniqueFilename(obj.simulationName, '.txt');
            csvFilename = obj.getUniqueFilename(obj.simulationName, '.csv');

            % Save log data to a text file
            fid = fopen(txtFilename, 'w');
            if fid ~= -1
                fprintf(fid, '%s\n', logMessages{:});
                fclose(fid);
                disp(['Log data saved to ', txtFilename]);
            else
                warning('Failed to write simulation log to text file.');
            end

            % Create a table with log messages
            logTable = table(logMessages', 'VariableNames', {'Logs'});
            % Save log data to a CSV file
            try
                writetable(logTable, csvFilename);
                disp(['Log data saved to ', csvFilename]);
            catch ME
                warning('Failed to write simulation log to CSV file: %s', ME.message);
            end
        end

        %% Initialize Predefined Pressure Matrices
        function initializePressureMatrices(obj)
            % initializePressureMatrices Creates predefined pressure matrices with example values

            % Initialize the structure to hold pressure matrices
            obj.simParams.pressureMatrices = struct();

            % Pressure matrix for 4 tires
            obj.simParams.pressureMatrices.Tires4 = [
                800000; % Tire 1
                800000; % Tire 2
                800000; % Tire 3
                800000  % Tire 4
            ];

            % Pressure matrix for 6 tires
            obj.simParams.pressureMatrices.Tires6 = [
                750000; % Tire 1
                760000; % Tire 2
                770000; % Tire 3
                780000; % Tire 4
                790000; % Tire 5
                800000  % Tire 6
            ];

            % Pressure matrix for 8 tires
            obj.simParams.pressureMatrices.Tires8 = [
                800000; % Tire 1
                810000; % Tire 2
                820000; % Tire 3
                830000; % Tire 4
                840000; % Tire 5
                850000; % Tire 6
                860000; % Tire 7
                870000  % Tire 8
            ];

            % Pressure matrix for 10 tires
            obj.simParams.pressureMatrices.Tires10 = [
                800000; % Tire 1
                800000; % Tire 2
                800000; % Tire 3
                800000; % Tire 4
                800000; % Tire 5
                800000; % Tire 6
                800000; % Tire 7
                800000; % Tire 8
                800000; % Tire 9
                800000  % Tire 10
            ];

            % Pressure matrix for 12 tires
            obj.simParams.pressureMatrices.Tires12 = [
                780000; % Tire 1
                790000; % Tire 2
                800000; % Tire 3
                810000; % Tire 4
                820000; % Tire 5
                830000; % Tire 6
                840000; % Tire 7
                850000; % Tire 8
                860000; % Tire 9
                870000; % Tire 10
                880000; % Tire 11
                890000  % Tire 12
            ];

            % Pressure matrix for 14 tires
            obj.simParams.pressureMatrices.Tires14 = [
                780000; % Tire 1
                790000; % Tire 2
                800000; % Tire 3
                810000; % Tire 4
                820000; % Tire 5
                830000; % Tire 6
                840000; % Tire 7
                850000; % Tire 8
                860000; % Tire 9
                870000; % Tire 10
                880000; % Tire 11
                890000; % Tire 12
                880000; % Tire 13
                890000  % Tire 14
            ];

            % Pressure matrix for 16 tires
            obj.simParams.pressureMatrices.Tires16 = [
                800000; % Tire 1
                805000; % Tire 2
                810000; % Tire 3
                815000; % Tire 4
                820000; % Tire 5
                825000; % Tire 6
                830000; % Tire 7
                835000; % Tire 8
                840000; % Tire 9
                845000; % Tire 10
                850000; % Tire 11
                855000; % Tire 12
                860000; % Tire 13
                865000; % Tire 14
                870000; % Tire 15
                875000  % Tire 16
            ];

            % Pressure matrix for 18 tires
            obj.simParams.pressureMatrices.Tires18 = [
                780000; % Tire 1
                785000; % Tire 2
                790000; % Tire 3
                795000; % Tire 4
                800000; % Tire 5
                805000; % Tire 6
                810000; % Tire 7
                815000; % Tire 8
                820000; % Tire 9
                825000; % Tire 10
                830000; % Tire 11
                835000; % Tire 12
                840000; % Tire 13
                845000; % Tire 14
                850000; % Tire 15
                855000; % Tire 16
                860000; % Tire 17
                865000  % Tire 18
            ];

            % Pressure matrix for 20 tires
            obj.simParams.pressureMatrices.Tires20 = [
                800000; % Tire 1
                802000; % Tire 2
                804000; % Tire 3
                806000; % Tire 4
                808000; % Tire 5
                810000; % Tire 6
                812000; % Tire 7
                814000; % Tire 8
                816000; % Tire 9
                818000; % Tire 10
                820000; % Tire 11
                822000; % Tire 12
                824000; % Tire 13
                826000; % Tire 14
                828000; % Tire 15
                830000; % Tire 16
                832000; % Tire 17
                834000; % Tire 18
                836000; % Tire 19
                838000  % Tire 20
            ];
        end

        %% Generate Unique Filename
        function uniqueName = getUniqueFilename(~, baseName, extension)
            % getUniqueFilename Generates a unique filename by appending a timestamp
            %
            % Parameters:
            %   baseName  - The base name of the file without extension
            %   extension - The file extension (e.g., '.mat', '.csv', '.txt')
            %
            % Returns:
            %   uniqueName - A unique filename with the specified extension

            % Initialize the unique name
            uniqueName = [baseName, extension];

            % Check if the file exists
            if exist(uniqueName, 'file') == 2
                % Append a timestamp to make the filename unique
                timestamp = datestr(now, 'yyyymmdd_HHMMSS');
                uniqueName = [baseName, '_', timestamp, extension];

                % Ensure the new filename also doesn't exist
                while exist(uniqueName, 'file') == 2
                    pause(1); % Wait for a second to get a new timestamp
                    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
                    uniqueName = [baseName, '_', timestamp, extension];
                end
            end
        end
                                
        function [output] = initializeSim(obj)
            % runSimulation Executes the vehicle simulation and computes the dynamics
        
            % Initialize output variables
            tractorX = [];
            tractorY = [];
            tractorTheta = [];
            trailerX = [];
            trailerY = [];
            trailerTheta = [];
            steeringAnglesSim = [];
            speedData = [];
            globalVehicleFlags = struct('isWiggling', false, 'isRollover', false, 'isJackknife', false, 'isSkidding', false);
        
            % Initialize flat tire indices
            flatTireIndicesTractor = [];
            flatTireIndicesTrailer = [];
            flatTireIndices = []; % Combined flat tire indices
        
            % Initialize log messages storage
            logMessages = {};  % Preallocate as empty cell array
        
            % --- Initialize Waitbar ---
            hWaitbar = waitbar(0, 'Starting Simulation...', 'Name', 'Simulation Progress');
            cleanupObj = onCleanup(@() closeIfOpen(hWaitbar)); % Ensure waitbar closes on function exit
            % --- End of Waitbar Initialization ---
        
            try
                % Extract simulation data
                if obj.hasUI
                    % Get parameters from UI fields
                    simParams = obj.getSimulationParameters();
                    if isempty(simParams)
                        uialert(obj.guiManager.tablePanel.Parent, 'Simulation parameters extraction failed.', 'Simulation Error');
                        return;
                    end
                    try
                        % Extract steering and acceleration commands
                        steeringCommands = simParams.steeringCommands;
                        accelerationCommands = simParams.accelerationCommands;
                        tirePressureCommands = simParams.tirePressureCommands;
        
                        if isempty(steeringCommands)
                            error('Steering commands are empty.');
                        end
                    catch ME
                        uialert(obj.guiManager.tablePanel.Parent, 'Please load a valid Excel file with steering and acceleration commands.', 'Data Loading Error');
                        return;
                    end
                else
                    % Get parameters from simulation parameters
                    simParams = obj.simParams;
                    try
                        % Extract steering and acceleration commands
                        steeringCommands = simParams.steeringCommands;
                        accelerationCommands = simParams.accelerationCommands;
                        tirePressureCommands = simParams.tirePressureCommands;
        
                        if isempty(steeringCommands)
                            error('Steering commands are empty.');
                        end
                    catch ME
                        fprintf('Data Loading Error: %s\n', ME.message);
                        return;
                    end
                end
        
                % Assign parameters to variables for easier access
                tractorMass = simParams.tractorMass;
                boxMasses = [];
                if simParams.includeTrailer
                    if isfield(simParams,'trailerBoxWeightDistributions') && ~isempty(simParams.trailerBoxWeightDistributions)
                        % Compute the mass of each trailer box from its load distribution
                        boxMasses = cellfun(@(ld) sum(ld(:,4)) / 9.81, simParams.trailerBoxWeightDistributions);
                        trailerMass = sum(boxMasses);
                    else
                        trailerMass = simParams.trailerMass;
                    end
                    fprintf('Total vehicle mass updated: %.2f kg\n', tractorMass + trailerMass);
                    trailerWheelbase = simParams.trailerWheelbase; % Defined when trailer is included
                else
                    trailerMass = 0;
                    trailerWheelbase = 0; % Set to 0 when trailer is not included
                end
                initialVelocity = simParams.initialVelocity;
                I_trailerMultiplier = simParams.I_trailerMultiplier;
                maxDeltaDeg = simParams.maxDeltaDeg;
                dtMultiplier = simParams.dtMultiplier;
                windowSize = simParams.windowSize;
                tractorLength = simParams.tractorLength;
                tractorWidth = simParams.tractorWidth;
                tractorHeight = simParams.tractorHeight;
                tractorCoGHeight = simParams.tractorCoGHeight;
                tractorWheelbase = simParams.tractorWheelbase;
                tractorTrackWidth = simParams.tractorTrackWidth;
                tractorNumAxles = simParams.tractorNumAxles;
                tractorAxleSpacing = simParams.tractorAxleSpacing;
                numTiresPerAxleTrailer = simParams.numTiresPerAxleTrailer;
                numTiresPerAxleTractor = simParams.numTiresPerAxleTractor; % **New Variable**
        
                % --- Tires Configuration Parameters ---
                tractorTireHeight = simParams.tractorTireHeight; % meters
                tractorTireWidth = simParams.tractorTireWidth;   % meters
                trailerTireHeight = simParams.trailerTireHeight; % meters
                trailerTireWidth = simParams.trailerTireWidth;   % meters
                % --- End of Tires Configuration Parameters ---
        
                % --- Integrated Acceleration Limiter Parameters ---
                % maxAccelAtZeroSpeed = simParams.maxAccelAtZeroSpeed;
                minAccelAtMaxSpeed = simParams.minAccelAtMaxSpeed;
                % maxDecelAtZeroSpeed = simParams.maxDecelAtZeroSpeed;
                minDecelAtMaxSpeed = simParams.minDecelAtMaxSpeed;
                accelCurve = readmatrix(simParams.accelCurveFilePath);
                decelCurve = readmatrix(simParams.decelCurveFilePath);
                maxSpeedForAccelLimiting = simParams.maxSpeedForAccelLimiting;
                % --- End of Acceleration Limiter Parameters ---
        
                % --- PID Speed Controller Parameters ---
                Kp = simParams.Kp;
                Ki = simParams.Ki;
                Kd = simParams.Kd;
                % --- End of PID Speed Controller Parameters ---
        
                % --- Speed Limiting Parameter ---
                maxSpeed = simParams.maxSpeed; % m/s
                % --- End of Speed Limiting Parameter ---
        
                % --- **Aerodynamics Parameters Integration** ---
                airDensity = simParams.airDensity;     % kg/m³; retrieved from GUI
                dragCoeff = simParams.dragCoeff;       % Dimensionless; retrieved from GUI
        
                if simParams.includeTrailer
                    trailerWidth = simParams.trailerWidth;
                    trailerHeight = simParams.trailerHeight;
                    trailerLength = simParams.trailerLength;
                else
                    trailerWidth = 0;
                    trailerHeight = 0;
                    trailerLength = 0;
                end
        
                frontalWidth = max(tractorWidth, trailerWidth);
                frontalHeight = max(tractorHeight, trailerHeight);
                frontalArea = frontalWidth * frontalHeight;  % In square meters
        
                sideWidth = max(tractorWidth, trailerWidth);
                sideLength = (tractorLength + trailerLength) - tractorAxleSpacing;
                sideArea = sideWidth * sideLength;  % In square meters
                % --- End of Aerodynamics Parameters Integration ---
        
                % --- **Road Conditions Parameters** ---
                slopeAngleDeg = simParams.slopeAngle;  % In degrees
                slopeAngleRad = deg2rad(slopeAngleDeg); % Convert to radians
                roadRoughness = simParams.roadRoughness; % 0 (smooth) to 1 (very rough)
                % --- End of Road Conditions Parameters ---
        
                % Calculate total number of tires
                totalTiresTractor = 2 + (tractorNumAxles * numTiresPerAxleTractor);
        
                if simParams.includeTrailer
                    trailerNumAxles = sum(simParams.trailerAxlesPerBox);
                    totalTiresTrailer = trailerNumAxles * numTiresPerAxleTrailer;
                else
                    totalTiresTrailer = 0;
                end
                totalNumberOfTires = totalTiresTractor + totalTiresTrailer;

                % --- Select Pressure Matrix Based on Total Number of Tires ---
                pressureMatrixKey = sprintf('Tires%d', totalNumberOfTires);
                if isfield(simParams.pressureMatrices, pressureMatrixKey)
                    selectedPressures = simParams.pressureMatrices.(pressureMatrixKey);
                    logMessages{end+1} = sprintf('Selected pressure matrix for %d tires.', totalNumberOfTires);
                else
                    error('No pressure matrix found for %d tires.', totalNumberOfTires);
                end
                % --- End of Pressure Matrix Selection ---
                
                % Adjust time step based on multiplier
                dt_original = 0.01; % Original time step
                dt = dt_original * dtMultiplier;
                if dt > 0.01
                    dt = 0.01;
                    logMessages{end+1} = sprintf('Adjusted time step to %.4f seconds for simulation stability.', dt);
                end
                
                % --- Process Steering and Acceleration Data ---
                % Handle ramp and step commands in steering angles and acceleration
                [timeProcessed, steerAngles, accelerationData, tirePressureData, steeringEnded, accelerationEnded, tirePressureEnded] = processSignalData( ...
                    steeringCommands, ...          % Steering command string
                    accelerationCommands, ...      % Acceleration command string
                    tirePressureCommands, ...      % Tire Pressure command string
                    dt, ...
                    selectedPressures' ...
                    );

                selectedPressures = tirePressureData(1, :);
                % --- Split the pressures into tractor and trailer pressures ---
                tractorTirePressures = selectedPressures(1:totalTiresTractor);
                if simParams.includeTrailer
                    trailerTirePressures = selectedPressures(totalTiresTractor+1:end);
                else
                    trailerTirePressures = [];
                end
        
                % Validate the lengths
                if length(tractorTirePressures) ~= totalTiresTractor
                    error('Mismatch in number of tractor tire pressures.');
                end
                if simParams.includeTrailer && length(trailerTirePressures) ~= totalTiresTrailer
                    error('Mismatch in number of trailer tire pressures.');
                end
                % --- End of Split Pressures ---
        
                % --- Begin Flat Tire Logic ---
                % Define Pressure Threshold
                % Convert 14 psi to Pascals (1 psi ≈ 6895 Pa)
                pressureThresholdPa = 14 * 6895; % 14 psi in Pa
        
                % Log the pressure threshold
                logMessages{end+1} = sprintf('Pressure Threshold set to %.2f Pa (14 psi).', pressureThresholdPa);
        
                % --- Check and Set Flat Tires for Tractor ---
                for i = 1:length(tractorTirePressures)
                    if tractorTirePressures(i) < pressureThresholdPa
                        % Collect the index of the flat tire
                        flatTireIndicesTractor(end+1) = i;
                        % Log the flat tire event
                        logMessages{end+1} = sprintf('Warning: Tractor Tire %d is flat (Pressure: %.2f Pa < %.2f Pa).', i, tractorTirePressures(i), pressureThresholdPa);
                        % Mark the tire as flat by setting its contact area to zero
                        tractorTirePressures(i) = 0; 
                    end
                end
        
                % --- Check and Set Flat Tires for Trailer ---
                if simParams.includeTrailer
                    for i = 1:length(trailerTirePressures)
                        if trailerTirePressures(i) < pressureThresholdPa
                            % Adjust the index to account for the total number of tractor tires
                            adjustedIndex = i + length(tractorTirePressures);
                            % Collect the index of the flat tire
                            flatTireIndicesTrailer(end+1) = adjustedIndex;
                            % Log the flat tire event
                            logMessages{end+1} = sprintf('Warning: Trailer Tire %d is flat (Pressure: %.2f Pa < %.2f Pa).', i, trailerTirePressures(i), pressureThresholdPa);
                            % Mark the tire as flat by setting its contact area to zero
                            trailerTirePressures(i) = 0; 
                        end
                    end
                end
                % Combine flat tire indices
                flatTireIndices = [flatTireIndicesTractor, flatTireIndicesTrailer];
                % --- End Flat Tire Logic ---
        
                % --- Compute Contact Areas ---
                % Compute per-tire loads (assuming equal distribution for simplicity)
                perTireLoadTractor = (tractorMass * 9.81) / totalTiresTractor; % N
                if simParams.includeTrailer
                    if isfield(simParams,'trailerBoxWeightDistributions') && ~isempty(simParams.trailerBoxWeightDistributions)
                        % Individual loads are provided per tire in the weight distributions
                        trailerLoadsPerTire = vertcat(simParams.trailerBoxWeightDistributions{:});
                        perTireLoadTrailer = trailerLoadsPerTire(:,4);
                        if length(perTireLoadTrailer) ~= length(trailerTirePressures)
                            logMessages{end+1} = sprintf(['Warning: trailerBoxWeightDistributions length (%d) ' ...
                                'does not match number of trailer tires (%d). Using uniform distribution.'], ...
                                length(perTireLoadTrailer), length(trailerTirePressures));
                            perTireLoadTrailer = (trailerMass * 9.81) / length(trailerTirePressures) * ...
                                ones(length(trailerTirePressures),1);
                        end
                    else
                        perTireLoadTrailer = (trailerMass * 9.81) / totalTiresTrailer * ones(totalTiresTrailer,1); % N each
                    end
                end
        
                % Compute per-tire contact areas
                % Avoid division by zero by setting contact area to zero if pressure is zero (flat tire)
                tractorContactAreas = zeros(size(tractorTirePressures));
                for i = 1:length(tractorTirePressures)
                    if tractorTirePressures(i) > 0
                        tractorContactAreas(i) = perTireLoadTractor / tractorTirePressures(i); % m^2
                    else
                        tractorContactAreas(i) = 0; % Flat tire
                    end
                end
        
                if simParams.includeTrailer
                    trailerContactAreas = zeros(size(trailerTirePressures));
                    for i = 1:length(trailerTirePressures)
                        if trailerTirePressures(i) > 0
                            trailerContactAreas(i) = perTireLoadTrailer(i) / trailerTirePressures(i); % m^2
                        else
                            trailerContactAreas(i) = 0; % Flat tire
                        end
                    end
                else
                    trailerContactAreas = [];
                end
                % --- End of Compute Contact Areas ---
        
                % --- Adjust Friction Coefficient Based on Road Surface Type ---
                % Use predefined friction coefficients for different surfaces
                switch simParams.roadSurfaceType
                    case 'Dry Asphalt'
                        mu = 0.85;
                    case 'Wet Asphalt'
                        mu = 0.6;
                    case 'Gravel'
                        mu = 0.5;
                    case 'Snow'
                        mu = 0.2;
                    case 'Ice'
                        mu = 0.1;
                    otherwise
                        mu = simParams.roadFrictionCoefficient; % Use the provided value
                end
                % Log the adjusted friction coefficient
                logMessages{end+1} = sprintf('Adjusted friction coefficient (mu) based on road surface type (%s): %.2f', simParams.roadSurfaceType, mu);
                % --- End of Friction Coefficient Adjustment ---
        
                % Log simulation parameters
                logMessages{end+1} = 'Simulation Parameters:';
                logMessages{end+1} = sprintf('Tractor Mass (including loads): %.2f kg', tractorMass);
                if simParams.includeTrailer
                    logMessages{end+1} = sprintf('Trailer Mass: %.2f kg', trailerMass);
                else
                    logMessages{end+1} = 'Trailer is not included in the simulation.';
                end
                logMessages{end+1} = sprintf('Coefficient of Friction (mu): %.2f', mu);
                logMessages{end+1} = sprintf('Initial Velocity: %.2f m/s', initialVelocity);
                logMessages{end+1} = sprintf('PID Controller Gains - Kp: %.2f, Ki: %.2f, Kd: %.2f', Kp, Ki, Kd);
                logMessages{end+1} = sprintf('Maximum Speed Limit: %.2f m/s', maxSpeed);
                logMessages{end+1} = sprintf('Time Step Multiplier: %.2f', dtMultiplier);
                logMessages{end+1} = sprintf('Maximum Articulation Angle: %.2f degrees', maxDeltaDeg);
                logMessages{end+1} = sprintf('Air Density: %.3f kg/m³', airDensity);
                logMessages{end+1} = sprintf('Drag Coefficient: %.2f', dragCoeff);
                logMessages{end+1} = sprintf('Slope Angle: %.2f degrees', slopeAngleDeg);
                logMessages{end+1} = sprintf('Road Surface Type: %s', simParams.roadSurfaceType);
                logMessages{end+1} = sprintf('Road Roughness: %.2f', roadRoughness);
                logMessages{end+1} = '----------------------------------------';
        
                % --- Tires Configuration Logging ---
                logMessages{end+1} = 'Tires Configuration:';
                logMessages{end+1} = sprintf('Tractor Tire Height: %.2f m', tractorTireHeight);
                logMessages{end+1} = sprintf('Tractor Tire Width: %.2f m', tractorTireWidth);
                if simParams.includeTrailer
                    logMessages{end+1} = sprintf('Trailer Tire Height: %.2f m', trailerTireHeight);
                    logMessages{end+1} = sprintf('Trailer Tire Width: %.2f m', trailerTireWidth);
                end
                logMessages{end+1} = '----------------------------------------';
        
                % Validate load inputs to prevent front overloading
                % maxFrontLoad = 0.6 * tractorMass * 9.81; % 60% of tractor mass in N
                % totalFrontLoad = (W_FrontLeft + W_FrontRight) * 9.81;
        
                % if totalFrontLoad > maxFrontLoad
                %     msg = sprintf('Total front load exceeds 60%% of tractor mass (%.2f N). Please reduce the front load.', maxFrontLoad);
                %     if obj.hasUI
                %         uialert(obj.guiManager.tablePanel.Parent, msg, 'Load Input Error');
                %     else
                %         logMessages{end+1} = sprintf('Load Input Error: %s', msg);
                %     end
                %     % Save logs before returning
                %     [txtFilename, csvFilename] = obj.saveLogs(logMessages); % Capture filenames
                %     return;
                % end
        
                % --- Retrieve Brake System Parameters ---
                maxBrakingForce = simParams.maxBrakingForce;
                brakeEfficiency = simParams.brakeEfficiency;
                brakeType = simParams.brakeType;
                brakeResponseRate = 1;
                brakeBias = simParams.brakeBias;
        
                % Initialize BrakeSystem with parameters from simParams
                brakeSystem = BrakeSystem( ...
                    maxBrakingForce, ...
                    brakeEfficiency, ...
                    brakeType, ...
                    brakeResponseRate, ...
                    brakeBias ...
                    );
                logMessages{end+1} = 'BrakeSystem initialized successfully.';
                % --- End of Brake System Parameters ---
        
                % --- Retrieve Transmission Parameters ---
                maxGear = simParams.maxGear;
                gearRatios = simParams.gearRatios;
                finalDriveRatio = simParams.finalDriveRatio;
                shiftUpSpeed = simParams.shiftUpSpeed;
                shiftDownSpeed = simParams.shiftDownSpeed;
                engineBrakeTorque = simParams.engineBrakeTorque;
                shiftDelay = simParams.shiftDelay;
                % --- End of Transmission Parameters ---
        
                % Log Brake System Parameters
                logMessages{end+1} = 'Brake System Parameters:';
                logMessages{end+1} = sprintf('Max Braking Force: %.2f N', simParams.maxBrakingForce);
                logMessages{end+1} = sprintf('Brake Efficiency: %.2f', simParams.brakeEfficiency);
                logMessages{end+1} = sprintf('Brake Type: %s', simParams.brakeType);
                logMessages{end+1} = '----------------------------------------';
        
                % Log Transmission Parameters
                logMessages{end+1} = 'Transmission Parameters:';
                logMessages{end+1} = sprintf('Max Gear: %d', simParams.maxGear);
                logMessages{end+1} = sprintf('Final Drive Ratio: %.2f', simParams.finalDriveRatio);
                logMessages{end+1} = sprintf('Shift Up Speeds: %s m/s', mat2str(simParams.shiftUpSpeed));
                logMessages{end+1} = sprintf('Shift Down Speeds: %s m/s', mat2str(simParams.shiftDownSpeed));
                logMessages{end+1} = sprintf('Engine Brake Torque: %.2f Nm', obj.simParams.engineBrakeTorque);
                logMessages{end+1} = sprintf('Shift Delay: %.2f s', simParams.shiftDelay);
                logMessages{end+1} = '----------------------------------------';
        
                % --- Instantiate Clutch ---
                % Define clutch parameters for a Class 8 truck
                maxClutchTorque = simParams.maxClutchTorque;           % Maximum torque the clutch can handle (Nm), typical for heavy-duty trucks
                engagementSpeed = simParams.engagementSpeed;            % Time to engage the clutch (seconds), assuming gradual engagement
                disengagementSpeed = simParams.disengagementSpeed;         % Time to disengage the clutch (seconds), assuming faster disengagement

                clutch = Clutch( ...
                    maxClutchTorque, ...
                    engagementSpeed, ...
                    disengagementSpeed ...
                    );
                logMessages{end+1} = 'Clutch initialized successfully.';
                % --- End of Clutch Instantiation ---
                % --- Instantiate Transmission ---
                filterWindowSize = 500;  % Moving average window size
                transmission = Transmission( ...
                    maxGear, ...
                    gearRatios, ...
                    finalDriveRatio, ...
                    shiftUpSpeed, ...
                    shiftDownSpeed, ...
                    engineBrakeTorque, ...
                    shiftDelay, ...
                    clutch, ...
                    filterWindowSize ...
                    );
                logMessages{end+1} = 'Transmission initialized successfully for Class 8 Truck.';
                % --- End of Transmission Initialization ---
        
                % --- Update Engine Efficiency Based on Fuel Consumption Rate ---
                % Define base fuel consumption rate and corresponding base efficiency
                baseFuelConsumptionRate = 10; % Example value in liters/hour
                baseEfficiency = 0.85;        % Base drivetrain efficiency (0 to 1)
        
                % Ensure fuelConsumptionRate is positive to avoid division by zero
                if simParams.fuelConsumptionRate <= 0
                    error('Fuel Consumption Rate must be positive.');
                end
        
                % Calculate updated efficiency
                % Assumption: Efficiency inversely proportional to fuel consumption rate
                % You can adjust the relationship as needed
                efficiency = baseEfficiency * (baseFuelConsumptionRate / simParams.fuelConsumptionRate);
        
                % Clamp efficiency between 0.5 and 1.0 for realistic values
                efficiency = max(0.5, min(1.0, efficiency));
        
                % Log the updated efficiency
                logMessages{end+1} = sprintf('Engine efficiency updated to %.2f based on fuel consumption rate of %.2f liters/hour.', efficiency, simParams.fuelConsumptionRate);
                % --- End of Efficiency Update ---
        
                % --- Initialize Engine Class ---
                % --- Engine Parameters ---
                % Hardcoded values for a Class 8 truck
                maxTorque = 2000; % Maximum engine torque (Nm)
                maxRPM = 2100;    % Maximum engine RPM
                minRPM = 600;     % Minimum engine RPM (idle RPM)
                gearRatiosEngine = gearRatios; % Use the same gear ratios as Transmission
                differentialRatio = finalDriveRatio; % Differential gear ratio
                % --- End of Engine Parameters ---
        
                % --- Read the non-linear torque curve from Excel file ---
                % Assuming the file is named "torque_curve.xlsx" and has columns "RPM" and "Torque"
                try
                    torqueData = readtable(simParams.torqueFileName);
                    if any(strcmp(torqueData.Properties.VariableNames, {'RPM', 'Torque'}))
                        rpmValues = torqueData.RPM;
                        torqueValues = torqueData.Torque;
        
                        % Define the torque curve as an interpolation based on the loaded data
                        torqueCurve = @(rpm) interp1(rpmValues, torqueValues, rpm, 'linear', 'extrap');
        
                        logMessages{end+1} = sprintf('Loaded non-linear torque curve from "%s".', simParams.torqueFileName);
                    else
                        error('File format error: Expected columns "RPM" and "Torque" not found.');
                    end
                catch ME
                    logMessages{end+1} = sprintf('Error loading torque curve from file: %s', ME.message);
                    % Fall back to default torque curve if loading fails
                    torqueCurvePeakRPM = 800; % RPM
                    torqueCurveWidth = 600; % RPM
                    torqueCurve = @(rpm) maxTorque * exp(-((rpm - torqueCurvePeakRPM).^2) / (2 * torqueCurveWidth^2)); % Example Gaussian torque curve
                    logMessages{end+1} = 'Using default Gaussian torque curve due to error.';
                end
        
                % Instantiate the Engine with updated efficiency and clutch
                engine = Engine(maxTorque, torqueCurve, maxRPM, minRPM, gearRatiosEngine, differentialRatio, efficiency, 0.1, clutch);
                logMessages{end+1} = 'Engine initialized successfully.';
                % --- End of Engine Initialization ---
        
                % --- Instantiate the SpeedController ---
                desiredSpeed = initialVelocity; % Assuming initialVelocity is the desired speed
                obj.pid_SpeedController = pid_SpeedController( ...
                    desiredSpeed, ...
                    maxSpeed, ...
                    Kp, ...
                    Ki, ...
                    Kd, ...
                    minAccelAtMaxSpeed, ...
                    minDecelAtMaxSpeed, ...
                    'FilterType', 'sma', 'SMAWindowSize', 50, ...
                    'Lambda1Accel', simParams.lambda1Accel, 'Lambda2Accel', simParams.lambda2Accel, ...
                    'Lambda1Vel', simParams.lambda1Vel, 'Lambda2Vel', simParams.lambda2Vel, ...
                    'Lambda1Jerk', simParams.lambda1Jerk, 'Lambda2Jerk', simParams.lambda2Jerk ...
                    ); % maxAccel and minAccel set to 2.0 and -2.0 m/s^2 respectively
                logMessages{end+1} = 'pid_SpeedController initialized successfully.';
                % --- End of SpeedController Initialization ---

                % Set the waypoints
                % obj.pid_SpeedController.setWaypoints(simParams.waypoints);
        
                % --- Instantiate the limiter_LateralControl ---
                maxAngleAtZeroSpeed = simParams.maxSteeringAngleAtZeroSpeed;
                % minAngleAtMaxSpeed = simParams.minSteeringAngleAtMaxSpeed;
                maxSpeedSteer = simParams.maxSteeringSpeed;
                % maxRateAtZeroSpeed = 90;
                % minRateAtMaxSpeed = 20;

                obj.limiter_LateralControl = limiter_LateralControl( ...
                    simParams.steeringCurveFilePath ...
                    );
                logMessages{end+1} = 'limiter_LateralControl initialized successfully.';
                % --- End of limiter_LateralControl Initialization ---
        
                % --- Instantiate the limiter_LongitudinalControl ---
                gaussianWindow = 11;  % Gaussian filter window size (must be odd)
                gaussianStd = 1.0;     % Gaussian filter standard deviation
                obj.limiter_LongitudinalControl = limiter_LongitudinalControl( ...
                    accelCurve, ...
                    decelCurve, ...
                    maxSpeedForAccelLimiting, ...
                    windowSize, ...
                    gaussianWindow, ...
                    gaussianStd ...
                    );
                obj.jerkController = jerk_Controller(0.7 * 9.81);
                obj.accController = acc_Controller(0.75, 2.0, simParams.trailerLength, tractorWheelbase, 5.5);
                obj.localizer = VehicleLocalizer(simParams.waypoints, 1.0);
                logMessages{end+1} = 'limiter_LongitudinalControl initialized successfully.';
                % --- End of limiter_LongitudinalControl Initialization ---

                % --- Instantiate the curveSpeedLimiter ---
                obj.curveSpeedLimiter = curveSpeed_Limiter();
                logMessages{end+1} = 'curveSpeed_Limiter initialized successfully.';
                % --- End of curveSpeedLimiter Initialization ---
        
                time = timeProcessed; % Update time vector
                steerAngles = -steerAngles;
        
                % Interpolate data if dtMultiplier ≠ 1
                if dtMultiplier ~= 1
                    new_time = time(1):dt:time(end);
                    
                    % Interpolate steering and acceleration signals using linear interpolation
                    steerAngles = interp1(time, steerAngles, new_time, 'linear', 'extrap');
                    accelerationData = interp1(time, accelerationData, new_time, 'linear', 'extrap');
                    tirePressureData = interp1(time, tirePressureData, new_time, 'linear', 'extrap');
                    
                    % Convert logical flags to double for interpolation
                    steeringEnded_double = double(steeringEnded);
                    accelerationEnded_double = double(accelerationEnded);
                    tirePressureEnded_double = double(tirePressureEnded);
                    
                    % Interpolate steeringEnded and accelerationEnded using nearest-neighbor interpolation
                    steeringEnded_interp = interp1(time, steeringEnded_double, new_time, 'nearest', 'extrap');
                    accelerationEnded_interp = interp1(time, accelerationEnded_double, new_time, 'nearest', 'extrap');
                    tirePressureEnded_interp = interp1(time, tirePressureEnded_double, new_time, 'nearest', 'extrap');
                    
                    % Convert interpolated flags back to logical arrays
                    steeringEnded = logical(steeringEnded_interp);
                    accelerationEnded = logical(accelerationEnded_interp);
                    tirePressureEnded = logical(tirePressureEnded_interp);
                    
                    % Update the time vector
                    time = new_time;
                    
                    % Log the interpolation
                    logMessages{end+1} = 'Interpolated simulation data to new time steps.';
                end
        
                % Update the number of steps based on new time vector
                numSteps = length(time);
        
                %% Tractor Tire Pressures and Contact Areas
                % Calculate total load for tractor
                totalLoadTractor = tractorMass * 9.81;  % Total weight in Newtons
        
                % Compute per-tire loads (assuming equal distribution for simplicity)
                perTireLoadTractor = totalLoadTractor / totalTiresTractor;
        
                % Create VehicleParameters instance for tractor with computed contact area
                tractorParams = VehicleParameters( ...
                    true, ...
                    tractorLength, ...
                    tractorWidth, ...
                    tractorHeight, ...
                    tractorCoGHeight, ...
                    tractorWheelbase, ...
                    tractorTrackWidth, ...
                    tractorNumAxles, ...
                    tractorAxleSpacing, ...
                    tractorContactAreas ...
                    );
        
                % Update masses
                tractorParams.mass = tractorMass;
        
                % Set tractor tire dimensions
                tractorParams.updateTireDimensions('tractor', tractorTireHeight, tractorTireWidth);
        
                %% Trailer Tire Pressures and Contact Areas (if trailer included)
                if simParams.includeTrailer
                    % Calculate total load for trailer
                    totalLoadTrailer = trailerMass * 9.81;  % Total weight in Newtons

                    % Compute per-tire loads
                    if isfield(simParams,'trailerBoxWeightDistributions') && ~isempty(simParams.trailerBoxWeightDistributions)
                        perTireLoadTrailer = vertcat(simParams.trailerBoxWeightDistributions{:});
                        perTireLoadTrailer = perTireLoadTrailer(:,4); % already in Newtons
                        if length(perTireLoadTrailer) ~= length(trailerTirePressures)
                            logMessages{end+1} = sprintf(['Warning: trailerBoxWeightDistributions length (%d) ' ...
                                'does not match number of trailer tires (%d). Using uniform distribution.'], ...
                                length(perTireLoadTrailer), length(trailerTirePressures));
                            perTireLoadTrailer = (trailerMass * 9.81) / length(trailerTirePressures) * ...
                                ones(length(trailerTirePressures),1);
                        end
                    else
                        perTireLoadTrailer = (trailerMass * 9.81) / totalTiresTrailer * ones(totalTiresTrailer,1); % N each
                    end
        
                    % Create VehicleParameters instance for trailer with computed contact area
                    trailerParams = VehicleParameters( ...
                        false, ...
                        simParams.trailerLength, ...
                        simParams.trailerWidth, ...
                        simParams.trailerHeight, ...
                        simParams.trailerCoGHeight, ...
                        simParams.trailerWheelbase, ...
                        simParams.trailerTrackWidth, ...
                        sum(simParams.trailerAxlesPerBox), ...
                        simParams.trailerAxleSpacing, ...
                        trailerContactAreas ...
                        );
                    trailerParams.mass = trailerMass;
                    trailerParams.boxNumAxles = simParams.trailerAxlesPerBox;
                    trailerParams.boxMasses  = boxMasses;
        
                    % Set trailer tire dimensions
                    trailerParams.updateTireDimensions('trailer', trailerTireHeight, trailerTireWidth);
                end
        
                % Initial conditions for tractor
                x = 0;
                y = 0;
                theta = 0; % Orientation of the tractor (radians)
                u = 0; % Initial longitudinal speed (m/s)
                v = 0; % Initial lateral velocity of tractor (m/s)
                r = 0; % Initial yaw rate of tractor (rad/s)
        
                % Compute axle positions for the tractor
                numAxlesTractor = tractorParams.numAxles;
                wheelbaseTractor = tractorParams.wheelbase;
        
                % Define positions for the 2 front tires
                front_x = tractorParams.wheelbase / 2;
                front_y_left = tractorParams.width / 2;
                front_y_right = -tractorParams.width / 2;
        
                if numAxlesTractor > 1
                    % Calculate positions from front axle at +wheelbase/2 to rear axle at -wheelbase/2
                    axleSpacingTractor = wheelbaseTractor / (numAxlesTractor + 1);
                    axlePositionsTractor = front_x - axleSpacingTractor : -axleSpacingTractor : front_x - numAxlesTractor * axleSpacingTractor;
                else
                    axlePositionsTractor = front_x - wheelbaseTractor / 2;
                end
        
                % Positions of front, middle, and rear axles
                frontAxlePosition = axlePositionsTractor(1);
                rearAxlePosition = axlePositionsTractor(end);
                middleAxleIndex = ceil(numAxlesTractor / 2);
                middleAxlePosition = axlePositionsTractor(middleAxleIndex);
        
                % Define hitch position between middle and rear axle
                hitchPosition = simParams.trailerHitchDistance + ((middleAxlePosition + rearAxlePosition) / 2); % Between middle and rear axle
        
                % Set the hitch_offset to be at the calculated hitch position
                hitch_offset = hitchPosition;
        
                % Initialize arrays to store positions for plotting
                tractorX = zeros(1, numSteps);
                tractorY = zeros(1, numSteps);
                tractorTheta = zeros(1, numSteps);
                steeringAnglesSim = zeros(1, numSteps);
                speedData = zeros(1, numSteps);
                horsepowerSim = zeros(1, numSteps); % Store horsepower data
        
                % Preallocate desiredGear and stability flag arrays
                desiredGear = zeros(1, numSteps);
                isWigglingArray = false(1, numSteps);
                isRolloverArray = false(1, numSteps);
                isSkiddingArray = false(1, numSteps);
                isJackknifeArray = false(1, numSteps);
        
                % --- Calculate Moment of Inertia for Tractor ---
                % Tractor parameters
                m_tractor = simParams.tractorMass;          % Tractor mass (kg)
                L_tractor = simParams.tractorLength;        % Tractor length (m)
                W_tractor = simParams.tractorWidth;         % Tractor width (m)
        
                % Moment of inertia of the tractor (approximate, assuming rectangular prism)
                I_tractor = (1/12) * m_tractor * (L_tractor^2 + W_tractor^2);
        
                % --- Check if Trailer is Included ---
                if simParams.includeTrailer
                    % Trailer parameters
                    m_trailer = trailerParams.mass;
                    L_trailer = trailerParams.length;
                    W_trailer = trailerParams.width;          % Ensure trailerParams.width exists
                    h_CoG_trailer = trailerParams.h_CoG;
                    trackWidth_trailer = trailerParams.trackWidth;
                    wheelbase_trailer = trailerParams.wheelbase;
        
                    % Moment of inertia of the trailer (approximate)
                    I_trailer = (1/12) * m_trailer * (L_trailer^2 + W_trailer^2);
                    I_trailer = I_trailer * I_trailerMultiplier; % Apply multiplier
        
                    % Convert max articulation angle to radians
                    max_delta = deg2rad(maxDeltaDeg);
        
                    % Initialize trailer states
                    delta = 0; % Initial articulation angle
                    psi_trailer = theta + delta; % Orientation of the trailer
        
                    % Compute initial position of the hitch point
                    x_hitch = x + hitch_offset * cos(theta);
                    y_hitch = y + hitch_offset * sin(theta);
        
                    % Compute initial position of the trailer's center of mass
                    % Distance from hitch to trailer's center of gravity
                    distance_hitch_to_cog = L_trailer / 2 - (middleAxlePosition - hitchPosition);
        
                    x_trailer = x_hitch - distance_hitch_to_cog * cos(psi_trailer);
                    y_trailer = y_hitch - distance_hitch_to_cog * sin(psi_trailer);
        
                    % Initialize trailer velocities and yaw rate
                    u_trailer = u; % For simplicity, assume same initial speed as tractor
                    v_trailer = 0; % Assuming negligible sideslip difference
                    r_trailer = 0; % Initial yaw rate of trailer (rad/s)
        
                    trailerX = zeros(1, numSteps);
                    trailerY = zeros(1, numSteps);
                    trailerTheta = zeros(1, numSteps);
                else
                    % If trailer is not included, define default values
                    I_trailer = 0;
                    m_trailer = 0;
                    L_trailer = 0;
                    W_trailer = 0;
                    h_CoG_trailer = 0;
                    trackWidth_trailer = 0;
                    wheelbase_trailer = 0;
                    max_delta = 0;
                    loadDistributionTrailer = [];
                    centerOfGravityTrailer = [];
                end
        
                % --- Compute Total Moment of Inertia ---
                I_total = I_tractor + I_trailer;
        
                % --- Log Inertia Values ---
                logMessages{end+1} = sprintf('Tractor Moment of Inertia (I_tractor): %.2f kg·m²', I_tractor);
                if simParams.includeTrailer
                    logMessages{end+1} = sprintf('Trailer Moment of Inertia (I_trailer): %.2f kg·m²', I_trailer);
                else
                    logMessages{end+1} = 'No trailer included. Trailer Moment of Inertia (I_trailer) set to 0 kg·m².';
                end
                logMessages{end+1} = sprintf('Total Moment of Inertia (I_total): %.2f kg·m²', I_total);
        
                % Initialize load distribution and tire positions for the tractor
                % [x, y, z, load, contact_area]
                loadDistribution = [];
        
                % Compute total front load (assuming the front carries 3 times the tractor mass)
                front_load_total = tractorMass * 3 * 9.81; % Total front load in Newtons
                front_load_per_tire = front_load_total / 2;  % Load per front tire
        
                % Add Front Left Tire to loadDistribution with contact area placeholder
                loadDistribution(end+1, :) = [
                    front_x,        % x-position (m)
                    front_y_left,   % y-position (m)
                    tractorCoGHeight,  % z-position (m)
                    front_load_per_tire,  % vertical load (N)
                    tractorContactAreas(1)  % contact area (m²)
                ]; % Front Left Tire
        
                % Add Front Right Tire to loadDistribution with contact area placeholder
                loadDistribution(end+1, :) = [
                    front_x,         % x-position (m)
                    front_y_right,   % y-position (m)
                    tractorCoGHeight,  % z-position (m)
                    front_load_per_tire,  % vertical load (N)
                    tractorContactAreas(2)  % contact area (m²)
                ]; % Front Right Tire
        
                % Calculate the positions of each axle based on the number of axles
                if tractorNumAxles > 0
                    if tractorNumAxles > 1
                        % Evenly space the axles between the front and rear based on wheelbase
                        axle_spacing = tractorWheelbase / (tractorNumAxles + 1);
                        axle_positions = front_x - axle_spacing : -axle_spacing : front_x - tractorNumAxles * axle_spacing;
                    else
                        % Single axle positioned at the center of the wheelbase
                        axle_positions = front_x - tractorWheelbase / 2;
                    end
        
                    % Iterate over each axle to add corresponding tires
                    for i = 1:tractorNumAxles
                        axle_x = axle_positions(i);
                        for j = 1:numTiresPerAxleTractor
                            % Alternate y-positions for left and right tires
                            if j == 1
                                y_pos = tractorParams.width / 2;
                                contactArea = tractorContactAreas(2 + (i-1)*numTiresPerAxleTractor + j);
                            else
                                y_pos = -tractorParams.width / 2;
                                contactArea = tractorContactAreas(2 + (i-1)*numTiresPerAxleTractor + j);
                            end
        
                            % Compute total rear load (assuming the rear carries 2 times the tractor mass)
                            rear_load_total = tractorMass * 2 * 9.81; % Total rear load in Newtons
                            rear_load_per_tire = rear_load_total / (tractorNumAxles * numTiresPerAxleTractor); % Load per rear tire
        
                            % Add Tire to loadDistribution with contact area
                            loadDistribution(end+1, :) = [
                                axle_x,       % x-position (m)
                                y_pos,        % y-position (m)
                                tractorCoGHeight,  % z-position (m)
                                rear_load_per_tire,    % vertical load (N)
                                contactArea             % contact area (m²)
                            ];
                        end
                    end
                end
        
                % --- Select and Validate Pressure Matrices for Tractor ---
                % Calculate total number of tractor tires
                totalNumberOfTiresTractor = size(loadDistribution, 1);
        
                % Generate the pressure matrix key based on the number of tractor tires
                pressureMatrixKeyTractor = sprintf('Tires%d', totalNumberOfTiresTractor);
        
                % Check if the pressure matrix exists
                % (Already handled earlier, assuming selectedPressuresTractor is derived from selectedPressures)
        
                % --- Compute Contact Area Based on Tire Pressure for Tractor ---
                for i = 1:size(loadDistribution,1)
                    if tractorTirePressures(i) > 0
                        loadDistribution(i,5) = loadDistribution(i,4) / tractorTirePressures(i); % contact area in m²
                    else
                        loadDistribution(i,5) = 0; % Flat tire or invalid pressure
                        logMessages{end+1} = sprintf('Warning: Tractor Tire %d has zero or invalid pressure (%.2f Pa). Setting contact area to 0.', i, tractorTirePressures(i));
                    end
                end
        
                % **Adjust Vertical Loads Based on Road Roughness**
                % Road roughness affects the vertical loads on tires
                loadDistribution(:,4) = loadDistribution(:,4) .* (1 - 0.3 * roadRoughness); % Reduce loads on rough surfaces
                logMessages{end+1} = sprintf('Adjusted tractor vertical loads based on road roughness (%.2f).', roadRoughness);
        
                % --- Center of Gravity Calculation for Tractor ---
                loads = loadDistribution(:,4);
                positions = loadDistribution(:,1:3);
                centerOfGravity = KinematicsCalculator.calculateCenterOfGravity(loads, positions);
        
                %% Initialize load distribution and center of gravity for the trailer
                if simParams.includeTrailer
                    if isfield(simParams,'trailerBoxWeightDistributions') && ~isempty(simParams.trailerBoxWeightDistributions)
                        % Load distribution is directly provided per trailer box
                        loadDistributionTrailer = vertcat(simParams.trailerBoxWeightDistributions{:});
                        expectedRows = sum(simParams.trailerAxlesPerBox) * simParams.numTiresPerAxleTrailer;
                        if size(loadDistributionTrailer,1) ~= expectedRows
                            logMessages{end+1} = sprintf('Normalizing trailerBoxWeightDistributions from %d to %d rows.', size(loadDistributionTrailer,1), expectedRows);
                            loadDistributionTrailer = normalizeTrailerLoadDistribution(loadDistributionTrailer, expectedRows);
                        end

                        % Append computed contact areas to the distribution
                        numRowsTrailer = size(loadDistributionTrailer,1);
                        numAreasTrailer = length(trailerContactAreas);
                        if numRowsTrailer ~= numAreasTrailer
                            logMessages{end+1} = sprintf(['Warning: trailerContactAreas length (%d) ' ...
                                'does not match trailerBoxWeightDistributions rows (%d). ' ...
                                'Using averaged contact area.'], numAreasTrailer, numRowsTrailer);
                            if numAreasTrailer > 0
                                avgArea = mean(trailerContactAreas);
                            else
                                avgArea = trailerTireWidth * trailerTireHeight;
                            end
                            trailerContactAreasToUse = repmat(avgArea, numRowsTrailer, 1);
                        else
                            trailerContactAreasToUse = trailerContactAreas(:);
                        end
                        loadDistributionTrailer(:,5) = trailerContactAreasToUse;

                        % Adjust vertical loads for road roughness
                        loadDistributionTrailer(:,4) = loadDistributionTrailer(:,4) .* (1 - 0.3 * roadRoughness);

                        % Compute center of gravity
                        loadsTrailer = loadDistributionTrailer(:,4);
                        positionsTrailer = loadDistributionTrailer(:,1:3);
                        centerOfGravityTrailer = KinematicsCalculator.calculateCenterOfGravity(loadsTrailer, positionsTrailer);
                    else
                        % Determine axle positions for each trailer box
                        numAxlesTrailerVec = trailerParams.boxNumAxles;
                        if isempty(numAxlesTrailerVec)
                            % Fallback to total axles if per-box info not available
                            numAxlesTrailerVec = trailerParams.numAxles;
                        end
                        numAxlesTrailer = sum(numAxlesTrailerVec);
                        numTiresPerAxleTrailer = simParams.numTiresPerAxleTrailer;
                        numTiresTrailer = numAxlesTrailer * numTiresPerAxleTrailer;

                        axlePositionsTrailer = zeros(1, numAxlesTrailer);
                        nextIdx = 1;
                        offset = 0;
                        for b = 1:numel(numAxlesTrailerVec)
                            nAx = numAxlesTrailerVec(b);
                            positionsBox = linspace(-trailerParams.wheelbase/2, trailerParams.wheelbase/2, nAx) - offset;
                            axlePositionsTrailer(nextIdx:nextIdx+nAx-1) = positionsBox;
                            nextIdx = nextIdx + nAx;
                            offset = offset + trailerParams.wheelbase + simParams.trailerBoxSpacing;
                        end
                        loadDistributionTrailer = zeros(numTiresTrailer, 5); % [x, y, z, load, contact_area]
                        weightPerTireTrailer = (trailerParams.mass * 9.81) / numTiresTrailer;

                        % Calculate the contact area per trailer tire
                        contactAreaPerTire = repmat(trailerTireWidth * trailerTireHeight, numTiresTrailer, 1); % m²
                        totalContactArea = sum(contactAreaPerTire);

                        % Store tire information for logging
                        tireInfo = cell(numTiresTrailer, 1); % For logging purposes

                        for i_axle = 1:numAxlesTrailer
                            axlePos = axlePositionsTrailer(i_axle);
                            tireIndexStart = (i_axle-1)*numTiresPerAxleTrailer + 1;
                            y_positions_trailer = linspace(-trailerParams.trackWidth / 2, trailerParams.trackWidth / 2, numTiresPerAxleTrailer);
                            for j_tire = 1:numTiresPerAxleTrailer
                                tireIndex = tireIndexStart + j_tire - 1;
                                y_pos = y_positions_trailer(j_tire);
                                loadDistributionTrailer(tireIndex, :) = [axlePos, y_pos, trailerParams.h_CoG, weightPerTireTrailer, trailerContactAreas(tireIndex)];
                                tireInfo{tireIndex} = sprintf('Trailer Tire %d: Position (%.2f, %.2f), Load %.2f N', ...
                                    tireIndex, axlePos, y_pos, weightPerTireTrailer);
                            end
                        end

                        logMessages{end+1} = 'Trailer Tire Positions and Initial Loads:';
                        logMessages = [logMessages(:); tireInfo(:)];

                        for i = 1:size(loadDistributionTrailer,1)
                            if trailerTirePressures(i) > 0
                                loadDistributionTrailer(i,5) = loadDistributionTrailer(i,4) / trailerTirePressures(i);
                            else
                                loadDistributionTrailer(i,5) = 0;
                                logMessages{end+1} = sprintf('Warning: Trailer Tire %d has zero or invalid pressure (%.2f Pa). Setting contact area to 0.', i, trailerTirePressures(i));
                            end
                        end

                        totalContactAreaTrailer = sum(loadDistributionTrailer(:,5));
                        logMessages{end+1} = sprintf('Total Trailer Contact Area: %.4f m²', totalContactAreaTrailer);

                        loadDistributionTrailer(:,4) = loadDistributionTrailer(:,4) .* (1 - 0.3 * roadRoughness);
                        logMessages{end+1} = 'Adjusted trailer vertical loads based on road roughness.';

                        loadsTrailer = loadDistributionTrailer(:,4);
                        positionsTrailer = loadDistributionTrailer(:,1:3);
                        centerOfGravityTrailer = KinematicsCalculator.calculateCenterOfGravity(loadsTrailer, positionsTrailer);
                    end
                else
                    % If trailer is not included, use the tractor's load distribution
                    % No additional action needed here
                end
        
                % --- Initialize Tire Model ---
                B = 10;    % Stiffness factor
                C = 1.9;   % Shape factor
                D = 8000;  % Peak factor (N)
                E = 0.97;  % Curvature factor
                % --- Incorporate Tire Width into Tire Model (Optional) ---
                tireWidth = simParams.tractorTireWidth; % meters
                % Assuming PacejkaMagicFormula can accept tireWidth as an additional parameter
                simpleTireModel = PacejkaMagicFormula(B, C, D, E);
                % --- End of Tire Width Integration ---
        
                % Define coefficients for high-fidelity tire model
                coefficients = struct();
                % Longitudinal Coefficients
                coefficients.pCx1 = simParams.pCx1;
                coefficients.pDx1 = simParams.pDx1;    % N
                coefficients.pDx2 = simParams.pDx2;    % N
                coefficients.pEx1 = simParams.pEx1;
                coefficients.pEx2 = simParams.pEx2;
                coefficients.pEx3 = simParams.pEx3;
                coefficients.pEx4 = simParams.pEx4;
                coefficients.pKx1 = simParams.pKx1;
                coefficients.pKx2 = simParams.pKx2;
                coefficients.pKx3 = simParams.pKx3;
        
                % Updated Lateral Coefficients
                coefficients.pCy1 = simParams.pCy1;    % Shape factor
                coefficients.pDy1 = simParams.pDy1;    % Peak lateral force
                coefficients.pDy2 = simParams.pDy2;    % Peak lateral force
                coefficients.pEy1 = simParams.pEy1;   % Curvature factor (unusually high)
                coefficients.pEy2 = simParams.pEy2;    % Curvature adjustments
                coefficients.pEy3 = simParams.pEy3;
                coefficients.pEy4 = simParams.pEy4;
                coefficients.pKy1 = simParams.pKy1;    % Cornering stiffness
                coefficients.pKy2 = simParams.pKy2;
                coefficients.pKy3 = simParams.pKy3;
        
                highFidelityTireModel = Pacejka96TireModel(coefficients);
        
                % Initialize Suspension Model using GUI-provided parameters
                K_spring = simParams.K_spring;       % Spring stiffness (N/m)
                C_damping = simParams.C_damping;     % Damping coefficient (N·s/m)
                restLength = simParams.restLength;   % Rest length (m)
        
                % **Adjust Suspension Parameters Based on Road Roughness**
                K_spring = K_spring * (1 - 0.2 * roadRoughness);
                C_damping = C_damping * (1 + 0.5 * roadRoughness);
                logMessages{end+1} = sprintf('Adjusted suspension stiffness to %.2f N/m and damping to %.2f N·s/m based on road roughness.', K_spring, C_damping);
        
                suspensionModel = LeafSpringSuspension( ...
                    K_spring, ...
                    C_damping, ...
                    restLength, ...
                    tractorMass, ...
                    tractorTrackWidth, ...
                    tractorWheelbase ...
                    );
        
                %% --- Instantiate the ForceCalculator Considering Both Load Distributions ---
                if simParams.includeTrailer
                    vehicleType = 'tractor-trailer';
                    % Combine tractor and trailer load distributions
                    loadDistributionToUse = [loadDistribution; loadDistributionTrailer];
        
                    % Log the combined load distribution
                    logMessages{end+1} = sprintf('Combined load distribution for tractor and trailer with a total of %d tires.', size(loadDistributionToUse, 1));
                else
                    vehicleType = 'tractor';
                    loadDistributionToUse = loadDistribution; % Use tractor's load distribution only
        
                    % Log the tractor-only load distribution
                    logMessages{end+1} = sprintf('Using load distribution for tractor only with a total of %d tires.', size(loadDistributionToUse, 1));
                end
        
                % --- Center of Gravity Calculation ---
                if simParams.includeTrailer
                    combinedLoads = loadDistributionToUse(:,4);
                    combinedPositions = loadDistributionToUse(:,1:3);
                    centerOfGravity = KinematicsCalculator.calculateCenterOfGravity(combinedLoads, combinedPositions);
        
                    % Log the combined CoG
                    logMessages{end+1} = sprintf('Combined Center of Gravity: (%.2f, %.2f, %.2f)', centerOfGravity(1), centerOfGravity(2), centerOfGravity(3));
                else
                    loads = loadDistributionToUse(:,4);
                    positions = loadDistributionToUse(:,1:3);
                    centerOfGravity = KinematicsCalculator.calculateCenterOfGravity(loads, positions);
        
                    % Log the tractor CoG
                    logMessages{end+1} = sprintf('Tractor Center of Gravity: (%.2f, %.2f, %.2f)', centerOfGravity(1), centerOfGravity(2), centerOfGravity(3));
                end
        
                % Define wind speed and direction
                wind_speed = simParams.windSpeed; % m/s
                wind_angle_deg = simParams.windAngleDeg; % degrees from the positive X-axis towards positive Y-axis
                wind_angle_rad = deg2rad(wind_angle_deg);
        
                % Calculate wind components
                wind_u = wind_speed * cos(wind_angle_rad); % X-component
                wind_v = wind_speed * sin(wind_angle_rad); % Y-component
                wind_w = 0; % Z-component (no vertical wind)
        
                % Define wind vector
                windVector = [wind_u; wind_v; wind_w]; % [u; v; w] in global frame (m/s)
        
                if simParams.includeTrailer
                    totalMassToUse = tractorParams.mass + trailerParams.mass; % total mass
                else
                    totalMassToUse = tractorParams.mass;
                end
        
                %% --- Instantiate the ForceCalculator with Combined Load Distribution ---
                % Define additional parameters for ForceCalculator
                if simParams.includeTrailer
                    trailerMassVal = trailerParams.mass;
                    trailerWheelbaseVal = trailerParams.wheelbase;
                    numTrailerTiresVal = simParams.numTiresPerAxleTrailer * trailerParams.numAxles;
                else
                    trailerMassVal = 0;
                    trailerWheelbaseVal = 0;
                    numTrailerTiresVal = 0;
                end
        
                trailerInertiaVal = 0;
                if simParams.includeTrailer
                    %% Instantiate HitchModel
                    % Define Hitch Points in respective frames
                    tractorHitchPoint = [hitch_offset; 0; tractorCoGHeight];       % [x; y; z] in tractor's frame
                    trailerKingpinPoint = [0; 0; trailerParams.h_CoG];              % [x; y; z] in trailer's frame (kingpin at origin)
        
                    % Define Stiffness and Damping Coefficients for HitchModel
                    stiffness = struct(...
                        'x', simParams.stiffnessX, ...     % Longitudinal stiffness (N/m)
                        'y', simParams.stiffnessY, ...     % Lateral stiffness (N/m)
                        'z', simParams.stiffnessZ, ...     % Vertical stiffness (N/m)
                        'roll', simParams.stiffnessRoll, ...  % Roll stiffness (N·m/rad)
                        'pitch', simParams.stiffnessPitch, ... % Pitch stiffness (N·m/rad)
                        'yaw', simParams.stiffnessYaw ...    % Yaw stiffness (N·m/rad)
                    );
        
                    damping = struct(...
                        'x', simParams.dampingX, ...     % Longitudinal damping (N·s/m)
                        'y', simParams.dampingY, ...     % Lateral damping (N·s/m)
                        'z', simParams.dampingZ, ...     % Vertical damping (N·s/m)
                        'roll', simParams.dampingRoll, ...  % Roll damping (N·m·s/rad)
                        'pitch', simParams.dampingPitch, ... % Pitch damping (N·m·s/rad)
                        'yaw', simParams.dampingYaw ...    % Yaw damping (N·m·s/rad)
                    );
        
                    % Instantiate HitchModel for tractor to first trailer box
                    dt_hitch = dt; % Use the same time step
                    hitchModel = HitchModel(tractorHitchPoint, trailerKingpinPoint, stiffness, damping, max_delta, wheelbase_trailer, loadDistributionTrailer, dt_hitch);
                    % Align initial hitch orientation with trailer
                    hitchModel = hitchModel.initializeState(psi_trailer, 0);
                    % Instantiate Spinner HitchModels for additional trailer boxes
                    spinnerModels = {};
                    nSpinners = max(simParams.trailerNumBoxes - 1, 0);
                    for iSpinner = 1:nSpinners
                        spCfg = simParams.spinnerConfigs{iSpinner};
                        stiff_sp = spCfg.stiffness;
                        damp_sp = spCfg.damping;
                        tractorHitchPoint_sp = [simParams.trailerBoxSpacing; 0; simParams.trailerCoGHeight];
                        trailerKingpinPoint_sp = [0; 0; simParams.trailerCoGHeight];
                        spinnerModels{iSpinner} = HitchModel(tractorHitchPoint_sp, trailerKingpinPoint_sp, stiff_sp, damp_sp, max_delta, simParams.trailerWheelbase, loadDistributionTrailer, dt_hitch);
                        % Ensure additional boxes start aligned
                        spinnerModels{iSpinner} = spinnerModels{iSpinner}.initializeState(psi_trailer, 0);
                    end
                    % Preallocate per-box trailer orientations for multi-box articulation
                    nBoxes = simParams.trailerNumBoxes;
                    trailerThetaBoxes = zeros(nBoxes, numSteps);
        
                    % Define roll angle threshold (in radians)
                    rollThreshold = deg2rad(30); % 30 degrees in radians
                    hitchInstabilityThreshold = 10000; % 10,000 N
        
                    % Calculate trailer inertia
                    trailerInertiaVal = simParams.I_trailerMultiplier * hitchModel.trailerInertia;
                end
        
                % --- Instantiate the ForceCalculator with Combined Load Distribution ---
                forceCalc = ForceCalculator(...
                    vehicleType, ...                      % vehicleType ('tractor' or 'tractor-trailer')
                    totalMassToUse, ...                   % total mass
                    mu, ...                               % frictionCoefficient (adjusted based on road surface)
                    [u; v; 0], ...                        % velocity vector
                    dragCoeff, ...                        % dragCoeff 
                    airDensity, ...                       % airDensity
                    frontalArea, ...                      % frontalArea
                    sideArea, ...                         % sideArea
                    0.8, ...                              % sideForceCoefficient
                    Inf, ...                              % turnRadius (set as needed)
                    loadDistributionToUse, ...            % combined loadDistribution
                    centerOfGravity, ...                  % centerOfGravity
                    tractorCoGHeight, ...                 % z-coordinate of CoG
                    [0; 0; r], ...                        % angular velocity vector
                    slopeAngleRad, ...                    % slopeAngle (in radians)
                    tractorTrackWidth, ...                % trackWidth
                    tractorWheelbase, ...                 % wheelbase
                    simpleTireModel, ...                  % tireModel (now defined)
                    suspensionModel, ...                  % suspensionModel
                    trailerInertiaVal, ...                % trailerInertia (0 if no trailer)
                    dt, ...                               % time step
                    trailerMassVal, ...                   % trailerMass (0 if no trailer)
                    trailerWheelbaseVal, ...              % trailerWheelbase (0 if no trailer)
                    numTrailerTiresVal, ...               % numTrailerTires (0 if no trailer)
                    boxMasses, ...                        % trailer box masses
                    'highFidelity', ...                   % simulation fidelity
                    highFidelityTireModel, ...            % high-fidelity tire model
                    windVector, ...                       % wind speed and direction as a vector in 3D
                    brakeSystem ...                       % BrakeSystem instance
                );
                        
                % -------------------------------------------------------------------
                % %%% NEW LINES %%%: Initialize wheelSpeeds, wheelRadius, wheelInertia
                % -------------------------------------------------------------------
                % Assume only the tractor wheels are driven. Including trailer
                % tires here would unrealistically multiply available traction
                % when additional boxes are added.
                numDriveTires = totalTiresTractor;
                    
                % %%% NEW LINES for WHEEL INERTIA CALCULATION %%%
                %
                % Let's do a simple approximation for the wheel inertia:
                %  1) Assume each wheel + tire weighs ~80 kg (example for large truck).
                %  2) Outer radius ~0.50 m, inner radius ~0.45 m => average ~0.475 m.
                %  3) For a "solid disc" approximation: I = 0.5 * m * R^2
                %  4) For a "thin ring": I = m * R^2
                %
                % We'll pick a factor in between (say ~0.7) to reflect a rim+spokes distribution.
                % That means: I = 0.7 * m * R^2
                %
                wheelMass   = 80;         % (kg) approximate mass per wheel+tire
                outerRadius = 0.50;       % (m)
                innerRadius = 0.45;       % (m)
                Ravg        = (outerRadius + innerRadius)/2; 
                factor      = 0.7;        % factor between 0.5 (solid disc) and 1.0 (thin ring)
                I_perWheel  = factor * wheelMass * (Ravg^2);  % => ~0.7 * 80 * 0.475^2
            
                % Now, if each wheel is the same, total wheel inertia might just be I_perWheel.
                % But the ForceCalculator uses "wheelInertia" as the *per-wheel* inertia.
                %
                % We have multiple wheels, but "wheelInertia" can be a scalar if all wheels are identical:
                forceCalc.wheelSpeeds  = zeros(numDriveTires, 1);  % all zeros initially
                forceCalc.wheelInertia = I_perWheel;               % (kg·m^2) per wheel
                forceCalc.enableSpeedController = simParams.enableSpeedController;
                % --- Set Flat Tires in ForceCalculator ---
                if ~isempty(flatTireIndices)
                    forceCalc = forceCalc.setFlatTire(flatTireIndices);
                end
        
                % --- Logging Final Load Distribution ---
                logMessages{end+1} = 'Load distribution initialized successfully for the vehicle.';
        
                % Initial state for DynamicsUpdater (tractor)
                initialState.position = [x; y];
                initialState.velocity = u; % Scalar longitudinal speed
                initialState.orientation = theta;
                initialState.lateralVelocity = v;
                initialState.yawRate = r;
                initialState.rollAngle = 0;   % Initialize roll angle to zero
                initialState.rollRate = 0;    % Initialize roll rate to zero
        
                % Compute Hitch Point Distance (example value)
                hitchPointDistance = hitch_offset; % meters
        
                % Update to include trackWidth for DynamicsUpdater
                trackWidth = tractorTrackWidth;
        
                % For the tractor
                K_roll_tractor = 50000;  % N·m/rad
                D_roll_tractor = 5000;   % N·m·s/rad
                I_roll_tractor = 2500;   % kg·m²
        
                % For the trailer (if included)
                K_roll_trailer = 30000;  % N·m/rad
                D_roll_trailer = 3000;   % N·m·s/rad
                I_roll_trailer = 2000;   % kg·m²
        
                % Instantiate the KinematicsCalculator with the appropriate CoG
                kinematicsCalc = KinematicsCalculator( ...
                    forceCalc, ...
                    tractorCoGHeight, ...
                    tractorCoGHeight, ... % Use trailer's CoG if included
                    K_roll_tractor, ...
                    D_roll_tractor, ...
                    I_roll_tractor, ...
                    K_roll_trailer, ...
                    D_roll_trailer, ...
                    I_roll_trailer, ...
                    dt ...
                    );
        
                if simParams.includeTrailer
                    wheelBasetoUse = tractorWheelbase + trailerWheelbase;
                    widthtoUse = tractorWidth + trailerWidth;
                else
                    wheelBasetoUse = tractorWheelbase;
                    widthtoUse = tractorWidth;
                end
        
                % --- Instantiate the DynamicsUpdater ---
                dynamicsUpdater = DynamicsUpdater( ...
                    forceCalc, ...
                    kinematicsCalc, ...
                    initialState, ...
                    totalMassToUse, ...
                    wheelBasetoUse, ...
                    tractorCoGHeight, ...
                    widthtoUse, ...
                    dt, ...
                    vehicleType, ...
                    trackWidth, ...
                    K_roll_tractor, ...   % N·m/rad
                    D_roll_tractor, ...   % N·m·s/rad
                    transmission ...      % Transmission instance
                    );
        
                % Directional stability parameters
                U = -1.8; % deg/g (negative for oversteer)
                L = 3.5; % m (wheelbase)
                
                % Tolerance
                tolerance = 0.10; % 10% tolerance
                if simParams.includeTrailer
                    %% Instantiate StabilityChecker
                    stabilityChecker = StabilityChecker(...
                        dynamicsUpdater, ...
                        dynamicsUpdater.forceCalculator, ...
                        dynamicsUpdater.kinematicsCalculator, ...
                        m_trailer, ...   % m_trailer
                        L_trailer, ...   % L_trailer
                        mu, ...          % mu
                        tractorCoGHeight ... % h_CoG (of trailer)
                        );
                end
        
                %% Initialize arrays to store simulation data
                n = numSteps; % Number of time steps
                timeArray = time;
                positionX = zeros(n, 1);
                positionY = zeros(n, 1);
                orientationArray = zeros(n, 1);
                velocityU = zeros(n, 1);
                lateralVelocityV = zeros(n, 1);
                yawRateR = zeros(n, 1);
                rollAngleArray = zeros(n, 1);     % New array to store roll angles
                rollRateArray = zeros(n, 1);      % New array to store roll rates
                accelerationLongitudinal = zeros(n, 1);
                accelerationLateral = zeros(n, 1);
                % Initialize jerk arrays (time derivative of acceleration)
                jerkLongitudinal = zeros(n, 1);
                jerkLateral = zeros(n, 1);
                steeringAnglesSim = zeros(n, 1);
        
                %% Simulation Loop
                logMessages{end+1} = '--- Starting Simulation ---';
        
                % Define Middle Axle Distance from Tractor's Front Axle
                middleAxleDistance = simParams.tractorHitchDistance; % in meters (Adjust based on your tractor's configuration)
        
                % Define Additional Rearward Shift if necessary
                additionalRearShift = 0.5; % Set to 0 for direct alignment, for now it is 50cm / half meter
        
                % Update Hitch Offset to Align Trailer's Hitch with Middle Axle
                hitch_offset_plot = middleAxleDistance + additionalRearShift; % Should be equal to middleAxleDistance if additionalRearShift is 0
        
                % Log hitch alignment parameters
                logMessages{end+1} = sprintf('Middle Axle Distance: %.2f meters', middleAxleDistance);
                logMessages{end+1} = sprintf('Additional Rearward Shift: %.2f meters', additionalRearShift);
                logMessages{end+1} = sprintf('Hitch Offset Plot: %.2f meters', hitch_offset_plot);
                logMessages{end+1} = '----------------------------------------';
        
                % Initialize a flag to track if speed has reached zero
                speedZeroReached = false;
        
                % Define how often to update the waitbar (e.g., every 1%)
                reportInterval = ceil(numSteps / 100); % Update every 1%
        
                % --- BrakeSystem Integration Begins ---
                % Ensure that the ForceCalculator has the BrakeSystem instance
                forceCalc.brakeSystem = brakeSystem;
                % --- BrakeSystem Integration Ends ---
                
                % Vehicle parameters
                % Example of speed-dependent lookahead:
                baseLookahead = 50; % base value
                speedFactor = min(u/10, 3); % scale with speed but limit growth
                lookaheadDistance = baseLookahead * speedFactor;

                alpha = 0.1;
                predictionTime = 0.5; % sec 
                numPredictions = 500;
                planningHorizon = 500;
                historyBufferSize = 30;
                gaussianBufferSize = 3;
                curvatureShiftThreshold = 0.1; % Shift down when radius < 10 meters
                gaussianSigma = 1;

                obj.simParams.guiManager.generateWaypoints();
                if obj.simulationName == "simulation_Vehicle2"
                    offsetX = obj.uiManager.getOffsetX();
                    offsetY = obj.uiManager.getOffsetY();
                    rotationAngleDeg = obj.uiManager.getRotationAngleVehicle2(); % Degrees
                    rotationAngleRad = deg2rad(rotationAngleDeg); % Radians% Define Rotation Matrix
                    R = [cos(rotationAngleRad), -sin(rotationAngleRad);
                         sin(rotationAngleRad),  cos(rotationAngleRad)];
                elseif obj.simulationName == "simulation_Vehicle1"
                    offsetX = obj.uiManager.getvehicle1OffsetX();
                    offsetY = obj.uiManager.getvehicle1OffsetY();
                    rotationAngleDeg = obj.uiManager.getRotationAngleVehicle1(); % Degrees
                    rotationAngleRad = deg2rad(rotationAngleDeg); % Radians% Define Rotation Matrix
                    R = [cos(rotationAngleRad), -sin(rotationAngleRad);
                         sin(rotationAngleRad),  cos(rotationAngleRad)];
                end

                % Create purePursuit_PathFollower object
                purePursuitPathFollower = purePursuit_PathFollower( ...
                    simParams.waypoints, ...
                    wheelBasetoUse, ...
                    lookaheadDistance, ...
                    simParams.maxSteeringAngleAtZeroSpeed, ...
                    alpha, ...
                    predictionTime, ...
                    numPredictions, ...
                    planningHorizon, ...
                    0.1, ...
                    R, ...
                    offsetX, ...
                    offsetY, ...
                    historyBufferSize, ...
                    transmission, ...
                    curvatureShiftThreshold, ...
                    gaussianBufferSize, ...
                    gaussianSigma ...
                    );
                throttle = Throttle(1,0.1);

                % --- Auto-export of all local variables  --------------------------
                thisFuncVars = setdiff(who, {'obj', 'output', 'fn', 'k'});   % all except 'obj' & 'output'
                for k = 1:numel(thisFuncVars)
                    varName = thisFuncVars{k};
                    output.(varName) = eval(varName);             % output.varName = varName;
                end
                % ------------------------------------------------------------------------
            catch ME
                try
                    [txtFilename, csvFilename] = obj.saveLogs(logMessages); % Capture filenames
                    disp(['Logs saved to ', txtFilename, ' and ', csvFilename]);
                catch logError
                    warning('Failed to save simulation logs due to: %s', logError.message);
                end
                rethrow(ME); % Re-throw the error after logging
            end
        end
        
        function [output] = computeNextSimFrame(obj,input)
            try    
                % --- Auto-import of all variables within 'input' ------------
                fn = fieldnames(input);          % names
                tot = numel(fn);
                for k = 1:tot
                    fld = fn{k};                 % name (string)
                    eval([fld ' = input.' fld ';']);      % creates local variable
                end
                % -------------------------------------------------------------------------
                for i = 1:numSteps
                    % --- Update Waitbar ---
                    if mod(i, reportInterval) == 0 || i == numSteps
                        progress = (i / numSteps) * 100;
                        waitbar(i / numSteps, hWaitbar, sprintf('Simulation Progress: %.2f%%', progress));
                    end
                    
                    templim = find(tirePressureEnded, 1, 'first');
                    if  i <= templim
                        selectedPressures = tirePressureData(i, :);
                        simParams.pressureMatrices = selectedPressures;
                        % obj.guiManager.setPressureMatrices(selectedPressures);
                    else
                        selectedPressures = tirePressureData(find(tirePressureEnded, 1, 'first'), :);
                        simParams.pressureMatrices = selectedPressures;
                    end

                    % --- Split the pressures into tractor and trailer pressures ---
                    tractorTirePressures = selectedPressures(1:totalTiresTractor);
                    if simParams.includeTrailer
                        trailerTirePressures = selectedPressures(totalTiresTractor+1:end);
                    else
                        trailerTirePressures = [];
                    end

                    % Validate the lengths
                    if length(tractorTirePressures) ~= totalTiresTractor
                        error('Mismatch in number of tractor tire pressures.');
                    end
                    if simParams.includeTrailer && length(trailerTirePressures) ~= totalTiresTrailer
                        error('Mismatch in number of trailer tire pressures.');
                    end
                    % --- End of Split Pressures ---

                    % --- Begin Flat Tire Logic ---
                    % Define Pressure Threshold
                    % Convert 14 psi to Pascals (1 psi ≈ 6895 Pa)
                    pressureThresholdPa = 14 * 6895; % 14 psi in Pa

                    % Log the pressure threshold
                    logMessages{end+1} = sprintf('Pressure Threshold set to %.2f Pa (14 psi).', pressureThresholdPa);

                    % --- Check and Set Flat Tires for Tractor ---
                    for n = 1:length(tractorTirePressures)
                        if tractorTirePressures(n) < pressureThresholdPa
                            % Collect the index of the flat tire
                            flatTireIndicesTractor(end+1) = n;
                            % Log the flat tire event
                            logMessages{end+1} = sprintf('Warning: Tractor Tire %d is flat (Pressure: %.2f Pa < %.2f Pa).', n, tractorTirePressures(n), pressureThresholdPa);
                            % Mark the tire as flat by setting its contact area to zero
                            tractorTirePressures(n) = 0; 
                        end
                    end

                    % --- Check and Set Flat Tires for Trailer ---
                    if simParams.includeTrailer
                        for n = 1:length(trailerTirePressures)
                            if trailerTirePressures(n) < pressureThresholdPa
                                % Adjust the index to account for the total number of tractor tires
                                adjustedIndex = n + length(tractorTirePressures);
                                % Collect the index of the flat tire
                                flatTireIndicesTrailer(end+1) = adjustedIndex;
                                % Log the flat tire event
                                logMessages{end+1} = sprintf('Warning: Trailer Tire %d is flat (Pressure: %.2f Pa < %.2f Pa).', n, trailerTirePressures(n), pressureThresholdPa);
                                % Mark the tire as flat by setting its contact area to zero
                                trailerTirePressures(n) = 0; 
                            end
                        end
                    end
                    % Combine flat tire indices
                    flatTireIndices = [flatTireIndicesTractor, flatTireIndicesTrailer];
                    % --- End Flat Tire Logic ---

                    % --- Set Flat Tires in ForceCalculator ---
                    if ~isempty(flatTireIndices)
                        forceCalc = forceCalc.setFlatTire(flatTireIndices);
                    end

                    % --- Compute Contact Areas ---
                    % Compute per-tire loads (assuming equal distribution for simplicity)
                    perTireLoadTractor = (tractorMass * 9.81) / totalTiresTractor; % N
                    if simParams.includeTrailer
                        if isfield(simParams,'trailerBoxWeightDistributions') && ~isempty(simParams.trailerBoxWeightDistributions)
                            perTireLoadTrailer = vertcat(simParams.trailerBoxWeightDistributions{:});
                            perTireLoadTrailer = perTireLoadTrailer(:,4); % in Newtons
                            if length(perTireLoadTrailer) ~= length(trailerTirePressures)
                                logMessages{end+1} = sprintf(['Warning: trailerBoxWeightDistributions length (%d) ' ...
                                    'does not match number of trailer tires (%d). Using uniform distribution.'], ...
                                    length(perTireLoadTrailer), length(trailerTirePressures));
                                perTireLoadTrailer = (trailerMass * 9.81) / length(trailerTirePressures) * ...
                                    ones(length(trailerTirePressures),1);
                            end
                        else
                            perTireLoadTrailer = (trailerMass * 9.81) / totalTiresTrailer * ones(totalTiresTrailer,1); % N each
                        end
                    end

                    % Compute per-tire contact areas
                    % Avoid division by zero by setting contact area to zero if pressure is zero (flat tire)
                    tractorContactAreas = zeros(size(tractorTirePressures));
                    for n = 1:length(tractorTirePressures)
                        if tractorTirePressures(n) > 0
                            tractorContactAreas(n) = perTireLoadTractor / tractorTirePressures(n); % m^2
                        else
                            tractorContactAreas(n) = 0; % Flat tire
                        end
                    end

                    if simParams.includeTrailer
                        trailerContactAreas = zeros(size(trailerTirePressures));
                        for n = 1:length(trailerTirePressures)
                            if trailerTirePressures(n) > 0
                                trailerContactAreas(n) = perTireLoadTrailer(n) / trailerTirePressures(n); % m^2
                            else
                                trailerContactAreas(n) = 0; % Flat tire
                            end
                        end
                    else
                        trailerContactAreas = [];
                    end
                    %--- End of Compute Contact Areas ---

                    currentSpeed = u;

                    forceCalc.turnRadius = dynamicsUpdater.forceCalculator.turnRadius;

                    % --- Update purePursuit_PathFollower with Current State ---
                    currentState = [x, y];
                    purePursuitPathFollower = purePursuitPathFollower.updateState(currentState, theta, u);
                
                    % --- Compute Control Commands from purePursuit_PathFollower ---
                    [purePursuitPathFollower, desiredSteeringAngleDeg] = purePursuitPathFollower.computeSteering();

                    % --- Apply Steering and Acceleration Commands ---
                    % Limit steering angle and acceleration based on simulation constraints
                    limitedSteerAngleDeg = obj.limiter_LateralControl.computeSteeringAngle(desiredSteeringAngleDeg(1), currentSpeed);
                
                    % --- Steering Control ---
                    desiredSteerAngleDeg = steerAngles(i); % In degrees
                    if ~steeringEnded(i)
                        limitedSteerAngleDeg = obj.limiter_LateralControl.computeSteeringAngle(desiredSteerAngleDeg, currentSpeed);
                    end
                    steerAngleRad = deg2rad(limitedSteerAngleDeg);
                    steerAngleRad = AckermannGeometry.enforceSteeringLimits(steerAngleRad);
        
                    % --- Compute Desired Acceleration ---
                    accData = accelerationData(i);
                    if ~accelerationEnded(i)
                        desired_acceleration = accelerationData(i);
                        desired_acceleration_sim(i) = desired_acceleration;
                        logMessages{end+1} = sprintf('Step %d: Using Excel-provided acceleration: %.4f m/s^2', i, desired_acceleration);
                    else
                        % Obtain upcoming path geometry for speed planning
                        curIdx = obj.localizer.localize(dynamicsUpdater.position');
                        lookAhead = min(curIdx + purePursuitPathFollower.planningHorizon - 1, numel(purePursuitPathFollower.radiusOfCurvature));
                        upcomingRadii = purePursuitPathFollower.radiusOfCurvature(curIdx:lookAhead);
                        waypointSpacing = 1.0;
                        curveIdx = find(~isinf(upcomingRadii),1,'first');
                        if isempty(curveIdx)
                            distToCurve = Inf;
                        else
                            distToCurve = (curveIdx-1)*waypointSpacing;
                        end
                        inCurve = ~isinf(upcomingRadii);
                        baseSpeed = obj.pid_SpeedController.desiredSpeed;
                        [limitedSpeed, accelOverride] = obj.curveSpeedLimiter.limitSpeed(currentSpeed, baseSpeed, distToCurve, inCurve, dt);
                        obj.pid_SpeedController.desiredSpeed = limitedSpeed;
                        inCurve = ~isinf(dynamicsUpdater.forceCalculator.turnRadius);
                        % if inCurve
                        %     desired_acceleration_pid = 0;
                        %     obj.pid_SpeedController.controllerActive = false;
                        % else
                            desired_acceleration_pid = obj.pid_SpeedController.computeAcceleration(currentSpeed, time(i), dynamicsUpdater.forceCalculator.turnRadius, upcomingRadii);
                            obj.pid_SpeedController.controllerActive = true;
                        % end
                        distToCurve = obj.localizer.distanceToNextCurve(curIdx, upcomingRadii);
                        [desired_acceleration, predictedRotation] = obj.accController.adjust(currentSpeed, desired_acceleration_pid, distToCurve, dynamicsUpdater.forceCalculator.turnRadius, dt);
                        logMessages{end+1} = sprintf('Step %d: ACC predicted trailer rotation %.4f rad.', i, predictedRotation);
                        obj.pid_SpeedController.desiredSpeed = baseSpeed;
                        if ~isnan(accelOverride)
                            desired_acceleration = min(accelOverride,desired_acceleration);
                        end
                        desired_acceleration_sim(i) = 0;
                        logMessages{end+1} = sprintf('Step %d: Computed acceleration using pid_SpeedController: %.4f m/s^2', i, desired_acceleration);
                    end
        
                    % --- Speed Limiter Integration ---
                    if currentSpeed >= maxSpeed
                        desired_acceleration = 0; % No further acceleration
                        logMessages{end+1} = sprintf('Step %d: Max speed reached (%.2f m/s). Throttle set to 0.', i, currentSpeed);
                    end
        
                    % --- Apply Acceleration Limiter ---
                    limited_acceleration = obj.limiter_LongitudinalControl.applyLimits(desired_acceleration, currentSpeed);
                    limited_acceleration = movmean(limited_acceleration, floor(windowSize/dt));

                    [limited_acceleration, steerAngleRad] = obj.jerkController.limit(limited_acceleration, steerAngleRad, time(i));
                    [~, ~, R] = AckermannGeometry.calculateAckermannSteeringAngles(steerAngleRad, tractorWheelbase, tractorTrackWidth);
                    dynamicsUpdater.forceCalculator.turnRadius = R;
                    dynamicsUpdater.forceCalculator.steeringAngle = steerAngleRad;

                    limited_acceleration_sig(i) = limited_acceleration;
                    logMessages{end+1} = sprintf('Step %d: Limited acceleration: %.4f m/s^2', i, limited_acceleration);
                    steeringAnglesSim(i) = steerAngleRad;
                    % --- End of Acceleration Limiter ---
        
                    % --- Transmission Gear Update ---
                    [transmission, desiredGear(i)] = transmission.updateGear(currentSpeed, limited_acceleration, time(i), dt);
                    logMessages{end+1} = sprintf('Step %d: Current Gear after update: %d', i, transmission.currentGear);
                    % --- End of Transmission Gear Update ---
                    ClutchPedal(i) = clutch.engagementPercentage;
        
                    % --- Determine Desired Braking Force Based on Desired Acceleration ---
                    if limited_acceleration < 0
                        % Negative acceleration: Deceleration command
                        desired_decel = abs(limited_acceleration);
                        brakeForceInput = desired_decel * totalMassToUse; % F = m * a
                        brakeSystem = brakeSystem.setDesiredBrakingForce(brakeForceInput);
                        logMessages{end+1} = sprintf('Step %d: Desired braking force set to %.2f N.', i, brakeForceInput);
                    else
                        % Positive acceleration: No braking
                        brakeSystem = brakeSystem.setDesiredBrakingForce(0);
                        brakeForceInput = 0;
                        logMessages{end+1} = sprintf('Step %d: Desired braking force set to 0 N.', i);
                    end
                    % --- End of Desired Braking Force Determination ---
        
                    % --- Apply Brakes ---
                    brakeForceInputSig(i) = brakeForceInput;
                    brakeSystem = brakeSystem.applyBrakes(dt);
                    F_brake = brakeSystem.computeTotalBrakingForce();
                    F_brake = movmean(F_brake, floor(windowSize/dt));
                    F_brakesig(i) = F_brake;
                    logMessages{end+1} = sprintf('Step %d: Braking Force applied: %.2f N.', i, F_brake);
                    % --- End of Brake Application ---
        
                    % --- Transmission Engine Braking ---
                    % Get engine braking torque from Transmission
                    engineBrakeTorque = transmission.getEngineBrakeTorque();
                    logMessages{end+1} = sprintf('Step %d: Engine Brake Torque: %.2f Nm.', i, engineBrakeTorque);
        
                    % Convert engine braking torque to braking force
                    wheelRadius = tractorTireHeight / 2; % meters
                    F_engineBrake = engineBrakeTorque / wheelRadius; % F = Torque / Radius
                    logMessages{end+1} = sprintf('Step %d: Engine Braking Force: %.2f N.', i, F_engineBrake);
        
                    % --- Combine BrakeSystem and Engine Braking Forces ---
                    %total_F_brake = F_brake + F_engineBrake;
                    %logMessages{end+1} = sprintf('Step %d: Total Braking Force (BrakeSystem + Engine): %.2f N.', i, total_F_brake);
        
                    % --- Update ForceCalculator with Combined Braking Force ---
                    forceCalc = forceCalc.updateBrakingForce(-F_brake); % Apply total braking force oppositely
                    logMessages{end+1} = sprintf('Step %d: Braking Force updated in ForceCalculator as %.2f N.', i, -F_brake);
                    % --- End of Braking Force Update ---
        
                    % --- Determine Throttle Position ---
                    if currentSpeed >= maxSpeed || limited_acceleration <= 0
                        throttlePosition = 0; % No throttle during deceleration or at max speed
                    else
                        % Calculate maximum possible acceleration
                        [engineTorqueMax, wheelTorqueMax] = engine.getTorque(1, transmission.currentGear); % Full throttle
                        maxAvailableForce = wheelTorqueMax / wheelRadius; % F = Torque / Radius
                        maxPossibleAcceleration = maxAvailableForce / totalMassToUse; % a = F / m
        
                        % Limit desired acceleration to max possible
                        limited_acceleration = min(limited_acceleration, maxPossibleAcceleration);
                        % Calculate required traction force
                        F_required = limited_acceleration * totalMassToUse; % F = m * a
                        wheelTorqueRequired = F_required * wheelRadius; % Torque = Force * Radius
                
                        % Compute loadTorque
                        loadTorque = wheelTorqueRequired / (transmission.gearRatios(transmission.currentGear) * ...
                                           transmission.finalDriveRatio * engine.efficiency);
        
                        % Compute throttle position needed
                        throttlePosition = wheelTorqueRequired / wheelTorqueMax;
                        throttlePosition = max(0, min(1, throttlePosition)); % Clamp between 0 and 1
                    end

                    if clutch.isEngaged
                        throttle = throttle.setThrottle(throttlePosition);
                        throttle = throttle.updateThrottle(clutch.engagementPercentage, dt);
                        throttlePositionSig(i) = throttlePosition;
                        logMessages{end+1} = sprintf('Step %d: Throttle Position set to %.2f%%.', i, throttlePosition * 100);
                    else
                        throttle = throttle.setThrottle(0);
                        throttle = throttle.updateThrottle(clutch.engagementPercentage, dt);
                        throttlePositionSig(i) = 0;
                    end
        
                    % --- Get Actual Torque from Engine ---
                    [engineTorque, wheelTorque] = engine.getTorque(throttlePosition, transmission.currentGear);
                                        
                    % 2) Convert brake force to brake torque
                    %    Suppose F_brake is the total braking force on wheels (N)
                    %    Then brakeTorque = F_brake * radius
                    %    We already have wheelRadius=0.5 => so:
                    brakeTorque = F_brake * forceCalc.wheelRadius; 
                    % If wheelRadius is a scalar, that's easy. If it's per-wheel array, you might distribute brake force differently.

                    % ---------------------------------------------------------------
                    % %%% NEW LINES %%%: Integrate wheel speeds
                    % ---------------------------------------------------------------
                    % We integrate wheel speeds using net torque from engine & brake:
                    % We'll do an approximate "engineTorque = total torque / #wheels" if distributing equally. 
                    % For simplicity, pass total engineTorque, total brakeTorque and let updateWheelSpeeds
                    % distribute them equally. 
                    forceCalc = forceCalc.updateWheelSpeeds(dt, engineTorque, brakeTorque);

                    % Include engine braking torque when throttle is zero
                    if throttlePosition == 0
                        engineTorque = -transmission.engineBrakeTorque; % Negative torque for engine braking
                        wheelTorque = engineTorque * transmission.currentGear * transmission.finalDriveRatio * engine.efficiency;
                        logMessages{end+1} = sprintf('Step %d: Engine Braking Torque applied: %.2f Nm.', i, engineTorque);        
                        % Recalculate loadTorque due to engine braking
                        loadTorque = engineBrakeTorque / (transmission.gearRatios(transmission.currentGear) * ...
                                         transmission.finalDriveRatio * engine.efficiency);
                    elseif clutch.isEngaged      
                        engineTorque = transmission.engineBrakeTorque; % Negative torque for engine braking
                        % Recalculate loadTorque based on wheelTorque
                        loadTorque = (wheelTorque / (transmission.gearRatios(transmission.currentGear) * ...
                                     transmission.finalDriveRatio * engine.efficiency));
                    else
                        engineTorque = 0;
                        loadTorque = (wheelTorque / (transmission.gearRatios(transmission.currentGear) * ...
                                     transmission.finalDriveRatio * engine.efficiency));
                    end

                    % --- Update Engine RPM ---
                    engine.updateRPM(throttle.getThrottle(), loadTorque, dt, transmission.currentGear);
                    logMessages{end+1} = sprintf('Step %d: Engine Torque: %.2f Nm, Wheel Torque: %.2f Nm.', i, engineTorque, wheelTorque);
        
                    % --- **Account for Number of Drive Tires in F_traction Calculation** ---
                    % In practice only the tractor axles are powered. Counting
                    % trailer tires here would unrealistically increase the
                    % available traction with additional boxes.
                    numDriveTiresTractor = totalTiresTractor; % assume all tractor tires are driven
                    numDriveTiresTrailer = 0;                  % trailer tires free-roll

                    totalDriveTires = numDriveTiresTractor;
                    logMessages{end+1} = sprintf('Total Drive Tires: %d (Tractor only)', totalDriveTires);
                    % --- End of Drive Tires Accounting ---
        
                    % --- Update ForceCalculator with Traction Force ---
                    % Engine torque is distributed across the driven tractor
                    % wheels. Traction force is therefore the total wheel torque
                    % divided by the wheel radius.
                    F_traction = wheelTorque / wheelRadius;
                    forceCalc.updateTractionForce(F_traction);
                    logMessages{end+1} = sprintf('Step %d: Traction Force updated in ForceCalculator as %.2f N.', i, F_traction);
        
                    % --- Calculate Horsepower ---
                    engineRPM = engine.getRPM(); % Retrieve the current RPM from the engine object
                    horsePower = (engineTorque * engineRPM) / 7127; % Convert torque and RPM to horsepower
                    logMessages{end+1} = sprintf('Step %d: Horsepower: %.2f HP', i, horsePower);
        
                    % Update the force calculator's calculated forces
                    dynamicsUpdater.forceCalculator.calculatedForces.traction = [F_traction; 0; 0];
        
                    % Optionally store horsepower in a data array if you wish to log it over time
                    horsepowerSim(i) = horsePower; % This array can be saved or plotted later
        
                    %% Integration of HitchModel Forces if trailer is included
                    if simParams.includeTrailer
                        % Define tractor and trailer states for HitchModel
                        tractorState = struct('position', [x; y; 0], ...
                                               'orientation', [0; 0; theta], ...
                                               'velocity', [u; v; 0], ...
                                               'angularVelocity', [0; 0; r]);
        
                        trailerState = struct('position', [x_trailer; y_trailer; 0], ...
                                               'orientation', [0; 0; psi_trailer], ...
                                               'velocity', [u_trailer; v_trailer; 0], ...
                                               'angularVelocity', [0; 0; r_trailer]);
        
                        % Calculate Hitch Forces and Moments
                        [hitchModel, F_hitch, M_hitch] = hitchModel.calculateForces(tractorState, trailerState);
                        % [stabilityChecker, hitchModel.stiffnessCoefficients.yaw] = stabilityChecker.recommendHitchUpdates(dt);
                        dynamicsUpdater.forceCalculator.calculatedForces.hitchMomentZ = M_hitch;
                        dynamicsUpdater.forceCalculator.calculatedForces.hitch = F_hitch;
        
                        % Update trailer's orientation and yaw rate from HitchModel
                        psi_trailer = hitchModel.angularState.psi;
                        r_trailer = hitchModel.angularState.omega;
        
                        % Update trailerState for next iteration
                        trailerState.orientation(3) = psi_trailer;
                        trailerState.angularVelocity(3) = r_trailer;
        
                        % Update trailer's velocity (assuming simple model for demonstration)
                        u_trailer = u; % For simplicity, trailer's longitudinal speed equals tractor's speed
                        v_trailer = v; % Assuming negligible sideslip difference
        
                        % Apply Hitch Forces to ForceCalculator
                        dynamicsUpdater.forceCalculator.calculatedForces.hitch = F_hitch;
        
                        %% Update Trailer's State in DynamicsUpdater
                        dynamicsUpdater = dynamicsUpdater.setTrailerVelocity([u_trailer; v_trailer; 0]);
                        dynamicsUpdater = dynamicsUpdater.setTrailerAngularVelocity([0; 0; r_trailer]);
                    end
        
                    %% Update dynamics for the tractor
                    dynamicsUpdater = dynamicsUpdater.updateState();
        
                    % Retrieve updated state variables for the tractor
                    x = dynamicsUpdater.position(1);
                    y = dynamicsUpdater.position(2);
                    theta = dynamicsUpdater.orientation;
                    u = dynamicsUpdater.velocity;
                    v = dynamicsUpdater.lateralVelocity;
                    r = dynamicsUpdater.yawRate;
                    phi = dynamicsUpdater.rollAngle;   % Retrieve roll angle
                    p = dynamicsUpdater.rollRate;      % Retrieve roll rate

                    stabilityChecker.forceCalculator = dynamicsUpdater.forceCalculator;

                    %% Update StabilityChecker and Check Stability if trailer is included
                    if simParams.includeTrailer
                        % Update omega_trailer and currentHitchAngle
                        % Update StabilityChecker's dynamic parameters
                        % Extract current dynamic parameters
                        newVelocity = [u; v; 0];               % [u; v; w] in m/s
                        newAngularVelocity = [0; 0; r];        % [p; q; r] in rad/s
                        newTurnRadius = dynamicsUpdater.forceCalculator.turnRadius; % Assuming 'turnRadius' is a property of Transmission
                        newVehicleMass = totalMassToUse;        % Total mass (tractor + trailer)
        
                        % Update StabilityChecker with new dynamics
                        stabilityChecker = stabilityChecker.updateDynamics(newVelocity, newAngularVelocity, newTurnRadius, newVehicleMass, theta);
        
                        % Update Hitch Angle (delta) based on current articulation
                        % Assuming 'delta' represents the current hitch articulation angle in radians
                        % Update this value based on your simulation's articulation logic
                        stabilityChecker.currentHitchAngle = psi_trailer; % Ensure 'delta' is defined and updated appropriately

                        % stabilityChecker = stabilityChecker.updateDamping(simParams.dampingX);
                        % Check stability conditions
                        stabilityChecker = stabilityChecker.checkStability();
        
                        % Get stability flags
                        [isWiggling, isRollover, isSkidding, isJackknife] = stabilityChecker.getStabilityFlags();

                        % recommendedParams = stabilityChecker.generateRecommendedPacejkaParameters(highFidelityTireModel);
                        % highFidelityTireModel.pCy1 = recommendedParams.pCy1;
                        % highFidelityTireModel.pDy1 = recommendedParams.pDy1; 
                        % highFidelityTireModel.pKy1 = recommendedParams.pKy1;
                        % highFidelityTireModel.pEx1 = recommendedParams.pEx1;
                        % highFidelityTireModel.pEx2 = recommendedParams.pEx2;
                        % highFidelityTireModel.pEx3 = recommendedParams.pEx3;
                        % highFidelityTireModel.pEx4 = recommendedParams.pEx4;
                        % 
                        % recommendedHitchParams = stabilityChecker.generateRecommendedHitchParameters(hitchModel);
                        % hitchModel.stiffnessCoefficients.x = recommendedHitchParams.stiffnessCoefficients.x;
                        % hitchModel.stiffnessCoefficients.y = recommendedHitchParams.stiffnessCoefficients.y;
                        % hitchModel.stiffnessCoefficients.roll = recommendedHitchParams.stiffnessCoefficients.roll;
                        % hitchModel.stiffnessCoefficients.pitch = recommendedHitchParams.stiffnessCoefficients.pitch;
                        % hitchModel.stiffnessCoefficients.yaw = recommendedHitchParams.stiffnessCoefficients.yaw;
                        % hitchModel.dampingCoefficients.x = recommendedHitchParams.dampingCoefficients.x;
                        % hitchModel.dampingCoefficients.y = recommendedHitchParams.dampingCoefficients.y;
                        % hitchModel.dampingCoefficients.roll = recommendedHitchParams.dampingCoefficients.roll;
                        % hitchModel.dampingCoefficients.pitch = recommendedHitchParams.dampingCoefficients.pitch;
                        % hitchModel.dampingCoefficients.yaw = recommendedHitchParams.dampingCoefficients.yaw;
        
                        % Update globalVehicleFlags
                        globalVehicleFlags.isWiggling = globalVehicleFlags.isWiggling || isWiggling;
                        globalVehicleFlags.isRollover = globalVehicleFlags.isRollover || isRollover;
                        globalVehicleFlags.isSkidding = globalVehicleFlags.isSkidding || isSkidding;
                        globalVehicleFlags.isJackknife = globalVehicleFlags.isJackknife || isJackknife;
        
                        % Store the flags for plotting
                        isWigglingArray(i) = isWiggling;
                        isRolloverArray(i) = isRollover;
                        isSkiddingArray(i) = isSkidding;
                        isJackknifeArray(i) = isJackknife;
                    end

                    % --- Ensure that the longitudinal speed does not go below zero ---
                    if u < 0
                        u = 0;
                        dynamicsUpdater.velocity = 0; % Ensure updater's velocity is zero
                        logMessages{end+1} = sprintf('Step %d: Corrected negative longitudinal speed to zero.', i);
                    end
                    % --- End of Speed Correction ---
        
                    % obj.pid_SpeedController.updatePosition(x,y);
                    % Store positions and angles for plotting
                    tractorX(i) = x;
                    tractorY(i) = y;
                    tractorTheta(i) = theta;
                    steeringAnglesSim(i) = steerAngleRad; % Store steering angle
                    % Store Roll Dynamics
                    rollAngleArray(i) = phi;   % Store roll angle
                    rollRateArray(i) = p;      % Store roll rate
        
                    %% Align Trailer's Orientation and Position with Tractor if included
                    if simParams.includeTrailer
                        % Compute Hitch Point Position based on Tractor's Position and Orientation
                        x_hitch = x - hitch_offset_plot * cos(theta);
                        y_hitch = y - hitch_offset_plot * sin(theta);
        
                        % Set Trailer's Hitch Position to Hitch Point Position
                        trailerX(i) = x_hitch;
                        trailerY(i) = y_hitch;
        
                        % Calculate Trailer's Orientation based on Movement Direction
                        if i > 1
                            trailerTheta(i) = psi_trailer;
                        else
                            trailerTheta(i) = theta;
                        end
                        % Store orientations for each trailer box
                        % Store main trailer orientation
                        trailerThetaBoxes(1, i) = trailerTheta(i);
                        % Update and store orientations for each additional trailer box via spinner models
                        for j = 1:nSpinners
                            % Determine tractor state for this spinner
                            if j == 1
                                tractorState_sp = struct( ...
                                    'position', [x_hitch; y_hitch; 0], ...
                                    'orientation', [0; 0; psi_trailer], ...
                                    'velocity', [u_trailer; v_trailer; 0], ...
                                    'angularVelocity', [0; 0; r_trailer] ...
                                );
                            else
                                prevPsi  = trailerThetaBoxes(j, i);
                                prevOmega = spinnerModels{j-1}.angularState.omega;
                                tractorState_sp = struct( ...
                                    'position', [0; 0; 0], ...
                                    'orientation', [0; 0; prevPsi], ...
                                    'velocity', [0; 0; 0], ...
                                    'angularVelocity', [0; 0; prevOmega] ...
                                );
                            end

                            % Current trailer box state taken from spinner model's last state
                            currPsi  = spinnerModels{j}.angularState.psi;
                            currOmega = spinnerModels{j}.angularState.omega;
                            trailerState_sp = struct( ...
                                'position', [0; 0; 0], ...
                                'orientation', [0; 0; currPsi], ...
                                'velocity', [0; 0; 0], ...
                                'angularVelocity', [0; 0; currOmega] ...
                            );

                            % Update spinner model dynamics
                            [spinnerModels{j}, ~, ~] = spinnerModels{j}.calculateForces(tractorState_sp, trailerState_sp);

                            % Store updated box orientation
                            trailerThetaBoxes(j+1, i) = spinnerModels{j}.angularState.psi;
                        end
                    end
        
                    % Collect log messages
                    logMessages{end+1} = sprintf('Step %d:', i);
                    logMessages{end+1} = sprintf('  Tractor Position: (%.2f, %.2f)', x, y);
                    logMessages{end+1} = sprintf('  Tractor Theta: %.4f radians', theta);
                    logMessages{end+1} = sprintf('  Tractor Roll Angle: %.4f radians', phi);
                    if simParams.includeTrailer
                        logMessages{end+1} = sprintf('  Trailer Hitch Position: (%.2f, %.2f)', trailerX(i), trailerY(i));
                        logMessages{end+1} = sprintf('  Trailer Theta: %.4f radians', trailerTheta(i));
                    end
                    logMessages{end+1} = '----------------------------------------';

                    %% Store simulation data for Excel output
                    positionX(i) = x;
                    positionY(i) = y;
                    orientationArray(i) = theta;
                    velocityU(i) = u;
                    lateralVelocityV(i) = v;
                    yawRateR(i) = r;
                    accelerationLongitudinal(i) = dynamicsUpdater.a_long;
                    accelerationLateral(i) = dynamicsUpdater.a_lat;
                    accelerationsim(i) = norm([dynamicsUpdater.a_long, dynamicsUpdater.a_lat]);
                    % Compute jerk (derivative of acceleration)
                    if i > 1
                        jerkLongitudinal(i) = (accelerationLongitudinal(i) - accelerationLongitudinal(i-1)) / dt;
                        jerkLateral(i) = (accelerationLateral(i) - accelerationLateral(i-1)) / dt;
                    else
                        jerkLongitudinal(i) = 0;
                        jerkLateral(i) = 0;
                    end
                    % steeringAnglesSim already stored earlier
                end
                % --- Auto-export of all local variables  --------------------------
                thisFuncVars = setdiff(who, {'obj', 'output', 'fn', 'tot', 'k', 'fld', 'i', 'input'});   % all except 'obj' & 'output'
                for k = 1:numel(thisFuncVars)
                    varName = thisFuncVars{k};
                    output.(varName) = eval(varName);             % output.varName = varName;
                end
                % ------------------------------------------------------------------------
            catch ME
                try
                    [txtFilename, csvFilename] = obj.saveLogs(logMessages); % Capture filenames
                    disp(['Logs saved to ', txtFilename, ' and ', csvFilename]);
                catch logError
                    warning('Failed to save simulation logs due to: %s', logError.message);
                end
                rethrow(ME); % Re-throw the error after logging
            end
        end
        
        function speedData = closeSim(obj,in)
            try    
                % --- Auto-import of all variables within 'input' ------------
                functn = fieldnames(in);          % names
                tot = numel(functn);
                for pum = 1:tot
                    fld = functn{pum};                 % name (string)
                    eval([fld ' = in.' fld ';']);      % creates local variable
                end
                % -------------------------------------------------------------------------
                logMessages{end+1} = '--- Simulation Completed ---';
        
                % Plot Stability Flags
                if simParams.includeTrailer
                    figure('Name', 'Stability Checker Flags');
        
                    subplot(4,1,1);
                    plot(timeArray, isWigglingArray, 'r', 'LineWidth', 1.5);
                    title('Stability Flags Over Time');
                    ylabel('Wiggling');
                    ylim([-0.1 1.1]);
                    grid on;
        
                    subplot(4,1,2);
                    plot(timeArray, isRolloverArray, 'g', 'LineWidth', 1.5);
                    ylabel('Rollover');
                    ylim([-0.1 1.1]);
                    grid on;
        
                    subplot(4,1,3);
                    plot(timeArray, isSkiddingArray, 'b', 'LineWidth', 1.5);
                    ylabel('Skidding');
                    ylim([-0.1 1.1]);
                    grid on;
        
                    subplot(4,1,4);
                    plot(timeArray, isJackknifeArray, 'm', 'LineWidth', 1.5);
                    ylabel('Jackknife');
                    xlabel('Time (s)');
                    ylim([-0.1 1.1]);
                    grid on;
        
                    % Improve layout
                    sgtitle('Stability Checker Flags Over Time');
        
                    % Optionally, save the figure
                    figFilename = obj.getUniqueFilename(obj.simulationName, '.png');
                    saveas(gcf, figFilename);
                    logMessages{end+1} = sprintf('Stability flags plot saved to %s.', figFilename);
                end
        
                %% Assign Speed Data
                speedData = velocityU(:);  % Ensure it's a column vector
        
                %% Write simulation data to MATLAB .mat file (faster than Excel)
                simulationData = struct(...
                    'Time', timeArray, ...
                    'PositionX', positionX, ...
                    'PositionY', positionY, ...
                    'Orientation', orientationArray, ...
                    'VelocityU', velocityU, ...
                    'LateralVelocityV', lateralVelocityV, ...
                    'YawRateR', yawRateR, ...
                    'RollAngle', rollAngleArray, ...          % Include roll angle in output
                    'RollRate', rollRateArray, ...            % Include roll rate in output
                    'AccelerationLongitudinal', accelerationLongitudinal, ...
                    'AccelerationLateral', accelerationLateral, ...
                    'JerkLongitudinal', jerkLongitudinal, ...
                    'JerkLateral', jerkLateral, ...
                    'SteeringAngle', -steeringAnglesSim, ...
                    'GlobalVehicleFlags', globalVehicleFlags, ...
                    'Horsepower', horsepowerSim, ... % Include horsepower data
                    'CurrentGear', desiredGear, ...
                    'CommandedAcceleration', desired_acceleration_sim, ...
                    'CommandedSteerAngles', -steerAngles, ...
                    'Acceleration', accelerationsim, ...
                    'Clutch', ClutchPedal, ...
                    'Throttle', throttlePositionSig, ...
                    'SteeringWheelAngle', -steeringAnglesSim*20 ...
                );
        
                if simParams.includeTrailer
                    simulationData.TrailerX = trailerX;
                    simulationData.TrailerY = trailerY;
                    simulationData.TrailerTheta = trailerTheta;
                    % Export per-box trailer orientations for multi-box follow-the-lead
                    simulationData.trailerThetaBoxes = trailerThetaBoxes;
                end
        
                % Plot acceleration limits
                obj.limiter_LongitudinalControl.plotLimits();
                
                % Plot Gaussian filter coefficients
                %obj.limiter_LongitudinalControl.plotGaussianResponse();

                % Generate unique filename for .mat file
                matFilename = obj.getUniqueFilename(obj.simulationName, '.mat');
                save(matFilename, '-struct', 'simulationData');
        
                %% Save log data to a text and CSV file
                [txtFilename, csvFilename] = obj.saveLogs(logMessages); % Capture filenames
        
                %% Confirmation Messages
                if exist(matFilename, 'file')
                    disp(['Simulation data saved to ', matFilename]);
                else
                    warning('Failed to save simulation data.');
                end
        
                if exist(txtFilename, 'file') && exist(csvFilename, 'file')
                    disp(['Logs saved to ', txtFilename, ' and ', csvFilename]);
                else
                    warning('Failed to save simulation logs.');
                end
        
                fprintf('--- Simulation Data and Logs Saved ---\n');
                % Auto-export all local simulation variables to output struct
                vars = setdiff(who, {'obj','input','output','fn','k'});
                for idx = 1:numel(vars)
                    name = vars{idx};
                    output.(name) = eval(name);
                end
            catch ME
                % Handle any errors and ensure logs are saved
                %logMessages{end+1} = sprintf('Simulation Error: %s', ME.message);
                try
                    [txtFilename, csvFilename] = obj.saveLogs(logMessages); % Capture filenames
                    disp(['Logs saved to ', txtFilename, ' and ', csvFilename]);
                catch logError
                    warning('Failed to save simulation logs due to: %s', logError.message);
                end
                rethrow(ME); % Re-throw the error after logging
            end
        end

        %% Mirror Angle Function
        function mirroredTheta = mirrorAngle(~, theta, psi_trailer)
            % mirrorAngle Reflects the trailer angle across a specified angle

            % Convert psi_trailer to a unit vector
            v = [cos(psi_trailer); sin(psi_trailer)];

            % Define the reflection matrix across the line at angle theta
            M = [cos(2*theta), sin(2*theta);
                 sin(2*theta), -cos(2*theta)];

            % Apply the reflection matrix
            v_mirrored = M * v;

            % Convert the mirrored vector back to an angle
            mirroredTheta = atan2(v_mirrored(2), v_mirrored(1));

            % Normalize the angle to be within [-pi, pi]
            mirroredTheta = wrapToPi(mirroredTheta);
        end
    end
end

% --- Added: Helper Function to Close Waitbar Safely ---
function closeIfOpen(h)
    if isvalid(h)
        close(h);
    end
end

function [time, signal, freeFlag] = processSignalColumn(commandString, initialValue, signalName, dt)
    % processSignalColumn
    % 
    % Once a multi-tire "pressure_" command is encountered, we break out of 
    % the loop so the commanded pressure remains for the rest of the simulation.
    %
    % Accepts multi-tire commands like:
    %   pressure_(t:150-[tire:9,psi:70];[tire:2,psi:72];[tire:1,psi:7])
    % 
    % Also supports single-tire commands:
    %   pressure_(t:150;tire:9;psi:70)
    %
    % And your usual:
    %   ramp_X(duration), step_X(duration), keep_X(duration), simval_(duration)
    %
    % If `initialValue` is 1xM => M tires. The final signal is NxM.
    % 
    % The key difference: once we do a multi-tire pressure command, 
    % we do a single step and then `break` => ignore subsequent commands.

    commands = split(commandString, '|');
    commands = strtrim(commands);

    % Check if single or multi-tire
    if isscalar(initialValue)
        lastRow  = initialValue;  % 1×1
        numTires = 1;
    else
        lastRow  = initialValue;  % 1×M
        numTires = numel(initialValue);
    end

    time   = [];
    signal = [];
    freeFlag = [];

    smoothingWindow = 5;  % reduce or remove if smoothing causes overshoot

    rampPattern      = '^ramp_(\-?\d+\.?\d*)\(\s*(\d+\.?\d*)\)$';
    stepPattern      = '^step_(\-?\d+\.?\d*)\(\s*(\d+\.?\d*)\)$';
    keepPattern      = '^keep_(\-?\d+\.?\d*)\(\s*(\d+\.?\d*)\)$';
    simvalPattern    = '^simval_\((\d+\.?\d*)\)$';
    
    % Single-tire pressure pattern (optional):
    singlePressurePattern = '^pressure_\(\s*t\s*:\s*(\d+\.?\d*)\s*;\s*tire\s*:\s*(\d+)\s*;\s*psi\s*:\s*(\d+\.?\d*)\s*\)$';

    % Multi-tire pattern, e.g.: pressure_(t:150-[tire:9,psi:70];[tire:2,psi:72])
    multiPressurePattern  = '^pressure_\(\s*t\s*:\s*(\d+\.?\d*)\s*-\s*(.*)\)$';

    for iCmd = 1:numel(commands)
        cmd = commands{iCmd};

        %% 1) RAMP
        if ~isempty(regexp(cmd, rampPattern, 'once'))
            tokens = regexp(cmd, rampPattern, 'tokens');
            targetVal = str2double(tokens{1}{1});
            duration  = str2double(tokens{1}{2});
            numSteps  = ceil(duration / dt);
            tLocal    = (0:numSteps-1)' * dt;

            % Build ramp
            if numTires == 1
                vals = linspace(lastRow, targetVal, numSteps)';
                vals = movmean(vals, smoothingWindow); 
                vals(vals<0) = 0;  % clamp negative
            else
                vals = zeros(numSteps, numTires);
                for c=1:numTires
                    partial = linspace(lastRow(c), targetVal, numSteps);
                    partial = movmean(partial, smoothingWindow);
                    partial(partial<0) = 0;  % clamp negative
                    vals(:, c) = partial;
                end
            end

            if ~isempty(time)
                tLocal = tLocal + time(end);
            end
            time   = [time; tLocal];
            signal = [signal; vals];
            freeFlag = [freeFlag; false(numSteps,1)];

            if numTires == 1
                lastRow = max(targetVal, 0);
            else
                lastRow(:) = max(targetVal, 0);
            end

        %% 2) STEP
        elseif ~isempty(regexp(cmd, stepPattern, 'once'))
            tokens = regexp(cmd, stepPattern, 'tokens');
            targetVal = str2double(tokens{1}{1});
            duration  = str2double(tokens{1}{2});
            numSteps  = ceil(duration / dt);
            tLocal    = (0:numSteps-1)' * dt;

            stepMat = repmat(lastRow, numSteps,1);
            targetVal = max(targetVal, 0); % clamp
            stepMat(:) = targetVal;

            if ~isempty(time)
                tLocal = tLocal + time(end);
            end
            time   = [time; tLocal];
            signal = [signal; stepMat];
            freeFlag= [freeFlag; false(numSteps,1)];

            if numTires==1
                lastRow = targetVal;
            else
                lastRow(:) = targetVal;
            end

        %% 3) KEEP
        elseif ~isempty(regexp(cmd, keepPattern, 'once'))
            tokens = regexp(cmd, keepPattern, 'tokens');
            targetVal = str2double(tokens{1}{1});
            duration  = str2double(tokens{1}{2});
            numSteps  = ceil(duration / dt);
            tLocal    = (0:numSteps-1)'*dt;

            targetVal = max(targetVal, 0);
            keepMat = repmat(lastRow, numSteps,1);
            keepMat(:)= targetVal;

            if ~isempty(time)
                tLocal = tLocal + time(end);
            end
            time   = [time; tLocal];
            signal = [signal; keepMat];
            freeFlag= [freeFlag; false(numSteps,1)];

            if numTires==1
                lastRow= targetVal;
            else
                lastRow(:)= targetVal;
            end

        %% 4) SIMVAL
        elseif ~isempty(regexp(cmd, simvalPattern, 'once'))
            tokens   = regexp(cmd, simvalPattern, 'tokens');
            duration = str2double(tokens{1}{1});
            numSteps = ceil(duration / dt);
            tLocal   = (0:numSteps-1)'*dt;

            simMat = repmat(lastRow, numSteps,1);
            simMat(simMat<0) = 0;  % clamp
            if ~isempty(time)
                tLocal = tLocal + time(end);
            end
            time   = [time; tLocal];
            signal = [signal; simMat];
            freeFlag= [freeFlag; true(numSteps,1)];

        %% 5) SINGLE-TIRE PRESSURE
        elseif ~isempty(regexp(cmd, singlePressurePattern, 'once'))
            % e.g. pressure_(t:150;tire:9;psi:70)
            tokens = regexp(cmd, singlePressurePattern, 'tokens');
            tTime   = str2double(tokens{1}{1});
            tireIdx = str2double(tokens{1}{2});
            psiVal  = str2double(tokens{1}{3});
            psiVal  = max(psiVal, 0); % clamp

            % 1) hold lastRow for tTime
            numSteps = ceil(tTime / dt);
            tLocal   = (0:numSteps-1)'*dt;
            holdMat  = repmat(lastRow, numSteps,1);
            holdMat(holdMat<0) = 0;

            if ~isempty(time)
                tLocal = tLocal + time(end);
            end
            time   = [time; tLocal];
            signal = [signal; holdMat];
            freeFlag= [freeFlag; false(numSteps,1)];

            % 2) step that one tire over next dt
            stepDuration= dt;
            tLocal2 = (0:1)'*stepDuration + time(end);

            stepMat = repmat(lastRow, 2,1);
            if tireIdx<= numTires
                stepMat(2, tireIdx) = psiVal;
            else
                warning('Tire index %d out of range (numTires=%d).', tireIdx, numTires);
            end
            stepMat(stepMat<0)=0;

            time   = [time; tLocal2];
            signal = [signal; stepMat];
            freeFlag= [freeFlag; false(2,1)];

            lastRow = stepMat(end,:);

            fprintf('[%s] Single-Tire Pressure => T=%.1f => Tire#%d => %.1f psi\n',...
                    signalName, tTime, tireIdx, psiVal);

        %% 6) MULTI-TIRE PRESSURE
        elseif ~isempty(regexp(cmd, multiPressurePattern, 'once'))
            % e.g. pressure_(t:150-[tire:9,psi:70];[tire:2,psi:72])
            tokens = regexp(cmd, multiPressurePattern, 'tokens');
            tTime   = str2double(tokens{1}{1});
            chunkStr= strtrim(tokens{1}{2});  % e.g. "[tire:9,psi:70];[tire:2,psi:72]"

            % 1) hold lastRow for tTime
            numSteps = ceil(tTime / dt);
            tLocal   = (0:numSteps-1)'*dt;
            holdMat  = repmat(lastRow, numSteps,1);
            holdMat(holdMat<0)=0;

            if ~isempty(time)
                tLocal = tLocal + time(end);
            end
            time   = [time; tLocal];
            signal = [signal; holdMat];
            freeFlag= [freeFlag; false(numSteps,1)];

            % 2) parse sub-chunks => step them
            subChunks = split(chunkStr, ';');
            subChunks = strtrim(subChunks);

            stepDuration = dt;
            tLocal2 = (0:1)'*stepDuration + time(end);
            stepMat = repmat(lastRow, 2,1);

            for sc = 1:numel(subChunks)
                oneChunk = subChunks{sc}; 
                subTok   = regexp(oneChunk, '^\[tire\s*:\s*(\d+)\s*,\s*psi\s*:\s*(\d+\.?\d*)\]$', 'tokens','once');
                if ~isempty(subTok)
                    cTire = str2double(subTok{1});
                    cPsi  = max(str2double(subTok{2}), 0); % clamp
                    if cTire<= numTires
                        stepMat(2, cTire) = cPsi;
                    else
                        warning('Tire index %d out of range (numTires=%d).', cTire, numTires);
                    end
                else
                    warning('Bad sub-chunk format: "%s". Expected "[tire:X,psi:Y]".', oneChunk);
                end
            end
            stepMat(stepMat<0)=0;

            time   = [time; tLocal2];
            signal = [signal; stepMat];
            freeFlag= [freeFlag; false(2,1)];

            lastRow = stepMat(end,:);
            fprintf('[%s] Multi-Tire Pressure => t=%.1f => %d sub-chunks.\n',...
                    signalName, tTime, numel(subChunks));

            % *** KEY CHANGE *** 
            % Stop parsing more commands => keep these pressures for the rest
            break;

        else
            error('Invalid command "%s" in %s. Commands: ramp_, step_, keep_, simval_, pressure_.', ...
                  cmd, signalName);
        end
    end

    %% If no commands => single sample
    if isempty(time)
        time = 0;
        if isscalar(initialValue)
            signal = initialValue; % Nx1
        else
            signal = initialValue; % NxM
        end
        freeFlag = false;
    else
        [time, idx] = unique(time, 'stable');
        signal = signal(idx,:);
        freeFlag = freeFlag(idx);
    end
end

%% Nested Helper Function to Process Individual Signal Columns
function [time, steerAngles, accelerationData, tirePressureData, ...
          steeringEnded, accelerationEnded, tirePressureEnded] = ...
         processSignalData(steeringCommands, accelerationCommands, ...
                           tirePressureCommands, dt, defaultTirePressures)
    % processSignalData extends your logic to incorporate an explicit
    % "defaultTirePressures" input for initial multi-tire pressures.
    %
    % Inputs:
    %   steeringCommands      - e.g., 'ramp_-30(1)|keep_-30(0.8)'
    %   accelerationCommands  - e.g., 'simval_(10)'
    %   tirePressureCommands  - e.g., 'pressure_(t:150;tire:9;psi:70)'
    %   dt                    - time step (seconds)
    %   defaultTirePressures  - 1×M row of initial pressures for M tires, e.g. [100 100 100 100]
    %
    % Outputs:
    %   time               = Nx1 global time vector
    %   steerAngles        = Nx1
    %   accelerationData   = Nx1
    %   tirePressureData   = NxM
    %   steeringEnded      = Nx1
    %   accelerationEnded  = Nx1
    %   tirePressureEnded  = Nx1

    %% 1) Steering, Acceleration and Tire Pressure Parsing in Parallel
    initialSteerAngle = 0;  % single scalar
    initialAcceleration = 0;

    pool = gcp('nocreate');
    if isempty(pool)
        pool = backgroundPool;
    end

    futures = parallel.FevalFuture.empty(3,0);

    if ~isempty(steeringCommands)
        futures(1) = parfeval(pool, @processSignalColumn, 3, ...
            steeringCommands, initialSteerAngle, 'SteeringAngle', dt);
    end

    if ~isempty(accelerationCommands)
        futures(2) = parfeval(pool, @processSignalColumn, 3, ...
            accelerationCommands, initialAcceleration, 'Acceleration', dt);
    end

    if ~isempty(tirePressureCommands)
        futures(3) = parfeval(pool, @processSignalColumn, 3, ...
            tirePressureCommands, defaultTirePressures, 'TirePressure', dt);
    end

    % Gather results (fetch in the order futures were created)
    idx = 1;
    if ~isempty(steeringCommands)
        [timeSteer, sVal, freeSteerFlag] = fetchOutputs(futures(idx));
        endTimeSteer = timeSteer(end);
        idx = idx + 1;
    else
        timeSteer = 0;
        sVal = initialSteerAngle;
        freeSteerFlag = false;
        endTimeSteer = 0;
    end

    if ~isempty(accelerationCommands)
        [timeAccel, aVal, freeAccelFlag] = fetchOutputs(futures(idx));
        endTimeAccel = timeAccel(end);
        idx = idx + 1;
    else
        timeAccel = 0;
        aVal = initialAcceleration;
        freeAccelFlag = false;
        endTimeAccel = 0;
    end

    if ~isempty(tirePressureCommands)
        [timeTire, pVal, freePressFlag] = fetchOutputs(futures(idx));
        endTimeTire = timeTire(end);
    else
        timeTire = 0;
        pVal = defaultTirePressures;  % 1×M
        freePressFlag = false;
        endTimeTire = 0;
    end

    %% 4) Build the global time vector
    globalEndTime = max([endTimeSteer, endTimeAccel, endTimeTire]);
    numSteps = ceil(globalEndTime / dt) + 1;
    time = (0:dt:(numSteps-1)*dt)';

    %% 5) Interpolate the 1D signals: steering & acceleration
    if numel(timeSteer)>1
        steerAngles = interp1(timeSteer, sVal, time, 'linear','extrap');
        freeSteerFlagInterp = interp1(timeSteer, double(freeSteerFlag), time, 'nearest','extrap');
    else
        steerAngles = repmat(sVal, size(time));
        freeSteerFlagInterp = repmat(double(freeSteerFlag), size(time));
    end

    if numel(timeAccel)>1
        accelerationData = interp1(timeAccel, aVal, time, 'linear','extrap');
        freeAccelFlagInterp = interp1(timeAccel, double(freeAccelFlag), time, 'nearest','extrap');
    else
        accelerationData = repmat(aVal, size(time));
        freeAccelFlagInterp = repmat(double(freeAccelFlag), size(time));
    end

    %% 6) Interpolate the multi-tire pressure matrix pVal => NxM
    if numel(timeTire)>1
        [oldRows, oldCols] = size(pVal);  % e.g. (NtSteps × M)
        newRows = numel(time);
        tirePressureData = zeros(newRows, oldCols);

        for c=1:oldCols
            tirePressureData(:,c) = interp1(timeTire, pVal(:,c), time, 'linear','extrap');
        end

        freePressFlagInterp = interp1(timeTire, double(freePressFlag), time, 'nearest','extrap');
    else
        % no commands => just replicate pVal across all rows
        [~, oldCols] = size(pVal);
        tirePressureData = repmat(pVal, numSteps,1);
        freePressFlagInterp = repmat(double(freePressFlag), numSteps,1);
    end

    %% 7) Convert to logical
    freeSteerFlagInterp  = logical(freeSteerFlagInterp);
    freeAccelFlagInterp  = logical(freeAccelFlagInterp);
    freePressFlagInterp  = logical(freePressFlagInterp);

    %% 8) Ended flags
    steeringEnded     = (time >= endTimeSteer) | freeSteerFlagInterp;
    accelerationEnded = (time >= endTimeAccel) | freeAccelFlagInterp;
    tirePressureEnded = (time >= endTimeTire)  | freePressFlagInterp;

    %% 9) Force column vectors for 1D signals
    steerAngles      = steerAngles(:);
    accelerationData = accelerationData(:);
    steeringEnded    = steeringEnded(:);
    accelerationEnded= accelerationEnded(:);
    tirePressureEnded= tirePressureEnded(:);
    % tirePressureData stays NxM
end

%% normalizeTrailerLoadDistribution
function ld = normalizeTrailerLoadDistribution(ld, targetRows)
    % normalizeTrailerLoadDistribution Resizes a trailer load matrix to the
    % desired number of rows while conserving total load.

    currentRows = size(ld,1);
    if currentRows == targetRows
        return;
    elseif currentRows < targetRows
        reps = ceil(targetRows / currentRows);
        ld = repmat(ld, reps, 1);
        ld = ld(1:targetRows, :);
    else
        idxBounds = round(linspace(0, currentRows, targetRows+1));
        newLd = zeros(targetRows, size(ld,2));
        for k = 1:targetRows
            seg = ld(idxBounds(k)+1:idxBounds(k+1), :);
            newLd(k,1:3) = mean(seg(:,1:3), 1);
            newLd(k,4) = sum(seg(:,4));
            if size(ld,2) >= 5
                newLd(k,5) = sum(seg(:,5));
            end
        end
        ld = newLd;
    end
end

%% Helper to fix statuses after interpolation
% If you do an interpolation of a 0...0,1 series, you might get 
% partial values in between. We want strictly 0 or 1, so let's
% clamp everything except the final time sample to 0, and final sample to 1.
function statusOut = zeroUntilLastSample(statusIn)
    % zeroUntilLastSample Clamps all values to zero except the last sample -> 1
    %
    % Example:
    %   in:  [0 0 1 1 1]
    %   out: [0 0 0 0 1]
    %
    % Sometimes used if we only want a "free" signal at the final step.

    statusOut = zeros(size(statusIn));
    if ~isempty(statusIn)
        statusOut(end) = 1;
    end
end

function y = smoothNonlinearTransform(x, a)
% smoothNonlinearTransform Apply a smooth nonlinear transformation to a signal
%
% Syntax:
%   y = smoothNonlinearTransform(x, a)
%
% Inputs:
%   x - Input signal (vector or matrix)
%   a - Nonlinearity coefficient (controls the amount of curvature)
%       Typical values: small positive or negative numbers (e.g., 0.1)
%
% Outputs:
%   y - Transformed signal
%
% Example:
%   t = linspace(-10, 10, 1000);
%   y = smoothNonlinearTransform(t, 0.1);
%   plot(t, y);
%   xlabel('Input');
%   ylabel('Transformed Output');
%   title('Polynomial-Based Smooth Nonlinear Transformation');

    % Input validation
    if nargin < 2
        error('smoothNonlinearTransform requires two input arguments: x and a');
    end
    
    % Apply the transformation
    y = x + a * x.^3;
end