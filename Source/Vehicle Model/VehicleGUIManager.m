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
% * @file VehicleGUIManager.m
% * @brief Manages the graphical user interface (GUI) for the Vehicle Simulation.
% */
% * @author Miguel Marina <karel.capek.robotics@gmail.com>
classdef VehicleGUIManager < handle
    % VehicleGUIManager Manages the graphical user interface (GUI) for the Vehicle Simulation.
    %
    % This class handles the creation and management of all UI components 
    % related to configuring the vehicle and trailer simulation. This includes setting up 
    % configuration tabs, input fields, dropdowns, and command input boxes.

    properties (Access = public)
        mapCommandsBox
        resolutionBox
        generateWaypointsButton
        configTabGroup
        basicConfigTab
        advancedConfigTab
        controlLimitsTab       % Tab for Control Limits
        pidControllerTab       % Tab for PID Controller
        tiresConfigTab         % Tab for Tires Configuration
        vehicleParamsTab
        trailerParamsTab
        stiffnessDampingTab    % Tab for Stiffness & Damping
        xCoefficientsTab       % Tab for X Coefficients
        yCoefficientsTab       % Tab for Y Coefficients
        aerodynamicsTab        % Tab for Aerodynamics
        roadConditionsTab      % Tab for Road Conditions
        pressureMatricesTab    % Tab for Pressure Matrices
        stabilityCheckerConfigTab
        transmissionConfigTab  % Tab for Transmission Configuration
        gearRatiosTab          % *** Gear Ratios Tab ***
        suspensionModelTab     % Tab for Suspension Model
        engineConfigTab        % Tab for Engine Configuration
        brakeConfigTab         % Tab for Brake Configuration
        clutchConfigTab        % Tab for Clutch Configuration
        commandsTab            % New tab for driving commands

        % *** New Command Input Boxes ***
        commandPanel
        steeringCommandsBox
        accelerationCommandsBox
        % *** End of New Command Input Boxes ***

        % Basic Configuration Fields
        tractorMassField
        trailerBoxWeightFields    % Cell array of weight fields per trailer box
        includeTrailerCheckbox   % Checkbox to include/remove trailer
        enableLoggingCheckbox    % Checkbox to enable/disable log messages
        velocityField
        vehicleTypeDropdown      % *** New Vehicle Type Dropdown ***

        % Advanced Configuration Fields
        I_trailerMultiplierField
        maxDeltaField
        dtMultiplierField
        windowSizeField

        % Vehicle Parameters (Tractor) Fields
        tractorLengthField
        tractorWidthField
        tractorHeightField
        tractorCoGHeightField
        tractorWheelbaseField
        tractorTrackWidthField
        tractorNumAxlesDropdown   % Dropdown for Number of Axles
        tractorAxleSpacingField
        numTiresPerAxleTrailerDropDown

        trailerNumBoxesField       % Number of trailer boxes
        trailerAxlesPerBoxField    % Axles per trailer box
        trailerBoxSpacingField     % Distance between trailer boxes (m)

        % New Tractor Tires per Axle Dropdown
        numTiresPerAxleTractorDropDown

        % Trailer Parameters Fields
        trailerLengthField
        trailerWidthField
        trailerHeightField
        trailerCoGHeightField
        trailerWheelbaseField
        trailerTrackWidthField
        trailerAxleSpacingField
        trailerHitchDistanceField
        tractorHitchDistanceField

        % Control Limits Tab Fields
        % Steering Controller Fields
        maxSteeringAngleAtZeroSpeedField
        minSteeringAngleAtMaxSpeedField
        maxSteeringSpeedField

        % New Acceleration and Deceleration Curve Fields
        accelCurveFilePathField    % Path to Acceleration Curve File
        decelCurveFilePathField    % Path to Deceleration Curve File
        decelerationCurve
        accelerationCurve
        accelCurveBrowseButton
        decelCurveBrowseButton
        maxSpeedForAccelLimitingField

        % Acceleration Limiter Fields
        % maxAccelAtZeroSpeedField
        minAccelAtMaxSpeedField
        % maxDecelAtZeroSpeedField
        minDecelAtMaxSpeedField
        % maxSpeedForAccelLimitingField

        % Maximum Speed Field
        maxSpeedField

        % PID Controller Fields
        KpField
        KiField
        KdField
        lambda1AccelField
        lambda2AccelField
        lambda1JerkField
        lambda2JerkField
        lambda1VelField
        lambda2VelField
        enableSpeedControllerCheckbox

        % Tires Configuration Fields
        % Tractor Tires
        tractorTireHeightField
        tractorTireWidthField

        % Trailer Tires
        trailerTireHeightField
        trailerTireWidthField

        % Stiffness Fields
        stiffnessXField
        stiffnessYField
        stiffnessZField
        stiffnessRollField
        stiffnessPitchField
        stiffnessYawField

        % Damping Fields
        dampingXField
        dampingYField
        dampingZField
        dampingRollField
        dampingPitchField
        dampingYawField

        % X Coefficients Fields
        pCx1Field
        pDx1Field
        pDx2Field
        pEx1Field
        pEx2Field
        pEx3Field
        pEx4Field
        pKx1Field
        pKx2Field
        pKx3Field

        % Y Coefficients Fields
        pCy1Field
        pDy1Field
        pDy2Field
        pEy1Field
        pEy2Field
        pEy3Field
        pEy4Field
        pKy1Field
        pKy2Field
        pKy3Field

        % Aerodynamics Fields
        airDensityField
        dragCoeffField
        windSpeedField         
        windAngleDegField     
        windAngleRad            

        % Road Conditions Fields
        slopeAngleField
        roadFrictionCoefficientField
        roadSurfaceTypeDropdown
        roadRoughnessField

        % Pressure Matrices Properties
        pressureMatrixPanels      % Struct to hold panels for each tire count
        pressureMatrices          % Struct to hold pressure matrices

        % Label to display total number of tires
        totalTiresLabel

        % Suspension Model Fields
        K_springField
        C_dampingField
        restLengthField

        % Stability Checker Configurations Fields
        rollThresholdField
        hitchInstabilityThresholdField
        rollStiffnessField
        rollDampingField

        % Engine Configuration Fields
        maxEngineTorqueField       % Maximum Engine Torque (Nm)
        maxPowerField              % Maximum Power (W)
        idleRPMField               % Idle RPM
        redlineRPMField            % Redline RPM
        engineBrakeTorqueField     % Engine Brake Torque (Nm)
        fuelConsumptionRateField   % Fuel Consumption Rate (kg/s)

        % Transmission Configuration Fields (Without Gear Ratios)
        maxGearField               % Maximum Gear Number
        finalDriveRatioField       % Final Drive Ratio
        shiftUpSpeedField          % Shift Up Speed Thresholds
        shiftDownSpeedField        % Shift Down Speed Thresholds
        shiftDelayField            % Shift Delay Time (s)

        % *** Gear Ratios Configuration Fields ***
        gearRatiosTable            % UITable for Gear Ratios
        addGearButton              % Button to add a new gear
        removeGearButton           % Button to remove selected gear
        engineGearMappingTable     % UITable for Engine Gear Mapping
        % *** End of Gear Ratios Configuration Fields ***

        % *** New Brake Configuration Fields ***
        brakingForceField           % Braking Force (N)
        maxBrakingForceField        % *** New: Maximum Braking Force (N) ***
        brakeEfficiencyField        % Brake Efficiency (%)
        brakeBiasField              % Brake Bias (Front/Rear %)
        brakeTypeDropdown
        % *** End of Brake Configuration Fields ***

        % *** New Engine Torque File Selection Fields ***
        torqueFileButton            % Button to select the torque Excel file
        torqueFileNameField         % Text box to display the selected file name
        torqueFilePath              % Variable to store the full path of the selected file
        % *** End of New Engine Torque File Selection Fields ***

        maxClutchTorqueField
        engagementSpeedField
        disengagementSpeedField

        % *** New Path Follower Tab ***
        pathFollowerTab
        waypointsTable
        addWaypointButton
        removeWaypointButton
        defaultWaypoints
        % *** End of Path Follower Tab Addition ***

        % --- Steering Curve File Properties ---
        steeringCurveFilePathField    % Path to Steering Curve Excel File
        steeringCurveBrowseButton
        steeringCurveData             % Data loaded from Steering Curve Excel File
        % --- End of Steering Curve File Properties ---

        tirePressureCommandsBox

        figureHandle % Handle to the main figure

        % *** New Property for Gear Ratios Data ***
        gearRatiosData   % MATLAB table to store Gear Number and Ratio
        spinnerTabs      % Cell array of uitab objects for spinner configurations
        spinnerConfig    % Struct array holding handles for spinner stiffness & damping fields
        vehicleModel     % Reference to the associated VehicleModel
        % *** End of New Property ***
    end

    methods (Access = public)
        % Constructor
        function obj = VehicleGUIManager(parent, vehicleModel)
            if nargin < 2
                vehicleModel = [];
            end
            obj.vehicleModel = vehicleModel;
            % Store the actual figure handle even if a child component is passed
            obj.figureHandle = ancestor(parent, 'figure');
            obj.initializePressureMatrices();  % Initialize pressure matrices first
            obj.initializeGearRatiosData();    % Initialize Gear Ratios Data
            obj.initializeDefaultWaypoints();
            obj.createGUI(parent);
            % Create default trailer weight fields based on initial number of boxes
            if isprop(obj, 'trailerNumBoxesField')
                obj.createTrailerWeightFields(obj.trailerNumBoxesField.Value);
            end
        end

        % Initialize default waypoints for Path Follower
        function initializeDefaultWaypoints(obj)
            % Define waypoints for a smoother oval path
            obj.defaultWaypoints = [
                100, 200        
                105.128, 200    
                110.256, 200    
                115.385, 200    
                120.513, 200    
                125.641, 200    
                130.769, 200    
                135.897, 200    
                141.026, 200    
                146.154, 200    
                151.282, 200    
                156.41, 200     
                161.538, 200    
                166.667, 200    
                171.795, 200    
                176.923, 200    
                182.051, 200    
                187.179, 200    
                192.308, 200    
                197.436, 200    
                202.564, 200    
                207.692, 200    
                212.821, 200    
                217.949, 200    
                223.077, 200    
                228.205, 200    
                233.333, 200    
                238.462, 200    
                243.59, 200     
                248.718, 200    
                253.846, 200    
                258.974, 200    
                264.103, 200    
                269.231, 200    
                274.359, 200    
                279.487, 200    
                284.615, 200    
                289.744, 200    
                294.872, 200    
                300, 200        
                300, 199        
                305.331, 199.279
                310.603, 200.114
                315.76, 201.496 
                320.744, 203.409
                325.5, 205.833  
                329.977, 208.74 
                334.126, 212.1  
                337.9, 215.874  
                341.26, 220.023 
                344.167, 224.5  
                346.591, 229.256
                348.504, 234.24 
                349.886, 239.397
                350.721, 244.669
                351, 250        
                350.721, 255.331
                349.886, 260.603
                348.504, 265.76 
                346.591, 270.744
                344.167, 275.5  
                341.26, 279.977 
                337.9, 284.126  
                334.126, 287.9  
                329.977, 291.26 
                325.5, 294.167  
                320.744, 296.591
                315.76, 298.504 
                310.603, 299.886
                305.331, 300.721
                300, 301        
                300, 300        
                294.872, 300    
                289.744, 300    
                284.615, 300    
                279.487, 300    
                274.359, 300    
                269.231, 300    
                264.103, 300    
                258.974, 300    
                253.846, 300    
                248.718, 300    
                243.59, 300     
                238.462, 300    
                233.333, 300    
                228.205, 300    
                223.077, 300    
                217.949, 300    
                212.821, 300    
                207.692, 300    
                202.564, 300    
                197.436, 300    
                192.308, 300    
                187.179, 300    
                182.051, 300    
                176.923, 300    
                171.795, 300    
                166.667, 300    
                161.538, 300    
                156.41, 300     
                151.282, 300    
                146.154, 300    
                141.026, 300    
                135.897, 300    
                130.769, 300    
                125.641, 300    
                120.513, 300    
                115.385, 300    
                110.256, 300    
                105.128, 300    
                100, 300        
                100, 301        
                94.669, 300.721 
                89.3965, 299.886
                84.2401, 298.504
                79.2564, 296.591
                74.5, 294.167   
                70.023, 291.26  
                65.8743, 287.9  
                62.0996, 284.126
                58.7401, 279.977
                55.8327, 275.5  
                53.4092, 270.744
                51.4961, 265.76 
                50.1145, 260.603
                49.2794, 255.331
                49, 250         
                49.2794, 244.669
                50.1145, 239.397
                51.4961, 234.24 
                53.4092, 229.256
                55.8327, 224.5  
                58.7401, 220.023
                62.0996, 215.874
                65.8743, 212.1  
                70.023, 208.74  
                74.5, 205.833   
                79.2564, 203.409
                84.2401, 201.496
                89.3965, 200.114
                94.669, 199.279 
                100, 199                                         
            ];
        end

        % Initializes predefined pressure matrices using a loop
        function initializePressureMatrices(obj)
            % Initialize the structure to hold pressure matrices
            obj.pressureMatrices = struct();

            % Default pressure matrices for even tire counts between 4 and 20
            for nTires = 4:2:20
                tag = sprintf('Tires%d', nTires);
                obj.pressureMatrices.(tag) = obj.generatePressureMatrix(nTires);
            end
        end

        % Generates a pressure matrix for the given number of tires
        function pressures = generatePressureMatrix(obj, nTires)
            basePressure = 800000; % Pa
            pressures = repmat(basePressure, nTires, 1);
        end

        % Initializes the Gear Ratios Data as a MATLAB table
        function initializeGearRatiosData(obj)
            % Define initial gear numbers and ratios
            initialGears = (1:18)';
            initialRatios = [
                14.40;
                12.53;
                10.91;
                9.51;
                8.34;
                7.31;
                6.39;
                5.57;
                4.83;
                4.17;
                3.59;
                3.08;
                2.62;
                2.21;
                1.86;
                1.55;
                1.28;
                1.07
            ];

            % Create the table
            obj.gearRatiosData = table(initialGears, initialRatios, ...
                'VariableNames', {'Gear', 'Ratio'});
        end

        % Creates all GUI components and layouts.
        function createGUI(obj, parent)
            % Initialize configuration tab group
            obj.configTabGroup = uitabgroup(parent, 'Position', [10, 150, 480, 470]);

            % Create existing tabs
            obj.basicConfigTab = uitab(obj.configTabGroup, 'Title', 'Basic Configuration');
            obj.advancedConfigTab = uitab(obj.configTabGroup, 'Title', 'Advanced Configuration');
            obj.controlLimitsTab = uitab(obj.configTabGroup, 'Title', 'Control Limits');
            obj.pidControllerTab = uitab(obj.configTabGroup, 'Title', 'PID Controller');
            obj.vehicleParamsTab = uitab(obj.configTabGroup, 'Title', 'Tractor Parameters');
            obj.trailerParamsTab = uitab(obj.configTabGroup, 'Title', 'Trailer Parameters');
            obj.tiresConfigTab = uitab(obj.configTabGroup, 'Title', 'Tires Configuration');
            obj.stiffnessDampingTab = uitab(obj.configTabGroup, 'Title', 'Hitch Stiffness & Damping');
            obj.xCoefficientsTab = uitab(obj.configTabGroup, 'Title', 'Tire Pacejka 96 Longitudinal Coefficients');
            obj.yCoefficientsTab = uitab(obj.configTabGroup, 'Title', 'Tire Pacejka 96 Lateral Coefficients');
            obj.aerodynamicsTab = uitab(obj.configTabGroup, 'Title', 'Aerodynamics');
            obj.roadConditionsTab = uitab(obj.configTabGroup, 'Title', 'Road Conditions');  % Existing Tab
            obj.pressureMatricesTab = uitab(obj.configTabGroup, 'Title', 'Pressure Matrices');  % Existing Tab
            obj.stabilityCheckerConfigTab = uitab(obj.configTabGroup, 'Title', 'Stability Checker Configurations');
            obj.clutchConfigTab = uitab(obj.configTabGroup, 'Title', 'Clutch Configuration');

            % *** Create Transmission Configuration Tab ***
            obj.transmissionConfigTab = uitab(obj.configTabGroup, 'Title', 'Transmission Configuration');

            % *** Create Gear Ratios Tab ***
            obj.gearRatiosTab = uitab(obj.configTabGroup, 'Title', 'Gear Ratios');

            obj.suspensionModelTab = uitab(obj.configTabGroup, 'Title', 'Suspension Model');
            obj.engineConfigTab = uitab(obj.configTabGroup, 'Title', 'Engine Configuration');

            % *** Create Brake Configuration Tab ***
            obj.brakeConfigTab = uitab(obj.configTabGroup, 'Title', 'Brake Configuration');

            % *** Create Path Follower Tab ***
            obj.pathFollowerTab = uitab(obj.configTabGroup, 'Title', 'Path Follower');
            obj.createPathFollowerTab(); % Create the Path Follower UI components

            % Commands Tab for steering, acceleration and tire pressure inputs
            obj.commandsTab = uitab(obj.configTabGroup, 'Title', 'Commands');
            obj.createCommandsTab();

            %% Basic Configuration Panel
            % Tractor Mass
            uilabel(obj.basicConfigTab, 'Position', [10, 400, 150, 20], 'Text', 'Tractor Mass (kg):');
            obj.tractorMassField = uieditfield(obj.basicConfigTab, 'numeric', ...
                'Position', [170, 400, 100, 20], 'Value', 9070, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Checkbox to include/remove the trailer
            obj.includeTrailerCheckbox = uicheckbox(obj.basicConfigTab, 'Position', [10, 320, 260, 20], ...
                'Text', 'Include Trailer', 'Value', true, ...
                'ValueChangedFcn', @(src, event)obj.includeTrailerChanged(src, event));
            % Checkbox to enable/disable internal log messages
            obj.enableLoggingCheckbox = uicheckbox(obj.basicConfigTab, 'Position', [10, 300, 260, 20], ...
                'Text', 'Enable Logging', 'Value', true, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Initial Velocity
            uilabel(obj.basicConfigTab, 'Position', [10, 240, 150, 20], 'Text', 'Initial Velocity (m/s):');
            obj.velocityField = uieditfield(obj.basicConfigTab, 'numeric', ...
                'Position', [170, 240, 100, 20], 'Value', 10, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            %% Advanced Configuration Panel
            % Trailer Inertia Multiplier
            uilabel(obj.advancedConfigTab, 'Position', [10, 280, 300, 20], 'Text', 'Trailer Inertia Multiplier:');
            obj.I_trailerMultiplierField = uieditfield(obj.advancedConfigTab, 'numeric', ...
                'Position', [320, 280, 100, 20], 'Value', 1, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Max Articulation Angle
            uilabel(obj.advancedConfigTab, 'Position', [10, 240, 300, 20], 'Text', 'Max Articulation Angle (deg):');
            obj.maxDeltaField = uieditfield(obj.advancedConfigTab, 'numeric', ...
                'Position', [320, 240, 100, 20], 'Value', 70, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Time Step Multiplier
            uilabel(obj.advancedConfigTab, 'Position', [10, 200, 300, 20], 'Text', 'Time Step Multiplier:');
            obj.dtMultiplierField = uieditfield(obj.advancedConfigTab, 'numeric', ...
                'Position', [320, 200, 100, 20], 'Value', 0.5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Input Smoothing Window Size
            uilabel(obj.advancedConfigTab, 'Position', [10, 160, 300, 20], 'Text', 'Signal Smoothing Window Size (sec):');
            obj.windowSizeField = uieditfield(obj.advancedConfigTab, 'numeric', ...
                'Position', [320, 160, 100, 20], 'Value', 1, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            %% Control Limits Tab
            
            % --- Steering Controller Parameters ---
            uicontrol(obj.controlLimitsTab, 'Style', 'text', ...
                'Position', [10, 430, 460, 20], ...
                'String', '--- Steering Controller Parameters ---', ...
                'FontWeight', 'bold', 'FontSize', 10);
            
            % --- Steering Curve File Selection ---
            uilabel(obj.controlLimitsTab, 'Position', [10, 400, 200, 20], 'Text', 'Steering Curve File:');
            obj.steeringCurveFilePathField = uieditfield(obj.controlLimitsTab, 'text', ...
                'Position', [220, 400, 200, 20], 'Value', 'class8_truck_SteeringCurve.xlsx', ...
                'Editable', 'off', ...
                'Tooltip', 'Path to the Steering Curve Excel File');
            
            obj.steeringCurveBrowseButton = uibutton(obj.controlLimitsTab, 'push', ...
                'Position', [430, 400, 60, 20], 'Text', 'Browse...', ...
                'Tooltip', 'Select Steering Curve Excel File', ...
                'ButtonPushedFcn', @(src, event)obj.selectSteeringCurveFile());
            
            % --- Steering Angle Parameters ---
            % Max Steering Angle at 0 Speed
            uilabel(obj.controlLimitsTab, 'Position', [10, 370, 300, 20], 'Text', 'Max Steering Angle at 0 Speed (deg):');
            obj.maxSteeringAngleAtZeroSpeedField = uieditfield(obj.controlLimitsTab, 'numeric', ...
                'Position', [320, 370, 100, 20], 'Value', 30, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());
            
            % Max Speed for Steering Limitation
            uilabel(obj.controlLimitsTab, 'Position', [10, 340, 300, 20], 'Text', 'Max Speed for Steering Limit (m/s):');
            obj.maxSteeringSpeedField = uieditfield(obj.controlLimitsTab, 'numeric', ...
                'Position', [320, 340, 100, 20], 'Value', 30, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());
            
            % --- Acceleration Limiter Parameters ---
            uicontrol(obj.controlLimitsTab, 'Style', 'text', ...
                'Position', [10, 310, 460, 20], ...
                'String', '--- Acceleration Limiter Parameters ---', ...
                'FontWeight', 'bold', 'FontSize', 10);
            
            % Min Acceleration at Max Speed
            uilabel(obj.controlLimitsTab, 'Position', [10, 280, 300, 20], 'Text', 'Acceleration at Max Speed (m/s²):');
            obj.minAccelAtMaxSpeedField = uieditfield(obj.controlLimitsTab, 'numeric', ...
                'Position', [320, 280, 100, 20], 'Value', 1.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());
            
            % Min Deceleration at Max Speed
            uilabel(obj.controlLimitsTab, 'Position', [10, 250, 300, 20], 'Text', 'Deceleration at Max Speed (m/s²):');
            obj.minDecelAtMaxSpeedField = uieditfield(obj.controlLimitsTab, 'numeric', ...
                'Position', [320, 250, 100, 20], 'Value', -1.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());
            
            % --- Acceleration Curve Parameters ---
            uilabel(obj.controlLimitsTab, 'Position', [10, 220, 300, 20], 'Text', 'Acceleration Curve File:');
            obj.accelCurveFilePathField = uieditfield(obj.controlLimitsTab, 'text', ...
                'Position', [320, 220, 150, 20], 'Value', 'class8_truck_AccelCurve.xlsx', ...
                'Editable', 'off', ...
                'Tooltip', 'Path to the Acceleration Curve Excel File');
            
            obj.accelCurveBrowseButton = uibutton(obj.controlLimitsTab, 'push', ...
                'Position', [480, 220, 80, 20], 'Text', 'Browse...', ...
                'Tooltip', 'Select Acceleration Curve File', ...
                'ButtonPushedFcn', @(src, event)obj.selectAccelCurveFile());
            
            % --- Deceleration Curve Parameters ---
            uilabel(obj.controlLimitsTab, 'Position', [10, 190, 300, 20], 'Text', 'Deceleration Curve File:');
            obj.decelCurveFilePathField = uieditfield(obj.controlLimitsTab, 'text', ...
                'Position', [320, 190, 150, 20], 'Value', 'class8_truck_DecelCurve.xlsx', ...
                'Editable', 'off', ...
                'Tooltip', 'Path to the Deceleration Curve Excel File');
            
            obj.decelCurveBrowseButton = uibutton(obj.controlLimitsTab, 'push', ...
                'Position', [480, 190, 80, 20], 'Text', 'Browse...', ...
                'Tooltip', 'Select Deceleration Curve File', ...
                'ButtonPushedFcn', @(src, event)obj.selectDecelCurveFile());
            
            % --- Maximum Speed Parameter ---
            uilabel(obj.controlLimitsTab, 'Position', [10, 160, 300, 20], 'Text', 'Maximum Speed Limit (m/s):');
            obj.maxSpeedField = uieditfield(obj.controlLimitsTab, 'numeric', ...
                'Position', [320, 160, 100, 20], 'Value', 25.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());
            
            % --- Max Speed for Acceleration Limiting ---
            uilabel(obj.controlLimitsTab, 'Position', [10, 130, 300, 20], 'Text', 'Max Speed for Accel Limiting (m/s):');
            obj.maxSpeedForAccelLimitingField = uieditfield(obj.controlLimitsTab, 'numeric', ...
                'Position', [320, 130, 100, 20], 'Value', 30.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());
            % --- End of Control Limits Parameters ---


            %% PID Controller Tab
            uicontrol(obj.pidControllerTab, 'Style', 'text', 'Position', [10, 420, 460, 20], ...
                'String', '--- PID Speed Controller Parameters ---', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % Proportional Gain (Kp)
            uilabel(obj.pidControllerTab, 'Position', [10, 400, 300, 20], 'Text', 'Proportional Gain (Kp):');
            obj.KpField = uieditfield(obj.pidControllerTab, 'numeric', ...
                'Position', [320, 400, 100, 20], 'Value', 1.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Integral Gain (Ki)
            uilabel(obj.pidControllerTab, 'Position', [10, 370, 300, 20], 'Text', 'Integral Gain (Ki):');
            obj.KiField = uieditfield(obj.pidControllerTab, 'numeric', ...
                'Position', [320, 370, 100, 20], 'Value', 0.5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Derivative Gain (Kd)
            uilabel(obj.pidControllerTab, 'Position', [10, 340, 300, 20], 'Text', 'Derivative Gain (Kd):');
            obj.KdField = uieditfield(obj.pidControllerTab, 'numeric', ...
                'Position', [320, 340, 100, 20], 'Value', 0.1, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Levant differentiator parameters
            uilabel(obj.pidControllerTab, 'Position', [10, 310, 300, 20], 'Text', 'Lambda1 Accel:');
            obj.lambda1AccelField = uieditfield(obj.pidControllerTab, 'numeric', ...
                'Position', [320, 310, 100, 20], 'Value', 1.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            uilabel(obj.pidControllerTab, 'Position', [10, 280, 300, 20], 'Text', 'Lambda2 Accel:');
            obj.lambda2AccelField = uieditfield(obj.pidControllerTab, 'numeric', ...
                'Position', [320, 280, 100, 20], 'Value', 1.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            uilabel(obj.pidControllerTab, 'Position', [10, 250, 300, 20], 'Text', 'Lambda1 Jerk:');
            obj.lambda1JerkField = uieditfield(obj.pidControllerTab, 'numeric', ...
                'Position', [320, 250, 100, 20], 'Value', 1.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            uilabel(obj.pidControllerTab, 'Position', [10, 220, 300, 20], 'Text', 'Lambda2 Jerk:');
            obj.lambda2JerkField = uieditfield(obj.pidControllerTab, 'numeric', ...
                'Position', [320, 220, 100, 20], 'Value', 1.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            uilabel(obj.pidControllerTab, 'Position', [10, 190, 300, 20], 'Text', 'Lambda1 Velocity:');
            obj.lambda1VelField = uieditfield(obj.pidControllerTab, 'numeric', ...
                'Position', [320, 190, 100, 20], 'Value', 1.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            uilabel(obj.pidControllerTab, 'Position', [10, 160, 300, 20], 'Text', 'Lambda2 Velocity:');
            obj.lambda2VelField = uieditfield(obj.pidControllerTab, 'numeric', ...
                'Position', [320, 160, 100, 20], 'Value', 1.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Checkbox to Enable/Disable Speed Controller
            obj.enableSpeedControllerCheckbox = uicontrol(obj.pidControllerTab, 'Style', 'checkbox', ...
                'Position', [10, 130, 300, 20], 'String', 'Enable Speed Controller', ...
                'Value', 1, 'Callback', @(src, event)obj.configurationChanged());
            % --- End of PID Controller Parameters ---

            %% Vehicle Parameters Panel (Tractor)
            % Tractor Parameters Label
            uilabel(obj.vehicleParamsTab, 'Position', [10, 440, 200, 20], 'Text', 'Tractor Parameters', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % Tractor Length
            uilabel(obj.vehicleParamsTab, 'Position', [10, 400, 200, 20], 'Text', 'Length (m):');
            obj.tractorLengthField = uieditfield(obj.vehicleParamsTab, 'numeric', ...
                'Position', [220, 400, 100, 20], 'Value', 6.5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Tractor Width
            uilabel(obj.vehicleParamsTab, 'Position', [10, 370, 200, 20], 'Text', 'Width (m):');
            obj.tractorWidthField = uieditfield(obj.vehicleParamsTab, 'numeric', ...
                'Position', [220, 370, 100, 20], 'Value', 2.5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Tractor Height
            uilabel(obj.vehicleParamsTab, 'Position', [10, 340, 200, 20], 'Text', 'Height (m):');
            obj.tractorHeightField = uieditfield(obj.vehicleParamsTab, 'numeric', ...
                'Position', [220, 340, 100, 20], 'Value', 3.8, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Tractor CG Height
            uilabel(obj.vehicleParamsTab, 'Position', [10, 310, 200, 20], 'Text', 'CG Height (m):');
            obj.tractorCoGHeightField = uieditfield(obj.vehicleParamsTab, 'numeric', ...
                'Position', [220, 310, 100, 20], 'Value', 1.5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Tractor Wheelbase
            uilabel(obj.vehicleParamsTab, 'Position', [10, 280, 200, 20], 'Text', 'Wheelbase (m):');
            obj.tractorWheelbaseField = uieditfield(obj.vehicleParamsTab, 'numeric', ...
                'Position', [220, 280, 100, 20], 'Value', 4.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Tractor Track Width
            uilabel(obj.vehicleParamsTab, 'Position', [10, 250, 200, 20], 'Text', 'Track Width (m):');
            obj.tractorTrackWidthField = uieditfield(obj.vehicleParamsTab, 'numeric', ...
                'Position', [220, 250, 100, 20], 'Value', 2.1, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Number of Axles (1-2)
            uilabel(obj.vehicleParamsTab, 'Position', [10, 220, 200, 20], 'Text', 'Number of Axles (1-2):');
            obj.tractorNumAxlesDropdown = uidropdown(obj.vehicleParamsTab, ...
                'Position', [220, 220, 100, 20], ...
                'Items', {'1', '2'}, ...
                'Value', '2', ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Tractor Axle Spacing
            uilabel(obj.vehicleParamsTab, 'Position', [10, 190, 200, 20], 'Text', 'Axle Spacing (m):');
            obj.tractorAxleSpacingField = uieditfield(obj.vehicleParamsTab, 'numeric', ...
                'Position', [220, 190, 100, 20], 'Value', 1.310, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % --- New Dropdown for Tractor Tires per Axle ---
            uilabel(obj.vehicleParamsTab, 'Position', [10, 130, 250, 20], 'Text', 'Number of Tires per Axle (Tractor):');
            obj.numTiresPerAxleTractorDropDown = uidropdown(obj.vehicleParamsTab, ...
                'Position', [270, 130, 100, 20], ...
                'Items', {'2', '4'}, ...
                'Value', '4', ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());
            % --- End of Tractor Tires per Axle ---

            %% Trailer Parameters Panel
            % Trailer Parameters Label
            uilabel(obj.trailerParamsTab, 'Position', [10, 440, 200, 20], 'Text', 'Trailer Parameters', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % Trailer Length
            uilabel(obj.trailerParamsTab, 'Position', [10, 400, 200, 20], 'Text', 'Length (m):');
            obj.trailerLengthField = uieditfield(obj.trailerParamsTab, 'numeric', ...
                'Position', [220, 400, 100, 20], 'Value', 12.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Trailer Width
            uilabel(obj.trailerParamsTab, 'Position', [10, 370, 200, 20], 'Text', 'Width (m):');
            obj.trailerWidthField = uieditfield(obj.trailerParamsTab, 'numeric', ...
                'Position', [220, 370, 100, 20], 'Value', 2.5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Trailer Height
            uilabel(obj.trailerParamsTab, 'Position', [10, 340, 200, 20], 'Text', 'Height (m):');
            obj.trailerHeightField = uieditfield(obj.trailerParamsTab, 'numeric', ...
                'Position', [220, 340, 100, 20], 'Value', 4.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Trailer CG Height
            uilabel(obj.trailerParamsTab, 'Position', [10, 310, 200, 20], 'Text', 'CG Height (m):');
            obj.trailerCoGHeightField = uieditfield(obj.trailerParamsTab, 'numeric', ...
                'Position', [220, 310, 100, 20], 'Value', 1.5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Trailer Wheelbase
            uilabel(obj.trailerParamsTab, 'Position', [10, 280, 200, 20], 'Text', 'Wheelbase (m):');
            obj.trailerWheelbaseField = uieditfield(obj.trailerParamsTab, 'numeric', ...
                'Position', [220, 280, 100, 20], 'Value', 8.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Trailer Track Width
            uilabel(obj.trailerParamsTab, 'Position', [10, 250, 200, 20], 'Text', 'Track Width (m):');
            obj.trailerTrackWidthField = uieditfield(obj.trailerParamsTab, 'numeric', ...
                'Position', [220, 250, 100, 20], 'Value', 2.1, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());


            % Trailer Axle Spacing
            uilabel(obj.trailerParamsTab, 'Position', [10, 190, 200, 20], 'Text', 'Axle Spacing (m):');
            obj.trailerAxleSpacingField = uieditfield(obj.trailerParamsTab, 'numeric', ...
                'Position', [220, 190, 100, 20], 'Value', 1.310, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Trailer Hitch Distance
            uilabel(obj.trailerParamsTab, 'Position', [10, 160, 200, 20], 'Text', 'Trailer Hitch Distance (m):');
            obj.trailerHitchDistanceField = uieditfield(obj.trailerParamsTab, 'numeric', ...
                'Position', [220, 160, 100, 20], 'Value', 1.310, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Tractor Hitch Distance
            uilabel(obj.trailerParamsTab, 'Position', [10, 130, 200, 20], 'Text', 'Tractor Hitch Distance (m):');
            obj.tractorHitchDistanceField = uieditfield(obj.trailerParamsTab, 'numeric', ...
                'Position', [220, 130, 100, 20], 'Value', 4.5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Number of Tires per Axle on Trailer
            uilabel(obj.trailerParamsTab, 'Position', [10, 100, 200, 20], 'Text', 'Number of Tires per Axle on Trailer:');
            obj.numTiresPerAxleTrailerDropDown = uidropdown(obj.trailerParamsTab, ...
                'Position', [220, 100, 100, 20], ...
                'Items', {'2', '4'}, ...
                'Value', '4', ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());
            
            % --- Multi-Trailer Configuration ---
            uilabel(obj.trailerParamsTab, 'Position', [10, 70, 200, 20], 'Text', 'Num Trailer Boxes:');
            obj.trailerNumBoxesField = uieditfield(obj.trailerParamsTab, 'numeric', ...
                'Position', [220, 70, 100, 20], 'Value', 1, 'Limits', [1 Inf], ...
                'RoundFractionalValues', true, 'ValueChangedFcn', @(src, event)obj.trailerNumBoxesChanged(src, event));
            uilabel(obj.trailerParamsTab, 'Position', [10, 40, 200, 20], 'Text', 'Axles per Box (comma-separated):');
            obj.trailerAxlesPerBoxField = uieditfield(obj.trailerParamsTab, 'text', ...
                'Position', [220, 40, 100, 20], 'Value', '2', ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());
            % Distance between trailer boxes
            uilabel(obj.trailerParamsTab, 'Position', [10, 10, 200, 20], 'Text', 'Box Spacing (m):');
            obj.trailerBoxSpacingField = uieditfield(obj.trailerParamsTab, 'numeric', ...
                'Position', [220, 10, 100, 20], 'Value', 1.310, 'Limits', [0.5, Inf], ...
                'RoundFractionalValues', false, 'ValueChangedFcn', @(src, event)obj.configurationChanged());

            %% Tires Configuration Tab (Existing)
            % --- Tractor Tires Configuration ---
            uicontrol(obj.tiresConfigTab, 'Style', 'text', 'Position', [10, 390, 460, 20], ...
                'String', '--- Tractor Tires Configuration ---', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % Tractor Tire Height
            uilabel(obj.tiresConfigTab, 'Position', [10, 360, 200, 20], 'Text', 'Tractor Tire Height (m):');
            obj.tractorTireHeightField = uieditfield(obj.tiresConfigTab, 'numeric', ...
                'Position', [220, 360, 100, 20], 'Value', 1.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Tractor Tire Width
            uilabel(obj.tiresConfigTab, 'Position', [10, 330, 200, 20], 'Text', 'Tractor Tire Width (m):');
            obj.tractorTireWidthField = uieditfield(obj.tiresConfigTab, 'numeric', ...
                'Position', [220, 330, 100, 20], 'Value', 0.3, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % --- Trailer Tires Configuration ---
            uicontrol(obj.tiresConfigTab, 'Style', 'text', 'Position', [10, 290, 460, 20], ...
                'String', '--- Trailer Tires Configuration ---', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % Trailer Tire Height
            uilabel(obj.tiresConfigTab, 'Position', [10, 260, 200, 20], 'Text', 'Trailer Tire Height (m):');
            obj.trailerTireHeightField = uieditfield(obj.tiresConfigTab, 'numeric', ...
                'Position', [220, 260, 100, 20], 'Value', 1.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Trailer Tire Width
            uilabel(obj.tiresConfigTab, 'Position', [10, 230, 200, 20], 'Text', 'Trailer Tire Width (m):');
            obj.trailerTireWidthField = uieditfield(obj.tiresConfigTab, 'numeric', ...
                'Position', [220, 230, 100, 20], 'Value', 0.3, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());
            % --- End of Tires Configuration ---

            %% Stiffness & Damping Tab (Existing)
            % --- Stiffness Configuration ---
            uicontrol(obj.stiffnessDampingTab, 'Style', 'text', 'Position', [10, 420, 220, 20], ...
                'String', '--- Stiffness Parameters ---', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % Stiffness X
            uilabel(obj.stiffnessDampingTab, 'Position', [10, 390, 150, 20], 'Text', 'Stiffness X (N/m):');
            obj.stiffnessXField = uieditfield(obj.stiffnessDampingTab, 'numeric', ...
                'Position', [170, 390, 100, 20], 'Value', 1e4, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Stiffness Y
            uilabel(obj.stiffnessDampingTab, 'Position', [10, 360, 150, 20], 'Text', 'Stiffness Y (N/m):');
            obj.stiffnessYField = uieditfield(obj.stiffnessDampingTab, 'numeric', ...
                'Position', [170, 360, 100, 20], 'Value', 1e4, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Stiffness Z
            uilabel(obj.stiffnessDampingTab, 'Position', [10, 330, 150, 20], 'Text', 'Stiffness Z (N/m):');
            obj.stiffnessZField = uieditfield(obj.stiffnessDampingTab, 'numeric', ...
                'Position', [170, 330, 100, 20], 'Value', 1e4, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Stiffness Roll
            uilabel(obj.stiffnessDampingTab, 'Position', [10, 300, 170, 20], 'Text', 'Stiffness Roll (N·m/rad):');
            obj.stiffnessRollField = uieditfield(obj.stiffnessDampingTab, 'numeric', ...
                'Position', [170, 300, 100, 20], 'Value', 5e3, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Stiffness Pitch
            uilabel(obj.stiffnessDampingTab, 'Position', [10, 270, 170, 20], 'Text', 'Stiffness Pitch (N·m/rad):');
            obj.stiffnessPitchField = uieditfield(obj.stiffnessDampingTab, 'numeric', ...
                'Position', [170, 270, 100, 20], 'Value', 1e3, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Stiffness Yaw
            uilabel(obj.stiffnessDampingTab, 'Position', [10, 240, 170, 20], 'Text', 'Stiffness Yaw (N·m/rad):');
            obj.stiffnessYawField = uieditfield(obj.stiffnessDampingTab, 'numeric', ...
                'Position', [170, 240, 100, 20], 'Value', 2e4, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % --- Damping Configuration ---
            uicontrol(obj.stiffnessDampingTab, 'Style', 'text', 'Position', [300, 420, 220, 20], ...
                'String', '--- Damping Parameters ---', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % Damping X
            uilabel(obj.stiffnessDampingTab, 'Position', [300, 390, 160, 20], 'Text', 'Damping X (N·s/m):');
            obj.dampingXField = uieditfield(obj.stiffnessDampingTab, 'numeric', ...
                'Position', [470, 390, 100, 20], 'Value', 1e5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Damping Y
            uilabel(obj.stiffnessDampingTab, 'Position', [300, 360, 160, 20], 'Text', 'Damping Y (N·s/m):');
            obj.dampingYField = uieditfield(obj.stiffnessDampingTab, 'numeric', ...
                'Position', [470, 360, 100, 20], 'Value', 1e5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Damping Z
            uilabel(obj.stiffnessDampingTab, 'Position', [300, 330, 160, 20], 'Text', 'Damping Z (N·s/m):');
            obj.dampingZField = uieditfield(obj.stiffnessDampingTab, 'numeric', ...
                'Position', [470, 330, 100, 20], 'Value', 5e4, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Damping Roll
            uilabel(obj.stiffnessDampingTab, 'Position', [300, 300, 180, 20], 'Text', 'Damping Roll (N·m·s/rad):');
            obj.dampingRollField = uieditfield(obj.stiffnessDampingTab, 'numeric', ...
                'Position', [470, 300, 100, 20], 'Value', 4e5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Damping Pitch
            uilabel(obj.stiffnessDampingTab, 'Position', [300, 270, 180, 20], 'Text', 'Damping Pitch (N·m·s/rad):');
            obj.dampingPitchField = uieditfield(obj.stiffnessDampingTab, 'numeric', ...
                'Position', [470, 270, 100, 20], 'Value', 4e5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Damping Yaw
            uilabel(obj.stiffnessDampingTab, 'Position', [300, 240, 180, 20], 'Text', 'Damping Yaw (N·m·s/rad):');
            obj.dampingYawField = uieditfield(obj.stiffnessDampingTab, 'numeric', ...
                'Position', [470, 240, 100, 20], 'Value', 2e4, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            %% X Coefficients Tab
            % Title for X Coefficients
            uicontrol(obj.xCoefficientsTab, 'Style', 'text', 'Position', [10, 420, 460, 20], ...
                'String', '--- X Coefficients Configuration ---', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % pCx1
            uilabel(obj.xCoefficientsTab, 'Position', [10, 390, 150, 20], 'Text', 'Shape factor pCx1:');
            obj.pCx1Field = uieditfield(obj.xCoefficientsTab, 'numeric', ...
                'Position', [170, 390, 100, 20], 'Value', 7e5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pDx1
            uilabel(obj.xCoefficientsTab, 'Position', [10, 360, 150, 20], 'Text', 'Peak lateral force pDx1 (N):');
            obj.pDx1Field = uieditfield(obj.xCoefficientsTab, 'numeric', ...
                'Position', [170, 360, 100, 20], 'Value', 7e5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pDx2
            uilabel(obj.xCoefficientsTab, 'Position', [10, 330, 150, 20], 'Text', 'Peak lateral force pDx2 (N):');
            obj.pDx2Field = uieditfield(obj.xCoefficientsTab, 'numeric', ...
                'Position', [170, 330, 100, 20], 'Value', 7e5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pEx1
            uilabel(obj.xCoefficientsTab, 'Position', [10, 300, 150, 20], 'Text', 'Curvature factor pEx1:');
            obj.pEx1Field = uieditfield(obj.xCoefficientsTab, 'numeric', ...
                'Position', [170, 300, 100, 20], 'Value', 10.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pEx2
            uilabel(obj.xCoefficientsTab, 'Position', [10, 270, 150, 20], 'Text', 'Curvature adjustments pEx2:');
            obj.pEx2Field = uieditfield(obj.xCoefficientsTab, 'numeric', ...
                'Position', [170, 270, 100, 20], 'Value', 0.1, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pEx3
            uilabel(obj.xCoefficientsTab, 'Position', [10, 240, 150, 20], 'Text', 'Curvature adjustments pEx3:');
            obj.pEx3Field = uieditfield(obj.xCoefficientsTab, 'numeric', ...
                'Position', [170, 240, 100, 20], 'Value', 0.05, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pEx4
            uilabel(obj.xCoefficientsTab, 'Position', [10, 210, 150, 20], 'Text', 'Curvature adjustments pEx4:');
            obj.pEx4Field = uieditfield(obj.xCoefficientsTab, 'numeric', ...
                'Position', [170, 210, 100, 20], 'Value', 0.03, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pKx1
            uilabel(obj.xCoefficientsTab, 'Position', [10, 180, 150, 20], 'Text', 'Cornering stiffness pKx1:');
            obj.pKx1Field = uieditfield(obj.xCoefficientsTab, 'numeric', ...
                'Position', [170, 180, 100, 20], 'Value', 6e4, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pKx2
            uilabel(obj.xCoefficientsTab, 'Position', [10, 150, 150, 20], 'Text', 'Cornering stiffness pKx2:');
            obj.pKx2Field = uieditfield(obj.xCoefficientsTab, 'numeric', ...
                'Position', [170, 150, 100, 20], 'Value', 0.8, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pKx3
            uilabel(obj.xCoefficientsTab, 'Position', [10, 120, 150, 20], 'Text', 'Cornering stiffness pKx3:');
            obj.pKx3Field = uieditfield(obj.xCoefficientsTab, 'numeric', ...
                'Position', [170, 120, 100, 20], 'Value', 0.8, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            %% Y Coefficients Tab
            % Title for Y Coefficients
            uicontrol(obj.yCoefficientsTab, 'Style', 'text', 'Position', [10, 420, 460, 20], ...
                'String', '--- Y Coefficients Configuration ---', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % pCy1
            uilabel(obj.yCoefficientsTab, 'Position', [10, 390, 150, 20], 'Text', 'Shape factor pCy1:');
            obj.pCy1Field = uieditfield(obj.yCoefficientsTab, 'numeric', ...
                'Position', [170, 390, 100, 20], 'Value', 7e5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pDy1
            uilabel(obj.yCoefficientsTab, 'Position', [10, 360, 150, 20], 'Text', 'Peak lateral force pDy1 (N):');
            obj.pDy1Field = uieditfield(obj.yCoefficientsTab, 'numeric', ...
                'Position', [170, 360, 100, 20], 'Value', 7e5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pDy2
            uilabel(obj.yCoefficientsTab, 'Position', [10, 330, 150, 20], 'Text', 'Peak lateral force pDy2 (N):');
            obj.pDy2Field = uieditfield(obj.yCoefficientsTab, 'numeric', ...
                'Position', [170, 330, 100, 20], 'Value', 7e5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pEy1
            uilabel(obj.yCoefficientsTab, 'Position', [10, 300, 150, 20], 'Text', 'Curvature factor pEy1:');
            obj.pEy1Field = uieditfield(obj.yCoefficientsTab, 'numeric', ...
                'Position', [170, 300, 100, 20], 'Value', 10.0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pEy2
            uilabel(obj.yCoefficientsTab, 'Position', [10, 270, 150, 20], 'Text', 'Curvature adjustments pEy2:');
            obj.pEy2Field = uieditfield(obj.yCoefficientsTab, 'numeric', ...
                'Position', [170, 270, 100, 20], 'Value', 0.1, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pEy3
            uilabel(obj.yCoefficientsTab, 'Position', [10, 240, 150, 20], 'Text', 'Curvature adjustments pEy3:');
            obj.pEy3Field = uieditfield(obj.yCoefficientsTab, 'numeric', ...
                'Position', [170, 240, 100, 20], 'Value', 0.05, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pEy4
            uilabel(obj.yCoefficientsTab, 'Position', [10, 210, 150, 20], 'Text', 'Curvature adjustments pEy4:');
            obj.pEy4Field = uieditfield(obj.yCoefficientsTab, 'numeric', ...
                'Position', [170, 210, 100, 20], 'Value', 0.03, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pKy1
            uilabel(obj.yCoefficientsTab, 'Position', [10, 180, 150, 20], 'Text', 'Cornering stiffness pKy1:');
            obj.pKy1Field = uieditfield(obj.yCoefficientsTab, 'numeric', ...
                'Position', [170, 180, 100, 20], 'Value', 60000, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pKy2
            uilabel(obj.yCoefficientsTab, 'Position', [10, 150, 150, 20], 'Text', 'Cornering stiffness pKy2:');
            obj.pKy2Field = uieditfield(obj.yCoefficientsTab, 'numeric', ...
                'Position', [170, 150, 100, 20], 'Value', 0.8, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % pKy3
            uilabel(obj.yCoefficientsTab, 'Position', [10, 120, 150, 20], 'Text', 'Cornering stiffness pKy3:');
            obj.pKy3Field = uieditfield(obj.yCoefficientsTab, 'numeric', ...
                'Position', [170, 120, 100, 20], 'Value', 0.8, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            %% Aerodynamics Tab
            % Air Density
            uilabel(obj.aerodynamicsTab, 'Position', [10, 400, 200, 20], 'Text', 'Air Density (kg/m³):');
            obj.airDensityField = uieditfield(obj.aerodynamicsTab, 'numeric', ...
                'Position', [220, 400, 100, 20], 'Value', 1.225, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Drag Coefficient
            uilabel(obj.aerodynamicsTab, 'Position', [10, 360, 200, 20], 'Text', 'Drag Coefficient:');
            obj.dragCoeffField = uieditfield(obj.aerodynamicsTab, 'numeric', ...
                'Position', [220, 360, 100, 20], 'Value', 0.8, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % *** New Wind Speed Field ***
            uilabel(obj.aerodynamicsTab, 'Position', [10, 320, 200, 20], 'Text', 'Wind Speed (m/s):');
            obj.windSpeedField = uieditfield(obj.aerodynamicsTab, 'numeric', ...
                'Position', [220, 320, 100, 20], 'Value', 0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % *** New Wind Angle (Degrees) Field ***
            uilabel(obj.aerodynamicsTab, 'Position', [10, 280, 200, 20], 'Text', 'Wind Angle (deg):');
            obj.windAngleDegField = uieditfield(obj.aerodynamicsTab, 'numeric', ...
                'Position', [220, 280, 100, 20], 'Value', 45, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            %% Road Conditions Tab
            % --- Road Conditions Configuration ---
            uicontrol(obj.roadConditionsTab, 'Style', 'text', 'Position', [10, 420, 460, 20], ...
                'String', '--- Road Conditions Configuration ---', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % Slope Angle
            uilabel(obj.roadConditionsTab, 'Position', [10, 380, 200, 20], 'Text', 'Slope Angle (degrees):');
            obj.slopeAngleField = uieditfield(obj.roadConditionsTab, 'numeric', ...
                'Position', [220, 380, 100, 20], 'Value', 0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Road Friction Coefficient (μ)
            uilabel(obj.roadConditionsTab, 'Position', [10, 340, 250, 20], 'Text', 'Road Friction Coefficient (μ):');
            obj.roadFrictionCoefficientField = uieditfield(obj.roadConditionsTab, 'numeric', ...
                'Position', [270, 340, 100, 20], 'Value', 0.9, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Road Surface Type
            uilabel(obj.roadConditionsTab, 'Position', [10, 300, 200, 20], 'Text', 'Road Surface Type:');
            obj.roadSurfaceTypeDropdown = uidropdown(obj.roadConditionsTab, ...
                'Position', [220, 300, 150, 20], ...
                'Items', {'Dry Asphalt', 'Wet Asphalt', 'Snow', 'Ice', 'Custom'}, ...
                'Value', 'Dry Asphalt', ...
                'ValueChangedFcn', @(src, event)obj.roadSurfaceTypeChanged(src, event));

            % Road Roughness
            uilabel(obj.roadConditionsTab, 'Position', [10, 260, 200, 20], 'Text', 'Road Roughness:');
            obj.roadRoughnessField = uieditfield(obj.roadConditionsTab, 'numeric', ...
                'Position', [220, 260, 100, 20], 'Value', 0, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            %% Pressure Matrices Tab
            % Create label to display total number of tires
            obj.totalTiresLabel = uilabel(obj.pressureMatricesTab, ...
                'Position', [10, 450, 300, 20], ...
                'Text', 'Total Number of Tires: Calculating...', ...
                'FontWeight', 'bold', 'FontSize', 12);

            % Initialize pressure matrix panels
            obj.pressureMatrixPanels = struct();

            % Create panels for each pressure matrix
            tireCounts = {'4', '6', '8', '10', '12', '14', '16', '18', '20'};
            for i = 1:length(tireCounts)
                nTires = str2double(tireCounts{i});
                panelTag = sprintf('Tires%d', nTires);
                panel = uipanel(obj.pressureMatricesTab, 'Title', sprintf('Pressures for %d Tires', nTires), ...
                    'Position', [10, 10, 460, 430], 'Visible', 'off', 'Tag', panelTag);

                % Create table for pressures with editable columns
                uitable(panel, 'Data', num2cell(obj.pressureMatrices.(panelTag)), ...
                    'ColumnName', {'Pressure (Pa)'}, ...
                    'RowName', arrayfun(@(x) sprintf('Tire %d', x), 1:nTires, 'UniformOutput', false), ...
                    'Position', [10, 10, 440, 410], 'Tag', sprintf('Table%d', nTires), ...
                    'ColumnEditable', true, ...  % Make the column editable
                    'CellEditCallback', @(src, event)obj.pressureMatrixEdited(src, event));

                % Store panel in struct
                obj.pressureMatrixPanels.(panelTag) = panel;
            end

            %% *** Transmission Configuration Tab ***
            % Add UI components for Transmission Configuration (excluding Gear Ratios)

            % Title for Transmission Configuration
            uilabel(obj.transmissionConfigTab, 'Position', [10, 420, 460, 20], ...
                'Text', '--- Transmission Configuration ---', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % Maximum Gear Number
            uilabel(obj.transmissionConfigTab, 'Position', [10, 380, 150, 20], 'Text', 'Maximum Gear Number:');
            obj.maxGearField = uieditfield(obj.transmissionConfigTab, 'numeric', ...
                'Position', [170, 380, 100, 20], 'Value', 6, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Final Drive Ratio
            uilabel(obj.transmissionConfigTab, 'Position', [10, 350, 150, 20], 'Text', 'Final Drive Ratio:');
            obj.finalDriveRatioField = uieditfield(obj.transmissionConfigTab, 'numeric', ...
                'Position', [170, 350, 100, 20], 'Value', 3.42, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Shift Up Speed Thresholds (m/s)
            uilabel(obj.transmissionConfigTab, 'Position', [10, 320, 150, 20], 'Text', 'Shift Up Speeds (m/s):');
            obj.shiftUpSpeedField = uieditfield(obj.transmissionConfigTab, 'text', ...
                'Position', [170, 320, 250, 20], ...
                'Value', '3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37', ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Shift Down Speed Thresholds (m/s)
            uilabel(obj.transmissionConfigTab, 'Position', [10, 290, 150, 20], 'Text', 'Shift Down Speeds (m/s):');
            obj.shiftDownSpeedField = uieditfield(obj.transmissionConfigTab, 'text', ...
                'Position', [170, 290, 250, 20], ...
                'Value', '2.5, 4.5, 6.5, 8.5, 10.5, 12.5, 14.5, 16.5, 18.5, 20.5, 22.5, 24.5, 26.5, 28.5, 30.5, 32.5, 34.5, 36.5', ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Shift Delay Time
            uilabel(obj.transmissionConfigTab, 'Position', [10, 260, 150, 20], 'Text', 'Shift Delay (s):');
            obj.shiftDelayField = uieditfield(obj.transmissionConfigTab, 'numeric', ...
                'Position', [170, 260, 100, 20], 'Value', 0.5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            %% *** Gear Ratios Tab ***
            % Add UI components for Gear Ratios Configuration

            % Title for Gear Ratios Configuration
            uilabel(obj.gearRatiosTab, 'Position', [10, 420, 460, 20], ...
                'Text', '--- Gear Ratios Configuration ---', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % Gear Ratios Table
            uilabel(obj.gearRatiosTab, 'Position', [10, 380, 150, 20], 'Text', 'Gear Ratios:');
            obj.gearRatiosTable = uitable(obj.gearRatiosTab, ...
                'Position', [10, 130, 460, 270], ...
                'ColumnName', {'Gear Number', 'Ratio'}, ...
                'RowName', {}, ... % Will be set dynamically
                'Data', table2cell(obj.gearRatiosData), ... % Initialize with table data
                'ColumnEditable', [false true], ... % Gear Number is not editable
                'CellEditCallback', @(src, event)obj.gearRatiosEdited(src, event));

            % Set Row Names Dynamically
            obj.updateGearRatiosUITable();

            % Add Gear Button
            obj.addGearButton = uicontrol(obj.gearRatiosTab, 'Style', 'pushbutton', ...
                'Position', [10, 100, 100, 20], 'String', 'Add Gear', ...
                'Callback', @(src, event)obj.addGear());

            % Remove Gear Button
            obj.removeGearButton = uicontrol(obj.gearRatiosTab, 'Style', 'pushbutton', ...
                'Position', [120, 100, 120, 20], 'String', 'Remove Selected Gear', ...
                'Callback', @(src, event)obj.removeGear());

            % Engine Gear Mapping Table
            % Uncomment and configure if needed
            % uilabel(obj.gearRatiosTab, 'Position', [10, 160, 150, 20], 'Text', 'Engine Gear Mapping:');
            % obj.engineGearMappingTable = uitable(obj.gearRatiosTab, ...
            %     'Position', [10, 10, 460, 130], ...
            %     'ColumnName', {'Engine Gear', 'Transmission Gear'}, ...
            %     'RowName', {}, ...
            %     'Data', {}, ...
            %     'ColumnEditable', [false true], ... % Engine Gear is not editable
            %     'CellEditCallback', @(src, event)obj.engineGearMappingEdited(src, event));

            %% Suspension Model Tab
            % Add UI components for Suspension Model

            % Spring Stiffness K_spring
            uilabel(obj.suspensionModelTab, 'Position', [10, 300, 200, 20], 'Text', 'Spring Stiffness K\_spring (N/m):');
            obj.K_springField = uieditfield(obj.suspensionModelTab, 'numeric', ...
                'Position', [220, 300, 150, 20], 'Value', 30000, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Damping Coefficient C_damping
            uilabel(obj.suspensionModelTab, 'Position', [10, 250, 200, 20], 'Text', 'Damping Coefficient C\_damping (N·s/m):');
            obj.C_dampingField = uieditfield(obj.suspensionModelTab, 'numeric', ...
                'Position', [220, 250, 150, 20], 'Value', 1500, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Rest Length
            uilabel(obj.suspensionModelTab, 'Position', [10, 200, 200, 20], 'Text', 'Rest Length (m):');
            obj.restLengthField = uieditfield(obj.suspensionModelTab, 'numeric', ...
                'Position', [220, 200, 150, 20], 'Value', 0.5, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            %% *** Engine Configuration Tab ***
            % Add UI components for Engine Configuration

            % Title for Engine Configuration
            uilabel(obj.engineConfigTab, 'Position', [10, 420, 460, 20], ...
                'Text', '--- Engine Configuration Parameters ---', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % Max Engine Torque
            uilabel(obj.engineConfigTab, 'Position', [10, 390, 200, 20], 'Text', 'Max Engine Torque (Nm):');
            obj.maxEngineTorqueField = uieditfield(obj.engineConfigTab, 'numeric', ...
                'Position', [220, 390, 150, 20], 'Value', 3000, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Max Power
            uilabel(obj.engineConfigTab, 'Position', [10, 360, 200, 20], 'Text', 'Max Power (W):');
            obj.maxPowerField = uieditfield(obj.engineConfigTab, 'numeric', ...
                'Position', [220, 360, 150, 20], 'Value', 400000, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Idle RPM
            uilabel(obj.engineConfigTab, 'Position', [10, 330, 200, 20], 'Text', 'Idle RPM:');
            obj.idleRPMField = uieditfield(obj.engineConfigTab, 'numeric', ...
                'Position', [220, 330, 150, 20], 'Value', 600, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Redline RPM
            uilabel(obj.engineConfigTab, 'Position', [10, 300, 200, 20], 'Text', 'Redline RPM:');
            obj.redlineRPMField = uieditfield(obj.engineConfigTab, 'numeric', ...
                'Position', [220, 300, 150, 20], 'Value', 2100, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Engine Brake Torque
            uilabel(obj.engineConfigTab, 'Position', [10, 270, 200, 20], 'Text', 'Engine Brake Torque (Nm):');
            obj.engineBrakeTorqueField = uieditfield(obj.engineConfigTab, 'numeric', ...
                'Position', [220, 270, 150, 20], 'Value', 1500, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Fuel Consumption Rate
            uilabel(obj.engineConfigTab, 'Position', [10, 240, 200, 20], 'Text', 'Fuel Consumption Rate (kg/s):');
            obj.fuelConsumptionRateField = uieditfield(obj.engineConfigTab, 'numeric', ...
                'Position', [220, 240, 150, 20], 'Value', 0.2, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % *** New Torque File Selection ---
            % Label for Torque File
            uilabel(obj.engineConfigTab, 'Position', [10, 210, 150, 20], 'Text', 'Torque File (Excel):');

            % Text Box to Display Selected File Name
            obj.torqueFileNameField = uieditfield(obj.engineConfigTab, 'text', ...
                'Position', [170, 210, 200, 20], ...
                'Editable', 'off', ...
                'Value', 'class8_truck_torque_curve.xlsx', ...
                'Tooltip', 'Selected Torque Excel File');

            % Button to Browse and Select Torque File
            obj.torqueFileButton = uibutton(obj.engineConfigTab, 'push', ...
                'Position', [380, 210, 80, 20], ...
                'Text', 'Browse...', ...
                'Tooltip', 'Select an Excel file for torque data', ...
                'ButtonPushedFcn', @(src, event)obj.selectTorqueFile());

            % Optionally, initialize torqueFilePath as empty
            obj.torqueFilePath = '';
            % *** End of Torque File Selection ---

            %% *** Brake Configuration Tab ***
            % --- Brake Configuration Parameters ---
            uicontrol(obj.brakeConfigTab, 'Style', 'text', 'Position', [10, 420, 460, 20], ...
                'String', '--- Brake Configuration Parameters ---', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % *** New: Maximum Braking Force ***
            uilabel(obj.brakeConfigTab, 'Position', [10, 390, 200, 20], 'Text', 'Max Braking Force (N):');
            obj.maxBrakingForceField = uieditfield(obj.brakeConfigTab, 'numeric', ...
                'Position', [220, 390, 150, 20], 'Value', 60000, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());
            % *** End of Max Braking Force ***

            % Braking Force
            uilabel(obj.brakeConfigTab, 'Position', [10, 360, 200, 20], 'Text', 'Braking Force (N):');
            obj.brakingForceField = uieditfield(obj.brakeConfigTab, 'numeric', ...
                'Position', [220, 360, 150, 20], 'Value', 50000, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Brake Efficiency
            uilabel(obj.brakeConfigTab, 'Position', [10, 330, 200, 20], 'Text', 'Brake Efficiency (%):');
            obj.brakeEfficiencyField = uieditfield(obj.brakeConfigTab, 'numeric', ...
                'Position', [220, 330, 150, 20], 'Value', 85, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % *** Brake Bias Field ***
            uilabel(obj.brakeConfigTab, 'Position', [10, 300, 200, 20], 'Text', 'Brake Bias (Front/Rear %):');
            obj.brakeBiasField = uieditfield(obj.brakeConfigTab, 'numeric', ...
                'Position', [220, 300, 150, 20], 'Value', 50, ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());
            % *** End of Brake Bias Field ***

            % *** Brake Type Dropdown ***
            uilabel(obj.brakeConfigTab, 'Position', [10, 270, 200, 20], 'Text', 'Brake Type:');
            obj.brakeTypeDropdown = uidropdown(obj.brakeConfigTab, ...
                'Position', [220, 270, 150, 20], ...
                'Items', {'Disk', 'Drum', 'Air Disk', 'Air Drum'}, ... % Initial items, will be updated based on vehicle type
                'Value', 'Air Disk', ...
                'ValueChangedFcn', @(src, event)obj.configurationChanged());
            % *** End of Brake Type Dropdown ***

           %% *** Create Clutch Configuration Tab ***
            % Add UI components for Clutch Configuration

            % Title for Clutch Configuration
            uilabel(obj.clutchConfigTab, 'Position', [10, 420, 460, 20], ...
                'Text', '--- Clutch Configuration Parameters ---', ...
                'FontWeight', 'bold', 'FontSize', 10);

            % Maximum Clutch Torque
            uilabel(obj.clutchConfigTab, 'Position', [10, 390, 200, 20], 'Text', 'Max Clutch Torque (Nm):');
            obj.maxClutchTorqueField = uieditfield(obj.clutchConfigTab, 'numeric', ...
                'Position', [220, 390, 150, 20], 'Value', 3500, ... % Hardcoded value for Class 8 truck
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Clutch Engagement Speed
            uilabel(obj.clutchConfigTab, 'Position', [10, 360, 200, 20], 'Text', 'Clutch Engagement Speed (1/10 s):');
            obj.engagementSpeedField = uieditfield(obj.clutchConfigTab, 'numeric', ...
                'Position', [220, 360, 150, 20], 'Value', 1.0, ... % Hardcoded value
                'ValueChangedFcn', @(src, event)obj.configurationChanged());

            % Clutch Disengagement Speed
            uilabel(obj.clutchConfigTab, 'Position', [10, 330, 200, 20], 'Text', 'Clutch Disengagement Speed (1/10 s):');
            obj.disengagementSpeedField = uieditfield(obj.clutchConfigTab, 'numeric', ...
                'Position', [220, 330, 150, 20], 'Value', 0.5, ... % Hardcoded value
                'ValueChangedFcn', @(src, event)obj.configurationChanged());



            % Show the appropriate pressure matrix based on initial configuration
            obj.updatePressureMatrices();

            %% Set Initial Visibility of Trailer Parameters Tab Contents
            obj.setTrailerTabVisibility(obj.includeTrailerCheckbox.Value);
        end

        % Updates the RowName of the Gear Ratios UITable based on gear numbers
        function updateGearRatiosUITable(obj)
            numGears = height(obj.gearRatiosData);
            rowNames = arrayfun(@(x) sprintf('Gear %d', x), obj.gearRatiosData.Gear, 'UniformOutput', false);
            obj.gearRatiosTable.RowName = rowNames;
        end

        % Sets the visibility of all components within the Trailer Parameters tab.
        function setTrailerTabVisibility(obj, isVisible)
            % Determine enable state
            enableState = 'off';
            if isVisible
                enableState = 'on';
            end

            % Get all children of the trailerParamsTab
            children = obj.trailerParamsTab.Children;

            % Iterate through each child and set Enable state
            for i = 1:length(children)
                if isprop(children(i), 'Enable')
                    children(i).Enable = enableState;
                end
            end
        end

        % Callback when the Include Trailer checkbox value changes.
        function includeTrailerChanged(obj, src, event)
            if src.Value
                % Trailer is included: enable trailer configuration fields
                obj.numTiresPerAxleTrailerDropDown.Enable = 'on';
                obj.numTiresPerAxleTrailerDropDown.Items = {'2', '4'};
                obj.numTiresPerAxleTrailerDropDown.Value = obj.numTiresPerAxleTrailerDropDown.Items{1};
                if isprop(obj, 'trailerNumBoxesField')
                    obj.trailerNumBoxesField.Enable = 'on';
                end
                if isprop(obj, 'trailerAxlesPerBoxField')
                    obj.trailerAxlesPerBoxField.Enable = 'on';
                end
                if isprop(obj, 'trailerBoxSpacingField')
                    obj.trailerBoxSpacingField.Enable = 'on';
                end
                obj.setTrailerTabVisibility(true);
            else
                % Trailer is excluded: disable trailer configuration fields
                obj.numTiresPerAxleTrailerDropDown.Items = {'0'};
                obj.numTiresPerAxleTrailerDropDown.Value = '0';
                obj.numTiresPerAxleTrailerDropDown.Enable = 'off';
                if isprop(obj, 'trailerNumBoxesField')
                    obj.trailerNumBoxesField.Enable = 'off';
                end
                if isprop(obj, 'trailerAxlesPerBoxField')
                    obj.trailerAxlesPerBoxField.Enable = 'off';
                end
                if isprop(obj, 'trailerBoxSpacingField')
                    obj.trailerBoxSpacingField.Enable = 'off';
                end
                obj.setTrailerTabVisibility(false);
            end
            obj.updatePressureMatrices();  % Update pressure matrices when trailer inclusion changes
        end

        % Callback when the Road Surface Type dropdown value changes.
        function roadSurfaceTypeChanged(obj, src, event)
            switch src.Value
                case 'Dry Asphalt'
                    obj.roadFrictionCoefficientField.Value = 0.9;
                    obj.roadRoughnessField.Value = 0;
                case 'Wet Asphalt'
                    obj.roadFrictionCoefficientField.Value = 0.7;
                    obj.roadRoughnessField.Value = 0.1;
                case 'Snow'
                    obj.roadFrictionCoefficientField.Value = 0.3;
                    obj.roadRoughnessField.Value = 0.2;
                case 'Ice'
                    obj.roadFrictionCoefficientField.Value = 0.1;
                    obj.roadRoughnessField.Value = 0.3;
                case 'Custom'
                    % Allow the user to input custom values
                    % Do nothing; fields are already editable
            end
            obj.updatePressureMatrices();  % Update pressure matrices if road conditions affect tires
        end

        function configurationChanged(obj)
            % This method is called whenever a relevant configuration parameter changes    
            
            % --- Handle Steering Curve Data ---
            if ~isempty(obj.steeringCurveData)
                % Extract maximum steering angle at 0 speed and minimum at max speed
                % Assuming the data is sorted by speed in ascending order
                obj.maxSteeringAngleAtZeroSpeedField = obj.steeringCurveData.SteeringAngle(1);
                obj.minSteeringAngleAtMaxSpeedField = obj.steeringCurveData.SteeringAngle(end);
                
                % Optionally, you can implement interpolation or other logic based on the curve
                % For example, create a function handle for steering angle vs. speed
                obj.steeringAngleFunction = @(speed) interp1(obj.steeringCurveData.Speed, obj.steeringCurveData.SteeringAngle, speed, 'linear', 'extrap');
                
                disp('Steering angles updated based on the steering curve data.');
            else
                % Fallback to default or existing values if no steering curve is loaded
                obj.maxSteeringAngleAtZeroSpeedField = 30; % Default value
                obj.minSteeringAngleAtMaxSpeedField = 10; % Default value
            end
            
            % Compute wind_angle_rad based on wind_angle_deg
            obj.windAngleRad = deg2rad(obj.windAngleDegField.Value);
            
            % Retrieve brake bias
            currentBrakeBias = obj.brakeBiasField.Value;
            
            % Validate brake bias (should be between 0 and 100)
            if currentBrakeBias < 0 || currentBrakeBias > 100
                uialert(obj.figureHandle, ... % Changed from obj.brakeConfigTab to obj.figureHandle
                    'Brake Bias must be between 0 and 100%.', ...
                    'Invalid Brake Bias', 'Icon', 'error');
                obj.brakeBiasField.Value = 50; % Reset to default if invalid
                currentBrakeBias = 50;
            end
            
            % Retrieve brake type from GUI
            currentBrakeType = obj.brakeTypeDropdown.Value;

            % Update stored stiffness and damping for spinners
            if isprop(obj, 'spinnerConfig') && ~isempty(obj.spinnerConfig)
                for i = 1:numel(obj.spinnerConfig)
                    sc = obj.spinnerConfig(i);
                    obj.spinnerConfig(i).stiffness = struct( ...
                        'x', sc.stiffnessXField.Value, ...
                        'y', sc.stiffnessYField.Value, ...
                        'z', sc.stiffnessZField.Value, ...
                        'roll', sc.stiffnessRollField.Value, ...
                        'pitch', sc.stiffnessPitchField.Value, ...
                        'yaw', sc.stiffnessYawField.Value );

                    obj.spinnerConfig(i).damping = struct( ...
                        'x', sc.dampingXField.Value, ...
                        'y', sc.dampingYField.Value, ...
                        'z', sc.dampingZField.Value, ...
                        'roll', sc.dampingRollField.Value, ...
                        'pitch', sc.dampingPitchField.Value, ...
                        'yaw', sc.dampingYawField.Value );
                end
            end

            % Refresh parent vehicle model parameters
            if ~isempty(obj.vehicleModel) && isa(obj.vehicleModel, 'VehicleModel')
                obj.vehicleModel.simParams = obj.vehicleModel.getSimulationParameters();
            end

            % Update pressure matrices if necessary
            obj.updatePressureMatrices();
            % Validate that acceleration and deceleration curves are loaded
            if isempty(obj.accelerationCurve) || isempty(obj.decelerationCurve)
                % uialert(obj.figureHandle, ...
                %     'Please ensure both acceleration and deceleration curves are loaded.', ...
                %     'Missing Curves', 'Icon', 'warning');
                obj.accelerationCurve = readmatrix(obj.accelCurveFilePathField.Value);
                obj.decelerationCurve = readmatrix(obj.decelCurveFilePathField.Value);
            end
            
            % Additional updates or validations can be performed here
        end

        % Updates the Pressure Matrices tab based on the current configuration
        function updatePressureMatrices(obj)
            % Calculate total number of tires based on current configuration
            % Total Tires = 2 (Frontal Tires) + (Tractor Axles * Tractor Tires per Axle) + (Trailer Axles * Trailer Tires per Axle)

            % Tractor configuration
            tractorAxles = str2double(obj.tractorNumAxlesDropdown.Value);
            tractorTiresPerAxle = str2double(obj.numTiresPerAxleTractorDropDown.Value);
            tractorTotalTires = tractorAxles * tractorTiresPerAxle;

            % Trailer configuration
            if obj.includeTrailerCheckbox.Value
                trailerAxles = obj.getTrailerNumAxles();
                trailerTiresPerAxle = str2double(obj.numTiresPerAxleTrailerDropDown.Value);
                trailerTotalTires = trailerAxles * trailerTiresPerAxle;
            else
                trailerTotalTires = 0;
            end

            % Compute total tires
            totalTires = 2 + tractorTotalTires + trailerTotalTires;

            % Update the totalTiresLabel
            obj.totalTiresLabel.Text = sprintf('Total Number of Tires: %d', totalTires);

            % Create the pressure matrix panel if it doesn't exist
            obj.createPressureMatrixPanel(totalTires);

            % Determine the panel tag
            panelTag = sprintf('Tires%d', totalTires);

            % Hide all existing panels
            panelFields = fieldnames(obj.pressureMatrixPanels);
            for i = 1:length(panelFields)
                currentTag = panelFields{i};
                obj.pressureMatrixPanels.(currentTag).Visible = 'off';
            end

            % Show the corresponding panel
            if isfield(obj.pressureMatrixPanels, panelTag)
                obj.pressureMatrixPanels.(panelTag).Visible = 'on';
            end
        end

        %/**
        % * @brief Sets the pressure matrices in the GUI.
        % *
        % * This function receives a struct with a single field representing the tire count
        % * (e.g., 'Tires18') and its corresponding pressure values. It updates the appropriate
        % * uitable within the GUI to display each pressure value as a separate row.
        % *
        % * @param pressureMatrices Struct containing pressure data for one tire count.
        % */
        function setPressureMatrices(obj, pressureMatrices)
            % Validate Input Struct
            fields = fieldnames(pressureMatrices);
            if length(fields) ~= 1
                error('setPressureMatrices expects a struct with exactly one field representing the tire count (e.g., Tires18).');
            end
            
            % Extract Tire Count Tag and Pressures
            tiresTag = fields{1}; % e.g., 'Tires18'
            pressures = pressureMatrices.(tiresTag); % e.g., [780000, 780000, ..., 820000]
            
            % Validate Pressure Data
            if ~isnumeric(pressures) || ~isvector(pressures)
                error('Pressure data must be a numeric vector.');
            end
            
            % Convert Pressures to Column Vector
            pressures = pressures(:); % Ensures a Nx1 column vector
            
            % Check if the corresponding pressure matrix panel exists
            if ~isfield(obj.pressureMatrixPanels, tiresTag)
                error('No pressure matrix panel found for %s.', tiresTag);
            end
            
            % Access the Specific UIPanel
            panel = obj.pressureMatrixPanels.(tiresTag);
            
            % Find the uitable within the UIPanel
            table = findobj(panel, 'Type', 'uitable');
            if isempty(table)
                error('No uitable found within the panel %s.', tiresTag);
            end
            
            % Update the uitable Data and RowName
            table.Data = num2cell(pressures); % Assign pressures as cell array
            table.RowName = arrayfun(@(x) sprintf('Tire %d', x), 1:length(pressures), 'UniformOutput', false);
            
            % Optional: Set Column Name and Editability
            table.ColumnName = {'Pressure (Pa)'};
            table.ColumnEditable = true; % Set to true if you want users to edit pressures
            
            % Update Visibility: Show current panel and hide others
            allTags = fieldnames(obj.pressureMatrixPanels);
            for i = 1:length(allTags)
                if strcmp(allTags{i}, tiresTag)
                    obj.pressureMatrixPanels.(allTags{i}).Visible = 'on';
                else
                    obj.pressureMatrixPanels.(allTags{i}).Visible = 'off';
                end
            end
            
            % Confirmation Message
            fprintf('Pressure matrix for %s loaded successfully.\n', tiresTag);
        end

        % Retrieves the pressure matrix for the current configuration from the GUI.
        function pressureMatrices = getPressureMatrices(obj)
            % Calculate total number of tires based on current configuration
            tractorAxles = str2double(obj.tractorNumAxlesDropdown.Value);
            tractorTiresPerAxle = str2double(obj.numTiresPerAxleTractorDropDown.Value);
            tractorTotalTires = tractorAxles * tractorTiresPerAxle;

            % Trailer configuration
            if obj.includeTrailerCheckbox.Value
                trailerAxles = obj.getTrailerNumAxles();
                trailerTiresPerAxle = str2double(obj.numTiresPerAxleTrailerDropDown.Value);
                trailerTotalTires = trailerAxles * trailerTiresPerAxle;
            else
                trailerTotalTires = 0;
            end

            % Compute total tires
            totalTires = 2 + tractorTotalTires + trailerTotalTires;

            % Determine the panel tag
            panelTag = sprintf('Tires%d', totalTires);

            % Retrieve the pressure matrix from the current panel
            if isfield(obj.pressureMatrixPanels, panelTag)
                panel = obj.pressureMatrixPanels.(panelTag);
                table = findobj(panel, 'Type', 'uitable');
                if isempty(table)
                    warning('No table found for %s.', panelTag);
                    pressureMatrices = struct();
                    return;
                end
                data = table.Data;
                pressures = cell2mat(data(:, 1));
            else
                % If the panel does not exist, generate default pressures
                if ~isfield(obj.pressureMatrices, panelTag)
                    obj.pressureMatrices.(panelTag) = obj.generatePressureMatrix(totalTires);
                end
                pressures = obj.pressureMatrices.(panelTag);
            end

            pressureMatrices.(panelTag) = pressures;
        end

        % Creates a pressure matrix panel for a given number of tires
        function createPressureMatrixPanel(obj, totalTires)
            panelTag = sprintf('Tires%d', totalTires);

            % Ensure we have a pressure matrix for this tire count
            if ~isfield(obj.pressureMatrices, panelTag)
                obj.pressureMatrices.(panelTag) = obj.generatePressureMatrix(totalTires);
            end

            % Create the panel if it does not exist
            if ~isfield(obj.pressureMatrixPanels, panelTag)
                panel = uipanel(obj.pressureMatricesTab, ...
                    'Title', sprintf('Pressures for %d Tires', totalTires), ...
                    'Position', [10, 10, 460, 430], ...
                    'Visible', 'off', ...
                    'Tag', panelTag);

                % Create table for pressures with editable columns
                uitable(panel, 'Data', num2cell(obj.pressureMatrices.(panelTag)), ...
                    'ColumnName', {'Pressure (Pa)'}, ...
                    'RowName', arrayfun(@(x) sprintf('Tire %d', x), 1:totalTires, 'UniformOutput', false), ...
                    'Position', [10, 10, 440, 410], 'Tag', sprintf('Table%d', totalTires), ...
                    'ColumnEditable', true, ...  % Make the column editable
                    'CellEditCallback', @(src, event)obj.pressureMatrixEdited(src, event));

                % Store panel in struct
                obj.pressureMatrixPanels.(panelTag) = panel;
            end
        end

        % Callback when a pressure matrix table is edited
        function pressureMatrixEdited(obj, src, event)
            % Update the pressureMatrices structure when a table cell is edited
            tableTag = src.Tag;  % e.g., 'Table4'
            matches = regexp(tableTag, 'Table(\d+)', 'tokens');
            if isempty(matches)
                warning('Unexpected table tag format: %s', tableTag);
                return;
            end
            nTires = str2double(matches{1}{1});
            pressures = cell2mat(src.Data(:, 1));
            obj.pressureMatrices.(sprintf('Tires%d', nTires)) = pressures;
        end

        %% *** Transmission Configuration Methods ***
        % Callback when a gear ratio is edited
        function gearRatiosEdited(obj, src, event)
            % Validate and update gear ratios based on user input
            if isempty(event.NewData) || isnan(event.NewData) || event.NewData <= 0
                uialert(obj.figureHandle, ... % Changed from obj.gearRatiosTab to obj.figureHandle
                    'Invalid gear ratio. Please enter a positive numeric value.', ...
                    'Invalid Input', 'Icon', 'error');
                % Revert to old data
                src.Data{event.Indices(1), event.Indices(2)} = event.PreviousData;
                return;
            end
            % Optionally, enforce unique gear numbers
            % (Assuming gear numbers are sequential and unique)
            Gears = cell2mat(src.Data(:,1));
            if any(diff(Gears) <= 0)
                uialert(obj.figureHandle, ... % Changed from obj.gearRatiosTab to obj.figureHandle
                    'Gear numbers must be unique and in ascending order.', ...
                    'Invalid Gear Number', 'Icon', 'error');
                % Revert to old data
                src.Data{event.Indices(1), event.Indices(2)} = event.PreviousData;
                return;
            end
            % Update the gearRatiosData table
            gearIdx = event.Indices(1);
            obj.gearRatiosData.Ratio(gearIdx) = event.NewData;

            % Update the uitable's Data
            obj.gearRatiosTable.Data = table2cell(obj.gearRatiosData);

            % Update Row Names
            obj.updateGearRatiosUITable();

            % Optional: Further processing can be done here
            disp(['Gear ', num2str(obj.gearRatiosData.Gear(gearIdx)), ' ratio updated to ', num2str(event.NewData)]);
        end

        % Method to add a new gear
        function addGear(obj)
            % Determine the next gear number
            if isempty(obj.gearRatiosData.Gear)
                nextGear = 1;
            else
                nextGear = max(obj.gearRatiosData.Gear) + 1;
            end

            % Prompt user for the new gear ratio (optional)
            prompt = sprintf('Enter ratio for Gear %d:', nextGear);
            dlgtitle = 'Add New Gear';
            dims = [1 50];
            definput = {'1.0'};
            answer = inputdlg(prompt, dlgtitle, dims, definput);
            if isempty(answer)
                % User canceled the input dialog
                return;
            end
            newRatio = str2double(answer{1});
            if isnan(newRatio) || newRatio <= 0
                uialert(obj.figureHandle, ...
                    'Invalid ratio entered. Please enter a positive numeric value.', ...
                    'Invalid Input', 'Icon', 'error');
                return;
            end

            % Append the new gear to gearRatiosData
            newGear = table(nextGear, newRatio, 'VariableNames', {'Gear', 'Ratio'});
            obj.gearRatiosData = [obj.gearRatiosData; newGear];

            % Update the uitable's Data and RowName
            obj.gearRatiosTable.Data = table2cell(obj.gearRatiosData);
            obj.updateGearRatiosUITable();

            % Confirmation Message
            disp(['Added Gear ', num2str(nextGear), ' with ratio ', num2str(newRatio)]);
        end

        % Method to remove the selected gear
        function removeGear(obj)
            % Get selected cells
            selected = obj.gearRatiosTable.Selection;

            if isempty(selected)
                uialert(obj.figureHandle, ... % Changed from obj.gearRatiosTab to obj.figureHandle
                    'Please select a gear to remove.', ...
                    'No Selection', 'Icon', 'warning');
                return;
            end

            % Determine unique rows selected
            rows = unique(selected(:,1));

            % Confirm deletion
            choice = questdlg(sprintf('Are you sure you want to remove %d selected gear(s)?', length(rows)), ...
                'Confirm Removal', 'Yes', 'No', 'No');
            if strcmp(choice, 'No')
                return;
            end

            % Remove selected rows from gearRatiosData
            obj.gearRatiosData(rows, :) = [];

            % Update the uitable's Data and RowName
            obj.gearRatiosTable.Data = table2cell(obj.gearRatiosData);
            obj.updateGearRatiosUITable();

            % Confirmation Message
            disp(['Removed ', num2str(length(rows)), ' gear(s).']);
        end

        % Callback when engine gear mapping is edited
        function engineGearMappingEdited(obj, src, event)
            % Validate and update engine to transmission gear mapping
            if isempty(event.NewData) || isnan(event.NewData) || event.NewData <= 0
                uialert(obj.figureHandle, ... % Changed from obj.gearRatiosTab to obj.figureHandle
                    'Invalid transmission gear number. Please enter a positive numeric value.', ...
                    'Invalid Input', 'Icon', 'error');
                % Revert to old data
                src.Data{event.Indices(1), event.Indices(2)} = event.PreviousData;
                return;
            end
            % Ensure the transmission gear exists
            maxTransmissionGear = max(cell2mat(obj.gearRatiosTable.Data(:,1)));
            if event.NewData > maxTransmissionGear
                uialert(obj.figureHandle, ... % Changed from obj.gearRatiosTab to obj.figureHandle
                    sprintf('Transmission gear %d does not exist. Please add it first.', event.NewData), ...
                    'Invalid Transmission Gear', 'Icon', 'error');
                % Revert to old data
                src.Data{event.Indices(1), event.Indices(2)} = event.PreviousData;
                return;
            end
            % Additional mapping validations can be added here
        end

        %% *** Brake Configuration Methods ***
        % Add any brake-specific methods here if needed

        %% *** New Callback Methods for Command Input Boxes ***
        % Callback when steering commands change
        function steeringCommandsChanged(obj, src, event)
            % Handle steering commands input
            steeringCommands = src.Value;
            % Process steeringCommands as needed
            disp(['Steering Commands Updated: ', steeringCommands]);
        end

        function tirePressureCommandsChanged(obj, src, event)
            % Handle tire pressure commands input
            tirePressureCommands = src.Value;
            disp(['Tire Pressure Commands Updated: ', tirePressureCommands]);
            
            % You can parse and handle the commands here as needed, 
            % for example updating an internal property, 
            % or triggering a method that applies the tire pressure in simulation.
        end

        % Callback when acceleration commands change
        function accelerationCommandsChanged(obj, src, event)
            % Handle acceleration commands input
            accelerationCommands = src.Value;
            % Process accelerationCommands as needed
            disp(['Acceleration Commands Updated: ', accelerationCommands]);
        end
        %% *** End of New Callback Methods ***

        %% *** Torque File Selection Callback ***
        function selectTorqueFile(obj)
            % Open a file selection dialog for Excel files
            [file, path] = uigetfile({'*.xlsx;*.xls', 'Excel Files (*.xlsx, *.xls)'}, 'Select Torque Excel File');

            % Check if the user did not cancel the dialog
            if isequal(file, 0)
                disp('User canceled file selection.');
                return;
            end

            % Store the full path of the selected file
            obj.torqueFilePath = fullfile(path, file);

            % Update the text box with the selected file name
            obj.torqueFileNameField.Value = file;

            % Optionally, notify the user
            uialert(obj.figureHandle, ...
                sprintf('Selected Torque File:\n%s', obj.torqueFilePath), ...
                'Torque File Selected', 'Icon', 'info');

            % Additional processing can be done here if needed
        end
        %% *** End of Torque File Selection Callback ***

        %% *** Example Method Using the Torque File ***
        % (This method can be called when running the simulation)
        function runSimulation(obj)
            % Check if a torque file has been selected
            if isempty(obj.torqueFilePath)
                uialert(obj.figureHandle, ... % Changed from obj.engineConfigTab to obj.figureHandle
                    'Please select a torque file before running the simulation.', ...
                    'Torque File Missing', 'Icon', 'warning');
                return;
            end

            % Load the torque data from the Excel file
            try
                torqueData = readtable(obj.torqueFilePath);
                % Proceed with using torqueData in your simulation
                % ...
                disp('Torque data loaded successfully.');
            catch ME
                uialert(obj.figureHandle, ... % Changed from obj.engineConfigTab to obj.figureHandle
                    sprintf('Failed to load torque file:\n%s', ME.message), ...
                    'File Load Error', 'Icon', 'error');
                return;
            end

            % ... rest of the simulation code ...
        end
        %% *** End of Example Method ***

        %% *** Create Path Follower Tab ***
        % Create the Path Follower Tab components
        function createPathFollowerTab(obj)
            uilabel(obj.pathFollowerTab, 'Text', '--- Path Follower Configuration ---', ...
                'FontWeight', 'bold', 'FontSize', 10, 'Position', [10, 420, 400, 20]);

            % *** New: Map Commands Input Box ***
            uilabel(obj.pathFollowerTab, 'Text', 'Map Commands:', 'Position', [10, 380, 100, 20]);
            obj.mapCommandsBox = uieditfield(obj.pathFollowerTab, 'text', ...
                'Position', [120, 380, 350, 20], 'Value', 'straight(100,200,300,200)|curve(300,250,50,270,90,ccw)|straight(300,300,100,300)|curve(100,250,50,90,270,ccw)', ...
                'Tooltip', 'Enter map commands in the format: segment_type(parameters)|...');
            % *** End of Map Commands Input Box ***

            % *** New: Resolution Input Box ***
            uilabel(obj.pathFollowerTab, 'Text', 'Resolution:', 'Position', [10, 350, 100, 20]);
            obj.resolutionBox = uieditfield(obj.pathFollowerTab, 'numeric', ...
                'Position', [120, 350, 100, 20], 'Value', 1, ...
                'Limits', [0.1, 10], ... % Define reasonable limits
                'Tooltip', 'Define the resolution for waypoint generation (waypoints per meter)');
            % *** End of Resolution Input Box ***

            % *** New: Generate Waypoints Button ***
            obj.generateWaypointsButton = uibutton(obj.pathFollowerTab, 'Text', 'Generate Waypoints', ...
                'Position', [230, 350, 150, 25], ...
                'ButtonPushedFcn', @(src,event)obj.generateWaypoints());
            % *** End of Generate Waypoints Button ***

            % Waypoints Table Label
            uilabel(obj.pathFollowerTab, 'Text', 'Waypoints (X, Y):', 'Position', [10, 320, 100, 20]);

            % Create a table to display and edit waypoints
            obj.waypointsTable = uitable(obj.pathFollowerTab, ...
                'Position', [10, 50, 460, 250], ... % Reduced height from 280 to 250
                'ColumnName', {'X', 'Y'}, ...
                'Data', num2cell(obj.defaultWaypoints), ...
                'ColumnEditable', [true, true]);

            % Add Waypoint Button
            obj.addWaypointButton = uibutton(obj.pathFollowerTab, 'Text', 'Add Waypoint', ...
                'Position', [10, 10, 100, 30], ...
                'ButtonPushedFcn', @(src,event) obj.addWaypoint());

            % Remove Waypoint Button
            obj.removeWaypointButton = uibutton(obj.pathFollowerTab, 'Text', 'Remove Selected Waypoint', ...
                'Position', [120, 10, 150, 30], ...
                'ButtonPushedFcn', @(src,event) obj.removeWaypoint());
        end
        
        % Callback when number of trailer boxes changes
        function trailerNumBoxesChanged(obj, src, ~)
            nBoxes = src.Value;
            obj.createTrailerWeightFields(nBoxes);
            obj.updateSpinnerTabs(nBoxes);
            obj.configurationChanged();
        end
        
        % Update spinner configuration tabs based on number of trailer boxes
        function updateSpinnerTabs(obj, nBoxes)
            % Remove existing spinner tabs
            if isprop(obj, 'spinnerTabs') && ~isempty(obj.spinnerTabs)
                for idx = 1:numel(obj.spinnerTabs)
                    if isvalid(obj.spinnerTabs{idx})
                        delete(obj.spinnerTabs{idx});
                    end
                end
                obj.spinnerTabs = {};
                obj.spinnerConfig = struct();
            end
            % Only create spinner tabs if more than one box and trailer included
            if obj.includeTrailerCheckbox.Value && nBoxes > 1
                nSpinners = nBoxes - 1;
                for i = 1:nSpinners
                    tabTitle = sprintf('Spinner %d Stiffness & Damping', i);
                    tab = uitab(obj.configTabGroup, 'Title', tabTitle);
                    obj.spinnerTabs{end+1} = tab;
                    % --- Stiffness Parameters ---
                    uicontrol(tab, 'Style', 'text', 'Position', [10, 420, 220, 20], ...
                        'String', '--- Stiffness Parameters ---', 'FontWeight', 'bold', 'FontSize', 10);
                    stiffFields = {'X','Y','Z','Roll','Pitch','Yaw'};
                    stiffDefaults = [1e4,1e4,1e4,5e3,1e3,2e3];
                    for j = 1:numel(stiffFields)
                        yPos = 420 - 30*j;
                        field = stiffFields{j};
                        j_4='N·m/rad';
                        if j<4
                            j_4='N/m';
                        end
                        labelText = sprintf('Stiffness %s (%s):', field, j_4);
                        uilabel(tab, 'Position', [10, yPos, 150, 20], 'Text', labelText);
                        editField = uieditfield(tab, 'numeric', 'Position', [170, yPos, 100, 20], ...
                            'Value', stiffDefaults(j), 'ValueChangedFcn', @(src,evt)obj.configurationChanged());
                        obj.spinnerConfig(i).(['stiffness' field 'Field']) = editField;
                    end
                    % --- Damping Parameters ---
                    uicontrol(tab, 'Style', 'text', 'Position', [300, 420, 220, 20], ...
                        'String', '--- Damping Parameters ---', 'FontWeight', 'bold', 'FontSize', 10);
                    dampDefaults = [1e5,1e5,5e4,4e5,4e5,8e5];
                    for j = 1:numel(stiffFields)
                        yPos = 420 - 30*j;
                        field = stiffFields{j};
                        j_4='N·m·s/rad';
                        if j<4
                            j_4='N·s/m';
                        end
                        labelText = sprintf('Damping %s (%s):', field, j_4);
                        j_4=180;
                        if j<4
                            j_4=160;
                        end
                        lblWidth = j_4;
                        uilabel(tab, 'Position', [300, yPos, lblWidth, 20], 'Text', labelText);
                        editField = uieditfield(tab, 'numeric', 'Position', [470, yPos, 100, 20], ...
                            'Value', dampDefaults(j), 'ValueChangedFcn', @(src,evt)obj.configurationChanged());
                        obj.spinnerConfig(i).(['damping' field 'Field']) = editField;
                    end
                end
            end
        end

        % Create or update trailer box weight fields in the Basic Configuration tab
        function createTrailerWeightFields(obj, nBoxes)
            % Delete existing fields
            if ~isempty(obj.trailerBoxWeightFields)
                for i = 1:numel(obj.trailerBoxWeightFields)
                    if isvalid(obj.trailerBoxWeightFields{i})
                        delete(obj.trailerBoxWeightFields{i});
                    end
                end
            end
            obj.trailerBoxWeightFields = cell(nBoxes,4);
            yStart = 200; % position below initial velocity field
            for b = 1:nBoxes
                baseY = yStart - (b-1)*60;
                uilabel(obj.basicConfigTab, 'Position',[10, baseY+20,150,20], ...
                    'Text', sprintf('Trailer Box %d Weights (kg):', b));
                labels = {'FL','FR','RL','RR'};
                for j = 1:4
                    xPos = 10 + (j-1)*110;
                    obj.trailerBoxWeightFields{b,j} = uieditfield(obj.basicConfigTab,'numeric', ...
                        'Position',[xPos, baseY, 100,20],'Value',1000, ...
                        'ValueChangedFcn',@(src,evt)obj.configurationChanged());
                    obj.trailerBoxWeightFields{b,j}.Placeholder = labels{j};
                end
            end
        end

        % Helper to obtain the total number of trailer axles
        function nAxles = getTrailerNumAxles(obj)
            if isprop(obj, 'trailerNumAxlesDropdown') && ~isempty(obj.trailerNumAxlesDropdown)
                nAxles = str2double(obj.trailerNumAxlesDropdown.Value);
            elseif isprop(obj, 'trailerAxlesPerBoxField') && ~isempty(obj.trailerAxlesPerBoxField)
                vals = str2num(obj.trailerAxlesPerBoxField.Value); %#ok<ST2NM>
                if isempty(vals)
                    nAxles = 0;
                else
                    nAxles = sum(vals);
                end
            else
                nAxles = 0;
            end
        end

        %% Create Commands Tab
        % Hosts steering, acceleration and tire pressure command inputs
        function createCommandsTab(obj)
            obj.commandPanel = uipanel(obj.commandsTab, 'Position', [10, 10, 480, 170], 'Title', 'Commands');

            % Steering Commands
            uilabel(obj.commandPanel, 'Position', [10, 120, 150, 20], 'Text', 'Steering Commands:');
            obj.steeringCommandsBox = uieditfield(obj.commandPanel, 'text', ...
                'Position', [170, 120, 300, 20], ...
                'Value', 'simval_(195)|ramp_-30(1)|keep_-30(0.8)|ramp_0(0.2)|keep_0(1)', ...
                'ValueChangedFcn', @(src, event)obj.steeringCommandsChanged(src, event));

            % Acceleration Commands
            uilabel(obj.commandPanel, 'Position', [10, 80, 150, 20], 'Text', 'Acceleration Commands:');
            obj.accelerationCommandsBox = uieditfield(obj.commandPanel, 'text', ...
                'Position', [170, 80, 300, 20], ...
                'Value', 'simval_(200)', ...
                'ValueChangedFcn', @(src, event)obj.accelerationCommandsChanged(src, event));

            % Tire Pressure Commands
            uilabel(obj.commandPanel, 'Position', [10, 40, 150, 20], 'Text', 'Tire Pressure Commands:');
            obj.tirePressureCommandsBox = uieditfield(obj.commandPanel, 'text', ...
                'Position', [170, 40, 300, 20], ...
                'Value', 'pressure_(t:150-[tire:9,psi:70];[tire:2,psi:72];[tire:1,psi:7])', ...
                'ValueChangedFcn', @(src, event)obj.tirePressureCommandsChanged(src, event));
        end

        % Add a new waypoint to the table
        function addWaypoint(obj)
            currentData = obj.waypointsTable.Data;
            if isempty(currentData)
                newData = {100, 200}; % If empty, add a starting point
            else
                lastX = currentData{end, 1};
                lastY = currentData{end, 2};
                newData = {lastX+20, lastY}; % Example: add new waypoint ahead in X
            end
            obj.waypointsTable.Data = [currentData; newData];
        end

        % Remove the selected waypoint from the table
        function removeWaypoint(obj)
            if isempty(obj.waypointsTable.Selection)
                uialert(obj.figureHandle, ...
                    'Please select a waypoint to remove (click on the row).', ...
                    'No Selection', 'Icon', 'warning');
                return;
            end

            selectedRows = unique(obj.waypointsTable.Selection(:,1));
            data = obj.waypointsTable.Data;
            data(selectedRows, :) = [];
            obj.waypointsTable.Data = data;
        end


        % Generate waypoints based on map commands and resolution
        function generateWaypoints(obj)
            % Retrieve map commands and resolution from input boxes
            mapCommands = obj.mapCommandsBox.Value;
            resolution = obj.resolutionBox.Value;

            % Validate inputs
            if isempty(mapCommands)
                uialert(obj.figureHandle, ...
                    'Map Commands cannot be empty. Please enter valid commands.', ...
                    'Invalid Input', 'Icon', 'error');
                return;
            end

            if isnan(resolution) || resolution <= 0
                uialert(obj.figureHandle, ...
                    'Resolution must be a positive numeric value.', ...
                    'Invalid Input', 'Icon', 'error');
                return;
            end

            try
                % Generate waypoints based on map commands and resolution
                waypoints = obj.generateWaypointsHelper(mapCommands, resolution);

                % Update the waypointsTable with the generated waypoints
                obj.waypointsTable.Data = num2cell(waypoints);

                % Confirmation Message
                % uialert(obj.figureHandle, ...
                %     sprintf('Waypoints generated successfully.\nTotal Waypoints: %d', size(waypoints,1)), ...
                %     'Waypoints Generated', 'Icon', 'info');
            catch ME
                % Handle errors during waypoint generation
                uialert(obj.figureHandle, ...
                    sprintf('Error generating waypoints:\n%s', ME.message), ...
                    'Waypoint Generation Error', 'Icon', 'error');
            end
        end

        % Generates waypoints based on map commands and resolution
        function waypoints = generateWaypointsHelper(obj, pathString, resolution)
            % generateWaypoints Helper method to parse path commands and generate waypoints.
            %
            % pathString: String defining the path with segments separated by '|'.
            % resolution: Number of waypoints per meter (default = 1).
            % returns Nx2 matrix of [x, y] coordinates.

            if nargin < 3
                resolution = 1;
            end

            segments = split(pathString, '|');
            waypoints = [];

            for i = 1:length(segments)
                segment = strtrim(segments{i});
                if startsWith(segment, 'straight')
                    params = obj.parseStraight(segment);
                    wp = obj.generateStraightWaypoints(params, resolution);
                elseif startsWith(segment, 'curve')
                    params = obj.parseCurve(segment);
                    wp = obj.generateCurveWaypoints(params, resolution);
                else
                    error('Unknown segment type: %s', segment);
                end
                waypoints = [waypoints; wp]; %#ok<AGROW>
            end
        end

        % Parse straight segment
        function params = parseStraight(obj, segment)
            % parseStraight Extracts parameters from a straight segment command.
            % Example: 'straight(100,200,300,200)'

            tokens = regexp(segment, '^straight\(([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)\)$', 'tokens');
            if isempty(tokens)
                error('Invalid straight segment format: %s', segment);
            end
            tokens = tokens{1};
            params.startX = str2double(tokens{1});
            params.startY = str2double(tokens{2});
            params.endX = str2double(tokens{3});
            params.endY = str2double(tokens{4});
        end

        % Parse curve segment
        function params = parseCurve(obj, segment)
            % parseCurve Extracts parameters from a curve segment command.
            % Example: 'curve(300,250,50,270,90,ccw)'

            tokens = regexp(segment, '^curve\(([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),\s*(cw|ccw)\)$', 'tokens', 'ignorecase');
            if isempty(tokens)
                error('Invalid curve segment format: %s', segment);
            end
            tokens = tokens{1};
            params.centerX = str2double(tokens{1});
            params.centerY = str2double(tokens{2});
            params.radius = str2double(tokens{3});
            params.startAngle = str2double(tokens{4});
            params.endAngle = str2double(tokens{5});
            params.direction = lower(strtrim(tokens{6}));
        end

        % Generate straight waypoints
        function wp = generateStraightWaypoints(obj, params, resolution)
            % generateStraightWaypoints Generates waypoints for a straight segment.
            dx = params.endX - params.startX;
            dy = params.endY - params.startY;
            distance = sqrt(dx^2 + dy^2);
            numPoints = max(2, ceil(distance * resolution));
            x = linspace(params.startX, params.endX, numPoints)';
            y = linspace(params.startY, params.endY, numPoints)';
            wp = [x, y];
        end

        % Generate curve waypoints
        function wp = generateCurveWaypoints(obj, params, resolution)
            % generateCurveWaypoints Generates waypoints for a curve segment on the outer side.

            % Define an offset distance for the outer side
            offset = 2.5; % Adjust this value as needed or make it a parameter

            % Convert angles to radians
            startRad = deg2rad(params.startAngle);
            endRad = deg2rad(params.endAngle);

            % Determine angle direction and adjust theta accordingly
            if strcmp(params.direction, 'cw')
                if endRad > startRad
                    endRad = endRad - 2*pi;
                end
                theta = linspace(startRad, endRad, max(2, ceil(params.radius * abs(startRad - endRad) * resolution)));
                % For clockwise, outward normal is -radial direction
                outwardNormalFactor = -1;
            else
                if endRad < startRad
                    endRad = endRad + 2*pi;
                end
                theta = linspace(startRad, endRad, max(2, ceil(params.radius * abs(endRad - startRad) * resolution)));
                % For counter-clockwise, outward normal is +radial direction
                outwardNormalFactor = 1;
            end

            % Calculate waypoints on the original curve
            x = params.centerX + params.radius * cos(theta)';
            y = params.centerY + params.radius * sin(theta)';

            % Compute outward normals (radial vectors)
            nx = cos(theta)';
            ny = sin(theta)';

            % Apply offset to move waypoints outward
            x_outer = x + outwardNormalFactor * nx;
            y_outer = y + outwardNormalFactor * ny;

            wp = [x_outer, y_outer];
        end

        % Callback to select Acceleration Curve File
        function selectAccelCurveFile(obj)
            [file, path] = uigetfile({'*.xlsx', 'Excel files'}, 'Select Acceleration Curve File');
            if isequal(file, 0)
                disp('User canceled acceleration curve file selection.');
                return;
            end
            obj.accelCurveFilePathField.Value = fullfile(path, file);
            obj.loadAccelCurve(fullfile(path, file));
        end
    
        % Callback to select Deceleration Curve File
        function selectDecelCurveFile(obj)
            [file, path] = uigetfile({'*.xlsx', 'Excel files'}, 'Select Deceleration Curve File');
            if isequal(file, 0)
                disp('User canceled deceleration curve file selection.');
                return;
            end
            obj.decelCurveFilePathField.Value = fullfile(path, file);
            obj.loadDecelCurve(fullfile(path, file));
        end
    
        % Method to load Acceleration Curve from file
        function loadAccelCurve(obj, filepath)
            try
                data = load(filepath);
                if isfield(data, 'accelerationCurve')
                    obj.accelerationCurve = data.accelerationCurve;
                    disp('Acceleration curve loaded successfully.');
                else
                    uialert(obj.figureHandle, ...
                        'The selected file does not contain the variable "accelerationCurve".', ...
                        'Invalid Acceleration Curve File', 'Icon', 'error');
                end
            catch ME
                uialert(obj.figureHandle, ...
                    sprintf('Failed to load acceleration curve:\n%s', ME.message), ...
                    'Load Error', 'Icon', 'error');
            end
        end
    
        % Method to load Deceleration Curve from file
        function loadDecelCurve(obj, filepath)
            try
                data = load(filepath);
                if isfield(data, 'decelerationCurve')
                    obj.decelerationCurve = data.decelerationCurve;
                    disp('Deceleration curve loaded successfully.');
                else
                    uialert(obj.figureHandle, ...
                        'The selected file does not contain the variable "decelerationCurve".', ...
                        'Invalid Deceleration Curve File', 'Icon', 'error');
                end
            catch ME
                uialert(obj.figureHandle, ...
                    sprintf('Failed to load deceleration curve:\n%s', ME.message), ...
                    'Load Error', 'Icon', 'error');
            end
        end

        % --- Steering Curve File Selection Callback ---
        function selectSteeringCurveFile(obj)
            [file, path] = uigetfile({'*.xlsx;*.xls', 'Excel Files (*.xlsx, *.xls)'}, 'Select Steering Curve Excel File');
            if isequal(file, 0)
                disp('User canceled steering curve file selection.');
                return;
            end
            
            % Update the file path field
            obj.steeringCurveFilePathField.Value = fullfile(path, file);
            
            % Load the steering curve data
            obj.loadSteeringCurve(fullfile(path, file));
        end
        
        % Method to load Steering Curve from Excel file
        function loadSteeringCurve(obj, filepath)
            try
                % Assuming the Excel file has two columns: Speed (m/s) and Steering Angle (deg)
                data = readtable(filepath);
                
                % Validate the table has the required columns
                if width(data) < 2
                    error('Steering curve file must have at least two columns: Speed and Steering Angle.');
                end
                
                % Assign to steeringCurveData property
                obj.steeringCurveData = data;
                
                disp('Steering curve data loaded successfully.');
                
                % Trigger configuration changed if necessary
                obj.configurationChanged();
            catch ME
                uialert(obj.figureHandle, ...
                    sprintf('Failed to load steering curve file:\n%s', ME.message), ...
                    'Steering Curve Load Error', 'Icon', 'error');
            end
        end
    end
end