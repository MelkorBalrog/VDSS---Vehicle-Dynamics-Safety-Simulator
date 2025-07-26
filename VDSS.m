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
% * @brief Sets up the UI, runs vehicle simulations, and plots results.
% *
% * The `VDSS` function initializes the simulation environment by creating the user interface,
% * loading vehicle configurations, and managing user interactions through callback functions.
% * It leverages the `SimManager` class to execute simulations, detect collisions, and plot results.
% *
% * @author Miguel Marina <karel.capek.robotics@gmail.com>
% * @version 2.1
% * @date 2024-11-03
% */
function VDSS
    % VDSS sets up the UI, runs vehicle simulations, and plots results.

    %% Define Simulation Time Step
    dt = 0.005; % Time step in seconds (adjust as per your simulation)
    
    % Initialize a parallel pool if not already open
    if isempty(gcp('nocreate'))
        parpool;
    end
    
    %% Instantiate UIManager with Callback Functions
    callbacks = struct(...
        'LoadVehicle1Excel', @loadVehicle1Excel, ...
        'LoadVehicle2Excel', @loadVehicle2Excel, ...
        'StartSimulation', @startSimulation, ...
        'LoadSimulationData', @loadSimulationData, ...
        'SaveSimulationData', @saveSimulationData, ...
        'SaveConfiguration', @saveConfiguration, ...
        'LoadConfiguration', @loadConfiguration, ...
        'SaveSimulationResults', @saveSimulationResults, ...
        'SavePlotAsPNG', @savePlotAsPNG, ...
        'SaveManualControlData', @saveManualControlData ...
    );
    
    uiManager = UIManager(callbacks);
    
    %% Instantiate DataManager
    dataManager = DataManager(uiManager);
    dataManager.dt = dt; % Set the simulation time step

    %% Instantiate Simulation Configuration Classes for Truck 1 and Truck 2
    % Assuming VehicleModel is defined elsewhere and handles vehicle parameters
    vehicleSimConfig1 = VehicleModel(uiManager.vehicleTab1, [], true, 'simulation_Vehicle1', uiManager); % Truck 1 Configuration
    vehicleSimConfig2 = VehicleModel(uiManager.vehicleTab2, [], true, 'simulation_Vehicle2', uiManager); % Truck 2 Configuration

    %% Initialize Default Parameters
    SedanConfig = struct();
    
    % --- Basic Configuration ---
    SedanConfig.includeTrailer = false; % Include trailer by default
    SedanConfig.tractorMass = 2500; % kg 
    SedanConfig.trailerMass = 0; % kg
    SedanConfig.initialVelocity = 10; % m/s
    SedanConfig.W_FrontLeft = 750; % kg
    SedanConfig.W_FrontRight = 750; % kg
    SedanConfig.W_RearLeft = 500; % kg    
    SedanConfig.W_RearRight = 500; % kg

    % --- Advanced Configuration ---
    SedanConfig.I_trailerMultiplier = 1; % Multiplier for inertia
    SedanConfig.maxDeltaDeg = 70; % degrees
    SedanConfig.dtMultiplier = 0.5; % Time step multiplier
    SedanConfig.windowSize = 1; % For moving average

    % --- Tractor Parameters ---
    SedanConfig.tractorLength = 4.5; % meters
    SedanConfig.tractorWidth = 1.8; % meters
    SedanConfig.tractorHeight = 1.4; % meters
    SedanConfig.tractorCoGHeight = 0.5; % meters
    SedanConfig.tractorWheelbase = 2.5; % meters
    SedanConfig.tractorTrackWidth = 1.5; % meters
    SedanConfig.tractorNumAxles = 1;
    SedanConfig.tractorAxleSpacing = 1.310; % meters
    SedanConfig.numTiresPerAxleTractor = 2; % Tires per axle on Tractor

    % --- Trailer Parameters ---
    SedanConfig.trailerLength = 12.0; % meters
    SedanConfig.trailerWidth = 2.5; % meters
    SedanConfig.trailerHeight = 4.0; % meters
    SedanConfig.trailerCoGHeight = 1.5; % meters
    SedanConfig.trailerWheelbase = 8.0; % meters
    SedanConfig.trailerTrackWidth = 2.1; % meters
    SedanConfig.trailerAxlesPerBox = [2];
    SedanConfig.trailerNumAxles = sum(SedanConfig.trailerAxlesPerBox);
    SedanConfig.trailerNumBoxes = numel(SedanConfig.trailerAxlesPerBox);
    SedanConfig.trailerAxleSpacing = 1.310; % meters
    SedanConfig.trailerHitchDistance = 1.310; % meters
    SedanConfig.tractorHitchDistance = 4.5; % meters
    SedanConfig.numTiresPerAxleTrailer = 4; % Tires per axle on Trailer

    % --- Control Limits Parameters ---
    SedanConfig.maxSteeringAngleAtZeroSpeed = 30; % degrees
    SedanConfig.minSteeringAngleAtMaxSpeed = 10;  % degrees
    SedanConfig.maxSteeringSpeed = 30;            % m/s

    % --- Acceleration Limiter Parameters ---
    SedanConfig.maxAccelAtZeroSpeed = 3.0;   % m/s^2
    SedanConfig.minAccelAtMaxSpeed = 1.0;    % m/s^2
    SedanConfig.maxDecelAtZeroSpeed = -3.0;  % m/s^2
    SedanConfig.minDecelAtMaxSpeed = -1.0;   % m/s^2
    SedanConfig.maxSpeedForAccelLimiting = 30.0; % m/s
    % --- End of Acceleration Limiter Parameters ---
    
    % --- PID Speed Controller Parameters ---
    SedanConfig.Kp = 1.0;  % Proportional gain
    SedanConfig.Ki = 0.5;  % Integral gain
    SedanConfig.Kd = 0.1;  % Derivative gain
    % Levant differentiator lambdas for PID derivatives
    SedanConfig.lambda1Accel = 1.0;
    SedanConfig.lambda2Accel = 1.0;
    SedanConfig.lambda1Vel = 1.0;
    SedanConfig.lambda2Vel = 1.0;
    SedanConfig.lambda1Jerk = 1.0;
    SedanConfig.lambda2Jerk = 1.0;
    SedanConfig.enableSpeedController = true;
    % --- End of PID Speed Controller Parameters ---

    SedanConfig.torqueFileName = "sedan_torque_curve.xlsx";
    SedanConfig.accelCurveFilePath = "sedan_AccelCurve.xlsx";
    SedanConfig.decelCurveFilePath = "sedan_DecelCurve.xlsx";
    SedanConfig.steeringCurveFilePath = "sedan_SteeringCurve.xlsx";
    
    % --- Speed Limiting Parameter ---
    SedanConfig.maxSpeed = 25.0; % m/s (Average speed limit for vehicles)
    % --- End of Speed Limiting Parameter ---
    
    % --- Tires Configuration Parameters ---
    SedanConfig.tractorTireHeight = 0.5; % meters
    SedanConfig.tractorTireWidth = 0.2;  % meters
    SedanConfig.trailerTireHeight = 0.5; % meters
    SedanConfig.trailerTireWidth = 0.2;  % meters
    % --- End of Tires Configuration Parameters ---
    
    % --- Aerodynamics Parameters ---
    SedanConfig.airDensity = 1.225;     % kg/m³; standard air density at sea level
    SedanConfig.dragCoeff = 0.8;        % Example drag coefficient; adjust based on vehicle's aerodynamics
    SedanConfig.windSpeed = 5;          % m/s
    SedanConfig.windAngleDeg = 45;      % degrees
    % --- End of Aerodynamics Parameters ---
    
    % --- Engine Configuration Parameters ---
    SedanConfig.maxEngineTorque = 250;           % Maximum Engine Torque (Nm)
    SedanConfig.maxPower = 150000;         % Maximum Power (W)
    SedanConfig.idleRPM = 800;             % Idle RPM
    SedanConfig.redlineRPM = 6500;         % Redline RPM
    SedanConfig.engineBrakeTorque = 400;   % Engine braking torque in Nm
    SedanConfig.fuelConsumptionRate = 0.15; % Fuel Consumption Rate (kg/s)
    % --- End of Engine Configuration Parameters ---
    
    % --- Transmission Configuration Parameters ---
    % Define the gear ratios for 5 gears
    gearRatios = [3.5, 2.1, 1.4, 1.0, 0.8]; % Gear Ratios for 5 Gears
    
    % Create the enumeration for each gear
    numGears = length(gearRatios);
    
    % *** UPDATED PART: Define gearRatios as a table with 'Gear' and 'Ratio' fields ***
    SedanConfig.gearRatios = table((1:numGears)', gearRatios', ...
        'VariableNames', {'Gear', 'Ratio'});
    % *** END OF UPDATED PART ***
    
    SedanConfig.numGears = numGears; % Number of Gears
    SedanConfig.maxGear = numGears;  % Maximum Gear Number
    SedanConfig.finalDriveRatio = 2.8; % Final drive ratio
    SedanConfig.shiftUpSpeed = [10, 20, 30, 45, 60]; % Upshift speeds (in m/s)
    SedanConfig.shiftDownSpeed = [8, 15, 25, 40, 55]; % Downshift speeds (in m/s)
    SedanConfig.shiftDelay = 0.5;        % Shift delay in seconds
    % --- End of Transmission Configuration Parameters ---
    
    % --- Brake Configuration Parameters ---
    SedanConfig.brakingForce = 50000;    % N
    SedanConfig.brakeEfficiency = 85;     % %
    SedanConfig.brakeBias = 50;           % % (Front/Rear)
    SedanConfig.maxBrakingForce = 60000;
    % --- End of Brake Configuration Parameters ---
    
    % --- Road Conditions Parameters ---
    SedanConfig.slopeAngle = 0;             % degrees
    SedanConfig.roadFrictionCoefficient = 0.9;  % μ (default value)
    SedanConfig.roadSurfaceType = 'Dry Asphalt';
    SedanConfig.roadRoughness = 0;          % 0 (smooth) to 1 (very rough)
    % --- End of Road Conditions Parameters ---

    SedanConfig.brakeType = "Disk";
    
    % --- Pressure Matrices Initialization ---
    SedanConfig.pressureMatrices = struct();
    SedanConfig.pressureMatrices.Tires4 = [800000; 800000; 800000; 800000];
    % --- End of Pressure Matrices Initialization ---
    
    % --- Suspension Model Parameters ---
    SedanConfig.K_spring = 30000;   % Spring stiffness (N/m)
    SedanConfig.C_damping = 1500;   % Damping coefficient (N·s/m)
    SedanConfig.restLength = 0.5;    % Rest length (m)
    SedanConfig.stiffnessX = 1e5;    % Stiffness X (N/m)
    SedanConfig.stiffnessY = 1e5;    % Stiffness Y (N/m)
    SedanConfig.stiffnessZ = 1e5;    % Stiffness Z (N/m)
    SedanConfig.stiffnessRoll = 5e4; % Stiffness Roll (N·m/rad)
    SedanConfig.stiffnessPitch = 5e4; % Stiffness Pitch (N·m/rad)
    SedanConfig.stiffnessYaw = 1e5;   % Stiffness Yaw (N·m/rad)
    SedanConfig.dampingX = 5e4;      % Damping X (N·s/m)
    SedanConfig.dampingY = 5e4;      % Damping Y (N·s/m)
    SedanConfig.dampingZ = 5e4;      % Damping Z (N·s/m)
    SedanConfig.dampingRoll = 1e5;   % Damping Roll (N·m·s/rad)
    SedanConfig.dampingPitch = 1e5;  % Damping Pitch (N·m·s/rad)
    SedanConfig.dampingYaw = 2e5;    % Damping Yaw (N·m·s/rad)
    % --- End of Suspension Model Parameters ---

    SedanConfig.maxClutchTorque = 250;           % Maximum torque the clutch can handle (Nm), typical for sedans
    SedanConfig.engagementSpeed = 0.8;           % Time to engage the clutch (seconds), assuming a responsive engagement
    SedanConfig.disengagementSpeed = 0.4;        % Time to disengage the clutch (seconds), assuming faster disengagement

    SedanConfig.steeringCommands = 'simval_(200)';
    SedanConfig.accelerationCommands = 'simval_(200)';
    SedanConfig.tirePressureCommands = 'pressure_(t:1-[tire:2,psi:72];[tire:1,psi:7])';

    SedanConfig.mapCommands = 'straight(100,205,300,205)|curve(300,250,45,270,90,ccw)|straight(300,295,100,295)|curve(100,250,45,90,270,ccw)';
    SedanConfig.waypoints = [
        100, 205        
        105.128, 205    
        110.256, 205    
        115.385, 205    
        120.513, 205    
        125.641, 205    
        130.769, 205    
        135.897, 205    
        141.026, 205    
        146.154, 205    
        151.282, 205    
        156.41, 205     
        161.538, 205    
        166.667, 205    
        171.795, 205    
        176.923, 205    
        182.051, 205    
        187.179, 205    
        192.308, 205    
        197.436, 205    
        202.564, 205    
        207.692, 205    
        212.821, 205    
        217.949, 205    
        223.077, 205    
        228.205, 205    
        233.333, 205    
        238.462, 205    
        243.59, 205     
        248.718, 205    
        253.846, 205    
        258.974, 205    
        264.103, 205    
        269.231, 205    
        274.359, 205    
        279.487, 205    
        284.615, 205    
        289.744, 205    
        294.872, 205    
        300, 205        
        300, 204        
        305.34, 204.311 
        310.608, 205.24 
        315.733, 206.774
        320.645, 208.893
        325.277, 211.568
        329.568, 214.762
        333.459, 218.433
        336.898, 222.531
        339.837, 227    
        342.238, 231.78 
        344.068, 236.807
        345.301, 242.012
        345.922, 247.325
        345.922, 252.675
        345.301, 257.988
        344.068, 263.193
        342.238, 268.22 
        339.837, 273    
        336.898, 277.469
        333.459, 281.567
        329.568, 285.238
        325.277, 288.432
        320.645, 291.107
        315.733, 293.226
        310.608, 294.76 
        305.34, 295.689 
        300, 296        
        300, 295        
        294.872, 295    
        289.744, 295    
        284.615, 295    
        279.487, 295    
        274.359, 295    
        269.231, 295    
        264.103, 295    
        258.974, 295    
        253.846, 295    
        248.718, 295    
        243.59, 295     
        238.462, 295    
        233.333, 295    
        228.205, 295    
        223.077, 295    
        217.949, 295    
        212.821, 295    
        207.692, 295    
        202.564, 295    
        197.436, 295    
        192.308, 295    
        187.179, 295    
        182.051, 295    
        176.923, 295    
        171.795, 295    
        166.667, 295    
        161.538, 295    
        156.41, 295     
        151.282, 295    
        146.154, 295    
        141.026, 295    
        135.897, 295    
        130.769, 295    
        125.641, 295    
        120.513, 295    
        115.385, 295    
        110.256, 295    
        105.128, 295    
        100, 295        
        100, 296        
        94.6597, 295.689
        89.3917, 294.76 
        84.2671, 293.226
        79.3552, 291.107
        74.7226, 288.432
        70.4318, 285.238
        66.5408, 281.567
        63.1023, 277.469
        60.1628, 273    
        57.7621, 268.22 
        55.9325, 263.193
        54.6988, 257.988
        54.0778, 252.675
        54.0778, 247.325
        54.6988, 242.012
        55.9325, 236.807
        57.7621, 231.78 
        60.1628, 227    
        63.1023, 222.531
        66.5408, 218.433
        70.4318, 214.762
        74.7226, 211.568
        79.3552, 208.893
        84.2671, 206.774
        89.3917, 205.24 
        94.6597, 204.311
        100, 204        
    ];
    
    % --- Flat Tire Indices ---
    SedanConfig.flatTireIndices = []; % e.g., [1, 3] to indicate tires 1 and 3 are flat
    % --- End of Flat Tire Indices ---
    
    % --- Wind Angle in Radians ---
    SedanConfig.windAngleRad = deg2rad(SedanConfig.windAngleDeg); % Convert to radians
    
    %% Assign Simulation Parameters to Vehicle Configurations
    % Initialize default parameters for both vehicle configurations
    vehicleSimConfig1 = vehicleSimConfig1.initializeDefaultParameters();
    vehicleSimConfig2 = vehicleSimConfig2.initializeDefaultParameters();
    
    % Assign the SedanConfig to VehicleSimConfig2 (assuming it's Truck 2)
    vehicleSimConfig2.setSimulationParameters(SedanConfig);
    vehicleSimConfig2.guiManager.updatePressureMatrices();

    vehicleSimConfig1.guiManager.generateWaypoints();
    vehicleSimConfig2.guiManager.generateWaypoints();
    
    %% Instantiate Simulation Objects
    vehicleSim1 = VehicleModel(uiManager.vehicleTab1, [], false, 'simulation_Vehicle1', uiManager); % Truck 1 Simulation Object
    vehicleSim2 = VehicleModel(uiManager.vehicleTab2, [], false, 'simulation_Vehicle2', uiManager); % Truck 2 Simulation Object
    
    %% Instantiate ConfigurationManager
    % Assuming ConfigurationManager is defined elsewhere and handles saving/loading configurations
    configManager = ConfigurationManager(uiManager, vehicleSimConfig1, vehicleSimConfig2);
    
    %% Plotting Area Access
    sharedAx = uiManager.sharedAx;

    map = uiManager.laneCommandsField.Value;
    
    boundTypeDropdown = uiManager.boundTypeDropdown;
    AverageVehicle1Mass = uiManager.vehicle1MassField;
    AverageVehicle2Mass = uiManager.vehicle2MassField;
    
    %% Instantiate CollisionDetector Class
    % Assuming CollisionDetector is defined elsewhere and handles collision detection logic
    collisionDetector = CollisionDetector();
    
    %% Instantiate PlotManager
    % Assuming PlotManager is updated to handle two vehicles and their trailers
    plotManager = PlotManager(sharedAx, uiManager, collisionDetector);

    %% Instantiate SimManager
    % Assuming SimManager is updated to handle two vehicles and their trailers
    simulationManager = SimManager(map, dataManager, plotManager, collisionDetector, ...
                                    vehicleSimConfig1, vehicleSimConfig2, ...
                                    vehicleSim1, vehicleSim2, ...
                                    uiManager, configManager, dt);
    
    %% Define Callback Functions
    
    %/**
    % * @brief Loads Excel data for Truck 1 Simulation.
    % *
    % * This nested function delegates the loading of Truck 1 simulation data to the `DataManager`.
    % *
    % * @return None
    % */
    function loadVehicle1Excel()
        dataManager.loadVehicle1Excel(vehicleSimConfig1, vehicleSim1);
    end
    
    %/**
    % * @brief Loads Excel data for Truck 2 Simulation.
    % *
    % * This nested function delegates the loading of Truck 2 simulation data to the `DataManager`.
    % *
    % * @return None
    % */
    function loadVehicle2Excel()
        dataManager.loadVehicle2Excel(vehicleSimConfig2, vehicleSim2);
    end
    
    %/**
    % * @brief Saves Manual Control Data to Excel.
    % *
    % * This nested function delegates the saving of manual control data to the `DataManager`.
    % *
    % * @return None
    % */
    function saveManualControlData()
        dataManager.saveManualControlData();
    end
    
    %/**
    % * @brief Saves the current plot as a PNG image.
    % *
    % * This nested function delegates the saving of the current plot to the `DataManager`.
    % *
    % * @return None
    % */
    function savePlotAsPNG()
        dataManager.savePlotAsPNG(sharedAx);
    end
    
    %/**
    % * @brief Saves Simulation Results to Excel.
    % *
    % * This nested function delegates the saving of simulation results to the `DataManager`.
    % *
    % * @return None
    % */
    function saveSimulationResults()
        dataManager.saveSimulationResults();
    end
    
    %/**
    % * @brief Saves the current simulation configuration.
    % *
    % * This nested function delegates the saving of the simulation configuration to the `ConfigurationManager`.
    % *
    % * @return None
    % */
    function saveConfiguration()
        configManager.saveConfiguration();
    end

    %/**
    % * @brief Loads a saved simulation configuration.
    % *
    % * This nested function delegates the loading of a simulation configuration to the `ConfigurationManager`.
    % *
    % * @return None
    % */
    function loadConfiguration()
        configManager.loadConfiguration();
    end
    
    %/**
    % * @brief Starts the simulation by invoking the `SimManager`.
    % *
    % * This nested function delegates the simulation execution to the `SimManager`.
    % *
    % * @return None
    % */
    function startSimulation()
        simulationManager.runSimulations();
        try
            gw = GraphicsWindow(World3D(), true, true);
            tractorParams1 = simulationManager.createVehicleParams(vehicleSim1.simParams, ...
                vehicleSim1.simParams.tractorTireHeight, vehicleSim1.simParams.tractorTireWidth);
            if vehicleSim1.simParams.includeTrailer
                trailerParams1 = simulationManager.createTrailerParams(vehicleSim1.simParams, ...
                    vehicleSim1.simParams.trailerTireHeight, vehicleSim1.simParams.trailerTireWidth, 1);
            else
                trailerParams1 = [];
            end
            truck = Truck3D(tractorParams1, trailerParams1, true, true);
            animator = Sim3DAnimator(dataManager, gw, truck, tractorParams1, trailerParams1);
            animator.run();
        catch ME
            disp(['3D Animation failed: ' ME.message]);
        end
    end
    
    %/**
    % * @brief Loads saved simulation data from a .mat file and animates without rerunning.
    % */
    function loadSimulationData()
        [file, path] = uigetfile('*.mat', 'Load Simulation Data');
        if isequal(file,0) || isequal(path,0)
            disp('User canceled loading simulation data.');
            return;
        end
        S = load(fullfile(path, file));
        if isfield(S, 'simData')
            simData = S.simData;
            % Restore simulation data for playback
            dataManager.globalVehicle1Data = simData.globalVehicle1Data;
            dataManager.globalTrailer1Data = simData.globalTrailer1Data;
            dataManager.globalVehicle2Data = simData.globalVehicle2Data;
            dataManager.globalTrailer2Data = simData.globalTrailer2Data;
            dataManager.collisionData = simData.collisionData;
            % Restore simulation parameters and time step
            try
                vehicleSim1.simParams = simData.simParams1;
                vehicleSim2.simParams = simData.simParams2;
            catch
            end
            try
                simulationManager.dt = simData.dt;
                dataManager.dt = simData.dt;
            catch
            end
            % Animate using existing simulation manager (preserves placement), start paused
            uiManager.pauseFlag = true;
            uiManager.stopFlag  = false;
            simulationManager.useSavedData = true;
            simulationManager.runSimulations();
            simulationManager.useSavedData = false;
        else
            uialert(uiManager.fig, 'Selected file does not contain valid simulation data.', 'Load Error');
        end
    end
    
    %/**
    % * @brief Saves current simulation data to a .mat file.
    % */
    function saveSimulationData()
        % Collect simulation data for saving and playback
        simData.globalVehicle1Data = dataManager.globalVehicle1Data;
        simData.globalTrailer1Data = dataManager.globalTrailer1Data;
        simData.globalVehicle2Data = dataManager.globalVehicle2Data;
        simData.globalTrailer2Data = dataManager.globalTrailer2Data;
        simData.collisionData = dataManager.collisionData;
        % Save simulation parameters and time step
        try
            simData.simParams1 = vehicleSim1.simParams;
            simData.simParams2 = vehicleSim2.simParams;
        catch
        end
        simData.dt = simulationManager.dt;
        % Save initial offsets and rotations for playback alignment
        try
            simData.offset1.x = uiManager.getvehicle1OffsetX();
            simData.offset1.y = uiManager.getvehicle1OffsetY();
            simData.offset1.rotate = uiManager.getRotateVehicle1();
            simData.offset1.theta = uiManager.getRotationAngleVehicle1();
            simData.offset2.x = uiManager.getOffsetX();
            simData.offset2.y = uiManager.getOffsetY();
            simData.offset2.rotate = uiManager.getRotateVehicle2();
            simData.offset2.theta = uiManager.getRotationAngleVehicle2();
        catch
            % UI offsets unavailable
        end
        [file, path] = uiputfile('*.mat', 'Save Simulation Data');
        if isequal(file,0) || isequal(path,0)
            disp('User canceled saving simulation data.');
            return;
        end
        save(fullfile(path,file), 'simData');
        uialert(uiManager.fig, 'Simulation data saved successfully.', 'Save Complete');
    end
    
end
