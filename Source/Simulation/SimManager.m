% Author: Miguel Marina <karel.capek.robotics@gmail.com>
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
% * @class SimManager
% * @brief Handles running simulations, detecting collisions, and plotting results for Vehicles in parallel.
% * @see PARALLEL_COMPUTING.md for detailed strategies on implementing parallel execution and performance improvements.
% *
% * This revised version launches collision detection in parallel to animating the trajectories.
% *
% * @version 2.6
% * @date 2024-10-31
% */
classdef SimManager < handle
    properties
        map
        % Simulation Components
        dataManager
        plotManager
        collisionDetector

        % Vehicle Simulations and Configurations
        vehicleSim1
        vehicleSim2
        vehicleSimConfig1
        vehicleSimConfig2

        % Managers
        uiManager
        configManager

        % Simulation Parameters
        dt

        % Collision point coordinates
        collisionX
        collisionY

        % Storage for simulation results
        sim1Results
        sim2Results
        % Flag indicating whether to use loaded simulation data
        useSavedData = false;
        % Placeholder for loaded data (if any)
        savedData;
    end

    methods
        function obj = SimManager(map, dataManager, plotManager, collisionDetector, ...
                                   vehicleSimConfig1, vehicleSimConfig2, ...
                                   vehicleSim1, vehicleSim2, ...
                                   uiManager, configManager, dt)
            obj.map = map;
            obj.dataManager = dataManager;
            obj.plotManager = plotManager;
            obj.collisionDetector = collisionDetector;
            obj.vehicleSimConfig1 = vehicleSimConfig1;
            obj.vehicleSimConfig2 = vehicleSimConfig2;
            obj.vehicleSim1 = vehicleSim1;
            obj.vehicleSim2 = vehicleSim2;
            obj.uiManager = uiManager;
            obj.configManager = configManager;
            obj.dt = dt;

            % Initialize collision point coordinates
            obj.collisionX = NaN;
            obj.collisionY = NaN;
        end

        %/**
        % * @brief Executes the simulation workflow, launches collision checking in parallel,
        % *        and animates the vehicle trajectories.
        % */
        function runSimulations(obj)
            try
            %% 1. Prepare parallel pool and clear previous plots
            % Ensure a parallel pool is available for concurrent execution and
            % close it once this method finishes.
            pool = gcp('nocreate');
            poolCreated = false;
            if isempty(pool)
                pool = parpool('local');
                poolCreated = true;
            end
            poolCleanup = onCleanup(@() closeLocalPool(poolCreated));
                disp('Clearing previous plots...');
                obj.plotManager.clearPlots();

                %% 2. Update Simulation Parameters for Both Vehicles
                disp('Updating simulation parameters for Vehicle 1...');
                obj.vehicleSim1.simParams = obj.vehicleSimConfig1.getSimulationParameters();
                disp('Updating simulation parameters for Vehicle 2...');
                obj.vehicleSim2.simParams = obj.vehicleSimConfig2.getSimulationParameters();

                %% Override command parameters with values from the UIManager command tabs
                if isprop(obj.uiManager, 'vehicle1SteeringCommandsField') && ...
                   isprop(obj.uiManager, 'vehicle1AccelerationCommandsField') && ...
                   isprop(obj.uiManager, 'vehicle1TirePressureCommandsField')
                    obj.vehicleSim1.simParams.steeringCommands = obj.uiManager.vehicle1SteeringCommandsField.Value;
                    obj.vehicleSim1.simParams.accelerationCommands = obj.uiManager.vehicle1AccelerationCommandsField.Value;
                    obj.vehicleSim1.simParams.tirePressureCommands = obj.uiManager.vehicle1TirePressureCommandsField.Value;
                elseif isprop(obj.uiManager, 'steeringCommandsField') && ...
                       isprop(obj.uiManager, 'accelerationCommandsField') && ...
                       isprop(obj.uiManager, 'tirePressureCommandsField')
                    obj.vehicleSim1.simParams.steeringCommands = obj.uiManager.steeringCommandsField.Value;
                    obj.vehicleSim1.simParams.accelerationCommands = obj.uiManager.accelerationCommandsField.Value;
                    obj.vehicleSim1.simParams.tirePressureCommands = obj.uiManager.tirePressureCommandsField.Value;
                end

                if isprop(obj.uiManager, 'vehicle2SteeringCommandsField') && ...
                   isprop(obj.uiManager, 'vehicle2AccelerationCommandsField') && ...
                   isprop(obj.uiManager, 'vehicle2TirePressureCommandsField')
                    obj.vehicleSim2.simParams.steeringCommands = obj.uiManager.vehicle2SteeringCommandsField.Value;
                    obj.vehicleSim2.simParams.accelerationCommands = obj.uiManager.vehicle2AccelerationCommandsField.Value;
                    obj.vehicleSim2.simParams.tirePressureCommands = obj.uiManager.vehicle2TirePressureCommandsField.Value;
                end

                %% Override command parameters with values from the UIManager command tabs
                if isprop(obj.uiManager, 'vehicle1SteeringCommandsField') && ...
                   isprop(obj.uiManager, 'vehicle1AccelerationCommandsField') && ...
                   isprop(obj.uiManager, 'vehicle1TirePressureCommandsField')
                    obj.vehicleSim1.simParams.steeringCommands = obj.uiManager.vehicle1SteeringCommandsField.Value;
                    obj.vehicleSim1.simParams.accelerationCommands = obj.uiManager.vehicle1AccelerationCommandsField.Value;
                    obj.vehicleSim1.simParams.tirePressureCommands = obj.uiManager.vehicle1TirePressureCommandsField.Value;
                elseif isprop(obj.uiManager, 'steeringCommandsField') && ...
                       isprop(obj.uiManager, 'accelerationCommandsField') && ...
                       isprop(obj.uiManager, 'tirePressureCommandsField')
                    obj.vehicleSim1.simParams.steeringCommands = obj.uiManager.steeringCommandsField.Value;
                    obj.vehicleSim1.simParams.accelerationCommands = obj.uiManager.accelerationCommandsField.Value;
                    obj.vehicleSim1.simParams.tirePressureCommands = obj.uiManager.tirePressureCommandsField.Value;
                end

                if isprop(obj.uiManager, 'vehicle2SteeringCommandsField') && ...
                   isprop(obj.uiManager, 'vehicle2AccelerationCommandsField') && ...
                   isprop(obj.uiManager, 'vehicle2TirePressureCommandsField')
                    obj.vehicleSim2.simParams.steeringCommands = obj.uiManager.vehicle2SteeringCommandsField.Value;
                    obj.vehicleSim2.simParams.accelerationCommands = obj.uiManager.vehicle2AccelerationCommandsField.Value;
                    obj.vehicleSim2.simParams.tirePressureCommands = obj.uiManager.vehicle2TirePressureCommandsField.Value;
                end

                % Set vehicleType fields dynamically
                obj.vehicleSim1.simParams.vehicleType = 'Truck';
                obj.vehicleSim2.simParams.vehicleType = 'Tractor';

                %% 3. Retrieve Offsets and Rotation Settings
                disp('Retrieving offsets and rotation settings...');
                offsetX = obj.uiManager.getOffsetX();
                offsetY = obj.uiManager.getOffsetY();
                vehicle1OffsetX = obj.uiManager.getvehicle1OffsetX();
                vehicle1OffsetY = obj.uiManager.getvehicle1OffsetY();
                rotateVehicle2 = obj.uiManager.getRotateVehicle2();
                rotateVehicle1 = obj.uiManager.getRotateVehicle1();
                rotationAngleDeg = obj.uiManager.getRotationAngleVehicle2(); % Degrees
                vehicle1rotationAngleDeg = obj.uiManager.getRotationAngleVehicle1(); % Degrees
                rotationAngleRad = deg2rad(-rotationAngleDeg); % Radians for Vehicle 2
                vehicle1rotationAngleRad = deg2rad(-vehicle1rotationAngleDeg); % Radians for Vehicle 1

                %% 4. Validate Offsets
                disp('Validating offsets...');
                if isempty(offsetX) || isempty(offsetY) || ~isnumeric(offsetX) || ~isnumeric(offsetY)
                    uialert(obj.uiManager.fig, 'Please enter valid numeric offset values for the Tractor.', 'Input Error');
                    return;
                end

                %% 5. Retrieve Wheel Dimensions
                disp('Retrieving wheel dimensions...');
                wheelHeightTruck1 = obj.vehicleSim1.simParams.tractorTireHeight;
                wheelWidthTruck1  = obj.vehicleSim1.simParams.tractorTireWidth;

                if obj.vehicleSim1.simParams.includeTrailer
                    wheelHeightTrailer1 = obj.vehicleSim1.simParams.trailerTireHeight;
                    wheelWidthTrailer1  = obj.vehicleSim1.simParams.trailerTireWidth;
                else
                    wheelHeightTrailer1 = [];
                    wheelWidthTrailer1  = [];
                end

                wheelHeightTractor2 = obj.vehicleSim2.simParams.tractorTireHeight;
                wheelWidthTractor2  = obj.vehicleSim2.simParams.tractorTireWidth;

                if obj.vehicleSim2.simParams.includeTrailer
                    wheelHeightTrailer2 = obj.vehicleSim2.simParams.trailerTireHeight;
                    wheelWidthTrailer2  = obj.vehicleSim2.simParams.trailerTireWidth;
                else
                    wheelHeightTrailer2 = [];
                    wheelWidthTrailer2  = [];
                end

                %% 6. Validate Wheel Dimensions
                disp('Validating wheel dimensions...');
                if isempty(wheelHeightTruck1) || wheelHeightTruck1 <= 0 || ...
                   isempty(wheelWidthTruck1)  || wheelWidthTruck1 <= 0
                    uialert(obj.uiManager.fig, 'Please ensure Tractor Wheel Height and Width are positive values in simulation parameters for Vehicle 1.', 'Input Error');
                    return;
                end

                if ~isempty(wheelHeightTrailer1) && (~isnumeric(wheelHeightTrailer1) || wheelHeightTrailer1 <= 0 || ...
                   ~isnumeric(wheelWidthTrailer1)  || wheelWidthTrailer1 <= 0)
                    uialert(obj.uiManager.fig, 'Please ensure Trailer Wheel Height and Width are positive values in simulation parameters for Vehicle 1.', 'Input Error');
                    return;
                end

                if isempty(wheelHeightTractor2) || wheelHeightTractor2 <= 0 || ...
                   isempty(wheelWidthTractor2)  || wheelWidthTractor2 <= 0
                    uialert(obj.uiManager.fig, 'Please ensure Tractor Wheel Height and Width are positive values in simulation parameters for Vehicle 2.', 'Input Error');
                    return;
                end

                if ~isempty(wheelHeightTrailer2) && (~isnumeric(wheelHeightTrailer2) || wheelHeightTrailer2 <= 0 || ...
                   ~isnumeric(wheelWidthTrailer2)  || wheelWidthTrailer2 <= 0)
                    uialert(obj.uiManager.fig, 'Please ensure Trailer Wheel Height and Width are positive values in simulation parameters for Vehicle 2.', 'Input Error');
                    return;
                end

                %% 7. Initialize the Progress Bar
                disp('Initializing progress bar...');
                hWaitbar = waitbar(0.15, 'Starting Simulations...', 'Name', 'Simulation Progress');
                cleanupObj = onCleanup(@() closeIfOpen(hWaitbar));

                %% 8. Instantiate Vehicle and Trailer Parameters for Plotting (for real-time animation)
                disp('Instantiating vehicle parameters for plotting...');
                vehicleParams1 = obj.createVehicleParams(obj.vehicleSim1.simParams, wheelHeightTruck1, wheelWidthTruck1);
                if obj.vehicleSim1.simParams.includeTrailer
                    trailerParams1 = obj.createTrailerParams(obj.vehicleSim1.simParams, wheelHeightTrailer1, wheelWidthTrailer1, 1);
                else
                    trailerParams1 = [];
                end
                vehicleParams2 = obj.createVehicleParams(obj.vehicleSim2.simParams, wheelHeightTractor2, wheelWidthTractor2);
                if obj.vehicleSim2.simParams.includeTrailer
                    trailerParams2 = obj.createTrailerParams(obj.vehicleSim2.simParams, wheelHeightTrailer2, wheelWidthTrailer2, 2);
                else
                    trailerParams2 = [];
                end

                %% Retrieve and validate mass values for collision detection
                disp('Retrieving and validating mass values for real-time animation...');
                VehicleUnderAnalysisMaxMass = obj.uiManager.getVehicle1Mass();
                J2980AssumedMaxMass        = obj.uiManager.getVehicle2Mass();
                if ~isnumeric(VehicleUnderAnalysisMaxMass) || VehicleUnderAnalysisMaxMass <= 0
                    uialert(obj.uiManager.fig, 'Invalid Tractor Vehicle Mass entered. Please enter a positive numeric value.', 'Input Error');
                    return;
                end
                if ~isnumeric(J2980AssumedMaxMass) || J2980AssumedMaxMass <= 0
                    uialert(obj.uiManager.fig, 'Invalid Vehicle Under Analysis Mass entered. Please enter a positive numeric value.', 'Input Error');
                    return;
                end

                %% Determine total steps for real-time animation
                totalSteps = length(obj.vehicleSim1.simParams.steeringCommands);
                fprintf('Total Simulation Steps set to: %d\n', totalSteps);

                %% 9. Run Vehicle Simulations (or use loaded data)
                if obj.useSavedData
                    disp('Using loaded simulation data, skipping simulation runs and transfer.');
                else
                    v1 = obj.vehicleSim1.initializeSim();
                    v2 = obj.vehicleSim2.initializeSim();
                    % if isappdata(0, 'SuppressDebug') && getappdata(0, 'SuppressDebug')
                    %     disp('Running simulations asynchronously...');
                    %     futures = cell(1, 2);
                    %     totalSteps = v1.numSteps;
                    % 
                    %     hWaitbar = waitbar(0, 'Starting Simulation...', 'Name', 'Simulation Progress');
                    %     cleanupObj = onCleanup(@() closeIfOpen(hWaitbar)); % Ensure waitbar closes on function exit
                    %     reportInterval = ceil(totalSteps / 100); % Update every 1%
                    % 
                    %     for iStep = 1:totalSteps
                    %         if mod(iStep, reportInterval) == 0 || iStep == totalSteps
                    %             progress = (iStep / totalSteps) * 100;
                    %             waitbar(iStep / totalSteps, hWaitbar, sprintf('Simulation Progress: %.2f%%', progress));
                    %         end
                    % 
                    %         futures{1} = parfeval(@SimManager.runVehicleSim1, 1, obj.vehicleSim1,v1,iStep);
                    %         futures{2} = parfeval(@SimManager.runVehicleSim2, 1, obj.vehicleSim2,v2,iStep);
                    %         simResultsArray = cell(1, 2);
                    % 
                    %         for idx = 1:2
                    %             future = futures{idx};
                    %             disp(['Fetching outputs for simulation ', num2str(idx), '...']);
                    %             result = fetchOutputs(future);
                    %             simResultsArray{idx} = result;
                    %         end
                    %     end
                    % 
                    %     obj.sim1Results = simResultsArray{1};
                    %     obj.sim2Results = simResultsArray{2};
                    % else
                        % totalSteps = v1.numSteps;
                        % for iStep = 1:totalSteps
                            % progress = (iStep / totalSteps) * 100;
                            % waitbar(iStep / totalSteps, hWaitbar, sprintf('Simulation Progress: %.2f%%', progress));
                            v1 = SimManager.runVehicleSim1(obj.vehicleSim1,v1);
                            v2 = SimManager.runVehicleSim2(obj.vehicleSim2,v2);
                        % end
                    % end

                    % v1 = obj.vehicleSim1.initializeSim();
                    % v2 = obj.vehicleSim2.initializeSim();
                    % 
                    % disp('Running simulations asynchronously...');
                    % 

                    res1 = obj.vehicleSim1.closeSim(v1);
                    res2 = obj.vehicleSim2.closeSim(v2);
                    % Ensure box orientation arrays exist even if not multi-box
                    if ~isfield(v1, 'trailerXBoxes')
                        v1.trailerXBoxes = [];
                        v1.trailerYBoxes = [];
                    end
                    if ~isfield(v2, 'trailerXBoxes')
                        v2.trailerXBoxes = [];
                        v2.trailerYBoxes = [];
                    end

                    % Compute trailer box positions for multi-box configuration
                    if obj.vehicleSim1.simParams.includeTrailer && isfield(obj.vehicleSim1.simParams, 'trailerNumBoxes')
                        nBoxes1  = obj.vehicleSim1.simParams.trailerNumBoxes;
                        spacing1 = obj.vehicleSim1.simParams.trailerBoxSpacing;
                        boxLen1  = obj.vehicleSim1.simParams.trailerLength;
                        T1       = length(v1.trailerX);
                        theta1   = v1.trailerTheta(:)';
                        % Prepare per-box orientations and compute absolute yaw for each box
                        % Use Box orientations from simulation (absolute yaw) for chaining
                        absThetas1 = v1.trailerThetaBoxes;  % nBoxes1 x T1
                        % Preallocate box COM arrays
                        v1.trailerXBoxes = zeros(nBoxes1, T1);
                        v1.trailerYBoxes = zeros(nBoxes1, T1);
                        % First box COM is the trailer hitch position
                        v1.trailerXBoxes(1,:) = v1.trailerX(:)';
                        v1.trailerYBoxes(1,:) = v1.trailerY(:)';
                        % Distance between box centers: full box length + spacing
                        offset = boxLen1 + spacing1;
                        % Subsequent box COMs offset along absolute orientation of previous box
                        for bi = 2:nBoxes1
                            v1.trailerXBoxes(bi, :) = v1.trailerXBoxes(bi-1, :) - offset .* cos(absThetas1(bi-1, :));
                            v1.trailerYBoxes(bi, :) = v1.trailerYBoxes(bi-1, :) - offset .* sin(absThetas1(bi-1, :));
                        end
                    end
                    if obj.vehicleSim2.simParams.includeTrailer && isfield(obj.vehicleSim2.simParams, 'trailerNumBoxes')
                        nBoxes2  = obj.vehicleSim2.simParams.trailerNumBoxes;
                        spacing2 = obj.vehicleSim2.simParams.trailerBoxSpacing;
                        boxLen2  = obj.vehicleSim2.simParams.trailerLength;
                        T2       = length(v2.trailerX);
                        theta2   = v2.trailerTheta(:)';
                        % Prepare per-box orientations and compute absolute yaw for each box
                        % Get absolute yaw for each box directly from simulation output
                        absThetas2 = v2.trailerThetaBoxes;  % already nBoxes2 x T2
                        % Preallocate box COM arrays
                        v2.trailerXBoxes = zeros(nBoxes2, T2);
                        v2.trailerYBoxes = zeros(nBoxes2, T2);
                        % First box COM is the trailer hitch position
                        v2.trailerXBoxes(1,:) = v2.trailerX(:)';
                        v2.trailerYBoxes(1,:) = v2.trailerY(:)';
                        % Distance between box centers: full box length + spacing
                        offset2 = boxLen2 + spacing2;
                        % Subsequent box COMs offset along absolute orientation of previous box
                        for bi = 2:nBoxes2
                            v2.trailerXBoxes(bi, :) = v2.trailerXBoxes(bi-1, :) - offset2 .* cos(absThetas2(bi-1, :));
                            v2.trailerYBoxes(bi, :) = v2.trailerYBoxes(bi-1, :) - offset2 .* sin(absThetas2(bi-1, :));
                        end
                    end

                    % Ensure trailerThetaBoxes field is present to avoid missing field errors
                    if ~isfield(v1, 'trailerThetaBoxes')
                        v1.trailerThetaBoxes = [];
                    end
                    if ~isfield(v2, 'trailerThetaBoxes')
                        v2.trailerThetaBoxes = [];
                    end
                    % Package simulation results for Vehicle 1
                    fields1 = {'X','Y','Theta','trailerX','trailerY','trailerTheta','flags','steeringAngles','speedData'};
                    vals1   = {v1.tractorX, v1.tractorY, v1.tractorTheta, v1.trailerX, v1.trailerY, v1.trailerTheta, v1.globalVehicleFlags, v1.steeringAnglesSim, res1};
                    % Include per-box data if present
                    if ~isempty(v1.trailerXBoxes)
                        fields1 = [fields1 {'trailerXBoxes','trailerYBoxes','trailerThetaBoxes'}];
                        vals1   = [vals1   {v1.trailerXBoxes, v1.trailerYBoxes, v1.trailerThetaBoxes}];
                    end
                    obj.sim1Results = cell2struct(vals1, fields1, 2);

                    % Package simulation results for Vehicle 2
                    fields2 = {'X','Y','Theta','trailerX','trailerY','trailerTheta','flags','steeringAngles','speedData'};
                    vals2   = {v2.tractorX, v2.tractorY, v2.tractorTheta, v2.trailerX, v2.trailerY, v2.trailerTheta, v2.globalVehicleFlags, v2.steeringAnglesSim, res2};
                    if ~isempty(v2.trailerXBoxes)
                        fields2 = [fields2 {'trailerXBoxes','trailerYBoxes','trailerThetaBoxes'}];
                        vals2   = [vals2   {v2.trailerXBoxes, v2.trailerYBoxes, v2.trailerThetaBoxes}];
                    end
                    obj.sim2Results = cell2struct(vals2, fields2, 2);

                    %% 9. Transfer Simulation Results
                    disp('Transferring simulation results to DataManager...');
                    obj.transferSimulationResults();
                end

                %% 10. Equalize Simulation Data Lengths
                disp('Equalizing simulation data lengths...');
                sim1Length = length(obj.dataManager.globalVehicle1Data.X);
                sim2Length = length(obj.dataManager.globalVehicle2Data.X);
                maxLength  = max(sim1Length, sim2Length);

                obj.dataManager.globalVehicle1Data.X = extendData(obj.dataManager.globalVehicle1Data.X, maxLength);
                obj.dataManager.globalVehicle1Data.Y = extendData(obj.dataManager.globalVehicle1Data.Y, maxLength);
                obj.dataManager.globalVehicle1Data.Theta = extendData(obj.dataManager.globalVehicle1Data.Theta, maxLength);
                obj.dataManager.globalVehicle1Data.Speed = extendData(obj.dataManager.globalVehicle1Data.Speed, maxLength);
                obj.dataManager.globalVehicle1Data.SteeringAngle = extendData(obj.dataManager.globalVehicle1Data.SteeringAngle, maxLength);

                if obj.vehicleSim1.simParams.includeTrailer
                    obj.dataManager.globalTrailer1Data.X = extendData(obj.dataManager.globalTrailer1Data.X, maxLength);
                    obj.dataManager.globalTrailer1Data.Y = extendData(obj.dataManager.globalTrailer1Data.Y, maxLength);
                    obj.dataManager.globalTrailer1Data.Theta = extendData(obj.dataManager.globalTrailer1Data.Theta, maxLength);
                    obj.dataManager.globalTrailer1Data.SteeringAngle = extendData(obj.dataManager.globalTrailer1Data.SteeringAngle, maxLength);
                    if isfield(obj.dataManager.globalTrailer1Data, 'XBoxes') && ~isempty(obj.dataManager.globalTrailer1Data.XBoxes)
                        obj.dataManager.globalTrailer1Data.XBoxes = extendData(obj.dataManager.globalTrailer1Data.XBoxes, maxLength);
                        obj.dataManager.globalTrailer1Data.YBoxes = extendData(obj.dataManager.globalTrailer1Data.YBoxes, maxLength);
                        obj.dataManager.globalTrailer1Data.ThetaBoxes = extendData(obj.dataManager.globalTrailer1Data.ThetaBoxes, maxLength);
                    end
                    % Extend each trailer box data to match the simulation length
                    if isfield(obj.dataManager.globalTrailer1Data, 'Boxes') && ~isempty(obj.dataManager.globalTrailer1Data.Boxes)
                        nBoxes1 = numel(obj.dataManager.globalTrailer1Data.Boxes);
                        for bi = 1:nBoxes1
                            obj.dataManager.globalTrailer1Data.Boxes(bi).X     = extendData(obj.dataManager.globalTrailer1Data.Boxes(bi).X, maxLength);
                            obj.dataManager.globalTrailer1Data.Boxes(bi).Y     = extendData(obj.dataManager.globalTrailer1Data.Boxes(bi).Y, maxLength);
                            obj.dataManager.globalTrailer1Data.Boxes(bi).Theta = extendData(obj.dataManager.globalTrailer1Data.Boxes(bi).Theta, maxLength);
                        end
                    end
                end

                obj.dataManager.globalVehicle2Data.X = extendData(obj.dataManager.globalVehicle2Data.X, maxLength);
                obj.dataManager.globalVehicle2Data.Y = extendData(obj.dataManager.globalVehicle2Data.Y, maxLength);
                obj.dataManager.globalVehicle2Data.Theta = extendData(obj.dataManager.globalVehicle2Data.Theta, maxLength);
                obj.dataManager.globalVehicle2Data.Speed = extendData(obj.dataManager.globalVehicle2Data.Speed, maxLength);
                obj.dataManager.globalVehicle2Data.SteeringAngle = extendData(obj.dataManager.globalVehicle2Data.SteeringAngle, maxLength);

                if obj.vehicleSim2.simParams.includeTrailer
                    obj.dataManager.globalTrailer2Data.X = extendData(obj.dataManager.globalTrailer2Data.X, maxLength);
                    obj.dataManager.globalTrailer2Data.Y = extendData(obj.dataManager.globalTrailer2Data.Y, maxLength);
                    obj.dataManager.globalTrailer2Data.Theta = extendData(obj.dataManager.globalTrailer2Data.Theta, maxLength);
                    obj.dataManager.globalTrailer2Data.SteeringAngle = extendData(obj.dataManager.globalTrailer2Data.SteeringAngle, maxLength);
                    if isfield(obj.dataManager.globalTrailer2Data, 'XBoxes') && ~isempty(obj.dataManager.globalTrailer2Data.XBoxes)
                        obj.dataManager.globalTrailer2Data.XBoxes = extendData(obj.dataManager.globalTrailer2Data.XBoxes, maxLength);
                        obj.dataManager.globalTrailer2Data.YBoxes = extendData(obj.dataManager.globalTrailer2Data.YBoxes, maxLength);
                        obj.dataManager.globalTrailer2Data.ThetaBoxes = extendData(obj.dataManager.globalTrailer2Data.ThetaBoxes, maxLength);
                    end
                    % Extend each trailer box data to match the simulation length
                    if isfield(obj.dataManager.globalTrailer2Data, 'Boxes') && ~isempty(obj.dataManager.globalTrailer2Data.Boxes)
                        nBoxes2 = numel(obj.dataManager.globalTrailer2Data.Boxes);
                        for bi = 1:nBoxes2
                            obj.dataManager.globalTrailer2Data.Boxes(bi).X     = extendData(obj.dataManager.globalTrailer2Data.Boxes(bi).X, maxLength);
                            obj.dataManager.globalTrailer2Data.Boxes(bi).Y     = extendData(obj.dataManager.globalTrailer2Data.Boxes(bi).Y, maxLength);
                            obj.dataManager.globalTrailer2Data.Boxes(bi).Theta = extendData(obj.dataManager.globalTrailer2Data.Boxes(bi).Theta, maxLength);
                        end
                    end
                end

                % Apply initial offsets and rotations to trajectory coordinates
                if ~obj.useSavedData
                % Vehicle 1 transform
                try
                    ox1 = obj.uiManager.getvehicle1OffsetX();
                    oy1 = obj.uiManager.getvehicle1OffsetY();
                    ang1 = deg2rad(obj.uiManager.getRotationAngleVehicle1());
                    if ~obj.uiManager.getRotateVehicle1(), ang1 = 0; end
                    R1 = [cos(ang1), -sin(ang1); sin(ang1), cos(ang1)];
                    pts1 = R1 * [obj.dataManager.globalVehicle1Data.X(:)'; obj.dataManager.globalVehicle1Data.Y(:)'];
                    obj.dataManager.globalVehicle1Data.X = pts1(1, :)' + ox1;
                    obj.dataManager.globalVehicle1Data.Y = pts1(2, :)' + oy1;
                    if obj.vehicleSim1.simParams.includeTrailer
                        % Transform primary trailer path
                        t1 = R1 * [obj.dataManager.globalTrailer1Data.X(:)'; obj.dataManager.globalTrailer1Data.Y(:)'];
                        obj.dataManager.globalTrailer1Data.X = t1(1, :)' + ox1;
                        obj.dataManager.globalTrailer1Data.Y = t1(2, :)' + oy1;
                        % Transform each additional trailer box
                        if isfield(obj.dataManager.globalTrailer1Data, 'Boxes')
                            for bi = 1:numel(obj.dataManager.globalTrailer1Data.Boxes)
                                Bx = obj.dataManager.globalTrailer1Data.Boxes(bi).X;
                                By = obj.dataManager.globalTrailer1Data.Boxes(bi).Y;
                                Bt = obj.dataManager.globalTrailer1Data.Boxes(bi).Theta;
                                Btrans = R1 * [Bx'; By'];
                                obj.dataManager.globalTrailer1Data.Boxes(bi).X     = Btrans(1, :)' + ox1;
                                obj.dataManager.globalTrailer1Data.Boxes(bi).Y     = Btrans(2, :)' + oy1;
                                obj.dataManager.globalTrailer1Data.Boxes(bi).Theta = mod(Bt + ang1 + pi, 2*pi) - pi;
                                if isfield(obj.dataManager.globalTrailer1Data, 'XBoxes') && ~isempty(obj.dataManager.globalTrailer1Data.XBoxes)
                                    obj.dataManager.globalTrailer1Data.XBoxes(bi,:) = Btrans(1, :) + ox1;
                                    obj.dataManager.globalTrailer1Data.YBoxes(bi,:) = Btrans(2, :) + oy1;
                                    obj.dataManager.globalTrailer1Data.ThetaBoxes(bi,:) = mod(Bt + ang1 + pi, 2*pi) - pi;
                                end
                            end
                        end
                    end
                catch
                end
                % Vehicle 2 transform
                try
                    ox2 = obj.uiManager.getOffsetX();
                    oy2 = obj.uiManager.getOffsetY();
                    ang2 = deg2rad(obj.uiManager.getRotationAngleVehicle2());
                    if ~obj.uiManager.getRotateVehicle2(), ang2 = 0; end
                    R2 = [cos(ang2), -sin(ang2); sin(ang2), cos(ang2)];
                    pts2 = R2 * [obj.dataManager.globalVehicle2Data.X(:)'; obj.dataManager.globalVehicle2Data.Y(:)'];
                    obj.dataManager.globalVehicle2Data.X = pts2(1, :)' + ox2;
                    obj.dataManager.globalVehicle2Data.Y = pts2(2, :)' + oy2;
                    if obj.vehicleSim2.simParams.includeTrailer
                        % Transform primary trailer path
                        t2 = R2 * [obj.dataManager.globalTrailer2Data.X(:)'; obj.dataManager.globalTrailer2Data.Y(:)'];
                        obj.dataManager.globalTrailer2Data.X = t2(1, :)' + ox2;
                        obj.dataManager.globalTrailer2Data.Y = t2(2, :)' + oy2;
                        % Transform each additional trailer box
                        if isfield(obj.dataManager.globalTrailer2Data, 'Boxes')
                            for bi = 1:numel(obj.dataManager.globalTrailer2Data.Boxes)
                                Bx2 = obj.dataManager.globalTrailer2Data.Boxes(bi).X;
                                By2 = obj.dataManager.globalTrailer2Data.Boxes(bi).Y;
                                Bt2 = obj.dataManager.globalTrailer2Data.Boxes(bi).Theta;
                                Btrans2 = R2 * [Bx2'; By2'];
                                obj.dataManager.globalTrailer2Data.Boxes(bi).X     = Btrans2(1, :)' + ox2;
                                obj.dataManager.globalTrailer2Data.Boxes(bi).Y     = Btrans2(2, :)' + oy2;
                                obj.dataManager.globalTrailer2Data.Boxes(bi).Theta = mod(Bt2 + ang2 + pi, 2*pi) - pi;
                                if isfield(obj.dataManager.globalTrailer2Data, 'XBoxes') && ~isempty(obj.dataManager.globalTrailer2Data.XBoxes)
                                    obj.dataManager.globalTrailer2Data.XBoxes(bi,:) = Btrans2(1, :) + ox2;
                                    obj.dataManager.globalTrailer2Data.YBoxes(bi,:) = Btrans2(2, :) + oy2;
                                    obj.dataManager.globalTrailer2Data.ThetaBoxes(bi,:) = mod(Bt2 + ang2 + pi, 2*pi) - pi;
                                end
                            end
                        end
                    end
                catch
                end
                end  % end fresh-run transform guard
                waitbar(0.25, hWaitbar, 'Simulation Data Equalized.');

                %% 11. Determine Total Steps
                disp('Determining total simulation steps...');
                totalSteps = maxLength;
                fprintf('Total Simulation Steps set to: %d\n', totalSteps);

                %% 12. Initialize Collision Data (will be filled later)
                disp('Initializing collision data...');
                obj.dataManager.collisionData.X = NaN(totalSteps, 1);
                obj.dataManager.collisionData.Y = NaN(totalSteps, 1);

                %% 13. Validate Vehicle Data
                disp('Validating vehicle data...');
                if isempty(obj.dataManager.globalVehicle1Data.X) || isempty(obj.dataManager.globalVehicle1Data.Y) || ...
                   isempty(obj.dataManager.globalVehicle2Data.X) || isempty(obj.dataManager.globalVehicle2Data.Y)
                    uialert(obj.uiManager.fig, 'One or both vehicle simulations returned empty data. Please check simulation parameters.', 'Data Error');
                    return;
                end

                %% 14. Apply Offsets and Rotations (only for fresh runs)
                if ~obj.useSavedData
                %% 14. Apply Offsets (Vehicle 2)
                disp('Applying offsets to Tractor Vehicle positions...');
                obj.dataManager.globalVehicle2Data.X = obj.dataManager.globalVehicle2Data.X;
                obj.dataManager.globalVehicle2Data.Y = obj.dataManager.globalVehicle2Data.Y;
                if obj.vehicleSim2.simParams.includeTrailer
                    obj.dataManager.globalTrailer2Data.X = obj.dataManager.globalTrailer2Data.X;
                    obj.dataManager.globalTrailer2Data.Y = obj.dataManager.globalTrailer2Data.Y;
                end

                %% 15. Apply Rotation and Offset to Vehicle 1 if Enabled
                if rotateVehicle1
                    fprintf('Rotation enabled for Vehicle 1. Applying rotation of %.2f degrees.\n', vehicle1rotationAngleDeg);
                    R = [cos(vehicle1rotationAngleRad), -sin(vehicle1rotationAngleRad);
                         sin(vehicle1rotationAngleRad),  cos(vehicle1rotationAngleRad)];
                    rotatedPositionsV1 = (R * ([obj.dataManager.globalVehicle1Data.X, obj.dataManager.globalVehicle1Data.Y]).').';
                    vehicle1offsetP = [vehicle1OffsetX, vehicle1OffsetY];
                    rotatedPositionsV1 = (rotatedPositionsV1 + vehicle1offsetP)';
                    obj.dataManager.globalVehicle1Data.X = rotatedPositionsV1(1, :)';
                    obj.dataManager.globalVehicle1Data.Y = rotatedPositionsV1(2, :)';
                    obj.dataManager.globalVehicle1Data.Theta = ...
                        mod(obj.dataManager.globalVehicle1Data.Theta + vehicle1rotationAngleRad + pi, 2*pi) - pi;

                    if obj.vehicleSim1.simParams.includeTrailer
                        % Transform primary trailer path for rotation
                        X_trailer = obj.dataManager.globalTrailer1Data.X;
                        Y_trailer = obj.dataManager.globalTrailer1Data.Y;
                        positionsTrailer = [X_trailer, Y_trailer];
                        rotatedTrailer = (R * positionsTrailer.').';
                        rotatedTrailer = (rotatedTrailer + vehicle1offsetP)';
                        obj.dataManager.globalTrailer1Data.X = rotatedTrailer(1, :)';
                        obj.dataManager.globalTrailer1Data.Y = rotatedTrailer(2, :)';
                        obj.dataManager.globalTrailer1Data.Theta = mod(obj.dataManager.globalTrailer1Data.Theta + vehicle1rotationAngleRad + pi, 2*pi) - pi;
                        % Transform each additional trailer box
                        if isfield(obj.dataManager.globalTrailer1Data, 'Boxes')
                            for bi = 1:numel(obj.dataManager.globalTrailer1Data.Boxes)
                                Bx = obj.dataManager.globalTrailer1Data.Boxes(bi).X;
                                By = obj.dataManager.globalTrailer1Data.Boxes(bi).Y;
                                Bt = obj.dataManager.globalTrailer1Data.Boxes(bi).Theta;
                                Btrans = (R * [Bx, By].').';
                                Btrans = (Btrans + vehicle1offsetP)';
                                obj.dataManager.globalTrailer1Data.Boxes(bi).X     = Btrans(1, :)';
                                obj.dataManager.globalTrailer1Data.Boxes(bi).Y     = Btrans(2, :)';
                                obj.dataManager.globalTrailer1Data.Boxes(bi).Theta = mod(Bt + vehicle1rotationAngleRad + pi, 2*pi) - pi;
                                if isfield(obj.dataManager.globalTrailer1Data, 'XBoxes') && ~isempty(obj.dataManager.globalTrailer1Data.XBoxes)
                                    obj.dataManager.globalTrailer1Data.XBoxes(bi,:) = Btrans(1, :);
                                    obj.dataManager.globalTrailer1Data.YBoxes(bi,:) = Btrans(2, :);
                                    obj.dataManager.globalTrailer1Data.ThetaBoxes(bi,:) = mod(Bt + vehicle1rotationAngleRad + pi, 2*pi) - pi;
                                end
                            end
                        end
                    end
                else
                    % Compute shift based on Vehicle 1 initial position
                    initialPos1 = [obj.dataManager.globalVehicle1Data.X(1), obj.dataManager.globalVehicle1Data.Y(1)];
                    shift1 = [vehicle1OffsetX, vehicle1OffsetY] - initialPos1;
                    % Apply same shift to both tractor and trailer
                    obj.dataManager.globalVehicle1Data.X = obj.dataManager.globalVehicle1Data.X + shift1(1);
                    obj.dataManager.globalVehicle1Data.Y = obj.dataManager.globalVehicle1Data.Y + shift1(2);
                    if obj.vehicleSim1.simParams.includeTrailer
                        obj.dataManager.globalTrailer1Data.X = obj.dataManager.globalTrailer1Data.X + shift1(1);
                        obj.dataManager.globalTrailer1Data.Y = obj.dataManager.globalTrailer1Data.Y + shift1(2);
                        % Shift each additional trailer box
                        if isfield(obj.dataManager.globalTrailer1Data, 'Boxes')
                            for bi = 1:numel(obj.dataManager.globalTrailer1Data.Boxes)
                                obj.dataManager.globalTrailer1Data.Boxes(bi).X = obj.dataManager.globalTrailer1Data.Boxes(bi).X + shift1(1);
                                obj.dataManager.globalTrailer1Data.Boxes(bi).Y = obj.dataManager.globalTrailer1Data.Boxes(bi).Y + shift1(2);
                                if isfield(obj.dataManager.globalTrailer1Data, 'XBoxes') && ~isempty(obj.dataManager.globalTrailer1Data.XBoxes)
                                    obj.dataManager.globalTrailer1Data.XBoxes(bi,:) = obj.dataManager.globalTrailer1Data.XBoxes(bi,:) + shift1(1);
                                    obj.dataManager.globalTrailer1Data.YBoxes(bi,:) = obj.dataManager.globalTrailer1Data.YBoxes(bi,:) + shift1(2);
                                end
                            end
                        end
                    end
                end

                %% 16. Apply Rotation and Offset to Vehicle 2 if Enabled
                if rotateVehicle2
                    fprintf('Rotation enabled for Vehicle 2. Applying rotation of %.2f degrees.\n', rotationAngleDeg);
                    R = [cos(rotationAngleRad), -sin(rotationAngleRad);
                         sin(rotationAngleRad),  cos(rotationAngleRad)];
                    rotatedPositionsV2 = (R * ([obj.dataManager.globalVehicle2Data.X, obj.dataManager.globalVehicle2Data.Y]).').';
                    offsetP = [offsetX, offsetY];
                    rotatedPositionsV2 = (rotatedPositionsV2 + offsetP)';
                    obj.dataManager.globalVehicle2Data.X = rotatedPositionsV2(1, :)';
                    obj.dataManager.globalVehicle2Data.Y = rotatedPositionsV2(2, :)';
                    obj.dataManager.globalVehicle2Data.Theta = ...
                        mod(obj.dataManager.globalVehicle2Data.Theta + rotationAngleRad + pi, 2*pi) - pi;

                    if obj.vehicleSim2.simParams.includeTrailer
                        % Transform primary trailer path for rotation
                        X_trailer = obj.dataManager.globalTrailer2Data.X;
                        Y_trailer = obj.dataManager.globalTrailer2Data.Y;
                        positionsTrailer = [X_trailer, Y_trailer];
                        rotatedTrailer = (R * positionsTrailer.').';
                        rotatedTrailer = (rotatedTrailer + offsetP)';
                        obj.dataManager.globalTrailer2Data.X = rotatedTrailer(1, :)';
                        obj.dataManager.globalTrailer2Data.Y = rotatedTrailer(2, :)';
                        obj.dataManager.globalTrailer2Data.Theta = mod(obj.dataManager.globalTrailer2Data.Theta + rotationAngleRad + pi, 2*pi) - pi;
                        % Transform each additional trailer box
                        if isfield(obj.dataManager.globalTrailer2Data, 'Boxes')
                            for bi = 1:numel(obj.dataManager.globalTrailer2Data.Boxes)
                                Bx2 = obj.dataManager.globalTrailer2Data.Boxes(bi).X;
                                By2 = obj.dataManager.globalTrailer2Data.Boxes(bi).Y;
                                Bt2 = obj.dataManager.globalTrailer2Data.Boxes(bi).Theta;
                                Btrans2 = (R * [Bx2, By2].').';
                                Btrans2 = (Btrans2 + offsetP)';
                                obj.dataManager.globalTrailer2Data.Boxes(bi).X     = Btrans2(1, :)';
                                obj.dataManager.globalTrailer2Data.Boxes(bi).Y     = Btrans2(2, :)';
                                obj.dataManager.globalTrailer2Data.Boxes(bi).Theta = mod(Bt2 + rotationAngleRad + pi, 2*pi) - pi;
                                if isfield(obj.dataManager.globalTrailer2Data, 'XBoxes') && ~isempty(obj.dataManager.globalTrailer2Data.XBoxes)
                                    obj.dataManager.globalTrailer2Data.XBoxes(bi,:) = Btrans2(1, :);
                                    obj.dataManager.globalTrailer2Data.YBoxes(bi,:) = Btrans2(2, :);
                                    obj.dataManager.globalTrailer2Data.ThetaBoxes(bi,:) = mod(Bt2 + rotationAngleRad + pi, 2*pi) - pi;
                                end
                            end
                        end
                    end
                else
                    % Compute shift based on Vehicle 2 initial position
                    initialPos2 = [obj.dataManager.globalVehicle2Data.X(1), obj.dataManager.globalVehicle2Data.Y(1)];
                    shift2 = [offsetX, offsetY] - initialPos2;
                    % Apply same shift to both tractor and trailer
                    obj.dataManager.globalVehicle2Data.X = obj.dataManager.globalVehicle2Data.X + shift2(1);
                    obj.dataManager.globalVehicle2Data.Y = obj.dataManager.globalVehicle2Data.Y + shift2(2);
                    if obj.vehicleSim2.simParams.includeTrailer
                        obj.dataManager.globalTrailer2Data.X = obj.dataManager.globalTrailer2Data.X + shift2(1);
                        obj.dataManager.globalTrailer2Data.Y = obj.dataManager.globalTrailer2Data.Y + shift2(2);
                        % Shift each additional trailer box
                        if isfield(obj.dataManager.globalTrailer2Data, 'Boxes')
                            for bi = 1:numel(obj.dataManager.globalTrailer2Data.Boxes)
                                obj.dataManager.globalTrailer2Data.Boxes(bi).X = obj.dataManager.globalTrailer2Data.Boxes(bi).X + shift2(1);
                                obj.dataManager.globalTrailer2Data.Boxes(bi).Y = obj.dataManager.globalTrailer2Data.Boxes(bi).Y + shift2(2);
                                if isfield(obj.dataManager.globalTrailer2Data, 'XBoxes') && ~isempty(obj.dataManager.globalTrailer2Data.XBoxes)
                                    obj.dataManager.globalTrailer2Data.XBoxes(bi,:) = obj.dataManager.globalTrailer2Data.XBoxes(bi,:) + shift2(1);
                                    obj.dataManager.globalTrailer2Data.YBoxes(bi,:) = obj.dataManager.globalTrailer2Data.YBoxes(bi,:) + shift2(2);
                                end
                            end
                        end
                    end
                end
                end  % end fresh-run transform guard


                %% 19. Launch Collision Detection in Parallel
                disp('Launching collision detection in background...');
                % We pass in all the needed arguments. This will run the collision
                % detection logic in the background while we animate on the main thread.
                collisionFuture = parfeval(@obj.runCollisionCheck, 1, totalSteps, ...
                    vehicleParams1, trailerParams1, vehicleParams2, trailerParams2, ...
                    VehicleUnderAnalysisMaxMass, J2980AssumedMaxMass);

                %% 20. Animate Vehicles
                disp('Starting animation...');
                zoom(obj.plotManager.sharedAx, 'on');
                set(obj.plotManager.sharedAx, 'XLimMode', 'manual', 'YLimMode', 'manual');
                % Prepare lane map and initial markers once
                mapWidth  = 600;
                mapHeight = 600;
                mapObj = LaneMap(mapWidth, mapHeight);
                mapObj.LaneCommands = obj.map;
                mapObj.LaneColor    = [0.8275, 0.8275, 0.8275];
                mapObj.LaneWidth    = 5;

                % Plot lane map if enabled in simulation controls
                if obj.uiManager.getMapTrajectoryFlag()
                % Initialize LaneMap and capture map image handle
                mapObj = LaneMap(mapWidth, mapHeight);
                mapObj.LaneCommands = obj.map;
                mapObj.LaneColor    = [0.8275, 0.8275, 0.8275];
                % Plot lane map and get handle for toggling
                mapHandle = mapObj.plotLaneMapWithCommands(obj.plotManager.sharedAx, ...
                                                          mapObj.LaneCommands, ...
                                                          mapObj.LaneColor);
                end

                % Ensure trajectories and markers are on top of the track
                try
                    pm = obj.plotManager;
                    trajHandles = [pm.veh1Line, pm.trl1Line, pm.trl1RearLine, ...
                                    pm.veh2Line, pm.trl2Line, pm.trl2RearLine, ...
                                    pm.veh1StartMarker, pm.trl1StartMarker, ...
                                    pm.veh2StartMarker, pm.trl2StartMarker];
                    for h = trajHandles
                        uistack(h, 'top');
                    end
                catch
                end

                hold(obj.plotManager.sharedAx, 'on');

                obj.plotManager.highlightInitialPositions(obj.dataManager);

                includeTrailer2 = isfield(obj.vehicleSim2.simParams, 'includeTrailer') && ...
                                      obj.vehicleSim2.simParams.includeTrailer;

                % Highlight initial positions
                obj.plotManager.highlightInitialPositions(obj.dataManager);
                % Initialize wheel markers for vehicles and trailers
                obj.plotManager.initWheelMarkers(obj.dataManager, vehicleParams1, trailerParams1, vehicleParams2, trailerParams2);
                % Initialize simulation control flags (pause until Play pressed)
                obj.uiManager.pauseFlag = true;
                obj.uiManager.stopFlag  = false;
                % Start playback loop
                iStep = 1;
                while iStep <= totalSteps
                    % Toggle map visibility based on control
                    try
                        if obj.uiManager.getMapTrajectoryFlag()
                            mapHandle.Visible = 'on';
                        else
                            mapHandle.Visible = 'off';
                        end
                    catch
                        % ignore if mapHandle invalid
                    end
                    % Check for stop request
                    if obj.uiManager.getStopFlag()
                        disp('Simulation stopped by user.');
                        break;
                    end
                    % Wait while paused
                    while obj.uiManager.getPauseFlag()
                        pause(0.1);
                    end
                    % Update trajectories and outlines
                    obj.plotManager.updateTrajectories(obj.dataManager, iStep, ...
                        obj.vehicleSim1.simParams, obj.vehicleSim2.simParams);
                    obj.plotManager.updateVehicleOutlines(obj.dataManager, iStep, ...
                        vehicleParams1, trailerParams1, vehicleParams2, trailerParams2);
                    drawnow limitrate; % faster frame updates by limiting draw rate
                    % Get dynamic playback speed
                    try
                        playbackSpeed = obj.uiManager.getPlaybackSpeed();
                        if ~isnumeric(playbackSpeed) || playbackSpeed <= 0
                            playbackSpeed = 1;
                        end
                    catch
                        playbackSpeed = 1;
                    end
                    % Compute skip and delay
                    skipSteps = max(1, floor(playbackSpeed));
                    frameDelay = obj.dt / playbackSpeed;
                    pause(frameDelay);
                    iStep = iStep + skipSteps;
                end

                disp('Animation complete. Fetching collision results from the background...');
                % fetchOutputs returns the single output directly, not in a cell
                % array, so we assign it directly to collisionData
                collisionData   = fetchOutputs(collisionFuture);

                % You can handle collisionData as you wish (e.g., store it, re-plot).
                if collisionData.collisionDetected
                    fprintf('Collision was detected at step %d.\n', collisionData.collisionStep);
                    obj.collisionX = collisionData.collisionX;
                    obj.collisionY = collisionData.collisionY;
                    % Store collision data in DataManager
                    obj.dataManager.collisionData.X(collisionData.collisionStep) = collisionData.collisionX;
                    obj.dataManager.collisionData.Y(collisionData.collisionStep) = collisionData.collisionY;
                    % Optionally, you can re-plot the final or collision state, highlight collision, etc.
                else
                    disp('No collision detected in the entire simulation.');
                end

            catch ME
                disp(['An error occurred: ', ME.message]);
                obj.saveLogs({sprintf('Simulation Error: %s', ME.message)});
                rethrow(ME);
                waitbar(1, hWaitbar, 'Simulation Error Occurred.');
                pause(1);
                closeIfOpen(hWaitbar);
            end
        end

        %/**
        % * @brief This method runs the "for i=1:totalSteps" collision check in the background.
        % * @param totalSteps The total number of simulation steps.
        % */
        function collisionData = runCollisionCheck(obj, totalSteps, ...
                vehicleParams1, trailerParams1, vehicleParams2, trailerParams2, ...
                VehicleUnderAnalysisMaxMass, J2980AssumedMaxMass)

            collisionData.collisionDetected = false;
            collisionData.collisionStep     = NaN;
            collisionData.collisionX        = NaN;
            collisionData.collisionY        = NaN;

            fprintf('--- Starting Collision Detection in Parallel ---\n');
            collisionFlags = false(totalSteps,1);
            colX = NaN(totalSteps,1);
            colY = NaN(totalSteps,1);
            detector = obj.collisionDetector; %#ok<NASGU>
            parfor i = 2:totalSteps
                tractorCorners1 = VehiclePlotter.getVehicleCorners(...
                    obj.dataManager.globalVehicle1Data.X(i), ...
                    obj.dataManager.globalVehicle1Data.Y(i), ...
                    obj.dataManager.globalVehicle1Data.Theta(i), ...
                    vehicleParams1, ...
                    true, ...
                    0, ...
                    vehicleParams1.numTiresPerAxle ...
                );

                tractorCorners2 = VehiclePlotter.getVehicleCorners(...
                    obj.dataManager.globalVehicle2Data.X(i), ...
                    obj.dataManager.globalVehicle2Data.Y(i), ...
                    obj.dataManager.globalVehicle2Data.Theta(i), ...
                    vehicleParams2, ...
                    true, ...
                    0, ...
                    vehicleParams2.numTiresPerAxle ...
                );

                [collisionFlags(i), colX(i), colY(i)] = ...
                    obj.checkCornersForCollision(i, tractorCorners1, tractorCorners2, ...
                                             trailerParams1, trailerParams2);
            end

            idx = find(collisionFlags, 1, 'first');
            if ~isempty(idx)
                collisionData.collisionDetected = true;
                collisionData.collisionStep     = idx;
                collisionData.collisionX        = colX(idx);
                collisionData.collisionY        = colY(idx);
                fprintf('Collision detected at Step %d.\n', idx);
            end

            fprintf('--- Collision Detection in Parallel Completed ---\n');
        end

        %/**
        % * @brief Checks the corners for collision at step i, returning [didCollide, colX, colY].
        % */
        function [didCollide, collisionX, collisionY] = checkCornersForCollision(...
            obj, i, tractorCorners1, tractorCorners2, trailerParams1, trailerParams2)

            didCollide = false;
            collisionX = NaN;
            collisionY = NaN;

            % Collect corner sets for each vehicle and all of its trailer boxes
            group1 = [{tractorCorners1}, ...
                      obj.getTrailerBoxesCorners(obj.dataManager.globalTrailer1Data, ...
                                                trailerParams1, i)];
            group2 = [{tractorCorners2}, ...
                      obj.getTrailerBoxesCorners(obj.dataManager.globalTrailer2Data, ...
                                                trailerParams2, i)];

            % Check all combinations for collision
            collisionFound = false;
            for a = 1:numel(group1)
                for b = 1:numel(group2)
                    collided = obj.collisionDetector.checkCollision(group1{a}, group2{b});
                    if collided
                        collisionFound = true;
                        idxA = a; idxB = b; %#ok<NASGU>
                        break;
                    end
                end
                if collisionFound
                    break;
                end
            end

            if collisionFound
                didCollide = true;

                % Build union polygons for each vehicle group for accurate center
                poly1 = polyshape(group1{1}(:,1), group1{1}(:,2));
                for k = 2:numel(group1)
                    poly1 = union(poly1, polyshape(group1{k}(:,1), group1{k}(:,2)));
                end

                poly2 = polyshape(group2{1}(:,1), group2{1}(:,2));
                for k = 2:numel(group2)
                    poly2 = union(poly2, polyshape(group2{k}(:,1), group2{k}(:,2)));
                end

                intersection = intersect(poly1, poly2);
                if ~isempty(intersection.Vertices)
                    collisionX = mean(intersection.Vertices(:,1));
                    collisionY = mean(intersection.Vertices(:,2));
                else
                    collisionX = 0.5 * (obj.dataManager.globalVehicle1Data.X(i) + ...
                                        obj.dataManager.globalVehicle2Data.X(i));
                    collisionY = 0.5 * (obj.dataManager.globalVehicle1Data.Y(i) + ...
                                        obj.dataManager.globalVehicle2Data.Y(i));
                end
            end
        end

        % (Remaining methods unchanged except for removing the old in-line collision loop)

        function transferSimulationResults(obj)
            requiredFields = {'X', 'Y', 'Theta', 'speedData', 'steeringAngles'};
            if isstruct(obj.sim1Results)
                missingFields = setdiff(requiredFields, fieldnames(obj.sim1Results));
                if ~isempty(missingFields)
                    error('sim1Results is missing required fields: %s', strjoin(missingFields, ', '));
                end
                obj.dataManager.globalVehicle1Data.X = obj.sim1Results.X(:);
                obj.dataManager.globalVehicle1Data.Y = obj.sim1Results.Y(:);
                obj.dataManager.globalVehicle1Data.Theta = obj.sim1Results.Theta(:);
                obj.dataManager.globalVehicle1Data.Speed = obj.sim1Results.speedData(:);
                obj.dataManager.globalVehicle1Data.SteeringAngle = obj.sim1Results.steeringAngles(:);
            else
                error('sim1Results is not a structure. Expected a structure.');
            end

            if obj.vehicleSim1.simParams.includeTrailer
                % Multi-box trailer: use Boxes if available
                if isfield(obj.sim1Results, 'trailerXBoxes') && isfield(obj.sim1Results, 'trailerYBoxes') && isfield(obj.sim1Results, 'trailerThetaBoxes')
                    nBoxes1 = size(obj.sim1Results.trailerXBoxes, 1);
                    Boxes1 = struct('X', cell(1,nBoxes1), 'Y', cell(1,nBoxes1), 'Theta', cell(1,nBoxes1));
                    for bi = 1:nBoxes1
                        Boxes1(bi).X     = obj.sim1Results.trailerXBoxes(bi, :)';
                        Boxes1(bi).Y     = obj.sim1Results.trailerYBoxes(bi, :)';
                        Boxes1(bi).Theta = obj.sim1Results.trailerThetaBoxes(bi, :)';
                    end
                    obj.dataManager.globalTrailer1Data.Boxes = Boxes1;
                    obj.dataManager.globalTrailer1Data.XBoxes = obj.sim1Results.trailerXBoxes;
                    obj.dataManager.globalTrailer1Data.YBoxes = obj.sim1Results.trailerYBoxes;
                    obj.dataManager.globalTrailer1Data.ThetaBoxes = obj.sim1Results.trailerThetaBoxes;
                    % Primary trailer path is first box
                    obj.dataManager.globalTrailer1Data.X     = Boxes1(1).X;
                    obj.dataManager.globalTrailer1Data.Y     = Boxes1(1).Y;
                    obj.dataManager.globalTrailer1Data.Theta = Boxes1(1).Theta;
                elseif isfield(obj.sim1Results, 'trailerX') && isfield(obj.sim1Results, 'trailerY') && isfield(obj.sim1Results, 'trailerTheta')
                    obj.dataManager.globalTrailer1Data.X     = obj.sim1Results.trailerX(:);
                    obj.dataManager.globalTrailer1Data.Y     = obj.sim1Results.trailerY(:);
                    obj.dataManager.globalTrailer1Data.Theta = obj.sim1Results.trailerTheta(:);
                    obj.dataManager.globalTrailer1Data.Boxes = struct('X', obj.dataManager.globalTrailer1Data.X, ...
                                                                     'Y', obj.dataManager.globalTrailer1Data.Y, ...
                                                                     'Theta', obj.dataManager.globalTrailer1Data.Theta);
                    obj.dataManager.globalTrailer1Data.XBoxes = obj.dataManager.globalTrailer1Data.X;
                    obj.dataManager.globalTrailer1Data.YBoxes = obj.dataManager.globalTrailer1Data.Y;
                    obj.dataManager.globalTrailer1Data.ThetaBoxes = obj.dataManager.globalTrailer1Data.Theta;
                else
                    error('sim1Results missing trailer fields for Vehicle 1.');
                end
                % Steering angles for trailer
                if isfield(obj.sim1Results, 'trailerSteeringAngles') && ~isempty(obj.sim1Results.trailerSteeringAngles)
                    obj.dataManager.globalTrailer1Data.SteeringAngle = obj.sim1Results.trailerSteeringAngles(:);
                else
                    obj.dataManager.globalTrailer1Data.SteeringAngle = zeros(length(obj.dataManager.globalTrailer1Data.X), 1);
                    warning('No trailerSteeringAngles in sim1Results. Assigning zeros.');
                end
            else
                obj.dataManager.globalTrailer1Data = struct('X', [], 'Y', [], 'Theta', [], 'SteeringAngle', [], 'Boxes', [], 'XBoxes', [], 'YBoxes', [], 'ThetaBoxes', []);
            end

            if isstruct(obj.sim2Results)
                missingFields = setdiff(requiredFields, fieldnames(obj.sim2Results));
                if ~isempty(missingFields)
                    error('sim2Results is missing required fields: %s', strjoin(missingFields, ', '));
                end
                obj.dataManager.globalVehicle2Data.X = obj.sim2Results.X(:);
                obj.dataManager.globalVehicle2Data.Y = obj.sim2Results.Y(:);
                obj.dataManager.globalVehicle2Data.Theta = obj.sim2Results.Theta(:);
                obj.dataManager.globalVehicle2Data.Speed = obj.sim2Results.speedData(:);
                obj.dataManager.globalVehicle2Data.SteeringAngle = obj.sim2Results.steeringAngles(:);
            else
                error('sim2Results is not a structure. Expected a structure.');
            end

            if obj.vehicleSim2.simParams.includeTrailer
                % Multi-box trailer for Vehicle 2
                if isfield(obj.sim2Results, 'trailerXBoxes') && isfield(obj.sim2Results, 'trailerYBoxes') && isfield(obj.sim2Results, 'trailerThetaBoxes')
                    nBoxes2 = size(obj.sim2Results.trailerXBoxes, 1);
                    Boxes2 = struct('X', cell(1,nBoxes2), 'Y', cell(1,nBoxes2), 'Theta', cell(1,nBoxes2));
                    for bi = 1:nBoxes2
                        Boxes2(bi).X     = obj.sim2Results.trailerXBoxes(bi, :)';
                        Boxes2(bi).Y     = obj.sim2Results.trailerYBoxes(bi, :)';
                        Boxes2(bi).Theta = obj.sim2Results.trailerThetaBoxes(bi, :)';
                    end
                    obj.dataManager.globalTrailer2Data.Boxes = Boxes2;
                    obj.dataManager.globalTrailer2Data.XBoxes = obj.sim2Results.trailerXBoxes;
                    obj.dataManager.globalTrailer2Data.YBoxes = obj.sim2Results.trailerYBoxes;
                    obj.dataManager.globalTrailer2Data.ThetaBoxes = obj.sim2Results.trailerThetaBoxes;
                    obj.dataManager.globalTrailer2Data.X     = Boxes2(1).X;
                    obj.dataManager.globalTrailer2Data.Y     = Boxes2(1).Y;
                    obj.dataManager.globalTrailer2Data.Theta = Boxes2(1).Theta;
                elseif isfield(obj.sim2Results, 'trailerX') && isfield(obj.sim2Results, 'trailerY') && isfield(obj.sim2Results, 'trailerTheta')
                    obj.dataManager.globalTrailer2Data.X     = obj.sim2Results.trailerX(:);
                    obj.dataManager.globalTrailer2Data.Y     = obj.sim2Results.trailerY(:);
                    obj.dataManager.globalTrailer2Data.Theta = obj.sim2Results.trailerTheta(:);
                    obj.dataManager.globalTrailer2Data.Boxes = struct('X', obj.dataManager.globalTrailer2Data.X, ...
                                                                     'Y', obj.dataManager.globalTrailer2Data.Y, ...
                                                                     'Theta', obj.dataManager.globalTrailer2Data.Theta);
                    obj.dataManager.globalTrailer2Data.XBoxes = obj.dataManager.globalTrailer2Data.X;
                    obj.dataManager.globalTrailer2Data.YBoxes = obj.dataManager.globalTrailer2Data.Y;
                    obj.dataManager.globalTrailer2Data.ThetaBoxes = obj.dataManager.globalTrailer2Data.Theta;
                else
                    error('sim2Results missing trailer fields for Vehicle 2.');
                end
                % Steering angles
                if isfield(obj.sim2Results, 'trailerSteeringAngles') && ~isempty(obj.sim2Results.trailerSteeringAngles)
                    obj.dataManager.globalTrailer2Data.SteeringAngle = obj.sim2Results.trailerSteeringAngles(:);
                else
                    obj.dataManager.globalTrailer2Data.SteeringAngle = zeros(length(obj.dataManager.globalTrailer2Data.X), 1);
                    warning('No trailerSteeringAngles in sim2Results. Assigning zeros.');
                end
            else
                obj.dataManager.globalTrailer2Data = struct('X', [], 'Y', [], 'Theta', [], 'SteeringAngle', [], 'Boxes', [], 'XBoxes', [], 'YBoxes', [], 'ThetaBoxes', []);
            end
        end

        function saveLogs(obj, logMessages)
            txtFilename = obj.getUniqueFilename('simulation_Vehicle1', '.txt');
            csvFilename = obj.getUniqueFilename('simulation_Vehicle1', '.csv');

            fid = fopen(txtFilename, 'w');
            if fid ~= -1
                fprintf(fid, '%s\n', logMessages{:});
                fclose(fid);
                disp(['Log data saved to ', txtFilename]);
            else
                warning('Failed to write simulation log to text file.');
            end

            logTable = table(logMessages', 'VariableNames', {'Logs'});
            try
                writetable(logTable, csvFilename);
                disp(['Log data saved to ', csvFilename]);
            catch ME
                warning('Failed to write simulation log to CSV file: %s', ME.message);
            end
        end

        function uniqueName = getUniqueFilename(~, baseName, extension)
            uniqueName = [baseName, extension];
            if exist(uniqueName, 'file') == 2
                timestamp = datestr(now, 'yyyymmdd_HHMMSS');
                uniqueName = [baseName, '_', timestamp, extension];
            end
        end

        function params = createVehicleParams(obj, simParams, wheelHeight, wheelWidth)
            if isfield(simParams, 'numTiresPerAxleTractor')
                numTiresPerAxle = simParams.numTiresPerAxleTractor;
            else
                numTiresPerAxle = 4;
                warning('Missing numTiresPerAxleTractor. Using default: 4');
            end

            params = struct(...
                'isTractor', true, ...
                'length', simParams.tractorLength, ...
                'width', simParams.tractorWidth, ...
                'height', simParams.tractorHeight, ...
                'CoGHeight', simParams.tractorCoGHeight, ...
                'wheelbase', simParams.tractorWheelbase, ...
                'trackWidth', simParams.tractorTrackWidth, ...
                'numAxles', simParams.tractorNumAxles, ...
                'axleSpacing', simParams.tractorAxleSpacing, ...
                'mass', simParams.tractorMass, ...
                'wheelWidth', wheelWidth, ...
                'wheelHeight', wheelHeight, ...
                'numTiresPerAxle', numTiresPerAxle, ...
                'HitchDistance', simParams.tractorHitchDistance ...
            );
        end

        function params = createTrailerParams(obj, simParams, wheelHeight, wheelWidth, trailerNumber)
            requiredFields = {'trailerLength', 'trailerWidth', 'trailerHeight', 'trailerCoGHeight', ...
                              'trailerWheelbase', 'trailerTrackWidth', 'trailerNumAxles', ...
                              'trailerAxleSpacing'};
            for i = 1:length(requiredFields)
                if ~isfield(simParams, requiredFields{i})
                    error('simParams missing %s', requiredFields{i});
                end
            end

            % Use configured number of tires per axle for trailer
            if isfield(simParams, 'numTiresPerAxleTrailer') && ~isempty(simParams.numTiresPerAxleTrailer)
                numTiresPerAxle = simParams.numTiresPerAxleTrailer;
            else
                numTiresPerAxle = 2; % default to single tires if not specified
                warning('Missing numTiresPerAxleTrailer in simParams. Using default: %d', numTiresPerAxle);
            end

            numAxlesTractor   = simParams.tractorNumAxles;
            wheelbaseTractor  = simParams.tractorWheelbase;
            if numAxlesTractor > 1
                axlePositionsTractor = linspace(wheelbaseTractor / 2, -wheelbaseTractor / 2, numAxlesTractor);
            else
                axlePositionsTractor = 0;
            end

            frontAxlePosition = axlePositionsTractor(1);
            rearAxlePosition  = axlePositionsTractor(end);
            middleAxleIndex   = ceil(numAxlesTractor / 2);
            middleAxlePosition = axlePositionsTractor(middleAxleIndex);

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
            fprintf('Total vehicle mass updated: %.2f kg\n', simParams.tractorMass + massVal);

            boxNumAxles = simParams.trailerAxlesPerBox;
            numAxles    = sum(boxNumAxles);

            params = struct(...
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
                'wheelWidth', wheelWidth, ...
                'wheelHeight', wheelHeight, ...
                'numTiresPerAxle', numTiresPerAxle, ...
                'W_Total', massVal, ...
                'includeTrailer', simParams.includeTrailer, ...
                'HitchDistance', simParams.trailerHitchDistance ...
            );

            if trailerNumber == 1
                obj.dataManager.TrailerLength1 = simParams.trailerLength;
            elseif trailerNumber == 2
                obj.dataManager.TrailerLength2 = simParams.trailerLength;
            end
        end

        function cornersCell = getTrailerBoxesCorners(obj, trailerData, trailerParams, step)
            % getTrailerBoxesCorners Returns corners for all trailer boxes at a given step
            cornersCell = {};
            if isempty(trailerParams)
                return;
            end
            if isfield(trailerData, 'Boxes') && ~isempty(trailerData.Boxes)
                nBoxes = numel(trailerData.Boxes);
                cornersCell = cell(1, nBoxes);
                for bi = 1:nBoxes
                    cornersCell{bi} = VehiclePlotter.getVehicleCorners( ...
                        trailerData.Boxes(bi).X(step), ...
                        trailerData.Boxes(bi).Y(step), ...
                        trailerData.Boxes(bi).Theta(step), ...
                        trailerParams, false, 0, trailerParams.numTiresPerAxle);
                end
            elseif isfield(trailerData, 'X') && ~isempty(trailerData.X)
                cornersCell = {VehiclePlotter.getVehicleCorners( ...
                    trailerData.X(step), trailerData.Y(step), trailerData.Theta(step), ...
                    trailerParams, false, 0, trailerParams.numTiresPerAxle)};
            end
        end
    end

    methods(Static)
        function simResults = runVehicleSim1(vehicleSim, v1)
            simResults = vehicleSim.computeNextSimFrame(v1);
        end

        function simResults = runVehicleSim2(vehicleSim, v2)
            simResults = vehicleSim.computeNextSimFrame(v2);
        end
        
        function simResultsArray = runBatchVehicleSimulations(vehicleSimArray)
            % runBatchVehicleSimulations Runs multiple vehicle simulations in parallel
            n = numel(vehicleSimArray);
            simResultsArray = cell(1, n);
            parfor k = 1:n
                simResultsArray{k} = SimManager.runVehicleSim1(vehicleSimArray{k});
            end
        end
    end
end

function extendedData = extendData(data, targetLength)
    currentLength = length(data);
    if currentLength == 0
        extendedData = zeros(targetLength, 1);
    elseif currentLength < targetLength
        lastValue  = data(end);
        extension  = repmat(lastValue, targetLength - currentLength, 1);
        extendedData = [data; extension];
    else
        extendedData = data;
    end
end

function closeIfOpen(h)
    if isvalid(h)
        close(h);
    end
end

function closeLocalPool(created)
    if created
        p = gcp('nocreate');
        if ~isempty(p)
            delete(p);
        end
    end
end