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
% @file ConfigurationManager.m
% @brief Handles saving and loading of simulation configurations.
%        Provides persistence for simulation settings.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}
%/**
% * @class ConfigurationManager
% * @brief Handles saving and loading of simulation configurations.
% *
% * The ConfigurationManager class manages the persistence of simulation settings by enabling
% * users to save current configurations to an XML file and load configurations from existing
% * XML files. It interacts with various UI components and simulation configuration objects
% * to ensure that all relevant parameters are accurately stored and retrieved.
% *
% * @author Miguel Marina
% * @version 2.5
% * @date 2024-10-15
% */
classdef ConfigurationManager < handle
    properties
        %/**
        % * @property uiManager
        % * @brief Reference to the UI Manager.
        % *
        % * Manages interactions with the user interface, providing access to various UI components.
        % */
        uiManager

        %/**
        % * @property truckSimConfig1
        % * @brief Reference to the Truck 1 Simulation Configuration.
        % *
        % * Contains parameters and settings specific to the Truck 1 simulation.
        % */
        truckSimConfig1

        %/**
        % * @property truckSimConfig2
        % * @brief Reference to the Truck 2 Simulation Configuration.
        % *
        % * Contains parameters and settings specific to the Truck 2 simulation.
        % */
        truckSimConfig2

        %/**
        % * @property filename
        % * @brief Filename for the configuration file.
        % *
        % * Stores the path and name of the XML configuration file being saved or loaded.
        % */
        filename
    end

    methods
        %/**
        % * @brief Constructor to initialize the ConfigurationManager.
        % *
        % * Initializes a new instance of the ConfigurationManager class with references to the UI Manager
        % * and simulation configuration objects.
        % *
        % * @param uiManager Reference to the UI Manager.
        % * @param truckSimConfig1 Reference to the Truck 1 Simulation Configuration object.
        % * @param truckSimConfig2 Reference to the Truck 2 Simulation Configuration object.
        % *
        % * @throws Error if the number of input arguments is less than three.
        % */
        function obj = ConfigurationManager(uiManager, truckSimConfig1, truckSimConfig2)
            % Constructor initializes the ConfigurationManager with necessary references.
            if nargin < 3
                error('ConfigurationManager requires at least three input arguments: uiManager, truckSimConfig1, truckSimConfig2.');
            end

            obj.uiManager = uiManager;
            obj.truckSimConfig1 = truckSimConfig1;
            obj.truckSimConfig2 = truckSimConfig2;
            obj.filename = ''; % Initialize filename as empty
        end

        %/**
        % * @brief Saves the current simulation configuration to an XML file.
        % *
        % * Gathers simulation parameters from both Truck 1 and Truck 2 simulation configurations,
        % * as well as current values from various UI fields. It then prompts the user to select a
        % * save location and writes the configuration data to the chosen XML file.
        % *
        % * @return None
        % *
        % * @warning If the user cancels the save operation, the method exits without saving.
        % */
        function saveConfiguration(obj)
            % Save the current configuration to an XML file.

            % Prompt user to select save location
            [file, path] = uiputfile('*.xml', 'Save Configuration');
            if isequal(file,0) || isequal(path,0)
                disp('User canceled Configuration save.');
                return;
            end
            obj.filename = fullfile(path, file);

            % Create XML document
            docNode = com.mathworks.xml.XMLUtils.createDocument('Configuration');
            root = docNode.getDocumentElement;

            %% Save Vehicle 1 Configuration
            vehicle1Config = obj.truckSimConfig1.getSimulationParameters(); % Assuming this returns a struct
            vehicle1Config = obj.removeTableFields(vehicle1Config, {'gearRatios'}); % Remove Excel data, keep gearRatios

            % Compute Total Tires and Active TiresN for Vehicle1
            numTrailerAxles1 = sum(vehicle1Config.trailerAxlesPerBox);
            totalTires1 = (vehicle1Config.numTiresPerAxleTractor * vehicle1Config.tractorNumAxles) + ...
                          (vehicle1Config.numTiresPerAxleTrailer * numTrailerAxles1);
            activeTiresN1 = sprintf('Tires%d', totalTires1);

            % Filter pressureMatrices for Vehicle1
            if isfield(vehicle1Config, 'pressureMatrices') && isfield(vehicle1Config.pressureMatrices, activeTiresN1)
                pressureMatrices1 = struct();
                pressureMatrices1.(activeTiresN1) = vehicle1Config.pressureMatrices.(activeTiresN1);
                vehicle1Config.pressureMatrices = pressureMatrices1;
                fprintf('Included active pressure matrix: %s for Vehicle1.\n', activeTiresN1);
            else
                warning('Active pressure matrix %s not found for Vehicle1.', activeTiresN1);
            end

            % Ensure waypoints for Vehicle1 are Nx2 numeric, or remove them
            if isfield(vehicle1Config, 'waypoints') && ~isempty(vehicle1Config.waypoints)
                if size(vehicle1Config.waypoints,2) ~= 2
                    warning('Vehicle1 waypoints is not Nx2 numeric array. Removing waypoints from config.');
                    vehicle1Config = rmfield(vehicle1Config, 'waypoints');
                end
            end

            % Serialize Vehicle1 Config and Append to XML
            vehicle1Element = obj.structToXMLElement(docNode, 'Vehicle1', vehicle1Config);
            root.appendChild(vehicle1Element);

            %% Save Vehicle 2 Configuration
            vehicle2Config = obj.truckSimConfig2.getSimulationParameters();
            vehicle2Config = obj.removeTableFields(vehicle2Config, {'gearRatios'});

            % Compute Total Tires and Active TiresN for Vehicle2
            numTrailerAxles2 = sum(vehicle2Config.trailerAxlesPerBox);
            totalTires2 = (vehicle2Config.numTiresPerAxleTractor * vehicle2Config.tractorNumAxles) + ...
                          (vehicle2Config.numTiresPerAxleTrailer * numTrailerAxles2);
            activeTiresN2 = sprintf('Tires%d', totalTires2);

            % Filter pressureMatrices for Vehicle2
            if isfield(vehicle2Config, 'pressureMatrices') && isfield(vehicle2Config.pressureMatrices, activeTiresN2)
                pressureMatrices2 = struct();
                pressureMatrices2.(activeTiresN2) = vehicle2Config.pressureMatrices.(activeTiresN2);
                vehicle2Config.pressureMatrices = pressureMatrices2;
                fprintf('Included active pressure matrix: %s for Vehicle2.\n', activeTiresN2);
            else
                warning('Active pressure matrix %s not found for Vehicle2.', activeTiresN2);
            end

            % Ensure waypoints for Vehicle2 are Nx2 numeric, or remove them
            if isfield(vehicle2Config, 'waypoints') && ~isempty(vehicle2Config.waypoints)
                if ~isnumeric(vehicle2Config.waypoints) || size(vehicle2Config.waypoints,2) ~= 2
                    warning('Vehicle2 waypoints is not Nx2 numeric array. Removing waypoints from config.');
                    vehicle2Config = rmfield(vehicle2Config, 'waypoints');
                end
            end

            % Serialize Vehicle2 Config and Append to XML
            vehicle2Element = obj.structToXMLElement(docNode, 'Vehicle2', vehicle2Config);
            root.appendChild(vehicle2Element);

            %% Save UI Fields
            uiFields = struct();
            
            % Vehicle 1 offsets and rotation
            uiFields.vehicle1OffsetX        = obj.uiManager.vehicle1OffsetXField.Value;
            uiFields.vehicle1OffsetY        = obj.uiManager.vehicle1OffsetYField.Value;
            uiFields.rotateVehicle1         = obj.uiManager.rotateVehicle1Checkbox.Value;
            uiFields.vehicle1RotationAngle  = obj.uiManager.vehicle1RotationAngleField.Value;
            
            % Vehicle 2 offsets and rotation
            uiFields.offsetX                = obj.uiManager.offsetXField.Value;
            uiFields.offsetY                = obj.uiManager.offsetYField.Value;
            uiFields.rotateVehicle2         = obj.uiManager.rotateVehicle2Checkbox.Value;
            uiFields.rotationAngle          = obj.uiManager.rotationAngleField.Value;
            
            % Other UI fields
            uiFields.severityBoundType      = obj.uiManager.boundTypeDropdown.Value;
            uiFields.AverageTruck1Mass      = obj.uiManager.vehicle1MassField.Value;
            uiFields.AverageTruck2Mass      = obj.uiManager.vehicle2MassField.Value;
            uiFields.laneCommandsField      = obj.uiManager.laneCommandsField.Value;
            % Add more UI fields if needed

            % Serialize UI Fields and Append to XML
            uiElement = obj.structToXMLElement(docNode, 'UIFields', uiFields);
            root.appendChild(uiElement);

            %% Write the XML document to file
            try
                xmlwrite(obj.filename, docNode);
                uialert(obj.uiManager.fig, 'Configuration saved successfully.', 'Save Success');
                fprintf('Configuration saved to %s.\n', obj.filename);
            catch ME
                uialert(obj.uiManager.fig, ['Failed to save configuration: ' ME.message], 'Save Error');
                disp(['Failed to save configuration: ', ME.message]);
            end
        end

        %/**
        % * @brief Loads a simulation configuration from an XML file.
        % *
        % * Prompts the user to select an XML file, reads the configuration data, and applies it to
        % * both the UI fields and simulation configuration objects.
        % *
        % * @return None
        % *
        % * @warning If the user cancels the load operation or if the file is corrupted, the method handles the error gracefully.
        % */
        function loadConfiguration(obj)
            % Load configuration from an XML file and apply it to the UI and simulation configs.
            [file, path] = uigetfile('*.xml', 'Load Configuration');
            if isequal(file,0) || isequal(path,0)
                disp('User canceled Configuration load.');
                return;
            end
            obj.filename = fullfile(path, file);

            try
                % Read the XML document
                docNode = xmlread(obj.filename);
                root = docNode.getDocumentElement;

                %% Load UI Fields
                uiElement = root.getElementsByTagName('UIFields').item(0);
                if ~isempty(uiElement)
                    uiFields = obj.xmlElementToStruct(uiElement);

                    % ---------------------
                    % Vehicle 1 Offset X
                    % ---------------------
                    if isfield(uiFields, 'vehicle1OffsetX')
                        val = uiFields.vehicle1OffsetX;
                        if ~isnumeric(val)
                            val = str2double(val);
                        end
                        if isnan(val)
                            error('Invalid vehicle1OffsetX value in configuration.');
                        end
                        lims = obj.uiManager.vehicle1OffsetXField.Limits;
                        if val < lims(1) || val > lims(2)
                            error('vehicle1OffsetX value (%f) outside acceptable range [%f, %f].', val, lims(1), lims(2));
                        end
                        obj.uiManager.vehicle1OffsetXField.Value = val;
                        disp(['Loaded vehicle1OffsetX: ', num2str(val)]);
                    else
                        warning('vehicle1OffsetX not found in UIFields.');
                    end

                    % ---------------------
                    % Vehicle 1 Offset Y
                    % ---------------------
                    if isfield(uiFields, 'vehicle1OffsetY')
                        val = uiFields.vehicle1OffsetY;
                        if ~isnumeric(val)
                            val = str2double(val);
                        end
                        if isnan(val)
                            error('Invalid vehicle1OffsetY value in configuration.');
                        end
                        lims = obj.uiManager.vehicle1OffsetYField.Limits;
                        if val < lims(1) || val > lims(2)
                            error('vehicle1OffsetY value (%f) outside acceptable range [%f, %f].', val, lims(1), lims(2));
                        end
                        obj.uiManager.vehicle1OffsetYField.Value = val;
                        disp(['Loaded vehicle1OffsetY: ', num2str(val)]);
                    else
                        warning('vehicle1OffsetY not found in UIFields.');
                    end

                    % ---------------------
                    % Rotate Vehicle 1
                    % ---------------------
                    if isfield(uiFields, 'rotateVehicle1')
                        rotateVal = uiFields.rotateVehicle1;
                        if islogical(rotateVal)
                            rVal = rotateVal;
                        elseif isnumeric(rotateVal)
                            rVal = logical(rotateVal);
                        elseif ischar(rotateVal)
                            rVal = strcmpi(rotateVal, 'true') || strcmpi(rotateVal, '1');
                        else
                            rVal = false;
                        end
                        obj.uiManager.rotateVehicle1Checkbox.Value = rVal;
                        disp(['Loaded rotateVehicle1: ', mat2str(rVal)]);

                        if rVal
                            obj.uiManager.vehicle1RotationAngleField.Enable = 'on';
                        else
                            obj.uiManager.vehicle1RotationAngleField.Enable = 'off';
                            obj.uiManager.vehicle1RotationAngleField.Value  = 0;
                        end
                    else
                        warning('rotateVehicle1 not found in UIFields.');
                    end

                    % ---------------------
                    % Vehicle 1 Rotation Angle
                    % ---------------------
                    if isfield(uiFields, 'vehicle1RotationAngle')
                        angleVal = uiFields.vehicle1RotationAngle;
                        if ~isnumeric(angleVal)
                            angleVal = str2double(angleVal);
                        end
                        if isnan(angleVal)
                            error('Invalid vehicle1RotationAngle value in configuration.');
                        end
                        lims = obj.uiManager.vehicle1RotationAngleField.Limits;
                        if angleVal < lims(1) || angleVal > lims(2)
                            error('vehicle1RotationAngle (%f) outside acceptable range [%f, %f].', angleVal, lims(1), lims(2));
                        end
                        obj.uiManager.vehicle1RotationAngleField.Value = angleVal;
                        disp(['Loaded vehicle1RotationAngle: ', num2str(angleVal)]);
                    else
                        warning('vehicle1RotationAngle not found in UIFields.');
                    end

                    % ---------------------
                    % Vehicle 2 Offset X
                    % ---------------------
                    if isfield(uiFields, 'offsetX')
                        offsetXValue = uiFields.offsetX;
                        if ~isnumeric(offsetXValue)
                            offsetXValue = str2double(offsetXValue);
                        end
                        if isnan(offsetXValue)
                            error('Invalid offsetX value in configuration.');
                        end
                        if offsetXValue < obj.uiManager.offsetXField.Limits(1) || offsetXValue > obj.uiManager.offsetXField.Limits(2)
                            error('offsetX value (%f) is outside the acceptable range [%f, %f].', ...
                                  offsetXValue, obj.uiManager.offsetXField.Limits(1), obj.uiManager.offsetXField.Limits(2));
                        end
                        obj.uiManager.offsetXField.Value = offsetXValue;
                        disp(['Loaded offsetX: ', num2str(offsetXValue)]);
                    else
                        warning('offsetX not found in UIFields.');
                    end

                    % ---------------------
                    % Vehicle 2 Offset Y
                    % ---------------------
                    if isfield(uiFields, 'offsetY')
                        offsetYValue = uiFields.offsetY;
                        if ~isnumeric(offsetYValue)
                            offsetYValue = str2double(offsetYValue);
                        end
                        if isnan(offsetYValue)
                            error('Invalid offsetY value in configuration.');
                        end
                        if offsetYValue < obj.uiManager.offsetYField.Limits(1) || offsetYValue > obj.uiManager.offsetYField.Limits(2)
                            error('offsetY value (%f) is outside the acceptable range [%f, %f].', ...
                                  offsetYValue, obj.uiManager.offsetYField.Limits(1), obj.uiManager.offsetYField.Limits(2));
                        end
                        obj.uiManager.offsetYField.Value = offsetYValue;
                        disp(['Loaded offsetY: ', num2str(offsetYValue)]);
                    else
                        warning('offsetY not found in UIFields.');
                    end

                    % ---------------------
                    % Severity Bound Type
                    % ---------------------
                    if isfield(uiFields, 'severityBoundType')
                        severityBoundType = uiFields.severityBoundType;
                        validOptions = obj.uiManager.boundTypeDropdown.Items;
                        if ismember(severityBoundType, validOptions)
                            obj.uiManager.boundTypeDropdown.Value = severityBoundType;
                            disp(['Loaded severityBoundType: ', severityBoundType]);
                        else
                            warning('Invalid severityBoundType in configuration. Using default.');
                        end
                    else
                        warning('severityBoundType not found in UIFields.');
                    end

                    % ---------------------
                    % Average Truck 1 Mass
                    % ---------------------
                    if isfield(uiFields, 'AverageTruck1Mass')
                        avgTruck1Mass = uiFields.AverageTruck1Mass;
                        if ~isnumeric(avgTruck1Mass)
                            avgTruck1Mass = str2double(avgTruck1Mass);
                        end
                        if isnan(avgTruck1Mass)
                            error('Invalid AverageTruck1Mass value in configuration.');
                        end
                        obj.uiManager.vehicle1MassField.Value = avgTruck1Mass;
                        disp(['Loaded AverageTruck1Mass: ', num2str(avgTruck1Mass)]);
                    else
                        warning('AverageTruck1Mass not found in UIFields.');
                    end

                    % ---------------------
                    % Average Truck 2 Mass
                    % ---------------------
                    if isfield(uiFields, 'AverageTruck2Mass')
                        avgTruck2Mass = uiFields.AverageTruck2Mass;
                        if ~isnumeric(avgTruck2Mass)
                            avgTruck2Mass = str2double(avgTruck2Mass);
                        end
                        if isnan(avgTruck2Mass)
                            error('Invalid AverageTruck2Mass value in configuration.');
                        end
                        obj.uiManager.vehicle2MassField.Value = avgTruck2Mass;
                        disp(['Loaded AverageTruck2Mass: ', num2str(avgTruck2Mass)]);
                    else
                        warning('AverageTruck2Mass not found in UIFields.');
                    end

                    % ---------------------
                    % Lane Commands Field
                    % ---------------------
                    if isfield(uiFields, 'laneCommandsField')
                        laneCommands = uiFields.laneCommandsField;
                        if iscell(laneCommands)
                            obj.uiManager.laneCommandsField.Value = laneCommands;
                        elseif ischar(laneCommands) || isstring(laneCommands)
                            lines = strsplit(char(laneCommands), '\n');
                            obj.uiManager.laneCommandsField.Value = lines;
                        else
                            warning('Invalid laneCommandsField format.');
                        end
                    else
                        warning('laneCommandsField not found.');
                    end

                    % ---------------------
                    % Rotate Vehicle 2
                    % ---------------------
                    if isfield(uiFields, 'rotateVehicle2')
                        rotateVehicle2Value = uiFields.rotateVehicle2;
                        if islogical(rotateVehicle2Value)
                            rotateVehicle2 = rotateVehicle2Value;
                        elseif isnumeric(rotateVehicle2Value)
                            rotateVehicle2 = logical(rotateVehicle2Value);
                        elseif ischar(rotateVehicle2Value)
                            rotateVehicle2 = strcmpi(rotateVehicle2Value, 'true') || strcmpi(rotateVehicle2Value, '1');
                        else
                            rotateVehicle2 = false;
                        end
                        obj.uiManager.rotateVehicle2Checkbox.Value = rotateVehicle2;
                        disp(['Loaded rotateVehicle2: ', mat2str(rotateVehicle2)]);

                        % Update rotation angle field enable state
                        if rotateVehicle2
                            obj.uiManager.rotationAngleField.Enable = 'on';
                        else
                            obj.uiManager.rotationAngleField.Enable = 'off';
                            obj.uiManager.rotationAngleField.Value = 0; % Reset angle when disabled
                        end
                    else
                        warning('rotateVehicle2 not found in UIFields. Using default value.');

                        % Update rotation angle field enable state
                        if obj.uiManager.rotateVehicle2Checkbox.Value
                            obj.uiManager.rotationAngleField.Enable = 'on';
                        else
                            obj.uiManager.rotationAngleField.Enable = 'off';
                            obj.uiManager.rotationAngleField.Value = 0; % Reset angle when disabled
                        end
                    end

                    % ---------------------
                    % Rotation Angle for Vehicle 2
                    % ---------------------
                    if isfield(uiFields, 'rotationAngle')
                        rotationAngleValue = uiFields.rotationAngle;
                        if ~isnumeric(rotationAngleValue)
                            rotationAngleValue = str2double(rotationAngleValue);
                        end
                        if isnan(rotationAngleValue)
                            error('Invalid rotationAngle value in configuration.');
                        end
                        if rotationAngleValue < obj.uiManager.rotationAngleField.Limits(1) || rotationAngleValue > obj.uiManager.rotationAngleField.Limits(2)
                            error('rotationAngle value (%f) is outside the acceptable range [%f, %f].', ...
                                  rotationAngleValue, obj.uiManager.rotationAngleField.Limits(1), obj.uiManager.rotationAngleField.Limits(2));
                        end
                        obj.uiManager.rotationAngleField.Value = rotationAngleValue;
                        disp(['Loaded rotationAngle: ', num2str(rotationAngleValue)]);
                    else
                        warning('rotationAngle not found in UIFields. Using default value.');
                        % Retain current value
                    end

                    % Load additional UI fields as needed
                else
                    warning('UIFields not found in XML file.');
                end

                %% Load Vehicle Configurations

                % Load Vehicle 1 Configuration
                vehicle1Element = root.getElementsByTagName('Vehicle1').item(0);
                if ~isempty(vehicle1Element)
                    vehicle1Config = obj.xmlElementToStruct(vehicle1Element);
                    obj.truckSimConfig1.setSimulationParameters(vehicle1Config);
                    disp('Loaded Vehicle1 configuration.');
                else
                    warning('Vehicle1 configuration not found in XML file.');
                end

                % Load Vehicle 2 Configuration
                vehicle2Element = root.getElementsByTagName('Vehicle2').item(0);
                if ~isempty(vehicle2Element)
                    vehicle2Config = obj.xmlElementToStruct(vehicle2Element);
                    obj.truckSimConfig2.setSimulationParameters(vehicle2Config);
                    disp('Loaded Vehicle2 configuration.');
                else
                    warning('Vehicle2 configuration not found in XML file.');
                end

                uialert(obj.uiManager.fig, 'Configuration loaded successfully.', 'Load Success');
                fprintf('Configuration loaded from %s.\n', obj.filename);
            catch ME
                uialert(obj.uiManager.fig, ['Failed to load configuration: ' ME.message], 'Load Error');
                disp(['Failed to load configuration: ', ME.message]);
            end
        end

        %/**
        % * @brief Converts a MATLAB struct to an XML element.
        % *
        % * @param docNode The XML document node.
        % * @param elementName Name of the XML element to create.
        % * @param s The MATLAB struct to convert.
        % * @return element The created XML element.
        % */
        function element = structToXMLElement(obj, docNode, elementName, s)
            % Converts a MATLAB struct to an XML element
            element = docNode.createElement(elementName);
            fields = fieldnames(s);
            for i = 1:length(fields)
                fieldName = fields{i};
                value = s.(fieldName);

                if strcmp(fieldName, 'gearRatios') && iscell(value)
                    % Check if gearRatios is a cell array containing both Gear and Ratio
                    if length(value) == 2
                        % Assume value{1} is Gear (Nx1), value{2} is Ratio (1xN)
                        Gear = value{1};
                        Ratio = value{2};

                        if ~isnumeric(Gear) || ~isnumeric(Ratio)
                            error('gearRatios must contain numeric Gear and Ratio.');
                        end

                        if length(Gear) ~= length(Ratio)
                            error('Number of Gears and Ratios must match.');
                        end

                        Ratio = Ratio'; % Convert Ratio to Nx1

                        % Create table
                        gearRatiosTable = table(Gear, Ratio, 'VariableNames', {'Gear', 'Ratio'});
                    else
                        % Assume value is a 1xN cell array of Ratio values
                        numGears = length(value);
                        Gear = (1:numGears)';
                        Ratio = cell2mat(value)';
                        gearRatiosTable = table(Gear, Ratio, 'VariableNames', {'Gear', 'Ratio'});
                    end

                    % Ensure both Gear and Ratio are column vectors
                    Gear = gearRatiosTable.Gear;
                    Ratio = gearRatiosTable.Ratio;

                    % Create gearRatios element
                    gearRatiosElement = docNode.createElement('gearRatios');
                    for row = 1:height(gearRatiosTable)
                        gearElement = docNode.createElement('gear');

                        % Create Gear Number Element
                        gearNumberElement = docNode.createElement('Gear');
                        gearNumberElement.appendChild(docNode.createTextNode(num2str(Gear(row))));
                        gearElement.appendChild(gearNumberElement);

                        % Create Gear Ratio Element
                        gearRatioElement = docNode.createElement('Ratio');
                        gearRatioElement.appendChild(docNode.createTextNode(num2str(Ratio(row))));
                        gearElement.appendChild(gearRatioElement);

                        gearRatiosElement.appendChild(gearElement);
                    end
                    element.appendChild(gearRatiosElement);

                elseif strcmp(fieldName, 'gearRatios') && istable(value)
                    % If gearRatios is already a table
                    gearRatiosElement = docNode.createElement('gearRatios');
                    for row = 1:height(value)
                        gearElement = docNode.createElement('gear');

                        % Create Gear Number Element
                        gearNumberElement = docNode.createElement('Gear');
                        gearNumberElement.appendChild(docNode.createTextNode(num2str(value.Gear(row))));
                        gearElement.appendChild(gearNumberElement);

                        % Create Gear Ratio Element
                        gearRatioElement = docNode.createElement('Ratio');
                        gearRatioElement.appendChild(docNode.createTextNode(num2str(value.Ratio(row))));
                        gearElement.appendChild(gearRatioElement);

                        gearRatiosElement.appendChild(gearElement);
                    end
                    element.appendChild(gearRatiosElement);

                elseif strcmp(fieldName, 'gearRatios') && isnumeric(value)
                    % If gearRatios are passed as a numeric array (1xN)
                    numGears = length(value);
                    Gear = (1:numGears)';
                    Ratio = value;
                    gearRatiosTable = table(Gear, Ratio, 'VariableNames', {'Gear', 'Ratio'});

                    % Serialize gearRatios as nested elements
                    gearRatiosElement = docNode.createElement('gearRatios');
                    for row = 1:height(gearRatiosTable)
                        gearElement = docNode.createElement('gear');

                        % Create Gear Number Element
                        gearNumberElement = docNode.createElement('Gear');
                        gearNumberElement.appendChild(docNode.createTextNode(num2str(gearRatiosTable.Gear(row))));
                        gearElement.appendChild(gearNumberElement);

                        % Create Gear Ratio Element
                        gearRatioElement = docNode.createElement('Ratio');
                        gearRatioElement.appendChild(docNode.createTextNode(num2str(gearRatiosTable.Ratio(row))));
                        gearElement.appendChild(gearRatioElement);

                        gearRatiosElement.appendChild(gearElement);
                    end
                    element.appendChild(gearRatiosElement);

                elseif strcmp(fieldName, 'pressureMatrices') && isstruct(value)
                    % Serialize pressureMatrices as nested elements
                    pressureMatricesElement = docNode.createElement('pressureMatrices');
                    matrixFields = fieldnames(value);
                    for j = 1:length(matrixFields)
                        matrixName = matrixFields{j};
                        matrixValues = value.(matrixName);
                        if isnumeric(matrixValues) && isvector(matrixValues)
                            matrixElement = docNode.createElement(matrixName);
                            % Serialize as comma-separated string
                            matrixStr = strjoin(arrayfun(@num2str, matrixValues, 'UniformOutput', false), ',');
                            matrixElement.appendChild(docNode.createTextNode(matrixStr));
                            pressureMatricesElement.appendChild(matrixElement);
                        else
                            warning('Skipping pressure matrix %s as it is not a numeric vector.', matrixName);
                        end
                    end
                    element.appendChild(pressureMatricesElement);

                elseif strcmp(fieldName, 'waypoints') && isnumeric(value) && size(value,2) == 2
                    % Handle waypoints as separate Waypoint elements with X and Y
                    waypointsElement = docNode.createElement('Waypoints');
                    for w = 1:size(value, 1)
                        waypointElement = docNode.createElement('Waypoint');

                        xElement = docNode.createElement('X');
                        xElement.appendChild(docNode.createTextNode(num2str(value(w,1))));
                        waypointElement.appendChild(xElement);

                        yElement = docNode.createElement('Y');
                        yElement.appendChild(docNode.createTextNode(num2str(value(w,2))));
                        waypointElement.appendChild(yElement);

                        waypointsElement.appendChild(waypointElement);
                    end
                    element.appendChild(waypointsElement);

                elseif strcmp(fieldName, 'waypoints') && iscell(value)
                    % Handle waypoints provided as a cell array
                    numericWaypoints = cell2mat(value);
                    if isnumeric(numericWaypoints) && size(numericWaypoints,2) == 2
                        waypointsElement = docNode.createElement('Waypoints');
                        for w = 1:size(numericWaypoints, 1)
                            waypointElement = docNode.createElement('Waypoint');

                            xElement = docNode.createElement('X');
                            xElement.appendChild(docNode.createTextNode(num2str(numericWaypoints(w,1))));
                            waypointElement.appendChild(xElement);

                            yElement = docNode.createElement('Y');
                            yElement.appendChild(docNode.createTextNode(num2str(numericWaypoints(w,2))));
                            waypointElement.appendChild(yElement);

                            waypointsElement.appendChild(waypointElement);
                        end
                        element.appendChild(waypointsElement);
                    else
                        warning('Waypoints cell array could not be converted to Nx2 numeric array. Skipping waypoints.');
                    end

                elseif strcmp(fieldName, 'waypoints') && isstruct(value)
                    % Handle waypoints if provided as a struct
                    if isfield(value, 'X') && isfield(value, 'Y')
                        waypointsArray = [value.X, value.Y];
                        waypointsElement = docNode.createElement('Waypoints');
                        for w = 1:size(waypointsArray, 1)
                            waypointElement = docNode.createElement('Waypoint');

                            xElement = docNode.createElement('X');
                            xElement.appendChild(docNode.createTextNode(num2str(waypointsArray(w,1))));
                            waypointElement.appendChild(xElement);

                            yElement = docNode.createElement('Y');
                            yElement.appendChild(docNode.createTextNode(num2str(waypointsArray(w,2))));
                            waypointElement.appendChild(yElement);

                            waypointsElement.appendChild(waypointElement);
                        end
                        element.appendChild(waypointsElement);
                    else
                        warning('Waypoints struct does not contain both X and Y fields. Skipping waypoints.');
                    end

                elseif strcmp(fieldName, 'guiManager')
                    donothing = true;
                elseif isstruct(value)
                    % Recursively add child elements for nested structs
                    childElement = obj.structToXMLElement(docNode, fieldName, value);
                    element.appendChild(childElement);

                elseif istable(value)
                    % Handle other tables if necessary or skip
                    fprintf('Skipping field %s because it is a table (Excel data).\n', fieldName);

                elseif strcmp(fieldName, 'waypoints') && isempty(value)
                    % If waypoints is empty, skip
                    continue;

                elseif isnumeric(value)
                    childElement = docNode.createElement(fieldName);
                    if isscalar(value)
                        % Scalar numeric value
                        valueStr = num2str(value);
                        valueStr = strtrim(valueStr); % Remove any leading/trailing whitespace
                        fprintf('Processing numeric field: %s, Value: %s\n', fieldName, valueStr);
                        childElement.appendChild(docNode.createTextNode(valueStr));
                    else
                        % Numeric array - serialize as comma-separated string
                        % Ensure it's a row vector for consistent formatting
                        valueRow = reshape(value, 1, []);
                        valueStr = strjoin(arrayfun(@num2str, valueRow, 'UniformOutput', false), ',');
                        fprintf('Processing numeric array field: %s, Value: %s\n', fieldName, valueStr);
                        childElement.appendChild(docNode.createTextNode(valueStr));
                    end
                    element.appendChild(childElement);

                elseif islogical(value)
                    childElement = docNode.createElement(fieldName);
                    if isscalar(value)
                        % Scalar logical value
                        valueStr = mat2str(value);
                        fprintf('Processing logical field: %s, Value: %s\n', fieldName, valueStr);
                        childElement.appendChild(docNode.createTextNode(valueStr));
                    else
                        % Logical array - serialize as comma-separated string
                        valueStr = strjoin(arrayfun(@mat2str, value, 'UniformOutput', false), ',');
                        fprintf('Processing logical array field: %s, Value: %s\n', fieldName, valueStr);
                        childElement.appendChild(docNode.createTextNode(valueStr));
                    end
                    element.appendChild(childElement);

                elseif ischar(value) || isstring(value)
                    childElement = docNode.createElement(fieldName);
                    % Convert MATLAB string to char array
                    valueStr = char(value);
                    % Handle empty strings
                    if isempty(valueStr)
                        valueStr = '';
                    end
                    fprintf('Processing string field: %s, Value: %s\n', fieldName, valueStr);
                    childElement.appendChild(docNode.createTextNode(valueStr));
                    element.appendChild(childElement);
                else
                    % Handle other data types by converting to string
                    childElement = docNode.createElement(fieldName);
                    valueStr = char(string(value));
                    % Handle empty strings
                    if isempty(valueStr)
                        valueStr = '';
                    end
                    fprintf('Processing other field: %s, Value: %s\n', fieldName, valueStr);
                    childElement.appendChild(docNode.createTextNode(valueStr));
                    element.appendChild(childElement);
                end
            end
        end

        %/**
        % * @brief Converts an XML element to a MATLAB struct, handling nested elements and comma-separated values.
        % *
        % * @param element The XML element to convert.
        % * @return A MATLAB struct representing the XML data.
        % */
        function s = xmlElementToStruct(obj, element)
            s = struct();
            childNodes = element.getChildNodes();
            numChildNodes = childNodes.getLength();
            for k = 0:numChildNodes - 1
                thisNode = childNodes.item(k);
                if thisNode.getNodeType() == thisNode.ELEMENT_NODE
                    nodeName = char(thisNode.getNodeName());
                    if strcmp(nodeName, 'gearRatios')
                        % Check if 'gearRatios' has nested 'gear' elements
                        gearNodes = thisNode.getElementsByTagName('gear');
                        if gearNodes.getLength() > 0
                            % Reconstruct the gearRatios table from nested 'gear' elements
                            numGears = gearNodes.getLength();
                            Gear = zeros(numGears, 1);
                            Ratio = zeros(numGears, 1);
                            for m = 0:numGears -1
                                gearNode = gearNodes.item(m);
                                GearNode = gearNode.getElementsByTagName('Gear').item(0);
                                RatioNode = gearNode.getElementsByTagName('Ratio').item(0);
                                if ~isempty(GearNode) && ~isempty(RatioNode)
                                    Gear(m +1) = str2double(char(GearNode.getTextContent()));
                                    Ratio(m +1) = str2double(char(RatioNode.getTextContent()));
                                else
                                    error('Incomplete gear information in XML.');
                                end
                            end
                            % Optionally, validate number of gears
                            s.(nodeName) = table(Gear, Ratio, 'VariableNames', {'Gear', 'Ratio'});
                        else
                            % Handle gearRatios as a comma-separated string
                            textContent = strtrim(char(thisNode.getTextContent()));
                            if isempty(textContent)
                                s.(nodeName) = table();
                            else
                                ratioValues = str2double(strsplit(textContent, ','));
                                if any(isnan(ratioValues))
                                    error('Invalid numeric values found in gearRatios.');
                                end
                                % Assuming Gear numbers start at 1
                                Gear = (1:length(ratioValues))';
                                Ratio = ratioValues';
                                s.(nodeName) = table(Gear, Ratio, 'VariableNames', {'Gear', 'Ratio'});
                            end
                        end
                    elseif strcmp(nodeName, 'pressureMatrices')
                        % Handle pressureMatrices as nested elements
                        pressureStruct = obj.xmlElementToStruct(thisNode);
                        s.(nodeName) = pressureStruct;
                    elseif strcmp(nodeName, 'Waypoints')
                        % Handle Waypoints element
                        waypointNodes = thisNode.getElementsByTagName('Waypoint');
                        numWaypoints = waypointNodes.getLength();
                        waypoints = zeros(numWaypoints, 2);
                        for m = 0:numWaypoints - 1
                            waypointNode = waypointNodes.item(m);
                            xNode = waypointNode.getElementsByTagName('X').item(0);
                            yNode = waypointNode.getElementsByTagName('Y').item(0);
                            if ~isempty(xNode) && ~isempty(yNode)
                                xVal = str2double(char(xNode.getTextContent()));
                                yVal = str2double(char(yNode.getTextContent()));
                                if isnan(xVal) || isnan(yVal)
                                    error('Invalid waypoint coordinates in XML.');
                                end
                                waypoints(m + 1, :) = [xVal, yVal];
                            else
                                error('Incomplete waypoint information in XML.');
                            end
                        end
                        s.(nodeName) = waypoints;
                    elseif thisNode.hasChildNodes()
                        % Check if the child node is a text node
                        if thisNode.getChildNodes().getLength() == 1 && ...
                                thisNode.getFirstChild().getNodeType() == thisNode.TEXT_NODE
                            % It's a text node
                            textContent = strtrim(char(thisNode.getTextContent()));

                            % Attempt to convert to numeric array if it contains commas
                            if contains(textContent, ',')
                                strParts = strsplit(textContent, ',');
                                numericParts = str2double(strParts);
                                if all(~isnan(numericParts))
                                    s.(nodeName) = numericParts;
                                else
                                    % Handle non-numeric comma-separated values
                                    s.(nodeName) = textContent;
                                end
                            else
                                % Attempt to convert single value to numeric or logical
                                numericValue = str2double(textContent);
                                if ~isnan(numericValue)
                                    s.(nodeName) = numericValue;
                                elseif strcmpi(textContent, 'true') || strcmpi(textContent, '1')
                                    s.(nodeName) = true;
                                elseif strcmpi(textContent, 'false') || strcmpi(textContent, '0')
                                    s.(nodeName) = false;
                                else
                                    s.(nodeName) = textContent;
                                end
                            end
                        else
                            % The child node has more elements, recursively process
                            s.(nodeName) = obj.xmlElementToStruct(thisNode);
                        end
                    else
                        % The node has no children
                        s.(nodeName) = '';
                    end
                end
            end
        end

        %/**
        % * @brief Removes any fields in a struct that are tables (Excel data), excluding specified fields.
        % *
        % * @param s The struct from which to remove table fields.
        % * @param excludeFields Cell array of field names to exclude from removal.
        % * @return s The struct without the specified table fields.
        % */
        function s = removeTableFields(obj, s, excludeFields)
            if nargin < 3
                excludeFields = {};
            end
            fields = fieldnames(s);
            for i = 1:length(fields)
                fieldName = fields{i};
                if istable(s.(fieldName)) && ~ismember(fieldName, excludeFields)
                    s = rmfield(s, fieldName);
                    fprintf('Removed table field: %s\n', fieldName);
                elseif isstruct(s.(fieldName))
                    s.(fieldName) = obj.removeTableFields(s.(fieldName), excludeFields);
                end
            end
        end
    end
end
