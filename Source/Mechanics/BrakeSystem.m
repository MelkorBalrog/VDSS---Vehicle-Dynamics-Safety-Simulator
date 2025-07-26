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
% * @file BrakeSystem.m
% * @brief Models the braking behavior of a vehicle with dynamic response.
% *
% * The BrakeSystem class manages brake commands, computes braking forces,
% * and ensures that braking forces are applied smoothly over time based on system capabilities.
% *
% * @author Miguel Marina <karel.capek.robotics@gmail.com>
% * @version 2.3
% * @date 2024-11-03
% */
classdef BrakeSystem
    % BrakeSystem Models the braking behavior of a vehicle.
    %
    %   The BrakeSystem class manages brake commands, computes braking forces,
    %   and ensures that braking forces are applied smoothly over time based on system capabilities.
    
    properties
        maxBrakingForce        % Maximum total braking force (N)
        brakeEfficiency        % Brake system efficiency (0 to 1)
        brakeBias              % Brake bias percentage (Front/Rear %)
        currentBrakeForce      % Current total braking force applied (N)
        brakeType              % Type of brakes ('Disk', 'Drum', 'Air Disk', 'Air Drum')
        brakeCommand           % Brake command input (0 to 1)
        brakeResponseRate      % Maximum rate of change of braking force (N/s)
        desiredBrakingForce    % Desired total braking force input (N)
        
        % ** New Properties for Brake Bias Distribution **
        currentBrakeForceFront % Current front braking force (N)
        currentBrakeForceRear  % Current rear braking force (N)
        
        % ** Brake Type Specific Properties **
        brakeTypeProperties    % Struct containing properties for each brake type
    end
    
    methods
        function obj = BrakeSystem(maxBrakingForce, brakeEfficiency, brakeType, brakeResponseRate, brakeBias)
            % Constructor for BrakeSystem
            %
            %   obj = BrakeSystem(maxBrakingForce, brakeEfficiency, brakeType, brakeResponseRate, brakeBias)
            %
            %   Inputs:
            %       maxBrakingForce   - Maximum total braking force (N)
            %       brakeEfficiency   - Efficiency (0 to 1)
            %       brakeType         - Type of brakes ('Disk', 'Drum', 'Air Disk', 'Air Drum')
            %       brakeResponseRate - Maximum rate of change of braking force (N/s)
            %       brakeBias         - Brake bias percentage (Front/Rear %)
            
            if nargin < 5
                brakeBias = 50; % Default brake bias (50% Front / 50% Rear)
            end
            if nargin < 4
                brakeResponseRate = 5000; % Default brake response rate (N/s)
            end
            if nargin < 3
                brakeType = 'Disk'; % Default brake type
            end
            if nargin < 2
                brakeEfficiency = 1.0; % Default to 100% efficiency
            end
            if nargin < 1
                maxBrakingForce = 100000; % Default max braking force (N)
            end
            
            obj.maxBrakingForce = maxBrakingForce;
            obj.brakeEfficiency = brakeEfficiency;
            obj.brakeType = brakeType;
            obj.brakeResponseRate = brakeResponseRate;
            obj.brakeBias = obj.validateBrakeBias(brakeBias); % Validate brake bias
            obj.currentBrakeForce = 0;
            obj.brakeCommand = 0;
            obj.desiredBrakingForce = 0;
            obj.currentBrakeForceFront = 0;
            obj.currentBrakeForceRear = 0;
            
            % Initialize brake type specific properties
            obj.initializeBrakeTypeProperties();
        end
        
        function obj = setBrakeCommand(obj, command)
            % Set the brake command input
            %
            %   obj = setBrakeCommand(obj, command)
            %
            %   Inputs:
            %       command - Brake command (0 to 1)
            
            obj.brakeCommand = max(0, min(1, command)); % Clamp between 0 and 1
            maxForce = obj.maxBrakingForce * obj.brakeEfficiency;
            obj.desiredBrakingForce = obj.brakeCommand * maxForce;
        end

        function obj = setDesiredBrakingForce(obj, force)
            % Set the desired total braking force input
            %
            %   obj = setDesiredBrakingForce(obj, force)
            %
            %   Inputs:
            %       force - Desired braking force in Newtons

            maxForce = obj.maxBrakingForce * obj.brakeEfficiency;
            force = max(0, min(force, maxForce));
            obj.desiredBrakingForce = force;
            if maxForce > 0
                obj.brakeCommand = force / maxForce;
            else
                obj.brakeCommand = 0;
            end
        end
        
        function obj = setBrakeBias(obj, bias)
            % Set the brake bias percentage (Front/Rear %)
            %
            %   obj = setBrakeBias(obj, bias)
            %
            %   Inputs:
            %       bias - Brake bias percentage (0 to 100)
            
            obj.brakeBias = obj.validateBrakeBias(bias);
        end
        
        function bias = validateBrakeBias(obj, bias)
            % Validate and adjust brake bias to be within 0 to 100%
            if bias < 0
                bias = 0;
                warning('Brake bias cannot be negative. Set to 0%% (Rear).');
            elseif bias > 100
                bias = 100;
                warning('Brake bias cannot exceed 100%%. Set to 100%% (Front).');
            end
        end
        
        function obj = setBrakeType(obj, brakeType)
            % Set the brake type
            %
            %   obj = setBrakeType(obj, brakeType)
            %
            %   Inputs:
            %       brakeType - Type of brakes ('Disk', 'Drum', 'Air Disk', 'Air Drum')
            
            validBrakeTypes = {'Disk', 'Drum', 'Air Disk', 'Air Drum'};
            if any(strcmp(brakeType, validBrakeTypes))
                obj.brakeType = brakeType;
                obj.initializeBrakeTypeProperties(); % Update properties based on brake type
            else
                error('Invalid brake type. Valid options are: Disk, Drum, Air Disk, Air Drum.');
            end
        end
        
        function obj = applyBrakes(obj, dt)
            % Apply brakes smoothly based on the desired braking force and response rate
            %
            %   obj = applyBrakes(obj, dt)
            %
            %   Inputs:
            %       dt - Time step over which to apply braking (s)
            
            % Compute desired total braking force based on input braking force
            desiredTotalBrakingForce = obj.desiredBrakingForce;
            
            % Compute desired front and rear braking forces based on brake bias
            desiredBrakeForceFront = (obj.brakeBias / 100) * desiredTotalBrakingForce;
            desiredBrakeForceRear  = desiredTotalBrakingForce - desiredBrakeForceFront;
            
            % Calculate the difference between desired and current braking forces
            deltaFFront = desiredBrakeForceFront - obj.currentBrakeForceFront;
            deltaFRear  = desiredBrakeForceRear - obj.currentBrakeForceRear;
            
            % Determine the maximum change in braking force for this time step
            maxDeltaF = obj.brakeResponseRate * dt;
            
            % Limit the change to the brake response rate for front brakes
            if abs(deltaFFront) > maxDeltaF
                deltaFFront = sign(deltaFFront) * maxDeltaF;
            end
            
            % Limit the change to the brake response rate for rear brakes
            if abs(deltaFRear) > maxDeltaF
                deltaFRear = sign(deltaFRear) * maxDeltaF;
            end
            
            % Update the current braking forces
            obj.currentBrakeForceFront = obj.currentBrakeForceFront + deltaFFront;
            obj.currentBrakeForceRear  = obj.currentBrakeForceRear  + deltaFRear;
            
            % Ensure braking forces stay within [0, maxBrakingForce * brakeEfficiency]
            maxFrontBrakingForce = (obj.brakeBias / 100) * obj.maxBrakingForce * obj.brakeEfficiency;
            maxRearBrakingForce  = obj.maxBrakingForce * obj.brakeEfficiency - maxFrontBrakingForce;
            
            obj.currentBrakeForceFront = max(0, min(obj.currentBrakeForceFront, maxFrontBrakingForce));
            obj.currentBrakeForceRear  = max(0, min(obj.currentBrakeForceRear, maxRearBrakingForce));
            
            % Update the total braking force
            obj.currentBrakeForce = obj.currentBrakeForceFront + obj.currentBrakeForceRear;
        end
        
        function F_brake = computeTotalBrakingForce(obj)
            % Compute and return the total braking force
            %
            %   F_brake = computeTotalBrakingForce(obj)
            
            F_brake = obj.currentBrakeForce;
        end
        
        function F_brake_front = computeFrontBrakingForce(obj)
            % Compute and return the front braking force
            %
            %   F_brake_front = computeFrontBrakingForce(obj)
            
            F_brake_front = obj.currentBrakeForceFront;
        end
        
        function F_brake_rear = computeRearBrakingForce(obj)
            % Compute and return the rear braking force
            %
            %   F_brake_rear = computeRearBrakingForce(obj)
            
            F_brake_rear = obj.currentBrakeForceRear;
        end
        
        function F_brake = getCurrentBrakeForce(obj)
            % Get the current total braking force
            %
            %   F_brake = getCurrentBrakeForce(obj)
            
            F_brake = obj.currentBrakeForce;
        end
        
        function F_brake_front = getCurrentFrontBrakeForce(obj)
            % Get the current front braking force
            %
            %   F_brake_front = getCurrentFrontBrakeForce(obj)
            
            F_brake_front = obj.currentBrakeForceFront;
        end
        
        function F_brake_rear = getCurrentRearBrakeForce(obj)
            % Get the current rear braking force
            %
            %   F_brake_rear = getCurrentRearBrakeForce(obj)
            
            F_brake_rear = obj.currentBrakeForceRear;
        end
        
        function initializeBrakeTypeProperties(obj)
            % Initialize properties specific to each brake type
            %
            %   This method sets properties such as default brake efficiency
            %   based on the selected brake type.
            
            switch obj.brakeType
                case 'Disk'
                    obj.brakeEfficiency = 0.95; % Example efficiency for Disk brakes
                    obj.maxBrakingForce = 100000; % Example value, adjust as needed
                case 'Drum'
                    obj.brakeEfficiency = 0.85; % Example efficiency for Drum brakes
                    obj.maxBrakingForce = 80000; % Example value, adjust as needed
                case 'Air Disk'
                    obj.brakeEfficiency = 0.90; % Example efficiency for Air Disk brakes
                    obj.maxBrakingForce = 150000; % Example value, adjust as needed
                case 'Air Drum'
                    obj.brakeEfficiency = 0.80; % Example efficiency for Air Drum brakes
                    obj.maxBrakingForce = 120000; % Example value, adjust as needed
                otherwise
                    obj.brakeEfficiency = 1.0;
                    obj.maxBrakingForce = 100000;
            end
        end
    end
end
