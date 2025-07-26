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
% @file Transmission.m
% @brief Simple transmission model providing gear ratios.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}

classdef Transmission < handle
    properties
        maxGear
        gearRatios
        finalDriveRatio
        shiftUpSpeed
        shiftDownSpeed
        engineBrakeTorque
        shiftDelay
        currentGear
        lastShiftTime
        clutch             % Clutch object
        
        % *** New Properties for Shifting Mechanism ***
        shiftInProgress    % Flag indicating if a shift is in progress
        shiftType          % Type of shift: 'up' or 'down'
        % *** End of New Properties ***
        
        % *** New Properties for Moving Average Filter ***
        speedHistory       % History of recent speed measurements
        accelHistory       % History of recent acceleration measurements
        filterWindowSize   % Number of samples to include in the moving average
        % *** End of New Properties ***
    end
    
    methods
        %% Transmission constructor
        % Initializes the transmission state and shift parameters.
        %
        % @param maxGear            Highest selectable gear
        % @param gearRatios         Vector of gear ratios
        % @param finalDriveRatio    Final drive ratio
        % @param shiftUpSpeed       Speed threshold to upshift
        % @param shiftDownSpeed     Speed threshold to downshift
        % @param engineBrakeTorque  Engine braking torque
        % @param shiftDelay         Delay time between shifts
        % @param clutch             Clutch object
        % @param filterWindowSize   Averaging window for speed filter
        function obj = Transmission(maxGear, gearRatios, finalDriveRatio, ...
                                     shiftUpSpeed, shiftDownSpeed, ...
                                     engineBrakeTorque, shiftDelay, clutch, filterWindowSize)
            if nargin < 9
                error('Transmission constructor requires all 9 parameters.');
            end
            obj.maxGear = maxGear;
            obj.gearRatios = gearRatios;
            obj.finalDriveRatio = finalDriveRatio;
            obj.shiftUpSpeed = shiftUpSpeed;
            obj.shiftDownSpeed = shiftDownSpeed;
            obj.engineBrakeTorque = engineBrakeTorque;
            obj.shiftDelay = shiftDelay;
            obj.currentGear = 1; % Start at first gear
            obj.lastShiftTime = -Inf; % Initialize to negative infinity
            obj.clutch = clutch; % Initialize clutch
            
            % *** Initialize New Shifting Properties ***
            obj.shiftInProgress = false;
            obj.shiftType = '';
            % *** End of Initialization ***
            
            % *** Initialize Moving Average Filter Properties ***
            obj.filterWindowSize = filterWindowSize;
            obj.speedHistory = zeros(1, filterWindowSize);
            obj.accelHistory = zeros(1, filterWindowSize);
            % Initialize history with the first measurements to avoid startup anomalies
            obj.speedHistory(:) = 0; % Assuming initial speed is 0
            obj.accelHistory(:) = 0; % Assuming initial acceleration is 0
            % *** End of Moving Average Initialization ***
        end
        
        function [obj, desiredGear] = updateGear(obj, currentSpeed, acceleration, currentTime, dt)
            % updateGear Determines if a gear shift is needed based on speed and acceleration
            desiredGear = obj.currentGear; % Default to current gear
            
            % Update clutch state
            obj.clutch.updateClutch(dt);
            
            % *** Update Moving Average Filters ***
            obj.speedHistory = [obj.speedHistory(2:end), currentSpeed];
            obj.accelHistory = [obj.accelHistory(2:end), acceleration];
            avgSpeed = mean(obj.speedHistory);
            avgAccel = mean(obj.accelHistory);
            % *** End of Moving Average Filters ***
            
            % *** Handle Ongoing Shift ***
            if obj.shiftInProgress
                % Wait until clutch is fully disengaged before shifting
                if obj.clutch.engagementPercentage >= 1 % Clutch is fully disengaged
                    % Proceed to shift
                    if strcmp(obj.shiftType, 'up')
                        obj.currentGear = obj.currentGear + 1;
                        fprintf('Shifted up to gear %d at time %.2f seconds.\n', obj.currentGear, currentTime);
                    elseif strcmp(obj.shiftType, 'down')
                        obj.currentGear = obj.currentGear - 1;
                        fprintf('Shifted down to gear %d at time %.2f seconds.\n', obj.currentGear, currentTime);
                    end
                    obj.lastShiftTime = currentTime; % Update last shift time
                    desiredGear = obj.currentGear;   % Update desired gear
                    
                    % After shifting, engage the clutch
                    obj.clutch.engage(currentTime);
                    obj.shiftInProgress = false;
                    obj.shiftType = '';
                end
                % After initiating clutch engagement, wait until it's fully engaged before allowing another shift
                return; % Exit to prevent initiating another shift during ongoing shift
            end
            % *** End of Handling Ongoing Shift ***
            
            % *** Prevent New Shift Until Clutch is Fully Engaged ***
            if obj.clutch.engagementPercentage > 0
                % Clutch is not fully engaged yet, wait
                return;
            end
            % *** End of Clutch Engagement Check ***
            
            % *** Determine If Shift is Needed Using Averaged Values ***
            if avgAccel > 0
                % Attempt to shift up
                if obj.currentGear < obj.maxGear && avgSpeed >= obj.shiftUpSpeed(obj.currentGear)
                    if (currentTime - obj.lastShiftTime) >= obj.shiftDelay
                        % Disengage the clutch to start shifting
                        obj.clutch.disengage(currentTime);
                        obj.shiftInProgress = true;
                        obj.shiftType = 'up';
                        fprintf('Initiating shift up from gear %d at time %.2f seconds. (Avg Speed: %.2f, Avg Accel: %.2f)\n', ...
                                obj.currentGear, currentTime, avgSpeed, avgAccel);
                    end
                end
            elseif avgAccel < 0
                % Attempt to shift down
                if obj.currentGear > 1 && avgSpeed <= obj.shiftDownSpeed(obj.currentGear)
                    if (currentTime - obj.lastShiftTime) >= obj.shiftDelay
                        % Disengage the clutch to start shifting
                        obj.clutch.disengage(currentTime);
                        obj.shiftInProgress = true;
                        obj.shiftType = 'down';
                        fprintf('Initiating shift down from gear %d at time %.2f seconds. (Avg Speed: %.2f, Avg Accel: %.2f)\n', ...
                                obj.currentGear, currentTime, avgSpeed, avgAccel);
                    end
                end
            end
            % *** End of Shift Determination ***
        end
        
        function obj = shiftDown(obj, currentTime)
            % shiftDown Initiates a manual downward gear shift
            if obj.currentGear > 1
                if ~obj.shiftInProgress && (currentTime - obj.lastShiftTime) >= obj.shiftDelay
                    if obj.clutch.engagementPercentage <= 0 % Clutch is fully engaged
                        % Disengage the clutch to start shifting
                        obj.clutch.disengage(currentTime);
                        obj.shiftInProgress = true;
                        obj.shiftType = 'down';
                        fprintf('Initiating manual shift down from gear %d at time %.2f seconds.\n', obj.currentGear, currentTime);
                    else
                        fprintf('Cannot shift down now. Clutch is not fully engaged.\n');
                    end
                else
                    fprintf('Cannot shift down now. Either a shift is in progress or shift delay not met.\n');
                end
            else
                fprintf('Already in the lowest gear. Cannot shift down.\n');
            end
        end
        
        function obj = shiftUp(obj, currentTime)
            % shiftUp Initiates a manual upward gear shift
            if obj.currentGear < obj.maxGear
                if ~obj.shiftInProgress && (currentTime - obj.lastShiftTime) >= obj.shiftDelay
                    if obj.clutch.engagementPercentage <= 0 % Clutch is fully engaged
                        % Disengage the clutch to start shifting
                        obj.clutch.disengage(currentTime);
                        obj.shiftInProgress = true;
                        obj.shiftType = 'up';
                        fprintf('Initiating manual shift up from gear %d at time %.2f seconds.\n', obj.currentGear, currentTime);
                    else
                        fprintf('Cannot shift up now. Clutch is not fully engaged.\n');
                    end
                else
                    fprintf('Cannot shift up now. Either a shift is in progress or shift delay not met.\n');
                end
            else
                fprintf('Already in the highest gear. Cannot shift up.\n');
            end
        end
        
        function engineBrakeTorque = getEngineBrakeTorque(obj)
            % getEngineBrakeTorque Returns the engine braking torque
            engineBrakeTorque = obj.engineBrakeTorque;
        end
        
        % Additional methods to manually engage/disengage the clutch
        function engageClutch(obj, currentTime)
            if obj.clutch.engagementPercentage > 0 % Only engage if not already fully engaged
                obj.clutch.engage(currentTime);
            end
        end
        
        function disengageClutch(obj, currentTime)
            if obj.clutch.engagementPercentage < 1 % Only disengage if not already fully disengaged
                obj.clutch.disengage(currentTime);
            end
        end
    end
end
