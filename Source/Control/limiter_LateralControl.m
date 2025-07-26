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
% * @file limiter_LateralControl.m
% * @author Miguel Marina <karel.capek.robotics@gmail.com>
% * @date 2024-12-14
% * @brief This class limits the steering angle of a truck based on a steering limit curve loaded from an Excel file.
% *        The maximum allowable steering angle varies with speed as defined in the provided curve,
% *        ensuring that steering inputs remain within safe operational limits.
% */

classdef limiter_LateralControl
    % limiter_LateralControl Class to limit the steering angle based on a speed-dependent curve
    %
    % This class limits the steering angle of a truck based on a steering limit curve
    % provided in an Excel file. The maximum allowable steering angle varies with speed
    % according to the curve, ensuring that steering inputs remain within safe operational limits.
    
    properties
        % @var speedData
        % Vector of speeds from the steering limit curve (m/s)
        speedData
        
        % @var maxSteeringAngleData
        % Vector of maximum steering angles corresponding to speedData (degrees)
        maxSteeringAngleData
    end
    
    methods
        %/**
        % * @brief Constructor for limiter_LateralControl
        % *
        % * Initializes the limiter_LateralControl by loading the steering limit curve from an Excel file.
        % *
        % * @param excelFilePath Path to the Excel file containing the steering limit curve.
        % *                      The Excel file should have two columns:
        % *                      - Column 1: Speed (m/s)
        % *                      - Column 2: Max Steering Angle (degrees)
        % *
        % * @throws Error if the Excel file cannot be read or the data is invalid.
        % */
        function obj = limiter_LateralControl(excelFilePath)
            % limiter_LateralControl constructor
            
            % Validate input arguments
            if nargin ~= 1
                error('limiter_LateralControl requires one input argument: the path to the Excel file containing the steering limit curve.');
            end
            
            % Attempt to read the Excel file
            try
                data = readtable(excelFilePath);
            catch ME
                error('Failed to read Excel file: %s\nError message: %s', excelFilePath, ME.message);
            end
            
            % Validate that the table has at least two columns
            if size(data, 2) < 2
                error('Excel file must contain at least two columns: Speed and Max Steering Angle.');
            end
            
            % Assume first two columns are Speed and Max Steering Angle
            speed = data{:,1};
            maxAngle = data{:,2};
            
            % Validate that speed and maxAngle are numeric
            if ~isnumeric(speed) || ~isnumeric(maxAngle)
                error('Speed and Max Steering Angle columns must be numeric.');
            end
            
            % Validate that speed is non-negative
            if any(speed < 0)
                error('Speed values must be non-negative.');
            end
            
            % Validate that maxAngle is non-negative
            if any(maxAngle < 0)
                error('Max Steering Angle values must be non-negative.');
            end
            
            % Validate that speed is in ascending order
            if any(diff(speed) < 0)
                error('Speed values must be in ascending order.');
            end
            
            % Assign properties
            obj.speedData = speed;
            obj.maxSteeringAngleData = maxAngle;
        end
        
        %/**
        % * @brief Computes the limited steering angle based on current speed using the steering limit curve.
        % *
        % * This method limits the desired steering angle according to the current speed
        % * of the truck, based on the steering limit curve loaded from the Excel file.
        % *
        % * @param desiredSteeringAngle Desired steering angle input (degrees)
        % * @param currentSpeed          Current speed of the truck (m/s)
        % *
        % * @return limitedSteeringAngle  Limited steering angle (degrees)
        % *
        % * @warning If currentSpeed is negative, its absolute value is used.
        % */
        function limitedSteeringAngle = computeSteeringAngle(obj, desiredSteeringAngle, currentSpeed)
            % computeSteeringAngle limits the desired steering angle based on the current speed using the steering limit curve.
            
            % Ensure currentSpeed is non-negative
            if currentSpeed < 0
                warning('Current speed is negative. Taking absolute value.');
                currentSpeed = abs(currentSpeed);
            end
            
            % Interpolate to find the maximum allowable steering angle at the current speed
            % Use 'linear' interpolation and 'extrap' to handle speeds outside the provided data range
            maxSteeringAngle = interp1(obj.speedData, obj.maxSteeringAngleData, currentSpeed, 'linear', 'extrap');
            
            % Optionally, clamp the maxSteeringAngle to reasonable bounds if extrapolation leads to unrealistic values
            maxSteeringAngle = max(0, maxSteeringAngle); % Ensure non-negative
            
            % Limit the desired steering angle to the allowable maximum
            limitedSteeringAngle = max(-maxSteeringAngle, min(maxSteeringAngle, desiredSteeringAngle));
        end
        
        %/**
        % * @brief Plots the steering angle limitation curve.
        % *
        % * This method visualizes how the maximum steering angle varies with speed based on the loaded curve.
        % */
        function plotSteeringLimit(obj)
            % plotSteeringLimit visualizes how the maximum steering angle varies with speed based on the steering limit curve.
            
            speeds = linspace(min(obj.speedData), max(obj.speedData) * 1.2, 500); % Extend to 120% of max speed for visualization
            maxSteeringAngles = interp1(obj.speedData, obj.maxSteeringAngleData, speeds, 'linear', 'extrap');
            
            % Ensure maxSteeringAngles are non-negative
            maxSteeringAngles = max(0, maxSteeringAngles);
            
            figure;
            plot(speeds, maxSteeringAngles, 'LineWidth', 2);
            xlabel('Speed (m/s)');
            ylabel('Maximum Steering Angle (degrees)');
            title('Steering Angle Limitation vs. Speed');
            grid on;
            ylim([0, max(obj.maxSteeringAngleData) * 1.1]);
        end
    end
end
