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
% @file limiter_LongitudinalControl.m
% @brief Limits acceleration and deceleration with smoothing and ramping.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}

classdef limiter_LongitudinalControl < handle
    % limiter_LongitudinalControl Limits the maximum acceleration and deceleration
    % rates based on speed using curve tables from Excel and ramps abrupt changes.
    %
    % This class uses curve tables for maximum acceleration and deceleration
    % based on the current speed of the vehicle. It interpolates between
    % data points to determine the allowable acceleration or deceleration.
    % It also detects abrupt changes in acceleration and ramps the response
    % over a specified number of steps to ensure smooth transitions.
    % Additionally, it applies a Gaussian filter to smooth the desired
    % acceleration inputs.
    %
    % Example:
    %   accelCurve = readmatrix('accelCurve.xlsx');  % [Speed, MaxAccel]
    %   decelCurve = readmatrix('decelCurve.xlsx');  % [Speed, MaxDecel]
    %   limiter = limiter_LongitudinalControl(accelCurve, decelCurve, 30.0, 5, 5, 1.0);
    %   for i = 1:10
    %       desiredAccel = someFunction(i);  % Define your desired acceleration
    %       limitedAccel = limiter.applyLimits(desiredAccel, currentSpeed);
    %       % Use limitedAccel in your control system
    %   end
    
    properties
        % Curve-based properties
        accelCurveSpeed       % Speed values for acceleration curve (m/s)
        accelCurveValue       % Max acceleration values corresponding to accelCurveSpeed (m/s^2)
        decelCurveSpeed       % Speed values for deceleration curve (m/s)
        decelCurveValue       % Max deceleration values corresponding to decelCurveSpeed (m/s^2)
        maxSpeed              % Maximum speed for limiting (m/s)
        
        % Ramping properties
        rampWindow            % Number of steps over which to ramp acceleration
        lastAppliedAccel      % Last applied acceleration (m/s^2)
        rampTargetAccel       % Target acceleration for ramping (m/s^2)
        rampIncrement         % Increment per step for ramping (m/s^2)
        rampStepsRemaining    % Steps remaining in the ramp
        
        % Gaussian filtering properties
        gaussianWindow        % Number of samples in the Gaussian filter window
        gaussianStd           % Standard deviation of the Gaussian filter
        gaussianCoeffs        % Gaussian filter coefficients
        gaussianBuffer        % Buffer to store recent desired acceleration values
    end
    
    methods
        %% limiter_LongitudinalControl constructor
        % Creates a limiter instance with curves and smoothing parameters.
        %
        % @param accelCurve       Nx2 matrix [Speed, MaxAccel] for acceleration limits
        % @param decelCurve       Nx2 matrix [Speed, MaxDecel] for deceleration limits
        % @param maxSpeed         Maximum speed for limiting (m/s)
        % @param rampWindow       Number of steps used for ramping
        % @param gaussianWindow   Window size for Gaussian filter (odd integer)
        % @param gaussianStd      Standard deviation of Gaussian filter
        function obj = limiter_LongitudinalControl(accelCurve, decelCurve, maxSpeed, rampWindow, gaussianWindow, gaussianStd)
            
            % Validate inputs
            arguments
                accelCurve {mustBeNumeric, mustBeTwoColumnMatrix}
                decelCurve {mustBeNumeric, mustBeTwoColumnMatrix}
                maxSpeed (1,1) double {mustBePositive}
                rampWindow (1,1) double {mustBePositive, mustBeInteger}
                gaussianWindow (1,1) double {mustBePositive, mustBeInteger, mustBeOdd} = 5  % Default to 5 if not specified
                gaussianStd (1,1) double {mustBePositive} = 1.0                       % Default to 1.0 if not specified
            end
            
            % Assign curve data
            obj.accelCurveSpeed = accelCurve(:,1);
            obj.accelCurveValue = accelCurve(:,2);
            obj.decelCurveSpeed = decelCurve(:,1);
            obj.decelCurveValue = decelCurve(:,2);
            obj.maxSpeed = maxSpeed;
            
            % Initialize ramping properties
            obj.rampWindow = rampWindow;
            obj.lastAppliedAccel = 0;
            obj.rampTargetAccel = 0;
            obj.rampIncrement = 0;
            obj.rampStepsRemaining = 0;
            
            % Initialize Gaussian filter properties
            obj.gaussianWindow = gaussianWindow;
            obj.gaussianStd = gaussianStd;
            obj.gaussianCoeffs = obj.computeGaussianCoefficients(gaussianWindow, gaussianStd);
            obj.gaussianBuffer = zeros(1, gaussianWindow);
        end
        
        %% Compute Gaussian Coefficients
        function coeffs = computeGaussianCoefficients(obj, window, std)
            % computeGaussianCoefficients Computes normalized Gaussian coefficients
            %
            % Parameters:
            %   window - Number of samples in the Gaussian window (must be odd)
            %   std    - Standard deviation of the Gaussian
            %
            % Returns:
            %   coeffs - Normalized Gaussian coefficients
        
            % Ensure window is odd
            if mod(window, 2) == 0
                error('gaussianWindow must be an odd integer.');
            end
            
            halfWindow = floor(window / 2);
            x = -halfWindow:halfWindow;
            coeffs = exp(-(x.^2) / (2 * std^2));
            coeffs = coeffs / sum(coeffs);  % Normalize to sum to 1
        end
        
        %% Compute Maximum Allowable Acceleration
        function maxAccel = computeMaxAcceleration(obj, speed)
            % computeMaxAcceleration Computes the maximum allowable acceleration
            %
            % Parameters:
            %   speed - Current speed of the vehicle (m/s)
            %
            % Returns:
            %   maxAccel - Maximum allowable acceleration at the given speed (m/s^2)
            
            % Ensure speed is within [0, maxSpeed]
            speed = max(min(speed, obj.maxSpeed), 0);
            
            % Interpolate from the acceleration curve
            maxAccel = interp1(obj.accelCurveSpeed, obj.accelCurveValue, speed, 'linear', 'extrap');
        end
        
        %% Compute Maximum Allowable Deceleration
        function maxDecel = computeMaxDeceleration(obj, speed)
            % computeMaxDeceleration Computes the maximum allowable deceleration
            %
            % Parameters:
            %   speed - Current speed of the vehicle (m/s)
            %
            % Returns:
            %   maxDecel - Maximum allowable deceleration at the given speed (m/s^2)
            %              Note: This value is negative.
            
            % Ensure speed is within [0, maxSpeed]
            speed = max(min(speed, obj.maxSpeed), 0);
            
            % Interpolate from the deceleration curve
            maxDecel = interp1(obj.decelCurveSpeed, obj.decelCurveValue, speed, 'linear', 'extrap');
        end
        
        %% Apply Acceleration Limits with Gaussian Filtering and Ramping
        function limitedAccel = applyLimits(obj, desiredAccel, speed)
            % applyLimits Applies the Gaussian filter, acceleration limits, and ramps abrupt changes
            %
            % Parameters:
            %   desiredAccel - Desired acceleration (m/s^2)
            %   speed        - Current speed of the vehicle (m/s)
            %
            % Returns:
            %   limitedAccel - Acceleration after applying Gaussian filter, limits, and ramping (m/s^2)
            %
            %   Behavior:
            %       - Desired acceleration is first passed through a Gaussian filter.
            %       - If desiredAccel is positive, it's limited by the interpolated maxAccel.
            %       - If desiredAccel is negative, it's limited by the interpolated maxDecel.
            %       - Abrupt changes are detected and the response is ramped over
            %         the specified number of steps.
            
            % Apply Gaussian filter to desiredAccel
            filteredAccel = obj.applyGaussianFilter(desiredAccel);
            
            if filteredAccel >= 0
                % Apply acceleration limits
                maxAccel = obj.computeMaxAcceleration(speed);
                limitedAccel = min(filteredAccel, maxAccel);
                debugLog('limiter_LongitudinalControl: Desired Accel = %.2f m/s^2, Filtered Accel = %.2f m/s^2, Max Allowed Accel = %.2f m/s^2\n', ...
                    desiredAccel, filteredAccel, maxAccel);
            else
                % Apply deceleration limits
                maxDecel = obj.computeMaxDeceleration(speed);
                limitedAccel = max(filteredAccel, maxDecel);  % Since maxDecel is negative
                debugLog('limiter_LongitudinalControl: Desired Decel = %.2f m/s^2, Filtered Decel = %.2f m/s^2, Max Allowed Decel = %.2f m/s^2\n', ...
                    desiredAccel, filteredAccel, maxDecel);
            end
            
            % Ramping logic
            if limitedAccel ~= obj.rampTargetAccel
                % New target acceleration detected, set up ramp
                obj.rampTargetAccel = limitedAccel;
                obj.rampIncrement = (obj.rampTargetAccel - obj.lastAppliedAccel) / obj.rampWindow;
                obj.rampStepsRemaining = obj.rampWindow;
            end
            
            if obj.rampStepsRemaining > 0
                % Ramp towards target acceleration
                limitedAccel = obj.lastAppliedAccel + obj.rampIncrement;
                obj.rampStepsRemaining = obj.rampStepsRemaining - 1;
            else
                % Ramp complete, use target acceleration
                limitedAccel = obj.rampTargetAccel;
            end
            
            % Update last applied acceleration
            obj.lastAppliedAccel = limitedAccel;
        end
        
        %% Apply Gaussian Filter
        function filtered = applyGaussianFilter(obj, desiredAccel)
            % applyGaussianFilter Applies the Gaussian filter to the desired acceleration
            %
            % Parameters:
            %   desiredAccel - Desired acceleration (m/s^2)
            %
            % Returns:
            %   filtered - Filtered acceleration (m/s^2)
            
            % Shift the buffer and add the new desiredAccel
            obj.gaussianBuffer = [desiredAccel, obj.gaussianBuffer(1:end-1)];
            
            % Compute the filtered acceleration using convolution
            filtered = dot(obj.gaussianCoeffs, obj.gaussianBuffer);
        end
        
        %% Plot Acceleration and Deceleration Limits
        function plotLimits(obj)
            % plotLimits Plots the acceleration and deceleration limits as functions of speed
            
            speeds = linspace(0, obj.maxSpeed, 1000);
            maxAccels = interp1(obj.accelCurveSpeed, obj.accelCurveValue, speeds, 'linear', 'extrap');
            maxDecels = interp1(obj.decelCurveSpeed, obj.decelCurveValue, speeds, 'linear', 'extrap');
            
            figure;
            plot(speeds, maxAccels, 'b-', 'LineWidth', 2);
            hold on;
            plot(speeds, maxDecels, 'r-', 'LineWidth', 2);
            xlabel('Speed (m/s)');
            ylabel('Acceleration (m/s^2)');
            title('Acceleration and Deceleration Limits vs Speed');
            legend({'Max Acceleration', 'Max Deceleration'}, 'Location', 'Best');
            grid on;
            hold off;
        end
        
        %% Plot Gaussian Filter Response (Optional)
        function plotGaussianResponse(obj)
            % plotGaussianResponse Plots the Gaussian filter coefficients
            
            figure;
            stem(obj.gaussianCoeffs, 'filled');
            xlabel('Sample Index');
            ylabel('Coefficient Value');
            title('Gaussian Filter Coefficients');
            grid on;
        end
    end
end

% --- Helper Functions for Input Validation ---
function mustBeTwoColumnMatrix(x)
    validateattributes(x, {'numeric'}, {'2d','ncols',2}, '', '');
end

% --- Helper Functions for Input Validation ---
function mustBeNegative(x)
    validateattributes(x, {'numeric'}, {'scalar', 'negative'}, '', '');
end

function mustBeNonnegative(x)
    validateattributes(x, {'numeric'}, {'scalar', 'nonnegative'}, '', '');
end

function mustBeOdd(x)
    validateattributes(x, {'numeric'}, {'scalar', 'integer'}, '', '');
    if mod(x,2) == 0
        error('Value must be an odd integer.');
    end
end
