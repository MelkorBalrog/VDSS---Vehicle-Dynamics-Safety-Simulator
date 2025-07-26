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
% @file Throttle.m
% @brief Models throttle input filtering and limits.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}

classdef Throttle < handle
    properties
        maxThrottle              % Maximum throttle value (0 to 1)
        desiredThrottle          % Desired throttle value set by driver (0 to 1)
        currentThrottle          % Current throttle value (0 to 1)
        adjustedThrottle         % Actual throttle considering clutch engagement
        maxThrottleChangeRate    % Maximum throttle change rate (units per second)
    end
    
    methods
        function obj = Throttle(maxThrottle, maxThrottleChangeRate)
            % Constructor for Throttle class
            %
            % Parameters:
            %   maxThrottle            - Maximum throttle value (0 to 1)
            %   maxThrottleChangeRate  - Maximum throttle change rate per second
            
            if nargin < 2
                maxThrottleChangeRate = 1; % Default to 1 unit per second if not specified
            end
            
            obj.maxThrottle = maxThrottle;
            obj.desiredThrottle = 0;    % Initially no throttle
            obj.currentThrottle = 0;    % Initially no throttle
            obj.adjustedThrottle = 0;   % Initially no throttle
            obj.maxThrottleChangeRate = maxThrottleChangeRate;
        end
        
        function obj = setThrottle(obj, value)
            % Set the desired throttle position
            %
            % Parameters:
            %   value - Desired throttle value (0 to maxThrottle)
            
            obj.desiredThrottle = max(0, min(value, obj.maxThrottle));
        end
        
        function obj = updateThrottle(obj, clutchEngagementPercentage, dt)
            % Gradually update the current throttle towards the desired throttle
            %
            % Parameters:
            %   clutchEngagementPercentage - Current clutch engagement factor (0 = engaged, 1 = disengaged)
            %   dt                         - Time step duration (seconds)
            
            % Calculate the maximum change in throttle for this time step
            maxChange = obj.maxThrottleChangeRate * dt;
            
            % Determine the difference between desired and current throttle
            delta = obj.desiredThrottle - obj.currentThrottle;
            
            if abs(delta) <= maxChange
                obj.currentThrottle = obj.desiredThrottle;
            else
                obj.currentThrottle = obj.currentThrottle + sign(delta) * maxChange;
            end
            
            % Adjust the actual throttle based on clutch engagement
            % When clutch is disengaged, throttle effect is reduced
            obj.adjustedThrottle = obj.currentThrottle * (1 - clutchEngagementPercentage);
        end
        
        function throttleValue = getThrottle(obj)
            % Retrieve the adjusted throttle value
            %
            % Returns:
            %   throttleValue - Adjusted throttle value considering clutch engagement
            
            throttleValue = obj.adjustedThrottle;
        end
    end
end
