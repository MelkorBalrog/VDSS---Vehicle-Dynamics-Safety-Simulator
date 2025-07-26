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
%{
% @file jerk_Controller.m
% @brief Jerk limiting controller for smooth steering and braking.
%        Ensures rate of change of acceleration and steering angle
%        stays below a specified jerk threshold.
%}

classdef jerk_Controller < handle
    % jerk_Controller Limits jerk for acceleration and steering commands.

    properties
        maxJerk      % Maximum allowed jerk (units per second^3)
        lastAccel    % Previously applied acceleration (m/s^2)
        lastSteer    % Previously applied steering angle (rad)
        lastTime     % Time of previous update (s)
    end

    methods
        function obj = jerk_Controller(maxJerk)
            % jerk_Controller constructor
            if nargin < 1
                maxJerk = 0.7 * 9.81; % default jerk limit
            end
            obj.maxJerk = maxJerk;
            obj.lastAccel = 0;
            obj.lastSteer = 0;
            obj.lastTime = 0;
        end

        function [limitedAccel, limitedSteer] = limit(obj, desiredAccel, desiredSteer, currentTime)
            % limit Apply jerk limiting to acceleration and steering angle.
            %   desiredAccel  - desired longitudinal acceleration (m/s^2)
            %   desiredSteer  - desired steering angle (rad)
            %   currentTime   - current time stamp (s)

            if obj.lastTime == 0
                dt = Inf;
            else
                dt = currentTime - obj.lastTime;
                if dt <= 0
                    dt = eps;
                end
            end

            maxDelta = obj.maxJerk * dt;

            deltaA = desiredAccel - obj.lastAccel;
            deltaA = max(-maxDelta, min(maxDelta, deltaA));
            limitedAccel = obj.lastAccel + deltaA;

            deltaSteer = desiredSteer - obj.lastSteer;
            deltaSteer = max(-maxDelta, min(maxDelta, deltaSteer));
            limitedSteer = obj.lastSteer + deltaSteer;

            obj.lastAccel = limitedAccel;
            obj.lastSteer = limitedSteer;
            obj.lastTime = currentTime;
        end

        function reset(obj)
            % reset Clear stored previous values
            obj.lastAccel = 0;
            obj.lastSteer = 0;
            obj.lastTime = 0;
        end
    end
end
