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
classdef acc_Controller < handle
    % ACC Controller that modulates acceleration output from a PID controller
    % when approaching a curve. It reduces speed to a fraction of the
    % approach speed and predicts trailer rotation assuming simple bicycle
    % model kinematics. Deceleration begins when the vehicle is within a
    % configurable time of the next curve, determined via map localization.

    properties
        speedReduction (1,1) double {mustBePositive} = 0.25
        maxDecel (1,1) double {mustBePositive} = 6.0
        trailerLength (1,1) double {mustBeNonnegative} = 12.0
        wheelbase (1,1) double {mustBePositive} = 3.0
        decelLookaheadTime (1,1) double {mustBePositive} = 5.5
    end

    properties(Access=private)
        decelActive(1,1) logical = false
        baseSpeed(1,1) double = NaN
        prevAccel(1,1) double = 0
    end

    methods
        function obj = acc_Controller(speedReduction, maxDecel, trailerLength, wheelbase, lookaheadTime)
            if nargin >= 1 && ~isempty(speedReduction); obj.speedReduction = speedReduction; end
            if nargin >= 2 && ~isempty(maxDecel); obj.maxDecel = maxDecel; end
            if nargin >= 3 && ~isempty(trailerLength); obj.trailerLength = trailerLength; end
            if nargin >= 4 && ~isempty(wheelbase); obj.wheelbase = wheelbase; end
            if nargin >= 5 && ~isempty(lookaheadTime); obj.decelLookaheadTime = lookaheadTime; end
        end

        function [accelOut, predictedRotation] = adjust(obj, currentSpeed, pidAccel, distToCurve, turnRadius, dt)
            %ADJUST Modifies PID acceleration based on upcoming curve distance.
            %   currentSpeed  - current vehicle speed [m/s]
            %   pidAccel      - acceleration request from speed controller [m/s^2]
            %   distToCurve   - distance to first upcoming curve [m]
            %   turnRadius    - current path radius (Inf when straight)
            %   dt            - time step [s]

            if nargin < 5 || isempty(turnRadius)
                turnRadius = inf;
            end
            if nargin < 6
                dt = 0.01;
            end

            jerkLimit = 0.7 * 9.81; % m/s^3
            triggerDist = currentSpeed * obj.decelLookaheadTime;

            if ~obj.decelActive && distToCurve <= triggerDist
                % Begin ramp down and capture pre-curve speed
                obj.decelActive = true;
                obj.baseSpeed = currentSpeed;
            elseif obj.decelActive && isinf(turnRadius) && distToCurve > triggerDist
                % Resume normal control when out of the curve and the next one
                % is sufficiently far away
                obj.decelActive = false;
            elseif ~obj.decelActive
                % continuously update base speed when cruising
                obj.baseSpeed = currentSpeed;
            end

            targetSpeed = obj.speedReduction * obj.baseSpeed;

            if obj.decelActive
                if currentSpeed > targetSpeed + 0.1
                    decelNeeded = (currentSpeed^2 - targetSpeed^2) / (2*max(distToCurve, 0.01));
                    decelNeeded = min(obj.maxDecel, max(0, decelNeeded));
                    desiredAccel = -decelNeeded;
                elseif currentSpeed < targetSpeed - 0.1
                    desiredAccel = max(pidAccel, 0);
                else
                    desiredAccel = 0;
                end
            else
                desiredAccel = pidAccel;
            end

            delta = desiredAccel - obj.prevAccel;
            maxDelta = jerkLimit * dt;
            if delta > maxDelta
                accelOut = obj.prevAccel + maxDelta;
            elseif delta < -maxDelta
                accelOut = obj.prevAccel - maxDelta;
            else
                accelOut = desiredAccel;
            end
            obj.prevAccel = accelOut;

            if isinf(turnRadius)
                predictedRotation = 0;
            else
                predictedRotation = currentSpeed * dt / turnRadius * (obj.trailerLength / obj.wheelbase);
            end
        end
    end

    methods(Access=private)
        function dist = computeStoppingDistance(~, v0, v1, decel)
            if decel <= 0
                dist = inf;
                return;
            end
            if v0 <= v1
                dist = 0;
            else
                dist = (v0^2 - v1^2) / (2*decel);
            end
        end
    end
end