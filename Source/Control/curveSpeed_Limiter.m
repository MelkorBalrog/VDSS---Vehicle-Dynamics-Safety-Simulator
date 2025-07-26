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
 @file curveSpeed_Limiter.m
 @brief Limits commanded speed when approaching a curve.
        Applies a predefined deceleration profile when approaching a
        curve: -1 m/s^2 for 0.5 s followed by -4.5 m/s^2 for 1 s.
        After the curve it ramps the speed back up using +6 m/s^2.
%}

classdef curveSpeed_Limiter < handle
    properties
        % Reduction factor applied at the centre of the curve (0-1)
        reductionFactor
        % Time in seconds prior to a curve when ramping begins
        rampDownTime
        % Maximum acceleration used when ramping back up (m/s^2)
        rampUpAccel

        % Internal state: current reduction multiplier (0-1)
        currentFactor
    end

    methods
        function obj = curveSpeed_Limiter(reductionFactor, rampDownTime, rampUpAccel)
            if nargin < 1 || isempty(reductionFactor)
                reductionFactor = 0.75; % 25% reduction
            end
            if nargin < 2 || isempty(rampDownTime)
                rampDownTime = 1.5; % seconds (0.5s at -1 m/s^2, 1s at -4.5 m/s^2)
            end
            if nargin < 3 || isempty(rampUpAccel)
                rampUpAccel = 1; % m/s^2 for ramp up phase
            end

            obj.reductionFactor = reductionFactor;
            obj.rampDownTime    = rampDownTime;
            obj.rampUpAccel     = rampUpAccel;
            obj.currentFactor   = 1.0;
        end

        function [dist, time] = stoppingDistance(obj, speed)
            % stoppingDistance  Calculate distance and time needed to reduce
            % speed to reductionFactor * speed using the deceleration profile.

            if nargin < 2
                speed = 0;
            end

            acc1 = 1;      % m/s^2 for the first 0.5 s
            t1max = 0.5;   % seconds
            acc2 = 4.5;    % m/s^2 for the next 1 s
            t2max = 1.0;   % seconds

            deltaV = speed * (1 - obj.reductionFactor);

            if deltaV <= acc1 * t1max
                t1 = deltaV / acc1;
                dist = speed * t1 - 0.5 * acc1 * t1^2;
                time = t1;
                return;
            end

            t1 = t1max;
            v1 = speed - acc1 * t1;
            dist1 = speed * t1 - 0.5 * acc1 * t1^2;

            remaining = deltaV - acc1 * t1;
            t2 = min(remaining / acc2, t2max);
            dist2 = v1 * t2 - 0.5 * acc2 * t2^2;

            dist = dist1 + dist2;
            time = t1 + t2;
        end

        function [limitedSpeed, accelOverride] = limitSpeed(obj, currentSpeed, targetSpeed, distToCurve, inCurve, dt)
            % limitSpeed Applies ramping to the target speed based on distance
            % to the upcoming curve and whether the vehicle is currently in a
            % curve. When deceleration is required this method also returns an
            % acceleration value that should override the PID output so the
            % vehicle follows the predefined profile.
            %
            % Parameters:
            %   currentSpeed - Current vehicle speed (m/s)
            %   targetSpeed  - PID commanded speed (m/s)
            %   distToCurve  - Distance to the start of the next curve (m)
            %   inCurve      - Logical flag indicating if the vehicle is in a curve
            %   dt           - Timestep of the simulation (s)

            if nargin < 5
                error('limitSpeed requires current speed, target speed, distance to curve and curve flag.');
            end
            if nargin < 6
                dt = 0.01; % default small timestep
            end

            accelOverride = NaN; % default: no override
            
            if inCurve
                obj.currentFactor = obj.reductionFactor;
            else
                [stopDist, rampTime] = obj.stoppingDistance(targetSpeed);
                timeToCurve = distToCurve / max(currentSpeed, eps);
                if distToCurve <= stopDist && distToCurve >= 0
                    elapsed = max(0, rampTime - timeToCurve);
                    if elapsed <= 0.5
                        deltaV = 1 * elapsed;
                        accelOverride = -1;
                    elseif elapsed > 0.5 && elapsed <= 1.5
                        deltaV = 1 * 0.5 + 4.5 * min(elapsed - 0.5, 1.0);
                        accelOverride = -4.5;
                    else
                        deltaV = 1 * 0.5 + 4.5 + 6 * min(elapsed - 1.5, 1.0);
                        accelOverride = -6;
                    end
                    factor = 1 - deltaV / max(targetSpeed, eps);
                    factor = max(obj.reductionFactor, min(1, factor));
                    obj.currentFactor = min(obj.currentFactor, factor);
                else
                    rate = obj.rampUpAccel * dt / max(targetSpeed, eps);
                    obj.currentFactor = min(1, obj.currentFactor + rate);
                end
            end

            limitedSpeed = targetSpeed * obj.currentFactor;
        end
    end
end