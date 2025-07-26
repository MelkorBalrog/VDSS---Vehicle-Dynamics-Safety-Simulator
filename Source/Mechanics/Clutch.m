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
% @file Clutch.m
% @brief Simple clutch model controlling torque transfer.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}

classdef Clutch < handle
    properties
        isEngaged                   % Boolean indicating if the clutch is fully engaged
        maxTorque                   % Maximum torque the clutch can handle (Nm)
        engagementSpeed             % Time taken to engage the clutch from disengaged to engaged (seconds)
        disengagementSpeed          % Time taken to disengage the clutch from engaged to disengaged (seconds)
        currentTorque               % Current torque transmitted by the clutch (Nm)
        engagementPercentage            % Current clutch engagement factor (0 = fully engaged, 1 = fully disengaged)
        desiredengagementPercentage     % Desired clutch engagement factor
    end
    
    methods
        function obj = Clutch(maxTorque, engagementSpeed, disengagementSpeed)
            if nargin < 3
                error('Clutch constructor requires maxTorque, engagementSpeed, and disengagementSpeed.');
            end
            obj.maxTorque = maxTorque;
            obj.engagementSpeed = engagementSpeed;
            obj.disengagementSpeed = disengagementSpeed;
            obj.engagementPercentage = 0; % Start fully engaged
            obj.desiredengagementPercentage = 0; % Initially, the clutch is engaged
            obj.updateTorque();
            obj.isEngaged = true; % Fully engaged
            obj.currentTorque = obj.maxTorque;
        end
        
        function engage(obj, currentTime)
            % Initiate clutch engagement to 0 (fully engaged)
            obj.desiredengagementPercentage = 0;
            fprintf('Clutch engagement initiated at time %.2f seconds.\n', currentTime);
        end
        
        function disengage(obj, currentTime)
            % Initiate clutch disengagement to 1 (fully disengaged)
            obj.desiredengagementPercentage = 1;
            fprintf('Clutch disengagement initiated at time %.2f seconds.\n', currentTime);
        end
        
        function updateClutch(obj, dt)
            % Update the clutch engagement factor based on desired state and speed
            if obj.engagementPercentage < obj.desiredengagementPercentage
                % Disengaging
                rate = 10 / obj.disengagementSpeed; % per second
                obj.engagementPercentage = obj.engagementPercentage + rate * dt;
                if obj.engagementPercentage >= obj.desiredengagementPercentage
                    obj.engagementPercentage = obj.desiredengagementPercentage;
                end
            elseif obj.engagementPercentage > obj.desiredengagementPercentage
                % Engaging
                rate = 10 / obj.engagementSpeed; % per second
                obj.engagementPercentage = obj.engagementPercentage - rate * dt;
                if obj.engagementPercentage <= obj.desiredengagementPercentage
                    obj.engagementPercentage = obj.desiredengagementPercentage;
                end
            end
            
            % Clamp engagementPercentage between 0 and 1
            obj.engagementPercentage = max(0, min(1, obj.engagementPercentage));
            
            % Update isEngaged based on engagementPercentage
            if obj.engagementPercentage <= 0
                obj.isEngaged = true;
            else
                obj.isEngaged = false;
            end
            
            % Update currentTorque
            obj.updateTorque();
        end
        
        function torque = getTorque(obj)
            % Retrieve the current torque based on clutch engagement factor
            torque = obj.currentTorque;
        end
    end
    
    methods (Access = private)
        function updateTorque(obj)
            % Update the current torque based on engagementPercentage
            obj.currentTorque = obj.maxTorque * (1 - obj.engagementPercentage);
        end
    end
end
