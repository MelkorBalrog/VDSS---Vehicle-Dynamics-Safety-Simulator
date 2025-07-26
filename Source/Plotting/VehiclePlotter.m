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
% @file VehiclePlotter.m
% @brief Static utilities for drawing vehicle shapes in plots.
%        Provides helper functions for geometry calculations.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}
%/**
% * @class VehiclePlotter
% * @brief Provides static methods for plotting vehicle components in simulations.
% *
% * The VehiclePlotter class offers static utility functions to compute vehicle geometries and render them
% * within simulation plots. It handles the calculation of vehicle corners, plotting of vehicles, axles, wheels,
% * hitch joints, and centers of gravity (CoG). This class is instrumental in visualizing the spatial
% * configurations and movements of different vehicle types within the simulation environment.
% *
% * @author Miguel Marina
% * @version 1.0
% * @date 2024-10-04
% */
classdef VehiclePlotter
    methods (Static)
        %/**
        % * @brief Computes the four corners of a vehicle based on its position and orientation.
        % *
        % * This method calculates the (X, Y) coordinates of the vehicle's four corners, taking into account
        % * whether the vehicle is a tractor, passenger vehicle, or trailer. It considers the vehicle's
        % * dimensions, steering angle, and number of tires per axle to accurately represent its geometry.
        % *
        % * @param x Position of the vehicle along the X-axis (center for tractor/passenger, hitch point for trailer).
        % * @param y Position of the vehicle along the Y-axis (center for tractor/passenger, hitch point for trailer).
        % * @param theta Orientation angle of the vehicle in radians.
        % * @param vehicleParams Structure containing vehicle parameters such as length, width, wheel dimensions, wheelbase, and number of axles.
        % * @param isTractor Boolean indicating if the vehicle is a tractor.
        % * @param isPassengerVehicle Boolean indicating if the vehicle is a passenger vehicle.
        % * @param steeringWheelAngle Steering angle of the vehicle's wheel in degrees.
        % * @param numTiresPerAxle Number of tires per axle.
        % *
        % * @return corners 4x2 matrix containing the (X, Y) coordinates of the vehicle's rectangle corners.
        % *
        % * @throws Error if "trailerHitchDistance" is not less than "length" for trailers.
        % */
        function corners = getVehicleCorners(x, y, theta, vehicleParams, isTractor, steeringWheelAngle, numTiresPerAxle)
            % getVehicleCorners computes the four corners of a vehicle.
            %
            % Inputs:
            %   x, y - Position of the vehicle (center for tractor/passenger, hitch point for trailer)
            %   theta - Orientation angle in radians
            %   vehicleParams - Struct containing vehicle parameters
            %   isTrailer - Boolean indicating if the vehicle is a tractor
            %   isTractor - Boolean indicating if the vehicle is a passenger vehicle
            %   steeringWheelAngle - Steering angle in degrees
            %   numTiresPerAxle - Number of tires per axle
            %
            % Output:
            %   corners - 4x2 matrix containing the (X, Y) coordinates of the rectangle corners

            % Extract parameters from vehicleParams
            length = vehicleParams.length;
            width = vehicleParams.width;
            wheelWidth = vehicleParams.wheelWidth;
            wheelHeight = vehicleParams.wheelHeight;
            wheelbase = vehicleParams.wheelbase;

            % Get hitch distance for trailer (distance from hitch to front of body)
            if isTractor
                trailerHitchDistance = 0;
            else
                if isfield(vehicleParams, 'HitchDistance')
                    trailerHitchDistance = vehicleParams.HitchDistance;
                else
                    trailerHitchDistance = 0;
                end
            end
            % Calculate the remaining length of the trailer body
            remainingLength = length - trailerHitchDistance;

            % Initialize localCorners based on vehicle type
            if ~isTractor
                % === Trailer Corner Calculation ===

                % Validate remainingLength
                if remainingLength <= 0
                    error('For trailers, "trailerHitchDistance" must be less than "length".');
                end

                % Define the rectangle corners in local trailer coordinates
                % Starting from hitch position (0,0), extend backward along X-axis by remainingLength
                % Order: front-left (hitch), front-right, back-right, back-left
                localCorners = [trailerHitchDistance, -remainingLength, -remainingLength, trailerHitchDistance;
                                -width/2, -width/2, width/2, width/2];
            else
                % === Tractor or Passenger Vehicle Corner Calculation ===
                % Front of the vehicle is at (0,0), extend back by 'length' along X-axis
                % Order: front-left, front-right, back-right, back-left
                localCorners = [0, -length, -length, 0;
                                -width/2, -width/2, width/2, width/2];
            end

            % Rotate the corners
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            rotatedCorners = R * localCorners;

            % Translate the corners to the vehicle position
            translatedCornersX = rotatedCorners(1, :) + x;
            translatedCornersY = rotatedCorners(2, :) + y;

            % Combine X and Y into a single matrix
            corners = [translatedCornersX; translatedCornersY]';

            % === Input Validation ===
            if size(corners, 1) ~= 4 || size(corners, 2) ~= 2
                error('getVehicleCorners:InvalidOutput', 'The corners matrix must be a 4x2 matrix of (X, Y) coordinates.');
            end
        end

        %/**
        % * @brief Plots the vehicle on the given axes.
        % *
        % * This method renders the vehicle's body, axles, wheels, and other components based on its position,
        % * orientation, and parameters. It differentiates between tractors, passenger vehicles, and trailers
        % * to accurately depict their respective geometries.
        % *
        % * @param ax Handle to the axes where the vehicle will be plotted.
        % * @param x Position of the vehicle along the X-axis.
        % * @param y Position of the vehicle along the Y-axis.
        % * @param theta Orientation angle of the vehicle in radians.
        % * @param vehicleParams Structure containing vehicle parameters such as length, width, wheel dimensions, wheelbase, and number of axles.
        % * @param color Color code for the vehicle plot (e.g., 'r' for red).
        % * @param isTractor Boolean indicating if the vehicle is a tractor.
        % * @param isPassengerVehicle Boolean indicating if the vehicle is a passenger vehicle.
        % * @param steeringWheelAngle Steering angle of the vehicle's wheel in degrees.
        % * @param numTiresPerAxle Number of tires per axle.
        % * @param numAxles Number of axles on the vehicle.
        % *
        % * @return None
        % */
        function h = plotVehicle(ax, x, y, theta, vehicleParams, color, isTractor, isPassengerVehicle, steeringWheelAngle, numTiresPerAxle, numAxles)
            % Extract parameters from vehicleParams
            length = vehicleParams.length;
            width = vehicleParams.width;
            wheelWidth = vehicleParams.wheelWidth;
            wheelHeight = vehicleParams.wheelHeight;
            wheelbase = vehicleParams.wheelbase;

            if (~isTractor && ~isPassengerVehicle)
                trailerHitchDistance = vehicleParams.HitchDistance;
                remainingLength = length - trailerHitchDistance;
                localCorners = [trailerHitchDistance, -remainingLength, -remainingLength, trailerHitchDistance, trailerHitchDistance;
                                -width/2, -width/2, width/2, width/2, -width/2];
            else
                localCorners = [0, -length, -length, 0, 0;
                                -width/2, -width/2, width/2, width/2, -width/2];
            end

            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            rotatedCorners = R * localCorners;

            % Translate the corners to the vehicle position
            translatedCornersX = rotatedCorners(1, :) + x;
            translatedCornersY = rotatedCorners(2, :) + y;

            % Plot the vehicle body
            hBody = plot(ax, translatedCornersX, translatedCornersY, color, 'LineWidth', 2);
            h = hBody;

            % Compute axle positions along the length of the vehicle
            axlePositions = linspace(-length/2 + wheelHeight/2, length/2 - wheelHeight/2, numAxles);

            % Plot axles and wheels
            for axleIndex = 1:numAxles
                axlePos = axlePositions(axleIndex);
                if isTractor && axleIndex == numAxles % Front axle for tractor
                    steeringAngle = steeringWheelAngle / 20;
                else
                    steeringAngle = 0;
                end
            end

            if isTractor
                if numAxles == 1
                    % Tractor rear axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - length/2 * cos(theta), y - length/2 * sin(theta), theta, -length/2 + vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                elseif numAxles == 2
                    % Tractor rear axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - length/2 * cos(theta), y - length/2 * sin(theta), theta, -length/2 + vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                    % Tractor middle axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - length/2 * cos(theta), y - length/2 * sin(theta), theta, -length/2 + 2*vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                end
                % Tractor front axle
                h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - length/2 * cos(theta), y - length/2 * sin(theta), theta, -length/2 + vehicleParams.axleSpacing + wheelbase, width, wheelWidth, wheelHeight, steeringWheelAngle, 2, vehicleParams.trackWidth)];
            elseif isPassengerVehicle
                h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - length/2 * cos(theta), y - length/2 * sin(theta), theta, -length/2 + vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                % Passenger Vehicle front axle
                h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - length/2 * cos(theta), y - length/2 * sin(theta), theta, -length/2 + vehicleParams.axleSpacing + wheelbase, width, wheelWidth, wheelHeight, steeringWheelAngle, 2, vehicleParams.trackWidth)];
            else
                if numAxles == 1
                    % Trailer rear axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                elseif numAxles == 2
                    % Trailer rear axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                    % Trailer middle axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + 2*vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                elseif numAxles == 3
                    % Trailer rear axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                    % Trailer middle axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + 2*vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                    % Trailer middle axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + 3*vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                elseif numAxles == 4
                    % Trailer rear axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                    % Trailer middle axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + 2*vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                    % Trailer middle axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + 3*vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                    % Trailer middle axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + 4*vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                elseif numAxles == 5
                    % Trailer rear axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                    % Trailer middle axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + 2*vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                    % Trailer middle axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + 3*vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                    % Trailer middle axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + 4*vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                    % Trailer middle axle
                    h = [h, VehiclePlotter.plotAxleAndWheels(ax, x - remainingLength/2 * cos(theta), y - remainingLength/2 * sin(theta), theta, -remainingLength/2 + 5*vehicleParams.axleSpacing, width, wheelWidth, wheelHeight, 0, numTiresPerAxle, vehicleParams.trackWidth)];
                end
            end

        end

        %/**
        % * @brief Plots the axle and associated wheels on the given axes.
        % *
        % * This helper method renders an axle line and the corresponding wheels based on the vehicle's orientation
        % * and position. It supports vehicles with different numbers of tires per axle.
        % *
        % * @param ax Handle to the axes where the axle and wheels will be plotted.
        % * @param x X-coordinate of the vehicle's reference point.
        % * @param y Y-coordinate of the vehicle's reference point.
        % * @param theta Orientation angle of the vehicle in radians.
        % * @param axlePos Position of the axle relative to the vehicle's reference point.
        % * @param width Width of the vehicle.
        % * @param wheelHeight Height of the wheels.
        % * @param wheelWidth Width of the wheels.
        % * @param steeringWheelAngle Steering angle of the wheel in degrees.
        % * @param numTiresPerAxle Number of tires per axle.
        % *
        % * @return None
        % */
        function h = plotAxleAndWheels(ax, x, y, theta, axlePos, width, wheelWidth, wheelHeight, steeringWheelAngle, numTiresPerAxle, trackWidth)
            % Define the axle line
            axleX = [-width/2, width/2];
            axleY = [0, 0];

            % Position the axle
            axlePosX = x + axlePos * cos(theta);
            axlePosY = y + axlePos * sin(theta);
            % Center point of the axle in global coordinates
            axleCenter = [axlePosX; axlePosY];

            % Compute the vector perpendicular to the heading
            perpVec = [cos(theta + pi/2); sin(theta + pi/2)];

            % Compute the axle end points using the perpendicular vector
            leftEnd  = [axlePosX; axlePosY] + (width/2) * perpVec;
            rightEnd = [axlePosX; axlePosY] - (width/2) * perpVec;
            rotatedAxleEnds = [leftEnd, rightEnd];

            % Plot the axle and initialize wheel handles
            hAxle = plot(ax, rotatedAxleEnds(1, :), rotatedAxleEnds(2, :), 'k', 'LineWidth', 2);
            wheelHandles = [];

            % Convert steering angle from degrees to radians
            phi = deg2rad(steeringWheelAngle);

            % Base wheel orientation
            wheelTheta = theta + phi;

            % Wheel centers for a single wheel on each side
            leftWheelCenter  = axleCenter + (trackWidth/2) * perpVec;
            rightWheelCenter = axleCenter - (trackWidth/2) * perpVec;

            % Offset distance for dual wheels: avoid overlap with small gap
            smallGap = 0.01;  % meters gap between tire edges
            dualGap = wheelHeight + smallGap;  % center-to-center distance for dual tires
            dualOffset = dualGap / 2;  % offset from side centerline for each tire
            offsetVec = dualOffset * [cos(theta + pi/2); sin(theta + pi/2)];

            if numTiresPerAxle == 2
                % Single tire on each side
                wheelHandles(end+1) = VehiclePlotter.plotWheel(ax, ...
                    leftWheelCenter(1), leftWheelCenter(2), ...
                    wheelWidth, wheelHeight, wheelTheta, 'k');
                wheelHandles(end+1) = VehiclePlotter.plotWheel(ax, ...
                    rightWheelCenter(1), rightWheelCenter(2), ...
                    wheelWidth, wheelHeight, wheelTheta, 'k');
            else
                % Dual tires on each side (e.g., 4 tires)
                wheelHandles(end+1) = VehiclePlotter.plotWheel(ax, ...
                    leftWheelCenter(1) + offsetVec(1), leftWheelCenter(2) + offsetVec(2), ...
                    wheelWidth, wheelHeight, wheelTheta, 'k');
                wheelHandles(end+1) = VehiclePlotter.plotWheel(ax, ...
                    leftWheelCenter(1) - offsetVec(1), leftWheelCenter(2) - offsetVec(2), ...
                    wheelWidth, wheelHeight, wheelTheta, 'k');
                wheelHandles(end+1) = VehiclePlotter.plotWheel(ax, ...
                    rightWheelCenter(1) + offsetVec(1), rightWheelCenter(2) + offsetVec(2), ...
                    wheelWidth, wheelHeight, wheelTheta, 'k');
                wheelHandles(end+1) = VehiclePlotter.plotWheel(ax, ...
                    rightWheelCenter(1) - offsetVec(1), rightWheelCenter(2) - offsetVec(2), ...
                    wheelWidth, wheelHeight, wheelTheta, 'k');
            end

            % Set axis equal for proper visualization
            axis equal;
            h = [hAxle, wheelHandles];
        end

        %/**
        % * @brief Plots an individual wheel on the given axes.
        % *
        % * This helper method renders a wheel as a filled rectangle based on its position, dimensions, and orientation.
        % *
        % * @param ax Handle to the axes where the wheel will be plotted.
        % * @param x X-coordinate of the wheel's center.
        % * @param y Y-coordinate of the wheel's center.
        % * @param wheelWidth Width of the wheel.
        % * @param wheelHeight Height of the wheel.
        % * @param theta Orientation angle of the wheel in radians.
        % * @param color Color code for the wheel fill.
        % *
        % * @return None
        % */
        function h = plotWheel(ax, x, y, wheelWidth, wheelHeight, theta, color)
            % Define a rectangle for the wheel
            wheelX = [-wheelWidth/2, wheelWidth/2, wheelWidth/2, -wheelWidth/2, -wheelWidth/2];
            wheelY = [-wheelHeight/2, -wheelHeight/2, wheelHeight/2, wheelHeight/2, -wheelHeight/2];

            % Rotate the wheel
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            rotatedWheel = R * [wheelX; wheelY];

            % Translate the wheel
            translatedWheelX = rotatedWheel(1, :) + x;
            translatedWheelY = rotatedWheel(2, :) + y;
            
            % Plot the wheel and return the handle for easier updates
            h = fill(ax, translatedWheelX, translatedWheelY, color);
        end

        %/**
        % * @brief Plots the hitch joint on the given axes.
        % *
        % * Marks the hitch joint point where a trailer is attached to a tractor or another vehicle.
        % *
        % * @param ax Handle to the axes where the hitch joint will be plotted.
        % * @param x X-coordinate of the vehicle's reference point.
        % * @param y Y-coordinate of the vehicle's reference point.
        % * @param theta Orientation angle of the vehicle in radians.
        % * @param length Length of the vehicle.
        % * @param color Color code for the hitch joint marker.
        % *
        % * @return None
        % */
        function plotHitchJoint(ax, x, y, theta, length, color)
            % Calculate the hitch joint position
            hitchX = x - length/2 * cos(theta);
            hitchY = y - length/2 * sin(theta);

            % Plot the hitch joint
            plot(ax, hitchX, hitchY, 'o', 'Color', color, 'MarkerFaceColor', color, 'MarkerSize', 6);
        end

        %/**
        % * @brief Plots the center of gravity (CoG) on the given axes.
        % *
        % * This method marks the CoG of the vehicle with distinct markers for better visualization.
        % *
        % * @param ax Handle to the axes where the CoG will be plotted.
        % * @param x X-coordinate of the vehicle's reference point.
        % * @param y Y-coordinate of the vehicle's reference point.
        % * @param cogX X-offset of the CoG from the vehicle's reference point.
        % * @param cogY Y-offset of the CoG from the vehicle's reference point.
        % * @param color Color code for the CoG markers.
        % *
        % * @return None
        % */
        function plotCoG(ax, x, y, cogX, cogY, color)
            % Calculate the global position of the CoG
            globalX = x + cogX;
            globalY = y + cogY;

            % Plot the CoG as a distinct marker
            plot(ax, globalX, globalY, 'x', 'Color', color, 'MarkerSize', 10, 'LineWidth', 2);
            % hold(ax, 'on');
            plot(ax, globalX, globalY, 'o', 'Color', color, 'MarkerFaceColor', color, 'MarkerSize', 6);
            % hold(ax, 'off');
        end
    end
end