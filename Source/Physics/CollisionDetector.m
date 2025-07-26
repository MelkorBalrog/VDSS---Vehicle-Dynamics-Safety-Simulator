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
% @file CollisionDetector.m
% @brief Handles collision detection between rectangular vehicles using SAT.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}
%/**
% * @class CollisionDetector
% * @brief Handles collision detection between two rectangular vehicles using the Separating Axis Theorem (SAT).
% *
% * The CollisionDetector class provides methods to determine if two vehicles collide based on their corner coordinates.
% * It also identifies which vehicle is the bullet (initiating) and which is the target, encapsulating the collision
% * detection logic for modularity and reusability.
% *
% * @author Miguel
% * @version 1.2
% * @date 2024-04-27
 % */
% ============================================================================
% Embedded Systems Best Practices:
%   - Precompute and cache axis normals for each bounding box once, not per collision
%   - Use fixed-size arrays and avoid dynamic concatenation in getAxes
%   - Vectorize polygon projection using matrix multiplications
%   - Implement overlap checks with direct min/max comparisons (no temporary arrays)
%   - Remove debug traces; guard any logging under a verbosity flag
%   - Align data in column-major order for efficient memory access
%   - Consider using bitmasks for axis selection to reduce operations
%   - Offload compute-intensive loops to MEX/C for high-performance applications
% ============================================================================
% Module Interface
% Methods:
%   checkCollision(obj, corners1, corners2): returns collision boolean using SAT
%   WhoIsBulletVehicle(obj, vehicle1, vehicle2, collisionType): identifies bullet and target vehicles
%
% Dependencies:
%   getAxes (private)
%
% Bottlenecks:
%   - Loop over edges to compute axes (8 axes), dynamic concatenation
%   - unique(axes, 'rows') overhead
%   - Projection loops per axis
%
% Proposed Optimizations:
%   - Compute and store axes for static vehicle shapes to avoid repeated recomputation
%   - Vectorize projection via matrix multiplication: projection = corners * axis'
%   - Limit axes to minimal necessary (4 unique normals) rather than all edges
%   - Inline simple overlap checks with direct min/max operations avoiding intermediate arrays
% ============================================================================
classdef CollisionDetector
    % CollisionDetector Class to handle collision detection between two rectangular vehicles using the Separating Axis Theorem (SAT).
    %
    % Author: Miguel
    % Version: 1.2
    % Date: 2024-04-27
    %
    % Purpose:
    %   - Determine if two vehicles collide based on their corner coordinates.
    %   - Determine which vehicle is the bullet (initiating) and which is the target.
    %   - Encapsulate collision detection logic for modularity and reusability.
    
    methods
        %/**
        % * @brief Determines if two rectangles collide using the Separating Axis Theorem (SAT).
        % *
        % * This method checks for collisions between two rectangular vehicles by projecting their corners onto potential separating axes.
        % * If no separating axis is found, a collision is detected.
        % *
        % * @param corners1 A 4x2 matrix containing the (X, Y) coordinates of the first rectangle's corners.
        % * @param corners2 A 4x2 matrix containing the (X, Y) coordinates of the second rectangle's corners.
        % *
        % * @return collision A boolean value indicating if a collision is detected (`true`) or not (`false`).
        % *
        % * @throws Error if the input matrices do not have the correct dimensions.
        function collision = checkCollision(obj, corners1, corners2)
            % checkCollision Determines if two rectangles collide using SAT.
            %
            % Inputs:
            %   corners1 - 4x2 matrix containing the (X, Y) coordinates of the first rectangle's corners.
            %   corners2 - 4x2 matrix containing the (X, Y) coordinates of the second rectangle's corners.
            %
            % Output:
            %   collision - Boolean indicating if a collision is detected (true) or not (false).

            % Validate input dimensions
            if size(corners1, 1) ~= 4 || size(corners1, 2) ~= 2
                error('corners1 must be a 4x2 matrix of (X, Y) coordinates.');
            end
            if size(corners2, 1) ~= 4 || size(corners2, 2) ~= 2
                error('corners2 must be a 4x2 matrix of (X, Y) coordinates.');
            end

            % Get the axes (normals) to be tested
            axes = obj.getAxes(corners1, corners2);

            % Vectorized projection of both rectangles onto each axis
            proj1 = corners1 * axes';
            p1min = min(proj1, [], 1);
            p1max = max(proj1, [], 1);
            proj2 = corners2 * axes';
            p2min = min(proj2, [], 1);
            p2max = max(proj2, [], 1);

            % Check for any separating axis
            if any(p1max < p2min | p2max < p1min)
                collision = false;
                return;
            end

            % No separating axis found, collision detected
            collision = true;
        end
        
        %/**
        % * @brief Determines which vehicle is the bullet (initiating) and which is the target.
        % *
        % * This method assigns roles to two vehicles based on the collision type or their velocities.
        % * For predefined collision types (e.g., 'Tractor', 'Trailer'), it assigns roles accordingly.
        % * If the collision type is undefined, it defaults to speed-based determination.
        % *
        % * @param vehicle1 A struct containing properties of the first vehicle (e.g., mass, velocity, type).
        % * @param vehicle2 A struct containing properties of the second vehicle.
        % * @param collisionType A string indicating the type of collision (e.g., 'Tractor', 'Trailer').
        % *
        % * @return bulletVehicle A struct representing the bullet vehicle.
        % * @return targetVehicle A struct representing the target vehicle.
        % *
        % * @warning Ensure that `vehicle1` and `vehicle2` structs contain necessary fields like `velocity`.
        function [bulletVehicle, targetVehicle] = WhoIsBulletVehicle(obj, vehicle1, vehicle2, collisionType)
            % WhoIsBulletVehicle Determines which vehicle is the bullet and which is the target.
            %
            % Inputs:
            %   vehicle1 - Struct containing properties of the first vehicle (e.g., mass, velocity, type).
            %   vehicle2 - Struct containing properties of the second vehicle.
            %   collisionType - String indicating the type of collision (e.g., 'Tractor', 'Trailer').
            %
            % Outputs:
            %   bulletVehicle - Struct representing the bullet vehicle.
            %   targetVehicle - Struct representing the target vehicle.

            % Define roles based on vehicle types and collision type
            % For example, Passenger Vehicle is always the bullet
            if strcmpi(collisionType, 'Tractor') || strcmpi(collisionType, 'Trailer')
                % Assuming vehicle1 is Passenger Vehicle and vehicle2 is Tractor/Trailer
                bulletVehicle = vehicle1; % Passenger Vehicle
                targetVehicle = vehicle2; % Tractor or Trailer
            else
                % Fallback to speed-based determination if collisionType is undefined
                speed1 = norm(vehicle1.velocity(1:2)); % [vx; vy]
                speed2 = norm(vehicle2.velocity(1:2));

                if speed1 > speed2
                    bulletVehicle = vehicle1;
                    targetVehicle = vehicle2;
                else
                    bulletVehicle = vehicle2;
                    targetVehicle = vehicle1;
                end
            end

            % Optional: Additional criteria can be added here if needed
            % For example, consider mass, direction, etc.
        end
    end

    methods (Access = private)
        %/**
        % * @brief Computes the set of unique axes (normals) to test for the Separating Axis Theorem (SAT).
        % *
        % * This method extracts the normals of all edges from both rectangles and removes duplicate axes
        % * to optimize performance during collision detection.
        % *
        % * @param corners1 A 4x2 matrix of the first rectangle's corners.
        % * @param corners2 A 4x2 matrix of the second rectangle's corners.
        % *
        % * @return axes An Nx2 matrix of unit vectors representing the axes to test.
        % */
        function axes = getAxes(obj, corners1, corners2)
            % getAxes Computes the set of unique axes (normals) to test for SAT.
            %
            % Inputs:
            %   corners1 - 4x2 matrix of the first rectangle's corners.
            %   corners2 - 4x2 matrix of the second rectangle's corners.
            %
            % Output:
            %   axes - Nx2 matrix of unit vectors representing the axes to test.

            % Compute edge normals for both rectangles
            edges1 = corners1([2:4,1], :) - corners1;
            normals1 = [-edges1(:,2), edges1(:,1)];
            edges2 = corners2([2:4,1], :) - corners2;
            normals2 = [-edges2(:,2), edges2(:,1)];

            % Combine and normalize, removing zero-length vectors
            axes_raw = [normals1; normals2];
            lens = sqrt(sum(axes_raw.^2, 2));
            valid = lens > 0;
            axes_norm = axes_raw(valid, :) ./ lens(valid);

            % Canonicalize axis directions (ensure unique representation)
            flip_idx = axes_norm(:,1) < 0 | (abs(axes_norm(:,1)) < eps & axes_norm(:,2) < 0);
            axes_norm(flip_idx, :) = -axes_norm(flip_idx, :);

            % Obtain unique axes (rows)
            axes = unique(axes_norm, 'rows');
        end

        %/**
        % * @brief Projects all corners of a polygon onto an axis.
        % *
        % * This method calculates the minimum and maximum scalar projections of a polygon's corners onto a given axis.
        % *
        % * @param corners A Nx2 matrix of (X, Y) coordinates.
        % * @param axis A 1x2 unit vector representing the axis.
        % *
        % * @return projection A 1x2 vector [min, max] of projections.
        % */
        function projection = projectPolygon(obj, corners, axis)
            % projectPolygon Projects all corners of a polygon onto an axis.
            %
            % Inputs:
            %   corners - Nx2 matrix of (X, Y) coordinates.
            %   axis    - 1x2 unit vector representing the axis.
            %
            % Output:
            %   projection - 1x2 vector [min, max] of projections.

            projections = corners * axis';
            projection = [min(projections), max(projections)];
        end

        %/**
        % * @brief Checks if two projections on an axis overlap.
        % *
        % * This method determines whether two scalar projections on the same axis intersect, indicating
        % * a possible collision along that axis.
        % *
        % * @param proj1 A 1x2 vector [min, max] of the first projection.
        % * @param proj2 A 1x2 vector [min, max] of the second projection.
        % *
        % * @return isOverlap A boolean indicating overlap (`true`) or no overlap (`false`).
        % */
        function isOverlap = overlaps(obj, proj1, proj2)
            % overlaps Checks if two projections on an axis overlap.
            %
            % Inputs:
            %   proj1 - 1x2 vector [min, max] of the first projection.
            %   proj2 - 1x2 vector [min, max] of the second projection.
            %
            % Output:
            %   isOverlap - Boolean indicating overlap (true) or no overlap (false).

            isOverlap = ~(proj1(2) < proj2(1) || proj2(2) < proj1(1));
        end
    end
end
