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
% @file VehicleCollisionSeverity.m
% @brief Estimates crash severity using 3D collision dynamics.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}
%/**
% * @class VehicleCollisionSeverity
% * @brief Simulates partially elastic 3D collisions between two vehicles and estimates crash severities based on SAE J2980.
% *
% * The VehicleCollisionSeverity class models 3D collisions between two vehicles, accounting for linear dynamics.
% * It calculates crash severities based on collision parameters and SAE J2980 thresholds, evaluating collision
% * severity from both Vehicle 1 vs. Vehicle 2 and Vehicle 2 vs. Vehicle 1 perspectives.
% *
% * @author Miguel
% * @version 2.5
% * @date 2024-10-04
 % */
% ============================================================================
% Module Interface
% Constructor:
%   obj = VehicleCollisionSeverity(m1, v1, r1, m2, v2, r2, e, n)
% Methods:
%   PerformCollision(): computes final velocities v1_final, v2_final
%   CalculateSeverity(): computes deltaV for each vehicle and maps to severity levels
%
% Inputs:
%   m1, m2: masses of vehicles
%   v1, v2: pre-collision velocities [vx; vy; vz]
%   r1, r2: r vectors to collision points
%   e: restitution coefficient
%   n: collision normal unit vector
%
% Outputs:
%   v1_final, v2_final: post-collision velocities
%   deltaV_vehicle1, deltaV_vehicle2: change in velocity in kph
%   Vehicle1Severity, Vehicle2Severity: severity ratings (S0, S1, ...)
%
% Dependencies:
%   determine_severity, get_thresholds (private)
%
% Bottlenecks:
%   - Repeated norm and dot product calculations
%   - get_thresholds uses switch/case for thresholds per collision type
%
% Proposed Optimizations:
%   - Vectorize severity threshold lookup via precomputed threshold maps
%   - Cache threshold tables for reuse across calls
%   - Precompute inverse mass terms (1/m1, 1/m2) outside PerformCollision
%   - Inline threshold selection and avoid switch-case overhead with direct indexing
% ============================================================================
% Embedded Systems Best Practices:
%   - Precompute and store mass inverses (1/m1, 1/m2) to avoid divisions in performance paths
%   - Use vectorized dot and norm operations; avoid loops for impulse and delta-V calculations
%   - Replace switch/case in get_thresholds with direct indexing into preloaded threshold arrays
%   - Pre-allocate output arrays for final velocities and severity results
%   - Inline simple arithmetic to reduce function call overhead
%   - Consolidate norm/v_rel calculations to minimize redundant computations
%   - Use single precision floats where numerical precision allows for speed gains
%   - Guard debug and fprintf calls under a compile-time or runtime verbosity flag
%   - Offload critical computations (PerformCollision, CalculateSeverity) to MEX/C modules via MATLAB Coder
classdef VehicleCollisionSeverity
    properties
        % Vehicle 1 Properties
        m1      % Mass of Vehicle 1 (kg)
        v1      % Velocity of Vehicle 1 before collision [vx; vy; vz] (m/s)
        r1      % Position vector from center of mass to collision point [x; y; z] (m)
        
        % Vehicle 2 Properties
        m2      % Mass of Vehicle 2 (kg)
        v2      % Velocity of Vehicle 2 before collision [vx; vy; vz] (m/s)
        r2      % Position vector from center of mass to collision point [x; y; z] (m)
        
        % Collision Parameters
        e       % Coefficient of restitution (0 <= e <= 1)
        n       % Collision normal unit vector [nx; ny; nz]
        
        % Severity Mapping Parameters
        J2980AssumedMaxMass = 0;
        VehicleUnderAnalysisMaxMass = 0;
        bound_type_str = 'Average'; % Default boundary type
        
        % Collision Types for Each Vehicle
        CollisionType_vehicle1 = ''; % Type of collision for Vehicle 1
        CollisionType_vehicle2 = ''; % Type of collision for Vehicle 2
        
        % Final Velocities After Collision
        v1_final    % Final velocity of Vehicle 1 after collision [vx; vy; vz] (m/s)
        v2_final    % Final velocity of Vehicle 2 after collision [vx; vy; vz] (m/s)
        m1_inv      % Precomputed inverse mass of Vehicle 1 (1/kg)
        m2_inv      % Precomputed inverse mass of Vehicle 2 (1/kg)
    end
    
    methods
        %% Constructor
        function obj = VehicleCollisionSeverity(m1, v1, r1, ...
                                                  m2, v2, r2, ...
                                                  e, n)
            % Constructor to initialize the VehicleCollisionSeverity object
            %
            % Inputs:
            %   m1, v1, r1 - Vehicle 1 parameters
            %   m2, v2, r2 - Vehicle 2 parameters
            %   e, n - Collision parameters

            if nargin ~= 8
                error('VehicleCollisionSeverity constructor requires eight input arguments.');
            end

            obj.m1 = m1;
            obj.v1 = v1;
            obj.r1 = r1;

            obj.m2 = m2;
            obj.v2 = v2;
            obj.r2 = r2;

            obj.e = e;
            obj.n = n;
            % Precompute inverse masses for performance
            obj.m1_inv = 1/obj.m1;
            obj.m2_inv = 1/obj.m2;
            
            % Initialize CollisionType for both vehicles
            obj.CollisionType_vehicle1 = '';
            obj.CollisionType_vehicle2 = '';
            
            % Initialize final velocities to initial velocities
            obj.v1_final = v1;
            obj.v2_final = v2;
        end
        
        %% Perform Collision
        function obj = PerformCollision(obj)
            %/**
            % * @brief Performs the collision calculations to determine final velocities.
            % *
            % * This method calculates the final velocities of both vehicles post-collision based on
            % * the conservation of linear momentum and the coefficient of restitution.
            % *
            % * @return obj The updated VehicleCollisionSeverity object with final velocities.
            % */
            
            % Calculate relative velocity along the normal direction
            v_rel_normal = dot(obj.v2 - obj.v1, obj.n);
            
            % If relative velocity is positive, vehicles are moving apart; no collision
            % if v_rel_normal > 0
            %     fprintf('Vehicles are moving apart. No collision response applied.\n');
            %     obj.v1_final = obj.v1;
            %     obj.v2_final = obj.v2;
            %     return;
            % end
            
            % Calculate impulse scalar using precomputed inverse masses
            J = -(1 + obj.e) * v_rel_normal / (obj.m1_inv + obj.m2_inv);
            
            % Update velocities using inverse masses
            obj.v1_final = obj.v1 + (J * obj.m1_inv) * obj.n;
            obj.v2_final = obj.v2 - (J * obj.m2_inv) * obj.n;
        end
        
        %% Calculate Severity
        function [deltaV_vehicle1, deltaV_vehicle2, Vehicle1Severity, Vehicle2Severity] = CalculateSeverity(obj)
            %/**
            % * @brief Calculates the severity of the collision for each vehicle based on delta-V thresholds.
            % *
            % * This method computes the change in velocity (delta-V) for both vehicles resulting from the collision
            % * and determines their respective severity ratings based on predefined thresholds.
            % *
            % * @return deltaV_vehicle1 Change in velocity for Vehicle 1 (kph).
            % * @return deltaV_vehicle2 Change in velocity for Vehicle 2 (kph).
            % * @return Vehicle1Severity Severity rating for Vehicle 1 ('S0', 'S1', etc.).
            % * @return Vehicle2Severity Severity rating for Vehicle 2 ('S0', 'S1', etc.).
            % */
            
            % Calculate delta-V for both vehicles
            deltaV_vehicle1 = norm(obj.v1_final - obj.v1) * 3.6; % Convert from m/s to kph
            deltaV_vehicle2 = norm(obj.v2_final - obj.v2) * 3.6; % Convert from m/s to kph
            
            % Determine collision thresholds for Vehicle 1
            thresholds_vehicle1 = obj.get_thresholds(obj.bound_type_str, obj.CollisionType_vehicle1, obj.VehicleUnderAnalysisMaxMass, obj.J2980AssumedMaxMass);
            % Determine collision thresholds for Vehicle 2
            thresholds_vehicle2 = obj.get_thresholds(obj.bound_type_str, obj.CollisionType_vehicle2, obj.VehicleUnderAnalysisMaxMass, obj.J2980AssumedMaxMass);
            
            % Determine severity ratings
            Vehicle1Severity = obj.determine_severity(deltaV_vehicle1, thresholds_vehicle1);
            Vehicle2Severity = obj.determine_severity(deltaV_vehicle2, thresholds_vehicle2);
        end
    end
    
    methods (Access = private)
        %/**
        % * @brief Retrieves the severity thresholds based on boundary type, collision type, and vehicle mass.
        % *
        % * @param bound_type_str Boundary type string ('LowerBound', 'HigherBound', or 'Average').
        % * @param collision_type Type of collision ('Head-On Collision', 'Rear-End Collision', 'Side Collision', or 'Oblique Collision').
        % * @param vehicle_mass Mass of the vehicle (kg).
        % *
        % * @return thresholds Nx2 matrix of [min, max] delta-V values in kph.
        % *
        % * @throws Error if an invalid boundary type or collision type is provided.
        % */
        function thresholds = get_thresholds(~, bound_type_str, collision_type, vehicle_mass, J2980MaxAssumedMass)
            % Retrieve the severity thresholds based on bound type, collision type, and vehicle mass
            %
            % Parameters:
            %   bound_type_str - 'LowerBound', 'HigherBound', or 'Average'
            %   collision_type - 'Head-On Collision', 'Rear-End Collision', 'Side Collision', or 'Oblique Collision'
            %   vehicle_mass - Mass of the vehicle (kg)
            %
            % Returns:
            %   thresholds - Nx2 matrix of [min, max] delta-V values in kph

            % Vectorized delta-V thresholds lookup based on SAE J2980 tables
            persistent KE_tables
            if isempty(KE_tables)
                KE_tables.LowerBound.HeadOnCollision   = [0,4;4,20;20,40;40,Inf];
                KE_tables.LowerBound.RearEndCollision  = [0,4;4,20;20,40;40,Inf];
                KE_tables.LowerBound.SideCollision     = [0,2;2,8;8,16;16,Inf];
                KE_tables.LowerBound.ObliqueCollision  = [0,3;3,14;14,28;28,Inf];
                KE_tables.HigherBound.HeadOnCollision  = [0,10;10,50;50,65;65,Inf];
                KE_tables.HigherBound.RearEndCollision = [0,10;10,50;50,60;60,Inf];
                KE_tables.HigherBound.SideCollision    = [0,3;10,30;30,40;40,Inf];
                KE_tables.HigherBound.ObliqueCollision = [0,6.5;10,40;40,50;50,Inf];
                KE_tables.Average.HeadOnCollision      = [0,7;7,35;35,52.5;52.5,Inf];
                KE_tables.Average.RearEndCollision     = [0,7;7,35;35,52.5;52.5,Inf];
                KE_tables.Average.SideCollision        = [0,2.5;2.5,6;6,39;39,Inf];
                KE_tables.Average.ObliqueCollision     = [0,4.75;4.75,20.5;20.5,47.5;47.5,Inf];
            end
            bt = bound_type_str;
            ct = regexprep(collision_type, '[- ]', '');
            baseDV = KE_tables.(bt).(ct);
            scale = sqrt(J2980MaxAssumedMass / vehicle_mass);
            thresholds = baseDV * scale;
        end

        %/**
        % * @brief Determines the severity rating based on delta-V and collision thresholds.
        % *
        % * Compares the delta-V value against the provided thresholds to assign a severity rating.
        % *
        % * @param delta_v Delta-V value in kph.
        % * @param collision_thresholds Nx2 matrix of [min, max] delta-V values in kph.
        % *
        % * @return severity Severity rating as 'S0', 'S1', etc.
        % */
        function severity = determine_severity(~, delta_v, collision_thresholds)
            % Determine severity rating based on delta-V and collision thresholds
            %
            % Parameters:
            %   delta_v - Delta-V in kph
            %   collision_thresholds - Thresholds for the collision type
            %
            % Returns:
            %   severity - Severity rating as 'S0', 'S1', etc.

            severity = 'S0'; % Default severity

            for i = 1:size(collision_thresholds, 1)
                min_val = collision_thresholds(i, 1);
                max_val = collision_thresholds(i, 2);
                if delta_v > min_val && delta_v <= max_val
                    severity = "S" + num2str(i - 1);
                    return;
                end
            end

            if delta_v > 0
                % If delta_v exceeds all thresholds, assign the highest severity
                severity = "S" + num2str(size(collision_thresholds, 1) - 1);
            end
        end
    end

    methods (Static, Access = public)
        %/**
        % * @brief Converts meters per second to kilometers per hour.
        % *
        % * @param vel Velocity in m/s.
        % *
        % * @return v_kph Velocity in km/h.
        % */
        function v_kph = ms_to_kph(vel)
            % Convert meters per second to kilometers per hour
            %
            % Parameters:
            %   vel - Velocity in m/s
            %
            % Returns:
            %   v_kph - Velocity in km/h
            v_kph = vel * 3.6;
        end

        %/**
        % * @brief Converts miles per hour to meters per second.
        % *
        % * @param vel Velocity in mph.
        % *
        % * @return v_ms Velocity in m/s.
        % */
        function v_ms = mph_to_ms(vel)
            % Convert miles per hour to meters per second
            %
            % Parameters:
            %   vel - Velocity in mph
            %
            % Returns:
            %   v_ms - Velocity in m/s
            v_ms = vel * 0.44704; % More accurate conversion
        end

        %/**
        % * @brief Calculates the coefficient of restitution based on impact velocity.
        % *
        % * Utilizes the Takeda correlation to determine the coefficient of restitution.
        % *
        % * @param impact_velocity Impact velocity in m/s.
        % *
        % * @return cor Coefficient of restitution (0 <= cor <= 1).
        % */
        function cor = get_cor_takeda(impact_velocity)
            % Calculate coefficient of restitution based on impact velocity
            %
            % Parameters:
            %   impact_velocity - Impact velocity in m/s
            %
            % Returns:
            %   cor - Coefficient of restitution (0 <= cor <= 1)
            cor = 0.574 * exp(-0.0396 * impact_velocity);
        end
    end
end
