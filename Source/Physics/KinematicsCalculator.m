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
% @file KinematicsCalculator.m
% @brief Performs kinematic calculations for vehicle dynamics, including roll dynamics.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}
%/**
% * @class KinematicsCalculator
% * @brief Performs kinematic calculations for vehicle dynamics, including roll dynamics.
% *
% * The KinematicsCalculator class provides methods to calculate distances, final velocities,
% * angular rates, and rotation matrices based on the vehicle's state. It also computes roll
% * angles considering lateral accelerations and roll moments. Additionally, it manages separate
% * roll dynamics for the tractor and trailer.
% *
% * @author Miguel
% * @version 2.0
% * @date 2024-10-22
 % */
% ============================================================================
% Module Interface
% Methods:
%   KinematicsCalculator: constructor initializes roll and lateral acceleration parameters
%   calculateKinematics: compute distances, velocities, rotation matrices (example method names)
%   updateRollDynamics: compute roll acceleration and update roll angle/rate
%   getLateralAcceleration: returns lateralAcceleration property
% Properties:
%   h_CoG_tractor, h_CoG_trailer, K_roll_tractor, D_roll_tractor, I_roll_tractor, K_roll_trailer, D_roll_trailer, I_roll_trailer, dt, forceCalculator, h_CoG, lateralAcceleration
% Dependencies:
%   ForceCalculator: for forces/moments
% Bottlenecks:
%   - Loop-based inertia calculation for roll dynamics (could be vectorized)
%   - Frequent printing/log statements hamper performance
% Proposed Optimizations:
%   - Vectorize inertia and cg calculations using matrix operations
%   - Remove or gate debug fprintf calls under a verbosity flag
%   - Combine repeated formulas into precomputed lookup tables for roll responses
% ============================================================================
% Embedded Systems Best Practices:
%   - Pre-allocate all data structures and reuse buffers
%   - Replace looped computations with vectorized matrix operations
%   - Gate debug/log outputs under a runtime verbosity flag
%   - Inline simple kinematic calculations to reduce function calls
%   - Use lookup tables for sine and cosine computations
%   - Avoid dynamic resizing of arrays; use fixed-size containers
%   - Consider using fixed-point data types for limited-precision needs
%   - Leverage MATLAB Coder for generating optimized compiled code
%   - Minimize branching and use logical indexing in main loops
% ============================================================================
classdef KinematicsCalculator
    properties
        %/**
        % * @property h_CoG_tractor
        % * @brief Center of gravity height of the tractor (meters).
        % *
        % * The vertical position of the tractor's center of gravity, influencing rollover dynamics.
        % */
        h_CoG_tractor               % Center of gravity height of the tractor (m)
        
        %/**
        % * @property h_CoG_trailer
        % * @brief Center of gravity height of the trailer (meters).
        % *
        % * The vertical position of the trailer's center of gravity, influencing rollover dynamics.
        % */
        h_CoG_trailer               % Center of gravity height of the trailer (m)
        
        %/**
        % * @property lateralAccelerationTractor
        % * @brief Lateral acceleration of the tractor (m/s²).
        % *
        % * The acceleration experienced by the tractor in the lateral (sideways) direction.
        % */
        lateralAccelerationTractor  % Lateral acceleration of the tractor (m/s²)
        
        %/**
        % * @property lateralAccelerationTrailer
        % * @brief Lateral acceleration of the trailer (m/s²).
        % *
        % * The acceleration experienced by the trailer in the lateral (sideways) direction.
        % */
        lateralAccelerationTrailer  % Lateral acceleration of the trailer (m/s²)
        
        %/**
        % * @property rollAngleTractor
        % * @brief Roll angle of the tractor (radians).
        % *
        % * Represents the current roll angle of the tractor.
        % */
        rollAngleTractor            % Roll angle of the tractor (rad)
        
        %/**
        % * @property rollAngleTrailer
        % * @brief Roll angle of the trailer (radians).
        % *
        % * Represents the current roll angle of the trailer.
        % */
        rollAngleTrailer            % Roll angle of the trailer (rad)
        
        %/**
        % * @property rollRateTractor
        % * @brief Roll rate of the tractor (radians per second).
        % *
        % * Represents the current roll rate of the tractor.
        % */
        rollRateTractor             % Roll rate of the tractor (rad/s)
        
        %/**
        % * @property rollRateTrailer
        % * @brief Roll rate of the trailer (radians per second).
        % *
        % * Represents the current roll rate of the trailer.
        % */
        rollRateTrailer             % Roll rate of the trailer (rad/s)
        
        %/**
        % * @property K_roll_tractor
        % * @brief Roll stiffness of the tractor (N·m/rad).
        % *
        % * Determines the stiffness of the tractor's roll response.
        % */
        K_roll_tractor              % Roll stiffness of the tractor (N·m/rad)
        
        %/**
        % * @property D_roll_tractor
        % * @brief Roll damping coefficient of the tractor (N·m·s/rad).
        % *
        % * Determines the damping effect in the tractor's roll dynamics.
        % */
        D_roll_tractor              % Roll damping of the tractor (N·m·s/rad)
        
        %/**
        % * @property I_roll_tractor
        % * @brief Roll inertia of the tractor (kg·m²).
        % *
        % * Represents the tractor's resistance to roll acceleration.
        % */
        I_roll_tractor              % Roll inertia of the tractor (kg·m²)
        
        %/**
        % * @property K_roll_trailer
        % * @brief Roll stiffness of the trailer (N·m/rad).
        % *
        % * Determines the stiffness of the trailer's roll response.
        % */
        K_roll_trailer              % Roll stiffness of the trailer (N·m/rad)
        
        %/**
        % * @property D_roll_trailer
        % * @brief Roll damping coefficient of the trailer (N·m·s/rad).
        % *
        % * Determines the damping effect in the trailer's roll dynamics.
        % */
        D_roll_trailer              % Roll damping of the trailer (N·m·s/rad)
        
        %/**
        % * @property I_roll_trailer
        % * @brief Roll inertia of the trailer (kg·m²).
        % *
        % * Represents the trailer's resistance to roll acceleration.
        % */
        I_roll_trailer              % Roll inertia of the trailer (kg·m²)
        
        %/**
        % * @property dt
        % * @brief Time step for integration (seconds).
        % *
        % * Determines the discrete time step used in roll dynamics integration.
        % */
        dt                          % Time step for integration (s)
        
        %/**
        % * @property forceCalculator
        % * @brief Instance of ForceCalculator.
        % *
        % * Used to retrieve forces and moments acting on the tractor and trailer.
        % */
        forceCalculator             % Instance of ForceCalculator
        
        %/**
        % * @property h_CoG
        % * @brief Center of gravity height (m).
        % *
        % * The vertical position of the vehicle's center of gravity, influencing rollover dynamics.
        % */
        h_CoG                       % Center of gravity height (m)
        
        %/**
        % * @property lateralAcceleration
        % * @brief Lateral acceleration (m/s²).
        % *
        % * The acceleration experienced by the vehicle in the lateral (sideways) direction.
        % */
        lateralAcceleration         % Lateral acceleration (m/s²)
        
        % Add other original properties as needed
    end
    
    methods
        %/**
        % * @brief Constructor to initialize the KinematicsCalculator.
        % *
        % * Initializes a new instance of the KinematicsCalculator class with the specified center of gravity heights
        % * and roll dynamics parameters for both tractor and trailer.
        % *
        % * @param forceCalc Instance of the ForceCalculator class.
        % * @param h_CoG_tractor (Optional) Height of the tractor's center of gravity (meters). Defaults to 1.5 meters if not provided.
        % * @param h_CoG_trailer (Optional) Height of the trailer's center of gravity (meters). Defaults to 2.0 meters if not provided.
        % * @param K_roll_tractor (Optional) Roll stiffness of the tractor (N·m/rad). Defaults to 200,000 N·m/rad if not provided.
        % * @param D_roll_tractor (Optional) Roll damping coefficient of the tractor (N·m·s/rad). Defaults to 5,000 N·m·s/rad if not provided.
        % * @param I_roll_tractor (Optional) Roll inertia of the tractor (kg·m²). Defaults to 5,000 kg·m² if not provided.
        % * @param K_roll_trailer (Optional) Roll stiffness of the trailer (N·m/rad). Defaults to 150,000 N·m/rad if not provided.
        % * @param D_roll_trailer (Optional) Roll damping coefficient of the trailer (N·m·s/rad). Defaults to 4,000 N·m·s/rad if not provided.
        % * @param I_roll_trailer (Optional) Roll inertia of the trailer (kg·m²). Defaults to 8,000 kg·m² if not provided.
        % * @param dt (Optional) Time step for integration (seconds). Defaults to 0.01 seconds if not provided.
        % *
        % * @throws None
        % */
        function obj = KinematicsCalculator(forceCalc, h_CoG_tractor, h_CoG_trailer, K_roll_tractor, D_roll_tractor, I_roll_tractor, K_roll_trailer, D_roll_trailer, I_roll_trailer, dt)
            % Initialize center of gravity heights
            if isempty(h_CoG_tractor)
                obj.h_CoG_tractor = 1.5; % Default value
                debugLog('h_CoG_tractor not provided. Using default: %.2f meters\n', obj.h_CoG_tractor);
            else
                obj.h_CoG_tractor = h_CoG_tractor;
                debugLog('h_CoG_tractor set to: %.2f meters\n', obj.h_CoG_tractor);
            end
            
            if isempty(h_CoG_trailer)
                obj.h_CoG_trailer = 2.0; % Default value
                debugLog('h_CoG_trailer not provided. Using default: %.2f meters\n', obj.h_CoG_trailer);
            else
                obj.h_CoG_trailer = h_CoG_trailer;
                debugLog('h_CoG_trailer set to: %.2f meters\n', obj.h_CoG_trailer);
            end
            
            % Initialize roll dynamics parameters for tractor
            if isempty(K_roll_tractor)
                obj.K_roll_tractor = 200000; % Default value (N·m/rad)
                debugLog('K_roll_tractor not provided. Using default: %.2f N·m/rad\n', obj.K_roll_tractor);
            else
                obj.K_roll_tractor = K_roll_tractor;
                debugLog('K_roll_tractor set to: %.2f N·m/rad\n', obj.K_roll_tractor);
            end
            
            if isempty(D_roll_tractor)
                obj.D_roll_tractor = 5000; % Default value (N·m·s/rad)
                debugLog('D_roll_tractor not provided. Using default: %.2f N·m·s/rad\n', obj.D_roll_tractor);
            else
                obj.D_roll_tractor = D_roll_tractor;
                debugLog('D_roll_tractor set to: %.2f N·m·s/rad\n', obj.D_roll_tractor);
            end
            
            if isempty(I_roll_tractor)
                obj.I_roll_tractor = 5000; % Default value (kg·m²)
                debugLog('I_roll_tractor not provided. Using default: %.2f kg·m²\n', obj.I_roll_tractor);
            else
                obj.I_roll_tractor = I_roll_tractor;
                debugLog('I_roll_tractor set to: %.2f kg·m²\n', obj.I_roll_tractor);
            end
            
            % Initialize roll dynamics parameters for trailer
            if isempty(K_roll_trailer)
                obj.K_roll_trailer = 150000; % Default value (N·m/rad)
                debugLog('K_roll_trailer not provided. Using default: %.2f N·m/rad\n', obj.K_roll_trailer);
            else
                obj.K_roll_trailer = K_roll_trailer;
                debugLog('K_roll_trailer set to: %.2f N·m/rad\n', obj.K_roll_trailer);
            end
            
            if isempty(D_roll_trailer)
                obj.D_roll_trailer = 4000; % Default value (N·m·s/rad)
                debugLog('D_roll_trailer not provided. Using default: %.2f N·m·s/rad\n', obj.D_roll_trailer);
            else
                obj.D_roll_trailer = D_roll_trailer;
                debugLog('D_roll_trailer not provided. Using default: %.2f N·m·s/rad\n', obj.D_roll_trailer);
            end
            
            if isempty(I_roll_trailer)
                obj.I_roll_trailer = 8000; % Default value (kg·m²)
                debugLog('I_roll_trailer not provided. Using default: %.2f kg·m²\n', obj.I_roll_trailer);
            else
                obj.I_roll_trailer = I_roll_trailer;
                debugLog('I_roll_trailer set to: %.2f kg·m²\n', obj.I_roll_trailer);
            end
            
            if isempty(dt)
                obj.dt = 0.01; % Default time step
                debugLog('dt not provided. Using default: %.4f seconds\n', obj.dt);
            else
                obj.dt = dt;
                debugLog('dt set to: %.4f seconds\n', obj.dt);
            end
            
            % Assign ForceCalculator instance
            obj.forceCalculator = forceCalc;
            
            % Initialize lateral accelerations and roll states
            obj.lateralAccelerationTractor = 0;
            obj.lateralAccelerationTrailer = 0;
            obj.rollAngleTractor = 0;
            obj.rollAngleTrailer = 0;
            obj.rollRateTractor = 0;
            obj.rollRateTrailer = 0;
            
            % Initialize original properties
            obj.h_CoG = h_CoG_tractor; % Assuming h_CoG refers to tractor's CoG
            obj.lateralAcceleration = 0; % Original lateralAcceleration remains, can be used if needed
            
            % Add other original initializations as needed
        end
        
        %/**
        % * @brief Calculates the distance traveled under constant acceleration.
        % *
        % * This method computes the distance traveled by the vehicle based on the initial velocity,
        % * constant acceleration, and time using the kinematic equation:
        % * distance = initialVelocity * time + 0.5 * acceleration * time².
        % *
        % * @param initialVelocity Initial velocity (m/s).
        % * @param acceleration Constant acceleration (m/s²).
        % * @param time Time duration (seconds).
        % *
        % * @return distance The calculated distance traveled (meters).
        % *
        % * @warning None
        % */
        function distance = calculateDistance(obj, initialVelocity, acceleration, time)
            distance = initialVelocity * time + 0.5 * acceleration * time.^2;
        end
        
        %/**
        % * @brief Calculates the final velocity after a given time under constant acceleration.
        % *
        % * This method computes the final velocity based on the initial velocity, constant acceleration,
        % * and time using the kinematic equation:
        % * finalVelocity = initialVelocity + acceleration * time.
        % *
        % * @param initialVelocity Initial velocity (m/s).
        % * @param acceleration Constant acceleration (m/s²).
        % * @param time Time duration (seconds).
        % *
        % * @return finalVelocity The calculated final velocity (m/s).
        % *
        % * @warning None
        % */
        function finalVelocity = calculateFinalVelocity(obj, initialVelocity, acceleration, time)
            finalVelocity = initialVelocity + acceleration * time;
        end
        
        %/**
        % * @brief Calculates the angular rates over a given time.
        % *
        % * This method computes the change in angular rates based on the initial angular velocity and time.
        % *
        % * @param angularVelocity Initial angular velocity vector [p; q; r] (rad/s).
        % * @param time Time duration (seconds).
        % *
        % * @return angularRates The calculated angular rates vector after the given time (rad/s).
        % *
        % * @warning None
        % */
        function angularRates = calculateAngularRates(obj, angularVelocity, time)
            angularRates = angularVelocity * time;
        end
        
        %/**
        % * @brief Returns the rotation matrix from body to inertial frame.
        % *
        % * This method computes the rotation matrix based on the provided roll (phi), pitch (theta),
        % * and yaw (psi) angles using the standard aerospace rotation sequence.
        % *
        % * @param phi Roll angle (radians).
        % * @param theta Pitch angle (radians).
        % * @param psi Yaw angle (radians).
        % *
        % * @return rotationMatrix The rotation matrix (3x3) transforming vectors from body frame to inertial frame.
        % *
        % * @warning None
        % */
        function rotationMatrix = getRotationMatrix(obj, phi, theta, psi)
            % Returns the rotation matrix from body to inertial frame
            rotationMatrix = [
                cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);
                sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
                sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
                sin(phi)*cos(theta);
                cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), ...
                cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), ...
                cos(phi)*cos(theta);
                ];
        end
        
        %/**
        % * @brief Updates the lateral accelerations for tractor and trailer based on lateral forces and masses.
        % *
        % * This method computes the lateral accelerations using the formula:
        % * lateralAcceleration = lateralForce / mass.
        % *
        % * @param tractorMass Mass of the tractor (kg).
        % * @param trailerMass Mass of the trailer (kg).
        % *
        % * @return obj The updated KinematicsCalculator object with the new lateral accelerations.
        % *
        % * @throws Error if masses are zero or negative.
        % */
        function obj = updateLateralAccelerations(obj, tractorMass, trailerMass)
            % Update the lateral accelerations based on lateral forces and masses
            if tractorMass <= 0 || trailerMass <= 0
                error('Masses must be positive to calculate lateral accelerations.');
            end
            
            forces = obj.forceCalculator.getCalculatedForces();
            
            % Lateral force on the tractor
            if isKey(forces, 'F_y_total')
                F_lateral_tractor = forces('F_y_total');
            else
                F_lateral_tractor = 0;
                warning('Lateral force on tractor not found in calculated forces.');
            end
            
            % Lateral force on the trailer
            if isKey(forces, 'F_y_trailer')
                F_lateral_trailer = forces('F_y_trailer');
            else
                F_lateral_trailer = 0;
                warning('Lateral force on trailer not found in calculated forces.');
            end
            
            % Update lateral accelerations
            obj.lateralAccelerationTractor = F_lateral_tractor / tractorMass;
            obj.lateralAccelerationTrailer = F_lateral_trailer / trailerMass;
            
            % Debug: Display updated lateral accelerations
            debugLog('Updated Lateral Acceleration (Tractor): %.4f m/s²\n', obj.lateralAccelerationTractor);
            debugLog('Updated Lateral Acceleration (Trailer): %.4f m/s²\n', obj.lateralAccelerationTrailer);
        end
        
        %/**
        % * @brief Updates the roll dynamics based on rolling moments and current state.
        % *
        % * This method updates the roll angles and roll rates for both the tractor and trailer using
        % * the roll dynamics equations:
        % * rollAccel = (momentRoll - D_roll * rollRate - K_roll * rollAngle) / I_roll
        % * rollRate = rollRate + rollAccel * dt
        % * rollAngle = rollAngle + rollRate * dt
        % *
        % * @return obj The updated KinematicsCalculator object with new roll angles and rates.
        % *
        % * @warning None
        % */
        function obj = updateRollDynamics(obj)
            g = 9.81; % Gravity
            
            forces = obj.forceCalculator.getCalculatedForces();
            
            % --- Tractor Roll Dynamics ---
            % Retrieve momentRoll for tractor
            if isKey(forces, 'momentRoll')
                M_roll_tractor = forces('momentRoll');
            else
                M_roll_tractor = 0;
                warning('momentRoll for tractor not found.');
            end
            
            % Roll acceleration for tractor
            rollAccelTractor = (M_roll_tractor - obj.D_roll_tractor * obj.rollRateTractor - obj.K_roll_tractor * obj.rollAngleTractor) / obj.I_roll_tractor;
            
            % Update roll rate and angle for tractor
            obj.rollRateTractor = obj.rollRateTractor + rollAccelTractor * obj.dt;
            obj.rollAngleTractor = obj.rollAngleTractor + obj.rollRateTractor * obj.dt;
            
            % --- Trailer Roll Dynamics ---
            % Retrieve momentRoll_trailer for trailer
            if isKey(forces, 'momentRoll_trailer')
                M_roll_trailer = forces('momentRoll_trailer');
            else
                M_roll_trailer = 0;
                %warning('momentRoll for trailer not found.');
            end
            
            % Roll acceleration for trailer
            rollAccelTrailer = (M_roll_trailer - obj.D_roll_trailer * obj.rollRateTrailer - obj.K_roll_trailer * obj.rollAngleTrailer) / obj.I_roll_trailer;
            
            % Update roll rate and angle for trailer
            obj.rollRateTrailer = obj.rollRateTrailer + rollAccelTrailer * obj.dt;
            obj.rollAngleTrailer = obj.rollAngleTrailer + obj.rollRateTrailer * obj.dt;
            
            % Debug: Display updated roll angles
            debugLog('Updated Roll Angle (Tractor): %.4f rad (%.2f degrees)\n', obj.rollAngleTractor, rad2deg(obj.rollAngleTractor));
            debugLog('Updated Roll Angle (Trailer): %.4f rad (%.2f degrees)\n', obj.rollAngleTrailer, rad2deg(obj.rollAngleTrailer));
        end
        
        %/**
        % * @brief Calculates the current roll angle of the tractor or trailer.
        % *
        % * This method returns the current roll angle for the specified vehicle part.
        % *
        % * @param vehiclePart A string specifying 'tractor' or 'trailer'.
        % *
        % * @return rollAngle The current roll angle (radians).
        % *
        % * @warning None
        % */
        function rollAngle = getRollAngle(obj, vehiclePart)
            if strcmp(vehiclePart, 'tractor')
                rollAngle = obj.rollAngleTractor;
                
                % Debug: Display the current roll angle for tractor
                debugLog('Current Roll Angle (Tractor): %.4f rad (%.2f degrees)\n', rollAngle, rad2deg(rollAngle));
            elseif strcmp(vehiclePart, 'trailer')
                rollAngle = obj.rollAngleTrailer;
                
                % Debug: Display the current roll angle for trailer
                debugLog('Current Roll Angle (Trailer): %.4f rad (%.2f degrees)\n', rollAngle, rad2deg(rollAngle));
            else
                error('Invalid vehicle part specified. Use ''tractor'' or ''trailer''.');
            end
        end
        
        %/**
        % * @brief Calculates the distance traveled under constant acceleration.
        % *
        % * [Original method preserved]
        % */
        function distance = calculateDistanceOriginal(obj, initialVelocity, acceleration, time)
            distance = initialVelocity * time + 0.5 * acceleration * time.^2;
        end
        
        %/**
        % * @brief Calculates the final velocity after a given time under constant acceleration.
        % *
        % * [Original method preserved]
        % */
        function finalVelocity = calculateFinalVelocityOriginal(obj, initialVelocity, acceleration, time)
            finalVelocity = initialVelocity + acceleration * time;
        end
        
        %/**
        % * @brief Calculates the angular rates over a given time.
        % *
        % * [Original method preserved]
        % */
        function angularRates = calculateAngularRatesOriginal(obj, angularVelocity, time)
            angularRates = angularVelocity * time;
        end
        
        %/**
        % * @brief Returns the rotation matrix from body to inertial frame.
        % *
        % * [Original method preserved]
        % */
        function rotationMatrix = getRotationMatrixOriginal(obj, phi, theta, psi)
            rotationMatrix = [
                cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);
                sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
                sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
                sin(phi)*cos(theta);
                cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), ...
                cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), ...
                cos(phi)*cos(theta);
                ];
        end
        
        %/**
        % * @brief Updates the lateral acceleration based on lateral force and mass.
        % *
        % * [Original method preserved]
        % */
        function obj = updateLateralAccelerationOriginal(obj, lateralForce, mass)
            % Update the lateral acceleration based on lateral force and mass
            if mass <= 0
                error('Mass must be positive to calculate lateral acceleration.');
            end
            obj.lateralAcceleration = lateralForce / mass;
            debugLog('Updated Lateral Acceleration: %.4f m/s²\n', obj.lateralAcceleration);
        end
        
        %/**
        % * @brief Calculates the current roll angle of the trailer.
        % *
        % * [Original method modified to handle 'tractor' and 'trailer']
        % */
        function rollAngle = getRollAngleOriginal(obj)
            % Calculate the current roll angle of the trailer
            % rollAngle = atan(lateralAcceleration * h_CoG / g)
            g = 9.81; % Acceleration due to gravity (m/s^2)
            rollAngle = atan(obj.lateralAcceleration * obj.h_CoG / g);
            
            % Debug: Display the calculated roll angle
            debugLog('Calculated Roll Angle: %.4f rad (%.2f degrees)\n', rollAngle, rad2deg(rollAngle));
        end
        
        %/**
        % * @brief Calculates the center of gravity based on load distributions.
        % *
        % * [Original static method preserved]
        % */
        function centerOfGravity = calculateCenterOfGravityOriginal(loads, positions)
            % Static method to calculate the center of gravity
            totalMass = sum(loads);
            if totalMass <= 0
                error('Total mass is zero or negative in calculateCenterOfGravity');
            end
            cgX = sum(loads .* positions(:, 1)) / totalMass;
            cgY = sum(loads .* positions(:, 2)) / totalMass;
            cgZ = sum(loads .* positions(:, 3)) / totalMass;

            centerOfGravity = [cgX; cgY; cgZ];
        end

        function hitchAngleRad = getHitchAngle(obj)
            % getHitchAngle: returns the articulation angle (tractor <-> trailer) in radians.
    
            % 1) Tractor yaw might be stored in ForceCalculator.orientation:
            tractorYaw  = obj.forceCalculator.orientation;  % or 0 if orientation is not used
    
            % 2) Trailer yaw from ForceCalculator.trailerPsi:
            trailerYaw  = obj.forceCalculator.trailerPsi;
    
            % 3) Articulation angle = trailerYaw - tractorYaw
            hitchAngleRad = trailerYaw - tractorYaw;
    
            debugLog('Tractor Yaw: %.4f rad, Trailer Yaw: %.4f rad, HitchAngle: %.4f rad\n', ...
                    tractorYaw, trailerYaw, hitchAngleRad);
        end
    end
    
    methods (Static)
        %/**
        % * @brief Calculates the center of gravity based on load distributions.
        % *
        % * [Original static method preserved]
        % */
        function centerOfGravity = calculateCenterOfGravity(loads, positions)
            % Static method to calculate the center of gravity
            totalMass = sum(loads);
            if totalMass <= 0
                error('Total mass is zero or negative in calculateCenterOfGravity');
            end
            cgX = sum(loads .* positions(:, 1)) / totalMass;
            cgY = sum(loads .* positions(:, 2)) / totalMass;
            cgZ = sum(loads .* positions(:, 3)) / totalMass;

            centerOfGravity = [cgX; cgY; cgZ];
        end
    end
end
