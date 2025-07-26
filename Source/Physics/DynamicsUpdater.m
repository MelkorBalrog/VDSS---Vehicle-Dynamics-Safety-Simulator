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
% @file DynamicsUpdater.m
% @brief Updates vehicle dynamics using applied forces and moments.
%        Includes roll dynamics in the state vector.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}
%/**
% * @class DynamicsUpdater
% * @brief Updates the dynamics of a vehicle (tractor-trailer, tractor, or passenger vehicle) based on applied forces, including roll dynamics.
% *
% * The DynamicsUpdater class now includes roll angle and roll rate in the state vector, allowing the simulation of roll dynamics under wind forces.
% * It also integrates braking forces from the ForceCalculator to handle deceleration smoothly and manages gear shifting through the Transmission class.
% * Additionally, acceleration changes are smoothed to prevent abrupt transitions.
% * 
% * @version 2.5
% * @date 2024-11-04
 % */
% ============================================================================
% Embedded Systems Best Practices:
%   - Use fixed-size pre-allocated arrays for state and derivative storage
%   - Pool and reuse buffers to avoid dynamic memory allocation in real-time loops
%   - Inline small utility functions to reduce function call overhead
%   - Eliminate or conditionally compile out debug/log statements for performance builds
%   - Use lookup tables for common nonlinear functions to reduce transcendental calls
%   - Avoid branching in inner loops; use vectorized condition masks
%   - Pre-compute constant transformation matrices and reuse across time steps
%   - Minimize use of containers.Map; use structs or arrays for constant fields
%   - Consider generating optimized C code (MEX) for computational hotspots
% ============================================================================
% Module Interface
% Methods:
%   updateState: selects integrator (Euler or RK4) and updates vehicle state
%   updateStateEuler: Euler integration of state vector
%   updateStateRK4: 4th order Runge-Kutta integration
%   stateDerivative: computes state derivatives [d(position); d(orientation); d(momentum); d(roll)]
%   Acceleration smoothing via smoothingCoefficient
% Properties updated: position, orientation, linearMomentum, angularMomentum, rollAngle, rollRate, a_long, a_lat, velocity, lateralVelocity, yawRate
%
% Dependencies:
%   ForceCalculator: computes forces/moments acting on vehicle
%   KinematicsCalculator: computes kinematic transformations
%   Transmission: handles gear updates
%
% Bottlenecks:
%   - Allocation of state/derivative arrays each step
%   - Conditional integrator switching overhead
%   - Frequent property updates and dynamic method calls
%
% Proposed Optimizations:
%   - Pre-allocate and reuse state and derivative buffers
%   - Merge Euler and RK4 code paths or use single vectorized integrator
%   - Replace containers.Map usage in ForceCalculator with static struct
%   - Inline small helper methods and leverage MATLAB Coder for performance
%   - Remove redundant updates to ForceCalculator properties
% ============================================================================
classdef DynamicsUpdater < handle
    properties
        position                % [x; y] position in meters
        orientation             % Yaw angle (theta) in radians
        mass                    % Vehicle mass (kg)
        wheelbase               % Distance between front and rear axles (meters)
        h_CoG                   % Height of Center of Gravity (meters)
        width                   % Vehicle width (meters)
        dt                      % Time step (seconds)
        steeringAngle           % Steering angle (radians)
        forceCalculator         % Instance of ForceCalculator class
        kinematicsCalculator    % Instance of KinematicsCalculator class (if any)
        speedThreshold          % Speed threshold to switch between Euler and RK4 integration (m/s)
        a_lat                   % Lateral acceleration (m/s²)
        a_long                  % Longitudinal acceleration (m/s²)
        vehicleType             % Type of vehicle: 'tractor-trailer', 'tractor', or 'PassengerVehicle'

        velocity
        lateralVelocity
        yawRate

        % --- Properties for Momentum ---
        linearMomentum          % [p_x; p_y] Linear momentum in vehicle frame (kg·m/s)
        angularMomentum         % Angular momentum (L_z) about yaw axis (kg·m²/s)

        % --- Properties for Trailer State ---
        trailerVelocity         % [u_trailer; v_trailer; 0] in m/s
        trailerAngularVelocity  % [0; 0; r_trailer] in rad/s

        % --- Properties for Roll Dynamics ---
        rollAngle               % Roll angle (phi) in radians
        rollRate                % Roll rate (p) in rad/s

        % --- Additional Properties ---
        trackWidth              % Track width of the vehicle (meters)
        K_roll                  % Roll stiffness coefficient (N·m/rad)
        C_roll                  % Roll damping coefficient (N·m·s/rad)

        % --- Transmission and Time ---
        transmission            % Instance of Transmission class
        currentTime             % Current simulation time (s)

        % --- Acceleration Smoothing ---
        smoothingCoefficient    % Smoothing coefficient for acceleration updates
    end

    methods
        % Constructor with roll dynamics and transmission initialization
        function obj = DynamicsUpdater(forceCalc, kinCalc, initialState, mass, wheelbase, h_CoG, width, dt, vehicleType, trackWidth, K_roll, C_roll, transmission, smoothingCoefficient)
            % Validate required fields in initialState
            requiredFields = {'position', 'orientation', 'velocity', 'lateralVelocity', 'yawRate', 'rollAngle', 'rollRate'};
            for i = 1:length(requiredFields)
                if ~isfield(initialState, requiredFields{i})
                    error('initialState must contain the field: %s', requiredFields{i});
                end
            end

            % Initialize properties
            obj.forceCalculator = forceCalc;
            obj.kinematicsCalculator = kinCalc;
            obj.position = initialState.position;         % [x; y]
            obj.orientation = initialState.orientation;   % Yaw angle (theta)
            obj.mass = mass;
            obj.wheelbase = wheelbase;
            obj.h_CoG = h_CoG;
            obj.width = width;
            obj.dt = dt;
            obj.steeringAngle = 0;
            obj.speedThreshold = 2; % m/s
            obj.vehicleType = vehicleType; % 'tractor-trailer', 'tractor', or 'PassengerVehicle'
            obj.trackWidth = trackWidth;

            % Initialize roll stiffness and damping
            obj.K_roll = K_roll;
            obj.C_roll = C_roll;

            % Initialize Transmission
            if nargin < 13 || isempty(transmission)
                error('Transmission instance must be provided.');
            else
                obj.transmission = transmission;
            end

            obj.currentTime = 0; % Initialize simulation time

            % Initialize trailer state if vehicleType is 'tractor-trailer'
            if strcmp(obj.vehicleType, 'tractor-trailer')
                obj.trailerVelocity = [0; 0; 0];
                obj.trailerAngularVelocity = [0; 0; 0];
            elseif strcmp(obj.vehicleType, 'tractor') || strcmp(obj.vehicleType, 'PassengerVehicle')
                % No trailer state
                obj.trailerVelocity = [];
                obj.trailerAngularVelocity = [];
            else
                error('Unsupported vehicle type: %s. Use ''tractor-trailer'', ''tractor'', or ''PassengerVehicle''.', obj.vehicleType);
            end

            % Initialize accelerations
            obj.a_long = 0;
            obj.a_lat = 0;

            % Initialize smoothing coefficient
            if nargin < 14 || isempty(smoothingCoefficient)
                obj.smoothingCoefficient = 0.1; % Default smoothing coefficient
            else
                obj.smoothingCoefficient = smoothingCoefficient;
            end

            % Initialize momentum properties
            obj.linearMomentum = obj.mass * [initialState.velocity; initialState.lateralVelocity]; % [p_x; p_y]
            I_z = obj.forceCalculator.inertia(3);
            obj.angularMomentum = I_z * initialState.yawRate; % L_z

            % Initialize roll angle and rate
            obj.rollAngle = initialState.rollAngle;       % Roll angle (phi)
            obj.rollRate = initialState.rollRate;         % Roll rate (p)
        end

        % Update state method
        function obj = updateState(obj)
            % Increment current time
            obj.currentTime = obj.currentTime + obj.dt;

            % Determine if the vehicle is moving
            speed = norm(obj.linearMomentum / obj.mass);

            % Choose integration method based on speed
            if speed < obj.speedThreshold
                obj = obj.updateStateEuler();
            else
                obj = obj.updateStateRK4();
            end
        end

        % Update state using Euler integration
        function obj = updateStateEuler(obj)
            % Assemble current state vector
            state = [obj.position; obj.orientation; obj.linearMomentum; obj.angularMomentum; obj.rollAngle; obj.rollRate];
            dt = obj.dt;

            % Apply brakes smoothly
            obj.forceCalculator.brakeSystem = obj.forceCalculator.brakeSystem.applyBrakes(dt);

            % Update Transmission based on current speed and acceleration
            velocity = obj.linearMomentum / obj.mass;
            obj.transmission = obj.transmission.updateGear(norm(velocity), obj.a_long, obj.currentTime, dt);

            % Compute state derivatives
            [dydt, accelerations] = obj.stateDerivative(state);
            state_new = state + dt * dydt;

            % Update state variables
            obj.position = state_new(1:2);
            obj.orientation = state_new(3);
            obj.linearMomentum = state_new(4:5);
            obj.angularMomentum = state_new(6);
            obj.rollAngle = state_new(7);
            obj.rollRate = state_new(8);

            % Update velocities
            obj.velocity = obj.linearMomentum(1) / obj.mass;
            obj.lateralVelocity = obj.linearMomentum(2) / obj.mass;
            I_z = obj.forceCalculator.inertia(3);
            obj.yawRate = obj.angularMomentum / I_z;

            % Smooth accelerations
            new_a_long = accelerations(1);
            new_a_lat = accelerations(2);
            alpha = obj.smoothingCoefficient;
            obj.a_long = alpha * new_a_long + (1 - alpha) * obj.a_long;
            obj.a_lat = alpha * new_a_lat + (1 - alpha) * obj.a_lat;

            % Update ForceCalculator properties
            obj.forceCalculator.velocity = [obj.velocity; obj.lateralVelocity; 0];
            obj.forceCalculator.orientation = obj.orientation;
            obj.forceCalculator.angularVelocity = [obj.rollRate; 0; obj.yawRate];
            obj.forceCalculator.steeringAngle = obj.steeringAngle;

            % --- Ensure Longitudinal Speed Does Not Go Below Zero ---
            if obj.velocity < 0
                obj.velocity = 0;
                % obj.linearMomentum(1) = 0;
                obj.a_long = 0;
                % Optionally, log this event
                % disp('Negative speed detected. Clamped to zero.');
            end

            % --- Prevent Movement When Vehicle Is Stationary ---
            if obj.velocity == 0 && obj.a_long == 0
                % Set lateral velocity and yaw rate to zero if forces are negligible
                obj.lateralVelocity = 0;
                obj.linearMomentum(2) = 0;
                obj.yawRate = 0;
                obj.angularMomentum = 0;
                obj.rollRate = 0;
                % Prevent position and orientation from changing
                % (Already handled by not updating them)
            end
        end

        % Update state using Runge-Kutta 4 (RK4) integration
        function obj = updateStateRK4(obj)
            % Assemble current state vector
            state = [obj.position; obj.orientation; obj.linearMomentum; obj.angularMomentum; obj.rollAngle; obj.rollRate];
            dt = obj.dt;

            % Apply brakes smoothly
            obj.forceCalculator.brakeSystem = obj.forceCalculator.brakeSystem.applyBrakes(dt);

            % Update Transmission based on current speed and acceleration
            velocity = obj.linearMomentum / obj.mass;
            obj.transmission = obj.transmission.updateGear(norm(velocity), obj.a_long, obj.currentTime, dt);

            % Compute k1
            [k1, accel1] = obj.stateDerivative(state);

            % Compute k2
            [k2, accel2] = obj.stateDerivative(state + dt/2 * k1);

            % Compute k3
            [k3, accel3] = obj.stateDerivative(state + dt/2 * k2);

            % Compute k4
            [k4, accel4] = obj.stateDerivative(state + dt * k3);

            % Update state using RK4 formula
            state_new = state + dt/6 * (k1 + 2*k2 + 2*k3 + k4);

            % Update state variables
            obj.position = state_new(1:2);
            obj.orientation = state_new(3);
            obj.linearMomentum = state_new(4:5);
            obj.angularMomentum = state_new(6);
            obj.rollAngle = state_new(7);
            obj.rollRate = state_new(8);

            % Update velocities
            obj.velocity = obj.linearMomentum(1) / obj.mass;
            obj.lateralVelocity = obj.linearMomentum(2) / obj.mass;
            I_z = obj.forceCalculator.inertia(3);
            obj.yawRate = obj.angularMomentum / I_z;

            % Compute accelerations as weighted average
            new_a_long = (accel1(1) + 2*accel2(1) + 2*accel3(1) + accel4(1))/6;
            new_a_lat = (accel1(2) + 2*accel2(2) + 2*accel3(2) + accel4(2))/6;

            % Smooth accelerations
            alpha = obj.smoothingCoefficient;
            obj.a_long = alpha * new_a_long + (1 - alpha) * obj.a_long;
            obj.a_lat = alpha * new_a_lat + (1 - alpha) * obj.a_lat;

            % Update ForceCalculator properties
            obj.forceCalculator.velocity = [obj.velocity; obj.lateralVelocity; 0];
            obj.forceCalculator.orientation = obj.orientation;
            obj.forceCalculator.angularVelocity = [obj.rollRate; 0; obj.yawRate];
            obj.forceCalculator.steeringAngle = obj.steeringAngle;

            % --- Ensure Longitudinal Speed Does Not Go Below Zero ---
            if obj.velocity < 0
                obj.velocity = 0;
                obj.linearMomentum(1) = 0;
                obj.a_long = 0;
                % Optionally, log this event
                % disp('Negative speed detected. Clamped to zero.');
            end

            % --- Prevent Movement When Vehicle Is Stationary ---
            if obj.velocity == 0 && obj.a_long == 0
                % Set lateral velocity and yaw rate to zero if forces are negligible
                obj.lateralVelocity = 0;
                obj.linearMomentum(2) = 0;
                obj.yawRate = 0;
                obj.angularMomentum = 0;
                obj.rollRate = 0;
                % Prevent position and orientation from changing
                % (Already handled by not updating them)
            end
        end

        % Compute state derivatives including roll dynamics and momentum
        function [dydt, accelerations] = stateDerivative(obj, state)
            % Extract state variables
            x = state(1);
            y = state(2);
            theta = state(3);    % Yaw angle
            p_x = state(4);      % Linear momentum in x
            p_y = state(5);      % Linear momentum in y
            L_z = state(6);      % Angular momentum around z-axis
            phi = state(7);      % Roll angle
            p = state(8);        % Roll rate

            m = obj.mass;
            I_z = obj.forceCalculator.inertia(3);      % Yaw moment of inertia from ForceCalculator

            % Compute velocities
            u = p_x / m;
            v = p_y / m;
            r = L_z / I_z;

            % Compute moment arm based on roll angle and height of CoG
            momentArm = obj.h_CoG * sin(phi);

            % Calculate vehicle load
            vehicleLoad = sum(obj.forceCalculator.loadDistribution(:,4));

            % Assemble acceleration vector
            acceleration = [obj.a_long; obj.a_lat; 0]; % [a_x; a_y; a_z]

            % Create vehicleState struct based on vehicleType
            if strcmp(obj.vehicleType, 'tractor-trailer')
                if isempty(obj.trailerVelocity) || isempty(obj.trailerAngularVelocity)
                    error('Trailer state not set in DynamicsUpdater for tractor-trailer.');
                end
                vehicleState = struct(...
                    'position', [x; y; 0], ...
                    'orientation', [phi; 0; theta], ... % Include roll angle
                    'velocity', [u; v; 0], ...
                    'angularVelocity', [p; 0; r], ...
                    'trailerVelocity', obj.trailerVelocity, ...
                    'trailerAngularVelocity', obj.trailerAngularVelocity, ...
                    'verticalDisplacement', 0, ...
                    'verticalVelocity', 0, ...
                    'momentArm', momentArm, ...
                    'lateralAcceleration', obj.a_lat, ...
                    'longitudinalAcceleration', obj.a_long, ...
                    'vehicleLoad', vehicleLoad, ...
                    'vehicleType', obj.vehicleType, ...
                    'acceleration', acceleration ...
                );
            elseif strcmp(obj.vehicleType, 'PassengerVehicle') || strcmp(obj.vehicleType, 'tractor')
                vehicleState = struct(...
                    'position', [x; y; 0], ...
                    'orientation', [phi; 0; theta], ...
                    'velocity', [u; v; 0], ...
                    'angularVelocity', [p; 0; r], ...
                    'verticalDisplacement', 0, ...
                    'verticalVelocity', 0, ...
                    'momentArm', momentArm, ...
                    'lateralAcceleration', obj.a_lat, ...
                    'longitudinalAcceleration', obj.a_long, ...
                    'vehicleLoad', vehicleLoad, ...
                    'vehicleType', obj.vehicleType, ...
                    'acceleration', acceleration ...
                );
            else
                error('Unsupported vehicle type: %s. Use ''tractor-trailer'', ''tractor'', or ''PassengerVehicle''.', obj.vehicleType);
            end

            % Calculate forces based on vehicleState
            obj.forceCalculator = obj.forceCalculator.calculateForces(vehicleState);

            % Retrieve calculated forces struct
            forces = obj.forceCalculator.getCalculatedForces();
            totalForce = forces.totalForce;            % In vehicle frame [F_x; F_y; F_z]
            totalMoment = forces.momentZ;              % Yaw moment (about Z-axis)
            momentRoll = forces.momentRoll;            % Rolling moment (about X-axis)

            % Rate of change of momentum
            dp_x_dt = totalForce(1);
            dp_y_dt = totalForce(2);

            % Yaw rate derivative (rate of change of angular momentum)
            dL_z_dt = totalMoment;

            %--------------------------------------------------------------
            % Longitudinal and lateral accelerations
            %--------------------------------------------------------------
            % Newton-Euler equations in the body frame:
            %   m*(du - r*v) = F_x
            %   m*(dv + r*u) = F_y
            % where u and v are the longitudinal and lateral velocities and r is
            % the yaw rate. Solving for the derivatives gives
            %   du/dt = F_x/m + r*v
            %   dv/dt = F_y/m - r*u
            % These correspond to the longitudinal (a_long) and lateral
            % (a_lat) accelerations used for load transfer and stability.

            a_long = dp_x_dt / m + r * v;
            a_lat  = dp_y_dt / m - r * u;

            % Roll dynamics using inertia from ForceCalculator
            I_xx = obj.forceCalculator.inertia(1);    % Roll moment of inertia from ForceCalculator
            h = obj.h_CoG;
            g = 9.81;                                  % Gravitational acceleration (m/s²)

            % Restoring moment due to gravity and suspension
            M_gravity = m * g * h * sin(phi);
            M_suspension = obj.K_roll * phi + obj.C_roll * p;

            % Roll rate derivative
            dpdt = (momentRoll - M_gravity - M_suspension) / I_xx;

            % Roll angle derivative
            dphidt = p;

            % Position derivatives
            dxdt = u * cos(theta) - v * sin(theta);
            dydt = u * sin(theta) + v * cos(theta);
            dthetadt = r;

            % --- Prevent Position and Orientation Changes When Stationary ---
            % Define small thresholds for forces and moments to consider them negligible
            force_threshold = 1e-3;    % Newtons
            moment_threshold = 1e-3;   % Newton-meters

            % Calculate total longitudinal and lateral forces and yaw moment
            total_longitudinal_force = abs(totalForce(1));
            total_lateral_force = abs(totalForce(2));
            total_yaw_moment = abs(totalMoment);

            if (total_longitudinal_force < force_threshold) && ...
               (total_lateral_force < force_threshold) && ...
               (total_yaw_moment < moment_threshold) && ...
               (norm([u; v]) == 0)
                % If all forces and moments are negligible and speed is zero,
                % prevent any state changes by setting derivatives to zero
                dxdt = 0;
                dydt = 0;
                dthetadt = 0;
                dp_x_dt = 0;
                dp_y_dt = 0;
                dL_z_dt = 0;
                dphidt = 0;
                dpdt = 0;
            end

            % Assemble state derivatives
            dydt = [dxdt; dydt; dthetadt; dp_x_dt; dp_y_dt; dL_z_dt; dphidt; dpdt];

            % Compute accelerations in vehicle frame
            accelerations = [a_long; a_lat];
        end

        %% Methods to Set Trailer State
        function obj = setTrailerVelocity(obj, trailerVelocity)
            if strcmp(obj.vehicleType, 'tractor-trailer')
                obj.trailerVelocity = trailerVelocity;
            else
                warning('Trailer velocity is only applicable to tractor-trailer configurations.');
            end
        end

        function obj = setTrailerAngularVelocity(obj, trailerAngularVelocity)
            if strcmp(obj.vehicleType, 'tractor-trailer')
                obj.trailerAngularVelocity = trailerAngularVelocity;
            else
                warning('Trailer angular velocity is only applicable to tractor-trailer configurations.');
            end
        end

        %% setMass
        function obj = setMass(obj, newMass)
            % setMass Updates the vehicle mass while preserving velocity.
            %
            % Updating the mass directly affects how linear momentum maps to
            % velocity. To avoid artificial jumps in speed when simulation
            % parameters change, rescale the stored momentum so that the current
            % velocity remains unchanged.

            if newMass <= 0
                error('Mass must be positive');
            end

            % Compute current velocity from momentum and old mass
            currentVel = obj.linearMomentum / obj.mass;

            % Update mass and rescale momentum to maintain velocity
            obj.mass = newMass;
            obj.linearMomentum = obj.mass * currentVel;
            obj.velocity = currentVel(1);
            obj.lateralVelocity = currentVel(2);
        end
    end
end
