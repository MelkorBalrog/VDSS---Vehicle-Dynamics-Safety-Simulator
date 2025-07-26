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
%! \file HitchModel.m
%! \brief Defines the HitchModel class for simulating tractor-trailer hitch dynamics.
%!
%! This class models the physical behavior of a tractor-trailer hitch, including
%! stiffness, damping, and angular inertia properties. It provides methods to
%! calculate forces and moments exerted by the hitch and update the trailer's
%! angular state using numerical integration.
%!
%! \author Miguel Marina <karel.capek.robotics@gmail.com>

classdef HitchModel
    %! \class HitchModel
    %! \brief A model for tractor-trailer hitch dynamics.
    %!
    %! The HitchModel class encapsulates the properties and methods required to
    %! simulate the articulation and dynamics of a tractor-trailer hitch system.
    %! It includes parameters for stiffness, damping, maximum articulation angle,
    %! and trailer inertia, among others.

    properties
        %! \var tractorHitchPoint
        %! \brief [x; y; z] position on tractor in tractor's frame (meters).
        %! 
        %! A 3x1 vector representing the position of the hitch point on the tractor.
        tractorHitchPoint      

        %! \var trailerKingpinPoint
        %! \brief [x; y; z] position on trailer in trailer's frame (meters).
        %! 
        %! A 3x1 vector representing the position of the kingpin point on the trailer.
        trailerKingpinPoint    

        %! \var stiffnessCoefficients
        %! \brief Struct containing stiffness coefficients for all axes.
        stiffnessCoefficients  

        %! \var dampingCoefficients
        %! \brief Struct containing damping coefficients for all axes.
        dampingCoefficients    

        %! \var maxDelta
        %! \brief Maximum articulation angle (radians).
        maxDelta               

        %! \var trailerWheelbase
        %! \brief Trailer wheelbase for position calculation (meters).
        trailerWheelbase       

        %! \var fifthWheelOffset
        %! \brief [x; y; z] offset of the fifth wheel from tractor hitch point (meters).
        %!
        %! A 3x1 vector representing the offset position of the fifth wheel relative
        %! to the tractor's hitch point.
        fifthWheelOffset       

        %! \var fifthWheelStiffnessYaw
        %! \brief Stiffness coefficient for yaw articulation (N·m/rad).
        fifthWheelStiffnessYaw 

        %! \var fifthWheelDampingYaw
        %! \brief Damping coefficient for yaw articulation (N·m·s/rad).
        fifthWheelDampingYaw   

        %! \var loadDistribution
        %! \brief [Nx4] Matrix: [x, y, z, load] for each load point.
        loadDistribution       

        %! \var trailerInertia
        %! \brief [kg·m²] Moment of inertia of the trailer.
        trailerInertia         

        %! \var angularState
        %! \brief Struct with angular position and velocity of the trailer.
        angularState           

        %! \var dt
        %! \brief Time step for integration (seconds).
        dt

        %! \var trailingCoefficient
        %! \brief Lever arm coefficient converting pulling force to yaw correcting torque (meters).
        trailingCoefficient

        %! \var trailingVelocityThreshold
        %! \brief Minimum tractor speed to apply trailing torque (m/s).
        trailingVelocityThreshold
    end

    methods
        function obj = HitchModel(tractorHitchPoint, trailerKingpinPoint, stiffness, damping, maxDelta, trailerWheelbase, loadDistribution, dt, trailingCoefficient)
            %! \brief Constructor for HitchModel.
            %!
            %! Initializes the HitchModel object with the specified parameters.
            %!
            %! \param tractorHitchPoint [x; y; z] position on tractor in tractor's frame (meters).
            %! \param trailerKingpinPoint [x; y; z] position on trailer in trailer's frame (meters).
            %! \param stiffness Struct containing stiffness coefficients for all axes.
            %! \param damping Struct containing damping coefficients for all axes.
            %! \param maxDelta Maximum articulation angle (radians).
            %! \param trailerWheelbase Trailer wheelbase for position calculation (meters).
            %! \param loadDistribution [Nx4] Matrix: [x, y, z, load] for each load point.
            %! \param dt Time step for integration (seconds).
            %! \param trailingCoefficient Lever arm coefficient for trailing torque (meters).
            %
            %! \throws Error if input vectors are not 3-dimensional.
            %! \throws Error if stiffness or damping structs lack required fields.

            % Validate input vectors
            if ~isvector(tractorHitchPoint) || length(tractorHitchPoint) ~= 3
                error('tractorHitchPoint must be a 3x1 or 1x3 vector.');
            end
            if ~isvector(trailerKingpinPoint) || length(trailerKingpinPoint) ~= 3
                error('trailerKingpinPoint must be a 3x1 or 1x3 vector.');
            end

            % Ensure column vectors
            obj.tractorHitchPoint = tractorHitchPoint(:);
            obj.trailerKingpinPoint = trailerKingpinPoint(:);
            obj.stiffnessCoefficients = stiffness;
            obj.dampingCoefficients = damping;
            obj.maxDelta = maxDelta;
            obj.trailerWheelbase = trailerWheelbase;

            if nargin < 9 || isempty(trailingCoefficient)
                obj.trailingCoefficient = trailerWheelbase/2;
            else
                obj.trailingCoefficient = trailingCoefficient;
            end
            obj.trailingVelocityThreshold = 0.5; % m/s

            if nargin < 9 || isempty(trailingCoefficient)
                obj.trailingCoefficient = trailerWheelbase/2;
            else
                obj.trailingCoefficient = trailingCoefficient;
            end
            obj.trailingVelocityThreshold = 0.5; % m/s

            % Validate 'stiffness' struct
            requiredFields = {'x', 'y', 'z', 'roll', 'pitch', 'yaw'};
            if ~isstruct(stiffness) || ~all(isfield(stiffness, requiredFields))
                error('stiffness must be a struct with fields: x, y, z, roll, pitch, yaw.');
            end

            % Validate 'damping' struct
            if ~isstruct(damping) || ~all(isfield(damping, requiredFields))
                error('damping must be a struct with fields: x, y, z, roll, pitch, yaw.');
            end

            % Initialize Fifth Wheel Properties with Sensible Defaults
            obj.fifthWheelOffset = [0; 0; 0.5]; % 0.5 meters above hitch point by default
            obj.fifthWheelStiffnessYaw = stiffness.yaw;   % Use provided yaw stiffness
            obj.fifthWheelDampingYaw = damping.yaw;       % Use provided yaw damping

            % Initialize Load Distribution
            if nargin < 7 || isempty(loadDistribution)
                % Default load distribution if not provided
                % Each row: [x, y, z, load]
                obj.loadDistribution = [
                    -obj.trailerWheelbase/2,  1, obj.trailerKingpinPoint(3), 3000;
                    -obj.trailerWheelbase/2, -1, obj.trailerKingpinPoint(3), 3000;
                     obj.trailerWheelbase/2,  1, obj.trailerKingpinPoint(3), 4000;
                     obj.trailerWheelbase/2, -1, obj.trailerKingpinPoint(3), 4000;
                ];
            else
                obj.loadDistribution = loadDistribution;
            end

            % Calculate Trailer Inertia Based on Load Distribution
            obj.trailerInertia = obj.calculateTrailerInertia();

            % Initialize angular state
        obj.angularState.psi = 0;     % Initial yaw angle
        obj.angularState.omega = 0;   % Initial yaw rate
        obj.dt = dt;                  % Time step for integration
        end

        function obj = initializeState(obj, psi, omega)
            %INITIALIZESTATE Set initial yaw angle and rate for the hitch model.
            %   OBJ = INITIALIZESTATE(OBJ, PSI, OMEGA) sets the internal
            %   angular state of the hitch to the provided yaw angle PSI and yaw
            %   rate OMEGA. If omitted, PSI defaults to 0 and OMEGA defaults to
            %   0.

            if nargin < 2 || isempty(psi)
                psi = 0;
            end
            if nargin < 3 || isempty(omega)
                omega = 0;
            end

            obj.angularState.psi = psi;
            obj.angularState.omega = omega;
        end

        function trailerInertia = calculateTrailerInertia(obj)
            %! \brief Calculate the trailer's moment of inertia around the yaw axis based on load distribution.
            %!
            %! \return trailerInertia [kg·m²] Moment of inertia of the trailer.

            % Extract positions and loads
            positions = obj.loadDistribution(:, 1:2);  % x, y positions
            loads = obj.loadDistribution(:, 4);        % loads (forces)

            % Convert loads to masses (m = F/g)
            g = 9.81; % Gravity
            masses = loads / g; % [kg]

            % Calculate radial distances squared from yaw axis (assumed at kingpin point)
            r_squared = sum((positions - obj.trailerKingpinPoint(1:2)').^2, 2); % [m²]

            % Calculate moment of inertia
            trailerInertia = sum(masses .* r_squared); % [kg·m²]
        end

        function [obj, F_total, M_total] = calculateForces(obj, tractorState, trailerState, pullingForce)
            %! \brief Computes the total forces and moments exerted by the hitch.
            %!
            %! Updates the internal angular state of the trailer using Runge-Kutta integration.
            %! Returns the forces and moments at the hitch.
            %!
            %! \param tractorState Struct containing tractor's state.
            %! \param trailerState Struct containing trailer's state.
            %! \param pullingForce Longitudinal force pulling the trailer (N).
            %!
            %! \return obj Updated HitchModel object with new angular state.
            %! \return F_total [Fx; Fy; Fz] Total force vector (N).
            %! \return M_total [Mx; My; Mz] Total moment vector (N·m).

            if nargin < 4
                pullingForce = 0;
            end
            %
            %! \details
            %! The method extracts the angular velocities of the tractor and trailer,
            %! calculates the relative yaw angle, constrains it within the maximum
            %! articulation angle, computes the torque due to hitch stiffness and damping,
            %! and integrates the angular state using the Runge-Kutta 4 method.
            %! It then updates the trailer's orientation and returns the calculated forces
            %! and moments.

            %% Extract States
            % Rotation matrices (assuming zero roll/pitch for simplicity)
            R_tr  = [cos(tractorState.orientation(3)), -sin(tractorState.orientation(3)), 0;
                     sin(tractorState.orientation(3)),  cos(tractorState.orientation(3)), 0;
                     0,                                  0,                                 1];
            R_trl = [cos(trailerState.orientation(3)), -sin(trailerState.orientation(3)), 0;
                     sin(trailerState.orientation(3)),  cos(trailerState.orientation(3)), 0;
                     0,                                  0,                                 1];

            %% Relative translations and velocities of hitch points
            hitchPos   = tractorState.position + R_tr * obj.tractorHitchPoint;
            kingpinPos = trailerState.position + R_trl * obj.trailerKingpinPoint;
            deltaPos   = kingpinPos - hitchPos;

            hitchVel   = tractorState.velocity + cross(tractorState.angularVelocity, R_tr * obj.tractorHitchPoint);
            kingpinVel = trailerState.velocity + cross(trailerState.angularVelocity, R_trl * obj.trailerKingpinPoint);
            deltaVel   = kingpinVel - hitchVel;

            %% Relative orientations and angular rates
            deltaAngles = wrapToPi(trailerState.orientation - tractorState.orientation);
            deltaYaw    = deltaAngles(3);
            deltaOmega  = trailerState.angularVelocity - tractorState.angularVelocity;
            omega_trailer = obj.angularState.omega;
            omega_tractor = tractorState.angularVelocity(3);

            %% Calculate forces using spring-damper model
            stiff = obj.stiffnessCoefficients;
            damp  = obj.dampingCoefficients;
            if norm(tractorState.velocity) > obj.trailingVelocityThreshold
                % Apply pulling force along tractor's longitudinal axis
                F_total = R_tr * [pullingForce; 0; 0];
            else
                F_total = [0; 0; 0];
            end

            %% Calculate torques around all axes
            M_roll  = -stiff.roll  * deltaAngles(1) - damp.roll  * deltaOmega(1);
            M_pitch = -stiff.pitch * deltaAngles(2) - damp.pitch * deltaOmega(2);
            M_yaw_spring  = -obj.fifthWheelStiffnessYaw * deltaYaw;
            M_yaw_damping = -obj.fifthWheelDampingYaw   * (omega_trailer - omega_tractor);
            if norm(tractorState.velocity) > obj.trailingVelocityThreshold
                M_trailing = -obj.trailingCoefficient * pullingForce * sin(deltaYaw);
            else
                M_trailing = 0;
            end
            torque_hitch  = M_yaw_spring + M_yaw_damping + M_trailing;
            M_total = [M_roll; M_pitch; torque_hitch];

            % Angular acceleration
            alpha = torque_hitch / obj.trailerInertia;

            %% Integrate Angular State Using Runge-Kutta 4
            dt = obj.dt;
            omega0 = omega_trailer;
            psi0 = obj.angularState.psi;

            % Define the derivative function
            dydt = @(t, y) [y(2); alpha];

            % RK4 integration
            k1 = dydt(0, [psi0; omega0]);
            k2 = dydt(dt/2, [psi0 + dt/2 * k1(1); omega0 + dt/2 * k1(2)]);
            k3 = dydt(dt/2, [psi0 + dt/2 * k2(1); omega0 + dt/2 * k2(2)]);
            k4 = dydt(dt, [psi0 + dt * k3(1); omega0 + dt * k3(2)]);

            psi_new = psi0 + dt/6 * (k1(1) + 2*k2(1) + 2*k3(1) + k4(1));
            omega_new = omega0 + dt/6 * (k1(2) + 2*k2(2) + 2*k3(2) + k4(2));

            %% Constrain the articulation angle within ±maxDelta
            % Compute the new relative yaw angle
            % After calculating psi_new and omega_new
            deltaYaw_new = wrapToPi(psi_new - tractorState.orientation(3));
            
            if deltaYaw_new > obj.maxDelta
                deltaYaw_new = obj.maxDelta;
                psi_new = tractorState.orientation(3) + deltaYaw_new;
                omega_new = 0; % Stop further rotation
            elseif deltaYaw_new < -obj.maxDelta
                deltaYaw_new = -obj.maxDelta;
                psi_new = tractorState.orientation(3) + deltaYaw_new;
                omega_new = 0; % Stop further rotation
            end
            
            % Update angular state
            obj.angularState.psi = psi_new;
            obj.angularState.omega = omega_new;

            %% Update Trailer Orientation
            trailerState.orientation(3) = psi_new;

            %% Calculate Forces and Moments at Hitch
            % F_total and M_total were computed above and returned

            % Return updated HitchModel object
        end
    end
end