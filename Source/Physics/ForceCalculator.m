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
%/**
% * @file ForceCalculator.m
% * @brief Calculates various forces acting on the vehicle and trailer,
% *        including the effects of wind-induced yaw moments on the trailer's yaw dynamics.
% *        Now correctly includes the wind's effect on the trailer's yaw angle (Psi).
% *        Integrated braking functionality to handle deceleration.
% *
% * This version has additional optional calculations at the end of `calculateForces`
% * to enhance rollover and jackknife detection by combining roll rate and hitch angle dynamics.
% *
% * @author Miguel Marina <karel.capek.robotics@gmail.com>
 % */
% ============================================================================
% Module Interface
% Constructor:
%   obj = ForceCalculator(vehicleType, mass, friction, velocity, dragCoeff, airDensity, frontalArea, sideArea, sideForceCoeff, ...
%                          turnRadius, loadDist, cog, h_CoG, angularVel, slopeAngle, trackWidth, wheelbase, tireModel, suspensionModel, trailerInertia, dt, trailerMass, trailerWheelbase, numTrailerTires, trailerBoxMasses, tireModelFlag, highFidelityTireModel, windVector, brakeSystem, [wheelSpeeds, wheelRadius, wheelInertia])
% Public Methods:
%   calculateForces(vehicleState): computes forces and moments, updates internal containers
%   getFilteredForces(): returns filtered force map
%   updateWheelSpeeds(dt, engineTorque, brakeTorque): updates wheel angular speeds
%   setFlatTire(flatTireIndices): simulate flat tires
% Properties:
%   Stores vehicle, trailer, tire, suspension, wind, and braking parameters
%
% Dependencies:
%   SuspensionModel: for suspension forces/moments
%   BrakeSystem: for braking force computation
%   TireModel/HighFidelityModel: for tire force calculations
%   containers.Map for dynamic force storage
%
% Bottlenecks:
%   - Loops over tires for force calculations (unvectorized)
%   - Repeated trigonometric and matrix operations inside loops
%   - Extensive conditional branching for vehicle types
%   - Use of containers.Map introduces dynamic field access overhead
%
% Proposed Optimizations:
%   - Vectorize per-tire force computations, replacing loops with array operations
%   - Precompute constant rotation matrices and reuse
%   - Replace containers.Map with pre-allocated struct or arrays for fixed fields
%   - Consolidate duplicate code paths for tractor, tractor-trailer, passenger
%   - Approximate aerodynamic forces using polynomial fits or lookup tables
%   - Use built-in MATLAB vectorization to compute drag/side forces in bulk
% ============================================================================
% Embedded Systems Best Practices:
%   - Pre-allocate and recycle all data buffers to avoid heap allocations
%   - Use fixed-size structs/arrays instead of dynamic containers or grows
%   - Pool objects and reuse memory for repeated simulation calls
%   - Minimize MATLAB function calls in hot loops; inline small routines
%   - Eliminate or gate off debug/fprintf statements under a compile-time flag
%   - Employ fixed-point arithmetic or lower precision floats where tolerable
%   - Use lookup tables for nonlinear functions (tire curves, aero drag)
%   - Avoid branching inside loops; replace with vectorized masks
%   - Precompute and cache constant matrices/coefficients outside time-critical loops
%   - Consider generating MEX or C code for critical inner loops via MATLAB Coder
% ============================================================================
classdef ForceCalculator
    properties
        % ----------------------------------------------------------------
        %  Existing Properties (unchanged)
        % ----------------------------------------------------------------
        vehicleType              % 'tractor-trailer', 'tractor', or 'passenger'
        enableSpeedController
        vehicleMass
        frictionCoefficient
        velocity                 % [u; v; w] in vehicle frame (m/s)
        dragCoefficient
        airDensity
        frontalArea
        sideArea                 % Side area (m²)
        sideForceCoefficient     % Side force coefficient (dimensionless)
        turnRadius
        loadDistribution         % Matrix [x, y, z, load (N), contact area (m²)]
        centerOfGravity          % [x; y; z] in vehicle frame (m)
        h_CoG                    % Height of center of gravity of the tractor (m)
        h_CoG_trailer            % Height of center of gravity of the trailer (m)
        angularVelocity          % [p; q; r] in vehicle frame (rad/s)
        inertia                  % [Ixx, Iyy, Izz] (kg·m²)
        gravity                  % Gravitational acceleration (m/s²)
        slopeAngle               % Slope angle (radians)
        calculatedForces         % struct to store calculated forces and moments
        orientation              % Orientation angle (theta) in radians

        trackWidth
        wheelbase
        steeringAngle
        C_alpha_front            % Front cornering stiffness (N/rad)
        C_alpha_rear             % Rear cornering stiffness (N/rad)
        massFront
        massRear
        tireModel                % Struct (simple) or high-fidelity tire model object
        suspensionModel          % Instance of suspension model
        trailerRollAngle
        trailerPitchAngle

        % --- Trailer Yaw Dynamics ---
        trailerPsi               % Current yaw angle of the trailer (radians)
        trailerOmega             % Current yaw angular velocity of the trailer (rad/s)
        trailerPosition          % Current [x; y; z] in global frame (m)
        trailerInertia           % Trailer's moment of inertia (yaw axis) (kg·m²)
        dt                       % Time step for integration (s)

        % --- Trailer ---
        trailerMass
        trailerWheelbase
        numTrailerTires
        trailerBoxMasses

        % --- Adjusted Tire Parameters ---
        mu_tires                 % Adjusted friction coefficients per tractor tire
        mu_tires_trailer         % Adjusted friction coefficients per trailer tire

        % Simple tire model parameters
        B_tires
        C_tires
        D_tires
        E_tires
        % High-fidelity tire model parameters
        pCx1_tires
        pDx1_tires
        pDx2_tires
        pEx1_tires
        pEx2_tires
        pEx3_tires
        pEx4_tires
        pKx1_tires
        pKx2_tires
        pKx3_tires
        pCy1_tires
        pDy1_tires
        pDy2_tires
        pEy1_tires
        pEy2_tires
        pEy3_tires
        pEy4_tires
        pKy1_tires
        pKy2_tires
        pKy3_tires
        rollingResistanceCoefficients % Rolling resistance per tire

        % --- Tire Model Selection ---
        tireModelFlag
        highFidelityTireModel

        % --- Flat Tire Simulation ---
        flatTireIndices

        % --- Additional Properties ---
        jointForce   % Only relevant for passenger vehicles

        % --- Wind Vector ---
        windVector   % [wind_u; wind_v; wind_w] in global frame (m/s)

        % --- Braking System ---
        brakeSystem
        brakingForce % Current braking force (N)

        % --- Filtering ---
        forceBuffers
        filterWindowSize
        calculatedForces_filtered % struct to store filtered forces

        % --- Slip Ratio Props ---
        wheelSpeeds     % wheel angular speeds [rad/s] per wheel
        wheelRadius     % wheel radius (m)
        wheelInertia    % wheel inertia (kg·m²)        
        
        % --- Surface Friction ---
        surfaceFrictionManager  % Instance managing per-tire friction
    end

    methods
        %% Constructor
        function obj = ForceCalculator(vehicleType, mass, friction, velocity, ...
                dragCoeff, airDensity, frontalArea, sideArea, sideForceCoeff, ...
                turnRadius, loadDist, cog, h_CoG, angularVel, slopeAngle, ...
                trackWidth, wheelbase, tireModel, suspensionModel, trailerInertia, ...
                dt, trailerMass, trailerWheelbase, numTrailerTires, trailerBoxMasses, ...
                tireModelFlag, highFidelityTireModel, windVector, brakeSystem, varargin)
            % Constructor for ForceCalculator
            %
            % Preserves all original parameters, plus optional 'wheelSpeeds, wheelRadius, wheelInertia'
            % if given via varargin.

            % Validate vehicleType
            if ~ismember(vehicleType, {'tractor-trailer', 'tractor', 'passenger'})
                error('vehicleType must be ''tractor-trailer'', ''tractor'', or ''passenger''.');
            end
            obj.vehicleType = vehicleType;

            % Assign main properties
            obj.vehicleMass         = mass;
            obj.frictionCoefficient = friction;
            obj.velocity            = velocity;     % [u; v; w]
            obj.dragCoefficient     = dragCoeff;
            obj.airDensity          = airDensity;
            obj.frontalArea         = frontalArea;
            obj.sideArea            = sideArea;
            obj.sideForceCoefficient= sideForceCoeff;
            obj.turnRadius          = turnRadius;
            obj.loadDistribution    = loadDist;     % [x, y, z, load(N), contactArea(m²)]
            obj.centerOfGravity     = cog;
            obj.h_CoG               = h_CoG;
            obj.h_CoG_trailer       = h_CoG;        % By default, same for trailer
            obj.angularVelocity     = angularVel;   % [p; q; r]
            obj.slopeAngle          = slopeAngle;
            obj.trackWidth          = trackWidth;
            obj.wheelbase           = wheelbase;
            obj.gravity             = 9.81;         % m/s²
            % Initialize calculated forces struct with zero values
            forceKeys = {'hitch','hitchLateralForce','F_drag_global','F_side_global',... 
                         'momentZ_wind','traction','traction_force','M_z','momentZ',...  
                         'F_y_total','momentRoll','totalForce','F_y_trailer','momentZ_trailer',...   
                         'F_total_trailer_local','F_total_trailer_global','yawMoment_trailer',...  
                         'momentRoll_trailer','trailerPsi','trailerOmega','rolloverRiskIndex',...  
                         'jackknifeRiskIndex','braking'};
            obj.calculatedForces = struct();
            for iKey = 1:numel(forceKeys)
                obj.calculatedForces.(forceKeys{iKey}) = 0;
            end
            obj.orientation         = 0;            % initial orientation
            obj.steeringAngle       = 0;
            obj.C_alpha_front       = 80000;        % example
            obj.C_alpha_rear        = 80000;        % example
            obj.tireModel           = tireModel;
            obj.suspensionModel     = suspensionModel;

            % --- Tire Model Selection ---
            if isempty(tireModelFlag)
                obj.tireModelFlag = 'simple';
            else
                obj.tireModelFlag = tireModelFlag;
            end
            if isempty(highFidelityTireModel)
                obj.highFidelityTireModel = [];
            else
                obj.highFidelityTireModel = highFidelityTireModel;
            end

            % --- Trailer Yaw Dynamics and Props ---
            if strcmp(vehicleType, 'tractor-trailer')
                obj.trailerPsi       = 0;
                obj.trailerOmega     = 0;
                obj.trailerPosition  = [0; 0; 0];
                if nargin < 21 || isempty(trailerInertia)
                    obj.trailerInertia = 10000;  % default trailer inertia
                    debugLog('Using default trailer inertia: %.2f kg·m²\n', obj.trailerInertia);
                else
                    obj.trailerInertia = trailerInertia;
                end
                if nargin < 22 || isempty(dt)
                    obj.dt = 0.01;  % default timestep
                    debugLog('Using default time step dt: %.4f s\n', obj.dt);
                else
                    obj.dt = dt;
                end

                % Must have trailerMass, trailerWheelbase, numTrailerTires
                if nargin >= 25
                    obj.trailerMass       = trailerMass;
                    obj.trailerWheelbase  = trailerWheelbase;
                    obj.numTrailerTires   = numTrailerTires;
                    if nargin >= 26
                        obj.trailerBoxMasses = trailerBoxMasses;
                    else
                        obj.trailerBoxMasses = [];
                    end
                else
                    error('Must provide trailer mass, wheelbase, and tire count for tractor-trailer.');
                end
            elseif strcmp(vehicleType, 'tractor') || strcmp(vehicleType, 'passenger')
                obj.trailerPsi       = [];
                obj.trailerOmega     = [];
                obj.trailerPosition  = [];
                obj.trailerInertia   = [];
                obj.dt               = [];
                obj.trailerMass      = [];
                obj.trailerWheelbase = [];
                obj.numTrailerTires  = [];
                obj.trailerBoxMasses = [];
                if strcmp(vehicleType, 'passenger')
                    obj.jointForce = [0; 0; 0];
                end
            end

            % Initialize tire parameters
            obj = obj.initializeTireParameters();

            % Initialize friction and rolling-resistance coefficients
            numTires = size(obj.loadDistribution, 1);
            obj.mu_tires = obj.frictionCoefficient * ones(numTires, 1);
            if strcmp(obj.vehicleType, 'tractor-trailer') || strcmp(obj.vehicleType, 'tractor')
                obj.mu_tires_trailer = obj.frictionCoefficient * ones(obj.numTrailerTires, 1);
            else
                obj.mu_tires_trailer = [];
            end

            if strcmp(obj.vehicleType, 'tractor-trailer')
                obj.rollingResistanceCoefficients = 0.027 * ones(numTires, 1);
            else
                obj.rollingResistanceCoefficients = 0.015 * ones(numTires, 1);
            end

            % Initialize flat tire indices
            obj.flatTireIndices = [];

            % --- Wind Vector ---
            if nargin < 30 || isempty(windVector)
                obj.windVector = [0; 0; 0];
                warning('Wind vector not provided. Default [0;0;0].');
            else
                if numel(windVector) ~= 3
                    error('windVector must be [wind_u; wind_v; wind_w], 3 elements.');
                end
                obj.windVector = windVector(:);
            end

            % --- Braking System ---
            if isempty(brakeSystem)
                error('BrakeSystem instance must be provided.');
            else
                obj.brakeSystem = brakeSystem;
                obj.brakingForce = 0;
            end

            % Calculate axle masses and inertia
            if strcmp(obj.vehicleType, 'tractor-trailer') || strcmp(obj.vehicleType, 'tractor')
                [obj.massFront, obj.massRear] = obj.calculateAxleMasses();
                obj = obj.calculateInertia();
            elseif strcmp(obj.vehicleType, 'passenger')
                [obj.massFront, obj.massRear] = obj.calculateAxleMassesPassenger();
                obj.inertia = obj.calculateInertiaPassenger();
            end

            % --- Filter Setup ---
            obj.filterWindowSize = 5;
            % Pre-allocate force buffers and filtered forces for moving average (fixed-size)
            obj.forceBuffers = struct();
            obj.calculatedForces_filtered = struct();
            for iKey = 1:numel(forceKeys)
                key = forceKeys{iKey};
                obj.forceBuffers.(key) = struct('data', zeros(obj.filterWindowSize,1), 'sum', 0, 'index', 0, 'count', 0);
                obj.calculatedForces_filtered.(key) = 0;
            end

            % --- Optional wheel parameters via varargin ---
            if length(varargin) >= 3
                obj.wheelSpeeds  = varargin{1};
                obj.wheelRadius  = varargin{2};
                obj.wheelInertia = varargin{3};
            else
                obj.wheelSpeeds  = zeros(numTires, 1); % default
                obj.wheelRadius  = 0.3;               % default
                obj.wheelInertia = 1.0;               % default
            end

            % Surface friction manager (optional)
            obj.surfaceFrictionManager = [];
        end
        
        %% computeTireForces (vectorized lateral forces and yaw moment)
        function [F_y_total, M_z] = computeTireForces(obj, loads, contactAreas, u, v, r)
            numTires = numel(loads);
            xPos = obj.loadDistribution(:,1);
            a = obj.wheelbase/2;
            b = obj.wheelbase/2;
            alpha_front = obj.steeringAngle - atan2(v + a*r, u);
            alpha_rear  = -atan2(v - b*r, u);
            alpha = alpha_rear * ones(numTires,1);
            alpha(xPos>0) = alpha_front;
            mu_t = obj.mu_tires;
            if strcmp(obj.tireModelFlag, 'simple')
                D = mu_t .* loads;
                B = obj.B_tires;
                C = obj.C_tires;
                E = obj.E_tires;
                Fy = D .* sin(C .* atan(B .* alpha - E .* (B .* alpha - atan(B .* alpha))));
            else
                % High-fidelity tire model: compute per-tire forces sequentially
                Fy = zeros(numTires,1);
                for i = 1:numTires
                    Fy(i) = obj.highFidelityTireModel.calculateLateralForce(alpha(i), loads(i), i);
                end
            end
            F_y_total = sum(Fy);
            M_z = sum(xPos .* Fy);
        end

        %% Accessor for filtered forces
        function forces_filtered = getFilteredForces(obj)
            forces_filtered = obj.calculatedForces_filtered;
        end

        %% updateWheelSpeeds
        function obj = updateWheelSpeeds(obj, dt, engineTorque, brakeTorque)
            % Distributes torque among wheels, calculates angular accel

            nWheels = length(obj.wheelSpeeds);
            if nWheels == 0
                warning('No wheelSpeeds defined. updateWheelSpeeds skipped.');
                return;
            end

            % If wheelInertia is scalar, replicate
            if length(obj.wheelInertia) == 1
                localInertia = repmat(obj.wheelInertia, nWheels,1);
            else
                localInertia = obj.wheelInertia;
            end

            T_engine_perWheel = engineTorque / nWheels;
            T_brake_perWheel  = brakeTorque  / nWheels;

            rollingTorque = 0;  % example placeholder

            for i = 1:nWheels
                T_net = T_engine_perWheel - T_brake_perWheel - rollingTorque;
                alpha = T_net / localInertia(i);
                obj.wheelSpeeds(i) = obj.wheelSpeeds(i) + alpha*dt;
                if obj.wheelSpeeds(i) < 0
                    obj.wheelSpeeds(i) = 0;
                end
            end
        end

        %% initializeTireParameters
        function obj = initializeTireParameters(obj)
            numTires = size(obj.loadDistribution, 1);
            obj.mu_tires = obj.frictionCoefficient * ones(numTires, 1);
            obj.rollingResistanceCoefficients = 0.015 * ones(numTires, 1);

            if strcmp(obj.tireModelFlag, 'highFidelity')
                if isempty(obj.highFidelityTireModel)
                    error('High-fidelity tireModel not provided with flag=highFidelity.');
                end
                coeffs = obj.highFidelityTireModel;
                obj.pCx1_tires = coeffs.pCx1*ones(numTires,1);
                obj.pDx1_tires = coeffs.pDx1*ones(numTires,1);
                obj.pDx2_tires = coeffs.pDx2*ones(numTires,1);
                obj.pEx1_tires = coeffs.pEx1*ones(numTires,1);
                obj.pEx2_tires = coeffs.pEx2*ones(numTires,1);
                obj.pEx3_tires = coeffs.pEx3*ones(numTires,1);
                obj.pEx4_tires = coeffs.pEx4*ones(numTires,1);
                obj.pKx1_tires = coeffs.pKx1*ones(numTires,1);
                obj.pKx2_tires = coeffs.pKx2*ones(numTires,1);
                obj.pKx3_tires = coeffs.pKx3*ones(numTires,1);
                obj.pCy1_tires = coeffs.pCy1*ones(numTires,1);
                obj.pDy1_tires = coeffs.pDy1*ones(numTires,1);
                obj.pDy2_tires = coeffs.pDy2*ones(numTires,1);
                obj.pEy1_tires = coeffs.pEy1*ones(numTires,1);
                obj.pEy2_tires = coeffs.pEy2*ones(numTires,1);
                obj.pEy3_tires = coeffs.pEy3*ones(numTires,1);
                obj.pEy4_tires = coeffs.pEy4*ones(numTires,1);
                obj.pKy1_tires = coeffs.pKy1*ones(numTires,1);
                obj.pKy2_tires = coeffs.pKy2*ones(numTires,1);
                obj.pKy3_tires = coeffs.pKy3*ones(numTires,1);
            else
                if isempty(obj.tireModel)
                    error('Simple tireModel not provided with flag=simple.');
                end
                reqFields = {'B','C','D','E'};
                for iField = 1:length(reqFields)
                    if ~isfield(obj.tireModel, reqFields{iField})
                        error('tireModel missing field %s for simple model.', reqFields{iField});
                    end
                end
                obj.B_tires = obj.tireModel.B * ones(numTires,1);
                obj.C_tires = obj.tireModel.C * ones(numTires,1);
                obj.D_tires = obj.tireModel.D * ones(numTires,1);
                obj.E_tires = obj.tireModel.E * ones(numTires,1);
            end
        end

        %% calculateAxleMasses
        function [massFront, massRear] = calculateAxleMasses(obj)
            % Original logic for front/rear mass distribution
            loads = obj.loadDistribution(:,4);
            x_positions = obj.loadDistribution(:,1);
            massFront = sum(loads(x_positions>0)) / obj.gravity;
            massRear  = sum(loads(x_positions<=0))/ obj.gravity;
        end

        %% calculateAxleMassesPassenger
        function [massFront, massRear] = calculateAxleMassesPassenger(obj)
            loads = obj.loadDistribution(:,4);
            x_positions = obj.loadDistribution(:,1);
            massFront = sum(loads(x_positions>0)) / obj.gravity;
            massRear  = sum(loads(x_positions<=0))/ obj.gravity;
        end

        %% calculateInertia
        function obj = calculateInertia(obj)
            Ixx=0; Iyy=0; Izz=0;
            numLoads = size(obj.loadDistribution, 1);
            for i=1:numLoads
                loadWeight = obj.loadDistribution(i,4);
                loadMass   = loadWeight / obj.gravity;
                r = obj.loadDistribution(i,1:3)' - obj.centerOfGravity;  % vector from CoG

                Ixx = Ixx + loadMass*(r(2)^2 + r(3)^2);
                Iyy = Iyy + loadMass*(r(1)^2 + r(3)^2);
                Izz = Izz + loadMass*(r(1)^2 + r(2)^2);
            end
            obj.inertia = [Ixx, Iyy, Izz];

            % Add trailer inertia to Izz if tractor-trailer
            if strcmp(obj.vehicleType,'tractor-trailer')
                obj.inertia(3) = obj.inertia(3) + obj.trailerInertia;
                debugLog('Added trailer inertia to total inertia. New Izz: %.2f kg·m²\n', obj.inertia(3));
            end
        end

        %% calculateInertiaPassenger
        function inertia = calculateInertiaPassenger(obj)
            % Vectorized inertia calculation for passenger loads
            loads = obj.loadDistribution(:,4);         % load weights (N)
            masses = loads / obj.gravity;             % convert to mass (kg)
            % position vectors relative to CoG [x,y,z]
            relPos = obj.loadDistribution(:,1:3) - obj.centerOfGravity';
            % compute inertia sums
            Ixx = sum(masses .* (relPos(:,2).^2 + relPos(:,3).^2));
            Iyy = sum(masses .* (relPos(:,1).^2 + relPos(:,3).^2));
            Izz = sum(masses .* (relPos(:,1).^2 + relPos(:,2).^2));
            inertia = [Ixx, Iyy, Izz];
        end

        %% setFlatTire
        function obj = setFlatTire(obj, flatTireIndices)
            obj.flatTireIndices = flatTireIndices;
        end

        %% calculateForces
        function obj = calculateForces(obj, vehicleState)
            % Calculate all relevant forces based on current vehicle state
            %
            % Preserves original logic, and at the END we add optional rollover & jackknife calculations.

            assert(isscalar(obj.orientation), 'Orientation must be scalar.');
            obj = obj.updatePacejkaAttributes(vehicleState);

            theta = obj.orientation;
            R_veh2glob = [ cos(theta), -sin(theta), 0;
                           sin(theta),  cos(theta), 0;
                           0,           0,          1];
            R_g2v = R_veh2glob';

            u = obj.velocity(1);
            v = obj.velocity(2);
            r = obj.angularVelocity(3);
            speed = norm([u,v]);

            totalForce_vehicle = [0;0;0];
            M_z   = 0;
            M_roll= 0;

            % Gravity in vehicle frame
            % Gravitational force components due to road slope
            % Positive slopeAngle should resist forward motion (uphill)
            F_g_vehicle = [-obj.vehicleMass*obj.gravity*sin(obj.slopeAngle);
                           0;
                          -obj.vehicleMass*obj.gravity*cos(obj.slopeAngle)];
            totalForce_vehicle = totalForce_vehicle + F_g_vehicle;

            if ~isfield(vehicleState,'acceleration')
                error('vehicleState.acceleration required.');
            end
            acceleration = vehicleState.acceleration;
            accel_magnitude = norm(acceleration);

            obj.calculatedForces.hitchLateralForce = 0;  % default

            F_y_total=0; 
            M_z=0;

            %---------------------------------------------
            % IF BRANCH: (tractor) & small acceleration
            %---------------------------------------------
            if strcmp(obj.vehicleType,'tractor') && (accel_magnitude < 0.01)
                if obj.enableSpeedController
                    % 1) Aerodynamic forces and wind-induced yaw moment
                    [F_drag_g, F_side_g, M_z_wind] = obj.computeAeroForces(R_veh2glob);
                    obj.calculatedForces.F_drag_global = F_drag_g;
                    obj.calculatedForces.F_side_global = F_side_g;
                    obj.calculatedForces.momentZ_wind  = M_z_wind;

                    F_drag_v = R_g2v * F_drag_g;
                    F_side_v = R_g2v * F_side_g;

                    % 2) Traction
                    if isfield(obj.calculatedForces,'traction')
                        F_traction_v = obj.calculatedForces.traction;
                    else
                        F_traction_v = [0;0;0];
                    end

                    if size(obj.loadDistribution,2) < 5
                        error('loadDistribution must have 5 columns.');
                    end
                    loads        = obj.loadDistribution(:,4);
                    contactAreas = obj.loadDistribution(:,5);
                    pressures = loads ./ contactAreas;
                    P_ref     = mean(pressures);
                    mu_tires_ = obj.frictionCoefficient * (P_ref ./ pressures);
                    mu_tires_ = max(min(mu_tires_, 1.0), 0.3);
                    if ~isempty(obj.surfaceFrictionManager)
                        pos_local = obj.loadDistribution(:,1:2);
                        pos_global = (R_veh2glob(1:2,1:2)*pos_local')' + vehicleState.position(1:2)';
                        surf_mu = obj.surfaceFrictionManager.getMuForTirePositions(pos_global);
                        ratio = surf_mu ./ obj.frictionCoefficient;
                        mu_tires_ = mu_tires_ .* ratio;
                    end
                    obj.mu_tires = mu_tires_;
                    % Compute vectorized lateral forces and yaw moment
                    [F_y_total, M_z] = obj.computeTireForces(loads, contactAreas, u, v, r);
                    % Add wind moment
                    if isfield(obj.calculatedForces, 'momentZ_wind')
                        M_z = M_z + obj.calculatedForces.momentZ_wind;
                    end

                    [~,M_susp] = obj.suspensionModel.calculateForcesAndMoments(vehicleState);
                    M_z = M_z + M_susp;

                    obj.calculatedForces.momentZ = M_z;
                    obj.calculatedForces.F_y_total = F_y_total;
                    obj.calculatedForces.M_z      = M_z;

                    % Combine lateral forces
                    F_side_v = R_g2v*F_side_g;
                    F_lat_v = [0;F_y_total;0] + F_side_v;

                    % Rolling resistance
                    % Rolling resistance scales with normal force and road slope
                    F_rr = cos(obj.slopeAngle) * sum(obj.rollingResistanceCoefficients.*loads);
                    F_rr_v = -F_rr*[1;0;0];

                    % Suspension
                    [F_susp_, ~] = obj.suspensionModel.calculateForcesAndMoments(vehicleState);
                    F_susp_v = [0;0;F_susp_];

                    % Hitch
                    if isfield(obj.calculatedForces,'hitch') && strcmp(obj.vehicleType,'tractor-trailer')
                        F_hitch_v = obj.calculatedForces.hitch;
                    else
                        F_hitch_v = [0;0;0];
                    end

                    totalForce_vehicle = totalForce_vehicle + F_traction_v + F_lat_v + ...
                                         F_drag_v + F_rr_v + F_susp_v + F_hitch_v + ...
                                         [obj.brakingForce;0;0];

                    % Summation of Lateral Forces for momentRoll
                    F_side_aero  = F_side_v(2);
                    F_tire_lat   = F_y_total;
                    hitchLat = 0;
                    if isfield(obj.calculatedForces,'hitchLateralForce')
                        hitchLat = obj.calculatedForces.hitchLateralForce;
                    end
                    M_roll_aero  = F_side_aero*obj.h_CoG;
                    M_roll_tires = F_tire_lat* obj.h_CoG;
                    M_roll_hitch = hitchLat*   obj.h_CoG;

                    M_roll_total = M_roll_aero + M_roll_tires + M_roll_hitch;
                    obj.calculatedForces.momentRoll = M_roll_total;

                    % --- Trailer Dynamics (tractor-trailer) ---
                    if strcmp(obj.vehicleType,'tractor-trailer')
                        if speed>0
                            R_tr2g = [ cos(obj.trailerPsi), -sin(obj.trailerPsi), 0;
                                       sin(obj.trailerPsi),  cos(obj.trailerPsi), 0;
                                       0,                    0,                  1];
                            vel_tr_glob = R_veh2glob*obj.velocity;
                            rel_vel_tr  = vel_tr_glob - obj.windVector;
                            v_tr_rel    = norm(rel_vel_tr(1:2));
                            if v_tr_rel==0
                                F_drag_tr_g = [0;0;0];
                            else
                                F_drag_tr   = 0.5*obj.airDensity*obj.dragCoefficient*obj.frontalArea*v_tr_rel^2;
                                dir_ = rel_vel_tr(1:2)/v_tr_rel;
                                F_drag_tr_g = -F_drag_tr*[dir_;0];
                            end

                            vwlat_tr = obj.windVector(2);
                            if vwlat_tr==0
                                F_side_tr=0;
                            else
                                F_side_tr=0.5*obj.airDensity*obj.sideForceCoefficient*...
                                          obj.sideArea*vwlat_tr^2 * sign(vwlat_tr);
                            end
                            F_side_tr_g = [0; F_side_tr; 0];

                            R_g2tr = R_tr2g';
                            F_drag_tr_local = R_g2tr*F_drag_tr_g;
                            F_side_tr_local = R_g2tr*F_side_tr_g;

                            u_tr_ = vel_tr_glob(1);
                            v_tr_ = vel_tr_glob(2);
                            trailer_r = obj.trailerOmega;
                            if u_tr_~=0
                                alpha_trailer = -atan2(v_tr_-(obj.trailerWheelbase/2)*trailer_r, u_tr_);
                            else
                                alpha_trailer=0;
                            end

                            Fz_trail_each = (obj.trailerMass*obj.gravity)/obj.numTrailerTires;
                            % Vectorized trailer tire lateral forces
                            if isempty(obj.mu_tires_trailer)
                                obj.mu_tires_trailer = obj.frictionCoefficient * ones(obj.numTrailerTires,1);
                            end
                            mu_tr = obj.mu_tires_trailer;
                            Fz_tr_each = (obj.trailerMass * obj.gravity) / obj.numTrailerTires;
                            if strcmp(obj.tireModelFlag, 'simple')
                                D_tr = mu_tr * Fz_tr_each;
                                B_tr = obj.B_tires(1);
                                C_tr = obj.C_tires(1);
                                E_tr = obj.E_tires(1);
                                F_y_trailer_tires = D_tr .* sin(C_tr .* atan(B_tr .* alpha_trailer - E_tr .* (B_tr .* alpha_trailer - atan(B_tr .* alpha_trailer))));
                            else
                                F_y_trailer_tires = arrayfun(@(m) obj.calculateTireForce(alpha_trailer, m, Fz_tr_each, 1), mu_tr);
                            end
                            F_y_trailer_total = sum(F_y_trailer_tires);

                            F_lateral_trailer = F_y_trailer_total + F_side_tr_local(2);
                            F_longitudinal_tr= F_drag_tr_local(1);

                            if ~isempty(obj.trailerBoxMasses)
                                totalTrMass = sum(obj.trailerBoxMasses);
                            else
                                totalTrMass = obj.trailerMass;
                            end
                            F_rr_tr = obj.rollingResistanceCoefficients(1)*(totalTrMass*obj.gravity*cos(obj.slopeAngle));
                            F_rr_tr_local = -F_rr_tr*[1;0;0];

                            F_total_tr_local = [F_longitudinal_tr; F_lateral_trailer;0] + ...
                                               F_side_tr_local + F_rr_tr_local;
                            F_total_tr_global= R_tr2g*F_total_tr_local;

                            M_z_tr = F_y_trailer_total*(obj.trailerWheelbase/2);
                            M_z_tr_wind = F_side_tr*(obj.trackWidth/2);
                            M_z_tr_total= M_z_tr + M_z_tr_wind;

                            obj.calculatedForces.F_y_trailer = F_y_trailer_total;
                            obj.calculatedForces.momentZ_trailer = M_z_tr_total;
                            obj.calculatedForces.F_total_trailer_local = F_total_tr_local;
                            obj.calculatedForces.F_total_trailer_global = F_total_tr_global;
                            obj.calculatedForces.yawMoment_trailer     = M_z_tr_total;

                            F_total_tr_vehicle= R_g2tr'*F_total_tr_global;
                            hitchLatForce = F_total_tr_vehicle(2);
                            obj.calculatedForces.hitchLateralForce = hitchLatForce;

                            % Trailer yaw integration
                            trailer_yaw_accel = M_z_tr_total/obj.trailerInertia;
                            obj.trailerOmega  = obj.trailerOmega + trailer_yaw_accel*obj.dt;
                            obj.trailerPsi    = obj.trailerPsi+obj.trailerOmega*obj.dt;
                            obj.trailerPosition= obj.trailerPosition + vel_tr_glob*obj.dt;
                            obj.calculatedForces.trailerPsi = obj.trailerPsi;
                            obj.calculatedForces.trailerOmega = obj.trailerOmega;
                        else
                            % speed=0 => no movement
                            obj.calculatedForces.momentRoll_trailer = 0;
                            obj.calculatedForces.momentZ_trailer = 0;
                            obj.calculatedForces.F_y_trailer = 0;
                            obj.calculatedForces.hitchLateralForce = 0;
                            obj.trailerOmega=0;
                            obj.trailerPsi=0;
                        end
                    end
                    obj.calculatedForces.totalForce = totalForce_vehicle;
                end

            %---------------------------------------------
            % ELSE branch: everything else
            %---------------------------------------------
            else
                if obj.enableSpeedController
                    % Aerodynamic forces and wind-induced yaw moment
                    [F_drag_g, F_side_g, M_z_wind] = obj.computeAeroForces(R_veh2glob);
                    obj.calculatedForces.F_drag_global = F_drag_g;
                    obj.calculatedForces.F_side_global = F_side_g;
                    obj.calculatedForces.momentZ_wind  = M_z_wind;

                    F_drag_v = R_g2v * F_drag_g;
                    F_side_v = R_g2v * F_side_g;

                    if isfield(obj.calculatedForces,'traction')
                        F_traction_v = obj.calculatedForces.traction;
                    else
                        F_traction_v= [0;0;0];
                    end


                    if size(obj.loadDistribution,2)<5
                        error('loadDistribution must have 5 columns.');
                    end
                    if size(obj.loadDistribution,2) < 5
                        error('loadDistribution must have 5 columns.');
                    end
                    loads        = obj.loadDistribution(:,4);
                    contactAreas = obj.loadDistribution(:,5);
                    pressures    = loads ./ contactAreas;
                    P_ref        = mean(pressures);
                    mu_tires_    = obj.frictionCoefficient * (P_ref ./ pressures);
                    mu_tires_    = max(min(mu_tires_, 1.0), 0.3);
                    if ~isempty(obj.surfaceFrictionManager)
                        pos_local = obj.loadDistribution(:,1:2);
                        pos_global = (R_veh2glob(1:2,1:2)*pos_local')' + vehicleState.position(1:2)';
                        surf_mu = obj.surfaceFrictionManager.getMuForTirePositions(pos_global);
                        ratio   = surf_mu ./ obj.frictionCoefficient;
                        mu_tires_ = mu_tires_ .* ratio;
                    end
                    obj.mu_tires = mu_tires_;
                    [F_y_total, M_z] = obj.computeTireForces(loads, contactAreas, u, v, r);
                    if isfield(obj.calculatedForces, 'momentZ_wind')
                        M_z = M_z + obj.calculatedForces.momentZ_wind;
                    end
                    [~,M_susp] = obj.suspensionModel.calculateForcesAndMoments(vehicleState);
                    M_z = M_z + M_susp;

                    obj.calculatedForces.momentZ   = M_z;
                    obj.calculatedForces.F_y_total = F_y_total;
                    obj.calculatedForces.M_z       = M_z;

                    F_lat_v= [0;F_y_total;0] + F_side_v;
                    F_rr = cos(obj.slopeAngle) * sum(obj.rollingResistanceCoefficients.*loads);
                    F_rr_v= -F_rr*[1;0;0];
                    [F_susp_, ~]= obj.suspensionModel.calculateForcesAndMoments(vehicleState);
                    F_susp_v= [0;0;F_susp_];

                    if isfield(obj.calculatedForces,'hitch') && strcmp(obj.vehicleType,'tractor-trailer')
                        F_hitch_v = obj.calculatedForces.hitch;
                    else
                        F_hitch_v= [0;0;0];
                    end
                    totalForce_vehicle = totalForce_vehicle + F_traction_v+ F_lat_v +...
                                         F_drag_v + F_rr_v + F_susp_v + F_hitch_v + ...
                                         [obj.brakingForce;0;0];

                    % --- Summation of Lateral Forces for momentRoll ---
                    F_side_aero  = F_side_v(2);
                    F_tire_lat   = F_y_total;
                    hitchLat=0;
                    if isfield(obj.calculatedForces,'hitchLateralForce')
                        hitchLat = obj.calculatedForces.hitchLateralForce;
                    end
                    M_roll_aero  = F_side_aero* obj.h_CoG;
                    M_roll_tires = F_tire_lat*  obj.h_CoG;
                    M_roll_hitch = hitchLat*    obj.h_CoG;
                    M_roll_total = M_roll_aero + M_roll_tires + M_roll_hitch;
                    obj.calculatedForces.momentRoll = M_roll_total;

                    obj = obj.updatePacejkaBasedOnSuspension(0,0);

                    if strcmp(obj.vehicleType,'tractor-trailer')
                        if speed>0
                            R_tr2g = [ cos(obj.trailerPsi),-sin(obj.trailerPsi),0;
                                       sin(obj.trailerPsi), cos(obj.trailerPsi), 0;
                                       0,0,1];
                            vel_tr_glob = R_veh2glob*obj.velocity;
                            rel_trailer= vel_tr_glob - obj.windVector;
                            v_rel_trailer = norm(rel_trailer(1:2));
                            if v_rel_trailer==0
                                F_drag_tr_g= [0;0;0];
                            else
                                F_drag_tr   = 0.5*obj.airDensity*obj.dragCoefficient*obj.frontalArea*v_rel_trailer^2;
                                dir_        = rel_trailer(1:2)/v_rel_trailer;
                                F_drag_tr_g = -F_drag_tr*[dir_;0];
                            end

                            v_wind_lat_tr= obj.windVector(2);
                            if v_wind_lat_tr==0
                                F_side_tr=0;
                            else
                                F_side_tr=0.5*obj.airDensity*obj.sideForceCoefficient*...
                                    obj.sideArea*(v_wind_lat_tr^2)* sign(v_wind_lat_tr);
                            end
                            F_side_tr_g= [0;F_side_tr;0];

                            R_g2tr= R_tr2g';
                            F_drag_tr_local= R_g2tr*F_drag_tr_g;
                            F_side_tr_local= R_g2tr*F_side_tr_g;
                            u_tr_= vel_tr_glob(1);
                            v_tr_= vel_tr_glob(2);
                            trailer_r= obj.trailerOmega;
                            if u_tr_~=0
                                alpha_trailer= -atan2(v_tr_-(obj.trailerWheelbase/2)*trailer_r, u_tr_);
                            else
                                alpha_trailer=0;
                            end

                            Fz_trailer_each= (obj.trailerMass*obj.gravity)/obj.numTrailerTires;
                            F_y_trailer_tires= zeros(obj.numTrailerTires,1);
                            if isempty(obj.mu_tires_trailer)
                                obj.mu_tires_trailer= obj.frictionCoefficient*ones(obj.numTrailerTires,1);
                            end
                            for it=1:obj.numTrailerTires
                                mu_= obj.mu_tires_trailer(it);
                                F_y_trailer_tires(it)= obj.calculateTireForce(alpha_trailer, mu_, Fz_trailer_each,1);
                            end
                            F_y_trailer_total= sum(F_y_trailer_tires);
                            F_lateral_trailer= F_y_trailer_total+ F_side_tr_local(2);
                            F_longitudinal_tr= F_drag_tr_local(1);

                            if ~isempty(obj.trailerBoxMasses)
                                totalTrMass = sum(obj.trailerBoxMasses);
                            else
                                totalTrMass = obj.trailerMass;
                            end
                            F_rr_tr= obj.rollingResistanceCoefficients(1)*(totalTrMass*obj.gravity*cos(obj.slopeAngle));
                            F_rr_tr_local= -F_rr_tr*[1;0;0];
                            F_total_tr_local= [F_longitudinal_tr;F_lateral_trailer;0] + ...
                                              F_side_tr_local+ F_rr_tr_local;
                            F_total_tr_global= R_tr2g*F_total_tr_local;

                            M_z_tr = F_y_trailer_total*(obj.trailerWheelbase/2);
                            M_z_tr_wind= F_side_tr*(obj.trackWidth/2);
                            M_z_tr_total= M_z_tr+M_z_tr_wind;

                            obj.calculatedForces.F_y_trailer            = F_y_trailer_total;
                            obj.calculatedForces.momentZ_trailer       = M_z_tr_total;
                            obj.calculatedForces.F_total_trailer_local  = F_total_tr_local;
                            obj.calculatedForces.F_total_trailer_global = F_total_tr_global;
                            obj.calculatedForces.yawMoment_trailer      = obj.calculatedForces.momentZ_trailer;

                            F_total_tr_vehicle= R_g2tr*F_total_tr_global;
                            hitchLatForce= F_total_tr_vehicle(2);
                            obj.calculatedForces.hitchLateralForce = hitchLatForce;

                            trailer_yaw_accel= M_z_tr_total/obj.trailerInertia;
                            obj.trailerOmega= obj.trailerOmega+ trailer_yaw_accel*obj.dt;
                            obj.trailerPsi  = obj.trailerPsi+ obj.trailerOmega*obj.dt;
                            obj.trailerPosition= obj.trailerPosition + vel_tr_glob*obj.dt;

                            obj.calculatedForces.trailerPsi   = obj.trailerPsi;
                            obj.calculatedForces.trailerOmega = obj.trailerOmega;
                        else
                        obj.calculatedForces.momentRoll_trailer = 0;
                        obj.calculatedForces.momentZ_trailer    = 0;
                        obj.calculatedForces.F_y_trailer         = 0;
                        obj.calculatedForces.hitchLateralForce   = 0;
                            obj.trailerOmega=0;
                            obj.trailerPsi=0;
                        end
                    end

                    obj.calculatedForces.totalForce = totalForce_vehicle;
                    % <--- existing code ends here for main forces
                end
            end

            %% -- OPTIONAL Rollover Risk & Jackknife Risk Calculation (UPDATED) --

            % 1) Rollover Risk
            % Extract rollRate (if present)
            rollRate = 0;  
            if isfield(vehicleState, 'rollRate')
                rollRate = abs(vehicleState.rollRate);
            end

            % Extract rollAngle (if present)
            rollAngle = 0; 
            if isfield(vehicleState, 'rollAngle')
                rollAngle = abs(vehicleState.rollAngle);
            end

            % Get total lateral force in vehicle frame
            lateralForce = 0;
            if isfield(obj.calculatedForces, 'F_y_total')
                lateralForce = obj.calculatedForces.F_y_total;
            end

            % Also add side force in vehicle frame (if it exists)
            if isfield(obj.calculatedForces, 'F_side_global')
                F_side_g = obj.calculatedForces.F_side_global;
                F_side_v = R_g2v * F_side_g;
                lateralForce = lateralForce + F_side_v(2);
            end

            % Example new constants
            k_rollRate  = 0.1;   % roll-rate weight
            k_rollAngle = 0.05;  % roll-angle weight

            % Normalized lateral moment ratio relative to half track width
            normLateralMoment = ( abs(lateralForce) * obj.h_CoG ) / ...
                                ( obj.vehicleMass * obj.gravity * (obj.trackWidth/2) );

            rolloverRiskIndex = normLateralMoment + ...
                                k_rollRate * rollRate + ...
                                k_rollAngle * rollAngle;
            obj.calculatedForces.rolloverRiskIndex = rolloverRiskIndex;

            % 2) Jackknife Risk
            % Hitch angle = difference between trailer yaw and tractor yaw
            hitchAngle = 0;
            if strcmp(obj.vehicleType, 'tractor-trailer')
                hitchAngle = obj.trailerPsi - obj.orientation;  
            end

            hitchLatForce = 0;
            if isfield(obj.calculatedForces, 'hitchLateralForce')
                hitchLatForce = abs(obj.calculatedForces.hitchLateralForce);
            end

            trailerYawRate = 0;
            if ~isempty(obj.trailerOmega)
                trailerYawRate = abs(obj.trailerOmega);
            end

            % Example new constants
            c_hitchAngle = 0.02;
            c_trailerYaw = 0.05;
            c_hitchForce = 1/5000;  % 5000 N => ~1.0 scale

            jackknifeRiskIndex = c_hitchAngle * abs(hitchAngle) + ...
                                 c_trailerYaw * trailerYawRate + ...
                                 c_hitchForce * hitchLatForce;
            obj.calculatedForces.jackknifeRiskIndex = jackknifeRiskIndex;

            %% Finally apply moving average filter
            obj = obj.applyMovingAverageFilter();
        end

        %% updateTractionForce
        function obj = updateTractionForce(obj, F_traction)
            tractionForce= [F_traction;0;0];
            obj.calculatedForces.traction = tractionForce;
            obj.calculatedForces.traction_force = tractionForce;
        end

        %% updateBrakingForce
        function obj = updateBrakingForce(obj, F_brake)
            obj.brakingForce= F_brake;
            obj.calculatedForces.braking = [F_brake;0;0];
        end

        %% calculateTireForce
        function F_y = calculateTireForce(obj, alpha, mu_tire, F_z_per_tire, tireIndex)
            speed= norm(obj.velocity(1:2));
            if speed==0
                F_y=0; 
                return;
            end

            if strcmp(obj.tireModelFlag,'simple')
                D_ = mu_tire*F_z_per_tire;
                B_ = obj.B_tires(tireIndex);
                C_ = obj.C_tires(tireIndex);
                E_ = obj.E_tires(tireIndex);
                % Simple Pacejka
                F_y = D_ * sin(C_* atan(B_*alpha - E_*(B_*alpha - atan(B_*alpha))));
            elseif strcmp(obj.tireModelFlag,'highFidelity')
                F_y = obj.highFidelityTireModel.calculateLateralForce(alpha, F_z_per_tire, tireIndex);
            else
                error('Unsupported tireModelFlag: %s', obj.tireModelFlag);
            end
        end

        %% getCalculatedForces
        function forces = getCalculatedForces(obj)
            forces = obj.calculatedForces;
        end

        %% updatePacejkaAttributes
        function obj = updatePacejkaAttributes(obj, vehicleState)
            speed= norm(obj.velocity(1:2));
            if speed>30
                adjustmentFactor= 0.95;
            else
                adjustmentFactor= 1.0;
            end

            if strcmp(obj.tireModelFlag,'highFidelity')
                obj.pCx1_tires = obj.pCx1_tires*adjustmentFactor;
                obj.pDx1_tires = obj.pDx1_tires*adjustmentFactor;
                obj.pDx2_tires = obj.pDx2_tires*adjustmentFactor;
                obj.pEx1_tires = obj.pEx1_tires*adjustmentFactor;
                obj.pEx2_tires = obj.pEx2_tires*adjustmentFactor;
                obj.pEx3_tires = obj.pEx3_tires*adjustmentFactor;
                obj.pEx4_tires = obj.pEx4_tires*adjustmentFactor;
                obj.pKx1_tires = obj.pKx1_tires*adjustmentFactor;
                obj.pKx2_tires = obj.pKx2_tires*adjustmentFactor;
                obj.pKx3_tires = obj.pKx3_tires*adjustmentFactor;
                obj.pCy1_tires = obj.pCy1_tires*adjustmentFactor;
                obj.pDy1_tires = obj.pDy1_tires*adjustmentFactor;
                obj.pDy2_tires = obj.pDy2_tires*adjustmentFactor;
                obj.pEy1_tires = obj.pEy1_tires*adjustmentFactor;
                obj.pEy2_tires = obj.pEy2_tires*adjustmentFactor;
                obj.pEy3_tires = obj.pEy3_tires*adjustmentFactor;
                obj.pEy4_tires = obj.pEy4_tires*adjustmentFactor;
                obj.pKy1_tires = obj.pKy1_tires*adjustmentFactor;
                obj.pKy2_tires = obj.pKy2_tires*adjustmentFactor;
                obj.pKy3_tires = obj.pKy3_tires*adjustmentFactor;

                % If needed, also update your highFidelityTireModel's internal arrays
                for idx=1:length(obj.pKy1_tires)
                    obj.highFidelityTireModel.pCx1(idx)= obj.pCx1_tires(idx);
                    obj.highFidelityTireModel.pDx1(idx)= obj.pDx1_tires(idx);
                    obj.highFidelityTireModel.pDx2(idx)= obj.pDx2_tires(idx);
                    obj.highFidelityTireModel.pEx1(idx)= obj.pEx1_tires(idx);
                    obj.highFidelityTireModel.pEx2(idx)= obj.pEx2_tires(idx);
                    obj.highFidelityTireModel.pEx3(idx)= obj.pEx3_tires(idx);
                    obj.highFidelityTireModel.pEx4(idx)= obj.pEx4_tires(idx);
                    obj.highFidelityTireModel.pKx1(idx)= obj.pKx1_tires(idx);
                    obj.highFidelityTireModel.pKx2(idx)= obj.pKx2_tires(idx);
                    obj.highFidelityTireModel.pKx3(idx)= obj.pKx3_tires(idx);
                    obj.highFidelityTireModel.pCy1(idx)= obj.pCy1_tires(idx);
                    obj.highFidelityTireModel.pDy1(idx)= obj.pDy1_tires(idx);
                    obj.highFidelityTireModel.pDy2(idx)= obj.pDy2_tires(idx);
                    obj.highFidelityTireModel.pEy1(idx)= obj.pEy1_tires(idx);
                    obj.highFidelityTireModel.pEy2(idx)= obj.pEy2_tires(idx);
                    obj.highFidelityTireModel.pEy3(idx)= obj.pEy3_tires(idx);
                    obj.highFidelityTireModel.pEy4(idx)= obj.pEy4_tires(idx);
                    obj.highFidelityTireModel.pKy1(idx)= obj.pKy1_tires(idx);
                    obj.highFidelityTireModel.pKy2(idx)= obj.pKy2_tires(idx);
                    obj.highFidelityTireModel.pKy3(idx)= obj.pKy3_tires(idx);
                end
            else
                obj.B_tires = obj.B_tires*adjustmentFactor;
                obj.C_tires = obj.C_tires*adjustmentFactor;
                obj.D_tires = obj.D_tires*adjustmentFactor;
                obj.E_tires = obj.E_tires*adjustmentFactor;
            end

            % Adjust for flat tires
            if ~isempty(obj.flatTireIndices)
                numFlat= length(obj.flatTireIndices);
                debugLog('Adjusting Pacejka params for %d flat tire(s).\n', numFlat);
                for idx= obj.flatTireIndices
                    if idx<1 || idx>length(obj.mu_tires)
                        warning('Flat tire index %d out of range.', idx);
                        continue;
                    end
                    stiffFactor=0.1;
                    peakFactor=0.3;
                    shapeAdjust=1.1;
                    curveAdjust=0.9;
                    frictionRed=0.7;
                    rrIncrease=5;

                    if strcmp(obj.tireModelFlag,'highFidelity')
                        if isempty(obj.highFidelityTireModel)
                            error('High-fidelity tire model missing while adjusting flats.');
                        end
                        obj.pCx1_tires(idx)= obj.pCx1_tires(idx)*stiffFactor;
                        obj.pDx1_tires(idx)= obj.pDx1_tires(idx)*peakFactor;
                        obj.pDx2_tires(idx)= obj.pDx2_tires(idx)*peakFactor;
                        obj.pEx1_tires(idx)= obj.pEx1_tires(idx)*curveAdjust;
                        obj.pEx2_tires(idx)= obj.pEx2_tires(idx)*curveAdjust;
                        obj.pEx3_tires(idx)= obj.pEx3_tires(idx)*curveAdjust;
                        obj.pEx4_tires(idx)= obj.pEx4_tires(idx)*curveAdjust;
                        obj.pKx1_tires(idx)= obj.pKx1_tires(idx)*stiffFactor;
                        obj.pKx2_tires(idx)= obj.pKx2_tires(idx)*stiffFactor;
                        obj.pKx3_tires(idx)= obj.pKx3_tires(idx)*stiffFactor;
                        obj.pCy1_tires(idx)= obj.pCy1_tires(idx)*shapeAdjust;
                        obj.pDy1_tires(idx)= obj.pDy1_tires(idx)*peakFactor;
                        obj.pDy2_tires(idx)= obj.pDy2_tires(idx)*peakFactor;
                        obj.pEy1_tires(idx)= obj.pEy1_tires(idx)*curveAdjust;
                        obj.pEy2_tires(idx)= obj.pEy2_tires(idx)*curveAdjust;
                        obj.pEy3_tires(idx)= obj.pEy3_tires(idx)*curveAdjust;
                        obj.pEy4_tires(idx)= obj.pEy4_tires(idx)*curveAdjust;
                        obj.pKy1_tires(idx)= obj.pKy1_tires(idx)*stiffFactor;
                        obj.pKy2_tires(idx)= obj.pKy2_tires(idx)*stiffFactor;
                        obj.pKy3_tires(idx)= obj.pKy3_tires(idx)*stiffFactor;
                        obj.mu_tires(idx)= obj.mu_tires(idx)*frictionRed;
                        obj.rollingResistanceCoefficients(idx)= obj.rollingResistanceCoefficients(idx)*rrIncrease;
                        debugLog('Adjusted highFidelity tire at index %d for flat.\n', idx);
                    else
                        if isempty(obj.tireModel)
                            error('Simple tireModel missing while adjusting flats.');
                        end
                        obj.B_tires(idx)= obj.B_tires(idx)*stiffFactor;
                        obj.D_tires(idx)= obj.D_tires(idx)*peakFactor;
                        obj.C_tires(idx)= obj.C_tires(idx)*shapeAdjust;
                        obj.E_tires(idx)= obj.E_tires(idx)*curveAdjust;
                        obj.mu_tires(idx)= obj.mu_tires(idx)*frictionRed;
                        obj.rollingResistanceCoefficients(idx)= obj.rollingResistanceCoefficients(idx)*rrIncrease;
                        debugLog('Adjusted simple tire at index %d for flat.\n', idx);
                    end
                end
            end
        end

        %% updatePacejkaBasedOnSuspension
        function obj = updatePacejkaBasedOnSuspension(obj, F_susp, M_susp)
            % Optionally adjust Pacejka parameters based on suspension loads
            F_susp_max = 150000;
            M_susp_max = 50000;
            fr = F_susp/F_susp_max;
            mr = M_susp/M_susp_max;
            fr = max(min(fr,1),-1);
            mr = max(min(mr,1),-1);

            stiffAdj= 1+ 0.03*fr;
            frictionAdj=1- 0.02*abs(fr);
            curveAdj= 1+0.1*mr;

            numTires= size(obj.loadDistribution,1);
            for idx=1:numTires
                if strcmp(obj.tireModelFlag,'highFidelity')
                    obj.pCx1_tires(idx)= obj.pCx1_tires(idx)*stiffAdj;
                    obj.pKx1_tires(idx)= obj.pKx1_tires(idx)*stiffAdj;
                    obj.pKx2_tires(idx)= obj.pKx2_tires(idx)*stiffAdj;
                    obj.pKx3_tires(idx)= obj.pKx3_tires(idx)*stiffAdj;
                    obj.pKy1_tires(idx)= obj.pKy1_tires(idx)*stiffAdj;
                    obj.pKy2_tires(idx)= obj.pKy2_tires(idx)*stiffAdj;
                    obj.pKy3_tires(idx)= obj.pKy3_tires(idx)*stiffAdj;

                    obj.pEy1_tires(idx)= obj.pEy1_tires(idx)*curveAdj;
                    obj.pEy2_tires(idx)= obj.pEy2_tires(idx)*curveAdj;
                    obj.pEy3_tires(idx)= obj.pEy3_tires(idx)*curveAdj;
                    obj.pEy4_tires(idx)= obj.pEy4_tires(idx)*curveAdj;

                    obj.mu_tires(idx)= obj.mu_tires(idx)*frictionAdj;

                    % Sync with highFidelityTireModel if needed...
                else
                    if isempty(obj.tireModel)
                        error('Simple tireModel is missing.');
                    end
                    obj.B_tires(idx)= obj.B_tires(idx)*stiffAdj;
                    obj.C_tires(idx)= obj.C_tires(idx)*stiffAdj;
                    obj.E_tires(idx)= obj.E_tires(idx)*curveAdj;
                    obj.mu_tires(idx)= obj.mu_tires(idx)*frictionAdj;
                end
            end
                debugLog('Pacejka updated based on suspension: stiff=%.3f, friction=%.3f, curve=%.3f.\n',...    
                stiffAdj, frictionAdj, curveAdj);
        end

        %% applyMovingAverageFilter
        function obj = applyMovingAverageFilter(obj)
            forceKeys = fieldnames(obj.calculatedForces);
            for iK=1:length(forceKeys)
                key= forceKeys{iK};
                val = obj.calculatedForces.(key);

            if isnumeric(val) && numel(val) == 1
                % Moving average buffer update for scalar forces
                buffStruct = obj.forceBuffers.(key);
                buffStruct.index = buffStruct.index + 1;
                if buffStruct.index > obj.filterWindowSize
                    buffStruct.index = 1;
                end
                oldSample = buffStruct.data(buffStruct.index);
                buffStruct.data(buffStruct.index) = val;
                buffStruct.sum = buffStruct.sum + val - oldSample;
                if buffStruct.count < obj.filterWindowSize
                    buffStruct.count = buffStruct.count + 1;
                end
                avgVal = buffStruct.sum / buffStruct.count;
                obj.forceBuffers.(key) = buffStruct;
                obj.calculatedForces_filtered.(key) = avgVal;
                end
            end
        end

        %% getSlipRatios
        function slipRatios = getSlipRatios(obj)
            if isempty(obj.wheelSpeeds)
                warning('No wheelSpeeds. Returning zero slipRatios.');
                slipRatios=0; 
                return;
            end
            if isempty(obj.wheelRadius)
                warning('No wheelRadius. Assuming 0.3 m.');
                wheelR=0.3;
            else
                wheelR=obj.wheelRadius;
            end

            v_vehicle= abs(obj.velocity(1));  % approx forward speed
            nWheels= length(obj.wheelSpeeds);
            slipRatios= zeros(nWheels,1);
            for i=1:nWheels
                w_i= obj.wheelSpeeds(i);
                if length(wheelR)==nWheels
                    R_i= wheelR(i);
                else
                    R_i= wheelR;
                end
                v_wheel= w_i*R_i;
                denom= max(v_vehicle, v_wheel);
                if denom<0.1
                    slipRatios(i)=0;
                else
                    slipRatios(i)= (v_vehicle- v_wheel)/ denom;
                end
            end
        end

        %% computeAeroForces
        % Computes aerodynamic drag, side force, and yaw moment in global frame
        function [F_drag_g, F_side_g, M_z_wind] = computeAeroForces(obj, R_veh2glob)
            % Global velocity relative to wind
            vel_glob = R_veh2glob * obj.velocity;
            rel_vel = vel_glob - obj.windVector;
            v_rel = norm(rel_vel(1:2));
            if v_rel == 0
                F_drag_g = [0;0;0];
            else
                F_drag = 0.5 * obj.airDensity * obj.dragCoefficient * obj.frontalArea * v_rel^2;
                dir = rel_vel(1:2) / v_rel;
                F_drag_g = -F_drag * [dir; 0];
            end
            % Lateral wind force
            v_wind_lat = obj.windVector(2);
            if v_wind_lat == 0
                F_side = 0;
            else
                F_side = 0.5 * obj.airDensity * obj.sideForceCoefficient * obj.sideArea * v_wind_lat^2 * sign(v_wind_lat);
            end
            F_side_g = [0; F_side; 0];
            % Yaw moment due to side force
            lever_arm = obj.trackWidth / 2;
            M_z_wind = F_side * lever_arm;
        end
        function obj = applyBrakingForce(obj)
            if obj.vehicleMass>0
                obj.brakingForce= obj.brakingForce; % already set
            else
                obj.brakingForce=0;
            end
            % Update braking force
            obj.calculatedForces.braking = [obj.brakingForce;0;0];
        end
    end
end
