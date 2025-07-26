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
% * @file VehicleParameters.m
% * @brief Defines the parameters for the Vehicle and Trailer Simulation.
% */
% * @author Miguel Marina <karel.capek.robotics@gmail.com>
classdef VehicleParameters < handle
    % VehicleParameters Defines parameters for the tractor and trailer.
    %
    % This class encapsulates all the physical and dynamic parameters required
    % for simulating a vehicle and its trailer. It includes methods to
    % calculate contact areas for tires based on their dimensions and positions.
    
    properties
        % --- Physical Dimensions ---
        length          % Length of the vehicle (meters)
        width           % Width of the vehicle (meters)
        height          % Height of the vehicle (meters)
        mass            % Mass of the vehicle (kg)
        h_CoG           % Height of the center of gravity (meters)
        centerOfGravity % 3D Center of gravity [x; y; z] (meters)
        wheelbase       % Wheelbase of the vehicle (meters)
        trackWidth      % Track width of the vehicle (meters)
        
        % --- Tire Specifications ---
        tractorTireHeight % Height of the tractor tires (meters)
        tractorTireWidth  % Width of the tractor tires (meters)
        trailerTireHeight % Height of the trailer tires (meters)
        trailerTireWidth  % Width of the trailer tires (meters)
        
        % --- Axle and Wheel Details ---
        numAxles        % Number of axles
        axleSpacing     % Spacing between axles (meters)
        frontalArea     % Frontal area for aerodynamic calculations (square meters)
        lateralDragCoefficient % Coefficient for lateral drag force
        
        % --- Tire Contact Parameters (New) ---
        contactLength    % Contact length of the tire with the ground (meters)
        boxNumAxles      % Vector with number of axles per trailer box (trailers only)
        boxMasses        % Vector with mass of each trailer box (trailers only)
    end
    
    methods
        %/**
        % * @brief Constructor to initialize the vehicle parameters.
        % *
        % * @param isTractor Boolean indicating if the instance is for the tractor (true) or trailer (false).
        % * @param length Length of the vehicle (meters).
        % * @param width Width of the vehicle (meters).
        % * @param height Height of the vehicle (meters).
        % * @param h_CoG Height of the center of gravity (meters).
        % * @param wheelbase Wheelbase of the vehicle (meters).
        % * @param trackWidth Track width of the vehicle (meters).
        % * @param numAxles Number of axles.
        % * @param axleSpacing Spacing between axles (meters).
        % * @param contactLength Contact length of the tire with the ground (meters, optional).
        % */
        function obj = VehicleParameters(isTractor, length, width, height, h_CoG, wheelbase, trackWidth, numAxles, axleSpacing, contactLength)
            % Constructor to initialize the parameters
            if nargin < 10
                contactLength = 0.2; % Default contact length (meters)
            end
            
            if nargin < 1 || isempty(isTractor)
                isTractor = true; % Default to tractor if not specified
            end
            
            if nargin < 2 || isempty(length)
                length = 6.5; % Default length (meters)
            end
            if nargin < 3 || isempty(width)
                width = 2.5; % Default width (meters)
            end
            if nargin < 4 || isempty(height)
                height = 3.8; % Default height (meters)
            end
            if nargin < 5 || isempty(h_CoG)
                h_CoG = 1.5; % Default center of gravity height (meters)
            end
            if nargin < 6 || isempty(wheelbase)
                wheelbase = 4.0; % Default wheelbase (meters)
            end
            if nargin < 7 || isempty(trackWidth)
                trackWidth = 2.1; % Default track width (meters)
            end
            if nargin < 8 || isempty(numAxles)
                numAxles = 3; % Default number of axles
            end
            if isempty(axleSpacing)
                axleSpacing = 1.310; % Default axle spacing (meters)
            end
            
            % Assign properties
            obj.length = length;
            obj.width = width;
            obj.height = height;
            obj.h_CoG = h_CoG;
            obj.wheelbase = wheelbase;
            obj.trackWidth = trackWidth;
            obj.numAxles = numAxles;
            obj.axleSpacing = axleSpacing;
            obj.contactLength = contactLength; % Assign contact length
            obj.boxNumAxles = []; % Default empty until set for trailers
            obj.boxMasses  = []; % Default empty until set for trailers
            
            % Initialize center of gravity position (assuming geometric center in x and y)
            obj.centerOfGravity = [0; 0; obj.h_CoG];
            
            % Calculate frontal area (simple rectangular approximation)
            obj.frontalArea = obj.width * obj.height;
            
            % Initialize lateral drag coefficient
            obj.lateralDragCoefficient = 0.5; % Default value, can be adjusted as needed
            
            % Initialize tire dimensions to default values
            if isTractor
                obj.tractorTireHeight = 0.5; % Default tractor tire height
                obj.tractorTireWidth = 0.2;  % Default tractor tire width
            else
                obj.trailerTireHeight = 0.5; % Default trailer tire height
                obj.trailerTireWidth = 0.2;  % Default trailer tire width
            end
        end
        
        %/**
        % * @brief Calculates the contact area for tractor tires.
        % *
        % * @return Contact area for a single tractor tire (square meters).
        % */
        function contactArea = getTractorTireContactArea(obj)
            % Calculate contact area for tractor tires
            contactArea = obj.tractorTireWidth * obj.contactLength; % m²
        end
        
        %/**
        % * @brief Calculates the contact area for trailer tires.
        % *
        % * @param numTires (Optional) Number of trailer tires. If not provided, assumes single contact area.
        % * @return Contact area for each trailer tire (square meters).
        % */
        function contactAreas = getTrailerTireContactArea(obj, numTires)
            % Calculate contact area for trailer tires
            if nargin < 2 || isempty(numTires)
                % Return single contact area if number of tires is not specified
                contactAreas = obj.trailerTireWidth * obj.contactLength; % m²
            else
                % Return an array of contact areas for each tire
                singleContactArea = obj.trailerTireWidth * obj.contactLength; % m²
                contactAreas = singleContactArea * ones(numTires, 1); % Vector of contact areas
            end
        end
        
        %/**
        % * @brief Updates the tire dimensions.
        % *
        % * @param tireType String indicating the type of tire ('tractor' or 'trailer').
        % * @param height Height of the tire (meters).
        % * @param width Width of the tire (meters).
        % */
        function updateTireDimensions(obj, tireType, height, width)
            % Update tire dimensions based on tire type
            switch lower(tireType)
                case 'tractor'
                    obj.tractorTireHeight = height;
                    obj.tractorTireWidth = width;
                case 'trailer'
                    obj.trailerTireHeight = height;
                    obj.trailerTireWidth = width;
                otherwise
                    error('Unknown tire type. Use "tractor" or "trailer".');
            end
        end
        
        %/**
        % * @brief Calculates the total contact area for all tractor tires.
        % *
        % * @return Total contact area for all tractor tires (square meters).
        % */
        function totalContactArea = getTotalTractorContactArea(obj)
            % Calculate total contact area for all tractor tires
            totalContactArea = obj.getTractorTireContactArea() * obj.numAxles * 2; % Assuming 2 tires per axle
        end
        
        %/**
        % * @brief Calculates the total contact area for all trailer tires.
        % *
        % * @param numTiresPerAxle Number of tires per axle on the trailer.
        % * @return Total contact area for all trailer tires (square meters).
        % */
        function totalContactArea = getTotalTrailerContactArea(obj, numTiresPerAxle)
            % Calculate total contact area for all trailer tires
            if nargin < 2
                numTiresPerAxle = 2; % Default to 2 tires per axle
            end
            totalContactArea = obj.getTrailerTireContactArea(obj.numAxles * numTiresPerAxle);
        end
        
        %/**
        % * @brief Calculates the center of gravity based on load distribution.
        % *
        % * @param loads Vector of loads (N) on each tire.
        % * @param positions Matrix of positions [x; y; z] for each tire (meters).
        % * @return Center of gravity position [x; y; z] (meters).
        % */
        function centerOfGravity = calculateCenterOfGravity(obj, loads, positions)
            % calculateCenterOfGravity Computes the center of gravity based on loads and positions
            totalLoad = sum(loads);
            if totalLoad == 0
                centerOfGravity = [0; 0; obj.h_CoG];
                return;
            end
            x_CoG = sum(loads .* positions(:,1)) / totalLoad;
            y_CoG = sum(loads .* positions(:,2)) / totalLoad;
            z_CoG = obj.h_CoG; % Assuming uniform height for simplicity
            centerOfGravity = [x_CoG; y_CoG; z_CoG];
        end
    end
end
