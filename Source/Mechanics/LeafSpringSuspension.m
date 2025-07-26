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
%! \file LeafSpringSuspension.m
%! \brief Defines the LeafSpringSuspension class for simulating vehicle suspension dynamics.
%!
%! This class models a leaf spring suspension system, including spring stiffness, damping, and load transfer.
%! It provides methods to calculate suspension forces and moments based on the vehicle's state and to update
%! the rest length of the suspension based on the current load.
%!
%! \author Miguel Marina <karel.capek.robotics@gmail.com>

classdef LeafSpringSuspension
    %! \class LeafSpringSuspension
    %! \brief A model for leaf spring vehicle suspension dynamics.
    %!
    %! The LeafSpringSuspension class encapsulates the properties and methods required to
    %! simulate the behavior of a leaf spring suspension system in a vehicle. It includes
    %! parameters for spring stiffness, damping, rest length, vehicle mass, track width, and wheelbase.
    
    properties
        %! \var K
        %! \brief Spring stiffness (N/m).
        K              
        
        %! \var C
        %! \brief Damping coefficient (N·s/m).
        C              
        
        %! \var restLength
        %! \brief Rest length of the suspension (m).
        restLength      
        
        %! \var vehicleMass
        %! \brief Mass of the vehicle (kg).
        vehicleMass     
        
        %! \var trackWidth
        %! \brief Distance between left and right wheels (m).
        trackWidth      
        
        %! \var wheelbase
        %! \brief Distance between front and rear wheels (m).
        wheelbase       
    end
    
    methods
        function obj = LeafSpringSuspension(K, C, restLength, vehicleMass, trackWidth, wheelbase)
            %! \brief Constructor for LeafSpringSuspension.
            %!
            %! Initializes the LeafSpringSuspension object with the specified parameters.
            %!
            %! \param K Spring stiffness (N/m).
            %! \param C Damping coefficient (N·s/m).
            %! \param restLength Rest length of the suspension (m).
            %! \param vehicleMass Mass of the vehicle (kg).
            %! \param trackWidth Distance between left and right wheels (m).
            %! \param wheelbase Distance between front and rear wheels (m).
            %
            %! \throws Error if the number of input arguments is not six.
            
            if nargin ~= 6
                error(['LeafSpringSuspension requires six parameters: ', ...
                       'K, C, restLength, vehicleMass, trackWidth, wheelbase.']);
            end
            obj.K = K;
            obj.C = C;
            obj.restLength = restLength;
            obj.vehicleMass = vehicleMass;
            obj.trackWidth = trackWidth;
            obj.wheelbase = wheelbase;
        end
        
        function [F_suspension, M_suspension] = calculateForcesAndMoments(obj, vehicleState)
            %! \brief Calculates suspension forces and moments based on vehicle state.
            %!
            %! \param vehicleState Struct containing the vehicle's current state.
            %!        Fields:
            %!            verticalDisplacement - Vertical displacement of the suspension (m).
            %!            verticalVelocity - Vertical velocity of the suspension (m/s).
            %!            momentArm - Moment arm for calculating moments (m).
            %!            lateralAcceleration - Lateral acceleration of the vehicle (m/s²).
            %!            longitudinalAcceleration - Longitudinal acceleration of the vehicle (m/s²).
            %!            vehicleLoad - Current load on the suspension (N).
            %!
            %! \return F_suspension Suspension force (N).
            %! \return M_suspension Suspension moment (N·m).
            %
            %! \details
            %! The method calculates the suspension force based on spring displacement and damping.
            %! It also accounts for load transfer due to lateral and longitudinal accelerations.
            
            displacement = vehicleState.verticalDisplacement;
            verticalVelocity = vehicleState.verticalVelocity;
            momentArm = vehicleState.momentArm;
            lateralAcceleration = vehicleState.lateralAcceleration;
            longitudinalAcceleration = vehicleState.longitudinalAcceleration;
            vehicleLoad = vehicleState.vehicleLoad;
    
            % Load Transfer Calculations
            lateralLoadTransfer = (obj.vehicleMass * lateralAcceleration * obj.restLength) / obj.trackWidth;
            longitudinalLoadTransfer = (obj.vehicleMass * longitudinalAcceleration * obj.restLength) / obj.wheelbase;
    
            % Modify the vertical displacement to account for load transfer
            adjustedDisplacement = displacement + (lateralLoadTransfer + longitudinalLoadTransfer) / obj.vehicleMass;
    
            % Spring force: F = -K * adjustedDisplacement
            F_spring = -obj.K * adjustedDisplacement;
    
            % Damping force: F = -C * velocity
            F_damping = -obj.C * verticalVelocity;
    
            % Total suspension force
            F_suspension_total = F_spring + F_damping;
    
            % Moment due to suspension
            M_suspension = momentArm * F_suspension_total;
    
            % Assign outputs
            F_suspension = F_suspension_total;
            M_suspension = M_suspension;
        end
        
        function obj = updateRestLength(obj, vehicleLoad)
            %! \brief Updates the rest length of the suspension based on vehicle load.
            %!
            %! \param vehicleLoad Current load on the suspension (N).
            %
            %! \details
            %! The rest length decreases linearly with increased load.
            
            % Simple linear model: restLength decreases with increased load
            deltaRestLengthPerNewton = 1e-5; % meters per Newton
            obj.restLength = obj.restLength - deltaRestLengthPerNewton * vehicleLoad;
        end
    end
end
