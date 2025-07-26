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
% @file Pacejka96TireModel.m
% @brief Implements the Pacejka '96 tire force model.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}
%/**
% * @class Pacejka96TireModel
% * @brief Implements the Pacejka '96 tire model for simulating tire forces.
% *
% * The Pacejka96TireModel class encapsulates the Pacejka '96 tire model, which is widely
% * used in vehicle dynamics to calculate lateral and longitudinal forces based on slip
% * angles and slip ratios. This class includes all necessary coefficients and provides
% * methods to compute the resultant tire forces.
% *
% * @author Miguel
% * @version 1.0
% * @date 2024-10-04
% */
classdef Pacejka96TireModel
    properties
        pCx1
        pDx1
        pDx2
        pEx1
        pEx2
        pEx3
        pEx4
        pKx1
        pKx2
        pKx3
        pCy1
        pDy1
        pDy2
        pEy1
        pEy2
        pEy3
        pEy4
        pKy1
        pKy2
        pKy3
    end
    
    methods
        %/**
        % * @brief Constructor to initialize Pacejka '96 coefficients.
        % *
        % * Initializes a new instance of the Pacejka96TireModel class with the specified coefficients.
        % *
        % * @param coefficients A struct containing all required Pacejka '96 coefficients:
        % *   - pCx1
        % *   - pDx1
        % *   - pDx2
        % *   - pEx1
        % *   - pEx2
        % *   - pEx3
        % *   - pEx4
        % *   - pKx1
        % *   - pKx2
        % *   - pKx3
        % *   - pCy1
        % *   - pDy1
        % *   - pDy2
        % *   - pEy1
        % *   - pEy2
        % *   - pEy3
        % *   - pEy4
        % *   - pKy1
        % *   - pKy2
        % *   - pKy3
        % *
        % * @throws Error if the input struct does not contain all required coefficients.
        % */
        function obj = Pacejka96TireModel(coefficients)
            if nargin ~= 1
                error('Pacejka96TireModel requires a coefficients struct.');
            end
            fieldsRequired = {'pCx1', 'pDx1', 'pDx2', 'pEx1', 'pEx2', 'pEx3', 'pEx4', ...
                              'pKx1', 'pKx2', 'pKx3', 'pCy1', 'pDy1', 'pDy2', ...
                              'pEy1', 'pEy2', 'pEy3', 'pEy4', 'pKy1', 'pKy2', 'pKy3'};
            for i = 1:length(fieldsRequired)
                if ~isfield(coefficients, fieldsRequired{i})
                    error('Missing coefficient: %s', fieldsRequired{i});
                end
                obj.(fieldsRequired{i}) = coefficients.(fieldsRequired{i});
            end
        end
        
        %/**
        % * @brief Calculates the lateral force using the Pacejka '96 model for a specific tire.
        % *
        % * This method computes the lateral tire force based on the slip angle and normal load
        % * using the Pacejka '96 tire model equations for the specified tire index.
        % *
        % * @param alpha Slip angle (radians).
        % * @param F_z Normal load on the tire (N).
        % * @param idx Tire index (integer).
        % *
        % * @return F_y The calculated lateral force (N).
        % *
        % * @warning If the normal load is zero or negative, the lateral force is set to zero.
        % * @warning If any of the calculated intermediate variables (C_y, D_y, K_y, E_y, B_y) are
        % * invalid (e.g., zero, NaN, or Inf), the lateral force is set to zero.
        % */
        function F_y = calculateLateralForce(obj, alpha, F_z, idx)
            % Ensure idx is valid
            if idx < 1 || idx > length(obj.pKy1)
                error('Invalid tire index: %d', idx);
            end
            
            % Extract per-tire parameters
            C_y = obj.pCy1(idx);
            D_y = F_z * (obj.pDy1(idx) + obj.pDy2(idx) * F_z);
            K_y = F_z * (obj.pKy1(idx) + obj.pKy2(idx) * F_z) * sin(2 * atan(F_z / obj.pKy3(idx)));
            E_y = (obj.pEy1(idx) + obj.pEy2(idx) * F_z + obj.pEy3(idx) * F_z^2) * (1 - obj.pEy4(idx) * sign(alpha));
            B_y = K_y / (C_y * D_y);
            
            % Handle potential invalid values
            if isnan(K_y) || isinf(K_y)
                warning('K_y is invalid (NaN or Inf) for tire %d, setting F_y to zero.', idx);
                F_y = 0;
                return;
            end
            if isnan(E_y) || isinf(E_y)
                warning('E_y is invalid (NaN or Inf) for tire %d, setting F_y to zero.', idx);
                F_y = 0;
                return;
            end
            if isnan(B_y) || isinf(B_y)
                %warning('B_y is invalid (NaN or Inf) for tire %d, setting F_y to zero.', idx);
                F_y = 0;
                return;
            end
            
            S_hy = 0; % Lateral shift
            S_vy = 0; % Lateral force shift
            
            % Pacejka formula
            alpha_effective = alpha + S_hy;
            phi_y = B_y * alpha_effective - E_y * (B_y * alpha_effective - atan(B_y * alpha_effective));
            F_y = D_y * sin(C_y * atan(phi_y)) + S_vy;
            
            % Ensure F_y is finite
            if isnan(F_y) || isinf(F_y)
                warning('Calculated F_y is invalid (NaN or Inf) for tire %d, setting F_y to zero.', idx);
                F_y = 0;
            end
        end
        
        %/**
        % * @brief Calculates the longitudinal force using the Pacejka '96 model for a specific tire.
        % *
        % * This method computes the longitudinal tire force based on the slip ratio and normal load
        % * using the Pacejka '96 tire model equations for the specified tire index.
        % *
        % * @param kappa Slip ratio (unitless, typically between -1 and 1).
        % * @param F_z Normal load on the tire (N).
        % * @param idx Tire index (integer).
        % *
        % * @return F_x The calculated longitudinal force (N).
        % *
        % * @warning If the normal load is zero or negative, the longitudinal force is set to zero.
        % * @warning If any of the calculated intermediate variables (C_x, D_x, K_x, E_x, B_x) are
        % * invalid (e.g., zero, NaN, or Inf), the longitudinal force is set to zero.
        % */
        function F_x = calculateLongitudinalForce(obj, kappa, F_z, idx)
            % Ensure idx is valid
            if idx < 1 || idx > length(obj.pKx1)
                error('Invalid tire index: %d', idx);
            end
            
            % Extract per-tire parameters
            C_x = obj.pCx1(idx);
            D_x = F_z * (obj.pDx1(idx) + obj.pDx2(idx) * F_z);
            K_x = F_z * (obj.pKx1(idx) + obj.pKx2(idx) * F_z) * sin(2 * atan(F_z / obj.pKx3(idx)));
            E_x = (obj.pEx1(idx) + obj.pEx2(idx) * F_z + obj.pEx3(idx) * F_z^2) * (1 - obj.pEx4(idx) * sign(kappa));
            B_x = K_x / (C_x * D_x);
            
            % Handle potential invalid values
            if isnan(K_x) || isinf(K_x)
                warning('K_x is invalid (NaN or Inf) for tire %d, setting F_x to zero.', idx);
                F_x = 0;
                return;
            end
            if isnan(E_x) || isinf(E_x)
                warning('E_x is invalid (NaN or Inf) for tire %d, setting F_x to zero.', idx);
                F_x = 0;
                return;
            end
            if isnan(B_x) || isinf(B_x)
                warning('B_x is invalid (NaN or Inf) for tire %d, setting F_x to zero.', idx);
                F_x = 0;
                return;
            end
            
            S_hx = 0; % Longitudinal shift
            S_vx = 0; % Longitudinal force shift
            
            % Pacejka formula
            kappa_effective = kappa + S_hx;
            phi_x = B_x * kappa_effective - E_x * (B_x * kappa_effective - atan(B_x * kappa_effective));
            F_x = D_x * sin(C_x * atan(phi_x)) + S_vx;
            
            % Ensure F_x is finite
            if isnan(F_x) || isinf(F_x)
                warning('Calculated F_x is invalid (NaN or Inf) for tire %d, setting F_x to zero.', idx);
                F_x = 0;
            end
        end
    end
end