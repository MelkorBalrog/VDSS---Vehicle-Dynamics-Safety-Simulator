% Author: Miguel Marina <karel.capek.robotics@gmail.com>
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
% generate_mex.m
% Script to generate MEX functions for performance-critical routines via MATLAB Coder

% Ensure MATLAB Coder is installed and add required source paths
addpath(genpath(fullfile('Source')));

fprintf('Generating MEX for CollisionDetector.checkCollision...\n');
codegen -config:mex CollisionDetector_mex_wrapper -args {zeros(4,2,'double'), zeros(4,2,'double')} -report;

% Add other wrappers as needed for additional hotspots
% Generate MEX for ForceCalculator.computeTireForces
fprintf('Generating MEX for ForceCalculator.computeTireForces...\n');
codegen -config:mex ForceCalculator_computeTireForces_wrapper -args {zeros(4,1,'double'), zeros(4,1,'double'), 0, 0, 0} -report;

% Generate MEX for DynamicsUpdater.stateDerivative
fprintf('Generating MEX for DynamicsUpdater.stateDerivative...\n');
codegen -config:mex DynamicsUpdater_stateDerivative_wrapper -args {zeros(8,1,'double')} -report;

% Generate MEX for StabilityChecker.checkStability
fprintf('Generating MEX for StabilityChecker.checkStability...\n');
codegen -config:mex StabilityChecker_checkStability_wrapper -args {} -report;

% Generate MEX for VehicleCollisionSeverity.CalculateSeverity
fprintf('Generating MEX for VehicleCollisionSeverity.CalculateSeverity...\n');
codegen -config:mex VehicleCollisionSeverity_CalculateSeverity_wrapper -args {} -report;
