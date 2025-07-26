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
function [dydt, accelerations] = DynamicsUpdater_stateDerivative_wrapper(stateVec)
% DynamicsUpdater_stateDerivative_wrapper - wrapper to call DynamicsUpdater.stateDerivative for MEX
% Usage:
%   [dydt, accelerations] = DynamicsUpdater_stateDerivative_wrapper(stateVec)
% Inputs:
%   stateVec - vector [x; y; theta; p_x; p_y; L_z; phi; p]

    persistent dyn
    if isempty(dyn)
        % Placeholder initialState struct
        initialState.position = [0;0];
        initialState.orientation = 0;
        initialState.velocity = 0;
        initialState.lateralVelocity = 0;
        initialState.yawRate = 0;
        initialState.rollAngle = 0;
        initialState.rollRate = 0;
        % Placeholder class instances and parameters
        vehicleType = 'tractor';
        mass = 10000; friction = 0.8; velocityVec = [0;0;0];
        dragCoeff = 0.3; airDensity = 1.225;
        frontalArea = 10; sideArea = 20; sideForceCoeff = 1.0;
        turnRadius = Inf; loadDist = [0,0,0,mass*9.81,1]; cog = [0;0;0];
        h_CoG = 1.0; angularVel = [0;0;0]; slopeAngle = 0;
        trackWidth = 2.0; wheelbase = 3.5;
        tireModel = struct('B',10,'C',1.9,'D',1.0,'E',0.97);
        suspensionModel = [];
        trailerInertia = 0; dt = 0.01; trailerMass = 0; trailerWheelbase = 0;
        numTrailerTires = 0; trailerBoxMasses = [];
        tireModelFlag = 'simple'; highFidelityModel = [];
        windVector = [0;0;0]; brakeSystem = BrakeSystem();
        fc = ForceCalculator(vehicleType, mass, friction, velocityVec, ...
            dragCoeff, airDensity, frontalArea, sideArea, sideForceCoeff, ...
            turnRadius, loadDist, cog, h_CoG, angularVel, slopeAngle, ...
            trackWidth, wheelbase, tireModel, suspensionModel, trailerInertia, ...
            dt, trailerMass, trailerWheelbase, numTrailerTires, trailerBoxMasses, ...
            tireModelFlag, highFidelityModel, windVector, brakeSystem);
        kin = KinematicsCalculator(fc, 1.0, 2.0, 200000, 5000, 5000, 150000, 4000, 8000, 0.01);
        clutch = Clutch(500, 0.5, 0.5);
        maxGear = 5;
        gearRatios = [3.5, 2.1, 1.4, 1.0, 0.8];
        finalDriveRatio = 2.8;
        shiftUpSpeed = [10, 20, 30, 45, 60];
        shiftDownSpeed = [8, 15, 25, 40, 55];
        engineBrakeTorque = 400;
        shiftDelay = 0.5;
        filterWindowSize = 5;
        transmission = Transmission(maxGear, gearRatios, finalDriveRatio, ...
            shiftUpSpeed, shiftDownSpeed, engineBrakeTorque, shiftDelay, ...
            clutch, filterWindowSize);
        dyn = DynamicsUpdater(fc, kin, initialState, 10000, 3.5, 1.0, 2.0, 0.01, 'tractor', 2.0, 5000, 500, transmission, 0.1);
    end
    [dydt, accelerations] = dyn.stateDerivative(stateVec);
end
