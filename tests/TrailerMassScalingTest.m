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
function tests = TrailerMassScalingTest
    tests = functiontests(localfunctions);
end

function testMassScalingWithMultipleBoxes(testCase)
    vm = VehicleModel([], [], false, 'sim', []);
    vm.initializeDefaultParameters();

    sp = vm.simParams;
    sp.baseTrailerMass = sp.trailerMass;
    sp.trailerNumBoxes = 3;
    sp.trailerMassScaled = false;

    vm.setSimulationParameters(sp);

    verifyEqual(testCase, vm.simParams.trailerMass, sp.baseTrailerMass * 3, 'AbsTol', 1e-10);
end

function testSetSimulationParametersIdempotent(testCase)
    vm = VehicleModel([], [], false, 'sim', []);
    vm.initializeDefaultParameters();

    sp = vm.simParams;
    sp.trailerNumBoxes = 2;
    sp.baseTrailerMass = sp.trailerMass;
    sp.trailerMassScaled = false;

    vm.setSimulationParameters(sp);
    firstMass = vm.simParams.trailerMass;

    % Call again with the already-set parameters
    vm.setSimulationParameters(vm.simParams);
    secondMass = vm.simParams.trailerMass;

    verifyEqual(testCase, firstMass, secondMass, 'AbsTol', 1e-10);
end
