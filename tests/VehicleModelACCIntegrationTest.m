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
function tests = VehicleModelACCIntegrationTest
    tests = functiontests(localfunctions);
end

function testACCIntegration(testCase)
    vm = VehicleModel([], [], false, 'sim', []);
    vm.initializeDefaultParameters();
    vm.initializeSim();
    verifyTrue(testCase, isprop(vm, 'accController'));
    verifyNotEmpty(testCase, vm.accController);
    verifyTrue(testCase, isprop(vm, 'localizer'));
    verifyNotEmpty(testCase, vm.localizer);
end
