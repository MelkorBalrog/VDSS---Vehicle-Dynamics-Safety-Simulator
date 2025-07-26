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
function tests = SurfaceFrictionManagerTest
    tests = functiontests(localfunctions);
end

function testMuForPositions(testCase)
    map = LaneMap(10,10);
    map = map.setCellValue(5,5,1);
    loc = VehicleLocalizer([0 0],1.0);
    mgr = SurfaceFrictionManager(map, loc, 0.8, 0.4);
    mu = mgr.getMuForTirePositions([5 5; 1 1]);
    verifyEqual(testCase, mu, [0.8;0.4], 'AbsTol',1e-10);
end
