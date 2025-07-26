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
function tests = VehicleLocalizerTest
    tests = functiontests(localfunctions);
end

function testLocalization(testCase)
    wps = [0 0; 1 0; 2 0; 3 0];
    loc = VehicleLocalizer(wps, 1.0);
    idx = loc.localize([2.2 0]);
    verifyEqual(testCase, idx, 3);
end

function testDistanceToCurve(testCase)
    wps = [0 0; 1 0; 2 0; 3 0];
    loc = VehicleLocalizer(wps, 1.0);
    upcoming = [Inf Inf 10];
    dist = loc.distanceToNextCurve(1, upcoming);
    verifyEqual(testCase, dist, 2, 'AbsTol', 1e-10);
end
