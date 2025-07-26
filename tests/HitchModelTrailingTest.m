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
function tests = HitchModelTrailingTest
    tests = functiontests(localfunctions);
end

function testTrailingBehavior(testCase)
    stiffness = struct('x',0,'y',0,'z',0,'roll',0,'pitch',0,'yaw',1000);
    damping   = struct('x',0,'y',0,'z',0,'roll',0,'pitch',0,'yaw',100);
    tractorHitchPoint = [0;0;0];
    trailerKingpinPoint = [0;0;0];
    loadDist = [0 0 0 9810];
    dt = 0.1;
    wheelbase = 4;
    hitch = HitchModel(tractorHitchPoint, trailerKingpinPoint, stiffness, damping, pi/2, wheelbase, loadDist, dt, wheelbase/2);
    tractorState = struct('position',[0;0;0],'orientation',[0;0;0],... 
        'velocity',[10;0;0],'angularVelocity',[0;0;0]);
    trailerState = struct('position',[0;0;0],'orientation',[0;0;0.2],... 
        'velocity',[10;0;0],'angularVelocity',[0;0;0]);
    pullingForce = 1000;
    initialDelta = wrapToPi(trailerState.orientation(3) - tractorState.orientation(3));
    for i = 1:5
        [hitch, ~, ~] = hitch.calculateForces(tractorState, trailerState, pullingForce);
        trailerState.orientation(3) = hitch.angularState.psi;
        trailerState.angularVelocity(3) = hitch.angularState.omega;
    end
    finalDelta = wrapToPi(hitch.angularState.psi - tractorState.orientation(3));
    verifyLessThan(testCase, abs(finalDelta), abs(initialDelta));
end

function testNoTrailingAtLowSpeed(testCase)
    stiffness = struct('x',0,'y',0,'z',0,'roll',0,'pitch',0,'yaw',1000);
    damping   = struct('x',0,'y',0,'z',0,'roll',0,'pitch',0,'yaw',100);
    tractorHitchPoint = [0;0;0];
    trailerKingpinPoint = [0;0;0];
    loadDist = [0 0 0 9810];
    dt = 0.1;
    wheelbase = 4;
    hitch = HitchModel(tractorHitchPoint, trailerKingpinPoint, stiffness, damping, pi/2, wheelbase, loadDist, dt, wheelbase/2);
    tractorState = struct('position',[0;0;0],'orientation',[0;0;0],...
        'velocity',[0;0;0],'angularVelocity',[0;0;0]);
    trailerState = struct('position',[0;0;0],'orientation',[0;0;0.2],...
        'velocity',[0;0;0],'angularVelocity',[0;0;0]);
    pullingForce = 1000;
    [hitch, ~, ~] = hitch.calculateForces(tractorState, trailerState, pullingForce);
    verifyEqual(testCase, hitch.angularState.psi, 0.2, 'AbsTol', 1e-6);
end
