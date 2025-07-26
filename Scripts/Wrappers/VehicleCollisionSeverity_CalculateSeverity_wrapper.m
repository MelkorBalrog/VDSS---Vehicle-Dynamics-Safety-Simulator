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
function [dV1,dV2,severity1,severity2] = VehicleCollisionSeverity_CalculateSeverity_wrapper()
    % VehicleCollisionSeverity_CalculateSeverity_wrapper - wrapper to call VehicleCollisionSeverity.CalculateSeverity for MEX generation

    % Simple placeholder initialization
    m1 = 1000; v1 = [10;0;0]; r1 = [0;0;0];
    m2 = 1500; v2 = [-10;0;0]; r2 = [0;0;0];
    e = 0.3; n = [1;0;0];
    vcs = VehicleCollisionSeverity(m1, v1, r1, m2, v2, r2, e, n);
    vcs.CollisionType_vehicle1 = 'Head-On Collision';
    vcs.CollisionType_vehicle2 = 'Head-On Collision';
    vcs.J2980AssumedMaxMass = 3000;
    vcs.VehicleUnderAnalysisMaxMass = 3000;
    vcs = vcs.PerformCollision();
    [dV1,dV2,severity1,severity2] = vcs.CalculateSeverity();
end

