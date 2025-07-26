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
classdef VehicleLocalizer < handle
    % VehicleLocalizer Localizes vehicle position on a map and provides
    % distance to the next curve along the path.

    properties
        waypoints
        waypointSpacing (1,1) double = 1.0
    end

    methods
        function obj = VehicleLocalizer(waypoints, spacing)
            if nargin >= 1 && ~isempty(waypoints)
                if iscell(waypoints)
                    waypoints = cell2mat(waypoints);
                end
                obj.waypoints = double(waypoints);
            else
                obj.waypoints = zeros(0,2);
            end
            if nargin >= 2 && ~isempty(spacing)
                obj.waypointSpacing = spacing;
            end
        end

        function idx = localize(obj, position)
            %LOCALIZE Returns the closest waypoint index to the given position.
            if isempty(obj.waypoints)
                idx = 1;
                return;
            end
            posVec = double(position(:)');
            if numel(posVec) > 2
                posVec = posVec(1:2);
            end
            diffs = obj.waypoints(:,1:2) - posVec;
            [~, idx] = min(sum(diffs.^2, 2));
        end

        function dist = distanceToNextCurve(obj, currentIdx, upcomingRadii)
            %DISTANCETONEXTCURVE Compute distance to first upcoming non-infinite radius
            curveIdx = find(~isinf(upcomingRadii), 1, 'first');
            if isempty(curveIdx)
                dist = Inf;
            else
                dist = (curveIdx-1) * obj.waypointSpacing;
            end
        end
    end
end