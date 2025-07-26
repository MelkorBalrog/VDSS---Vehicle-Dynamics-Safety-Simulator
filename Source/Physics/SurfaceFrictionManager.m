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
%{
@file SurfaceFrictionManager.m
@brief Determines friction coefficient for each tire based on lane occupancy.
%}
classdef SurfaceFrictionManager
    properties
        laneMap             % Instance of LaneMap defining the track
        localizer           % Instance of VehicleLocalizer (optional)
        trackMu double = 0.8    % Friction on paved track
        soilMu  double = 0.5    % Friction on soil/off-track
    end
    methods
        function obj = SurfaceFrictionManager(laneMap, localizer, trackMu, soilMu)
            if nargin < 1 || isempty(laneMap)
                error('laneMap must be provided');
            end
            obj.laneMap = laneMap;
            if nargin >= 2 && ~isempty(localizer)
                obj.localizer = localizer;
            else
                obj.localizer = [];
            end
            if nargin >= 3 && ~isempty(trackMu)
                obj.trackMu = trackMu;
            end
            if nargin >= 4 && ~isempty(soilMu)
                obj.soilMu = soilMu;
            end
        end

        function mu = getMuForTirePositions(obj, positions)
            %GETMUFORTIREPOSITIONS Returns friction coefficients per position.
            %   positions: Nx2 matrix of global [x,y] positions.
            n = size(positions,1);
            mu = obj.trackMu * ones(n,1);
            for i=1:n
                x = round(positions(i,1));
                y = round(positions(i,2));
                if x >= 1 && x <= obj.laneMap.NumCellsX && ...
                   y >= 1 && y <= obj.laneMap.NumCellsY
                    val = obj.laneMap.MapData(y,x);
                else
                    val = 0;
                end
                if val > 0
                    mu(i) = obj.trackMu;
                else
                    mu(i) = obj.soilMu;
                end
            end
        end
    end
end
