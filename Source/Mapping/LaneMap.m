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
% @file LaneMap.m
% @brief Represents a 2D lane occupancy grid used for mapping.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}

classdef LaneMap
    properties
        NumCellsX    % Number of cells in the X direction
        NumCellsY    % Number of cells in the Y direction
        MapData      % Matrix to store cell values (0 or 1)
        LaneWidth    % Current lane width in meters
        LaneCommands
        LaneColor
    end
    methods
        %% LaneMap constructor
        % Initializes the lane map with the given dimensions.
        %
        % @param numCellsX  Number of cells along the X axis.
        % @param numCellsY  Number of cells along the Y axis.
        function obj = LaneMap(numCellsX, numCellsY)
            % Constructor to initialize the map with the specified number of cells
            if nargin < 2
                error('Please provide both numCellsX and numCellsY.');
            end
            obj.NumCellsX = numCellsX;
            obj.NumCellsY = numCellsY;
            obj.MapData = zeros(obj.NumCellsY, obj.NumCellsX); % Initialize map data to zeros
            obj.LaneWidth = 5; % Default lane width (can be updated later)
        end

        %% setCellValue
        % Sets the value of a map cell.
        %
        % @param x      Column index in the map grid.
        % @param y      Row index in the map grid.
        % @param value  Value to assign (0 or 1).
        function obj = setCellValue(obj, x, y, value)
            % Set the value of a cell at position (x, y)
            if x >= 1 && x <= obj.NumCellsX && y >= 1 && y <= obj.NumCellsY
                obj.MapData(y, x) = value; % MATLAB matrices are row-major (row, column)
            else
                error('Cell position (%d, %d) is out of range.', x, y);
            end
        end

        %% getCellValue
        % Retrieves the value of a map cell.
        %
        % @param x  Column index in the grid.
        % @param y  Row index in the grid.
        % @retval value  Stored value at the given location.
        function value = getCellValue(obj, x, y)
            % Get the value of a cell at position (x, y)
            if x >= 1 && x <= obj.NumCellsX && y >= 1 && y <= obj.NumCellsY
                value = obj.MapData(y, x);
            else
                error('Cell position (%d, %d) is out of range.', x, y);
            end
        end

        %% addStraightLane
        % Adds a straight lane segment with consistent width.
        %
        % @param startX  Starting X coordinate in cell units.
        % @param startY  Starting Y coordinate in cell units.
        % @param endX    Ending X coordinate in cell units.
        % @param endY    Ending Y coordinate in cell units.
        % @param width   Optional lane width in meters.
        %
        % @retval obj   Updated LaneMap instance.
        % @retval endX  Final X coordinate after the lane is added.
        % @retval endY  Final Y coordinate after the lane is added.
        function [obj, endX, endY] = addStraightLane(obj, startX, startY, endX, endY, width)
            % Add a straight lane with consistent width
            % startX, startY, endX, endY: Coordinates in cell units
            % width: Lane width in meters (optional)
            if nargin < 6 || isempty(width)
                width = obj.LaneWidth; % Use existing lane width if not specified
            end

            % Update LaneWidth property if different
            if width ~= obj.LaneWidth
                obj.LaneWidth = width;
            end

            % Define lane radius (half the width)
            laneRadius = width / 2;

            % Compute direction vector
            dx = endX - startX;
            dy = endY - startY;

            % Compute the normal vector
            normLength = sqrt(dx^2 + dy^2);
            if normLength == 0
                error('Start and end points are the same.');
            end
            nx = -dy / normLength;
            ny = dx / normLength;

            % Compute a list of points along the straight line using Bresenham's algorithm
            [xIndices_orig, yIndices_orig] = obj.bresenhamLine(round(startX), round(startY), round(endX), round(endY));

            % Set lane area around each point
            for i = 1:length(xIndices_orig)
                xi = xIndices_orig(i);
                yi = yIndices_orig(i);
                if xi >= 1 && xi <= obj.NumCellsX && yi >= 1 && yi <= obj.NumCellsY
                    obj = obj.setCircularArea(xi, yi, laneRadius);
                end
            end

            % Debug messages
            fprintf('Added straight lane from (%.2f, %.2f) to (%.2f, %.2f)\n', startX, startY, endX, endY);
            fprintf('Lane radius: %.2f meters\n', laneRadius);

            % Return the end coordinates
            endX = xIndices_orig(end);
            endY = yIndices_orig(end);
        end

        %% addCurvedLane
        % Adds a curved lane segment with consistent width.
        %
        % @param centerX     Center X coordinate of curvature (cell units).
        % @param centerY     Center Y coordinate of curvature (cell units).
        % @param radius      Radius of curvature in cells.
        % @param startAngle  Starting angle in degrees.
        % @param endAngle    Ending angle in degrees.
        % @param direction   'cw' for clockwise or 'ccw' for counter-clockwise.
        % @param width       Optional lane width in meters.
        %
        % @retval obj   Updated LaneMap instance.
        % @retval endX  Final X coordinate after the lane is added.
        % @retval endY  Final Y coordinate after the lane is added.
        function [obj, endX, endY] = addCurvedLane(obj, centerX, centerY, radius, startAngle, endAngle, direction, width)
            % Adds a curved lane with consistent width and specified direction
            % centerX, centerY: Center of curvature in cell units
            % radius: Radius of curvature in cells
            % startAngle, endAngle: Angles in degrees
            % direction: 'cw' for clockwise or 'ccw' for counter-clockwise
            % width: Lane width in meters (optional)
            if nargin < 8 || isempty(width)
                width = obj.LaneWidth; % Use existing lane width if not specified
            end
            if nargin < 7 || isempty(direction)
                direction = 'ccw'; % Default to counter-clockwise if not specified
            end
            if ~ismember(direction, {'cw', 'ccw'})
                error('Direction must be either ''cw'' (clockwise) or ''ccw'' (counter-clockwise).');
            end

            % Update LaneWidth property if different
            if width ~= obj.LaneWidth
                obj.LaneWidth = width;
            end

            % Define lane radius (half the width)
            laneRadius = width / 2;

            % Convert angles from degrees to radians
            thetaStart = deg2rad(startAngle);
            thetaEnd = deg2rad(endAngle);

            % Adjust angle direction based on 'direction' parameter
            if strcmp(direction, 'cw')
                if thetaEnd > thetaStart
                    thetaEnd = thetaEnd - 2*pi;
                end
                theta = linspace(thetaStart, thetaEnd, max(ceil(radius * abs(thetaStart - thetaEnd)) * 5, 2));
            else % 'ccw'
                if thetaEnd < thetaStart
                    thetaEnd = thetaEnd + 2*pi;
                end
                theta = linspace(thetaStart, thetaEnd, max(ceil(radius * abs(thetaEnd - thetaStart)) * 5, 2));
            end

            % Compute x and y positions in cell indices for centerline curve
            x = centerX + radius * cos(theta);
            y = centerY + radius * sin(theta);

            % Round to nearest cell indices
            xIndices = round(x);
            yIndices = round(y);

            % Set lane area around each point
            for i = 1:length(xIndices)
                xi = xIndices(i);
                yi = yIndices(i);
                if xi >= 1 && xi <= obj.NumCellsX && yi >= 1 && yi <= obj.NumCellsY
                    obj = obj.setCircularArea(xi, yi, laneRadius);
                end
            end

            % Debug messages
            directionStr = 'counter-clockwise';
            if strcmp(direction, 'cw')
                directionStr = 'clockwise';
            end
            fprintf('Added curved lane centered at (%.2f, %.2f) with radius %.2f from %.2f° to %.2f° (%s)\n', ...
                centerX, centerY, radius, startAngle, endAngle, directionStr);

            % Calculate end coordinates based on endAngle
            endX = centerX + radius * cos(theta(end));
            endY = centerY + radius * sin(theta(end));
            endX = round(endX);
            endY = round(endY);
        end

        %% plotLaneMapWithCommands
        % Plots a continuous lane map using a string of lane commands.
        %
        % @param ax           Axes handle where the map will be plotted.
        % @param laneCommands Commands separated by '|' describing lane segments.
        % @param laneColor    1x3 RGB vector specifying the lane color.
        %
        % @retval obj Updated LaneMap instance.
        function h = plotLaneMapWithCommands(obj, ax, laneCommands, laneColor)
            % plotLaneMapWithCommands Plots a continuous lane map into the specified axes based on command strings.
            %
            % Parameters:
            %   ax           - Axes handle where the map will be plotted.
            %   laneCommands - String containing lane commands separated by '|'.
            %   laneColor    - 1x3 RGB vector specifying the lane color (e.g., [0 0 1] for blue).
            %
            % Example:
            %   laneCommands = "straight(100,250,200,250)|curve(200,250,50,0,90,ccw)|...";
            %   plotLaneMapWithCommands(ax, laneCommands, [0 0 1]); % Plot in blue.

            % Input Validation
            if nargin < 3
                error('Please provide laneCommands and laneColor.');
            end
            if nargin < 4 || isempty(laneColor)
                laneColor = [0.5 0.5 0.5]; % Default gray
            end
            if isempty(ax) || ~ishandle(ax) || ~strcmp(get(ax, 'Type'), 'axes')
                error('First argument must be a valid axes handle.');
            end
            if length(laneColor) ~= 3 || any(laneColor < 0) || any(laneColor > 1)
                error('laneColor must be a 1x3 RGB vector with values between 0 and 1.');
            end

            % Process Lane Commands
            obj = obj.processLaneCommands(laneCommands);

            % Plot the Map and capture handle
            h = obj.displayMap(ax, laneColor);
        end

        %% displayMap
        % Displays the lane map on a set of axes.
        %
        % @param ax        Axes handle used for plotting.
        % @param laneColor RGB triplet for the lane color.
        function h = displayMap(obj, ax, laneColor)
            % Display the map with lanes in the specified color and overlay grid lines
            % ax: Axes handle where the map will be plotted
            % laneColor: RGB triplet, e.g., [1 0 0] for red
            if nargin < 3
                laneColor = [0.5 0.5 0.5]; % Default gray
            end
            if nargin < 2 || isempty(ax)
                error('Please provide a valid axes handle.');
            end
            if length(laneColor) ~= 3 || any(laneColor < 0) || any(laneColor > 1)
                error('laneColor must be a 1x3 RGB vector with values between 0 and 1.');
            end

            % Manual Morphological Closing: Dilation followed by Erosion
            % Create a disk-shaped structuring element manually
            seSize = ceil(obj.LaneWidth / 2);
            if seSize < 1
                seSize = 1;
            end
            radius = seSize;
            [x, y] = meshgrid(-radius:radius, -radius:radius);
            se = (x.^2 + y.^2) <= radius^2;

            % Perform Dilation using convolution
            dilatedMap = conv2(obj.MapData, se, 'same') >= 1;

            % Perform Erosion: Invert, Dilate, Invert
            invertedDilated = ~dilatedMap;
            erodedMap = conv2(invertedDilated, se, 'same') >= 1;
            erodedMap = ~erodedMap;

            % Perform Closing: Dilation followed by Erosion
            closedMap = erodedMap;

            % Create an RGB image
            % Initialize to white
            rgbImage = ones(obj.NumCellsY, obj.NumCellsX, 3);

            % Assign lane color
            for channel = 1:3
                rgbImage(:,:,channel) = rgbImage(:,:,channel) .* ~closedMap + laneColor(channel) * closedMap;
            end

            % Plot the image on the provided axes
            axes(ax); % Set the current axes to the provided handle
            % Plot the image on the provided axes and return handle
            h = imagesc(ax, rgbImage);
            h.Tag = 'LaneMap';
            % axis(ax, 'equal');
            % axis(ax, [1 obj.NumCellsX 1 obj.NumCellsY]);
            % title(ax, 'Continuous Lane Map');
            % xlabel(ax, 'X Cells');
            % ylabel(ax, 'Y Cells');
            % set(ax, 'YDir', 'normal'); % Ensure Y-axis increases upwards
            % set(ax, 'TickLength', [0 0]); % Remove tick marks for a cleaner look
            grid(ax, 'on'); % Turn on grid lines

            %hold(ax, 'on'); % Hold the axes to overlay grid lines

            % % %% Define Grid Properties
            % gridColor = [0.8 0.8 0.8]; % Light gray color
            % gridLineStyle = '-';       % Solid lines
            % gridLineWidth = 0.5;       % Thin lines
            % 
            % %% Add Vertical Grid Lines
            % for xLine = 1:obj.NumCellsX
            %     line(ax, [xLine xLine], [1 obj.NumCellsY], 'Color', gridColor, ...
            %          'LineStyle', gridLineStyle, 'LineWidth', gridLineWidth);
            % end
            % 
            % %% Add Horizontal Grid Lines
            % for yLine = 1:obj.NumCellsY
            %     line(ax, [1 obj.NumCellsX], [yLine yLine], 'Color', gridColor, ...
            %          'LineStyle', gridLineStyle, 'LineWidth', gridLineWidth);
            % end
            % 
            % hold(ax, 'off'); % Release the hold on the axes

        end
    end

    methods (Access = private)
        %% bresenhamLine
        % Implements Bresenham's line algorithm.
        %
        % @param x1  Starting column index.
        % @param y1  Starting row index.
        % @param x2  Ending column index.
        % @param y2  Ending row index.
        %
        % @retval x  Vector of column indices along the line.
        % @retval y  Vector of row indices along the line.
        function [x, y] = bresenhamLine(~, x1, y1, x2, y2)
            % Bresenham's line algorithm to compute the points of a line
            x1 = round(x1); y1 = round(y1);
            x2 = round(x2); y2 = round(y2);
            dx = abs(x2 - x1);
            dy = abs(y2 - y1);
            steep = abs(dy) > abs(dx);
            if steep
                [x1, y1] = deal(y1, x1);
                [x2, y2] = deal(y2, x2);
                [dx, dy] = deal(dy, dx);
            end
            if x1 > x2
                [x1, x2] = deal(x2, x1);
                [y1, y2] = deal(y2, y1);
            end
            derr = 2 * dy - dx;
            ystep = (y2 > y1) * 2 - 1;
            y = y1;
            x = x1:x2;
            lineY = zeros(size(x));
            for i = 1:length(x)
                lineY(i) = y;
                if derr > 0
                    y = y + ystep;
                    derr = derr - 2 * dx;
                end
                derr = derr + 2 * dy;
            end
            if steep
                [x, y] = deal(lineY, x);
            else
                y = lineY;
            end
        end

        %% setCircularArea
        % Sets all cells within a circular area to 1.
        %
        % @param x      Center column index.
        % @param y      Center row index.
        % @param radius Radius in cells.
        %
        % @retval obj Updated LaneMap instance.
        function obj = setCircularArea(obj, x, y, radius)
            % Set all cells within a circle of given radius around (x, y) to 1
            % Vectorized implementation

            % Define the range
            x_min = max(round(x - radius), 1);
            x_max = min(round(x + radius), obj.NumCellsX);
            y_min = max(round(y - radius), 1);
            y_max = min(round(y + radius), obj.NumCellsY);

            [X, Y] = meshgrid(x_min:x_max, y_min:y_max);
            mask = (X - x).^2 + (Y - y).^2 <= radius^2;
            obj.MapData(y_min:y_max, x_min:x_max) = obj.MapData(y_min:y_max, x_min:x_max) | mask;
        end

        %% processLaneCommands
        % Processes lane command strings and updates the lane map.
        %
        % @param laneCommands String containing lane commands separated by '|'.
        %
        % @retval obj Updated LaneMap instance.
        function obj = processLaneCommands(obj, laneCommands)
            % processLaneCommands Processes lane command strings to interpret lane segments
            % and add them to the LaneMap object.
            %
            % Parameters:
            %   laneCommands - String containing lane commands separated by '|'.

            % Split the command string by '|'
            commands = split(laneCommands, '|');

            % Iterate over each command and process
            for i = 1:length(commands)
                cmd = strtrim(commands(i));
                % Determine command type
                if startsWith(cmd, 'straight')
                    % Parse straight lane parameters
                    params = regexp(cmd, '^straight\(([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)\)$', 'tokens');
                    if isempty(params)
                        error('Invalid straight lane command format: %s', cmd);
                    end
                    params = params{1};
                    params = params{1};
                    x1 = str2double(params{1});
                    y1 = str2double(params{2});
                    x2 = str2double(params{3});
                    y2 = str2double(params{4});
                    % Add straight lane
                    [obj, endX, endY] = obj.addStraightLane(x1, y1, x2, y2, obj.LaneWidth);
                elseif startsWith(cmd, 'curve')
                    % Parse curved lane parameters
                    params = regexp(cmd, '^curve\(([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),\s*(cw|ccw)\)$', 'tokens');
                    if isempty(params)
                        error('Invalid curved lane command format: %s', cmd);
                    end
                    params = params{1};
                    params = params{1};
                    centerX = str2double(params{1});
                    centerY = str2double(params{2});
                    radius = str2double(params{3});
                    startAngle = str2double(params{4});
                    endAngle = str2double(params{5});
                    direction = lower(strtrim(params{6}));
                    % Add curved lane
                    [obj, endX, endY] = obj.addCurvedLane(centerX, centerY, radius, startAngle, endAngle, direction, obj.LaneWidth);
                else
                    error('Unknown lane command: %s', cmd);
                end
            end
        end
    end
end
