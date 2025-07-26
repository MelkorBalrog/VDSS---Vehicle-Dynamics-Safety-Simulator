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
% @file PlotManager.m
% @brief Generates figures and manages plot updates for the simulation.
%        Handles different views for vehicle trajectories.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}

classdef PlotManager < handle
    % PlotManager class that manages and visualizes plots for vehicle simulations.

    properties
        sharedAx          % Shared axes for plotting vehicle trajectories
        uiManager         % Reference to the UIManager
        collisionDetector % Reference to the CollisionDetector

        % Handles to line objects for updating data each frame:
        veh1Line
        trl1Line
        trl1RearLine
        veh2Line
        trl2Line
        trl2RearLine
        trl1BoxLines      % Handles for additional trailer 1 box trajectories
        trl2BoxLines      % Handles for additional trailer 2 box trajectories

        % Handles to vehicle outlines for animation
        veh1Outline
        trl1Outline
        veh2Outline
        trl2Outline
        trl1BoxOutlines      % Handles for outlines of additional trailer 1 boxes
        trl2BoxOutlines      % Handles for outlines of additional trailer 2 boxes
        trl1BoxWheelPatches  % Handles for wheel patches of additional trailer 1 boxes
        trl2BoxWheelPatches  % Handles for wheel patches of additional trailer 2 boxes
        trl1BoxAxleLines     % Handles for axle lines of additional trailer 1 boxes
        trl2BoxAxleLines     % Handles for axle lines of additional trailer 2 boxes

        % Handles to full vehicle graphics (body and wheels)
        veh1Graphics
        trl1Graphics
        veh2Graphics
        trl2Graphics

        % Handles to initial-position markers
        veh1StartMarker
        trl1StartMarker
        veh2StartMarker
        trl2StartMarker

        % Handles to wheel patches for tires
        veh1WheelPatches
        trl1WheelPatches
        veh2WheelPatches
        trl2WheelPatches

        veh1AxleLines
        trl1AxleLines
        veh2AxleLines
        trl2AxleLines
    end

    methods
        %% Constructor
        function obj = PlotManager(sharedAx, uiManager, collisionDetector)
            % Constructor to initialize the PlotManager with shared axes and managers.
            if nargin < 3
                error('PlotManager requires three input arguments: sharedAx, uiManager, collisionDetector.');
            end
            obj.sharedAx = sharedAx;
            obj.uiManager = uiManager;
            obj.collisionDetector = collisionDetector;

            % Initialize the plotting area (once)
            cla(obj.sharedAx); 
            hold(obj.sharedAx, 'on');
            grid(obj.sharedAx, 'on');
            % Reset axle line handles
            obj.veh1AxleLines = gobjects(0);
            obj.trl1AxleLines = gobjects(0);
            obj.veh2AxleLines = gobjects(0);
            obj.trl2AxleLines = gobjects(0);

            % Basic labeling (only do it once)
            xlabel(obj.sharedAx, 'Longitudinal Distance (m)');
            ylabel(obj.sharedAx, 'Lateral Distance (m)');
            title(obj.sharedAx, 'Vehicle Simulation Trajectories');
            axis(obj.sharedAx, 'equal');

            % Example fixed axis limits or manual. You can set these once:
            xlim(obj.sharedAx, [0, 1000]);
            ylim(obj.sharedAx, [-10, 1010]);
            % Turn on or off as needed:
            % set(obj.sharedAx, 'XLimMode', 'manual', 'YLimMode', 'manual');

            % Pre-create empty line objects (so we can just set data later)
            obj.veh1Line = plot(obj.sharedAx, NaN, NaN, 'r-', 'LineWidth', 1.5, ...
                'DisplayName', 'Vehicle 1 Trajectory');
            obj.trl1Line = plot(obj.sharedAx, NaN, NaN, 'b-', 'LineWidth', 1.5, ...
                'DisplayName', 'Trailer 1 Trajectory');
            obj.trl1RearLine = plot(obj.sharedAx, NaN, NaN, 'b--', 'LineWidth', 1.5, ...
                'DisplayName', 'Trailer 1 Rear');

            obj.veh2Line = plot(obj.sharedAx, NaN, NaN, 'm-', 'LineWidth', 1.5, ...
                'DisplayName', 'Vehicle 2 Trajectory');
            obj.trl2Line = plot(obj.sharedAx, NaN, NaN, 'c-', 'LineWidth', 1.5, ...
                'DisplayName', 'Trailer 2 Trajectory');
            obj.trl2RearLine = plot(obj.sharedAx, NaN, NaN, 'c--', 'LineWidth', 1.5, ...
                'DisplayName', 'Trailer 2 Rear');

            % Pre-create initial position markers
            obj.veh1StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'r', 'filled', ...
                'DisplayName', 'Vehicle 1 Start');
            obj.trl1StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'b', 'filled', ...
                'DisplayName', 'Trailer 1 Start');
            obj.veh2StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'm', 'filled', ...
                'DisplayName', 'Vehicle 2 Start');
            obj.trl2StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'c', 'filled', ...
                'DisplayName', 'Trailer 2 Start');

            % Create vehicle outline handles for animation
            obj.veh1Outline = plot(obj.sharedAx, NaN, NaN, 'r-', 'LineWidth', 2);
            obj.trl1Outline = plot(obj.sharedAx, NaN, NaN, 'b-', 'LineWidth', 2);
            obj.veh2Outline = plot(obj.sharedAx, NaN, NaN, 'm-', 'LineWidth', 2);
            obj.trl2Outline = plot(obj.sharedAx, NaN, NaN, 'c-', 'LineWidth', 2);

            % Reset full vehicle graphic handles
            obj.veh1Graphics = gobjects(0);
            obj.trl1Graphics = gobjects(0);
            obj.veh2Graphics = gobjects(0);
            obj.trl2Graphics = gobjects(0);
            % Initialize additional trailer box line handles
            obj.trl1BoxLines = gobjects(0);
            obj.trl2BoxLines = gobjects(0);
            % Initialize additional trailer box outlines
            obj.trl1BoxOutlines = gobjects(0);
            obj.trl2BoxOutlines = gobjects(0);
            % Initialize additional trailer box wheel patches and axle lines
            obj.trl1BoxWheelPatches = {};
            obj.trl2BoxWheelPatches = {};
            obj.trl1BoxAxleLines = {};
            obj.trl2BoxAxleLines = {};

            % Initialize full vehicle graphic handles
            obj.veh1Graphics = gobjects(0);
            obj.trl1Graphics = gobjects(0);
            obj.veh2Graphics = gobjects(0);
            obj.trl2Graphics = gobjects(0);
        end

        %% Clear Plots
        function clearPlots(obj)
            % Clears all plotted data from the shared axes
            cla(obj.sharedAx);
            hold(obj.sharedAx, 'on');
            grid(obj.sharedAx, 'on');
            % Reset axle line handles
            obj.veh1AxleLines = gobjects(0);
            obj.trl1AxleLines = gobjects(0);
            obj.veh2AxleLines = gobjects(0);
            obj.trl2AxleLines = gobjects(0);

            obj.clearVehicleGraphics();
            % Reset additional trailer box line handles
            delete(obj.trl1BoxLines); obj.trl1BoxLines = gobjects(0);
            delete(obj.trl2BoxLines); obj.trl2BoxLines = gobjects(0);
            % Reset additional trailer box outlines
            delete(obj.trl1BoxOutlines); obj.trl1BoxOutlines = gobjects(0);
            delete(obj.trl2BoxOutlines); obj.trl2BoxOutlines = gobjects(0);
            % Reset wheel patches and axle lines for additional trailer boxes
            if ~isempty(obj.trl1BoxWheelPatches)
                for bi = 1:numel(obj.trl1BoxWheelPatches)
                    delete(obj.trl1BoxWheelPatches{bi});
                end
                obj.trl1BoxWheelPatches = {};
            end
            if ~isempty(obj.trl2BoxWheelPatches)
                for bi = 1:numel(obj.trl2BoxWheelPatches)
                    delete(obj.trl2BoxWheelPatches{bi});
                end
                obj.trl2BoxWheelPatches = {};
            end
            if ~isempty(obj.trl1BoxAxleLines)
                for bi = 1:numel(obj.trl1BoxAxleLines)
                    delete(obj.trl1BoxAxleLines{bi});
                end
                obj.trl1BoxAxleLines = {};
            end
            if ~isempty(obj.trl2BoxAxleLines)
                for bi = 1:numel(obj.trl2BoxAxleLines)
                    delete(obj.trl2BoxAxleLines{bi});
                end
                obj.trl2BoxAxleLines = {};
            end

            % Re-init lines so we don't create duplicates
            obj.veh1Line = plot(obj.sharedAx, NaN, NaN, 'r-', 'LineWidth', 1.5);
            obj.trl1Line = plot(obj.sharedAx, NaN, NaN, 'b-', 'LineWidth', 1.5);
            obj.trl1RearLine = plot(obj.sharedAx, NaN, NaN, 'b--', 'LineWidth', 1.5);

            obj.veh2Line = plot(obj.sharedAx, NaN, NaN, 'm-', 'LineWidth', 1.5);
            obj.trl2Line = plot(obj.sharedAx, NaN, NaN, 'c-', 'LineWidth', 1.5);
            obj.trl2RearLine = plot(obj.sharedAx, NaN, NaN, 'c--', 'LineWidth', 1.5);

            % And the markers
            obj.veh1StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'r', 'filled');
            obj.trl1StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'b', 'filled');
            obj.veh2StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'm', 'filled');
            obj.trl2StartMarker = scatter(obj.sharedAx, NaN, NaN, 100, 'c', 'filled');

            % Recreate vehicle outline handles
            obj.veh1Outline = plot(obj.sharedAx, NaN, NaN, 'r-', 'LineWidth', 2);
            obj.trl1Outline = plot(obj.sharedAx, NaN, NaN, 'b-', 'LineWidth', 2);
            obj.veh2Outline = plot(obj.sharedAx, NaN, NaN, 'm-', 'LineWidth', 2);
            obj.trl2Outline = plot(obj.sharedAx, NaN, NaN, 'c-', 'LineWidth', 2);

            % Reset stored vehicle graphics
            obj.veh1Graphics = gobjects(0);
            obj.trl1Graphics = gobjects(0);
            obj.veh2Graphics = gobjects(0);
            obj.trl2Graphics = gobjects(0);

            % Keep existing axes settings
            xlabel(obj.sharedAx, 'Longitudinal Distance (m)');
            ylabel(obj.sharedAx, 'Lateral Distance (m)');
            title(obj.sharedAx, 'Vehicle Simulation Trajectories');
            axis(obj.sharedAx, 'equal');
        end

        %% Clear only vehicle graphics
        function clearVehicleGraphics(obj)
            delete(obj.veh1Graphics); obj.veh1Graphics = gobjects(0);
            delete(obj.trl1Graphics); obj.trl1Graphics = gobjects(0);
            delete(obj.veh2Graphics); obj.veh2Graphics = gobjects(0);
            delete(obj.trl2Graphics); obj.trl2Graphics = gobjects(0);
        end

        %% Update Trajectories (Fast Approach)
        function updateTrajectories(obj, dataManager, iStep, simParams1, simParams2)
            % Instead of plotting the entire path each time,
            % let's only plot up to iStep by updating XData/YData of the line objects.

            % Vehicle 1 path up to iStep
            Xv1 = dataManager.globalVehicle1Data.X(1:iStep);
            Yv1 = dataManager.globalVehicle1Data.Y(1:iStep);
            set(obj.veh1Line, 'XData', Xv1, 'YData', Yv1);

            % If trailer 1 is present
            if ~isempty(dataManager.globalTrailer1Data.X)
                Xtrl1 = dataManager.globalTrailer1Data.X(1:iStep);
                Ytrl1 = dataManager.globalTrailer1Data.Y(1:iStep);
                set(obj.trl1Line, 'XData', Xtrl1, 'YData', Ytrl1);

                rearX_Trailer1 = Xtrl1 - (simParams1.trailerLength/2).*cos(dataManager.globalTrailer1Data.Theta(1:iStep));
                rearY_Trailer1 = Ytrl1 - (simParams1.trailerLength/2).*sin(dataManager.globalTrailer1Data.Theta(1:iStep));
                set(obj.trl1RearLine, 'XData', rearX_Trailer1, 'YData', rearY_Trailer1);
                % Additional trailer boxes trajectories
                if isfield(dataManager.globalTrailer1Data, 'Boxes') && numel(dataManager.globalTrailer1Data.Boxes) > 1
                    nBoxes1 = numel(dataManager.globalTrailer1Data.Boxes);
                    % Create line handles for extra boxes if needed
                    if numel(obj.trl1BoxLines) < nBoxes1-1
                        for k = (numel(obj.trl1BoxLines)+1):(nBoxes1-1)
                            obj.trl1BoxLines(k) = plot(obj.sharedAx, NaN, NaN, 'b-', 'LineWidth', 1);
                        end
                    end
                    % Update trajectories for each additional box
                    for bi = 2:nBoxes1
                        Xi = dataManager.globalTrailer1Data.Boxes(bi).X(1:iStep);
                        Yi = dataManager.globalTrailer1Data.Boxes(bi).Y(1:iStep);
                        set(obj.trl1BoxLines(bi-1), 'XData', Xi, 'YData', Yi);
                    end
                end
            else
                % If no trailer, set them NaN to keep them hidden
                set(obj.trl1Line, 'XData', NaN, 'YData', NaN);
                set(obj.trl1RearLine, 'XData', NaN, 'YData', NaN);
            end

            % Vehicle 2 path up to iStep
            Xv2 = dataManager.globalVehicle2Data.X(1:iStep);
            Yv2 = dataManager.globalVehicle2Data.Y(1:iStep);
            set(obj.veh2Line, 'XData', Xv2, 'YData', Yv2);

            % If trailer 2 is present
            if ~isempty(dataManager.globalTrailer2Data.X)
                Xtrl2 = dataManager.globalTrailer2Data.X(1:iStep);
                Ytrl2 = dataManager.globalTrailer2Data.Y(1:iStep);
                set(obj.trl2Line, 'XData', Xtrl2, 'YData', Ytrl2);

                rearX_Trailer2 = Xtrl2 - (simParams2.trailerLength/2).*cos(dataManager.globalTrailer2Data.Theta(1:iStep));
                rearY_Trailer2 = Ytrl2 - (simParams2.trailerLength/2).*sin(dataManager.globalTrailer2Data.Theta(1:iStep));
                set(obj.trl2RearLine, 'XData', rearX_Trailer2, 'YData', rearY_Trailer2);
                % Additional trailer boxes trajectories
                if isfield(dataManager.globalTrailer2Data, 'Boxes') && numel(dataManager.globalTrailer2Data.Boxes) > 1
                    nBoxes2 = numel(dataManager.globalTrailer2Data.Boxes);
                    % Create line handles for extra boxes if needed
                    if numel(obj.trl2BoxLines) < nBoxes2-1
                        for k = (numel(obj.trl2BoxLines)+1):(nBoxes2-1)
                            obj.trl2BoxLines(k) = plot(obj.sharedAx, NaN, NaN, 'c-', 'LineWidth', 1);
                        end
                    end
                    % Update trajectories for each additional box
                    for bi = 2:nBoxes2
                        Xi = dataManager.globalTrailer2Data.Boxes(bi).X(1:iStep);
                        Yi = dataManager.globalTrailer2Data.Boxes(bi).Y(1:iStep);
                        set(obj.trl2BoxLines(bi-1), 'XData', Xi, 'YData', Yi);
                    end
                end
            else
                % If no trailer, set them NaN to keep them hidden
                set(obj.trl2Line, 'XData', NaN, 'YData', NaN);
                set(obj.trl2RearLine, 'XData', NaN, 'YData', NaN);
            end
        end

        %% Highlight Initial Positions
        function highlightInitialPositions(obj, dataManager)
            % Update the initial position markers to the actual initial positions

            set(obj.veh1StartMarker, 'XData', dataManager.globalVehicle1Data.X(1), ...
                                     'YData', dataManager.globalVehicle1Data.Y(1));
            if ~isempty(dataManager.globalTrailer1Data.X)
                set(obj.trl1StartMarker, 'XData', dataManager.globalTrailer1Data.X(1), ...
                                         'YData', dataManager.globalTrailer1Data.Y(1));
            else
                set(obj.trl1StartMarker, 'XData', NaN, 'YData', NaN);
            end

            set(obj.veh2StartMarker, 'XData', dataManager.globalVehicle2Data.X(1), ...
                                     'YData', dataManager.globalVehicle2Data.Y(1));
            if ~isempty(dataManager.globalTrailer2Data.X)
                set(obj.trl2StartMarker, 'XData', dataManager.globalTrailer2Data.X(1), ...
                                         'YData', dataManager.globalTrailer2Data.Y(1));
            else
                set(obj.trl2StartMarker, 'XData', NaN, 'YData', NaN);
            end
        end

        %% Update Vehicle Outlines (with wheels)
        function updateVehicleOutlines(obj, dataManager, iStep, vehicleParams1, trailerParams1, vehicleParams2, trailerParams2)
            % Fast update: vehicle and trailer outlines only
            % Vehicle 1 (tractor)
            corners1 = VehiclePlotter.getVehicleCorners( ...
                dataManager.globalVehicle1Data.X(iStep), ...
                dataManager.globalVehicle1Data.Y(iStep), ...
                dataManager.globalVehicle1Data.Theta(iStep), ...
                vehicleParams1, true, 0, vehicleParams1.numTiresPerAxle);
            outline1 = [corners1; corners1(1,:)]; % close polygon
            set(obj.veh1Outline, 'XData', outline1(:,1), 'YData', outline1(:,2));

            % Trailer 1: plot using first entry in Boxes for multi-box configuration
            if ~isempty(trailerParams1) && isfield(dataManager.globalTrailer1Data, 'Boxes') && numel(dataManager.globalTrailer1Data.Boxes) >= 1
                Xi = dataManager.globalTrailer1Data.Boxes(1).X(iStep);
                Yi = dataManager.globalTrailer1Data.Boxes(1).Y(iStep);
                absT1 = dataManager.globalTrailer1Data.Boxes(1).Theta(iStep);
                pBox = trailerParams1;
                if isfield(trailerParams1,'boxNumAxles') && numel(trailerParams1.boxNumAxles)>=1
                    pBox.numAxles = trailerParams1.boxNumAxles(1);
                end
                cornersT1 = VehiclePlotter.getVehicleCorners(...
                    Xi, Yi, absT1, pBox, false, 0, trailerParams1.numTiresPerAxle);
                outlineT1 = [cornersT1; cornersT1(1,:)];
                set(obj.trl1Outline, 'XData', outlineT1(:,1), 'YData', outlineT1(:,2));
            else
                set(obj.trl1Outline, 'XData', NaN, 'YData', NaN);
            end
            % Additional trailer 1 box outlines (absolute orientation)
            if ~isempty(trailerParams1) && isfield(dataManager.globalTrailer1Data, 'Boxes') && numel(dataManager.globalTrailer1Data.Boxes) > 1
                nBoxes1 = numel(dataManager.globalTrailer1Data.Boxes);
                % Create outline handles for extra boxes if needed
                if numel(obj.trl1BoxOutlines) < nBoxes1-1
                    for k = (numel(obj.trl1BoxOutlines)+1):(nBoxes1-1)
                        obj.trl1BoxOutlines(k) = plot(obj.sharedAx, NaN, NaN, 'b-', 'LineWidth', 2);
                    end
                end
                % Update outlines for each additional box using Boxes data
                for bi = 2:nBoxes1
                    Xi = dataManager.globalTrailer1Data.Boxes(bi).X(iStep);
                    Yi = dataManager.globalTrailer1Data.Boxes(bi).Y(iStep);
                    absTi = dataManager.globalTrailer1Data.Boxes(bi).Theta(iStep);
                    pBox = trailerParams1;
                    if isfield(trailerParams1,'boxNumAxles') && numel(trailerParams1.boxNumAxles) >= bi
                        pBox.numAxles = trailerParams1.boxNumAxles(bi);
                    end
                    cornersBi = VehiclePlotter.getVehicleCorners(Xi, Yi, absTi, pBox, false, 0, trailerParams1.numTiresPerAxle);
                    outlineBi = [cornersBi; cornersBi(1,:)];
                    set(obj.trl1BoxOutlines(bi-1), 'XData', outlineBi(:,1), 'YData', outlineBi(:,2));
                end
            end

            % Vehicle 2 (tractor)
            corners2 = VehiclePlotter.getVehicleCorners( ...
                dataManager.globalVehicle2Data.X(iStep), ...
                dataManager.globalVehicle2Data.Y(iStep), ...
                dataManager.globalVehicle2Data.Theta(iStep), ...
                vehicleParams2, true, 0, vehicleParams2.numTiresPerAxle);
            outline2 = [corners2; corners2(1,:)];
            set(obj.veh2Outline, 'XData', outline2(:,1), 'YData', outline2(:,2));

            % Trailer 2: plot using first entry in Boxes for multi-box configuration
            if ~isempty(trailerParams2) && isfield(dataManager.globalTrailer2Data, 'Boxes') && numel(dataManager.globalTrailer2Data.Boxes) >= 1
                Xi2 = dataManager.globalTrailer2Data.Boxes(1).X(iStep);
                Yi2 = dataManager.globalTrailer2Data.Boxes(1).Y(iStep);
                absT2 = dataManager.globalTrailer2Data.Boxes(1).Theta(iStep);
                pBox2 = trailerParams2;
                if isfield(trailerParams2,'boxNumAxles') && numel(trailerParams2.boxNumAxles)>=1
                    pBox2.numAxles = trailerParams2.boxNumAxles(1);
                end
                cornersT2 = VehiclePlotter.getVehicleCorners(...
                    Xi2, Yi2, absT2, pBox2, false, 0, trailerParams2.numTiresPerAxle);
                outlineT2 = [cornersT2; cornersT2(1,:)];
                set(obj.trl2Outline, 'XData', outlineT2(:,1), 'YData', outlineT2(:,2));
            else
                set(obj.trl2Outline, 'XData', NaN, 'YData', NaN);
            end
            % Additional trailer 2 box outlines (absolute orientation)
            if ~isempty(trailerParams2) && isfield(dataManager.globalTrailer2Data, 'Boxes') && numel(dataManager.globalTrailer2Data.Boxes) > 1
                nBoxes2 = numel(dataManager.globalTrailer2Data.Boxes);
                % Create outline handles for extra boxes if needed
                if numel(obj.trl2BoxOutlines) < nBoxes2-1
                    for k = (numel(obj.trl2BoxOutlines)+1):(nBoxes2-1)
                        obj.trl2BoxOutlines(k) = plot(obj.sharedAx, NaN, NaN, 'c-', 'LineWidth', 2);
                    end
                end
                % Update outlines for each additional box using Boxes data
                for bi = 2:nBoxes2
                    Xi = dataManager.globalTrailer2Data.Boxes(bi).X(iStep);
                    Yi = dataManager.globalTrailer2Data.Boxes(bi).Y(iStep);
                    absTi2 = dataManager.globalTrailer2Data.Boxes(bi).Theta(iStep);
                    pBox2 = trailerParams2;
                    if isfield(trailerParams2,'boxNumAxles') && numel(trailerParams2.boxNumAxles) >= bi
                        pBox2.numAxles = trailerParams2.boxNumAxles(bi);
                    end
                    cornersBi = VehiclePlotter.getVehicleCorners(Xi, Yi, absTi2, pBox2, false, 0, trailerParams2.numTiresPerAxle);
                    outlineBi = [cornersBi; cornersBi(1,:)];
                    set(obj.trl2BoxOutlines(bi-1), 'XData', outlineBi(:,1), 'YData', outlineBi(:,2));
                end
            end
            % Update wheel patches (rectangles)
            % Vehicle 1 tires
            steer1 = rad2deg(dataManager.globalVehicle1Data.SteeringAngle(iStep));
            [shX1, shY1] = obj.computeAllWheelShapes( ...
                dataManager.globalVehicle1Data.X(iStep), ...
                dataManager.globalVehicle1Data.Y(iStep), ...
                dataManager.globalVehicle1Data.Theta(iStep), vehicleParams1, steer1);
            for k = 1:numel(shX1)
                set(obj.veh1WheelPatches(k), 'XData', shX1{k}, 'YData', shY1{k});
            end
            % Trailer 1 tires (use first Box data for absolute orientation)
            if ~isempty(trailerParams1) && isfield(dataManager.globalTrailer1Data, 'Boxes') && numel(dataManager.globalTrailer1Data.Boxes) >= 1
                Xi = dataManager.globalTrailer1Data.Boxes(1).X(iStep);
                Yi = dataManager.globalTrailer1Data.Boxes(1).Y(iStep);
                Ti = dataManager.globalTrailer1Data.Boxes(1).Theta(iStep);
                pBox = trailerParams1;
                if isfield(trailerParams1,'boxNumAxles') && ~isempty(trailerParams1.boxNumAxles)
                    pBox.numAxles = trailerParams1.boxNumAxles(1);
                end
                [shT1X, shT1Y] = obj.computeAllWheelShapes(Xi, Yi, Ti, pBox, 0);
                for k = 1:numel(shT1X)
                    set(obj.trl1WheelPatches(k), 'XData', shT1X{k}, 'YData', shT1Y{k});
                end
            end
            % Vehicle 2 tires
            steer2 = rad2deg(dataManager.globalVehicle2Data.SteeringAngle(iStep));
            [shX2, shY2] = obj.computeAllWheelShapes( ...
                dataManager.globalVehicle2Data.X(iStep), ...
                dataManager.globalVehicle2Data.Y(iStep), ...
                dataManager.globalVehicle2Data.Theta(iStep), vehicleParams2, steer2);
            for k = 1:numel(shX2)
                set(obj.veh2WheelPatches(k), 'XData', shX2{k}, 'YData', shY2{k});
            end
            % Trailer 2 tires (use first Box data for absolute orientation)
            if ~isempty(trailerParams2) && isfield(dataManager.globalTrailer2Data, 'Boxes') && numel(dataManager.globalTrailer2Data.Boxes) >= 1
                Xi2 = dataManager.globalTrailer2Data.Boxes(1).X(iStep);
                Yi2 = dataManager.globalTrailer2Data.Boxes(1).Y(iStep);
                Ti2 = dataManager.globalTrailer2Data.Boxes(1).Theta(iStep);
                pBox2 = trailerParams2;
                if isfield(trailerParams2,'boxNumAxles') && ~isempty(trailerParams2.boxNumAxles)
                    pBox2.numAxles = trailerParams2.boxNumAxles(1);
                end
                [shT2X, shT2Y] = obj.computeAllWheelShapes(Xi2, Yi2, Ti2, pBox2, 0);
                for k = 1:numel(shT2X)
                    set(obj.trl2WheelPatches(k), 'XData', shT2X{k}, 'YData', shT2Y{k});
                end
            end
            % Update axle lines for animation (connect left & right tire centers)
            % Vehicle 1 axles
            [xC1, yC1] = obj.computeWheelCenters( ...
                dataManager.globalVehicle1Data.X(iStep), ...
                dataManager.globalVehicle1Data.Y(iStep), ...
                dataManager.globalVehicle1Data.Theta(iStep), vehicleParams1);
            nPairs1 = numel(xC1) / 2;
            for ai = 1:nPairs1
                idx = (ai-1)*2 + 1;
                set(obj.veh1AxleLines(ai), ...
                    'XData', [xC1(idx), xC1(idx+1)], ...
                    'YData', [yC1(idx), yC1(idx+1)]);
            end
            % Trailer 1 axles (absolute orientation from Boxes data)
            if ~isempty(trailerParams1) && isfield(dataManager.globalTrailer1Data, 'Boxes') && numel(dataManager.globalTrailer1Data.Boxes) >= 1
                Xi = dataManager.globalTrailer1Data.Boxes(1).X(iStep);
                Yi = dataManager.globalTrailer1Data.Boxes(1).Y(iStep);
                Ti = dataManager.globalTrailer1Data.Boxes(1).Theta(iStep);
                pBox = trailerParams1;
                if isfield(trailerParams1,'boxNumAxles') && ~isempty(trailerParams1.boxNumAxles)
                    pBox.numAxles = trailerParams1.boxNumAxles(1);
                end
                [xC1T, yC1T] = obj.computeWheelCenters(Xi, Yi, Ti, pBox);
                nPairsT1 = numel(xC1T) / 2;
                for ai = 1:nPairsT1
                    idx = (ai-1)*2 + 1;
                    set(obj.trl1AxleLines(ai), 'XData', [xC1T(idx), xC1T(idx+1)], 'YData', [yC1T(idx), yC1T(idx+1)]);
                end
            end
            % Vehicle 2 axles
            [xC2, yC2] = obj.computeWheelCenters( ...
                dataManager.globalVehicle2Data.X(iStep), ...
                dataManager.globalVehicle2Data.Y(iStep), ...
                dataManager.globalVehicle2Data.Theta(iStep), vehicleParams2);
            nPairs2 = numel(xC2) / 2;
            for ai = 1:nPairs2
                idx = (ai-1)*2 + 1;
                set(obj.veh2AxleLines(ai), ...
                    'XData', [xC2(idx), xC2(idx+1)], ...
                    'YData', [yC2(idx), yC2(idx+1)]);
            end
            % Trailer 2 axles (absolute orientation from Boxes data)
            if ~isempty(trailerParams2) && isfield(dataManager.globalTrailer2Data, 'Boxes') && numel(dataManager.globalTrailer2Data.Boxes) >= 1
                Xi2 = dataManager.globalTrailer2Data.Boxes(1).X(iStep);
                Yi2 = dataManager.globalTrailer2Data.Boxes(1).Y(iStep);
                Ti2 = dataManager.globalTrailer2Data.Boxes(1).Theta(iStep);
                pBox2 = trailerParams2;
                if isfield(trailerParams2,'boxNumAxles') && ~isempty(trailerParams2.boxNumAxles)
                    pBox2.numAxles = trailerParams2.boxNumAxles(1);
                end
                [xC2T, yC2T] = obj.computeWheelCenters(Xi2, Yi2, Ti2, pBox2);
                nPairsT2 = numel(xC2T) / 2;
                for ai = 1:nPairsT2
                    idx = (ai-1)*2 + 1;
                    set(obj.trl2AxleLines(ai), 'XData', [xC2T(idx), xC2T(idx+1)], 'YData', [yC2T(idx), yC2T(idx+1)]);
                end
            end
            % Update wheel patches and axle lines for additional trailer boxes
            if ~isempty(trailerParams1) && isfield(dataManager.globalTrailer1Data, 'Boxes')
                nBoxes1 = numel(dataManager.globalTrailer1Data.Boxes);
                for bi = 2:nBoxes1
                    Xi = dataManager.globalTrailer1Data.Boxes(bi).X(iStep);
                    Yi = dataManager.globalTrailer1Data.Boxes(bi).Y(iStep);
                    Ti = dataManager.globalTrailer1Data.Boxes(bi).Theta(iStep);
                    % Wheel patches
                    pBox = trailerParams1;
                    if isfield(trailerParams1,'boxNumAxles') && numel(trailerParams1.boxNumAxles) >= bi
                        pBox.numAxles = trailerParams1.boxNumAxles(bi);
                    end
                    [shXb, shYb] = obj.computeAllWheelShapes(Xi, Yi, Ti, pBox, 0);
                    for k = 1:numel(shXb)
                        set(obj.trl1BoxWheelPatches{bi-1}(k), 'XData', shXb{k}, 'YData', shYb{k});
                    end
                    % Axle lines
                    [xCb, yCb] = obj.computeWheelCenters(Xi, Yi, Ti, pBox);
                    for ai = 1:(numel(xCb)/2)
                        idx = (ai-1)*2 + 1;
                        set(obj.trl1BoxAxleLines{bi-1}(ai), ...
                            'XData', [xCb(idx), xCb(idx+1)], ...
                            'YData', [yCb(idx), yCb(idx+1)]);
                    end
                end
            end
            if ~isempty(trailerParams2) && isfield(dataManager.globalTrailer2Data, 'Boxes')
                nBoxes2 = numel(dataManager.globalTrailer2Data.Boxes);
                for bi = 2:nBoxes2
                    Xi = dataManager.globalTrailer2Data.Boxes(bi).X(iStep);
                    Yi = dataManager.globalTrailer2Data.Boxes(bi).Y(iStep);
                    Ti = dataManager.globalTrailer2Data.Boxes(bi).Theta(iStep);
                    % Wheel patches
                    pBox2 = trailerParams2;
                    if isfield(trailerParams2,'boxNumAxles') && numel(trailerParams2.boxNumAxles) >= bi
                        pBox2.numAxles = trailerParams2.boxNumAxles(bi);
                    end
                    [shXb, shYb] = obj.computeAllWheelShapes(Xi, Yi, Ti, pBox2, 0);
                    for k = 1:numel(shXb)
                        set(obj.trl2BoxWheelPatches{bi-1}(k), 'XData', shXb{k}, 'YData', shYb{k});
                    end
                    % Axle lines
                    [xCb, yCb] = obj.computeWheelCenters(Xi, Yi, Ti, pBox2);
                    for ai = 1:(numel(xCb)/2)
                        idx = (ai-1)*2 + 1;
                        set(obj.trl2BoxAxleLines{bi-1}(ai), ...
                            'XData', [xCb(idx), xCb(idx+1)], ...
                            'YData', [yCb(idx), yCb(idx+1)]);
                    end
                end
            end
        end

        %% Initialize Wheel Patches for Tires
        function initWheelMarkers(obj, dataManager, vehicleParams1, trailerParams1, vehicleParams2, trailerParams2)
            i0 = 1;
            % Vehicle 1 tires (apply steering angle for front axle)
            steer1 = rad2deg(dataManager.globalVehicle1Data.SteeringAngle(i0));
            [shX1, shY1] = obj.computeAllWheelShapes( ...
                dataManager.globalVehicle1Data.X(i0), ...
                dataManager.globalVehicle1Data.Y(i0), ...
                dataManager.globalVehicle1Data.Theta(i0), vehicleParams1, steer1);
            obj.veh1WheelPatches = gobjects(1, numel(shX1));
            for k = 1:numel(shX1)
                obj.veh1WheelPatches(k) = fill(shX1{k}, shY1{k}, 'k', 'Parent', obj.sharedAx);
            end
            % Trailer 1 tires
            if ~isempty(trailerParams1) && isfield(dataManager.globalTrailer1Data, 'Boxes') && numel(dataManager.globalTrailer1Data.Boxes) >= 1
                tX0 = dataManager.globalTrailer1Data.Boxes(1).X(i0);
                tY0 = dataManager.globalTrailer1Data.Boxes(1).Y(i0);
                tTh0 = dataManager.globalTrailer1Data.Boxes(1).Theta(i0);
                pBox = trailerParams1;
                if isfield(trailerParams1,'boxNumAxles') && ~isempty(trailerParams1.boxNumAxles)
                    pBox.numAxles = trailerParams1.boxNumAxles(1);
                end
                % Compute shapes for first trailer box using its absolute orientation
                [shT1X, shT1Y] = obj.computeAllWheelShapes(tX0, tY0, tTh0, pBox, 0);
                obj.trl1WheelPatches = gobjects(1, numel(shT1X));
                for k = 1:numel(shT1X)
                    obj.trl1WheelPatches(k) = fill(shT1X{k}, shT1Y{k}, 'k', 'Parent', obj.sharedAx);
                end
            else
                obj.trl1WheelPatches = gobjects(0);
            end
            % Vehicle 2 tires (apply steering angle for front axle)
            steer2 = rad2deg(dataManager.globalVehicle2Data.SteeringAngle(i0));
            [shX2, shY2] = obj.computeAllWheelShapes( ...
                dataManager.globalVehicle2Data.X(i0), ...
                dataManager.globalVehicle2Data.Y(i0), ...
                dataManager.globalVehicle2Data.Theta(i0), vehicleParams2, steer2);
            obj.veh2WheelPatches = gobjects(1, numel(shX2));
            for k = 1:numel(shX2)
                obj.veh2WheelPatches(k) = fill(shX2{k}, shY2{k}, 'k', 'Parent', obj.sharedAx);
            end
            % Trailer 2 tires
            if ~isempty(trailerParams2) && isfield(dataManager.globalTrailer2Data, 'Boxes') && numel(dataManager.globalTrailer2Data.Boxes) >= 1
                tX20 = dataManager.globalTrailer2Data.Boxes(1).X(i0);
                tY20 = dataManager.globalTrailer2Data.Boxes(1).Y(i0);
                tTh20 = dataManager.globalTrailer2Data.Boxes(1).Theta(i0);
                pBox2 = trailerParams2;
                if isfield(trailerParams2,'boxNumAxles') && ~isempty(trailerParams2.boxNumAxles)
                    pBox2.numAxles = trailerParams2.boxNumAxles(1);
                end
                % Compute shapes for first trailer box using its absolute orientation
                [shT2X, shT2Y] = obj.computeAllWheelShapes(tX20, tY20, tTh20, pBox2, 0);
                obj.trl2WheelPatches = gobjects(1, numel(shT2X));
                for k = 1:numel(shT2X)
                    obj.trl2WheelPatches(k) = fill(shT2X{k}, shT2Y{k}, 'k', 'Parent', obj.sharedAx);
                end
            else
                obj.trl2WheelPatches = gobjects(0);
            end
            % Initialize axle lines for each vehicle/trailer
            i0 = 1;
            % Vehicle 1 axles
            [xC1, yC1] = obj.computeWheelCenters( ...
                dataManager.globalVehicle1Data.X(i0), ...
                dataManager.globalVehicle1Data.Y(i0), ...
                dataManager.globalVehicle1Data.Theta(i0), vehicleParams1);
            nPairs1 = numel(xC1) / 2;
            obj.veh1AxleLines = gobjects(1, nPairs1);
            for ai = 1:nPairs1
                idx = (ai-1)*2 + 1;
                obj.veh1AxleLines(ai) = plot(obj.sharedAx, ...
                    [xC1(idx),   xC1(idx+1)], ...
                    [yC1(idx),   yC1(idx+1)], ...
                    'k-', 'LineWidth', 2);
            end
            % Trailer 1 axles
            if ~isempty(trailerParams1) && isfield(dataManager.globalTrailer1Data, 'Boxes') && numel(dataManager.globalTrailer1Data.Boxes) >= 1
                tX0  = dataManager.globalTrailer1Data.Boxes(1).X(i0);
                tY0  = dataManager.globalTrailer1Data.Boxes(1).Y(i0);
                tTh0 = dataManager.globalTrailer1Data.Boxes(1).Theta(i0);
                pBox = trailerParams1;
                if isfield(trailerParams1,'boxNumAxles') && ~isempty(trailerParams1.boxNumAxles)
                    pBox.numAxles = trailerParams1.boxNumAxles(1);
                end
                [xCT1, yCT1] = obj.computeWheelCenters(tX0, tY0, tTh0, pBox);
                nPairsT1 = numel(xCT1) / 2;
                obj.trl1AxleLines = gobjects(1, nPairsT1);
                for ai = 1:nPairsT1
                    idx = (ai-1)*2 + 1;
                    obj.trl1AxleLines(ai) = plot(obj.sharedAx, ...
                        [xCT1(idx),   xCT1(idx+1)], ...
                        [yCT1(idx),   yCT1(idx+1)], ...
                        'k-', 'LineWidth', 2);
                end
            else
                obj.trl1AxleLines = gobjects(0);
            end
            % Vehicle 2 axles
            % Vehicle 2 axles
            [xC2, yC2] = obj.computeWheelCenters( ...
                dataManager.globalVehicle2Data.X(i0), ...
                dataManager.globalVehicle2Data.Y(i0), ...
                dataManager.globalVehicle2Data.Theta(i0), vehicleParams2);
            nPairs2 = numel(xC2) / 2;
            obj.veh2AxleLines = gobjects(1, nPairs2);
            for ai = 1:nPairs2
                idx = (ai-1)*2 + 1;
                obj.veh2AxleLines(ai) = plot(obj.sharedAx, ...
                    [xC2(idx),   xC2(idx+1)], ...
                    [yC2(idx),   yC2(idx+1)], ...
                    'k-', 'LineWidth', 2);
            end
            % Trailer 2 axles
            if ~isempty(trailerParams2) && isfield(dataManager.globalTrailer2Data, 'Boxes') && numel(dataManager.globalTrailer2Data.Boxes) >= 1
                tX0  = dataManager.globalTrailer2Data.Boxes(1).X(i0);
                tY0  = dataManager.globalTrailer2Data.Boxes(1).Y(i0);
                tTh0 = dataManager.globalTrailer2Data.Boxes(1).Theta(i0);
                pBox2 = trailerParams2;
                if isfield(trailerParams2,'boxNumAxles') && ~isempty(trailerParams2.boxNumAxles)
                    pBox2.numAxles = trailerParams2.boxNumAxles(1);
                end
                [xCT2, yCT2] = obj.computeWheelCenters(tX0, tY0, tTh0, pBox2);
                nPairsT2 = numel(xCT2) / 2;
                obj.trl2AxleLines = gobjects(1, nPairsT2);
                for ai = 1:nPairsT2
                    idx = (ai-1)*2 + 1;
                    obj.trl2AxleLines(ai) = plot(obj.sharedAx, ...
                        [xCT2(idx),   xCT2(idx+1)], ...
                        [yCT2(idx),   yCT2(idx+1)], ...
                        'k-', 'LineWidth', 2);
                end
            else
                obj.trl2AxleLines = gobjects(0);
            end
            % Additional trailer 1 boxes: wheel patches and axle lines
            if ~isempty(trailerParams1) && isfield(dataManager.globalTrailer1Data, 'Boxes')
                nBoxes1 = numel(dataManager.globalTrailer1Data.Boxes);
                if nBoxes1 > 1
                    obj.trl1BoxWheelPatches = cell(1, nBoxes1-1);
                    obj.trl1BoxAxleLines    = cell(1, nBoxes1-1);
                    for bi = 2:nBoxes1
                        tX0 = dataManager.globalTrailer1Data.Boxes(bi).X(i0);
                        tY0 = dataManager.globalTrailer1Data.Boxes(bi).Y(i0);
                        absTh0 = dataManager.globalTrailer1Data.Boxes(bi).Theta(i0);
                        pBox = trailerParams1;
                        if isfield(trailerParams1,'boxNumAxles') && numel(trailerParams1.boxNumAxles) >= bi
                            pBox.numAxles = trailerParams1.boxNumAxles(bi);
                        end
                        [shXb, shYb] = obj.computeAllWheelShapes(tX0, tY0, absTh0, pBox, 0);
                        np = numel(shXb);
                        obj.trl1BoxWheelPatches{bi-1} = gobjects(1, np);
                        for k = 1:np
                            obj.trl1BoxWheelPatches{bi-1}(k) = fill(shXb{k}, shYb{k}, 'k', 'Parent', obj.sharedAx);
                        end
                        [xCb, yCb] = obj.computeWheelCenters(tX0, tY0, absTh0, pBox);
                        nAx = numel(xCb)/2;
                        obj.trl1BoxAxleLines{bi-1} = gobjects(1, nAx);
                        for ai = 1:nAx
                            idx = (ai-1)*2 + 1;
                            obj.trl1BoxAxleLines{bi-1}(ai) = plot(obj.sharedAx, [xCb(idx), xCb(idx+1)], [yCb(idx), yCb(idx+1)], 'k-', 'LineWidth', 2);
                        end
                    end
                else
                    obj.trl1BoxWheelPatches = {};
                    obj.trl1BoxAxleLines    = {};
                end
            end
            % Additional trailer 2 boxes: wheel patches and axle lines
            if ~isempty(trailerParams2) && isfield(dataManager.globalTrailer2Data, 'Boxes')
                nBoxes2 = numel(dataManager.globalTrailer2Data.Boxes);
                if nBoxes2 > 1
                    obj.trl2BoxWheelPatches = cell(1, nBoxes2-1);
                    obj.trl2BoxAxleLines    = cell(1, nBoxes2-1);
                    for bi = 2:nBoxes2
                        tX0 = dataManager.globalTrailer2Data.Boxes(bi).X(i0);
                        tY0 = dataManager.globalTrailer2Data.Boxes(bi).Y(i0);
                        absTh20 = dataManager.globalTrailer2Data.Boxes(bi).Theta(i0);
                        pBox2 = trailerParams2;
                        if isfield(trailerParams2,'boxNumAxles') && numel(trailerParams2.boxNumAxles) >= bi
                            pBox2.numAxles = trailerParams2.boxNumAxles(bi);
                        end
                        [shXb, shYb] = obj.computeAllWheelShapes(tX0, tY0, absTh20, pBox2, 0);
                        np = numel(shXb);
                        obj.trl2BoxWheelPatches{bi-1} = gobjects(1, np);
                        for k = 1:np
                            obj.trl2BoxWheelPatches{bi-1}(k) = fill(shXb{k}, shYb{k}, 'k', 'Parent', obj.sharedAx);
                        end
                        [xCb, yCb] = obj.computeWheelCenters(tX0, tY0, absTh20, pBox2);
                        nAx = numel(xCb)/2;
                        obj.trl2BoxAxleLines{bi-1} = gobjects(1, nAx);
                        for ai = 1:nAx
                            idx = (ai-1)*2 + 1;
                            obj.trl2BoxAxleLines{bi-1}(ai) = plot(obj.sharedAx, [xCb(idx), xCb(idx+1)], [yCb(idx), yCb(idx+1)], 'k-', 'LineWidth', 2);
                        end
                    end
                else
                    obj.trl2BoxWheelPatches = {};
                    obj.trl2BoxAxleLines    = {};
                end
            end
            % Ensure vehicles are on top of other graphics
            try
                topObjs = [obj.veh1Outline, obj.trl1Outline, obj.veh2Outline, obj.trl2Outline, ...
                           obj.veh1WheelPatches, obj.trl1WheelPatches, obj.veh2WheelPatches, obj.trl2WheelPatches, ...
                           obj.veh1AxleLines, obj.trl1AxleLines, obj.veh2AxleLines, obj.trl2AxleLines];
                % Include additional boxes
                for bi = 1:numel(obj.trl1BoxWheelPatches)
                    topObjs = [topObjs, obj.trl1BoxWheelPatches{bi}, obj.trl1BoxAxleLines{bi}];
                end
                for bi = 1:numel(obj.trl2BoxWheelPatches)
                    topObjs = [topObjs, obj.trl2BoxWheelPatches{bi}, obj.trl2BoxAxleLines{bi}];
                end
                uistack(topObjs, 'top');
            catch
            end
        end

        %% Compute wheel center positions (left and right) for all axles
        function [xCenters, yCenters] = computeWheelCenters(obj, x, y, theta, params)
            tw = params.trackWidth;
            sp = params.axleSpacing;
            nRearAxles = params.numAxles;
            % Determine offsets along vehicle longitudinal axis
            if params.isTractor
                baseOffset = -params.length/2;
                rearOffsets = (1:nRearAxles) * sp - params.length/2;
                frontOffset = -params.length/2 + sp + params.wheelbase;
                offsets = [rearOffsets, frontOffset];
            else
                hd = params.HitchDistance;
                remLen = params.length - hd;
                baseOffset = hd - params.length/2;
                offsets = -remLen/2 + (1:nRearAxles) * sp;
            end
            % Compute origin shift based on baseOffset
            x0 = x + baseOffset * cos(theta);
            y0 = y + baseOffset * sin(theta);
            dirv = [cos(theta); sin(theta)];
            perp = [cos(theta + pi/2); sin(theta + pi/2)];
            nAxlesTotal = numel(offsets);
            xCenters = zeros(nAxlesTotal*2, 1);
            yCenters = zeros(nAxlesTotal*2, 1);
            for i = 1:nAxlesTotal
                pos = offsets(i);
                center = [x0; y0] + pos * dirv;
                leftC  = center + (tw/2) * perp;
                rightC = center - (tw/2) * perp;
                idx = (i-1)*2 + 1;
                xCenters(idx)   = leftC(1);
                yCenters(idx)   = leftC(2);
                xCenters(idx+1) = rightC(1);
                yCenters(idx+1) = rightC(2);
            end
        end
        
        %% Compute wheel polygon shapes for all tires at a given pose
        % Optional steeringAngleDeg (deg) for front axle steering
        function [shapesX, shapesY] = computeAllWheelShapes(obj, x, y, theta, params, steeringAngleDeg)
            % Compute polygon vertex lists for all wheel rectangles
            if nargin < 6
                steeringAngleDeg = 0;
            end
            % compute axle offsets from vehicle body origin and wheel rectangle basis
            len = params.length;
            tw = params.trackWidth;
            numAxles = params.numAxles;
            numTires = params.numTiresPerAxle;
            % baseOffset aligns vehicle body center to simulation origin x,y
            if params.isTractor
                baseOffset = -params.length/2;
            else
                % trailer: origin is hitch point, center at hitch + remainingLength/2
                hd = params.HitchDistance;
                remainingLength = params.length - hd;
                baseOffset = hd - params.length/2;
            end
            % compute axle offsets behind body origin
            sp = params.axleSpacing;
            if params.isTractor
                rearOffsets = (1:numAxles) * sp - params.length/2;
                frontOffset = -params.length/2 + sp + params.wheelbase;
                offsets = [rearOffsets, frontOffset];
            else
                remain = params.length - params.HitchDistance;
                offsets = -remain/2 + (1:numAxles) * sp;
            end
            dirv = [cos(theta); sin(theta)];
            perp = [cos(theta + pi/2); sin(theta + pi/2)];
            % origin shift for wheel centers
            x0 = x + baseOffset * cos(theta);
            y0 = y + baseOffset * sin(theta);
            % local wheel rectangle coords: longitudinal along vehicle (height), lateral across track (width)
            h = params.wheelHeight;  % longitudinal length of tire
            w = params.wheelWidth;   % lateral width of tire
            % localX along vehicle heading, localY across track
            localX = [-h/2,  h/2,  h/2, -h/2, -h/2];
            localY = [-w/2, -w/2,  w/2,  w/2, -w/2];
            shapesX = {};
            shapesY = {};
            gap = 0.01;
            % loop through each axle offset
            for i = 1:length(offsets)
                pos = offsets(i);
                center = [x0; y0] + pos * dirv;
                leftC = center + (tw/2) * perp;
                rightC = center - (tw/2) * perp;
                % decide tire count for this axle (only tractor front axle gets 2 tires)
                if params.isTractor && i == length(offsets)
                    tiresThisAxle = 2;
                else
                    tiresThisAxle = numTires;
                end
                % compute wheel center positions
                wheelCenters = {};
                if tiresThisAxle == 2
                    wheelCenters = {leftC, rightC};
                else
                    % dual tires: separate each side by small lateral gap
                    d = (w + gap)/2;
                    offVec = d * perp;
                    wheelCenters = {leftC+offVec, leftC-offVec, rightC+offVec, rightC-offVec};
                end
                % orientation for wheels
                phi = 0;
                if i == length(offsets)
                    phi = deg2rad(steeringAngleDeg);
                end
                Rw = [cos(theta+phi), -sin(theta+phi); sin(theta+phi), cos(theta+phi)];
                % build shapes
                for wc = wheelCenters
                    v = wc{1};
                    rot = Rw * [localX; localY];
                    shapesX{end+1} = rot(1,:) + v(1);
                    shapesY{end+1} = rot(2,:) + v(2);
                end
            end
        end

        %% Set Axis Limits (Optional to call once if you want)
        function setAxisLimits(obj, dataManager, stepsToPlot, collisionDetected)
            % You can still set axis limits once based on data
            % ...
            % For faster animation, consider not calling this at every frame.
            relevantX = [];
            relevantY = [];
            if collisionDetected
                relevantX = [relevantX; dataManager.globalVehicle1Data.X(1:stepsToPlot)];
                relevantX = [relevantX; dataManager.globalVehicle2Data.X(1:stepsToPlot)];
                relevantY = [relevantY; dataManager.globalVehicle1Data.Y(1:stepsToPlot)];
                relevantY = [relevantY; dataManager.globalVehicle2Data.Y(1:stepsToPlot)];
                if ~isempty(dataManager.globalTrailer1Data.X)
                    relevantX = [relevantX; dataManager.globalTrailer1Data.X(1:stepsToPlot)];
                    relevantY = [relevantY; dataManager.globalTrailer1Data.Y(1:stepsToPlot)];
                end
                if ~isempty(dataManager.globalTrailer2Data.X)
                    relevantX = [relevantX; dataManager.globalTrailer2Data.X(1:stepsToPlot)];
                    relevantY = [relevantY; dataManager.globalTrailer2Data.Y(1:stepsToPlot)];
                end
            else
                relevantX = [relevantX; dataManager.globalVehicle1Data.X(:)];
                relevantX = [relevantX; dataManager.globalVehicle2Data.X(:)];
                relevantY = [relevantY; dataManager.globalVehicle1Data.Y(:)];
                relevantY = [relevantY; dataManager.globalVehicle2Data.Y(:)];
                if ~isempty(dataManager.globalTrailer1Data.X)
                    relevantX = [relevantX; dataManager.globalTrailer1Data.X(:)];
                    relevantY = [relevantY; dataManager.globalTrailer1Data.Y(:)];
                end
                if ~isempty(dataManager.globalTrailer2Data.X)
                    relevantX = [relevantX; dataManager.globalTrailer2Data.X(:)];
                    relevantY = [relevantY; dataManager.globalTrailer2Data.Y(:)];
                end
            end

            if ~isempty(relevantX)
                minX = min(relevantX) - 10;
                maxX = max(relevantX) + 10;
                minY = min(relevantY) - 10;
                maxY = max(relevantY) + 10;

                if minX == maxX, minX = minX - 1; maxX = maxX + 1; end
                if minY == maxY, minY = minY - 1; maxY = maxY + 1; end

                xlim(obj.sharedAx, [minX, maxX]);
                ylim(obj.sharedAx, [minY, maxY]);
            end

            axis(obj.sharedAx, 'equal');
        end

        %% Plot Vehicles at a Single Step (Unchanged, but removed repeated axis calls)
        function plotVehicles(obj, dataManager, plotStep, vehicleParams1, trailerParams1, ...
                              vehicleParams2, trailerParams2, steeringAnglesSim1, steeringAnglesSim2)
            % Plot tractors and trailers at the current step. Previous vehicle
            % graphics are removed for a smooth animation.

            obj.clearVehicleGraphics();

            % Convert steering angles from radians to degrees
            steeringAngleTractor1 = rad2deg(steeringAnglesSim1(plotStep));
            steeringAngleTractor2 = rad2deg(steeringAnglesSim2(plotStep));

            % Plot Trailers First
            if ~isempty(dataManager.globalTrailer1Data.X)
                % compute world position of tractor middle axle for alignment
                th1 = dataManager.globalVehicle1Data.Theta(plotStep);
                len1 = vehicleParams1.length;
                axSp1 = vehicleParams1.axleSpacing;
                numAx1 = vehicleParams1.numAxles;
                midIdx1 = ceil(numAx1/2);
                frontOff1 = len1/2;
                midOff1 = midIdx1 * axSp1;
                xCOM1 = dataManager.globalVehicle1Data.X(plotStep);
                yCOM1 = dataManager.globalVehicle1Data.Y(plotStep);
                xFront1 = xCOM1 + frontOff1 * cos(th1);
                yFront1 = yCOM1 + frontOff1 * sin(th1);
                xMidAx1 = xFront1 - midOff1 * cos(th1);
                yMidAx1 = yFront1 - midOff1 * sin(th1);
                % compute trailer hitch based on tractor middle axle alignment
                hDist = trailerParams1.HitchDistance;
                hitchX = xMidAx1 - hDist * cos(th1);
                hitchY = yMidAx1 - hDist * sin(th1);
                % adjust to mid-point between middle and rear axle
                hitchX = hitchX - (vehicleParams1.axleSpacing/2) * cos(th1);
                hitchY = hitchY - (vehicleParams1.axleSpacing/2) * sin(th1);
                % trailer orientation: absolute = tractor orientation + trailer relative yaw
                relTheta1 = dataManager.globalTrailer1Data.Theta(plotStep);
                absTheta1 = dataManager.globalVehicle1Data.Theta(plotStep) + relTheta1;
                % plot primary trailer box
                obj.trl1Graphics = VehiclePlotter.plotVehicle(obj.sharedAx, ...
                    hitchX, hitchY, absTheta1, trailerParams1, 'b', false, false, 0, trailerParams1.numTiresPerAxle, trailerParams1.numAxles);
                % plot additional trailer boxes sequentially
                if isfield(dataManager.globalTrailer1Data, 'Boxes') && numel(dataManager.globalTrailer1Data.Boxes) > 1
                    nBoxes1 = numel(dataManager.globalTrailer1Data.Boxes);
                    for bi = 2:nBoxes1
                        Xi = dataManager.globalTrailer1Data.Boxes(bi).X(plotStep);
                        Yi = dataManager.globalTrailer1Data.Boxes(bi).Y(plotStep);
                        relTi = dataManager.globalTrailer1Data.Boxes(bi).Theta(plotStep);
                        absTi = dataManager.globalVehicle1Data.Theta(plotStep) + relTi;
                        hBox = VehiclePlotter.plotVehicle(obj.sharedAx, ...
                            Xi, Yi, absTi, trailerParams1, 'b', false, false, 0, trailerParams1.numTiresPerAxle, trailerParams1.numAxles);
                        obj.trl1Graphics = [obj.trl1Graphics, hBox];
                    end
                end
            end
            if ~isempty(dataManager.globalTrailer2Data.X)
                % compute world position of tractor 2 middle axle for alignment
                th2 = dataManager.globalVehicle2Data.Theta(plotStep);
                len2 = vehicleParams2.length;
                axSp2 = vehicleParams2.axleSpacing;
                numAx2 = vehicleParams2.numAxles;
                midIdx2 = ceil(numAx2/2);
                frontOff2 = len2/2;
                midOff2 = midIdx2 * axSp2;
                xCOM2 = dataManager.globalVehicle2Data.X(plotStep);
                yCOM2 = dataManager.globalVehicle2Data.Y(plotStep);
                xFront2 = xCOM2 + frontOff2 * cos(th2);
                yFront2 = yCOM2 + frontOff2 * sin(th2);
                xMidAx2 = xFront2 - midOff2 * cos(th2);
                yMidAx2 = yFront2 - midOff2 * sin(th2);
                % compute trailer hitch based on tractor2 middle axle alignment
                hDist2 = trailerParams2.HitchDistance;
                hitchX2 = xMidAx2 - hDist2 * cos(th2);
                hitchY2 = yMidAx2 - hDist2 * sin(th2);
                % adjust to mid-point between middle and rear axle of tractor2
                hitchX2 = hitchX2 - (vehicleParams2.axleSpacing/2) * cos(th2);
                hitchY2 = hitchY2 - (vehicleParams2.axleSpacing/2) * sin(th2);
                % trailer2 orientation: absolute = tractor2 orientation + trailer2 relative yaw
                relTheta2 = dataManager.globalTrailer2Data.Theta(plotStep);
                absTheta2 = dataManager.globalVehicle2Data.Theta(plotStep) + relTheta2;
                % plot primary trailer2 box
                obj.trl2Graphics = VehiclePlotter.plotVehicle(obj.sharedAx, ...
                    hitchX2, hitchY2, absTheta2, trailerParams2, 'c', false, false, 0, trailerParams2.numTiresPerAxle, trailerParams2.numAxles);
                % plot additional trailer2 boxes sequentially
                if isfield(dataManager.globalTrailer2Data, 'Boxes') && numel(dataManager.globalTrailer2Data.Boxes) > 1
                    nBoxes2 = numel(dataManager.globalTrailer2Data.Boxes);
                    for bi = 2:nBoxes2
                        Xi = dataManager.globalTrailer2Data.Boxes(bi).X(plotStep);
                        Yi = dataManager.globalTrailer2Data.Boxes(bi).Y(plotStep);
                        relTi2 = dataManager.globalTrailer2Data.Boxes(bi).Theta(plotStep);
                        absTi2 = dataManager.globalVehicle2Data.Theta(plotStep) + relTi2;
                        hBox2 = VehiclePlotter.plotVehicle(obj.sharedAx, ...
                            Xi, Yi, absTi2, trailerParams2, 'c', false, false, 0, trailerParams2.numTiresPerAxle, trailerParams2.numAxles);
                        obj.trl2Graphics = [obj.trl2Graphics, hBox2];
                    end
                end
            end

            % Then plot the tractors
            obj.veh1Graphics = VehiclePlotter.plotVehicle(obj.sharedAx, ...
                dataManager.globalVehicle1Data.X(plotStep), ...
                dataManager.globalVehicle1Data.Y(plotStep), ...
                dataManager.globalVehicle1Data.Theta(plotStep), ...
                vehicleParams1, ...
                'r', ...
                true, ...
                false, ...
                steeringAngleTractor1, ...
                vehicleParams1.numTiresPerAxle, ...
                vehicleParams1.numAxles);

            obj.veh2Graphics = VehiclePlotter.plotVehicle(obj.sharedAx, ...
                dataManager.globalVehicle2Data.X(plotStep), ...
                dataManager.globalVehicle2Data.Y(plotStep), ...
                dataManager.globalVehicle2Data.Theta(plotStep), ...
                vehicleParams2, ...
                'm', ...
                true, ...
                false, ...
                steeringAngleTractor2, ...
                vehicleParams2.numTiresPerAxle, ...
                vehicleParams2.numAxles);
        end

        %% Plot Collision Point
        function plotCollisionPoint(obj, dataManager, plotStep, dt, collisionSeverity, ...
                                    vehicleParams1, trailerParams1, collisionX, collisionY)
            % (Same logic, but remove repeated calls to axis/ylim for performance)
            elapsedTime = ((plotStep - 1) * dt);

            if isnan(collisionX) || isnan(collisionY)
                uialert(obj.uiManager.fig, 'Computed collision coordinates are NaN.', 'Collision Plot Error');
                fprintf('Error: Collision coordinates are NaN.\n');
                return;
            end
            scatter(obj.sharedAx, collisionX, collisionY, 150, 'k', 'filled', 'DisplayName', 'Collision Point');
            uialert(obj.uiManager.fig, ...
                sprintf('Collision at step %d (Time: %.2f s).', plotStep, elapsedTime), ...
                'Collision Alert');

            % Then do the collision severity, etc.
            obj.integrateCollisionSeverity(dataManager, plotStep, dt, collisionSeverity, ...
                                           vehicleParams1, trailerParams1, collisionX, collisionY);
        end

        %% Integrate Collision Severity
        function integrateCollisionSeverity(obj, dataManager, plotStep, dt, collisionSeverity, vehicleParams1, trailerParams1, collisionX, collisionY)
            % Integrates collision severity calculations and displays results.

            % Verify that required fields exist in collisionSeverity
            requiredFields = {'collisionPT', 'collisionPTrailer', 'vehicle2Params', 'vehicle1Params', ...
                              'AverageVehicle1Mass', 'AverageVehicle2Mass', 'bound_type_str', ...
                              'includeTrailer1', 'includeTrailer2'};
            missingFields = setdiff(requiredFields, fieldnames(collisionSeverity));
            if ~isempty(missingFields)
                uialert(obj.uiManager.fig, sprintf('Missing fields in collisionSeverity: %s', strjoin(missingFields, ', ')), ...
                        'Collision Severity Error');
                return;
            end

            % Determine Collision Direction and Set Vehicle Parameters Accordingly
            % **Added checks for trailer presence to prevent inversion when trailers are absent**
            if collisionSeverity.collisionPT
                % Collision from J2980 Vehicle to Truck
                % Vehicle 1: J2980 Vehicle (Target)
                % Vehicle 2: Truck (Bullet)
                m1 = collisionSeverity.vehicle2Params.mass;
                vehicle1_X = dataManager.globalVehicle1Data.X(plotStep);
                vehicle1_Y = dataManager.globalVehicle1Data.Y(plotStep);
                vehicle1_Theta = dataManager.globalVehicle1Data.Theta(plotStep);

                m2 = collisionSeverity.vehicle1Params.mass; % Assuming 'vehicle1Params' holds tractor mass
                vehicle2_X = dataManager.globalVehicle2Data.X(plotStep);
                vehicle2_Y = dataManager.globalVehicle2Data.Y(plotStep);
                vehicle2_Theta = dataManager.globalVehicle2Data.Theta(plotStep);

                % Compute Velocities by Finite Differences
                if plotStep > 1
                    v1_x = (dataManager.globalVehicle1Data.X(plotStep) - dataManager.globalVehicle1Data.X(plotStep - 1)) / dt;
                    v1_y = (dataManager.globalVehicle1Data.Y(plotStep) - dataManager.globalVehicle1Data.Y(plotStep - 1)) / dt;

                    v2_x = (dataManager.globalVehicle2Data.X(plotStep) - dataManager.globalVehicle2Data.X(plotStep - 1)) / dt;
                    v2_y = (dataManager.globalVehicle2Data.Y(plotStep) - dataManager.globalVehicle2Data.Y(plotStep - 1)) / dt;
                else
                    % For the first step, assume initial velocities are zero or predefined
                    v1_x = 0;
                    v1_y = 0;

                    v2_x = 0;
                    v2_y = 0;
                end

            elseif collisionSeverity.collisionPTrailer && collisionSeverity.includeTrailer1 && collisionSeverity.includeTrailer2
                % Collision from Truck to J2980 Vehicle
                % Vehicle 1: Truck (Target)
                % Vehicle 2: J2980 Vehicle (Bullet)
                m1 = collisionSeverity.vehicle1Params.mass;
                vehicle1_X = dataManager.globalVehicle2Data.X(plotStep);
                vehicle1_Y = dataManager.globalVehicle2Data.Y(plotStep);
                vehicle1_Theta = dataManager.globalVehicle2Data.Theta(plotStep);

                m2 = collisionSeverity.vehicle2Params.mass;
                vehicle2_X = dataManager.globalVehicle1Data.X(plotStep);
                vehicle2_Y = dataManager.globalVehicle1Data.Y(plotStep);
                vehicle2_Theta = dataManager.globalVehicle1Data.Theta(plotStep);

                % Compute Velocities by Finite Differences
                if plotStep > 1
                    v1_x = (dataManager.globalVehicle2Data.X(plotStep) - dataManager.globalVehicle2Data.X(plotStep - 1)) / dt;
                    v1_y = (dataManager.globalVehicle2Data.Y(plotStep) - dataManager.globalVehicle2Data.Y(plotStep - 1)) / dt;

                    v2_x = (dataManager.globalVehicle1Data.X(plotStep) - dataManager.globalVehicle1Data.X(plotStep - 1)) / dt;
                    v2_y = (dataManager.globalVehicle1Data.Y(plotStep) - dataManager.globalVehicle1Data.Y(plotStep - 1)) / dt;
                else
                    % For the first step, assume initial velocities are zero or predefined
                    v1_x = 0;
                    v1_y = 0;

                    v2_x = 0;
                    v2_y = 0;
                end

            else
                % **Handle cases where trailers are not present to prevent inversion**
                % Default to collisionPT logic without inversion
                m1 = collisionSeverity.vehicle2Params.mass;
                vehicle1_X = dataManager.globalVehicle1Data.X(plotStep);
                vehicle1_Y = dataManager.globalVehicle1Data.Y(plotStep);
                vehicle1_Theta = dataManager.globalVehicle1Data.Theta(plotStep);

                m2 = collisionSeverity.vehicle1Params.mass;
                vehicle2_X = dataManager.globalVehicle2Data.X(plotStep);
                vehicle2_Y = dataManager.globalVehicle2Data.Y(plotStep);
                vehicle2_Theta = dataManager.globalVehicle2Data.Theta(plotStep);

                % Compute Velocities by Finite Differences
                if plotStep > 1
                    v1_x = (dataManager.globalVehicle1Data.X(plotStep) - dataManager.globalVehicle1Data.X(plotStep - 1)) / dt;
                    v1_y = (dataManager.globalVehicle1Data.Y(plotStep) - dataManager.globalVehicle1Data.Y(plotStep - 1)) / dt;

                    v2_x = (dataManager.globalVehicle2Data.X(plotStep) - dataManager.globalVehicle2Data.X(plotStep - 1)) / dt;
                    v2_y = (dataManager.globalVehicle2Data.Y(plotStep) - dataManager.globalVehicle2Data.Y(plotStep - 1)) / dt;
                else
                    % For the first step, assume initial velocities are zero or predefined
                    v1_x = 0;
                    v1_y = 0;

                    v2_x = 0;
                    v2_y = 0;
                end
            end

            % Create initial velocity vectors
            collisionSeverity.v1_initial = [v1_x; v1_y; 0]; % Target's velocity
            collisionSeverity.v2_initial = [v2_x; v2_y; 0]; % Bullet's velocity

            % Compute Position Vectors from COM to Collision Point
            r1 = [collisionX - vehicle1_X; collisionY - vehicle1_Y; 0]; % Vector from Vehicle 1 COM to collision point
            r2 = [collisionX - vehicle2_X; collisionY - vehicle2_Y; 0]; % Vector from Vehicle 2 COM to collision point

            % Define Collision Normal Unit Vector
            n_vector = [vehicle2_X - vehicle1_X; vehicle2_Y - vehicle1_Y; 0];
            if norm(n_vector) == 0
                uialert(obj.uiManager.fig, 'Collision normal vector has zero magnitude.', 'Collision Error');
                return;
            end
            n_vector = n_vector / norm(n_vector); % Normalize to get a unit vector

            % Compute Heading Vectors for Both Vehicles
            h1 = [cos(vehicle1_Theta); sin(vehicle1_Theta); 0]; % Heading vector of Vehicle 1
            if norm(h1) == 0
                uialert(obj.uiManager.fig, 'Vehicle 1 heading vector has zero magnitude.', 'Collision Error');
                return;
            end
            h1 = h1 / norm(h1); % Ensure it's a unit vector

            h2 = [cos(vehicle2_Theta); sin(vehicle2_Theta); 0]; % Heading vector of Vehicle 2
            if norm(h2) == 0
                uialert(obj.uiManager.fig, 'Vehicle 2 heading vector has zero magnitude.', 'Collision Error');
                return;
            end
            h2 = h2 / norm(h2); % Ensure it's a unit vector

            % Determine Collision Types Separately for Each Vehicle
            collisionType_vehicle1 = obj.determineCollisionType(n_vector, h1);
            collisionType_vehicle2 = obj.determineCollisionType(-n_vector, h2);

            % Compute Relative Velocity Vector before collision
            v_relative_vec = collisionSeverity.v2_initial - collisionSeverity.v1_initial; % Correct relative velocity vector

            % Compute Coefficient of Restitution using get_cor_takeda
            impact_velocity = norm(v_relative_vec(1:2)); % Only horizontal components
            e = VehicleCollisionSeverity.get_cor_takeda(impact_velocity);
            fprintf('Computed Coefficient of Restitution (e): %.4f\n', e);

            % Instantiate VehicleCollisionSeverity Object with Current Velocities
            collisionSeverityObj = VehicleCollisionSeverity(m1, collisionSeverity.v1_initial, r1, ...
                                                           m2, collisionSeverity.v2_initial, r2, ...
                                                           e, n_vector);

            % Configure Severity Mapping Parameters from UI Inputs
            collisionSeverityObj.J2980AssumedMaxMass = collisionSeverity.J2980AssumedMaxMass;
            collisionSeverityObj.VehicleUnderAnalysisMaxMass = collisionSeverity.VehicleUnderAnalysisMaxMass;

            % Set Bound Type (retrieved from UI selection)
            collisionSeverityObj.bound_type_str = collisionSeverity.bound_type_str;

            % Set Collision Types in collisionSeverity object
            collisionSeverityObj.CollisionType_vehicle1 = collisionType_vehicle1;
            collisionSeverityObj.CollisionType_vehicle2 = collisionType_vehicle2;

            % Calculate Initial Speeds BEFORE Performing Collision
            initialSpeed_target = norm(collisionSeverity.v1_initial(1:2)) * 3.6;         % Target speed in kph
            initialSpeed_bullet = norm(collisionSeverity.v2_initial(1:2)) * 3.6;         % Bullet speed in kph

            % Perform collision to update velocities
            collisionSeverityObj = collisionSeverityObj.PerformCollision();

            % Perform Collision Severity Calculation
            [deltaV_target, deltaV_bullet, TargetSeverity, BulletSeverity] = collisionSeverityObj.CalculateSeverity();

            % Display Results and Inputs in a Message Box with Enhanced Formatting
            msgbox(sprintf(['\nCollision Severity Inputs:\n', ...
                            '- Collision Type for Vehicle 1: %s\n', ...
                            '- Collision Type for Vehicle 2: %s\n', ...
                            '- Initial Speed of Target: %.2f kph\n', ...
                            '- Initial Speed of Bullet: %.2f kph\n', ...
                            '- Average J2980 Vehicle Mass: %.2f kg\n', ...
                            '- Average Vehicle Under Analysis Mass: %.2f kg\n', ...
                            '- Bound Type: %s\n\n', ...
                            'Collision Severity Results:\n', ...
                            '- Target Severity: %s\n', ...
                            '- Bullet Severity: %s\n', ...
                            '- Target Delta-V: %.2f kph\n', ...
                            '- Bullet Delta-V: %.2f kph\n'], ...
                           collisionType_vehicle1, ...
                           collisionType_vehicle2, ...
                           initialSpeed_target, ...
                           initialSpeed_bullet, ...
                           collisionSeverity.J2980AssumedMaxMass, ...
                           collisionSeverity.VehicleUnderAnalysisMaxMass, ...
                           collisionSeverity.bound_type_str, ...
                           TargetSeverity, ...
                           BulletSeverity, ...
                           deltaV_target, ...
                           deltaV_bullet), ...
                   'Collision Severity');

            % Create the formatted report text
            reportText = sprintf(['\nCollision Severity Inputs:\n', ...
                                 '- Collision Type for Vehicle 1: %s\n', ...
                                 '- Collision Type for Vehicle 2: %s\n', ...
                                 '- Initial Speed of Target: %.2f kph\n', ...
                                 '- Initial Speed of Bullet: %.2f kph\n', ...
                                 '- Average J2980 Vehicle Mass: %.2f kg\n', ...
                                 '- Average Vehicle Under Analysis Mass: %.2f kg\n', ...
                                 '- Bound Type: %s\n\n', ...
                                 'Collision Severity Results:\n', ...
                                 '- Target Severity: %s\n', ...
                                 '- Bullet Severity: %s\n', ...
                                 '- Target Delta-V: %.2f kph\n', ...
                                 '- Bullet Delta-V: %.2f kph\n'], ...
                                collisionType_vehicle1, ...
                                collisionType_vehicle2, ...
                                initialSpeed_target, ...
                                initialSpeed_bullet, ...
                                collisionSeverity.J2980AssumedMaxMass, ...
                                collisionSeverity.VehicleUnderAnalysisMaxMass, ...
                                collisionSeverity.bound_type_str, ...
                                TargetSeverity, ...
                                BulletSeverity, ...
                                deltaV_target, ...
                                deltaV_bullet);
        
            % Define the filename with a timestamp to ensure uniqueness
            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
            filename = sprintf('collision_severity_report_%s.txt', timestamp);
        
            % Attempt to open the file for writing
            fileID = fopen(filename, 'w');
            if fileID == -1
                uialert(obj.uiManager.fig, 'Failed to open file for writing.', 'File Error');
                return;
            end
        
            % Write the report text to the file
            fprintf(fileID, '%s', reportText);
        
            % Close the file
            fclose(fileID);
        
            % Optional: Notify the user that the report was saved successfully
            uialert(obj.uiManager.fig, sprintf('Collision severity report saved to "%s".', filename), 'Report Saved');

            % Equal axis scaling
            axis(obj.sharedAx, 'equal');

            % Add Padding to Y-Axis
            padding = 0.8; % 20% padding
            ylim(obj.sharedAx, [0 - padding, 1000 + padding]);
        end

        %% Plot Trajectories (Up to stepsToPlot)
        function plotTrajectories(obj, dataManager, stepsToPlot, simParams1, simParams2)
            % Updates the line data for both vehicles and their trailers up to stepsToPlot

            % Vehicle 1:
            Xv1 = dataManager.globalVehicle1Data.X(1:stepsToPlot);
            Yv1 = dataManager.globalVehicle1Data.Y(1:stepsToPlot);
            set(obj.veh1Line, 'XData', Xv1, 'YData', Yv1);

            if ~isempty(dataManager.globalTrailer1Data.X)
                Xtrl1 = dataManager.globalTrailer1Data.X(1:stepsToPlot);
                Ytrl1 = dataManager.globalTrailer1Data.Y(1:stepsToPlot);
                set(obj.trl1Line, 'XData', Xtrl1, 'YData', Ytrl1);

                rearX_Trailer1 = Xtrl1 - (simParams1.trailerLength / 2) .* ...
                    cos(dataManager.globalTrailer1Data.Theta(1:stepsToPlot));
                rearY_Trailer1 = Ytrl1 - (simParams1.trailerLength / 2) .* ...
                    sin(dataManager.globalTrailer1Data.Theta(1:stepsToPlot));
                set(obj.trl1RearLine, 'XData', rearX_Trailer1, 'YData', rearY_Trailer1);
            else
                % If no trailer, hide lines
                set(obj.trl1Line, 'XData', NaN, 'YData', NaN);
                set(obj.trl1RearLine, 'XData', NaN, 'YData', NaN);
            end

            % Vehicle 2:
            Xv2 = dataManager.globalVehicle2Data.X(1:stepsToPlot);
            Yv2 = dataManager.globalVehicle2Data.Y(1:stepsToPlot);
            set(obj.veh2Line, 'XData', Xv2, 'YData', Yv2);

            if ~isempty(dataManager.globalTrailer2Data.X)
                Xtrl2 = dataManager.globalTrailer2Data.X(1:stepsToPlot);
                Ytrl2 = dataManager.globalTrailer2Data.Y(1:stepsToPlot);
                set(obj.trl2Line, 'XData', Xtrl2, 'YData', Ytrl2);

                rearX_Trailer2 = Xtrl2 - (simParams2.trailerLength / 2) .* ...
                    cos(dataManager.globalTrailer2Data.Theta(1:stepsToPlot));
                rearY_Trailer2 = Ytrl2 - (simParams2.trailerLength / 2) .* ...
                    sin(dataManager.globalTrailer2Data.Theta(1:stepsToPlot));
                set(obj.trl2RearLine, 'XData', rearX_Trailer2, 'YData', rearY_Trailer2);
            else
                % If no trailer, hide lines
                set(obj.trl2Line, 'XData', NaN, 'YData', NaN);
                set(obj.trl2RearLine, 'XData', NaN, 'YData', NaN);
            end
        end

        %% Determine Collision Type
        function collisionType = determineCollisionType(~, n_vector, headingVector)
            % (No change to logic)
            dotProduct = dot(n_vector(1:2), headingVector(1:2));
            dotProduct = max(min(dotProduct, 1), -1);
            angle = acosd(dotProduct);

            if angle < 30
                collisionType = 'Rear-End Collision';
            elseif angle > 150
                collisionType = 'Head-On Collision';
            elseif angle >= 60 && angle <= 120
                collisionType = 'Side Collision';
            else
                collisionType = 'Oblique Collision';
            end
        end
    end
end