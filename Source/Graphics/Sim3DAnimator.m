% Author: Miguel Marina <karel.capek.robotics@gmail.com>
%{
@file Sim3DAnimator.m
@brief Animates simulation results using the 3D graphics framework.
%}

classdef Sim3DAnimator < handle
    properties
        DataManager
        GraphicsWindow
        Truck
        Steps
        TractorParams
        TrailerParams
    end

    methods
        function obj = Sim3DAnimator(dataManager, graphicsWindow, truck, tractorParams, trailerParams)
            if nargin < 2 || isempty(graphicsWindow)
                graphicsWindow = GraphicsWindow();
            end
            obj.DataManager = dataManager;
            obj.GraphicsWindow = graphicsWindow;
            obj.Steps = numel(dataManager.globalVehicle1Data.X);
            obj.TractorParams = tractorParams;
            obj.TrailerParams = trailerParams;

            if nargin >= 3 && ~isempty(truck)
                obj.Truck = truck;
            else
                obj.Truck = Truck3D(tractorParams, trailerParams, graphicsWindow.UseGPU, graphicsWindow.UseParallel);
            end
            obj.Truck.addToWorld(obj.GraphicsWindow.World);
        end

        function run(obj, dt)
            if nargin < 2
                dt = obj.DataManager.dt;
            end
            for k = 1:obj.Steps
                obj.updateVehicles(k);
                obj.GraphicsWindow.render();
                pause(dt);
            end
        end

        function updateVehicles(obj, k)
            if k > obj.Steps
                return;
            end
            obj.Truck.update(obj.DataManager, k);
        end
    end
end
