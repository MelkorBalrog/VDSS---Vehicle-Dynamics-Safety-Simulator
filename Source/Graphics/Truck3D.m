% Author: Miguel Marina <karel.capek.robotics@gmail.com>
%{
@file Truck3D.m
@brief Assembles a tractor and trailer into a single truck for 3D animation.
%}

classdef Truck3D < handle
    properties
        Tractor Vehicle3D
        Trailer Vehicle3D
        UseGPU = false
        UseParallel = false
        TractorParams
        TrailerParams
    end
    methods
        function obj = Truck3D(tractorParams, trailerParams, useGPU, useParallel)
            if nargin < 4, useParallel = false; end
            if nargin < 3, useGPU = false; end
            obj.UseGPU = useGPU;
            obj.UseParallel = useParallel;
            obj.TractorParams = tractorParams;
            obj.TrailerParams = trailerParams;
            obj.Tractor = Vehicle3D(tractorParams, [1 0 0], useGPU, useParallel);
            if nargin >= 2 && ~isempty(trailerParams)
                obj.Trailer = Vehicle3D(trailerParams, [0 0 1], useGPU, useParallel);
            end
        end

        function addToWorld(obj, world)
            world.addObject(obj.Tractor.Body);
            for t = obj.Tractor.Tires
                world.addObject(t);
            end
            if ~isempty(obj.Trailer)
                world.addObject(obj.Trailer.Body);
                for t = obj.Trailer.Tires
                    world.addObject(t);
                end
            end
        end

        function update(obj, dataManager, step)
            xT = dataManager.globalVehicle1Data.X(step);
            yT = dataManager.globalVehicle1Data.Y(step);
            thT = dataManager.globalVehicle1Data.Theta(step);
            obj.Tractor.setState(xT, yT, thT);
            if ~isempty(obj.Trailer)
                vParams = obj.TractorParams;
                tParams = obj.TrailerParams;
                axSp = vParams.axleSpacing;
                len = vParams.length;
                numAx = vParams.numAxles;
                midIdx = ceil(numAx/2);
                frontOff = len/2;
                midOff = midIdx * axSp;
                xFront = xT + frontOff * cos(thT);
                yFront = yT + frontOff * sin(thT);
                xMid = xFront - midOff * cos(thT);
                yMid = yFront - midOff * sin(thT);
                hDist = tParams.HitchDistance;
                hitchX = xMid - hDist * cos(thT) - (vParams.axleSpacing/2)*cos(thT);
                hitchY = yMid - hDist * sin(thT) - (vParams.axleSpacing/2)*sin(thT);
                relTheta = dataManager.globalTrailer1Data.Theta(step);
                absTheta = thT + relTheta;
                obj.Trailer.setState(hitchX, hitchY, absTheta);
            end
        end

        function drawMeshes(obj, ax)
            if nargin < 2, ax = gca; end
            obj.Tractor.drawMesh(ax);
            if ~isempty(obj.Trailer)
                obj.Trailer.drawMesh(ax);
            end
        end
    end
end
