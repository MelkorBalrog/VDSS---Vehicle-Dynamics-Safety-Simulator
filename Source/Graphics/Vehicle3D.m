% Author: Miguel Marina <karel.capek.robotics@gmail.com>
%{
@file Vehicle3D.m
@brief Generic vehicle built from boxels and tires based on 2D plotting parameters.
%}

classdef Vehicle3D < handle
    properties
        Body   % Object3D representing the main body
        Tires  % array of Tire3D objects
        Params % struct of vehicle parameters from 2D plot
        UseGPU = false
        UseParallel = false
    end
    methods
        function obj = Vehicle3D(params, bodyColor, useGPU, useParallel)
            if nargin < 4
                useParallel = false;
            end
            if nargin < 3
                useGPU = false;
            end
            if nargin < 2 || isempty(bodyColor)
                bodyColor = [0.8 0.2 0.2];
            end
            obj.Params = params;
            obj.UseGPU = useGPU;
            obj.UseParallel = useParallel;
            obj.Body = VehiclePart3D('Body', Boxel.empty, bodyColor, useGPU, useParallel);

            b = Boxel([params.length/2 0 params.height/2], ...
                [params.length params.width params.height], bodyColor);
            obj.Body.addBoxel(b);
            obj.Tires = Tire3D.empty;
            axlePositions = linspace(-params.length/2 + params.axleSpacing, ...
                                     params.length/2 - params.axleSpacing, ...
                                     params.numAxles);
            for i = 1:numel(axlePositions)
                pos = axlePositions(i);
                tL = Tire3D(params.wheelHeight/2, params.wheelWidth, [0 0 0], 12, useGPU, useParallel);
                tL.Position = [pos params.trackWidth/2 params.wheelHeight/2];
                obj.Tires(end+1) = tL;
                tR = Tire3D(params.wheelHeight/2, params.wheelWidth, [0 0 0], 12, useGPU, useParallel);
                tR.Position = [pos -params.trackWidth/2 params.wheelHeight/2];
                obj.Tires(end+1) = tR;
            end
        end

        function setState(obj, x, y, theta)
            % setState Updates the vehicle pose in the world.
            obj.Body.Position = [x y 0];
            obj.Body.setOrientation(theta, 0, 0);

            R = obj.Body.Orientation;
            for i = 1:numel(obj.Tires)
                off = obj.Tires(i).Position;
                obj.Tires(i).Position = [x y 0] + (R * off.').';
                obj.Tires(i).setOrientation(theta, 0, 0);
            end
        end

        function drawMesh(obj, ax)
            if nargin < 2, ax = gca; end
            obj.Body.smoothSurface(ax);
            for i = 1:numel(obj.Tires)
                obj.Tires(i).smoothSurface(ax);
            end
        end
    end
end
