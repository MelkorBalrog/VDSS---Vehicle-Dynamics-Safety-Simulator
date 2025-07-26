% Author: Miguel Marina <karel.capek.robotics@gmail.com>
%{
@file VehiclePart3D.m
@brief Base class for vehicle parts composed of Boxel objects.
%}

classdef VehiclePart3D < Object3D
    properties
        Name
        Color = [0.5 0.5 0.5]
    end
    methods
        function obj = VehiclePart3D(name, boxels, color, useGPU, useParallel)
            if nargin < 1
                name = '';
            end
            if nargin < 2
                boxels = Boxel.empty;
            end
            if nargin < 3 || isempty(color)
                color = [0.5 0.5 0.5];
            end
            if nargin < 4
                useGPU = false;
            end
            if nargin < 5
                useParallel = false;
            end
            obj@Object3D(boxels, useGPU, useParallel);
            obj.Name = name;
            obj.Color = color;
            obj.applyColor();
        end

        function setColor(obj, color)
            % setColor Updates the color of the part and its boxels.
            obj.Color = color;
            obj.applyColor();
        end
    end

    methods (Access = private)
        function applyColor(obj)
            for i = 1:numel(obj.Boxels)
                obj.Boxels(i).Color = obj.Color;
            end
        end
    end
end
