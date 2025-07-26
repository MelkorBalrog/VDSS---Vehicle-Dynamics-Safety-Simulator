% Author: Miguel Marina <karel.capek.robotics@gmail.com>
%{
@file Trailer3D.m
@brief Simple trailer body built from Boxels.
%}

classdef Trailer3D < VehiclePart3D
    methods
        function obj = Trailer3D(length, width, height, color, useGPU, useParallel)
            if nargin < 4
                color = [0 0 1];
            end
            if nargin < 3
                height = 2.5;
            end
            if nargin < 2
                width = 1.0;
            end
            if nargin < 1
                length = 6.0;
            end
            if nargin < 5
                useGPU = false;
            end
            if nargin < 6
                useParallel = false;
            end

            obj@VehiclePart3D('Trailer', Boxel.empty, color, useGPU, useParallel);

            b = Boxel([length/2 0 height/2], [length width height], color);
            obj.addBoxel(b);
        end
    end
end
