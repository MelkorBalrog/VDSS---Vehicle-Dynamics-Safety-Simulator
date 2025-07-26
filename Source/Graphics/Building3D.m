% Author: Miguel Marina <karel.capek.robotics@gmail.com>
%{
@file Building3D.m
@brief Simple example object representing a building constructed from Boxels.
%}

classdef Building3D < Object3D
    methods
        function obj = Building3D(position, width, depth, height, color)
            if nargin < 5
                color = [0.8 0.8 0.8];
            end
            obj@Object3D(Boxel.empty, false, false);
            nFloors = max(1, round(height));
            for f = 1:nFloors
                b = Boxel([0 0 (f-0.5)], [width depth 1], color);
                obj.addBoxel(b);
            end
            obj.Position = position;
        end
    end
end
