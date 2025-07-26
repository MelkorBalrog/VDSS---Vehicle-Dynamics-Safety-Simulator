% Author: Miguel Marina <karel.capek.robotics@gmail.com>
%{
@file Road3D.m
@brief Simple road segment constructed from a single Boxel.
%}

classdef Road3D < Object3D
    methods
        function obj = Road3D(startPos, len, width, color)
            if nargin < 4
                color = [0.2 0.2 0.2];
            end

            obj@Object3D(Boxel.empty, false, false);
            b = Boxel([len/2 0 0], [len width 0.1], color);
            obj.addBoxel(b);
            obj.Position = startPos;
        end
    end
end
