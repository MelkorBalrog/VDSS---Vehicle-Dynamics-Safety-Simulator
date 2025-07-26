% Author: Miguel Marina <karel.capek.robotics@gmail.com>
%{
@file Boxel.m
@brief Represents a simple 3D box element used to build graphics objects.
%}

classdef Boxel
    properties
        Position % [x y z] position of the box center
        Size     % [length width height]
        Color    % [r g b] color values between 0 and 1
        UseGPU = false % Flag to compute vertices on the GPU
    end
    methods
        function obj = Boxel(position, sz, color)
            if nargin < 1
                position = [0 0 0];
            end
            if nargin < 2
                sz = [1 1 1];
            end
            if nargin < 3
                color = [0.5 0.5 0.5];
            end
            obj.Position = position;
            obj.Size = sz;
            obj.Color = color;
        end

        function h = draw(obj, ax)
            % draw Renders the boxel as a patch object in the provided axes.
            %
            %   h = draw(obj, ax) draws the boxel using the axes handle ax and
            %   returns the patch handle h.
            if nargin < 2 || isempty(ax)
                ax = gca;
            end

            verts = obj.vertices();

            if obj.UseGPU && gpuDeviceCount > 0
                verts = gpuArray(verts); %#ok<GPUARRAY>
                verts = gather(verts);
            end

            h = patch(ax, 'Vertices', verts, ...
                'Faces', obj.faces(), ...
                'FaceColor', obj.Color, 'EdgeColor', 'none');
        end

        function verts = vertices(obj)
            % vertices Returns an 8x3 matrix of cuboid vertices in world coordinates.
            [X, Y, Z] = obj.cuboidData();
            verts = [X(:) Y(:) Z(:)];
        end

        function verts = localVertices(obj)
            % localVertices Returns vertices relative to the object origin.
            c = obj.Position - obj.Size/2;
            X = [0 1 1 0 0 1 1 0]*obj.Size(1) + c(1);
            Y = [0 0 1 1 0 0 1 1]*obj.Size(2) + c(2);
            Z = [0 0 0 0 1 1 1 1]*obj.Size(3) + c(3);
            if obj.UseGPU && gpuDeviceCount > 0
                X = gpuArray(X); Y = gpuArray(Y); Z = gpuArray(Z); %#ok<GPUARRAY>
                X = gather(X); Y = gather(Y); Z = gather(Z);
            end
            verts = [X(:) Y(:) Z(:)];
        end

        function F = faces(obj)
            % faces Returns the face indices for the cuboid.
            F = obj.cuboidFaces();
        end
    end

    methods (Access = private)
        function [X, Y, Z] = cuboidData(obj)
            % cuboidData Generates vertices for the cuboid.
            c = obj.Position - obj.Size/2;
            X = [0 1 1 0 0 1 1 0]*obj.Size(1) + c(1);
            Y = [0 0 1 1 0 0 1 1]*obj.Size(2) + c(2);
            Z = [0 0 0 0 1 1 1 1]*obj.Size(3) + c(3);
            if obj.UseGPU && gpuDeviceCount > 0
                X = gpuArray(X); Y = gpuArray(Y); Z = gpuArray(Z); %#ok<GPUARRAY>
                X = gather(X); Y = gather(Y); Z = gather(Z);
            end
        end

        function F = cuboidFaces(~)
            % cuboidFaces Returns face indices for a cuboid.
            F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
        end
    end
end
