% Author: Miguel Marina <karel.capek.robotics@gmail.com>
%{
@file Object3D.m
@brief Combines multiple Boxel objects into a single drawable graphic.
%}

classdef Object3D
    properties
        Boxels = Boxel.empty
        Position = [0 0 0]
        Orientation = eye(3)
        UseGPU = false
        UseParallel = false
    end

    methods
        function obj = Object3D(boxels, useGPU, useParallel)
            if nargin < 1
                boxels = Boxel.empty;
            end
            if nargin < 2
                useGPU = false;
            end
            if nargin < 3
                useParallel = false;
            end
            obj.Boxels = boxels;
            obj.UseGPU = useGPU;
            obj.UseParallel = useParallel;
        end

        function verts = localVertices(obj)
            % localVertices Returns all transformed vertices of the object.
            %   This utility exposes the vertex collection used internally
            %   for mesh generation so that external code can query the
            %   object geometry. Vertices are returned in world
            %   coordinates accounting for the object's position and
            %   orientation.

            verts = obj.collectMesh();
        end

        function F = faces(obj)
            % faces Returns concatenated face indices for all boxels.
            %   This utility mirrors the localVertices method and exposes the
            %   triangulation connectivity so that higher level objects can
            %   generate patch meshes without querying each boxel directly.

            n = numel(obj.Boxels);
            F = zeros(0,4);
            offset = 0;
            for i = 1:n
                f = obj.Boxels(i).faces();
                F = [F; f + offset]; %#ok<AGROW>
                offset = offset + size(obj.Boxels(i).localVertices(),1);
            end
        end

        function addBoxel(obj, boxel)
            if obj.UseGPU
                boxel.UseGPU = true;
            end
            obj.Boxels(end+1) = boxel;
        end

        function setOrientation(obj, yaw, pitch, roll)
            % setOrientation Sets the object orientation.
            %   Accepts yaw, pitch, roll angles (rad) or a 3x3 rotation
            %   matrix when supplied as a single argument. Single scalar
            %   input is treated as a yaw angle. Any other input sizes are
            %   rejected to avoid invalid states that can lead to matrix
            %   dimension errors during rendering.

            if nargin == 2
                if isscalar(yaw)
                    pitch = 0;
                    roll = 0;
                elseif ismatrix(yaw)
                    if all(size(yaw) == [3 3])
                        obj.Orientation = yaw;
                        return;
                    else
                        error('Object3D:InvalidOrientation', ...
                            'Orientation matrix must be 3x3.');
                    end
                else
                    error('Object3D:InvalidOrientation', ...
                        'Invalid orientation input.');
                end
            end

            if nargin < 4, roll = 0; end
            if nargin < 3, pitch = 0; end
            if nargin < 2, yaw = 0; end
            Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
            Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
            Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
            obj.Orientation = Rz*Ry*Rx;
        end

        function h = draw(obj, ax)
            % draw Renders the composed object by drawing each boxel.
            if nargin < 2 || isempty(ax)
                ax = gca;
            end
            holdState = ishold(ax);
            hold(ax, 'on');
            n = numel(obj.Boxels);
            vertsCell = cell(1,n);
            facesCell = cell(1,n);
            colorCell = cell(1,n);
            boxels = obj.Boxels;
            R = obj.Orientation;
            pos = obj.Position;
            useGPU = obj.UseGPU && gpuDeviceCount > 0;
            if obj.UseParallel && ~isempty(gcp('nocreate'))
                parfor i = 1:n
                    b = boxels(i);
                    v = b.localVertices();
                    if useGPU
                        v = gpuArray(v); %#ok<GPUARRAY>
                        Rg = gpuArray(R); %#ok<GPUARRAY>
                        v = (Rg * v.').';
                        v = v + gpuArray(pos);
                        v = gather(v);
                    else
                        v = (R * v.').';
                        v = v + pos;
                    end
                    vertsCell{i} = v;
                    facesCell{i} = b.faces();
                    colorCell{i} = b.Color;
                end
            else
                for i = 1:n
                    b = boxels(i);
                    v = b.localVertices();
                    if useGPU
                        v = gpuArray(v); %#ok<GPUARRAY>
                        Rg = gpuArray(R); %#ok<GPUARRAY>
                        v = (Rg * v.').';
                        v = v + gpuArray(pos);
                        v = gather(v);
                    else
                        v = (R * v.').';
                        v = v + pos;
                    end
                    vertsCell{i} = v;
                    facesCell{i} = b.faces();
                    colorCell{i} = b.Color;
                end
            end

            for i = 1:n
                patch(ax, 'Vertices', vertsCell{i}, 'Faces', facesCell{i}, ...
                    'FaceColor', colorCell{i}, 'EdgeColor', 'none');
            end
            if ~holdState
                hold(ax, 'off');
            end
            h = []; % return handle array in future
        end

        function h = smoothSurface(obj, ax, alpha)
            % smoothSurface Draws a smoothed version of the composed object.
            %   This function computes an alpha shape of all vertices
            %   from the contained boxels to produce a smoother looking
            %   surface. A patch handle is returned.

            if nargin < 2 || isempty(ax)
                ax = gca;
            end
            if nargin < 3
                alpha = 1.5;
            end

            [facesS, vertsS] = obj.getSmoothedMesh(alpha);
            h = patch(ax, 'Vertices', vertsS, 'Faces', facesS, ...
                'FaceColor', 'interp', 'EdgeColor', 'none');
        end

        function [facesS, vertsS] = getSmoothedMesh(obj, alpha)
            % getSmoothedMesh Computes a smoothed mesh from all boxel vertices
            if nargin < 2
                alpha = 1.5;
            end
            verts = obj.collectMesh();
            shp = alphaShape(verts, alpha);
            [facesS, vertsS] = boundaryFacets(shp);
        end
    end

    methods (Access = private)
        function verts = collectMesh(obj)
            % collectMesh Collects transformed vertices from all boxels
            n = numel(obj.Boxels);
            if n == 0
                verts = zeros(0,3);
                return;
            end

            vertsCell = cell(1,n);
            R = obj.Orientation;
            pos = obj.Position;
            if obj.UseParallel && ~isempty(gcp('nocreate'))
                parfor i = 1:n
                    b = obj.Boxels(i);
                    v = b.localVertices();
                    v = (R * v.').';
                    v = v + pos;
                    vertsCell{i} = v; %#ok<PFOUS>
                end
            else
                for i = 1:n
                    b = obj.Boxels(i);
                    v = b.localVertices();
                    v = (R * v.').';
                    v = v + pos;
                    vertsCell{i} = v;
                end
            end
            verts = vertcat(vertsCell{:});
        end
    end
end
