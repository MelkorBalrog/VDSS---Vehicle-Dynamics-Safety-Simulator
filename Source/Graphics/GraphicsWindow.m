% Author: Miguel Marina <karel.capek.robotics@gmail.com>
%{
@file GraphicsWindow.m
@brief Provides a window for rendering a World3D scene and running animations.
%}

classdef GraphicsWindow < handle
    properties
        World % World3D instance to render
        Figure
        Axes
        UseGPU = false
        UseParallel = false
    end

    methods
        function obj = GraphicsWindow(world, useGPU, useParallel)
            if nargin < 1 || isempty(world)
                obj.World = World3D();
            else
                obj.World = world;
            end
            if nargin < 2
                useGPU = false;
            end

            if nargin < 3
                useParallel = false;
            end
            obj.UseGPU = useGPU;
            obj.UseParallel = useParallel;
            obj.World.UseGPU = useGPU;
            obj.World.UseParallel = useParallel;
            obj.Figure = figure('Name','VDSS 3D View');
            obj.Axes = axes('Parent', obj.Figure);
            axis(obj.Axes,'equal');
            view(obj.Axes,3);
            grid(obj.Axes,'on');
            xlabel(obj.Axes,'X'); ylabel(obj.Axes,'Y'); zlabel(obj.Axes,'Z');
            rotate3d(obj.Figure,'on');
            set(obj.Figure,'WindowScrollWheelFcn',@(src,evt)obj.scrollZoom(evt));
        end

        function render(obj)
            cla(obj.Axes);
            % Ensure axes limits update to the newly drawn content
            axis(obj.Axes,'auto');

            if ~isempty(obj.World)
                obj.World.UseGPU = obj.UseGPU;
                obj.World.UseParallel = obj.UseParallel;
                obj.World.draw(obj.Axes);
            end
            drawnow;
        end

        function animate(obj, updateFcn, steps, dt)
            if nargin < 4 || isempty(dt)
                dt = 0.05;
            end
            if nargin < 3 || isempty(steps)
                steps = 1;
            end
            for k = 1:steps
                if nargin >= 2 && ~isempty(updateFcn)
                    updateFcn(k);
                end
                obj.render();
                pause(dt);
            end
        end

        function zoomIn(obj)
            camzoom(obj.Axes, 1.2);
        end

        function zoomOut(obj)
            camzoom(obj.Axes, 0.8);
        end

        function scrollZoom(obj, evt)
            if evt.VerticalScrollCount > 0
                camzoom(obj.Axes, 1.1);
            else
                camzoom(obj.Axes, 0.9);
            end
        end
    end
end
