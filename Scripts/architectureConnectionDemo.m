%--------------------------------------------------------------------------
% This file is part of VDSS - Vehicle Dynamics Safety Simulator.
%
% VDSS is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% VDSS is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <https://www.gnu.org/licenses/>.
%--------------------------------------------------------------------------
%{
% @file architectureConnectionDemo.m
% @brief Interactive demo for drawing connections between blocks.
%        Endpoints remain attached to block surfaces and can be
%        moved along the edges.
%}
function architectureConnectionDemo
    f = uifigure('Name','Architecture Diagram Demo');
    ax = uiaxes('Parent', f);
    hold(ax, 'on');
    axis(ax, [0 400 0 300]);
    axis(ax, 'equal');

    blockA = drawrectangle(ax, 'Position', [50 120 100 60], 'Color', 'b');
    blockB = drawrectangle(ax, 'Position', [250 80 100 60], 'Color', 'r');

    lineObj = drawline(ax, 'Position', [rectCenter(blockA.Position); rectCenter(blockB.Position)]);
    updateConnection();

    addlistener(blockA, 'MovingROI', @(s,e)updateConnection());
    addlistener(blockB, 'MovingROI', @(s,e)updateConnection());
    addlistener(lineObj, 'MovingROI', @(s,e)constrainLine());

    function constrainLine()
        lineObj.Position(1,:) = intersectRect(blockA.Position, lineObj.Position(2,:));
        lineObj.Position(2,:) = intersectRect(blockB.Position, lineObj.Position(1,:));
    end

    function updateConnection()
        p1 = intersectRect(blockA.Position, lineObj.Position(2,:));
        p2 = intersectRect(blockB.Position, lineObj.Position(1,:));
        lineObj.Position = [p1; p2];
    end
end

function c = rectCenter(pos)
    c = [pos(1) + pos(3)/2, pos(2) + pos(4)/2];
end

function p = intersectRect(rectPos, target)
    center = rectCenter(rectPos);
    p = computeRectIntersection(center, target, rectPos);
end

function p = computeRectIntersection(center, target, rectPos)
    x1 = rectPos(1); y1 = rectPos(2); w = rectPos(3); h = rectPos(4);
    left = x1; right = x1 + w; bottom = y1; top = y1 + h;

    dx = target(1) - center(1);
    dy = target(2) - center(2);

    if abs(dx) < eps
        if dy >= 0
            p = [center(1), top];
        else
            p = [center(1), bottom];
        end
        return;
    end

    slope = dy / dx;

    if dx > 0
        x = right;
    else
        x = left;
    end
    y = center(2) + slope * (x - center(1));
    if y >= bottom && y <= top
        p = [x, y];
        return;
    end

    if dy > 0
        y = top;
    else
        y = bottom;
    end
    x = center(1) + (y - center(2)) / slope;
    p = [x, y];
end
