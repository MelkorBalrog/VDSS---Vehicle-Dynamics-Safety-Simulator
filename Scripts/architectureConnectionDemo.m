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
    action = drawrectangle(ax, 'Position', [150 200 80 40], 'Color', 'g');

    elements = [struct('shape',blockA,'type','block');
                struct('shape',blockB,'type','block');
                struct('shape',action,'type','action')];

    connectorTypes = {'flow','fork','join','merge'};
    selectedType = 'flow';
    typeDrop = uidropdown(f, 'Items', connectorTypes, 'Value', selectedType,
        'Position', [10 260 80 22], 'ValueChangedFcn', @(dd,ev)changeType(dd.Value));

    lineObj = drawline(ax, 'Position', [rectCenter(blockA.Position); rectCenter(blockB.Position)]);
    shapeObj = gobjects(0);
    lastValidPos = lineObj.Position;
    updateConnection();

    addlistener(blockA, 'MovingROI', @(s,e)updateConnection());
    addlistener(blockB, 'MovingROI', @(s,e)updateConnection());
    addlistener(blockA, 'ROIMoved', @(s,e)validateConnection());
    addlistener(blockB, 'ROIMoved', @(s,e)validateConnection());
    addlistener(lineObj, 'MovingROI', @(s,e)previewConnection());
    addlistener(lineObj, 'ROIMoved', @(s,e)validateConnection());

    function constrainLine()
        lineObj.Position(1,:) = intersectRect(blockA.Position, lineObj.Position(2,:));
        lineObj.Position(2,:) = intersectRect(blockB.Position, lineObj.Position(1,:));
    end

    function previewConnection()
        constrainLine();
        p1 = intersectRect(blockA.Position, lineObj.Position(2,:));
        p2 = intersectRect(blockB.Position, lineObj.Position(1,:));
        srcType = elementAtPoint(p1);
        dstType = elementAtPoint(p2);
        [valid,~] = isValidSysML(srcType, dstType, selectedType, p1, p2);
        if valid
            lastValidPos = [p1; p2];
        else
            lineObj.Position = lastValidPos;
        end
        updateConnectorShape();
    end

    function updateConnection()
        p1 = intersectRect(blockA.Position, lineObj.Position(2,:));
        p2 = intersectRect(blockB.Position, lineObj.Position(1,:));
        lineObj.Position = [p1; p2];
        updateConnectorShape();
    end

    function changeType(newType)
        selectedType = newType;
        updateConnectorShape();
    end

    function validateConnection()
        p1 = intersectRect(blockA.Position, lineObj.Position(2,:));
        p2 = intersectRect(blockB.Position, lineObj.Position(1,:));
        srcType = elementAtPoint(p1);
        dstType = elementAtPoint(p2);
        [valid,msg] = isValidSysML(srcType,dstType,selectedType,p1,p2);
        if valid
            lastValidPos = [p1; p2];
        else
            uialert(f, msg, 'Invalid Connection');
            lineObj.Position = lastValidPos;
        end
        updateConnectorShape();
    end

    function tf = isOnRight(rect, p)
        tf = abs(p(1) - (rect(1) + rect(3))) < 1e-3;
    end

    function tf = isOnLeft(rect, p)
        tf = abs(p(1) - rect(1)) < 1e-3;
    end

    function updateConnectorShape()
        if isgraphics(shapeObj)
            delete(shapeObj)
        end
        p1 = lineObj.Position(1,:);
        p2 = lineObj.Position(2,:);
        mid = (p1 + p2)/2;
        theta = atan2(p2(2)-p1(2), p2(1)-p1(1));
        switch selectedType
            case 'fork'
                bar = drawBar(ax, mid, theta);
                arr = drawArrow(ax, mid + rotVec([6 0], theta), theta);
                shapeObj = [bar, arr];
            case 'join'
                bar = drawBar(ax, mid, theta);
                arr = drawArrow(ax, mid - rotVec([6 0], theta), theta);
                shapeObj = [arr, bar];
            case 'merge'
                dia = drawDiamond(ax, mid, theta);
                arr = drawArrow(ax, mid + rotVec([6 0], theta), theta);
                shapeObj = [dia, arr];
            otherwise
                shapeObj = drawArrow(ax, mid, theta);
        end
    end

    function h = drawArrow(axc, pos, theta)
        sz = 6;
        p1 = pos + sz * [cos(theta) sin(theta)];
        p2 = pos + sz/2 * [cos(theta+2*pi/3) sin(theta+2*pi/3)];
        p3 = pos + sz/2 * [cos(theta-2*pi/3) sin(theta-2*pi/3)];
        h = patch(axc, [p1(1) p2(1) p3(1)], [p1(2) p2(2) p3(2)], 'k', 'FaceColor', 'k');
    end

    function h = drawBar(axc, pos, theta)
        L = 8; W = 2;
        v1 = rotVec([0.5*L 0.5*W], theta);
        v2 = rotVec([0.5*L -0.5*W], theta);
        v3 = rotVec([-0.5*L -0.5*W], theta);
        v4 = rotVec([-0.5*L 0.5*W], theta);
        verts = pos + [v1; v2; v3; v4];
        h = patch(axc, verts(:,1), verts(:,2), 'k', 'FaceColor', 'k');
    end

    function h = drawDiamond(axc, pos, theta)
        L = 8; W = 6;
        v1 = rotVec([0 L/2], theta);
        v2 = rotVec([W/2 0], theta);
        v3 = rotVec([0 -L/2], theta);
        v4 = rotVec([-W/2 0], theta);
        verts = pos + [v1; v2; v3; v4];
        h = patch(axc, verts(:,1), verts(:,2), 'w', 'EdgeColor', 'k');
    end

    function v = rotVec(vec, ang)
        v = [vec(1)*cos(ang)-vec(2)*sin(ang), vec(1)*sin(ang)+vec(2)*cos(ang)];
    end

    function type = elementAtPoint(p)
        for k = 1:numel(elements)
            if isInside(elements(k).shape.Position, p)
                type = elements(k).type;
                return;
            end
        end
        type = 'none';
    end

    function tf = isInside(rect, pt)
        tf = pt(1) >= rect(1) && pt(1) <= rect(1)+rect(3) && ...
             pt(2) >= rect(2) && pt(2) <= rect(2)+rect(4);
    end

    function [valid,msg] = isValidSysML(srcType,dstType,connType,p1,p2)
        if strcmp(srcType,'action') || strcmp(dstType,'action')
            valid = false;
            msg = 'Connections to actions are not allowed in this demo';
            return;
        end
        switch connType
            case {'flow','fork','join','merge'}
                valid = strcmp(srcType,'block') && strcmp(dstType,'block') && ...
                        isOnRight(blockA.Position,p1) && isOnLeft(blockB.Position,p2);
                msg = 'Connections must run from block A output to block B input';
            otherwise
                valid = false;
                msg = 'Unknown connector type';
        end
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

