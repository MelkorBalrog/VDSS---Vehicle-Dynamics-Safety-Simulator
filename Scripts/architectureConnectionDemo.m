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
    initialNode = drawcircle(ax, 'Center', [80 260], 'Radius', 8, 'Color', 'k');
    finalOuter = drawcircle(ax, 'Center', [320 260], 'Radius', 8, 'Color', 'k');
    finalInner = drawcircle(ax, 'Center', [320 260], 'Radius', 4, 'Color', 'k', ...
        'FaceColor', 'k');

    elements = [struct('shape',blockA,'type','block');
                struct('shape',blockB,'type','block');
                struct('shape',action,'type','action');
                struct('shape',initialNode,'type','initial');
                struct('shape',finalOuter,'type','final')];

    % keep inner circle in sync with outer
    addlistener(finalOuter,'MovingROI',@(s,e)moveInner());
    addlistener(finalOuter,'ROIMoved',@(s,e)moveInner());

    function moveInner()
        finalInner.Center = finalOuter.Center;
    end

    connectorTypes = {'flow','fork','join','merge'};
    selectedType = 'flow';
    typeDrop = uidropdown(f, 'Items', connectorTypes, 'Value', selectedType,
        'Position', [10 260 80 22], 'ValueChangedFcn', @(dd,ev)changeType(dd.Value));

    lineObj = drawline(ax, 'Position', [rectCenter(blockA.Position); rectCenter(blockB.Position)]);
    shapeObj = gobjects(0);
    lastValidPos = lineObj.Position;
    invalidShown = false;
    updateConnectorShape();

    addlistener(lineObj, 'MovingROI', @(s,e)previewConnection());
    addlistener(lineObj, 'ROIMoved', @(s,e)validateConnection());

    function previewConnection()
        p1 = lineObj.Position(1,:);
        p2 = lineObj.Position(2,:);
        srcType = elementAtPoint(p1);
        dstType = elementAtPoint(p2);
        [valid,msg] = isValidSysML(srcType, dstType, selectedType, p1, p2);
        if valid
            lastValidPos = [p1; p2];
            invalidShown = false;
        else
            if ~invalidShown
                uialert(f, msg, 'Invalid Connection');
                invalidShown = true;
            end
            lineObj.Position = lastValidPos;
        end
        updateConnectorShape();
    end

    function changeType(newType)
        selectedType = newType;
        updateConnectorShape();
    end

    function validateConnection()
        p1 = lineObj.Position(1,:);
        p2 = lineObj.Position(2,:);
        srcType = elementAtPoint(p1);
        dstType = elementAtPoint(p2);
        [valid,msg] = isValidSysML(srcType,dstType,selectedType,p1,p2);
        if valid
            lastValidPos = [p1; p2];
        else
            uialert(f, msg, 'Invalid Connection');
            lineObj.Position = lastValidPos;
        end
        invalidShown = false;
        updateConnectorShape();
    end

    function tf = isOnRight(rect, p)
        tf = p(1) >= rect(1) + rect(3) - 1;
    end

    function tf = isOnLeft(rect, p)
        tf = p(1) <= rect(1) + 1;
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
            if isInside(elements(k).shape, p)
                type = elements(k).type;
                return;
            end
        end
        type = 'none';
    end

    function tf = isInside(shape, pt)
        if isa(shape,'images.roi.Rectangle')
            pos = shape.Position;
            tf = pt(1) >= pos(1) && pt(1) <= pos(1)+pos(3) && ...
                 pt(2) >= pos(2) && pt(2) <= pos(2)+pos(4);
        elseif isa(shape,'images.roi.Circle')
            c = shape.Center; r = shape.Radius;
            tf = hypot(pt(1)-c(1), pt(2)-c(2)) <= r;
        else
            tf = false;
        end
    end

    function [valid,msg] = isValidSysML(srcType,dstType,connType,p1,p2)
        % Basic set of SysML 2.0 rules for demo purposes
        if strcmp(srcType,'final')
            if strcmp(dstType,'initial')
                msg = 'Cannot connect Final node to Initial node';
            else
                msg = 'Final nodes have no outgoing flows';
            end
            valid = false;
            return;
        elseif strcmp(dstType,'final') && ~strcmp(srcType,'initial')
            valid = false;
            msg = 'Final nodes cannot have incoming flows';
            return;
        end
        if strcmp(dstType,'initial')
            valid = false; msg = 'Initial nodes have no incoming flows'; return;
        end

        % special case for block architecture connections
        if strcmp(srcType,'block') || strcmp(dstType,'block')
            if ~strcmp(srcType,'block') || ~strcmp(dstType,'block')
                valid = false;
                msg = 'Blocks may only connect to other blocks';
                return;
            end
            if ~isOnRight(blockA.Position,p1) || ~isOnLeft(blockB.Position,p2)
                valid = false;
                msg = 'Connection must go from block A output to block B input';
                return;
            end
            valid = true; msg = ''; return;
        end

        % Activity diagram rules
        switch srcType
            case 'initial'
                valid = any(strcmp(dstType,{ 'action','decision','merge','fork','join'}));
            case 'action'
                valid = any(strcmp(dstType,{ 'action','decision','merge','fork','join','final'}));
            case 'decision'
                valid = any(strcmp(dstType,{ 'action','decision','merge','fork','join','final'}));
            case 'merge'
                valid = any(strcmp(dstType,{ 'action','decision','fork','join','final'}));
            case 'fork'
                valid = any(strcmp(dstType,{ 'action','decision','merge','fork','join','final'}));
            case 'join'
                valid = any(strcmp(dstType,{ 'action','decision','merge','fork','final'}));
            otherwise
                valid = false;
        end
        if valid
            msg = '';
        else
            msg = sprintf('Invalid connection from %s to %s', srcType, dstType);
        end
    end
end

function c = rectCenter(pos)
    c = [pos(1) + pos(3)/2, pos(2) + pos(4)/2];
end

