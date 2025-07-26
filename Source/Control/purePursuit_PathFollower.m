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
% @file purePursuit_PathFollower.m
% @brief Generates adaptive waypoints for pure pursuit path following.
%        Commands steering based on radius of curvature.
% @author Miguel Marina <karel.capek.robotics@gmail.com>
%}

classdef purePursuit_PathFollower
    % purePursuit_PathFollower - An advanced pure pursuit path follower that integrates
    % trajectory predictions to generate adaptive waypoints for smoother and
    % more accurate path following. It plans steering angles based on the
    % radius of curvature of the planned waypoints and commands gear shifts
    % during curvature for improved vehicle dynamics.
    %
    % Changes:
    % - Integrated with Transmission to command gear shifts during curvature.
    % - Ensures gear shifts down by one gear per curve and maintains the lower gear until the curve is completed.
    % - Implements controlled speed reduction upon shifting down.
    % - **Replaced Simple Moving Average (SMA) Filter with Gaussian Filter for Steering Angle Smoothing.**
    %
    % Author: Miguel Marina
    % Date: 2024-12-13

    properties
        % Path Parameters
        waypoints                % Nx2 matrix of [x,y] planned waypoints
        wheelbase                % Wheelbase of the vehicle (m)
        lookaheadDistance        % Lookahead distance for Pure Pursuit (m)
        maxSteeringAngle         % Maximum steering angle (degrees)

        % Offset Parameters
        offsetX                  % Global offset in x-direction (m)
        offsetY                  % Global offset in y-direction (m)
        R

        % Vehicle State
        position                 % Current vehicle position [x, y] (m)
        orientation              % Current vehicle heading (rad)
        speed                    % Current vehicle speed (m/s)
        currentWaypointIndex     % Index of the current target waypoint
        waypointThreshold        % Distance threshold to consider waypoint reached

        % Steering Smoothing
        alpha                    % Smoothing factor for steering angle (0 < alpha < 1)
        steeringAngleSmoothed    % Last smoothed steering angle (degrees)

        % Prediction Parameters
        predictionTime           % Time horizon to predict forward (s)
        lastSteeringAngle        % Store last computed steering angle to predict orientation change

        % Time Step and Rate Limiting
        timeStep                 % Time step between control updates (s)
        maxSteeringRate          % Maximum steering rate (degrees per second)
        steeringStep             % Steering angle increment per update (degrees)
        targetSteeringAngle      % Desired steering angle to reach

        % Additional Filtering
        steeringFilterFactor     % Additional low-pass filter factor for steering angle
        prevFilteredSteeringAngle% Store the last filtered steering angle

        % Trajectory Prediction Integration
        predictedTrajectory      % Px2 matrix of [x,y] predicted future positions
        numPredictions           % Number of trajectory predictions to consider (e.g., 10)
        generatedWaypoints       % Mx2 matrix of generated waypoints based on predictions

        % Planned Steering Angles
        plannedSteeringAngles    % Queue of planned steering angles (degrees)
        planningHorizon           % Number of steering angles to plan ahead

        % Averaging Window Properties
        bufferHorizon            % Time horizon to average steering angles (s)
        numBufferAngles          % Number of steering angles corresponding to bufferHorizon

        % Curvature Parameters
        curvatures               % Array of curvatures for upcoming waypoints
        radiusOfCurvature        % Array of radius of curvature for upcoming waypoints

        % *** New Properties for Gaussian Filter ***
        gaussianBufferSize        % Number of recent steering angles for Gaussian filter
        gaussianWeights           % Gaussian kernel weights (column vector [N x 1])
        steeringAngleGaussianHistory % Buffer to store recent steering angles for Gaussian filter (column vector [N x 1])
        gaussianSigma             % Standard deviation for Gaussian weights
        % *** End of New Properties ***

        % *** New Properties for Gear Shifting ***
        transmission               % Reference to the Transmission object
        curvatureShiftThreshold    % Threshold curvature to trigger gear shift (1/m)
        shiftedDownForCurrentCurve % Flag to prevent multiple shiftDown commands during the same curve
        % *** End of New Properties ***

        steeringAngleHistory
        historyBufferSize

        % Low-Pass Filter Properties
        lowPassAlpha             % Low-pass filter coefficient (e.g., 0.1 for strong smoothing)
        prevLowPassOutput        % Previous output of the low-pass filter
    end

    methods
        %% Constructor
        function obj = purePursuit_PathFollower(waypoints, wheelbase, lookaheadDistance, maxSteeringAngle, ...
                                     alpha, predictionTime, numPredictions, bufferHorizon, ...
                                     timeStep, R, offsetX, offsetY, historyBufferSize, ...
                                     transmission, curvatureShiftThreshold, gaussianBufferSize, gaussianSigma)
            % Initialize purePursuit_PathFollower with required parameters.
            %
            % Parameters:
            % - waypoints: Nx2 matrix of [x, y] planned waypoints
            % - wheelbase: Wheelbase of the vehicle (m)
            % - lookaheadDistance: Lookahead distance for Pure Pursuit (m)
            % - maxSteeringAngle: Maximum steering angle (degrees)
            % - alpha: Smoothing factor for steering angle (0 < alpha < 1)
            % - predictionTime: Time horizon to predict forward (s)
            % - numPredictions: Number of trajectory predictions to consider (e.g., 10)
            % - bufferHorizon: Time window for averaging steering angles (s)
            % - timeStep: Time step between control updates (s)
            % - offsetX: Global offset in x-direction (m)
            % - offsetY: Global offset in y-direction (m)
            % - historyBufferSize: Number of steering angles to average for smoothing
            % - transmission: Instance of Transmission class
            % - curvatureShiftThreshold: Curvature threshold to trigger gear shift (1/m)
            % - gaussianBufferSize: Number of steering angles for Gaussian filter (optional, default = 5)
            % - gaussianSigma: Standard deviation for Gaussian weights (optional, default = 1)
            %
            % Default Values:
            % - alpha = 0.05
            % - predictionTime = 0.5
            % - numPredictions = 10
            % - bufferHorizon = 5.0
            % - timeStep = 0.1
            % - offsetX = 0.0
            % - offsetY = 0.0
            % - historyBufferSize = 5
            % - curvatureShiftThreshold = 0.1 (corresponding to radius of 10 meters)
            % - gaussianBufferSize = 5 (default)
            % - gaussianSigma = 1 (default)

            if nargin < 4
                error('Please provide waypoints, wheelbase, lookaheadDistance, and maxSteeringAngle.');
            end
            if nargin < 5 || isempty(alpha)
                alpha = 0.05; % Further reduce alpha to smooth steering
            end
            if nargin < 6 || isempty(predictionTime)
                predictionTime = 0.5; % Default prediction time
            end
            if nargin < 7 || isempty(numPredictions)
                numPredictions = 10; % Default number of predictions to integrate
            end
            if nargin < 8 || isempty(bufferHorizon)
                bufferHorizon = 5.0; % Default buffer horizon (5 seconds)
            end
            if nargin < 9 || isempty(timeStep)
                timeStep = 0.1; % Default time step (s)
            end
            if nargin < 10 || isempty (R)
                R = [0,0,0,0];
            end
            if nargin < 11 || isempty(offsetX)
                offsetX = 0.0; % Default no offset
            end
            if nargin < 12 || isempty(offsetY)
                offsetY = 0.0; % Default no offset
            end
            if nargin < 13 || isempty(historyBufferSize)
                historyBufferSize = 5; % Default history buffer size for steering angle averaging
            end
            if nargin < 14 || isempty(transmission)
                error('Transmission object must be provided.');
            end
            if nargin < 15 || isempty(curvatureShiftThreshold)
                curvatureShiftThreshold = 0.1; % Default threshold (1/m, radius = 10 meters)
            end
            if nargin < 16 || isempty(gaussianBufferSize)
                gaussianBufferSize = 5; % Default Gaussian buffer size
            end
            if nargin < 17 || isempty(gaussianSigma)
                gaussianSigma = 1; % Default sigma for Gaussian weights
            end

            if iscell(waypoints)
                waypoints = cell2mat(waypoints); % Ensure numeric array
            end
            if size(waypoints, 2) ~= 2
                error('Waypoints must be an Nx2 numeric array.');
            end
            
            % Set default low-pass filter parameters if not defined
            obj.lowPassAlpha = 0.7; % example value, can be tuned
            obj.prevLowPassOutput = 0; % Initialize previous output

            obj.waypoints = waypoints;
            obj.wheelbase = wheelbase;
            obj.lookaheadDistance = lookaheadDistance;
            obj.maxSteeringAngle = maxSteeringAngle;
            obj.alpha = alpha;
            obj.predictionTime = predictionTime;
            obj.numPredictions = numPredictions;
            obj.bufferHorizon = bufferHorizon;
            obj.timeStep = timeStep;
            obj.offsetX = offsetX;
            obj.offsetY = offsetY;
            obj.R = R;
            obj.historyBufferSize = historyBufferSize;

            % *** Initialize Gaussian Filter Properties ***
            obj.gaussianBufferSize = gaussianBufferSize;
            obj.gaussianSigma = gaussianSigma;
            obj.steeringAngleGaussianHistory = zeros(obj.gaussianBufferSize,1); % Initialize as column vector

            % Create Gaussian kernel weights as column vector
            x = linspace(-3*obj.gaussianSigma, 3*obj.gaussianSigma, obj.gaussianBufferSize)';
            obj.gaussianWeights = exp(-0.5 * (x / obj.gaussianSigma).^2);
            obj.gaussianWeights = obj.gaussianWeights / sum(obj.gaussianWeights); % Normalize
            % Ensure gaussianWeights is a column vector
            obj.gaussianWeights = obj.gaussianWeights(:);
            % *** End of Gaussian Filter Properties ***

            % Initialize state with applied offset
            obj.position = 0; % Transpose before and after multiplication
            obj.orientation = 0;
            obj.speed = 0;
            obj.currentWaypointIndex = 1;
            obj.waypointThreshold = 15.0;  % Slightly larger threshold

            % Initialize steering
            obj.steeringAngleSmoothed = 0;
            obj.lastSteeringAngle = 0; % Start with zero steering angle
            obj.targetSteeringAngle = 0; % Initial target steering angle

            % Steering rate limiting to ensure transitions over one second
            obj.maxSteeringRate = 90; % degrees per second (example value)
            obj.steeringStep = obj.maxSteeringRate * obj.timeStep; % degrees per update

            % Additional low-pass filter factor for the final steering output
            % Closer to 1 = slower response but more stable
            obj.steeringFilterFactor = 0.95;
            obj.prevFilteredSteeringAngle = 0;

            % Initialize trajectory predictions and generated waypoints
            obj.predictedTrajectory = [];
            offsetP = (R * ([(-1)*(offsetX),(-1)*(offsetY)]).').';
            obj.generatedWaypoints = (R * (obj.waypoints).').' + offsetP; % Start with planned waypoints

            % Initialize planned steering angles buffer
            obj.plannedSteeringAngles = []; % Empty initially

            % Calculate the number of steering angles corresponding to bufferHorizon
            obj.numBufferAngles = ceil(obj.bufferHorizon / obj.timeStep);
            obj.planningHorizon = obj.numBufferAngles;

            % Initialize curvature arrays
            obj.curvatures = [];
            obj.radiusOfCurvature = [];

            % *** Initialize New Properties for Gear Shifting ***
            obj.transmission = transmission;
            obj.curvatureShiftThreshold = curvatureShiftThreshold;
            obj.shiftedDownForCurrentCurve = false;
            % *** End of New Properties ***
        end

        %% Update Vehicle State
        function obj = updateState(obj, position, orientation, speed)
            % Update the current state of the vehicle.
            %
            % Parameters:
            % - position: [x, y] current position (m)
            % - orientation: current heading (rad)
            % - speed: current speed (m/s)

            obj.position = position;       
            obj.orientation = orientation;
            obj.speed = speed;
        end

        %% Update Trajectory Predictions
        function obj = updatePredictions(obj, predictions)
            % Update the predicted trajectory and regenerate waypoints.
            %
            % Parameters:
            % - predictions: Px2 matrix of [x, y] predicted future positions

            if size(predictions, 2) ~= 2
                error('Predictions must be a Px2 numeric array.');
            end
            obj.predictedTrajectory = predictions(1:min(end, obj.numPredictions), :);

            % Generate new waypoints based on predictions and planned waypoints
            obj = obj.generateAdaptiveWaypoints();

            % Calculate curvature based on the updated waypoints
            obj = obj.calculateCurvature();

            % Plan steering angles based on the calculated curvature
            obj = obj.planSteeringAngles();
        end

        %% Compute Steering Angle
        function [obj, steeringAngle] = computeSteering(obj, currentTime)
            % Compute the steering angle based on the current state and planned angles.
            %
            % Parameters:
            % - currentTime: Current simulation time in seconds

            % Update waypoint if close enough
            obj = obj.checkAndAdvanceWaypoint();

            % If waypoints are available, compute steering towards the current waypoint
            if obj.currentWaypointIndex <= size(obj.generatedWaypoints,1)
                % Predict future state
                [predPos, predOrient] = obj.predictFutureState(obj.speed, obj.orientation, obj.lastSteeringAngle);

                % Compute raw steering angle from pure pursuit using predicted state
                rawSteeringAngle = obj.purePursuitSteering(predPos, predOrient);
            else
                % Maintain the last steering angle if all waypoints are reached
                rawSteeringAngle = obj.lastSteeringAngle;
            end

            % Apply smoothing (exponential moving average)
            obj.steeringAngleSmoothed = obj.alpha * rawSteeringAngle + (1 - obj.alpha) * obj.steeringAngleSmoothed;

            % Determine the new target steering angle
            obj.targetSteeringAngle = obj.steeringAngleSmoothed;

            % Calculate steering step towards the target angle
            angleDifference = obj.targetSteeringAngle - obj.lastSteeringAngle;
            angleDifference = obj.wrapToPiDeg(angleDifference); % Wrap between -180 and 180 degrees

            if abs(angleDifference) <= obj.steeringStep
                % If the difference is small enough, set directly to target
                newSteeringAngle = obj.targetSteeringAngle;
            else
                % Otherwise, step towards the target
                newSteeringAngle = obj.lastSteeringAngle + obj.steeringStep * sign(angleDifference);
            end

            % Apply additional low-pass filtering to further smooth the steering angle
            filteredAngle = obj.steeringFilterFactor * obj.prevFilteredSteeringAngle + ...
                            (1 - obj.steeringFilterFactor) * newSteeringAngle;

            % Update last steering angle and filtered angle memory
            obj.lastSteeringAngle = filteredAngle;
            obj.prevFilteredSteeringAngle = filteredAngle;

            % Plan path based on predictions and generated waypoints
            [obj, finalSteeringAngle] = obj.planPathWithPredictions(filteredAngle);

            % *** Apply Gaussian Filter to the Final Steering Angle ***
            % Update the Gaussian history buffer
            obj.steeringAngleGaussianHistory = [finalSteeringAngle; obj.steeringAngleGaussianHistory(1:end-1)];

            % Compute the weighted sum using Gaussian weights
            gaussianSteeringAngle = sum(obj.gaussianWeights .* obj.steeringAngleGaussianHistory);

            % Assign the Gaussian filtered steering angle as the final steering angle
            steeringAngle = gaussianSteeringAngle;
            % *** End of Gaussian Filtering ***

            % *** Low-Pass Filter ***
            % Smooth the signal further to reduce peaks at the start
            filteredAngle = obj.lowPassAlpha * gaussianSteeringAngle + (1 - obj.lowPassAlpha)*obj.prevLowPassOutput;

            % Update the previous output
            obj.prevLowPassOutput = filteredAngle;

            % Use the low-pass filtered output as the final steering angle
            steeringAngle = filteredAngle;

            % *** Gear Shifting Logic Based on Curvature ***
            % Analyze upcoming curvatures to determine if a gear shift is needed
            if ~isempty(obj.curvatures)
                % Consider the minimum radius of curvature ahead
                minRadius = min(obj.radiusOfCurvature(obj.currentWaypointIndex:end));

                % Calculate current curvature
                currentCurvature = obj.curvatures(obj.currentWaypointIndex);

                % Check if the curvature exceeds the threshold (i.e., radius below threshold)
                if currentCurvature > obj.curvatureShiftThreshold
                    % Ensure we haven't already shifted down for the current curve
                    if ~obj.shiftedDownForCurrentCurve && obj.transmission.currentGear > 1
                        % Command transmission to shift down by one gear
                        obj.transmission.shiftDown(currentTime);
                        obj.shiftedDownForCurrentCurve = true;
                        fprintf('Curvature detected (Radius: %.2f m). Commanded shift down to gear %d.\n', ...
                                obj.radiusOfCurvature(obj.currentWaypointIndex), obj.transmission.currentGear);

                        % Optional: Reduce speed when shifting down
                        desiredSpeed = max(obj.speed - 5, 0); % Reduce speed by 5 m/s, not below 0
                        obj.speed = desiredSpeed;
                        fprintf('Speed reduced to %.2f m/s due to gear shift down.\n', obj.speed);
                    end
                else
                    % If curvature is below the threshold and previously shifted down, allow shifting up
                    if obj.shiftedDownForCurrentCurve
                        obj.shiftedDownForCurrentCurve = false;
                    end
                end
            end
            % *** End of Gear Shifting Logic ***
        end
    end

    methods (Access = private)
        %% Generate Adaptive Waypoints Based on Predictions
        function obj = generateAdaptiveWaypoints(obj)
            % Generate waypoints by considering both planned waypoints and predicted trajectory
            % to correct the predicted path for a smooth and straightforward trajectory.

            % Step 1: Retrieve the current target waypoint
            if obj.currentWaypointIndex > size(obj.waypoints,1)
                % No more waypoints to follow
                obj.generatedWaypoints = obj.waypoints;
                return;
            end

            targetWp = obj.waypoints(obj.currentWaypointIndex, :);

            % Step 2: Analyze the predicted trajectory geometry
            if ~isempty(obj.predictedTrajectory)
                % Fit a line or curve to the predicted trajectory
                % For simplicity, we'll compute the average direction of the predictions
                directionVec = obj.predictedTrajectory(end, :) - obj.predictedTrajectory(1, :);
                distance = norm(directionVec);
                if distance == 0
                    avgDirection = [cos(obj.orientation), sin(obj.orientation)];
                else
                    avgDirection = directionVec / distance;
                end
            else
                % If no predictions, use the direction to the target waypoint
                directionVec = targetWp - obj.position;
                distance = norm(directionVec);
                if distance == 0
                    avgDirection = [cos(obj.orientation), sin(obj.orientation)];
                else
                    avgDirection = directionVec / distance;
                end
            end

            % Step 3: Generate waypoints in the direction of the average prediction
            waypointSpacing = 1.0; % meters between waypoints
            distanceToTarget = norm(targetWp - obj.position);
            numWaypoints = ceil(distanceToTarget / waypointSpacing);

            newWaypoints = zeros(numWaypoints, 2);
            for i = 1:numWaypoints
                newWaypoints(i, :) = obj.position + avgDirection * waypointSpacing * i;
            end

            % Step 4: Append the target waypoint to ensure inclusion
            if norm(newWaypoints(end, :) - targetWp) > 0.1 % 10 cm tolerance
                newWaypoints = [newWaypoints; targetWp];
            end

            % Step 5: Apply global offsets if any
            offsetP = (obj.R * ([(-1)*(obj.offsetX),(-1)*(obj.offsetY)]).').';
            obj.generatedWaypoints = (obj.R * (obj.waypoints).').' + offsetP; % Start with planned waypoints
            newWaypoints = newWaypoints + offsetP;

            % Step 6: Assign the generated waypoints
            obj.generatedWaypoints = newWaypoints;
        end

        %% Calculate Curvature Based on Waypoints
        function obj = calculateCurvature(obj)
            % Calculate the curvature and radius of curvature for the upcoming waypoints.

            numPoints = size(obj.generatedWaypoints, 1);
            obj.curvatures = zeros(numPoints, 1);
            obj.radiusOfCurvature = Inf(numPoints, 1); % Initialize as straight lines

            for i = 2:(numPoints-1)
                p_prev = obj.generatedWaypoints(i-1, :);
                p_current = obj.generatedWaypoints(i, :);
                p_next = obj.generatedWaypoints(i+1, :);

                % Calculate the angle between the segments
                v1 = p_current - p_prev;
                v2 = p_next - p_current;

                angle = atan2(v2(2), v2(1)) - atan2(v1(2), v1(1));
                angle = obj.wrapToPi(angle);

                % Calculate the distance between points
                a = norm(v1);
                b = norm(v2);
                c = norm(p_next - p_prev);

                % Calculate the radius of curvature using the circumradius formula
                if sin(angle) ~= 0
                    area = abs((a * b * sin(angle)) / 2);
                    radius = (a * b * c) / (4 * area);
                    curvature = 1 / radius;
                else
                    radius = Inf;
                    curvature = 0;
                end

                obj.curvatures(i) = curvature;
                obj.radiusOfCurvature(i) = radius;
            end

            % Handle the first and last points (set to zero curvature)
            obj.curvatures(1) = 0;
            obj.radiusOfCurvature(1) = Inf;
            obj.curvatures(end) = 0;
            obj.radiusOfCurvature(end) = Inf;
        end

        %% Plan Steering Angles for Upcoming Waypoints Based on Curvature
        function obj = planSteeringAngles(obj)
            % Plan a set of steering angles for the next 'planningHorizon' waypoints

            numToPlan = min(obj.planningHorizon, size(obj.generatedWaypoints,1) - obj.currentWaypointIndex + 1);
            if numToPlan <= 0
                return; % Nothing to plan
            end

            plannedAngles = zeros(numToPlan,1);
            for i = 1:numToPlan
                idx = obj.currentWaypointIndex + i -1;
                if idx > size(obj.generatedWaypoints,1)
                    targetWp = obj.generatedWaypoints(end, :);
                    curvature = obj.curvatures(end);
                else
                    targetWp = obj.generatedWaypoints(idx, :);
                    curvature = obj.curvatures(idx);
                end

                % Compute steering angle based on curvature
                steeringAngle = obj.steeringAngleFromCurvature(curvature);
                plannedAngles(i) = steeringAngle;
            end

            % Append planned angles to the buffer
            obj.plannedSteeringAngles = [obj.plannedSteeringAngles; plannedAngles];

            % Limit the buffer size to 'numBufferAngles' to cover bufferHorizon
            maxBufferSize = obj.numBufferAngles;
            if length(obj.plannedSteeringAngles) > maxBufferSize
                obj.plannedSteeringAngles = obj.plannedSteeringAngles(end - maxBufferSize +1 : end);
            end
        end

        %% Compute Steering Angle from Curvature
        function steeringAngle = steeringAngleFromCurvature(obj, curvature)
            % Compute the desired steering angle based on the curvature.
            %
            % Parameters:
            % - curvature: curvature value (1/m)
            %
            % Returns:
            % - steeringAngle: steering angle in degrees

            if curvature == 0
                steeringAngle = 0;
            else
                % Radius of curvature
                radius = 1 / curvature;

                % Steering angle calculation using bicycle model
                steeringAngleRad = atan(obj.wheelbase / radius);
                steeringAngle = rad2deg(steeringAngleRad);

                % Determine the sign based on the curvature direction
                if curvature < 0
                    steeringAngle = -abs(steeringAngle);
                else
                    steeringAngle = abs(steeringAngle);
                end

                % Clamp to maximum steering angle
                steeringAngle = max(min(steeringAngle, obj.maxSteeringAngle), -obj.maxSteeringAngle);
            end
        end

        %% Adjusted Waypoint Getter
        function wp = getCurrentWaypoint(obj, idx)
            % Retrieve the current waypoint from the generated waypoints
            wp = obj.generatedWaypoints(idx,:);
        end

        %% Check and Advance Waypoint
        function obj = checkAndAdvanceWaypoint(obj)
            % Check if the current waypoint is reached and advance to the next one

            if obj.currentWaypointIndex <= size(obj.generatedWaypoints,1)
                currentWp = obj.getCurrentWaypoint(obj.currentWaypointIndex);
                distToWaypoint = norm(obj.position - currentWp);

                hysteresisFactor = 0.9; 
                effectiveThreshold = obj.waypointThreshold * hysteresisFactor;

                if distToWaypoint < effectiveThreshold
                    obj.currentWaypointIndex = obj.currentWaypointIndex + 1;
                    if obj.currentWaypointIndex > size(obj.generatedWaypoints,1)
                        % *** Modified to Prevent Resetting Steering Angle ***
                        % Previously: Loop back to the first waypoint
                        obj.currentWaypointIndex = 1;

                        % Now: Clamp to the last waypoint to maintain steering direction
                        %obj.currentWaypointIndex = size(obj.generatedWaypoints,1); % Clamp to last waypoint
                        % Optionally, set a flag to indicate path completion
                        % obj.pathCompleted = true;
                    end
                end
            end
        end

        %% Predict Future State
        function [predPos, predOrient] = predictFutureState(obj, speed, orientation, steeringAngleDeg)
            % Predict the future state of the vehicle based on current speed and steering angle

            steeringAngleRad = deg2rad(steeringAngleDeg);

            orientRate = (speed / obj.wheelbase) * tan(steeringAngleRad);
            predOrient = orientation + orientRate * obj.predictionTime;

            dx = speed * obj.predictionTime * cos(predOrient);
            dy = speed * obj.predictionTime * sin(predOrient);
            predPos = obj.position + [dx, dy];
        end

        %% Pure Pursuit Steering
        function steeringAngle = purePursuitSteering(obj, posePos, poseOrient)
            % Compute the steering angle using the Pure Pursuit algorithm

            lookaheadPoint = obj.findLookaheadPoint(posePos);

            dx = lookaheadPoint(1) - posePos(1);
            dy = lookaheadPoint(2) - posePos(2);
            angleToTarget = atan2(dy, dx);
            headingError = obj.wrapToPi(angleToTarget - poseOrient);

            kappa = 2 * sin(headingError) / obj.lookaheadDistance;
            steeringAngleRad = atan(obj.wheelbase * kappa);

            steeringAngle = rad2deg(steeringAngleRad);
            steeringAngle = max(min(steeringAngle, obj.maxSteeringAngle), -obj.maxSteeringAngle);
        end

        %% Find Lookahead Point
        function lookaheadPoint = findLookaheadPoint(obj, posePos)
            % Find the lookahead point on the path for the Pure Pursuit algorithm

            if obj.currentWaypointIndex > size(obj.generatedWaypoints, 1)
                rawLookaheadPoint = obj.getCurrentWaypoint(size(obj.generatedWaypoints,1));
            else
                rawLookaheadPoint = obj.getCurrentWaypoint(obj.currentWaypointIndex);
                startIdx = obj.currentWaypointIndex;

                for i = startIdx:(size(obj.generatedWaypoints,1)-1)
                    p1 = obj.getCurrentWaypoint(i);
                    p2 = obj.getCurrentWaypoint(i+1);

                    [projPoint, ~] = obj.projectOnSegment(posePos, p1, p2);
                    dist = norm(projPoint - posePos);

                    if dist >= obj.lookaheadDistance
                        rawLookaheadPoint = projPoint;
                        break;
                    else
                        endpointDist = norm(p2 - posePos);
                        if endpointDist >= obj.lookaheadDistance
                            rawLookaheadPoint = p2;
                            break;
                        end
                    end
                end
            end

            persistent prevLookaheadPoint;
            if isempty(prevLookaheadPoint)
                prevLookaheadPoint = rawLookaheadPoint;
            end
            smoothFactor = 0.7; 
            lookaheadPoint = smoothFactor * rawLookaheadPoint + (1 - smoothFactor)*prevLookaheadPoint;
            prevLookaheadPoint = lookaheadPoint;
        end

        %% Project Point on Segment
        function [projPoint, t] = projectOnSegment(~, point, segStart, segEnd)
            % Project a point onto a line segment and compute the projection parameter t
            v = segEnd - segStart;
            w = point - segStart;
            t = dot(w,v) / dot(v,v);
            t = max(0, min(1, t));
            projPoint = segStart + t * v;
        end

        %% Wrap Angle to [-pi, pi]
        function angle = wrapToPi(~, angle)
            % Wrap an angle in radians to the range [-pi, pi]
            angle = mod(angle + pi, 2*pi) - pi;
        end

        %% Wrap Angle to [-180, 180] Degrees
        function angle = wrapToPiDeg(~, angle)
            % Wrap an angle in degrees to the range [-180, 180]
            angle = mod(angle + 180, 360) - 180;
        end

        %% Plan Path Based on Predictions and Generated Waypoints
        function [obj, finalSteeringAngle] = planPathWithPredictions(obj, currentSteeringAngle)
            % Plan the path by analyzing the predicted trajectory and generated waypoints.
            % If a zig-zag pattern is detected in the predicted trajectory, apply a moving
            % average filter to smooth the steering angles.

            % Step 1: Analyze the predicted trajectory for zig-zag patterns
            isZigZag = obj.detectZigZag();

            % Step 2: If zig-zag detected, apply moving average to planned steering angles
            if isZigZag
                % Apply moving average to the planned steering angles
                if ~isempty(obj.plannedSteeringAngles)
                    movingAvgAngle = mean(obj.plannedSteeringAngles);
                else
                    movingAvgAngle = currentSteeringAngle;
                end
            else
                % No zig-zag detected, use the current steering angle
                movingAvgAngle = currentSteeringAngle;
            end

            % Step 3: Further smooth the steering angle using the steering angle history
            obj.steeringAngleHistory = [obj.steeringAngleHistory; movingAvgAngle];
            if length(obj.steeringAngleHistory) > obj.historyBufferSize
                obj.steeringAngleHistory = obj.steeringAngleHistory(end - obj.historyBufferSize +1 : end);
            end
            finalSteeringAngle = mean(obj.steeringAngleHistory);
        end

        %% Detect Zig-Zag Pattern in Planned Steering Angles
        function isZigZag = detectZigZag(obj)
            % Detect if the planned steering angles have a zig-zag pattern based on
            % the variance or standard deviation of the angles.

            % Define a threshold for standard deviation to consider it as zig-zag
            zigZagThreshold = 5; % degrees, tunable parameter

            if isempty(obj.plannedSteeringAngles)
                isZigZag = false;
                return;
            end

            % Calculate standard deviation of the planned steering angles
            stdSteering = std(obj.plannedSteeringAngles);

            % Determine if the standard deviation exceeds the threshold
            if stdSteering > zigZagThreshold
                isZigZag = true;
            else
                isZigZag = false;
            end
        end
    end
end
