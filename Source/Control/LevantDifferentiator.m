% Author: Miguel Marina <karel.capek.robotics@gmail.com>
classdef LevantDifferentiator < handle
    %LEVANTDIFFERENTIATOR Implements a 1st-order Levant differentiator.
    %   This differentiator estimates the derivative of a signal using the
    %   robust sliding-mode algorithm proposed by Levant. It maintains the
    %   internal states z0 and z1 and updates them each call.
    %
    %   Example usage:
    %       diff = LevantDifferentiator(1.0, 1.0);
    %       xdot = diff.update(x, dt);
    %
    %   lambda1 and lambda2 are tunable gains controlling the convergence
    %   speed and noise rejection. Higher values yield faster convergence
    %   but can amplify noise.
    
    properties
        lambda1 double = 1.0  % Gain for z0 term
        lambda2 double = 1.0  % Gain for z1 term
        z0 double = 0         % Internal state estimate of signal
        z1 double = 0         % Internal state estimate of derivative
    end

    methods
        function obj = LevantDifferentiator(lambda1, lambda2)
            if nargin >= 1 && ~isempty(lambda1)
                obj.lambda1 = lambda1;
            end
            if nargin >= 2 && ~isempty(lambda2)
                obj.lambda2 = lambda2;
            end
        end

        function dx = update(obj, x, dt)
            %UPDATE Update the differentiator with new measurement x.
            %   dx = obj.update(x, dt) returns the derivative estimate.
            if dt <= 0
                dx = obj.z1;
                return;
            end
            e = obj.z0 - x;
            dz0 = obj.z1 - obj.lambda1 * sqrt(abs(e)) * sign(e);
            dz1 = -obj.lambda2 * sign(e);
            obj.z0 = obj.z0 + dz0 * dt;
            obj.z1 = obj.z1 + dz1 * dt;
            dx = obj.z1;
        end

        function reset(obj)
            obj.z0 = 0;
            obj.z1 = 0;
        end
    end
end
