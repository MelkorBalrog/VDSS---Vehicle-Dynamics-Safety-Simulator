% Author: Miguel Marina <karel.capek.robotics@gmail.com>
function tests = DynamicsUpdaterMassTest
    tests = functiontests(localfunctions);
end

function testSetMassPreservesVelocity(testCase)
    fc = StubForceCalc();
    kc = StubKinCalc();
    trans = StubTransmission();

    init.position = [0;0];
    init.orientation = 0;
    init.velocity = 5;
    init.lateralVelocity = 0;
    init.yawRate = 0;
    init.rollAngle = 0;
    init.rollRate = 0;

    du = DynamicsUpdater(fc, kc, init, 1000, 3, 1, 2, 0.01, 'tractor', 2, 0, 0, trans);
    du = du.setMass(2000);

    verifyEqual(testCase, du.velocity, 5, 'AbsTol', 1e-10);
    verifyEqual(testCase, du.linearMomentum(1), 2000*5, 'AbsTol', 1e-10);
end

classdef StubForceCalc
    properties
        inertia = [1 1 1];
    end
end

classdef StubKinCalc
end

classdef StubTransmission
    properties
        currentGear = 1;
    end
    methods
        function obj = updateGear(obj, ~, ~, ~, ~)
        end
    end
end
