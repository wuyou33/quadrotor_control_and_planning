function tests = collide_test
tests = functiontests(localfunctions);
end

function assertNoCollision(testcase, map, points)
c = collide(map, points);
verifyEqual(testcase, size(points, 1), length(c(:)));
verifyFalse(testcase, any(c));
end

function assertCollision(testcase, map, points)
c = collide(map, points);
verifyTrue(testcase, isvector(c));
verifyEqual(testcase, size(points, 1), length(c(:)));
verifyTrue(testcase, all(c));
end

function testMap0(testcase)
map = load_map('map0.txt', 0.2, 0.5, 0.2);

valid = [0.0  -1.0 2.0; 
         3.0  17.0 4.0; 
         0.0  -5.0 0.5];
collision = [0.0 2.0 1.0; 
             3.0 18.5 4.5];
assertNoCollision(testcase, map, valid);
assertCollision(testcase, map, collision);
end

