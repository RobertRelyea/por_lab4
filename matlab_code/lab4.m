% Create the new ArmRobot object
robot = ArmRobot('COM26');

% Set configuration of the robot
robot.setServoCenters([1500 1520 1400 1550 1500 1500]);
robot.setServoBounds([500 700 950 500 500 900],[2420 2100 2050 2500 2500 2000]);
robot.setLinkLengths([4.375 4.0625 .9375 11.87 12.18 6.25 5.625]);

% Connect to the Robot
robot.connect();

% Step 0: Home
home = [0 0 0 0 0 -10];
joints = [0 1 2 3 4 5];
robot.moveAbsolute(home, joints);
pause(3);

draw = [-90];
joints = [0];
robot.moveAbsoluteLinear(draw, joints, 30);
pause(1);

draw = [30];
joints = [0];
robot.moveRelativeLinear(draw, joints, 30);
pause(1);

%{
% Step 1: Rotate shoulder -30
draw = [-50];
joints = [1];
robot.moveRelative(draw, joints);
%pause(1);

% Step 2
draw = [-60];
joints = [2];
robot.moveRelative(draw, joints);
pause(.25);

% Step 
draw = [-90];
joints = [3];
robot.moveRelative(draw, joints);
pause(1);

draw = [30];
joints = [5];
robot.moveAbsolute(draw, joints);
pause(1);

% Step 
draw = [90];
joints = [3];
robot.moveRelative(draw, joints);
pause(.25);

% Step 
draw = [90];
joints = [4];
robot.moveRelative(draw, joints);
pause(.25);

% Step 1: Rotate shoulder -30
draw = [-50];
joints = [0];
robot.moveRelative(draw, joints);

pause(1)
draw = [-10];
joints = [5];
robot.moveAbsolute(draw, joints);

joints = [0 1 2 3 4 5];
robot.moveAbsolute(home, joints);
pause(1);
%}

% Close out the robot
robot.delete();
