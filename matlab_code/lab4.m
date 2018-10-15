% Create the new ArmRobot object
robot = ArmRobot('COM26');

% Set configuration of the robot
robot.setServoCenters([1500 1520 1400 1550 1500 1500]);
robot.setServoBounds([500 700 950 500 500 900],[2420 2100 2050 2500 2500 2000]);
robot.setLinkLengths([5 5 2 12 13.5 6 6.5]);

% Connect to the Robot
robot.connect();

display('Press Any Key to go to home position');
%pause();


home = [0 0 0 0 0 -10];
joints = [0 1 2 3 4 5];
robot.moveAbsolute(home, joints);

pause()

draw = [90 0 0 90 0 0];
robot.moveRelative(draw, joints);

pause();

draw = [-45 0 0 -45 0 0];
robot.moveRelative(draw, joints);

pause();

draw = [-45 0 0 -45 0 0];
robot.moveRelative(draw, joints);

% for j = 0:3
%     draw = [0 0 10 10 0 0];
%     robot.moveRelative(draw, joints);
% end



% for j = 0:10
%     robot.moveAbsolute(home, joints);
%     
%     %pause()
% 
%     for i = 0:30
%         draw = [0 0 i i 0 -10];
%         robot.moveAbsolute(draw, joints);
%     end
%     
%     for i = 30:-1:0
%         draw = [0 0 i i 0 -10];
%         robot.moveAbsolute(draw, joints);
%     end
% end

%robot.moveAbsolute(home, joints);




% 
% Step 0 - Home Position 
%home = [1500 1520 1400 1550 1500 1500];
%robot.moveJoints(home, joints);
%pause();

% % Step 1



% display('Press Any Key for Step 2');
% pause();
% 
% % Step 2
% display('Press Any Key for Step 3');
% pause();
% 
% % Step 3
% display('Press Any Key for Step 4');
% pause();
% 
%  
% % Step 4
% display('Press Any Key for Step 5');
% pause();
%  
% % Step 5
% display('Press Any Key for Step 6');
% pause();
%  
% % Step 6
% display('Press Any Key to all steps absolute');
% pause();
% 
% % Now Lets Try that All At Once (Absolute)
% display('Press Any Key to all steps relative');
% pause();
% 
% % Now Lets Try that All At Once (Relative)
% display('Press Any Key to all steps relative linear');
% pause();
% 
% % Now Lets Try that SMOOTHLY All At Once (Relative Linear)
% display('Press Any Key to all steps absolute linear');
% pause();

% Close out the robot
robot.delete();
