%% Build ABB IRB 140 Manipulator Using Denavit-Hartenberg Parameters
% By: Rakshith Badarinath
% 
% Date: 20/05/2019

%% Part-1: Develop D-H matrix for ABB IRB 140
% % 
% % 
% % 
% The DH parameters define the geometry of the robot with relation to how each 
% rigid body is attached to its parent. The DH matrix for ABB IRB 140 robot is 
% given by:     
% 
% % 
% Format for entering DH parameters: [a  alpha d thetha]

classic_dhparams = [0.070    -pi/2   0.352   0;
                    0.360     0      0      -pi/2;
                    0.000    -pi/2   0       0;
                    0.000     pi/2   0.380   0;
                    0.000    -pi/2   0       0;
                    0.000     0      0.065   pi];    
        
modified_dhparams = [0.000   0      0.352   0;
                     0.070  -pi/2   0      -pi/2;
                     0.360   0      0       0;
                     0.000  -pi/2   0.380   0;
                     0.000   pi/2   0       0;
                     0.000  -pi/2   0.065   pi];  
%% Create a rigid body tree object to build the robot.

abb_irb140_mdh = robotics.RigidBodyTree;

% Create the first rigid body and add it to the robot. To add a rigid body:
% # Create a |RigidBody| object and give it a unique name.
% # Create a |Joint| object and give it a unique name.
% # Use |setFixedTransform| to specify the body-to-body transformation using 
% DH parameters. The last element of the DH parameters, |theta|, is ignored because 
% the angle is dependent on the joint position.
% # Call |addBody| to attach the first body joint to the base frame of the robot.

jointNames = ['jnt1';'jnt2';'jnt3';'jnt4';'jnt5';'jnt6'];
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint(jointNames(1,:),'revolute');
jnt1.PositionLimits = [-pi pi];
jnt1.HomePosition = 0;

setFixedTransform(jnt1,modified_dhparams(1,:),'mdh');
body1.Joint = jnt1;

addBody(abb_irb140_mdh,body1,'base')

% Create and add other rigid bodies to the robot. Specify the previous body 
% name when calling |addBody| to attach it. Each fixed transform is relative to 
% the previous joint coordinate frame.

body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint(jointNames(2,:),'revolute');
jnt2.PositionLimits = [-pi/2 110*pi/180];
jnt2.HomePosition = -pi/2;

body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint(jointNames(3,:),'revolute');
jnt3.PositionLimits = [-230*pi/180 50*pi/180];
jnt3.HomePosition = 0;

body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint(jointNames(4,:),'revolute');
jnt4.PositionLimits = [-200*pi/180 200*pi/180];
jnt4.HomePosition = 0;

body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint(jointNames(5,:),'revolute');
jnt5.PositionLimits = [-120*pi/180 120*pi/180];
jnt5.HomePosition = 0;

body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint(jointNames(6,:),'revolute');
jnt6.PositionLimits = [-400*pi/180 400*pi/180];
jnt6.HomePosition = pi;

% Assign DH parameters
setFixedTransform(jnt2,modified_dhparams(2,:),'mdh');
setFixedTransform(jnt3,modified_dhparams(3,:),'mdh');
setFixedTransform(jnt4,modified_dhparams(4,:),'mdh');
setFixedTransform(jnt5,modified_dhparams(5,:),'mdh');
setFixedTransform(jnt6,modified_dhparams(6,:),'mdh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

addBody(abb_irb140_mdh,body2,'body1');
addBody(abb_irb140_mdh,body3,'body2');
addBody(abb_irb140_mdh,body4,'body3');
addBody(abb_irb140_mdh,body5,'body4');
addBody(abb_irb140_mdh,body6,'body5');

config = homeConfiguration(abb_irb140_mdh);
thetaHome = [config(1).JointPosition config(2).JointPosition config(3).JointPosition config(4).JointPosition config(5).JointPosition config(6).JointPosition];