%% Function 2: Doing Forward Kinematics
function v = findXYZRPY(q)

% This Function calculates the xyz rpy and puts it into a 6x1 vector
% The function has the variable q, which is a 1x6 matrix containing the
% values of theta

% Here we save the values from the vector q (theta) as t1, t2, t3, t4 t5 and t6
% which are used in the equations below
t1 = q(1);  t2 = q(2);
t3 = q(3);  t4 = q(4);
t5 = q(5);  t6 = q(6);

% We find the xyz using the given theta value
x = (49*sin(t1))/5 + 410*cos(t1)*cos(t2) - (107*3^(1/2)*(sin(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) - (cos(t5)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))))/2 + (3^(1/2)*cos(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)))/2))/5 - (107*3^(1/2)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))))/2 - (607*cos(t1)*cos(t2)*sin(t3))/2 - (607*cos(t1)*cos(t3)*sin(t2))/2;
y = 410*cos(t2)*sin(t1) - (49*cos(t1))/5 - (107*3^(1/2)*((cos(t5)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))))/2 - sin(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) + (3^(1/2)*cos(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)))/2))/5 + (107*3^(1/2)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))))/2 - (607*cos(t2)*sin(t1)*sin(t3))/2 - (607*cos(t3)*sin(t1)*sin(t2))/2;
z = (321*3^(1/2)*sin(t2 + t3)*sin(t4 + t5))/20 - 410*sin(t2) - (321*cos(t2 + t3)*cos(t5))/10 - (607*cos(t2 + t3))/2 - (107*3^(1/2)*sin(t2 + t3)*sin(t4))/2 - (107*3^(1/2)*sin(t4 - t5)*sin(t2 + t3))/20 + 551/2;

% We find the 5 values from the rotation matrix that are used for the roll
% pitch yaw equtions:

R33 = (3^(1/2)*sin(t2 + t3)*cos(t4)*sin(t5))/2 - (3*cos(t2 + t3)*cos(t5))/4 - (3^(1/2)*sin(t2 + t3)*sin(t4))/4 - cos(t2 + t3)/4 + (3^(1/2)*sin(t2 + t3)*cos(t5)*sin(t4))/4;
R32 = sin(t6)*((cos(t4 - t5)*sin(t2 + t3))/4 + (3*cos(t4 + t5)*sin(t2 + t3))/4 + (3^(1/2)*cos(t2 + t3)*sin(t5))/2) - cos(t6)*((sin(t4 - t5)*sin(t2 + t3))/8 - (3*sin(t2 + t3)*sin(t4 + t5))/8 + (3^(1/2)*cos(t2 + t3)*cos(t5))/4) + (3^(1/2)*cos(t6)*(cos(t2 + t3)/2 + (3^(1/2)*sin(t2 + t3)*sin(t4))/2))/2;
R31 = (3^(1/2)*sin(t6)*(cos(t2 + t3)/2 + (3^(1/2)*sin(t2 + t3)*sin(t4))/2))/2 - sin(t6)*((sin(t4 - t5)*sin(t2 + t3))/8 - (3*sin(t2 + t3)*sin(t4 + t5))/8 + (3^(1/2)*cos(t2 + t3)*cos(t5))/4) - cos(t6)*((cos(t4 - t5)*sin(t2 + t3))/4 + (3*cos(t4 + t5)*sin(t2 + t3))/4 + (3^(1/2)*cos(t2 + t3)*sin(t5))/2);
R11 = cos(t6)*(cos(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) + (sin(t5)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))))/2 - (3^(1/2)*sin(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)))/2) - (sin(t6)*(sin(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) - (cos(t5)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))))/2 + (3^(1/2)*cos(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)))/2))/2 + (3^(1/2)*sin(t6)*((3^(1/2)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))))/2 + (cos(t1)*cos(t2)*sin(t3))/2 + (cos(t1)*cos(t3)*sin(t2))/2))/2;
R21 = (3^(1/2)*sin(t6)*((cos(t2)*sin(t1)*sin(t3))/2 - (3^(1/2)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))))/2 + (cos(t3)*sin(t1)*sin(t2))/2))/2 - (sin(t6)*((cos(t5)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))))/2 - sin(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) + (3^(1/2)*cos(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)))/2))/2 - cos(t6)*(cos(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) + (sin(t5)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))))/2 + (3^(1/2)*sin(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)))/2);

% Then we calculate the RPY using the elements from the rotation matrix
% These below are the general equations for roll pitch and yaw
PitchEq= atan2(-R31, sqrt(R11^2 + R21^2));
RollEq = atan2(R21/cos(PitchEq), R11/cos(PitchEq));
YawEq = atan2(R32/cos(PitchEq), R33/cos(PitchEq));

% We take the calculated xyz and rpy and puts them into a vector v
v = [x;y;z;RollEq;PitchEq;YawEq];

% End  of the function
end