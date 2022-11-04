
%% Inverse kinematics test code that verifies a given set
function inverseKinematicsTest()

    q0 = [0 10 0 0 0 0].'; % TODO: Document this

    targetXYZRPY = [300; ...   % X [i m]
    141.6913; ...              % Y [i m]
    0.0946; ...                % Z [i m]
    0; ...             % Roll (In radians) needs to be positive from 0:360.
    -90.0000; ...      % Pitch (In radians) needs to be positive from 0:360
    -173.0043];        %Yaw   (In radians) needs to be positive from 0:360



    q = fminunc(@(q) MinKin(q, targetXYZRPY), q0); % Convert xyzRPY (targetXYZPRY) to theta values
    v = GetJacoPosition(q * pi / 180); % Get the newly found position
    v(1:3) = v(1:3); % xyz to mm
    v(4:end) = v(4:end) * 180 / pi; % RPY to degrees
    disp(v)

end

function res = MinKin(q, P)
%% Calculate x, y, z, roll, pitch and yaw for the current theta values
v = GetJacoPosition(q*pi/180);
v(4:end) = v(4:end)*180/pi;

%% Find the error from the values we want and the values we want with the current theta values
error = v-P;

%% Following lines is made to make adjust angular errors:
if any(abs(error(4:end))>180)
    rpy_error = abs(error(4:end));
    rpy_error(rpy_error>180) = rpy_error(rpy_error>180)-360;
    error(4:end) = rpy_error;
end

%% root-mean-square of the error to yield a numeric value that indicates how far from the solution the error is.
res = rms(error); 
end

%% Forward kinematics for this inverse kinematics calculations (Different from forwardKinematicsTest.m)
function v = GetJacoPosition(q)
 
t1 = q(1);  t2 = q(2);
t3 = q(3);  t4 = q(4);
t5 = q(5);  t6 = q(6);

% Calculating the x,y and z
x = (49*sin(t1))/5 + 410*cos(t1)*cos(t2) - (107*3^(1/2)*(sin(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) - (cos(t5)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))))/2 + (3^(1/2)*cos(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)))/2))/5 - (107*3^(1/2)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))))/2 - (607*cos(t1)*cos(t2)*sin(t3))/2 - (607*cos(t1)*cos(t3)*sin(t2))/2;
y = 410*cos(t2)*sin(t1) - (49*cos(t1))/5 - (107*3^(1/2)*((cos(t5)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))))/2 - sin(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) + (3^(1/2)*cos(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)))/2))/5 + (107*3^(1/2)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))))/2 - (607*cos(t2)*sin(t1)*sin(t3))/2 - (607*cos(t3)*sin(t1)*sin(t2))/2;
z = (321*3^(1/2)*sin(t2 + t3)*sin(t4 + t5))/20 - 410*sin(t2) - (321*cos(t2 + t3)*cos(t5))/10 - (607*cos(t2 + t3))/2 - (107*3^(1/2)*sin(t2 + t3)*sin(t4))/2 - (107*3^(1/2)*sin(t4 - t5)*sin(t2 + t3))/20 + 551/2;

% Fetching R, P, Y from the rotation matrix's four variables.
R33 = (3^(1/2)*sin(t2 + t3)*cos(t4)*sin(t5))/2 - (3*cos(t2 + t3)*cos(t5))/4 - (3^(1/2)*sin(t2 + t3)*sin(t4))/4 - cos(t2 + t3)/4 + (3^(1/2)*sin(t2 + t3)*cos(t5)*sin(t4))/4;
R32 = sin(t6)*((cos(t4 - t5)*sin(t2 + t3))/4 + (3*cos(t4 + t5)*sin(t2 + t3))/4 + (3^(1/2)*cos(t2 + t3)*sin(t5))/2) - cos(t6)*((sin(t4 - t5)*sin(t2 + t3))/8 - (3*sin(t2 + t3)*sin(t4 + t5))/8 + (3^(1/2)*cos(t2 + t3)*cos(t5))/4) + (3^(1/2)*cos(t6)*(cos(t2 + t3)/2 + (3^(1/2)*sin(t2 + t3)*sin(t4))/2))/2;
R31 = (3^(1/2)*sin(t6)*(cos(t2 + t3)/2 + (3^(1/2)*sin(t2 + t3)*sin(t4))/2))/2 - sin(t6)*((sin(t4 - t5)*sin(t2 + t3))/8 - (3*sin(t2 + t3)*sin(t4 + t5))/8 + (3^(1/2)*cos(t2 + t3)*cos(t5))/4) - cos(t6)*((cos(t4 - t5)*sin(t2 + t3))/4 + (3*cos(t4 + t5)*sin(t2 + t3))/4 + (3^(1/2)*cos(t2 + t3)*sin(t5))/2);
R11 = cos(t6)*(cos(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) + (sin(t5)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))))/2 - (3^(1/2)*sin(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)))/2) - (sin(t6)*(sin(t5)*(sin(t1)*sin(t4) - cos(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))) - (cos(t5)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))))/2 + (3^(1/2)*cos(t5)*(cos(t1)*cos(t2)*sin(t3) + cos(t1)*cos(t3)*sin(t2)))/2))/2 + (3^(1/2)*sin(t6)*((3^(1/2)*(cos(t4)*sin(t1) + sin(t4)*(cos(t1)*sin(t2)*sin(t3) - cos(t1)*cos(t2)*cos(t3))))/2 + (cos(t1)*cos(t2)*sin(t3))/2 + (cos(t1)*cos(t3)*sin(t2))/2))/2;
R21 = (3^(1/2)*sin(t6)*((cos(t2)*sin(t1)*sin(t3))/2 - (3^(1/2)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))))/2 + (cos(t3)*sin(t1)*sin(t2))/2))/2 - (sin(t6)*((cos(t5)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))))/2 - sin(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) + (3^(1/2)*cos(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)))/2))/2 - cos(t6)*(cos(t5)*(cos(t1)*sin(t4) + cos(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))) + (sin(t5)*(cos(t1)*cos(t4) - sin(t4)*(sin(t1)*sin(t2)*sin(t3) - cos(t2)*cos(t3)*sin(t1))))/2 + (3^(1/2)*sin(t5)*(cos(t2)*sin(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)))/2);
 
Roll = atan2(R32, R33);
Pitch = atan2(-R31, sqrt(R32^2 + R33^2));
Yaw = atan2(R21, R11);

% Returns the new x, y, z, Roll, Pitch and Yaw in a vector.
v = [x;y;z;Roll;Pitch;Yaw];
end