%% Function 1: Finding difference between the Desired Location and Current Location
function e = diffFrames(q,P)

% We get the xyz rpy position, by giving the current theta values (q) in radians
% findXYZRPY(q) is a function that finds xyz and rpy from the given theta
% value, by using the T06 and the equations for roll pitch yaw
v = findXYZRPY(q*pi/180);

% We change the rpy values back into degrees and save them so we can
% compare them to the desired orientation which is in degrees
v(4) = v(4)*180/pi;
v(5) = v(5)*180/pi;
v(6) = v(6)*180/pi;

% We calculate the error of the current orientation by comparing it to the
% desired orientation

% v[x y z r p y] current position - P[x y z r p y] desired location
error = v-P;

% If any of the three abselout value of the rpy error is greater than 180 degrees
% Then: * We save the rpy errors as rpy_error
%       * For the values that are greater than 180 degrees we subtract 360
%         degrees to see if the problem was that the joint was rotated a
%         full rotation too much and save it
%       * Save the new value of rpy_error as the error
if any(abs(error(4:end))>180)
    rpy_error = abs(error(4:end));
    rpy_error(rpy_error>180) = rpy_error(rpy_error>180)-360;
    error(4:end) = rpy_error;
end
e = rms(error);
% End of the function
end
