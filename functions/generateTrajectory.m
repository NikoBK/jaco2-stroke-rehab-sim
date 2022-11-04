

%% Generates a trajectory.
function generateTrajectory(frames, velocity, refreshRate, convertToRad, showMainFrames, showTrajectoryFrames, animate, reverseAnimation, clipName, exportGIF, exportMP4, exportFrames, ...
                            plotThetas, fminuncOpt, crazyMode) 

    % Plots the trajectory from center out to the left and right.
    for f=1: length(frames)
        renderFrame(frames(f).xyzRPY, frames(f).isEndFrame,  convertToRad, showMainFrames);       
    end

    % Renders all the frames inbetween the 3 frames that marks the entire
    % trajectory.
    renderTrajectoryFrames(frames,  velocity, refreshRate, convertToRad, showTrajectoryFrames, animate, reverseAnimation, clipName, ...
        exportGIF, exportMP4, exportFrames, plotThetas, fminuncOpt, crazyMode);
end

%% Returns a distance between two 3d points a and b.
% distance data type: double
function [distance] = getDistance(a, b)
    x = abs(a(1) - b(1)); % The absolute value of difference in x
    y = abs(a(2) - b(2)); % The absolute value of difference in y
    z = abs(a(3) - b(3)); % The absolute value of difference in z

    distance = sqrt(x^2 + y^2 + z^2);
end

%% Plots a frame and colors it based on it being a endframe or not.
function renderFrame(frame, isEndFrame, convertToRad, showFrame)

    % Parsing frame parameters
    frameX = frame(1);
    frameY = frame(2);
    frameZ = frame(3);
    
    % If convertToRad is true the Roll Pitch & Yaw will be converted
    % from degrees to radians. If not they remain the same.
    if convertToRad == true
        frameR  = frame(4)  * pi/180;
        frameP  = frame(5)  * pi/180;
        frameYa = frame(6)  * pi/180;        
    elseif convertToRad == false
        frameR  = frame(4);
        frameP  = frame(5);
        frameYa = frame(6);
    end

    % Translational transform matrix for the frame position (X, Y, Z)
    framePosTrans = transl(frameX, frameY, frameZ);

    % Translational transform matrix for the frame orientation (Roll, Pitch, Yaw)
    frameOriTrans = rpy2tr(frameR, frameP, frameYa);

    % The line color. (r: Red, g: Green, b: Blue). Red by default
    % If isEndFrame is true it will set the color to green.
    color = 'r';
    if (isEndFrame) 
        color = 'g'; 
    end 

    length = 100; % Length of the coordinate frame arms (default: 1)
    thickness = 3; % Line thickness.
    axis = [200 550 -200 250 -150 250]; % Set dimensions of the MATLAB axes to A = [xmin xmax ymin ymax zmin zmax]

    % Plots the frame if showFrame is true.
    if showFrame == true
        trplot(framePosTrans * frameOriTrans, 'color', color, 'length', length, 'thick', thickness,'axis', axis)
        hold on;
    end
end


%% Renders all the trajectory frames inbetween via-points, color indexed with "blue".
function renderTrajectoryFrames(frames, velocity, refreshRate, convertToRad, showFrames, animate, reverseAnimation, clipName, exportGIF, exportMP4, exportFrames, plotThetas, fminuncOpt, crazyMode)    

    % Parses each frame in frames counting from the top index down or
    % bottom up based on whether reverseAnimation is true or not.
    if reverseAnimation == true
        f = length(frames); % DO NOT CHANGE THIS
        fPos = 1;
        while f > 1
            frame1 = frames(f); % Current frame
            frame2 = frames(f - 1); % Next frame to the current frame
    
            % Draw a single trajectory frame
            [thetas, time, totalDistance] = drawTrajectoryFrame(frame1, frame2, velocity, refreshRate, convertToRad, showFrames, fminuncOpt);    
            totalTime(1, fPos) = time;
            totalDistance(1, fPos) = totalDistance;
            
            fPos = fPos + 1;
            f = f - 1;
        end
    elseif reverseAnimation == false
        f = 1; % DO NOT CHANGE THIS
        while f < length(frames)
            frame1 = frames(f); % Current frame
            frame2 = frames(f + 1); % Next frame to the current frame

            % Draw a single trajectory frame
            [thetas, time, distance] = drawTrajectoryFrame(frame1, frame2, velocity, refreshRate, convertToRad, showFrames, fminuncOpt);
            totalTime(1, f) = time;
            totalDistance(1, f) = distance;

            f = f + 1;
        end
    end

    % If to animate the function will start rendering the robot running the
    % thetavalues found above via. inverse kinematics.
    if animate == true
        renderMovement(thetas, clipName, exportGIF, exportMP4, exportFrames, crazyMode);
    end
    if plotThetas == true
        thetaPlotter(thetas);
    end
end

%% Draws a individual frame and renders it into the window.
% Output: outputThetas (All theta values for the given position at the time
% and position of the given frames
function [outputThetas, time, distance] = drawTrajectoryFrame(frame1, frame2, velocity, refreshRate, convertToRad, showFrames, fminuncOpt)

        % Distance between the current frame and the next one.
        [distance] = getDistance(frame2.xyzRPY, frame1.xyzRPY);

        % The time it takes to move a distance with a given velocity
        time = distance / velocity;

        % The time escelated at every 10th millisecond until it reaches the
        % time variable's numeric value.
        t = 0: refreshRate: time;

        % Calculating the a values based on the current frame and the one
        % after the current one.
        a0 = frame1.xyzRPY;
        a1 = zeros(length(frame1.xyzRPY));
        a2 = 3 / time^2 * (frame2.xyzRPY - frame1.xyzRPY);
        a3 = -2 / time^3 * (frame2.xyzRPY - frame1.xyzRPY);
        
        % Creating the positions matrix.
        for i = 1: length(frame1.xyzRPY)
            positions(i,:) = a0(i) + a1(i) * t + a2(i) * t.^2 + a3(i) * t.^3;
        end

        for i = 2: (length(t) - 1)
            
            lineLength = 100; % Length of the coordinate frame arms (default: 1)
            thickness = 3; % Line thickness.
    
            % Iterating through the stored x, y and z positions.
            x = positions(1, i);
            y = positions(2, i);
            z = positions(3, i);
            
            % Convert the Roll Pitch & Yaw to radians if convertToRad is
            % true.
            if convertToRad == true
                R = positions(4, i) * pi/180;
                P = positions(5, i) * pi/180;
                Y = positions(6, i) * pi/180;
            elseif convertToRad == false
                R = positions(4, i);
                P = positions(5, i);
                Y = positions(6, i);
            end
            
            % Store xyzRPY in a vector:
            xyzRPYVec = [x, y, z, R, P, Y].';

            % Set the current frames theta values:
            
            thetaValues = fminunc(@(thetaValues) diffFrames(thetaValues, xyzRPYVec), [0 0 0 0 0 0], fminuncOpt);

            % Stores all rows in the thetaValues matrix
            outputThetas(:, i - 1) = thetaValues(:);

            % The translational transformation matrix for every position.
            T = transl(x, y, z) * rpy2tr(R, P, Y);
    
            % Plots the frame if showFrames is true
            if showFrames == true
                trplot(T, 'color', 'b', 'thick', thickness, 'length', lineLength)
            end
        end
end












