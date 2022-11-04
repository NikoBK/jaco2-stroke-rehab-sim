
%% Plots a set of theta values for a specific trajectory over time for each joint.
function thetaPlotter(thetaSets)
    figure; % Create a new figure for this plot
    
    line_color = ['b' 'g' 'y' 'c' 'm' 'r']; % Color indications for seperating function lines
    t = 1 : length(thetaSets); % data count

    length(thetaSets)
    % Check the data length on the theta sets
    if length(thetaSets) == 132
        scaleFactor = 0.04485523990757;
    elseif length(thetaSets) == 72
        scaleFactor = 0.02442168102571;
    end

    xValue = t / scaleFactor; % Apply the scale factor to the data count to time in MS.
    ca = cell(1, length(line_color)); % Generate cells

    % Initialising theta vectors for each joint
    joint1Angles = zeros(length(thetaSets), 1);
    joint2Angles = zeros(length(thetaSets), 1);
    joint3Angles = zeros(length(thetaSets), 1);
    joint4Angles = zeros(length(thetaSets), 1);
    joint5Angles = zeros(length(thetaSets), 1);
    joint6Angles = zeros(length(thetaSets), 1);

    % Load the theta values into their relative vectors
    for l = 1 : length(thetaSets)
        joint1Angles(l) = thetaSets(1, l) * pi/180;
        joint2Angles(l) = thetaSets(2, l) * pi/180;
        joint3Angles(l) = thetaSets(3, l) * pi/180;
        joint4Angles(l) = thetaSets(4, l) * pi/180;
        joint5Angles(l) = thetaSets(5, l) * pi/180;
        joint6Angles(l) = thetaSets(6, l) * pi/180;
    end

    % Start drawing the functions
    for k = 1 : length(line_color)
      ca{k} = sprintf('joint %d', k);
      hold on

      % Get new values.    
      yValue = thetaSets(k, :);%theta1Angles(t);
      plot(xValue, yValue,'-', 'Color', line_color(k), 'LineWidth', 2)
      grid on;
    end

    % Misc plotting options..
    title('Joint angles over time','FontSize',16)
    ylabel('Joint angles (radians)')
    xlabel('Time (ms)')
    legend(ca, 'Location', 'southwest')
end
