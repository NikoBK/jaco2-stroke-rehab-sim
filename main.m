% Name:         main.m
% Created:      5/23/2022
% Authors: Group B302f
%
%
% INTRODUCTION:
% This is the main file for our 2nd semester project revolving
% around robotic manipulator programming and simulation. The purpose
% of this project is to code and simulate forward, inverse kinematics
% and trajectory planning for the Kinova Jaco2. This is done by hand, 
% by requirement from AAU guidelines. A few exceptions has been made
% such as allowance for tools that is made by mathworks, built in editor
% functions and various open source projects such as Peter Corke's toolbox.

% Clear console and cache
close all; clear; clc;

% Different frame configurations%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% NOTE: Please always set the RPY in degrees!
bowFrameA = frameCfg([400,      -200,     10,     0, 90, 0], true);
bowFrameB = frameCfg([338.69,   -108.24   10,     0, 90, 0], false);
bowFrameC = frameCfg([317.16,    0,       10,     0, 90, 0], false);
bowFrameD = frameCfg([338.69,    108.24,  10,     0, 90, 0], false);
bowFrameE = frameCfg([400,       200,     10,     0, 90, 0], true);

linFrameLeft   = frameCfg([350, 200,  10,   0, 90, 0],  true);
linFrameCenter = frameCfg([350, 0,    10,   0, 90, 0],  false);
linFrameRight  = frameCfg([350, -200, 10,   0, 90, 0],  true);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               USER INTERFACE FOR SIMULATION & ANIMATION            %

% Trajectory options:
bowTrajectory    = [bowFrameA, bowFrameB, bowFrameC, bowFrameD, bowFrameE]; % Bow shape trajectory
linearTrajectory = [linFrameRight, linFrameCenter, linFrameLeft]; % Linear  3-point trajectory

% Simulation settings:
velocity = 150;                         % mm/s
refreshRate = 0.01;                     % 10ms (100Hz. Jaco2 operating speed)
selectedTrajectory = bowTrajectory;     % Set this to the desired trajectory. See 'Trajectory Options' above to pick.
convertToRad = true;                    % Set this to 'true' if everything should be in radians.
showMainFrames = true;                  % Display the essential frames that shape the trajectory.
showTrajectoryFrames = false;           % Display the trajectory frames inbetween the essential frames.
plotJointAngles = true;                 % Plot the angles of each joint on the robot over elapsed simulation time
fminuncOptions = ...                    % Uses: options = optimoptions(@fminunc,'Display','off'); to mute fminunc functions
    optimoptions(@fminunc,'Display','off'); % ^

% Animation settings:
animate = true;               % True: Animates the robot on a trajectory
reverseAnimation = true;       % (JACO's POV) true: Starts left -> moves right. false: Starts right -> moves left 
clipName = 'animationClip';    % The name of your animation file. DO NOT INCLUDE .GIF/.MP4/etc..
exportGIF = true;              % Set this to true if you want the animation exported as a .GIF
exportMP4 = false;             % Set the GIF and Frames option to false and this to true if you want a .MP4
exportFrames = false;          % Set the others to false and this to true to get a frame per refreshRate.

% Fun stuff
crazyPartyMode = false; % I wonder what this does...

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   METHODS CALLED FROM MAIN                         %

% Start generating the trajectory..
generateTrajectory(selectedTrajectory, velocity, refreshRate, convertToRad, showMainFrames, showTrajectoryFrames, animate, reverseAnimation, clipName, exportGIF, exportMP4, exportFrames, plotJointAngles, fminuncOptions, crazyPartyMode);

% Test joint values via. forward kinematics..
% NOTE: Comment out functions unless you are using them.
%forwardKinematicsTest(-27.98315, 42.2000, -132.2000, 0, 0, 0);

% Test inverse kinematics equations:
%inverseKinematicsTest();

debugLog("Simulation complete!");
%% End of document
