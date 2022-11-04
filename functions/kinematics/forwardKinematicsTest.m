
%% Simulates forward kinematics using a set of theta values.
function forwardKinematics(theta1, theta2, theta3, theta4, theta5, theta6)
    
    % Setting up the robotic manipulator configurations
    jaco = loadrobot("kinovaJacoJ2N6S200"); % The robot framework for the Kinova Jaco 2.
    jacoConfig = homeConfiguration(jaco);     % The home configuration for the jaco.
    jacoNumJoints = numel(jacoConfig); % The number of joints on the Kinova Jaco 2.
    jacoEndDef = "j2n6s200_end_effector";
    joint6 = 'j2n6s200_link_6';
    joint4 = 'j2n6s200_link_4'; 
    jacoBase = 'root'; % String representation of the base for the Kinova Jaco 2.
    

    %% Definition for the individual joints in the Kinova Jaco 2
    joint1Angle = double(theta1 * (pi / 180));
    joint2Angle = double(theta2 * (pi / 180));
    joint3Angle = double(theta3 * (pi / 180));
    joint4Angle = double(theta4 * (pi / 180));
    joint5Angle = double(theta5 * (pi / 180));
    joint6Angle = double(theta6 * (pi / 180));
    gribber1Angle = 0;
    gribber2Angle = 0;

    %% Modifying the default Jaco configuration to our joint angles defined above
    jacoConfig(1).JointPosition = -joint1Angle; 
    jacoConfig(2).JointPosition = -joint2Angle + 90 * (pi / 180);
    jacoConfig(3).JointPosition =  joint3Angle - 90 * (pi / 180);
    jacoConfig(4).JointPosition = -joint4Angle;
    jacoConfig(5).JointPosition = -joint5Angle;
    jacoConfig(6).JointPosition = -joint6Angle;
    jacoConfig(7).JointPosition = gribber1Angle;
    jacoConfig(8).JointPosition = gribber2Angle;

    %% Transformation matrix:
    cfgTrans = getTransform(jaco, jacoConfig, joint6, jacoBase);
    cfgTransT04 = getTransform(jaco, jacoConfig, joint4, jacoBase);

    %% DH Parameters:
    alpha = [0; -90; 0; -90; 60; -60 ] * pi / 180;
    a = [ 0; 0; 410; 0; 0; 0 ];
    d = [ 275.5; 0; -9.8; 250.08; 85.56; 42.78 ];
    thetas = [ double(theta1); double(theta2); double(theta3); double(theta4); double(theta5); double(theta6) ] * pi/180;

    DH = [alpha a d thetas];

    %% Setting up the transformation matrices for each joint.
    T01 = transformation(alpha(1), a(1), d(1), thetas(1));
    T12 = transformation(alpha(2), a(2), d(2), thetas(2));
    T23 = transformation(alpha(3), a(3), d(3), thetas(3));
    T34 = transformation(alpha(4), a(4), d(4), thetas(4));
    T45 = transformation(alpha(5), a(5), d(5), thetas(5));
    T56 = transformation(alpha(6), a(6), d(6), thetas(6));

    %% Joint 0 - 3, 4 & 6 matrices.
    T06 = double(T01 * T12 * T23 * T34 * T45 * T56);
    T04 = double(T01 * T12 * T23 * T34);
    T03 = double(T01 * T12 * T23);
    T02 = double(T01 * T12);

    %% Render the robot.
    figure;
    robot = createRobot(double(alpha), double(a)/1000, double(d)/1000, double(thetas));
    renderCfg = homeConfiguration(robot);
    renderCfg(1).JointPosition = thetas(1);
    renderCfg(2).JointPosition = thetas(2);
    renderCfg(3).JointPosition = thetas(3);
    renderCfg(4).JointPosition = thetas(4);
    renderCfg(5).JointPosition = thetas(5);
    renderCfg(6).JointPosition = thetas(6);
    show(robot, renderCfg);

    generatedConfigFigure = figure('name', 'Generated Configuration');
    show(jaco, jacoConfig);
    movegui(generatedConfigFigure, 'northeast');
    
    %% Log the position of the end efector based on the joint angles
    debugLog(T06);
    debugLog(tr2rpy(T06, 'xyz'));
end