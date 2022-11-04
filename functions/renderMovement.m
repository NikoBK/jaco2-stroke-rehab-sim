
% Renders robotic movement
function renderMovement(thetaValues, clipName, exportGIF, exportMP4, exportFrames, crazyMode)

    alpha = [0; -90; 0; -90; 60; -60] * pi/180;
    a = [0; 0; 410; 0; 0; 0];
    d = [275.5; 0; -9.8; 250.08; 85.56; 42.78];
    thetas = [0; 42.2; -132.2; 0; 0; 0;] * pi/180;

    figure;
    robot = createRobot(double(alpha), double(a), double(d), double(thetas));
    robotCfg = homeConfiguration(robot);
    robotCfg(1).JointPosition = thetas(1);
    robotCfg(2).JointPosition = thetas(2);
    robotCfg(3).JointPosition = thetas(3);
    robotCfg(4).JointPosition = thetas(4);
    robotCfg(5).JointPosition = thetas(5);
    robotCfg(6).JointPosition = thetas(6);

    animationType = clipName + ".gif";
    if exportGIF == true
        animationType = clipName + ".gif";
    elseif exportMP4 == true
        animationType = clipName + ".mp4";
    elseif exportFrames == true
        animationType = clipName;
    end

    toRad = pi/180;
    if crazyMode == true
        toRad = 180/pi;
    end

    anim = Animate(animationType);
    for anim1 = 1: length(thetaValues)
        robotCfg(1).JointPosition = thetaValues(1, anim1) * toRad;
        robotCfg(2).JointPosition = thetaValues(2, anim1) * toRad;
        robotCfg(3).JointPosition = thetaValues(3, anim1) * toRad;
        robotCfg(4).JointPosition = thetaValues(4, anim1) * toRad;
        robotCfg(5).JointPosition = thetaValues(5, anim1) * toRad;
        robotCfg(6).JointPosition = thetaValues(6, anim1) * toRad;

        show(robot, robotCfg)
        anim.add();
    end
end

function robot = createRobot(alfa, a, d, theta)
robot = rigidBodyTree;
dhparams = [a, alfa,d, theta];

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
setFixedTransform(jnt1,dhparams(1,:),'mdh');
body1.Joint = jnt1;
addBody(robot,body1,'base')
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');

setFixedTransform(jnt2,dhparams(2,:),'mdh');
setFixedTransform(jnt3,dhparams(3,:),'mdh');
setFixedTransform(jnt4,dhparams(4,:),'mdh');
setFixedTransform(jnt5,dhparams(5,:),'mdh');
setFixedTransform(jnt6,dhparams(6,:),'mdh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')
addBody(robot,body6,'body5')

end