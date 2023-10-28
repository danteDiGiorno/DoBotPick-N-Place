%% Hardcode Movement Script
DoBotControl.MoveXYZ(0.3,-0.2,0.1,0,0,pi/4);
pause(1);
DoBotControl.MoveXYZ(0.1,0.1,0.2,0,0,0);
pause(1);
EndEffectorControl.Off();
pause(1);
DoBotControl.MoveXYZ(0.2,-0.1,0.1,0,0,pi/8);
pause(1);

pause(1);
[base,rearArm,foreArm,ee] = DoBotControl.GetJointState();
pause(1);
DoBotControl.RotateEndEffector(base,rearArm,foreArm,1.2);
DoBotControl.MoveXYZ(0.3,0.01,0.06,0,0,1.5);
pause(1);
EndEffectorControl.On();
pause(1);
DoBotControl.MoveXYZ(0.3,0.1,0.2,0,0,0);
pause(1);
DoBotControl.MoveXYZ(0.2,-0.1,0.1,0,0,pi/8);
pause(1);
DoBotControl.MoveXYZ(0.3,-0.05,-0.04,0,0,pi/8);

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
    safetyStateMsg.Data = 3;
pause

send(safetyStatePublisher,safetyStateMsg);

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
    safetyStateMsg.Data = 2;

pause

send(safetyStatePublisher,safetyStateMsg);

pause(1);
DoBotControl.MoveXYZ(0.3,-0.2,0.1,0,0,pi/8);