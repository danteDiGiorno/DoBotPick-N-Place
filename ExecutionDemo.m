%% Execution Script
DoBotControl.MoveXYZ(X,Y,Z,0,0,Y);
pause(1);
DoBotControl.MoveXYZ(X,Y,Z,0,0,Y);
pause(1);
EndEffectorControl.Off();
pause(1);
DoBotControl.MoveXYZ(0.2635,-0.1314,0.1060,0,0,pi/8);
pause(1);

pause(1);
[base,rearArm,foreArm,ee] = DoBotControl.GetJointState();
pause(1);
DoBotControl.RotateEndEffector(base,rearArm,foreArm,1.2);
DoBotControl.MoveXYZ(0.3000,0.0120,0.0550,0,0,1.2);
pause(1);
EndEffectorControl.On();
pause(1);
DoBotControl.MoveXYZ(0.3000,0.0120,0.1060,0,0,1.2);
pause(1);
DoBotControl.MoveXYZ(0.2635,-0.1314,0.1060,0,0,pi/8);
pause(1);
DoBotControl.MoveXYZ(0.2875,-0.0615,-0.0430,0,0,pi/8);

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
    safetyStateMsg.Data = 3;
pause

send(safetyStatePublisher,safetyStateMsg);

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
    safetyStateMsg.Data = 2;

pause

send(safetyStatePublisher,safetyStateMsg);

pause(1);
DoBotControl.MoveXYZ(0.2635,-0.1314,0.1060,0,0,pi/8);

%%
% % joySub = rossubscriber('/joy');
% % 
% % buttons = joySub.LatestMessage.Buttons;
% % 
% % execute = 1;
% % 
% % while execute == 1
% % 
% % if buttons(2) == 1
% %     [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
% %     safetyStateMsg.Data = 3;
% %     send(safetyStatePublisher,safetyStateMsg);
% % end
% % 
% % if buttons(3) == 1
% %     [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
% %     safetyStateMsg.Data = 2;
% %     send(safetyStatePublisher,safetyStateMsg);
% % end
% % 
% % pause
% % 
% % end