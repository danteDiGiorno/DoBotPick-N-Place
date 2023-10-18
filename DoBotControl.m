classdef DoBotControl
    % DOBOTCONTROL Class to control movement of the manipulator
    
    properties (Constant)
        % jointStateSubscriber = rossubscriber('/dobot/state');               % Subscriber to joint states of robot
        % jointService = rossvcclient('/dobot/joint_angles');                 % Service to control joint angles of robot
        % cartService = rossvcclient('/dobot/cartesian');                     % Service to control Cartesian coordinates of robot
    end

    properties
        % jointSubStatus = 0;                                                 % Status of joint state subscriber intialisation
        % jointSrvStatus = 0;                                                 % Status of joint state service initialisation
        % cartSrvStatus = 0;                                                  % Status of Cartesian service initialisation
    end
    
    methods
        function self = DoBotControl()
            % DOBOTCONTROL Define all functions required in the class
            self.GetJointState();
            self.GetXYZ();
            self.MoveXYZ(x,y,z);
            self.CalcEERot(startPos,startJS,endPos,rotTarget);
            self.RotateEndEffector(base,rearArm,foreArm,eeT);
        end
    end

    methods (Static)        
        function [base,rearArm,foreArm,ee] = GetJointState()
            % GETJOINTSTATE Return current joint state of DoBot

            % Initialise subscriber to joint states
            jointStateSubscriber = rossubscriber('/dobot_magician/joint_states');
            pause(2);
            currentJointState = jointStateSubscriber.LatestMessage.Position;

            % Store joint states
            base = currentJointState(1);
            rearArm = currentJointState(2);
            foreArm = currentJointState(3);
            ee = currentJointState(4);

            % Print to terminal
            fprintf('Current joint states are [%d,%d,%d,%d]',base,rearArm,foreArm,ee);
        end
        
        function [x,y,z] = GetXYZ()
            % GETCART Return current Cartesian coordinates of end effector

            % Initialise subscriber to end effector pose
            eePose = rossubscriber('/dobot_magician/end_effector_poses');
            pause(2);
            currentEEPoseMsg = eePose.LatestMessage;

            % Store position of the end effector
            x = currentEEPoseMsg.Pose.Position.X;
            y = currentEEPoseMsg.Pose.Position.Y;
            z = currentEEPoseMsg.Pose.Position.Z;

            % Store orientation of the end effector
            % currentEndEffectorQuat = [currentEndEffectorPoseMsg.Pose.Orientation.W,
            %                           currentEndEffectorPoseMsg.Pose.Orientation.X,
            %                           currentEndEffectorPoseMsg.Pose.Orientation.Y,
            %                           currentEndEffectorPoseMsg.Pose.Orientation.Z]
            % Convert from quaternion to euler
            % [roll,pitch,yaw] = quat2eul(currentEndEffectorQuat);
        end
        
        function MoveXYZ(x,y,z,R,P,Y)
            % MOVECART Move to an end effector pose in Cartesian coordinates
            % Function accepts [x,y,z] point as target with [R,P,Y]
            % rotation.
            % Parameters:
                % [IN] x = x-coordinate in DoBot's frame
                % [IN] y = y-coordinate in DoBot's frame
                % [IN] z = z-coordinate in DoBot's frame
                % [IN] R = roll value in DoBot's frame
                % [IN] P = pitch value in DoBot's frame
                % [IN] Y = yaw value in DoBot's frame

            % Define target position and rotation from input parameters
            eePos = [x,y,z];
            eeRotation = [R,P,Y];
            
            % Initialise publisher to target pose
            [targetEEPub,targetEEMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
            
            % Populate position message
            targetEEMsg.Position.X = eePos(1);
            targetEEMsg.Position.Y = eePos(2);
            targetEEMsg.Position.Z = eePos(3);
            
            % Populate rotation message
                % @NOTE: Convert R,P,Y to quaternion
            qua = eul2quat(eeRotation);
            targetEEMsg.Orientation.W = qua(1);
            targetEEMsg.Orientation.X = qua(2);
            targetEEMsg.Orientation.Y = qua(3);
            targetEEMsg.Orientation.Z = qua(4);
            
            % Send the position and rotation message via the publisher
            send(targetEEPub,targetEEMsg)
            pause(2);
            fprintf('Moving to [%d,%d,%d]\n',x,y,z);
        end

        function eeT = CalcEERot(startPos,startJS,endPos,rotTarget)
            % CALCEEREQPOSE Calculate required final end effector pose
            % Based on the total rotation required and the value of
            % rotation achieved by the base, with consideration of initial
            % joint state
            
            % Load values from inputs |||| Not needed??? Duplication???
            % startPos = [x0,y0,z0];                                          % Starting position (collection point)
            % endPos = [x1,y1,z1];                                            % Target position (deposition point)
            % startJs = [base0,rear0,fore0,ee0];                              % Starting joint state (collection point)

            % Calculate radius of startPos
            startRad = sqrt(startPos(1)^2 + startPos(2)^2);

            % Calculate radius of endPos
            endRad = sqrt(endPos(1)^2+endPos(2)^2);

            % Calculate distance between startPos and endPos
            dX = endPos(1) - startPos(1);
            dY = endPos(2) - startPos(2);
            dSE = sqrt(dX^2 + dY^2);

            % Calculate angle inscribed by the base, using cosine rule
            numerator = startRad^2 + endRad^2 - dSE^2;
            denominator = 2 * startRad * endRad;
            rotBase = acos(numerator/denominator);

            % Calculate target end effector rotation
            % EE rotation = final orientation - base rotation - initial EE rotation
            eeT = rotTarget - rotBase - startJS(4);
        end

        function RotateEndEffector(base,rearArm,foreArm,eeT)
            % ROTATEENDEFFECTOR Return joint state for piece orientation

            % Define the target joint state
            jointTarget = [base,rearArm,foreArm,eeT];

            % Initialise the target joint state publisher
            [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
            trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
            trajectoryPoint.Positions = jointTarget;
            targetJointTrajMsg.Points = trajectoryPoint;
            
            % Send the target joint state via the publisher
            send(targetJointTrajPub,targetJointTrajMsg);
            pause(2);
            fprintf('Rotating end effector to %d\n',eeT);
        end
    end
end