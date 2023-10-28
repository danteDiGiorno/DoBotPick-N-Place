classdef DoBotMove
    % Class to control DoBot movement
    
    properties (Constant)
    end

    properties
    end
    
    methods
        function self = DoBotMove()
            % Define functions
            self.GetJointState();
            self.GetXYZ();
            self.MoveXYZ(x,y,z);
            self.CalcEERot(startPos,startJS,endPos,rotTarget);
            self.RotateEndEffector(base,rearArm,foreArm,eeT);
        end
    end

    methods (Static)        
        function [base,rearArm,foreArm,ee] = GetJointState()
            % Return current joint state

            % Joint state subscriber 
            jointStateSub = rossubscriber('/dobot_magician/joint_states');
            pause(2);
            currentJointState = jointStateSub.LatestMessage.Position;

            % Store joint states
            base = currentJointState(1);
            rearArm = currentJointState(2);
            foreArm = currentJointState(3);
            ee = currentJointState(4);

            % Print current joint states
            fprintf('Current joint states are [%d,%d,%d,%d]',base,rearArm,foreArm,ee);
        end
        
        function [x,y,z] = GetXYZ()
            % Return coordinates

            % End Effector subscriber
            eePose = rossubscriber('/dobot_magician/end_effector_poses');
            pause(2);
            currentEEPoseMsg = eePose.LatestMessage;

            % Store position of the end effector
            x = currentEEPoseMsg.Pose.Position.X;
            y = currentEEPoseMsg.Pose.Position.Y;
            z = currentEEPoseMsg.Pose.Position.Z;
        end
        
        function MoveXYZ(x,y,z,R,P,Y)

            % Set target position and rotation from input
            eePos = [x,y,z];
            eeRotation = [R,P,Y];
            
            % Target Pose Publisher
            [targetEEPub,targetEEMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
            
            % Populate position
            targetEEMsg.Position.X = eePos(1);
            targetEEMsg.Position.Y = eePos(2);
            targetEEMsg.Position.Z = eePos(3);
            
            % Populate rotation
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
            % Calculate required final end effector pose
            % Based on the total rotation required and the value of
            % rotation achieved by the base, with consideration of initial
            % joint state
            

            % Radius of start position
            startRad = sqrt(startPos(1)^2 + startPos(2)^2);

            % Radius of end position
            endRad = sqrt(endPos(1)^2+endPos(2)^2);

            % Distance between start position and end position
            dX = endPos(1) - startPos(1);
            dY = endPos(2) - startPos(2);
            dSE = sqrt(dX^2 + dY^2);

            % Base angle
            numerator = startRad^2 + endRad^2 - dSE^2;
            denominator = 2 * startRad * endRad;
            rotBase = acos(numerator/denominator);

            % Calculate target end effector rotation
            eeT = rotTarget - rotBase - startJS(4);
        end

        function RotateEndEffector(base,rearArm,foreArm,eeT)
            % Return joint state for new orientation

            % Define the target joint state
            jointTarget = [base,rearArm,foreArm,eeT];

            % Set target joint state publisher
            [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
            trajPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
            trajPoint.Positions = jointTarget;
            targetJointTrajMsg.Points = trajPoint;
            
            % Publish and print joint state
            send(targetJointTrajPub,targetJointTrajMsg);
            pause(2);
            fprintf('Rotating end effector to %d\n',eeT);
        end
    end
end
