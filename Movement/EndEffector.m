classdef EndEffector
    % Class to control end effector
    
    properties (Constant)
    end

    properties
    end
    
    methods
        function self = EndEffector()
            % Define functions
            self.ToolState();
            self.On();
            self.Off();
        end
    end

    methods (Static)        
        function currentToolState = ToolState()
            % Check if suction is activated

            % Initialise subscriber and output
            toolStateSub = rossubscriber('/dobot_magician/tool_state');
            pause(2);
            currentToolState = toolStateSub.LatestMessage.Data;
        end

        function On()
            
            % Check suction and print
            currentToolState = EndEffectorControl.ToolState();
            fprintf('Current suction state: %d\n',currentToolState);
            
            % Break if already on
            while currentToolState == 1
                break
            end

            % If it off, turn on
            while currentToolState ~= 1
                [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
                toolStateMsg.Data = [1 0];
                send(toolStatePub,toolStateMsg);
                break
            end

            pause(2);
            currentToolState = EndEffectorControl.ToolState();
            
            fprintf('New suction state: %d\n',currentToolState);
        end

        function Off()
            % Turn suction on
            
            [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
            toolStateMsg.Data = [1 1];
            send(toolStatePub,toolStateMsg);

            pause(2);
            currentToolState = EndEffectorControl.ToolState();

            fprintf('New suction state: %d\n',currentToolState);
        end
    end
end
