classdef Test_UI < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        PointsEditField                 matlab.ui.control.NumericEditField
        PointsEditFieldLabel            matlab.ui.control.Label
        MotionPanel                     matlab.ui.container.Panel
        SetPlatformSpeedSlider          matlab.ui.control.Slider
        SetPlatformSpeedSliderLabel     matlab.ui.control.Label
        PathPlanningCheckBox            matlab.ui.control.CheckBox
        MovePlatformReset               matlab.ui.control.Button
        MovePlatformPtoP                matlab.ui.control.Button
        MovePlatform                    matlab.ui.control.Button
        SaveandSequenceSetpointsPanel   matlab.ui.container.Panel
        MovetoP6Button                  matlab.ui.control.Button
        MovetoP5Button                  matlab.ui.control.Button
        MovetoP4Button                  matlab.ui.control.Button
        MovetoP3Button                  matlab.ui.control.Button
        MovetoP2Button                  matlab.ui.control.Button
        MovetoP1Button                  matlab.ui.control.Button
        ClearP6Button                   matlab.ui.control.Button
        ClearP5Button                   matlab.ui.control.Button
        ClearP4Button                   matlab.ui.control.Button
        ClearP3Button                   matlab.ui.control.Button
        ClearP2Button                   matlab.ui.control.Button
        ClearP1Button                   matlab.ui.control.Button
        P1CheckBox                      matlab.ui.control.CheckBox
        P6CheckBox                      matlab.ui.control.CheckBox
        P5CheckBox                      matlab.ui.control.CheckBox
        P4CheckBox                      matlab.ui.control.CheckBox
        P3CheckBox                      matlab.ui.control.CheckBox
        P2CheckBox                      matlab.ui.control.CheckBox
        SaveP6Button                    matlab.ui.control.Button
        SaveP5Button                    matlab.ui.control.Button
        SaveP4Button                    matlab.ui.control.Button
        SaveP3Button                    matlab.ui.control.Button
        SaveP2Button                    matlab.ui.control.Button
        SaveP1Button                    matlab.ui.control.Button
        Logo_4                          matlab.ui.control.Button
        SerialPanel                     matlab.ui.container.Panel
        COMPortEditField                matlab.ui.control.NumericEditField
        COMPortEditFieldLabel           matlab.ui.control.Label
        ErrorMessage                    matlab.ui.control.TextArea
        SerialSwitch                    matlab.ui.control.Switch
        SerialLabel                     matlab.ui.control.Label
        CalibrationButton               matlab.ui.control.Button
        StewartPlatformGUIButtonGroup   matlab.ui.container.ButtonGroup
        NoahTannerScottBrownJacksonNorbegJacksonStanwickLabel  matlab.ui.control.Label
        W24SeniorProjectFall2023Label   matlab.ui.control.Label
        Logo_2                          matlab.ui.control.Button
        CalculatedActuatorLengthsPanel  matlab.ui.container.Panel
        PotcountsLabel_6                matlab.ui.control.Label
        PotcountsLabel_5                matlab.ui.control.Label
        PotcountsLabel_4                matlab.ui.control.Label
        PotcountsLabel_3                matlab.ui.control.Label
        PotcountsLabel_2                matlab.ui.control.Label
        PotcountsLabel                  matlab.ui.control.Label
        Actuator_6_Pot_Set              matlab.ui.control.NumericEditField
        Actuator_5_Pot_Set              matlab.ui.control.NumericEditField
        Actuator_1_Pot_Set              matlab.ui.control.NumericEditField
        Actuator_4_Pot_Set              matlab.ui.control.NumericEditField
        Actuator_3_Pot_Set              matlab.ui.control.NumericEditField
        Actuator_2_Pot_Set              matlab.ui.control.NumericEditField
        inchesLabel_4                   matlab.ui.control.Label
        inchesLabel_6                   matlab.ui.control.Label
        inchesLabel_5                   matlab.ui.control.Label
        inchesLabel_9                   matlab.ui.control.Label
        inchesLabel_8                   matlab.ui.control.Label
        inchesLabel_7                   matlab.ui.control.Label
        Actuator_6_Length_Set           matlab.ui.control.NumericEditField
        Actuator6Label_3                matlab.ui.control.Label
        Actuator_5_Length_Set           matlab.ui.control.NumericEditField
        Actuator5Label_3                matlab.ui.control.Label
        Actuator_4_Length_Set           matlab.ui.control.NumericEditField
        Actuator4Label_3                matlab.ui.control.Label
        Actuator_3_Length_Set           matlab.ui.control.NumericEditField
        Actuator3Label_3                matlab.ui.control.Label
        Actuator_2_Length_Set           matlab.ui.control.NumericEditField
        Actuator2Label_3                matlab.ui.control.Label
        Actuator_1_Length_Set           matlab.ui.control.NumericEditField
        Actuator1Label_3                matlab.ui.control.Label
        TopJointLocations63Panel        matlab.ui.container.Panel
        degreesLabel_20                 matlab.ui.control.Label
        degreesLabel_19                 matlab.ui.control.Label
        degreesLabel_21                 matlab.ui.control.Label
        Top_Act_5_6                     matlab.ui.control.NumericEditField
        Actuators56Label                matlab.ui.control.Label
        Top_Act_3_4                     matlab.ui.control.NumericEditField
        Actuators34Label                matlab.ui.control.Label
        Top_Act_1_2                     matlab.ui.control.NumericEditField
        Actuators12Label                matlab.ui.control.Label
        BaseJointLocationsPanel         matlab.ui.container.Panel
        degreesLabel_9                  matlab.ui.control.Label
        degreesLabel_8                  matlab.ui.control.Label
        degreesLabel_7                  matlab.ui.control.Label
        degreesLabel_6                  matlab.ui.control.Label
        degreesLabel_5                  matlab.ui.control.Label
        degreesLabel_4                  matlab.ui.control.Label
        Base_Act_6                      matlab.ui.control.NumericEditField
        Actuator6Label                  matlab.ui.control.Label
        Base_Act_5                      matlab.ui.control.NumericEditField
        Actuator5Label                  matlab.ui.control.Label
        Base_Act_4                      matlab.ui.control.NumericEditField
        Actuator4Label                  matlab.ui.control.Label
        Base_Act_3                      matlab.ui.control.NumericEditField
        Actuator3Label                  matlab.ui.control.Label
        Base_Act_2                      matlab.ui.control.NumericEditField
        Actuator2Label                  matlab.ui.control.Label
        Base_Act_1                      matlab.ui.control.NumericEditField
        Actuator1Label                  matlab.ui.control.Label
        PositionOrientationPanel        matlab.ui.container.Panel
        CalcJointButton                 matlab.ui.control.Button
        degreesLabel_3                  matlab.ui.control.Label
        degreesLabel_2                  matlab.ui.control.Label
        degreesLabel                    matlab.ui.control.Label
        PsiInput                        matlab.ui.control.NumericEditField
        Label_7                         matlab.ui.control.Label
        PhiInput                        matlab.ui.control.NumericEditField
        Label_6                         matlab.ui.control.Label
        ThetaInput                      matlab.ui.control.NumericEditField
        Label_5                         matlab.ui.control.Label
        inchesLabel_3                   matlab.ui.control.Label
        inchesLabel_2                   matlab.ui.control.Label
        inchesLabel                     matlab.ui.control.Label
        PzInput                         matlab.ui.control.NumericEditField
        PzLabel                         matlab.ui.control.Label
        PyInput                         matlab.ui.control.NumericEditField
        PyLabel                         matlab.ui.control.Label
        PxInput                         matlab.ui.control.NumericEditField
        PxLabel                         matlab.ui.control.Label
        Label                           matlab.ui.control.Label
    end

properties (Access = private)
    ser;
    Actuator_Current_Pos = [0;0;0;0;0;0];
    serial_state;
    P1=zeros(1,6);
    P2=zeros(1,6);
    P3=zeros(1,6);
    P4=zeros(1,6);
    P5=zeros(1,6);
    P6=zeros(1,6); 
    MoveComplete;
    
end    
    

methods (Access = private)
        
    function MoveToPoint(app,P)
    
        speed = round(app.SetPlatformSpeedSlider.Value);
        
        
        
        if app.PathPlanningCheckBox.Value == 1         
            j=1;
            printOnce = true;
            move_points = app.PointsEditField.Value;
            pos_array = zeros(length(P), move_points);
            for i = 1:length(P)
                pos_array(i,:) = round(linspace(app.Actuator_Current_Pos(i),P(i),move_points));
            end
            
            if app.serial_state == 1
                while j <= move_points
                    if (printOnce == true)
                       fprintf(app.ser, [strjoin({num2str(pos_array(1,j)), num2str(pos_array(2,j)), num2str(pos_array(3,j)), num2str(pos_array(4,j)), num2str(pos_array(5,j)), num2str(pos_array(6,j)), num2str(speed)}), '\n']);
                       printOnce = false;
                    end
                    MoveCheck = fread(app.ser,1);
                    if MoveCheck == 'D'
                        j = j +1;
                        printOnce = true; %intermediate move is done, send the next position
                    end    
                end
                app.ErrorMessage.Value = 'Move Complete';
            end
        end
        if app.PathPlanningCheckBox.Value == 0 && app.serial_state == 1
            fprintf(app.ser, [strjoin({num2str(P(1)), num2str(P(2)), num2str(P(3)), num2str(P(4)), num2str(P(5)), num2str(P(6)), num2str(speed)}), '\n']);
            MoveCheck = fread(app.ser,1);
            if MoveCheck == 'D'
                app.ErrorMessage.Value = 'Move Complete';
            end
        end
        
        app.ErrorMessage.Visible = 'on';              
        for k = 1:length(P)
            app.Actuator_Current_Pos(k) = P(1,k);
        end
    end
end

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.ErrorMessage.Visible = 'on';
            app.SerialSwitch.Enable = 'off';
            app.TopJointLocations63Panel.Visible = 'on';
%             app.t=timer();
%             app.t.Period = 1;
%             app.t.ExecutionMode = 'fixedRate';
%             app.t.TimerFcn = @
            
        
        end

        % Selection changed function: StewartPlatformGUIButtonGroup
        function StewartPlatformGUIButtonGroupSelectionChanged(app, event)

            if(app.GenericConfigurationButton.Value)
                    app.TopJointLocations63Panel.Visible = 'on';
                    app.BaseJointLocationsPanel.Position = [15,255,197,171];
            end
            if(app.ConfigurationButton_2.Value)
                    app.TopJointLocations63Panel.Visible = 'on';
                    app.BaseJointLocationsPanel.Position = [15,324,197,171];
            end
                        
        end

        % Button pushed function: CalibrationButton
        function CalibrationButtonPushed(app, event)
            app.CalcJointButton.Enable = 'off';
            app.ErrorMessage.Value = 'Calibrating...';
          
            
            %% Serial configuration
            % ComPortNumber   = num2str(app.COMPortEditField.Value);
            PORT            = "/dev/tty.usbmodem14101";
            BAUD_RATE       = 115200;
            
            % Establish a new serial connection (clear previous connections)
            % delete(instrfindall);
            app.ser = serialport( PORT, BAUD_RATE );

            fopen(app.ser);
            app.SerialSwitch.Value = 'On';
            app.serial_state = 1;
            pause(14);
            CalibrationState = fread(app.ser,1);
            if CalibrationState == 1
                app.CalibrationButton.Text = 'Calibrated';
                app.CalibrationButton.Enable = 'off';
                app.CalcJointButton.Enable = 'on';
                app.ErrorMessage.Value = '';
                app.SerialSwitch.Enable = 'off';
                app.MovePlatformReset.Enable = 'on';
            elseif CalibrationState == 0
                app.ErrorMessage.Value = 'Calibration Failed';
            else
                app.ErrorMessage.Value = 'Serial Compare Failed';
            end
        end

        % Button pushed function: CalcJointButton
        function CalcJointButtonPushed(app, event)
            app.ErrorMessage.Value = '';
            app.ErrorMessage.BackgroundColor = 'w';
            % SET UP VARIABLES
            %------------------------------------------------------------
            a_length = 17/2; %Length [inches] of a vector on base
            b_length = 13/2; %Length [inches] of a vector on top
            ball_joint_height = 1.032; %Height from magnetic base to sphere center [inches]
            ball_diameter = 18/25.4; %[inches]
            platform_thiccness = 0.25; %[inches]
            joint_joint_offset = 15.61; %[inches]
            %need to verify both dimensions
            angleOffset_6_3 =3.5; %[degrees]
            a_i = zeros(4,6);
            b_i = zeros(4,6);
            b_rot = zeros(4,6);
            d_vector = zeros(4,6);
            d_length = zeros(1,6);
            
            %Take in 
            
            Px = app.PxInput.Value;
            Py = app.PyInput.Value;
            Pz = app.PzInput.Value;

            %Remove offset from base top to ball center
            Pz = Pz - ball_joint_height - ball_diameter/2 - platform_thiccness; 
                        
            P = [Px;Py;Pz;1];
            
            % SET UP A and B Matrices
            %------------------------------------------------------------
            
            a_i(1,1) = a_length*cosd(app.Base_Act_1.Value); a_i(2,1) = a_length*sind(app.Base_Act_1.Value);
            a_i(1,2) = a_length*cosd(app.Base_Act_2.Value); a_i(2,2) = a_length*sind(app.Base_Act_2.Value);
            a_i(1,3) = a_length*cosd(app.Base_Act_3.Value); a_i(2,3) = a_length*sind(app.Base_Act_3.Value);
            a_i(1,4) = a_length*cosd(app.Base_Act_4.Value); a_i(2,4) = a_length*sind(app.Base_Act_4.Value);
            a_i(1,5) = a_length*cosd(app.Base_Act_5.Value); a_i(2,5) = a_length*sind(app.Base_Act_5.Value);
            a_i(1,6) = a_length*cosd(app.Base_Act_6.Value); a_i(2,6) = a_length*sind(app.Base_Act_6.Value);
            a_i(3,:) = 0;
            a_i(4,:) = 1;
            
            if(app.GenericConfigurationButton.Value)
 
                b_i(1,1) = b_length*cosd(app.Top_Act_1.Value); b_i(2,1) = b_length*sind(app.Top_Act_1.Value);
                b_i(1,2) = b_length*cosd(app.Top_Act_2.Value); b_i(2,2) = b_length*sind(app.Top_Act_2.Value);
                b_i(1,3) = b_length*cosd(app.Top_Act_3.Value); b_i(2,3) = b_length*sind(app.Top_Act_3.Value);
                b_i(1,4) = b_length*cosd(app.Top_Act_4.Value); b_i(2,4) = b_length*sind(app.Top_Act_4.Value);
                b_i(1,5) = b_length*cosd(app.Top_Act_5.Value); b_i(2,5) = b_length*sind(app.Top_Act_5.Value);
                b_i(1,6) = b_length*cosd(app.Top_Act_6.Value); b_i(2,6) = b_length*sind(app.Top_Act_6.Value);
                
            end
                
            if(app.ConfigurationButton_2.Value)
                
                b_i(1,1) = b_length*cosd(app.Top_Act_1_2.Value - angleOffset_6_3); b_i(2,1) = b_length*sind(app.Top_Act_1_2.Value - angleOffset_6_3);
                b_i(1,2) = b_length*cosd(app.Top_Act_1_2.Value + angleOffset_6_3); b_i(2,2) = b_length*sind(app.Top_Act_1_2.Value + angleOffset_6_3);
                b_i(1,3) = b_length*cosd(app.Top_Act_3_4.Value - angleOffset_6_3); b_i(2,3) = b_length*sind(app.Top_Act_3_4.Value - angleOffset_6_3);
                b_i(1,4) = b_length*cosd(app.Top_Act_3_4.Value + angleOffset_6_3); b_i(2,4) = b_length*sind(app.Top_Act_3_4.Value + angleOffset_6_3);
                b_i(1,5) = b_length*cosd(app.Top_Act_5_6.Value - angleOffset_6_3); b_i(2,5) = b_length*sind(app.Top_Act_5_6.Value - angleOffset_6_3);
                b_i(1,6) = b_length*cosd(app.Top_Act_5_6.Value + angleOffset_6_3); b_i(2,6) = b_length*sind(app.Top_Act_5_6.Value + angleOffset_6_3); 
                
            end
            
            b_i(3,:) = 0;
            b_i(4,:) = 1;
            
            % Rotate the B matrix and calculate the lengths of actuators
            for i = 1:6
                b_rot(:,i)=stewartrot(-app.ThetaInput.Value, -app.PhiInput.Value, -app.PsiInput.Value, b_i(:,i));
                d_vector(:,i)  = P(:,1) + b_rot(:,i) - a_i(:,i);
                d_length(1,i) = sqrt(d_vector(1,i)^2 + d_vector(2,i)^2 + d_vector(3,i)^2);
            end
            
            %Remove physical constants so length is on scale from 0 inches to 8 inches
            d_length = d_length - joint_joint_offset;
            
            app.Actuator_1_Length_Set.Value = d_length(1,1);
            app.Actuator_2_Length_Set.Value = d_length(1,2);
            app.Actuator_3_Length_Set.Value = d_length(1,3);
            app.Actuator_4_Length_Set.Value = d_length(1,4);
            app.Actuator_5_Length_Set.Value = d_length(1,5);
            app.Actuator_6_Length_Set.Value = d_length(1,6);
            
            if max(d_length) > 8 || min(d_length) < 0
                app.ErrorMessage.Visible = 'on';
                app.MovePlatform.Enable = 'off';
                app.ErrorMessage.Value = 'CALCULATED LENGTH(S) OUT OF RANGE';
                app.ErrorMessage.BackgroundColor = 'r';
                app.ErrorMessage.FontWeight = 'bold';
            else
                app.MovePlatform.Enable = 'on';
                app.SaveP1Button.Enable = 'on';
                app.SaveP2Button.Enable = 'on';
                app.SaveP3Button.Enable = 'on';
                app.SaveP4Button.Enable = 'on';
                app.SaveP5Button.Enable = 'on';
                app.SaveP6Button.Enable = 'on';
%                 app.ErrorMessage.Visible = 'off';
            end
            
            output_length = round(interp1([-8 16],[-1024 2048],d_length));
            app.Actuator_1_Pot_Set.Value = output_length(1,1);
            app.Actuator_2_Pot_Set.Value = output_length(1,2);
            app.Actuator_3_Pot_Set.Value = output_length(1,3);
            app.Actuator_4_Pot_Set.Value = output_length(1,4);
            app.Actuator_5_Pot_Set.Value = output_length(1,5);
            app.Actuator_6_Pot_Set.Value = output_length(1,6);

        end

        % Button pushed function: MovePlatform
        function MovePlatformButtonPushed(app, event)
            output_length(1) = app.Actuator_1_Pot_Set.Value;
            output_length(2) = app.Actuator_2_Pot_Set.Value;
            output_length(3) = app.Actuator_3_Pot_Set.Value;
            output_length(4) = app.Actuator_4_Pot_Set.Value;
            output_length(5) = app.Actuator_5_Pot_Set.Value;
            output_length(6) = app.Actuator_6_Pot_Set.Value;
           
            
            MoveToPoint(app, output_length)
        end

        % Value changed function: SerialSwitch
        function SerialSwitchValueChanged(app, event)
            %value = app.SerialSwitch.Value;
            switch app.SerialSwitch.Value
                case 'On'
                fopen(app.ser);
                app.serial_state = 1;
                case 'Off'
                app.serial_state = 0;
                fclose(app.ser);
            end
            
        end

        % Callback function
        function Logo_3ButtonPushed(app, event)
            web('https://www.calpoly.edu/');
        end

        % Button pushed function: Logo_2
        function Logo_2ButtonPushed(app, event)
            web('https://me.calpoly.edu/');
        end

        % Button pushed function: Logo_4
        function Logo_4ButtonPushed(app, event)
            web('http://heli-cal.com/');
        end

        % Callback function
        function LogoButtonPushed(app, event)
            web('https://www.progressiveautomations.com/');
        end

        % Button pushed function: SaveP1Button
        function SaveP1ButtonPushed(app, event)
            app.P1(1) = app.Actuator_1_Pot_Set.Value;
            app.P1(2) = app.Actuator_2_Pot_Set.Value;
            app.P1(3) = app.Actuator_3_Pot_Set.Value;
            app.P1(4) = app.Actuator_4_Pot_Set.Value;
            app.P1(5) = app.Actuator_5_Pot_Set.Value;
            app.P1(6) = app.Actuator_6_Pot_Set.Value; 
            
            app.P1CheckBox.Value = 1;
            app.MovePlatformPtoP.Enable = 'on';
            app.ClearP1Button.Enable = 'on';
            app.MovetoP1Button.Enable = 'on';
        end

        % Button pushed function: SaveP2Button
        function SaveP2ButtonPushed(app, event)
            app.P2(1) = app.Actuator_1_Pot_Set.Value;
            app.P2(2) = app.Actuator_2_Pot_Set.Value;
            app.P2(3) = app.Actuator_3_Pot_Set.Value;
            app.P2(4) = app.Actuator_4_Pot_Set.Value;
            app.P2(5) = app.Actuator_5_Pot_Set.Value;
            app.P2(6) = app.Actuator_6_Pot_Set.Value; 
            
      
            app.P2CheckBox.Value = 1;
            app.MovePlatformPtoP.Enable = 'on';
            app.ClearP2Button.Enable = 'on';
            app.MovetoP2Button.Enable = 'on';
        end

        % Button pushed function: SaveP3Button
        function SaveP3ButtonPushed(app, event)
            app.P3(1) = app.Actuator_1_Pot_Set.Value;
            app.P3(2) = app.Actuator_2_Pot_Set.Value;
            app.P3(3) = app.Actuator_3_Pot_Set.Value;
            app.P3(4) = app.Actuator_4_Pot_Set.Value;
            app.P3(5) = app.Actuator_5_Pot_Set.Value;
            app.P3(6) = app.Actuator_6_Pot_Set.Value; 
            
            app.P3CheckBox.Value = 1;
            app.MovePlatformPtoP.Enable = 'on';
            app.ClearP3Button.Enable = 'on';
            app.MovetoP3Button.Enable = 'on';
        end

        % Button pushed function: SaveP4Button
        function SaveP4ButtonPushed(app, event)
            app.P4(1) = app.Actuator_1_Pot_Set.Value;
            app.P4(2) = app.Actuator_2_Pot_Set.Value;
            app.P4(3) = app.Actuator_3_Pot_Set.Value;
            app.P4(4) = app.Actuator_4_Pot_Set.Value;
            app.P4(5) = app.Actuator_5_Pot_Set.Value;
            app.P4(6) = app.Actuator_6_Pot_Set.Value; 
            
            app.P4CheckBox.Value = 1;
            app.MovePlatformPtoP.Enable = 'on';
            app.ClearP4Button.Enable = 'on';
            app.MovetoP4Button.Enable = 'on';
        end

        % Button pushed function: SaveP5Button
        function SaveP5ButtonPushed(app, event)
            app.P5(1) = app.Actuator_1_Pot_Set.Value;
            app.P5(2) = app.Actuator_2_Pot_Set.Value;
            app.P5(3) = app.Actuator_3_Pot_Set.Value;
            app.P5(4) = app.Actuator_4_Pot_Set.Value;
            app.P5(5) = app.Actuator_5_Pot_Set.Value;
            app.P5(6) = app.Actuator_6_Pot_Set.Value; 
            
            app.P5CheckBox.Value = 1;
            app.MovePlatformPtoP.Enable = 'on';
            app.ClearP5Button.Enable = 'on';
            app.MovetoP5Button.Enable = 'on';
        end

        % Button pushed function: SaveP6Button
        function SaveP6ButtonPushed(app, event)
            app.P6(1) = app.Actuator_1_Pot_Set.Value;
            app.P6(2) = app.Actuator_2_Pot_Set.Value;
            app.P6(3) = app.Actuator_3_Pot_Set.Value;
            app.P6(4) = app.Actuator_4_Pot_Set.Value;
            app.P6(5) = app.Actuator_5_Pot_Set.Value;
            app.P6(6) = app.Actuator_6_Pot_Set.Value; 
            
            app.P6CheckBox.Value = 1;
            app.MovePlatformPtoP.Enable = 'on';
            app.ClearP6Button.Enable = 'on';
            app.MovetoP6Button.Enable = 'on';
        end

        % Button pushed function: ClearP1Button
        function ClearP1ButtonPushed(app, event)
            app.P1 = zeros(1,6);
            app.P1CheckBox.Value = 0;
            app.ClearP1Button.Enable = 'off';
            app.MovetoP1Button.Enable = 'off';
        end

        % Button pushed function: ClearP2Button
        function ClearP2ButtonPushed(app, event)
            app.P2 = zeros(1,6);
            app.P2CheckBox.Value = 0;
            app.ClearP2Button.Enable = 'off';
            app.MovetoP2Button.Enable = 'off';
        end

        % Button pushed function: ClearP3Button
        function ClearP3ButtonPushed(app, event)
            app.P3 = zeros(1,6);
            app.P3CheckBox.Value = 0;
            app.ClearP3Button.Enable = 'off';
            app.MovetoP3Button.Enable = 'off';
        end

        % Button pushed function: ClearP4Button
        function ClearP4ButtonPushed(app, event)
            app.P4 = zeros(1,6);
            app.P4CheckBox.Value = 0;
            app.ClearP4Button.Enable = 'off';
            app.MovetoP4Button.Enable = 'off';
        end

        % Button pushed function: ClearP5Button
        function ClearP5ButtonPushed(app, event)
            app.P5 = zeros(1,6);
            app.P5CheckBox.Value = 0;
            app.ClearP5Button.Enable = 'off';
            app.MovetoP5Button.Enable = 'off';
        end

        % Button pushed function: ClearP6Button
        function ClearP6ButtonPushed(app, event)
            app.P6 = zeros(1,6);
            app.P6CheckBox.Value = 0;
            app.ClearP6Button.Enable = 'off';
            app.MovetoP6Button.Enable = 'off';
        end

        % Button pushed function: MovePlatformPtoP
        function MovePlatformPtoPButtonPushed(app, event)
            output_length(1,:) = app.P1;
            output_length(2,:) = app.P2;
            output_length(3,:) = app.P3;
            output_length(4,:) = app.P4;
            output_length(5,:) = app.P5;
            output_length(6,:) = app.P6;
         
            % This checks for any rows of 0 and removes, leaving an array of non-zero positions
            % Limitation of inability to go to reset position as part of
            % the sequence
            output_length=output_length(any(output_length,2),:);
            
            for w = 1:size(output_length,1)    
                MoveToPoint(app, output_length(w,:))
            end
        end

        % Button pushed function: MovePlatformReset
        function MoveReset(app, event)
            Home_Pos = [10,10,10,10,10,10];
            if app.serial_state == 1
                fprintf(app.ser, [strjoin({num2str(Home_Pos(1)), num2str(Home_Pos(2)), num2str(Home_Pos(3)), num2str(Home_Pos(4)), num2str(Home_Pos(5)), num2str(Home_Pos(6)), num2str(60)}), '\n']);
            end
            
            app.ErrorMessage.Visible = 'on';
            app.ErrorMessage.Value = [strjoin({num2str(Home_Pos(1)), num2str(Home_Pos(2)), num2str(Home_Pos(3)), num2str(Home_Pos(4)), num2str(Home_Pos(5)), num2str(Home_Pos(6)), num2str(60)}), '\n'];      
            
            for k = 1:length(Home_Pos)
                app.Actuator_Current_Pos(k) = Home_Pos(k);
            end
        end

        % Button pushed function: MovetoP1Button
        function MovetoP1ButtonPushed(app, event)
            MoveToPoint(app, app.P1)
        end

        % Button pushed function: MovetoP2Button
        function MovetoP2ButtonPushed(app, event)
            MoveToPoint(app, app.P2)
        end

        % Button pushed function: MovetoP3Button
        function MovetoP3ButtonPushed(app, event)
            MoveToPoint(app, app.P3)
        end

        % Button pushed function: MovetoP4Button
        function MovetoP4ButtonPushed(app, event)
            MoveToPoint(app, app.P4)
        end

        % Button pushed function: MovetoP5Button
        function MovetoP5ButtonPushed(app, event)
            MoveToPoint(app, app.P5)
        end

        % Button pushed function: MovetoP6Button
        function MovetoP6ButtonPushed(app, event)
            MoveToPoint(app, app.P6)
        end

        % Callback function
        function CalibrationButton_2Pushed(app, event)
             MoveDone = fread(app.ser,1);
            if MoveDone == 'D'
                app.ErrorMessage.BackgroundColor = 'r';
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Get the file path for locating images
            pathToMLAPP = fileparts(mfilename('fullpath'));

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Color = [0 0 0];
            app.UIFigure.Position = [100 100 554 722];
            app.UIFigure.Name = 'UI Figure';
            app.UIFigure.Resize = 'off';

            % Create Label
            app.Label = uilabel(app.UIFigure);
            app.Label.VerticalAlignment = 'top';
            app.Label.FontName = 'Trebuchet MS';
            app.Label.Position = [37 432 25 22];
            app.Label.Text = '';

            % Create PositionOrientationPanel
            app.PositionOrientationPanel = uipanel(app.UIFigure);
            app.PositionOrientationPanel.TitlePosition = 'centertop';
            app.PositionOrientationPanel.Title = 'Position & Orientation';
            app.PositionOrientationPanel.BackgroundColor = [0.651 0.651 0.651];
            app.PositionOrientationPanel.FontName = 'Trebuchet MS';
            app.PositionOrientationPanel.FontWeight = 'bold';
            app.PositionOrientationPanel.FontSize = 16;
            app.PositionOrientationPanel.Position = [367 383 171 226];

            % Create PxLabel
            app.PxLabel = uilabel(app.PositionOrientationPanel);
            app.PxLabel.HorizontalAlignment = 'right';
            app.PxLabel.VerticalAlignment = 'top';
            app.PxLabel.FontName = 'Garamond';
            app.PxLabel.FontSize = 14;
            app.PxLabel.FontWeight = 'bold';
            app.PxLabel.Position = [13 172 25 18];
            app.PxLabel.Text = 'Px:';

            % Create PxInput
            app.PxInput = uieditfield(app.PositionOrientationPanel, 'numeric');
            app.PxInput.HorizontalAlignment = 'center';
            app.PxInput.FontName = 'Garamond';
            app.PxInput.FontSize = 14;
            app.PxInput.FontWeight = 'bold';
            app.PxInput.Position = [47 170 45 22];

            % Create PyLabel
            app.PyLabel = uilabel(app.PositionOrientationPanel);
            app.PyLabel.HorizontalAlignment = 'right';
            app.PyLabel.VerticalAlignment = 'top';
            app.PyLabel.FontName = 'Garamond';
            app.PyLabel.FontSize = 14;
            app.PyLabel.FontWeight = 'bold';
            app.PyLabel.Position = [13 151 25 18];
            app.PyLabel.Text = 'Py:';

            % Create PyInput
            app.PyInput = uieditfield(app.PositionOrientationPanel, 'numeric');
            app.PyInput.HorizontalAlignment = 'center';
            app.PyInput.FontName = 'Garamond';
            app.PyInput.FontSize = 14;
            app.PyInput.FontWeight = 'bold';
            app.PyInput.Position = [47 149 45 22];

            % Create PzLabel
            app.PzLabel = uilabel(app.PositionOrientationPanel);
            app.PzLabel.HorizontalAlignment = 'right';
            app.PzLabel.VerticalAlignment = 'top';
            app.PzLabel.FontName = 'Garamond';
            app.PzLabel.FontSize = 14;
            app.PzLabel.FontWeight = 'bold';
            app.PzLabel.Position = [13 130 25 18];
            app.PzLabel.Text = 'Pz:';

            % Create PzInput
            app.PzInput = uieditfield(app.PositionOrientationPanel, 'numeric');
            app.PzInput.HorizontalAlignment = 'center';
            app.PzInput.FontName = 'Garamond';
            app.PzInput.FontSize = 14;
            app.PzInput.FontWeight = 'bold';
            app.PzInput.Position = [47 128 45 22];
            app.PzInput.Value = 17;

            % Create inchesLabel
            app.inchesLabel = uilabel(app.PositionOrientationPanel);
            app.inchesLabel.HorizontalAlignment = 'center';
            app.inchesLabel.FontName = 'Garamond';
            app.inchesLabel.FontSize = 10;
            app.inchesLabel.FontWeight = 'bold';
            app.inchesLabel.Position = [104 168 44 22];
            app.inchesLabel.Text = 'inches';

            % Create inchesLabel_2
            app.inchesLabel_2 = uilabel(app.PositionOrientationPanel);
            app.inchesLabel_2.HorizontalAlignment = 'center';
            app.inchesLabel_2.FontName = 'Garamond';
            app.inchesLabel_2.FontSize = 10;
            app.inchesLabel_2.FontWeight = 'bold';
            app.inchesLabel_2.Position = [104 147 44 22];
            app.inchesLabel_2.Text = 'inches';

            % Create inchesLabel_3
            app.inchesLabel_3 = uilabel(app.PositionOrientationPanel);
            app.inchesLabel_3.HorizontalAlignment = 'center';
            app.inchesLabel_3.FontName = 'Garamond';
            app.inchesLabel_3.FontSize = 10;
            app.inchesLabel_3.FontWeight = 'bold';
            app.inchesLabel_3.Position = [104 126 44 22];
            app.inchesLabel_3.Text = 'inches';

            % Create Label_5
            app.Label_5 = uilabel(app.PositionOrientationPanel);
            app.Label_5.HorizontalAlignment = 'right';
            app.Label_5.VerticalAlignment = 'top';
            app.Label_5.FontName = 'Garamond';
            app.Label_5.FontSize = 14;
            app.Label_5.FontWeight = 'bold';
            app.Label_5.Position = [13 109 25 18];
            app.Label_5.Text = 'Θ:';

            % Create ThetaInput
            app.ThetaInput = uieditfield(app.PositionOrientationPanel, 'numeric');
            app.ThetaInput.HorizontalAlignment = 'center';
            app.ThetaInput.FontName = 'Garamond';
            app.ThetaInput.FontSize = 14;
            app.ThetaInput.FontWeight = 'bold';
            app.ThetaInput.Position = [47 107 45 22];

            % Create Label_6
            app.Label_6 = uilabel(app.PositionOrientationPanel);
            app.Label_6.HorizontalAlignment = 'right';
            app.Label_6.VerticalAlignment = 'top';
            app.Label_6.FontName = 'Garamond';
            app.Label_6.FontSize = 14;
            app.Label_6.FontWeight = 'bold';
            app.Label_6.Position = [13 88 25 18];
            app.Label_6.Text = 'Φ:';

            % Create PhiInput
            app.PhiInput = uieditfield(app.PositionOrientationPanel, 'numeric');
            app.PhiInput.HorizontalAlignment = 'center';
            app.PhiInput.FontName = 'Garamond';
            app.PhiInput.FontSize = 14;
            app.PhiInput.FontWeight = 'bold';
            app.PhiInput.Position = [47 86 45 22];

            % Create Label_7
            app.Label_7 = uilabel(app.PositionOrientationPanel);
            app.Label_7.HorizontalAlignment = 'right';
            app.Label_7.VerticalAlignment = 'top';
            app.Label_7.FontName = 'Garamond';
            app.Label_7.FontSize = 14;
            app.Label_7.FontWeight = 'bold';
            app.Label_7.Position = [13 67 25 18];
            app.Label_7.Text = 'Ψ:';

            % Create PsiInput
            app.PsiInput = uieditfield(app.PositionOrientationPanel, 'numeric');
            app.PsiInput.HorizontalAlignment = 'center';
            app.PsiInput.FontName = 'Garamond';
            app.PsiInput.FontSize = 14;
            app.PsiInput.FontWeight = 'bold';
            app.PsiInput.Position = [47 65 45 22];

            % Create degreesLabel
            app.degreesLabel = uilabel(app.PositionOrientationPanel);
            app.degreesLabel.HorizontalAlignment = 'center';
            app.degreesLabel.FontName = 'Garamond';
            app.degreesLabel.FontSize = 10;
            app.degreesLabel.FontWeight = 'bold';
            app.degreesLabel.Position = [104 105 51 22];
            app.degreesLabel.Text = 'degrees';

            % Create degreesLabel_2
            app.degreesLabel_2 = uilabel(app.PositionOrientationPanel);
            app.degreesLabel_2.HorizontalAlignment = 'center';
            app.degreesLabel_2.FontName = 'Garamond';
            app.degreesLabel_2.FontSize = 10;
            app.degreesLabel_2.FontWeight = 'bold';
            app.degreesLabel_2.Position = [104 84 51 22];
            app.degreesLabel_2.Text = 'degrees';

            % Create degreesLabel_3
            app.degreesLabel_3 = uilabel(app.PositionOrientationPanel);
            app.degreesLabel_3.HorizontalAlignment = 'center';
            app.degreesLabel_3.FontName = 'Garamond';
            app.degreesLabel_3.FontSize = 10;
            app.degreesLabel_3.FontWeight = 'bold';
            app.degreesLabel_3.Position = [104 63 51 22];
            app.degreesLabel_3.Text = 'degrees';

            % Create CalcJointButton
            app.CalcJointButton = uibutton(app.PositionOrientationPanel, 'push');
            app.CalcJointButton.ButtonPushedFcn = createCallbackFcn(app, @CalcJointButtonPushed, true);
            app.CalcJointButton.BackgroundColor = [1 1 1];
            app.CalcJointButton.FontName = 'Garamond';
            app.CalcJointButton.FontSize = 16;
            app.CalcJointButton.FontWeight = 'bold';
            app.CalcJointButton.Enable = 'off';
            app.CalcJointButton.Position = [10 8 152 50];
            app.CalcJointButton.Text = {'Calculate '; 'Actuator Lengths'};

            % Create BaseJointLocationsPanel
            app.BaseJointLocationsPanel = uipanel(app.UIFigure);
            app.BaseJointLocationsPanel.TitlePosition = 'centertop';
            app.BaseJointLocationsPanel.Title = 'Base Joint Locations';
            app.BaseJointLocationsPanel.BackgroundColor = [0.651 0.651 0.651];
            app.BaseJointLocationsPanel.FontName = 'Trebuchet MS';
            app.BaseJointLocationsPanel.FontWeight = 'bold';
            app.BaseJointLocationsPanel.FontSize = 16;
            app.BaseJointLocationsPanel.Position = [15 255 197 171];

            % Create Actuator1Label
            app.Actuator1Label = uilabel(app.BaseJointLocationsPanel);
            app.Actuator1Label.HorizontalAlignment = 'right';
            app.Actuator1Label.VerticalAlignment = 'top';
            app.Actuator1Label.FontName = 'Garamond';
            app.Actuator1Label.FontSize = 10;
            app.Actuator1Label.FontWeight = 'bold';
            app.Actuator1Label.Position = [7 113 64 22];
            app.Actuator1Label.Text = 'Actuator 1:';

            % Create Base_Act_1
            app.Base_Act_1 = uieditfield(app.BaseJointLocationsPanel, 'numeric');
            app.Base_Act_1.Limits = [0 40];
            app.Base_Act_1.ValueDisplayFormat = '%.0f';
            app.Base_Act_1.HorizontalAlignment = 'center';
            app.Base_Act_1.FontName = 'Garamond';
            app.Base_Act_1.FontSize = 14;
            app.Base_Act_1.FontWeight = 'bold';
            app.Base_Act_1.Position = [86 115 36 22];

            % Create Actuator2Label
            app.Actuator2Label = uilabel(app.BaseJointLocationsPanel);
            app.Actuator2Label.HorizontalAlignment = 'right';
            app.Actuator2Label.VerticalAlignment = 'top';
            app.Actuator2Label.FontName = 'Garamond';
            app.Actuator2Label.FontSize = 10;
            app.Actuator2Label.FontWeight = 'bold';
            app.Actuator2Label.Position = [1 92 71 22];
            app.Actuator2Label.Text = 'Actuator 2:';

            % Create Base_Act_2
            app.Base_Act_2 = uieditfield(app.BaseJointLocationsPanel, 'numeric');
            app.Base_Act_2.Limits = [20 100];
            app.Base_Act_2.ValueDisplayFormat = '%.0f';
            app.Base_Act_2.HorizontalAlignment = 'center';
            app.Base_Act_2.FontName = 'Garamond';
            app.Base_Act_2.FontSize = 10;
            app.Base_Act_2.FontWeight = 'bold';
            app.Base_Act_2.Position = [86 94 36 22];
            app.Base_Act_2.Value = 60;

            % Create Actuator3Label
            app.Actuator3Label = uilabel(app.BaseJointLocationsPanel);
            app.Actuator3Label.HorizontalAlignment = 'right';
            app.Actuator3Label.VerticalAlignment = 'top';
            app.Actuator3Label.FontName = 'Garamond';
            app.Actuator3Label.FontSize = 10;
            app.Actuator3Label.FontWeight = 'bold';
            app.Actuator3Label.Position = [1 71 71 22];
            app.Actuator3Label.Text = 'Actuator 3:';

            % Create Base_Act_3
            app.Base_Act_3 = uieditfield(app.BaseJointLocationsPanel, 'numeric');
            app.Base_Act_3.Limits = [80 160];
            app.Base_Act_3.ValueDisplayFormat = '%.0f';
            app.Base_Act_3.HorizontalAlignment = 'center';
            app.Base_Act_3.FontName = 'Garamond';
            app.Base_Act_3.FontSize = 14;
            app.Base_Act_3.FontWeight = 'bold';
            app.Base_Act_3.Position = [86 73 36 22];
            app.Base_Act_3.Value = 120;

            % Create Actuator4Label
            app.Actuator4Label = uilabel(app.BaseJointLocationsPanel);
            app.Actuator4Label.HorizontalAlignment = 'right';
            app.Actuator4Label.VerticalAlignment = 'top';
            app.Actuator4Label.FontName = 'Garamond';
            app.Actuator4Label.FontSize = 10;
            app.Actuator4Label.FontWeight = 'bold';
            app.Actuator4Label.Position = [1 50 71 22];
            app.Actuator4Label.Text = 'Actuator 4:';

            % Create Base_Act_4
            app.Base_Act_4 = uieditfield(app.BaseJointLocationsPanel, 'numeric');
            app.Base_Act_4.Limits = [140 220];
            app.Base_Act_4.ValueDisplayFormat = '%.0f';
            app.Base_Act_4.HorizontalAlignment = 'center';
            app.Base_Act_4.FontName = 'Garamond';
            app.Base_Act_4.FontSize = 14;
            app.Base_Act_4.FontWeight = 'bold';
            app.Base_Act_4.Position = [86 52 36 22];
            app.Base_Act_4.Value = 180;

            % Create Actuator5Label
            app.Actuator5Label = uilabel(app.BaseJointLocationsPanel);
            app.Actuator5Label.HorizontalAlignment = 'right';
            app.Actuator5Label.VerticalAlignment = 'top';
            app.Actuator5Label.FontName = 'Garamond';
            app.Actuator5Label.FontSize = 10;
            app.Actuator5Label.FontWeight = 'bold';
            app.Actuator5Label.Position = [1 29 71 22];
            app.Actuator5Label.Text = 'Actuator 5:';

            % Create Base_Act_5
            app.Base_Act_5 = uieditfield(app.BaseJointLocationsPanel, 'numeric');
            app.Base_Act_5.Limits = [200 280];
            app.Base_Act_5.ValueDisplayFormat = '%.0f';
            app.Base_Act_5.HorizontalAlignment = 'center';
            app.Base_Act_5.FontName = 'Garamond';
            app.Base_Act_5.FontSize = 14;
            app.Base_Act_5.FontWeight = 'bold';
            app.Base_Act_5.Position = [86 31 36 22];
            app.Base_Act_5.Value = 240;

            % Create Actuator6Label
            app.Actuator6Label = uilabel(app.BaseJointLocationsPanel);
            app.Actuator6Label.HorizontalAlignment = 'right';
            app.Actuator6Label.VerticalAlignment = 'top';
            app.Actuator6Label.FontName = 'Garamond';
            app.Actuator6Label.FontSize = 10;
            app.Actuator6Label.FontWeight = 'bold';
            app.Actuator6Label.Position = [1 8 71 22];
            app.Actuator6Label.Text = 'Actuator 6:';

            % Create Base_Act_6
            app.Base_Act_6 = uieditfield(app.BaseJointLocationsPanel, 'numeric');
            app.Base_Act_6.Limits = [260 340];
            app.Base_Act_6.HorizontalAlignment = 'center';
            app.Base_Act_6.FontName = 'Garamond';
            app.Base_Act_6.FontSize = 14;
            app.Base_Act_6.FontWeight = 'bold';
            app.Base_Act_6.Position = [86 10 36 22];
            app.Base_Act_6.Value = 300;

            % Create degreesLabel_4
            app.degreesLabel_4 = uilabel(app.BaseJointLocationsPanel);
            app.degreesLabel_4.HorizontalAlignment = 'center';
            app.degreesLabel_4.FontName = 'Garamond';
            app.degreesLabel_4.FontSize = 14;
            app.degreesLabel_4.FontWeight = 'bold';
            app.degreesLabel_4.Position = [129 54 51 18];
            app.degreesLabel_4.Text = 'degrees';

            % Create degreesLabel_5
            app.degreesLabel_5 = uilabel(app.BaseJointLocationsPanel);
            app.degreesLabel_5.HorizontalAlignment = 'center';
            app.degreesLabel_5.FontName = 'Garamond';
            app.degreesLabel_5.FontSize = 14;
            app.degreesLabel_5.FontWeight = 'bold';
            app.degreesLabel_5.Position = [129 33 51 18];
            app.degreesLabel_5.Text = 'degrees';

            % Create degreesLabel_6
            app.degreesLabel_6 = uilabel(app.BaseJointLocationsPanel);
            app.degreesLabel_6.HorizontalAlignment = 'center';
            app.degreesLabel_6.FontName = 'Garamond';
            app.degreesLabel_6.FontSize = 14;
            app.degreesLabel_6.FontWeight = 'bold';
            app.degreesLabel_6.Position = [129 12 51 18];
            app.degreesLabel_6.Text = 'degrees';

            % Create degreesLabel_7
            app.degreesLabel_7 = uilabel(app.BaseJointLocationsPanel);
            app.degreesLabel_7.HorizontalAlignment = 'center';
            app.degreesLabel_7.FontName = 'Garamond';
            app.degreesLabel_7.FontSize = 14;
            app.degreesLabel_7.FontWeight = 'bold';
            app.degreesLabel_7.Position = [129 96 51 18];
            app.degreesLabel_7.Text = 'degrees';

            % Create degreesLabel_8
            app.degreesLabel_8 = uilabel(app.BaseJointLocationsPanel);
            app.degreesLabel_8.HorizontalAlignment = 'center';
            app.degreesLabel_8.FontName = 'Garamond';
            app.degreesLabel_8.FontSize = 14;
            app.degreesLabel_8.FontWeight = 'bold';
            app.degreesLabel_8.Position = [129 75 51 18];
            app.degreesLabel_8.Text = 'degrees';

            % Create degreesLabel_9
            app.degreesLabel_9 = uilabel(app.BaseJointLocationsPanel);
            app.degreesLabel_9.HorizontalAlignment = 'center';
            app.degreesLabel_9.FontName = 'Garamond';
            app.degreesLabel_9.FontSize = 14;
            app.degreesLabel_9.FontWeight = 'bold';
            app.degreesLabel_9.Position = [129 117 51 18];
            app.degreesLabel_9.Text = 'degrees';

            % Create TopJointLocations63Panel
            app.TopJointLocations63Panel = uipanel(app.UIFigure);
            app.TopJointLocations63Panel.TitlePosition = 'centertop';
            app.TopJointLocations63Panel.Title = 'Top Joint Locations (6-3)';
            app.TopJointLocations63Panel.BackgroundColor = [0.651 0.651 0.651];
            app.TopJointLocations63Panel.FontName = 'Trebuchet MS';
            app.TopJointLocations63Panel.FontWeight = 'bold';
            app.TopJointLocations63Panel.FontSize = 16;
            app.TopJointLocations63Panel.Position = [16 439 197 170];

            % Create Actuators12Label
            app.Actuators12Label = uilabel(app.TopJointLocations63Panel);
            app.Actuators12Label.HorizontalAlignment = 'right';
            app.Actuators12Label.VerticalAlignment = 'top';
            app.Actuators12Label.FontName = 'Garamond';
            app.Actuators12Label.FontSize = 10;
            app.Actuators12Label.FontWeight = 'bold';
            app.Actuators12Label.Position = [-6 101 100 22];
            app.Actuators12Label.Text = 'Actuators 1 & 2:';

            % Create Top_Act_1_2
            app.Top_Act_1_2 = uieditfield(app.TopJointLocations63Panel, 'numeric');
            app.Top_Act_1_2.Limits = [-0.0001 0.0001];
            app.Top_Act_1_2.ValueDisplayFormat = '%.0f';
            app.Top_Act_1_2.Editable = 'off';
            app.Top_Act_1_2.HorizontalAlignment = 'center';
            app.Top_Act_1_2.FontName = 'Garamond';
            app.Top_Act_1_2.FontSize = 14;
            app.Top_Act_1_2.FontWeight = 'bold';
            app.Top_Act_1_2.Position = [98 103 36 22];

            % Create Actuators34Label
            app.Actuators34Label = uilabel(app.TopJointLocations63Panel);
            app.Actuators34Label.HorizontalAlignment = 'right';
            app.Actuators34Label.VerticalAlignment = 'top';
            app.Actuators34Label.FontName = 'Garamond';
            app.Actuators34Label.FontSize = 10;
            app.Actuators34Label.FontWeight = 'bold';
            app.Actuators34Label.Position = [-6 63 101 22];
            app.Actuators34Label.Text = 'Actuators 3 & 4:';

            % Create Top_Act_3_4
            app.Top_Act_3_4 = uieditfield(app.TopJointLocations63Panel, 'numeric');
            app.Top_Act_3_4.Limits = [119.99 120.01];
            app.Top_Act_3_4.ValueDisplayFormat = '%.0f';
            app.Top_Act_3_4.Editable = 'off';
            app.Top_Act_3_4.HorizontalAlignment = 'center';
            app.Top_Act_3_4.FontName = 'Garamond';
            app.Top_Act_3_4.FontSize = 14;
            app.Top_Act_3_4.FontWeight = 'bold';
            app.Top_Act_3_4.Position = [98 65 36 22];
            app.Top_Act_3_4.Value = 120;

            % Create Actuators56Label
            app.Actuators56Label = uilabel(app.TopJointLocations63Panel);
            app.Actuators56Label.HorizontalAlignment = 'right';
            app.Actuators56Label.VerticalAlignment = 'top';
            app.Actuators56Label.FontName = 'Garamond';
            app.Actuators56Label.FontSize = 10;
            app.Actuators56Label.FontWeight = 'bold';
            app.Actuators56Label.Position = [-6 25 101 22];
            app.Actuators56Label.Text = 'Actuators 5 & 6:';

            % Create Top_Act_5_6
            app.Top_Act_5_6 = uieditfield(app.TopJointLocations63Panel, 'numeric');
            app.Top_Act_5_6.Limits = [239.99 240.01];
            app.Top_Act_5_6.ValueDisplayFormat = '%.0f';
            app.Top_Act_5_6.Editable = 'off';
            app.Top_Act_5_6.HorizontalAlignment = 'center';
            app.Top_Act_5_6.FontName = 'Garamond';
            app.Top_Act_5_6.FontSize = 14;
            app.Top_Act_5_6.FontWeight = 'bold';
            app.Top_Act_5_6.Position = [98 27 36 22];
            app.Top_Act_5_6.Value = 240;

            % Create degreesLabel_21
            app.degreesLabel_21 = uilabel(app.TopJointLocations63Panel);
            app.degreesLabel_21.HorizontalAlignment = 'center';
            app.degreesLabel_21.FontName = 'Garamond';
            app.degreesLabel_21.FontSize = 10;
            app.degreesLabel_21.FontWeight = 'bold';
            app.degreesLabel_21.Position = [145 102 51 22];
            app.degreesLabel_21.Text = 'degrees';

            % Create degreesLabel_19
            app.degreesLabel_19 = uilabel(app.TopJointLocations63Panel);
            app.degreesLabel_19.HorizontalAlignment = 'center';
            app.degreesLabel_19.FontName = 'Garamond';
            app.degreesLabel_19.FontSize = 10;
            app.degreesLabel_19.FontWeight = 'bold';
            app.degreesLabel_19.Position = [146 64 51 22];
            app.degreesLabel_19.Text = 'degrees';

            % Create degreesLabel_20
            app.degreesLabel_20 = uilabel(app.TopJointLocations63Panel);
            app.degreesLabel_20.HorizontalAlignment = 'center';
            app.degreesLabel_20.FontName = 'Garamond';
            app.degreesLabel_20.FontSize = 10;
            app.degreesLabel_20.FontWeight = 'bold';
            app.degreesLabel_20.Position = [145 27 51 22];
            app.degreesLabel_20.Text = 'degrees';

            % Create CalculatedActuatorLengthsPanel
            app.CalculatedActuatorLengthsPanel = uipanel(app.UIFigure);
            app.CalculatedActuatorLengthsPanel.TitlePosition = 'centertop';
            app.CalculatedActuatorLengthsPanel.Title = 'Calculated Actuator Lengths';
            app.CalculatedActuatorLengthsPanel.BackgroundColor = [0.651 0.651 0.651];
            app.CalculatedActuatorLengthsPanel.FontName = 'Trebuchet MS';
            app.CalculatedActuatorLengthsPanel.FontWeight = 'bold';
            app.CalculatedActuatorLengthsPanel.FontSize = 16;
            app.CalculatedActuatorLengthsPanel.Position = [223 200 315 171];

            % Create Actuator1Label_3
            app.Actuator1Label_3 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.Actuator1Label_3.HorizontalAlignment = 'right';
            app.Actuator1Label_3.VerticalAlignment = 'top';
            app.Actuator1Label_3.FontName = 'Garamond';
            app.Actuator1Label_3.FontSize = 10;
            app.Actuator1Label_3.FontWeight = 'bold';
            app.Actuator1Label_3.Position = [7 113 64 22];
            app.Actuator1Label_3.Text = 'Actuator 1:';

            % Create Actuator_1_Length_Set
            app.Actuator_1_Length_Set = uieditfield(app.CalculatedActuatorLengthsPanel, 'numeric');
            app.Actuator_1_Length_Set.Editable = 'off';
            app.Actuator_1_Length_Set.HorizontalAlignment = 'center';
            app.Actuator_1_Length_Set.FontName = 'Garamond';
            app.Actuator_1_Length_Set.FontSize = 14;
            app.Actuator_1_Length_Set.FontWeight = 'bold';
            app.Actuator_1_Length_Set.Position = [75 115 45 22];

            % Create Actuator2Label_3
            app.Actuator2Label_3 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.Actuator2Label_3.HorizontalAlignment = 'right';
            app.Actuator2Label_3.VerticalAlignment = 'top';
            app.Actuator2Label_3.FontName = 'Garamond';
            app.Actuator2Label_3.FontSize = 10;
            app.Actuator2Label_3.FontWeight = 'bold';
            app.Actuator2Label_3.Position = [1 92 71 22];
            app.Actuator2Label_3.Text = 'Actuator 2:';

            % Create Actuator_2_Length_Set
            app.Actuator_2_Length_Set = uieditfield(app.CalculatedActuatorLengthsPanel, 'numeric');
            app.Actuator_2_Length_Set.Editable = 'off';
            app.Actuator_2_Length_Set.HorizontalAlignment = 'center';
            app.Actuator_2_Length_Set.FontName = 'Garamond';
            app.Actuator_2_Length_Set.FontSize = 14;
            app.Actuator_2_Length_Set.FontWeight = 'bold';
            app.Actuator_2_Length_Set.Position = [75 94 45 22];

            % Create Actuator3Label_3
            app.Actuator3Label_3 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.Actuator3Label_3.HorizontalAlignment = 'right';
            app.Actuator3Label_3.VerticalAlignment = 'top';
            app.Actuator3Label_3.FontName = 'Garamond';
            app.Actuator3Label_3.FontSize = 10;
            app.Actuator3Label_3.FontWeight = 'bold';
            app.Actuator3Label_3.Position = [1 71 71 22];
            app.Actuator3Label_3.Text = 'Actuator 3:';

            % Create Actuator_3_Length_Set
            app.Actuator_3_Length_Set = uieditfield(app.CalculatedActuatorLengthsPanel, 'numeric');
            app.Actuator_3_Length_Set.Editable = 'off';
            app.Actuator_3_Length_Set.HorizontalAlignment = 'center';
            app.Actuator_3_Length_Set.FontName = 'Garamond';
            app.Actuator_3_Length_Set.FontSize = 14;
            app.Actuator_3_Length_Set.FontWeight = 'bold';
            app.Actuator_3_Length_Set.Position = [75 73 45 22];

            % Create Actuator4Label_3
            app.Actuator4Label_3 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.Actuator4Label_3.HorizontalAlignment = 'right';
            app.Actuator4Label_3.VerticalAlignment = 'top';
            app.Actuator4Label_3.FontName = 'Garamond';
            app.Actuator4Label_3.FontSize = 10;
            app.Actuator4Label_3.FontWeight = 'bold';
            app.Actuator4Label_3.Position = [1 50 71 22];
            app.Actuator4Label_3.Text = 'Actuator 4:';

            % Create Actuator_4_Length_Set
            app.Actuator_4_Length_Set = uieditfield(app.CalculatedActuatorLengthsPanel, 'numeric');
            app.Actuator_4_Length_Set.Editable = 'off';
            app.Actuator_4_Length_Set.HorizontalAlignment = 'center';
            app.Actuator_4_Length_Set.FontName = 'Garamond';
            app.Actuator_4_Length_Set.FontSize = 14;
            app.Actuator_4_Length_Set.FontWeight = 'bold';
            app.Actuator_4_Length_Set.Position = [75 52 45 22];

            % Create Actuator5Label_3
            app.Actuator5Label_3 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.Actuator5Label_3.HorizontalAlignment = 'right';
            app.Actuator5Label_3.VerticalAlignment = 'top';
            app.Actuator5Label_3.FontName = 'Garamond';
            app.Actuator5Label_3.FontSize = 10;
            app.Actuator5Label_3.FontWeight = 'bold';
            app.Actuator5Label_3.Position = [1 29 71 22];
            app.Actuator5Label_3.Text = 'Actuator 5:';

            % Create Actuator_5_Length_Set
            app.Actuator_5_Length_Set = uieditfield(app.CalculatedActuatorLengthsPanel, 'numeric');
            app.Actuator_5_Length_Set.Editable = 'off';
            app.Actuator_5_Length_Set.HorizontalAlignment = 'center';
            app.Actuator_5_Length_Set.FontName = 'Garamond';
            app.Actuator_5_Length_Set.FontSize = 14;
            app.Actuator_5_Length_Set.FontWeight = 'bold';
            app.Actuator_5_Length_Set.Position = [75 31 45 22];

            % Create Actuator6Label_3
            app.Actuator6Label_3 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.Actuator6Label_3.HorizontalAlignment = 'right';
            app.Actuator6Label_3.VerticalAlignment = 'top';
            app.Actuator6Label_3.FontName = 'Garamond';
            app.Actuator6Label_3.FontSize = 10;
            app.Actuator6Label_3.FontWeight = 'bold';
            app.Actuator6Label_3.Position = [1 8 71 22];
            app.Actuator6Label_3.Text = 'Actuator 6:';

            % Create Actuator_6_Length_Set
            app.Actuator_6_Length_Set = uieditfield(app.CalculatedActuatorLengthsPanel, 'numeric');
            app.Actuator_6_Length_Set.Editable = 'off';
            app.Actuator_6_Length_Set.HorizontalAlignment = 'center';
            app.Actuator_6_Length_Set.FontName = 'Garamond';
            app.Actuator_6_Length_Set.FontSize = 14;
            app.Actuator_6_Length_Set.FontWeight = 'bold';
            app.Actuator_6_Length_Set.Position = [75 10 45 22];

            % Create inchesLabel_7
            app.inchesLabel_7 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.inchesLabel_7.HorizontalAlignment = 'center';
            app.inchesLabel_7.FontName = 'Garamond';
            app.inchesLabel_7.FontSize = 10;
            app.inchesLabel_7.FontWeight = 'bold';
            app.inchesLabel_7.Position = [119 50 51 22];
            app.inchesLabel_7.Text = 'inches';

            % Create inchesLabel_8
            app.inchesLabel_8 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.inchesLabel_8.HorizontalAlignment = 'center';
            app.inchesLabel_8.FontName = 'Garamond';
            app.inchesLabel_8.FontSize = 10;
            app.inchesLabel_8.FontWeight = 'bold';
            app.inchesLabel_8.Position = [119 29 51 22];
            app.inchesLabel_8.Text = 'inches';

            % Create inchesLabel_9
            app.inchesLabel_9 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.inchesLabel_9.HorizontalAlignment = 'center';
            app.inchesLabel_9.FontName = 'Garamond';
            app.inchesLabel_9.FontSize = 10;
            app.inchesLabel_9.FontWeight = 'bold';
            app.inchesLabel_9.Position = [119 8 51 22];
            app.inchesLabel_9.Text = 'inches';

            % Create inchesLabel_5
            app.inchesLabel_5 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.inchesLabel_5.HorizontalAlignment = 'center';
            app.inchesLabel_5.FontName = 'Garamond';
            app.inchesLabel_5.FontSize = 10;
            app.inchesLabel_5.FontWeight = 'bold';
            app.inchesLabel_5.Position = [119 92 51 22];
            app.inchesLabel_5.Text = 'inches';

            % Create inchesLabel_6
            app.inchesLabel_6 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.inchesLabel_6.HorizontalAlignment = 'center';
            app.inchesLabel_6.FontName = 'Garamond';
            app.inchesLabel_6.FontSize = 10;
            app.inchesLabel_6.FontWeight = 'bold';
            app.inchesLabel_6.Position = [119 71 51 22];
            app.inchesLabel_6.Text = 'inches';

            % Create inchesLabel_4
            app.inchesLabel_4 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.inchesLabel_4.HorizontalAlignment = 'center';
            app.inchesLabel_4.FontName = 'Garamond';
            app.inchesLabel_4.FontSize = 10;
            app.inchesLabel_4.FontWeight = 'bold';
            app.inchesLabel_4.Position = [119 113 51 22];
            app.inchesLabel_4.Text = 'inches';

            % Create Actuator_2_Pot_Set
            app.Actuator_2_Pot_Set = uieditfield(app.CalculatedActuatorLengthsPanel, 'numeric');
            app.Actuator_2_Pot_Set.Editable = 'off';
            app.Actuator_2_Pot_Set.HorizontalAlignment = 'center';
            app.Actuator_2_Pot_Set.FontName = 'Garamond';
            app.Actuator_2_Pot_Set.FontSize = 14;
            app.Actuator_2_Pot_Set.FontWeight = 'bold';
            app.Actuator_2_Pot_Set.Position = [178 94 53 22];

            % Create Actuator_3_Pot_Set
            app.Actuator_3_Pot_Set = uieditfield(app.CalculatedActuatorLengthsPanel, 'numeric');
            app.Actuator_3_Pot_Set.Editable = 'off';
            app.Actuator_3_Pot_Set.HorizontalAlignment = 'center';
            app.Actuator_3_Pot_Set.FontName = 'Garamond';
            app.Actuator_3_Pot_Set.FontSize = 14;
            app.Actuator_3_Pot_Set.FontWeight = 'bold';
            app.Actuator_3_Pot_Set.Position = [178 73 53 22];

            % Create Actuator_4_Pot_Set
            app.Actuator_4_Pot_Set = uieditfield(app.CalculatedActuatorLengthsPanel, 'numeric');
            app.Actuator_4_Pot_Set.Editable = 'off';
            app.Actuator_4_Pot_Set.HorizontalAlignment = 'center';
            app.Actuator_4_Pot_Set.FontName = 'Garamond';
            app.Actuator_4_Pot_Set.FontSize = 14;
            app.Actuator_4_Pot_Set.FontWeight = 'bold';
            app.Actuator_4_Pot_Set.Position = [178 52 53 22];

            % Create Actuator_1_Pot_Set
            app.Actuator_1_Pot_Set = uieditfield(app.CalculatedActuatorLengthsPanel, 'numeric');
            app.Actuator_1_Pot_Set.Editable = 'off';
            app.Actuator_1_Pot_Set.HorizontalAlignment = 'center';
            app.Actuator_1_Pot_Set.FontName = 'Garamond';
            app.Actuator_1_Pot_Set.FontSize = 14;
            app.Actuator_1_Pot_Set.FontWeight = 'bold';
            app.Actuator_1_Pot_Set.Position = [178 115 53 22];

            % Create Actuator_5_Pot_Set
            app.Actuator_5_Pot_Set = uieditfield(app.CalculatedActuatorLengthsPanel, 'numeric');
            app.Actuator_5_Pot_Set.Editable = 'off';
            app.Actuator_5_Pot_Set.HorizontalAlignment = 'center';
            app.Actuator_5_Pot_Set.FontName = 'Garamond';
            app.Actuator_5_Pot_Set.FontSize = 14;
            app.Actuator_5_Pot_Set.FontWeight = 'bold';
            app.Actuator_5_Pot_Set.Position = [178 32 53 22];

            % Create Actuator_6_Pot_Set
            app.Actuator_6_Pot_Set = uieditfield(app.CalculatedActuatorLengthsPanel, 'numeric');
            app.Actuator_6_Pot_Set.Editable = 'off';
            app.Actuator_6_Pot_Set.HorizontalAlignment = 'center';
            app.Actuator_6_Pot_Set.FontName = 'Garamond';
            app.Actuator_6_Pot_Set.FontSize = 14;
            app.Actuator_6_Pot_Set.FontWeight = 'bold';
            app.Actuator_6_Pot_Set.Position = [178 11 53 22];

            % Create PotcountsLabel
            app.PotcountsLabel = uilabel(app.CalculatedActuatorLengthsPanel);
            app.PotcountsLabel.HorizontalAlignment = 'center';
            app.PotcountsLabel.FontName = 'Garamond';
            app.PotcountsLabel.FontSize = 10;
            app.PotcountsLabel.FontWeight = 'bold';
            app.PotcountsLabel.Position = [236 115 69 22];
            app.PotcountsLabel.Text = 'Pot counts';

            % Create PotcountsLabel_2
            app.PotcountsLabel_2 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.PotcountsLabel_2.HorizontalAlignment = 'center';
            app.PotcountsLabel_2.FontName = 'Garamond';
            app.PotcountsLabel_2.FontSize = 10;
            app.PotcountsLabel_2.FontWeight = 'bold';
            app.PotcountsLabel_2.Position = [236 94 69 22];
            app.PotcountsLabel_2.Text = 'Pot counts';

            % Create PotcountsLabel_3
            app.PotcountsLabel_3 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.PotcountsLabel_3.HorizontalAlignment = 'center';
            app.PotcountsLabel_3.FontName = 'Garamond';
            app.PotcountsLabel_3.FontSize = 10;
            app.PotcountsLabel_3.FontWeight = 'bold';
            app.PotcountsLabel_3.Position = [236 74 69 22];
            app.PotcountsLabel_3.Text = 'Pot counts';

            % Create PotcountsLabel_4
            app.PotcountsLabel_4 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.PotcountsLabel_4.HorizontalAlignment = 'center';
            app.PotcountsLabel_4.FontName = 'Garamond';
            app.PotcountsLabel_4.FontSize = 10;
            app.PotcountsLabel_4.FontWeight = 'bold';
            app.PotcountsLabel_4.Position = [236 53 69 22];
            app.PotcountsLabel_4.Text = 'Pot counts';

            % Create PotcountsLabel_5
            app.PotcountsLabel_5 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.PotcountsLabel_5.HorizontalAlignment = 'center';
            app.PotcountsLabel_5.FontName = 'Garamond';
            app.PotcountsLabel_5.FontSize = 10;
            app.PotcountsLabel_5.FontWeight = 'bold';
            app.PotcountsLabel_5.Position = [236 31 69 22];
            app.PotcountsLabel_5.Text = 'Pot counts';

            % Create PotcountsLabel_6
            app.PotcountsLabel_6 = uilabel(app.CalculatedActuatorLengthsPanel);
            app.PotcountsLabel_6.HorizontalAlignment = 'center';
            app.PotcountsLabel_6.FontName = 'Garamond';
            app.PotcountsLabel_6.FontSize = 10;
            app.PotcountsLabel_6.FontWeight = 'bold';
            app.PotcountsLabel_6.Position = [236 10 69 22];
            app.PotcountsLabel_6.Text = 'Pot counts';

            % Create Logo_2
            app.Logo_2 = uibutton(app.UIFigure, 'push');
            app.Logo_2.ButtonPushedFcn = createCallbackFcn(app, @Logo_2ButtonPushed, true);
            app.Logo_2.Icon = fullfile(pathToMLAPP, 'download.png');
            app.Logo_2.IconAlignment = 'center';
            app.Logo_2.BackgroundColor = [1 1 1];
            app.Logo_2.FontName = 'Trebuchet MS';
            app.Logo_2.Position = [373 620 165 89];
            app.Logo_2.Text = {''; ''};

            % Create StewartPlatformGUIButtonGroup
            app.StewartPlatformGUIButtonGroup = uibuttongroup(app.UIFigure);
            app.StewartPlatformGUIButtonGroup.SelectionChangedFcn = createCallbackFcn(app, @StewartPlatformGUIButtonGroupSelectionChanged, true);
            app.StewartPlatformGUIButtonGroup.TitlePosition = 'centertop';
            app.StewartPlatformGUIButtonGroup.Title = 'Stewart Platform GUI';
            app.StewartPlatformGUIButtonGroup.BackgroundColor = [0.651 0.651 0.651];
            app.StewartPlatformGUIButtonGroup.FontName = 'Trebuchet MS';
            app.StewartPlatformGUIButtonGroup.FontWeight = 'bold';
            app.StewartPlatformGUIButtonGroup.FontSize = 14;
            app.StewartPlatformGUIButtonGroup.Position = [187 620 181 89];

            % Create W24SeniorProjectFall2023Label
            app.W24SeniorProjectFall2023Label = uilabel(app.StewartPlatformGUIButtonGroup);
            app.W24SeniorProjectFall2023Label.HorizontalAlignment = 'center';
            app.W24SeniorProjectFall2023Label.WordWrap = 'on';
            app.W24SeniorProjectFall2023Label.FontName = 'Trebuchet MS';
            app.W24SeniorProjectFall2023Label.FontWeight = 'bold';
            app.W24SeniorProjectFall2023Label.Position = [7 31 167 30];
            app.W24SeniorProjectFall2023Label.Text = {'W24 Senior Project'; 'Fall 2023'};

            % Create NoahTannerScottBrownJacksonNorbegJacksonStanwickLabel
            app.NoahTannerScottBrownJacksonNorbegJacksonStanwickLabel = uilabel(app.StewartPlatformGUIButtonGroup);
            app.NoahTannerScottBrownJacksonNorbegJacksonStanwickLabel.HorizontalAlignment = 'center';
            app.NoahTannerScottBrownJacksonNorbegJacksonStanwickLabel.WordWrap = 'on';
            app.NoahTannerScottBrownJacksonNorbegJacksonStanwickLabel.FontName = 'Trebuchet MS';
            app.NoahTannerScottBrownJacksonNorbegJacksonStanwickLabel.FontSize = 10;
            app.NoahTannerScottBrownJacksonNorbegJacksonStanwickLabel.FontWeight = 'bold';
            app.NoahTannerScottBrownJacksonNorbegJacksonStanwickLabel.Position = [7 8 167 25];
            app.NoahTannerScottBrownJacksonNorbegJacksonStanwickLabel.Text = 'Noah Tanner, Scott Brown, Jackson Norbeg, Jackson Stanwick';

            % Create SerialPanel
            app.SerialPanel = uipanel(app.UIFigure);
            app.SerialPanel.TitlePosition = 'centertop';
            app.SerialPanel.Title = 'Serial';
            app.SerialPanel.BackgroundColor = [0.651 0.651 0.651];
            app.SerialPanel.FontName = 'Trebuchet MS';
            app.SerialPanel.FontWeight = 'bold';
            app.SerialPanel.FontSize = 16;
            app.SerialPanel.Position = [223 383 134 226];

            % Create CalibrationButton
            app.CalibrationButton = uibutton(app.SerialPanel, 'push');
            app.CalibrationButton.ButtonPushedFcn = createCallbackFcn(app, @CalibrationButtonPushed, true);
            app.CalibrationButton.BackgroundColor = [1 1 1];
            app.CalibrationButton.FontName = 'Trebuchet MS';
            app.CalibrationButton.FontSize = 16;
            app.CalibrationButton.FontWeight = 'bold';
            app.CalibrationButton.Position = [18 10 100 32];
            app.CalibrationButton.Text = 'Calibrate';

            % Create SerialLabel
            app.SerialLabel = uilabel(app.SerialPanel);
            app.SerialLabel.HorizontalAlignment = 'center';
            app.SerialLabel.FontName = 'Garamond';
            app.SerialLabel.FontSize = 14;
            app.SerialLabel.FontWeight = 'bold';
            app.SerialLabel.Position = [48.5 105 38 22];
            app.SerialLabel.Text = 'Serial';

            % Create SerialSwitch
            app.SerialSwitch = uiswitch(app.SerialPanel, 'slider');
            app.SerialSwitch.ValueChangedFcn = createCallbackFcn(app, @SerialSwitchValueChanged, true);
            app.SerialSwitch.FontName = 'Garamond';
            app.SerialSwitch.FontSize = 14;
            app.SerialSwitch.FontWeight = 'bold';
            app.SerialSwitch.Position = [39 133 57 25];

            % Create ErrorMessage
            app.ErrorMessage = uitextarea(app.SerialPanel);
            app.ErrorMessage.Editable = 'off';
            app.ErrorMessage.FontName = 'Garamond';
            app.ErrorMessage.FontSize = 14;
            app.ErrorMessage.FontWeight = 'bold';
            app.ErrorMessage.Position = [7 46 120 59];

            % Create COMPortEditFieldLabel
            app.COMPortEditFieldLabel = uilabel(app.SerialPanel);
            app.COMPortEditFieldLabel.HorizontalAlignment = 'right';
            app.COMPortEditFieldLabel.FontName = 'Garamond';
            app.COMPortEditFieldLabel.FontSize = 16;
            app.COMPortEditFieldLabel.FontWeight = 'bold';
            app.COMPortEditFieldLabel.Position = [14 172 76 22];
            app.COMPortEditFieldLabel.Text = 'COM Port';

            % Create COMPortEditField
            app.COMPortEditField = uieditfield(app.SerialPanel, 'numeric');
            app.COMPortEditField.Limits = [0 100];
            app.COMPortEditField.FontName = 'Garamond';
            app.COMPortEditField.FontSize = 16;
            app.COMPortEditField.FontWeight = 'bold';
            app.COMPortEditField.Position = [97 172 23 22];

            % Create Logo_4
            app.Logo_4 = uibutton(app.UIFigure, 'push');
            app.Logo_4.ButtonPushedFcn = createCallbackFcn(app, @Logo_4ButtonPushed, true);
            app.Logo_4.Icon = fullfile(pathToMLAPP, 'download-1.png');
            app.Logo_4.IconAlignment = 'center';
            app.Logo_4.BackgroundColor = [1 1 1];
            app.Logo_4.FontName = 'Trebuchet MS';
            app.Logo_4.Position = [15 620 165 89];
            app.Logo_4.Text = {''; ''};

            % Create SaveandSequenceSetpointsPanel
            app.SaveandSequenceSetpointsPanel = uipanel(app.UIFigure);
            app.SaveandSequenceSetpointsPanel.TitlePosition = 'centertop';
            app.SaveandSequenceSetpointsPanel.Title = 'Save and Sequence Setpoints';
            app.SaveandSequenceSetpointsPanel.BackgroundColor = [0.651 0.651 0.651];
            app.SaveandSequenceSetpointsPanel.FontName = 'Trebuchet MS';
            app.SaveandSequenceSetpointsPanel.FontWeight = 'bold';
            app.SaveandSequenceSetpointsPanel.FontSize = 16;
            app.SaveandSequenceSetpointsPanel.Position = [224 19 314 171];

            % Create SaveP1Button
            app.SaveP1Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.SaveP1Button.ButtonPushedFcn = createCallbackFcn(app, @SaveP1ButtonPushed, true);
            app.SaveP1Button.Enable = 'off';
            app.SaveP1Button.Position = [16 116 80 22];
            app.SaveP1Button.Text = {'Save P1'; ''};

            % Create SaveP2Button
            app.SaveP2Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.SaveP2Button.ButtonPushedFcn = createCallbackFcn(app, @SaveP2ButtonPushed, true);
            app.SaveP2Button.Enable = 'off';
            app.SaveP2Button.Position = [16 95 80 22];
            app.SaveP2Button.Text = {'Save P2'; ''};

            % Create SaveP3Button
            app.SaveP3Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.SaveP3Button.ButtonPushedFcn = createCallbackFcn(app, @SaveP3ButtonPushed, true);
            app.SaveP3Button.Enable = 'off';
            app.SaveP3Button.Position = [16 74 80 22];
            app.SaveP3Button.Text = {'Save P3'; ''};

            % Create SaveP4Button
            app.SaveP4Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.SaveP4Button.ButtonPushedFcn = createCallbackFcn(app, @SaveP4ButtonPushed, true);
            app.SaveP4Button.Enable = 'off';
            app.SaveP4Button.Position = [16 53 80 22];
            app.SaveP4Button.Text = {'Save P4'; ''};

            % Create SaveP5Button
            app.SaveP5Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.SaveP5Button.ButtonPushedFcn = createCallbackFcn(app, @SaveP5ButtonPushed, true);
            app.SaveP5Button.Enable = 'off';
            app.SaveP5Button.Position = [16 32 80 22];
            app.SaveP5Button.Text = 'Save P5';

            % Create SaveP6Button
            app.SaveP6Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.SaveP6Button.ButtonPushedFcn = createCallbackFcn(app, @SaveP6ButtonPushed, true);
            app.SaveP6Button.Enable = 'off';
            app.SaveP6Button.Position = [16 11 80 22];
            app.SaveP6Button.Text = {'Save P6'; ''};

            % Create P2CheckBox
            app.P2CheckBox = uicheckbox(app.SaveandSequenceSetpointsPanel);
            app.P2CheckBox.Enable = 'off';
            app.P2CheckBox.Text = '';
            app.P2CheckBox.Position = [103 95 25 22];

            % Create P3CheckBox
            app.P3CheckBox = uicheckbox(app.SaveandSequenceSetpointsPanel);
            app.P3CheckBox.Enable = 'off';
            app.P3CheckBox.Text = '';
            app.P3CheckBox.Position = [103 74 25 22];

            % Create P4CheckBox
            app.P4CheckBox = uicheckbox(app.SaveandSequenceSetpointsPanel);
            app.P4CheckBox.Enable = 'off';
            app.P4CheckBox.Text = '';
            app.P4CheckBox.Position = [103 53 25 22];

            % Create P5CheckBox
            app.P5CheckBox = uicheckbox(app.SaveandSequenceSetpointsPanel);
            app.P5CheckBox.Enable = 'off';
            app.P5CheckBox.Text = '';
            app.P5CheckBox.Position = [103 32 25 22];

            % Create P6CheckBox
            app.P6CheckBox = uicheckbox(app.SaveandSequenceSetpointsPanel);
            app.P6CheckBox.Enable = 'off';
            app.P6CheckBox.Text = '';
            app.P6CheckBox.Position = [103 11 25 22];

            % Create P1CheckBox
            app.P1CheckBox = uicheckbox(app.SaveandSequenceSetpointsPanel);
            app.P1CheckBox.Enable = 'off';
            app.P1CheckBox.Text = '';
            app.P1CheckBox.Position = [103 116 25 22];

            % Create ClearP1Button
            app.ClearP1Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.ClearP1Button.ButtonPushedFcn = createCallbackFcn(app, @ClearP1ButtonPushed, true);
            app.ClearP1Button.Enable = 'off';
            app.ClearP1Button.Position = [127 116 80 22];
            app.ClearP1Button.Text = {'Clear P1'; ''};

            % Create ClearP2Button
            app.ClearP2Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.ClearP2Button.ButtonPushedFcn = createCallbackFcn(app, @ClearP2ButtonPushed, true);
            app.ClearP2Button.Enable = 'off';
            app.ClearP2Button.Position = [127 95 80 22];
            app.ClearP2Button.Text = 'Clear P2';

            % Create ClearP3Button
            app.ClearP3Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.ClearP3Button.ButtonPushedFcn = createCallbackFcn(app, @ClearP3ButtonPushed, true);
            app.ClearP3Button.Enable = 'off';
            app.ClearP3Button.Position = [127 74 80 22];
            app.ClearP3Button.Text = {'Clear P3'; ''};

            % Create ClearP4Button
            app.ClearP4Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.ClearP4Button.ButtonPushedFcn = createCallbackFcn(app, @ClearP4ButtonPushed, true);
            app.ClearP4Button.Enable = 'off';
            app.ClearP4Button.Position = [127 53 80 22];
            app.ClearP4Button.Text = 'Clear P4';

            % Create ClearP5Button
            app.ClearP5Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.ClearP5Button.ButtonPushedFcn = createCallbackFcn(app, @ClearP5ButtonPushed, true);
            app.ClearP5Button.Enable = 'off';
            app.ClearP5Button.Position = [127 32 80 22];
            app.ClearP5Button.Text = 'Clear P5';

            % Create ClearP6Button
            app.ClearP6Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.ClearP6Button.ButtonPushedFcn = createCallbackFcn(app, @ClearP6ButtonPushed, true);
            app.ClearP6Button.Enable = 'off';
            app.ClearP6Button.Position = [127 11 80 22];
            app.ClearP6Button.Text = {'Clear P6'; ''};

            % Create MovetoP1Button
            app.MovetoP1Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.MovetoP1Button.ButtonPushedFcn = createCallbackFcn(app, @MovetoP1ButtonPushed, true);
            app.MovetoP1Button.Enable = 'off';
            app.MovetoP1Button.Position = [224 116 80 22];
            app.MovetoP1Button.Text = 'Move to P1';

            % Create MovetoP2Button
            app.MovetoP2Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.MovetoP2Button.ButtonPushedFcn = createCallbackFcn(app, @MovetoP2ButtonPushed, true);
            app.MovetoP2Button.Enable = 'off';
            app.MovetoP2Button.Position = [224 95 80 22];
            app.MovetoP2Button.Text = 'Move to P2';

            % Create MovetoP3Button
            app.MovetoP3Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.MovetoP3Button.ButtonPushedFcn = createCallbackFcn(app, @MovetoP3ButtonPushed, true);
            app.MovetoP3Button.Enable = 'off';
            app.MovetoP3Button.Position = [224 74 80 22];
            app.MovetoP3Button.Text = 'Move to P3';

            % Create MovetoP4Button
            app.MovetoP4Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.MovetoP4Button.ButtonPushedFcn = createCallbackFcn(app, @MovetoP4ButtonPushed, true);
            app.MovetoP4Button.Enable = 'off';
            app.MovetoP4Button.Position = [224 53 80 22];
            app.MovetoP4Button.Text = 'Move to P4';

            % Create MovetoP5Button
            app.MovetoP5Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.MovetoP5Button.ButtonPushedFcn = createCallbackFcn(app, @MovetoP5ButtonPushed, true);
            app.MovetoP5Button.Enable = 'off';
            app.MovetoP5Button.Position = [224 32 80 22];
            app.MovetoP5Button.Text = 'Move to P5';

            % Create MovetoP6Button
            app.MovetoP6Button = uibutton(app.SaveandSequenceSetpointsPanel, 'push');
            app.MovetoP6Button.ButtonPushedFcn = createCallbackFcn(app, @MovetoP6ButtonPushed, true);
            app.MovetoP6Button.Enable = 'off';
            app.MovetoP6Button.Position = [224 11 80 22];
            app.MovetoP6Button.Text = 'Move to P6';

            % Create MotionPanel
            app.MotionPanel = uipanel(app.UIFigure);
            app.MotionPanel.TitlePosition = 'centertop';
            app.MotionPanel.Title = 'Motion';
            app.MotionPanel.BackgroundColor = [0.651 0.651 0.651];
            app.MotionPanel.FontName = 'Trebuchet MS';
            app.MotionPanel.FontWeight = 'bold';
            app.MotionPanel.FontSize = 16;
            app.MotionPanel.Position = [15 19 197 224];

            % Create MovePlatform
            app.MovePlatform = uibutton(app.MotionPanel, 'push');
            app.MovePlatform.ButtonPushedFcn = createCallbackFcn(app, @MovePlatformButtonPushed, true);
            app.MovePlatform.BackgroundColor = [1 1 1];
            app.MovePlatform.FontName = 'Garamond';
            app.MovePlatform.FontSize = 16;
            app.MovePlatform.FontWeight = 'bold';
            app.MovePlatform.Enable = 'off';
            app.MovePlatform.Position = [5 142 187 27];
            app.MovePlatform.Text = 'Move to Current Setpoint';

            % Create MovePlatformPtoP
            app.MovePlatformPtoP = uibutton(app.MotionPanel, 'push');
            app.MovePlatformPtoP.ButtonPushedFcn = createCallbackFcn(app, @MovePlatformPtoPButtonPushed, true);
            app.MovePlatformPtoP.BackgroundColor = [1 1 1];
            app.MovePlatformPtoP.FontName = 'Garamond';
            app.MovePlatformPtoP.FontSize = 16;
            app.MovePlatformPtoP.FontWeight = 'bold';
            app.MovePlatformPtoP.Enable = 'off';
            app.MovePlatformPtoP.Position = [6 104 186 27];
            app.MovePlatformPtoP.Text = 'Move Through Sequence';

            % Create MovePlatformReset
            app.MovePlatformReset = uibutton(app.MotionPanel, 'push');
            app.MovePlatformReset.ButtonPushedFcn = createCallbackFcn(app, @MoveReset, true);
            app.MovePlatformReset.BackgroundColor = [1 1 1];
            app.MovePlatformReset.FontName = 'Garamond';
            app.MovePlatformReset.FontSize = 16;
            app.MovePlatformReset.FontWeight = 'bold';
            app.MovePlatformReset.Enable = 'off';
            app.MovePlatformReset.Position = [6 68 186 27];
            app.MovePlatformReset.Text = 'Move to Reset Position';

            % Create PathPlanningCheckBox
            app.PathPlanningCheckBox = uicheckbox(app.MotionPanel);
            app.PathPlanningCheckBox.Text = 'Path Planning';
            app.PathPlanningCheckBox.FontName = 'Garamond';
            app.PathPlanningCheckBox.FontSize = 14;
            app.PathPlanningCheckBox.FontWeight = 'bold';
            app.PathPlanningCheckBox.Position = [6 173 107 22];

            % Create SetPlatformSpeedSliderLabel
            app.SetPlatformSpeedSliderLabel = uilabel(app.MotionPanel);
            app.SetPlatformSpeedSliderLabel.HorizontalAlignment = 'right';
            app.SetPlatformSpeedSliderLabel.FontName = 'Garamond';
            app.SetPlatformSpeedSliderLabel.FontSize = 10;
            app.SetPlatformSpeedSliderLabel.FontWeight = 'bold';
            app.SetPlatformSpeedSliderLabel.Position = [37 41 118 22];
            app.SetPlatformSpeedSliderLabel.Text = 'Set Platform Speed';

            % Create SetPlatformSpeedSlider
            app.SetPlatformSpeedSlider = uislider(app.MotionPanel);
            app.SetPlatformSpeedSlider.Limits = [20 100];
            app.SetPlatformSpeedSlider.MajorTicks = [20 40 60 80 100];
            app.SetPlatformSpeedSlider.MinorTicks = [20 25 30 35 40 45 50 55 60 65 70 75 80 85 90 95 100];
            app.SetPlatformSpeedSlider.FontSize = 10;
            app.SetPlatformSpeedSlider.Position = [12 34 168 3];
            app.SetPlatformSpeedSlider.Value = 100;

            % Create PointsEditFieldLabel
            app.PointsEditFieldLabel = uilabel(app.UIFigure);
            app.PointsEditFieldLabel.HorizontalAlignment = 'right';
            app.PointsEditFieldLabel.FontName = 'Trebuchet MS';
            app.PointsEditFieldLabel.FontSize = 14;
            app.PointsEditFieldLabel.FontWeight = 'bold';
            app.PointsEditFieldLabel.Position = [159 193 45 22];
            app.PointsEditFieldLabel.Text = 'Points';

            % Create PointsEditField
            app.PointsEditField = uieditfield(app.UIFigure, 'numeric');
            app.PointsEditField.Limits = [1 100];
            app.PointsEditField.RoundFractionalValues = 'on';
            app.PointsEditField.HorizontalAlignment = 'center';
            app.PointsEditField.FontName = 'Trebuchet MS';
            app.PointsEditField.FontSize = 14;
            app.PointsEditField.FontWeight = 'bold';
            app.PointsEditField.Position = [130 193 31 22];
            app.PointsEditField.Value = 50;

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Test_UI

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end