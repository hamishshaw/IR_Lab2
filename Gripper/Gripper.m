classdef Gripper < handle
    properties (Access = protected)
        %> Finger Model
        finger1; finger2; finger3;
        claw_open_close = 1; % 1 = open, 0 = close
    end

    properties (Constant, Access = protected)
        finger1Location  = trotx(-pi/2) * troty(pi/2) * transl(-0.0225,-0.043,0);
        finger2Location = trotx(-pi/2) * troty(-pi/2) * transl(-0.0225,-0.043,0.0175);
        finger3Location = trotx(-pi/2) * troty(-pi/2) * transl(-0.0225,-0.043,-0.0175);

        Fqo = deg2rad(30);

        step = 100;
    end

    properties
        %> Gripper base
         gBase;

        %> Finger Model
%         finger1; finger2; finger3;

        %> Workspace
        workspace = [-0.25 0.25 -0.25 0.25 -0.1 0.25];
        
    end

    methods
        function self = Gripper()
            self.GetGripperBaseModel();
            self.PlotAndColourGipperBase();
            self.GetFingerModel();
            self.PlotAndColourFinger1();
            self.PlotAndColourFinger2();
            self.PlotAndColourFinger3();
        end

        function gripperAnimate(self, RobotEndEffectorPos)
            self.gBase.base = RobotEndEffectorPos;
            self.gBase.animate(0);
            self.finger1.base = self.gBase.base * self.finger1Location;
            self.finger1.animate(0);
            self.finger2.base = self.gBase.base * self.finger2Location;
            self.finger2.animate(0);
            self.finger3.base = self.gBase.base * self.finger3Location;
            self.finger3.animate(0);
        end

        function closeGripper(self)
            if(self.claw_open_close)
                fqMatrix = jtraj(0,self.Fqo,self.step);
                for i = 1:self.step
                    self.finger1.animate(fqMatrix(i,:));
                    self.finger2.animate(fqMatrix(i,:));
                    self.finger3.animate(fqMatrix(i,:));
                    drawnow();
                end
                self.claw_open_close = 0;
            end
        end

        function openGripper(self)
            if(~self.claw_open_close)
                fqMatrix = jtraj(self.Fqo,0,self.step);
                for i = 1:self.step
                    self.finger1.animate(fqMatrix(i,:));
                    self.finger2.animate(fqMatrix(i,:));
                    self.finger3.animate(fqMatrix(i,:));
                    drawnow();
                end
                self.claw_open_close = 1;
            end
        end
    end

    methods (Access = protected)
        %% GetGripperBaseModel
        function GetGripperBaseModel(self)
            pause(0.001);
            name = ['Gripper_Base',datestr(now,'yyyymmddTHHMMSSFFF')];
            L1 = Link('d',0,'a',0,'alpha',0,'qlim',deg2rad([-360 360]));
            self.gBase = SerialLink(L1,'name',name);

        end

        %% GetFingerModel()
        function GetFingerModel(self)
            pause(0.001);
            name = ['Gripper_Finger1',datestr(now,'yyyymmddTHHMMSSFFF')];
            L1 = Link('d',0,'a',0,'alpha',pi/2,'offset',deg2rad(-30),'qlim',deg2rad([0 65]));
            self.finger1 = SerialLink(L1,'name',name);

            name = ['Gripper_Finger2',datestr(now,'yyyymmddTHHMMSSFFF')];
            self.finger2 = SerialLink(L1,'name',name);

            name = ['Gripper_Finger3',datestr(now,'yyyymmddTHHMMSSFFF')];
            self.finger3 = SerialLink(L1,'name',name);

            % Rotate finger 1 to the correct orientation
            self.finger1.base = self.gBase.base * self.finger1Location;

            % Rotate finger 2 to the correct orientation
            self.finger2.base = self.gBase.base * self.finger2Location;

            % Rotate finger 3 to the correct orientation
            self.finger3.base = self.gBase.base * self.finger3Location;
        end

        %% PlotAndColourGipperBase
        function PlotAndColourGipperBase(self)
            mpath = strrep(which(mfilename),[mfilename '.m'],'');

            [faceData, vertexData, plyData] = plyread([mpath,'ply/Gripper_Base.ply'],'tri');
            self.gBase.faces = {[],faceData};
            self.gBase.points = {[],vertexData};

            % Display robot
            self.gBase.plot3d(zeros(1,self.gBase.n),'noarrow','workspace',self.workspace);

            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.gBase.delay = 0;

            % Try to correctly colour the arm (if colours are in the ply file data)
            handle = findobj('Tag',self.gBase.name);
            h = get(handle, 'UserData');
            try
                h.link(2).Children.FaceVertexCData = [plyData.vertex.red ...
                                                  ,plyData.vertex.green ...
                                                  ,plyData.vertex.blue]/255;
                h.link(2).Children.FaceColor = 'interp';
            catch ME_1
                disp(ME_1);
            end
        end

        %% self.PlotAndColourFinger1
        function PlotAndColourFinger1(self)
            mpath = strrep(which(mfilename),[mfilename '.m'],'');
            
            for linkIndex = 0:self.finger1.n
                [faceData,vertexData,plyData{linkIndex + 1}] = plyread([mpath,'ply/LinkN_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.finger1.faces{linkIndex + 1} = faceData;
                self.finger1.points{linkIndex + 1} = vertexData;
            end

            % Display Robot
            self.finger1.plot3d(zeros,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.finger1.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.finger1.n
                handles = findobj('Tag', self.finger1.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end

        %% self.PlotAndColourFinger2
        function PlotAndColourFinger2(self)
            mpath = strrep(which(mfilename),[mfilename '.m'],'');
            
            for linkIndex = 0:self.finger2.n
                [faceData,vertexData,plyData{linkIndex + 1}] = plyread([mpath,'ply/LinkN_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.finger2.faces{linkIndex + 1} = faceData;
                self.finger2.points{linkIndex + 1} = vertexData;
            end

            % Display Robot
            self.finger2.plot3d(zeros,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.finger2.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.finger2.n
                handles = findobj('Tag', self.finger2.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end

        %% self.PlotAndColourFinger3
        function PlotAndColourFinger3(self)
            mpath = strrep(which(mfilename),[mfilename '.m'],'');
            
            for linkIndex = 0:self.finger3.n
                [faceData,vertexData,plyData{linkIndex + 1}] = plyread([mpath,'ply/LinkN_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.finger3.faces{linkIndex + 1} = faceData;
                self.finger3.points{linkIndex + 1} = vertexData;
            end

            % Display Robot
            self.finger3.plot3d(zeros,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.finger3.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.finger3.n
                handles = findobj('Tag', self.finger3.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
    end
end