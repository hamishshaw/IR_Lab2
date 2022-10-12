%% Kuka LBR iiwa 14 R820 Robot in the Rail
classdef LinearLBRiiwa14R820 < handle
    properties
        %> Robot model
        model;

        %> workspace
        workspace = [-3 2 -2 2 -1 2];

        %> Flag to indicate if gripper is used
        useGripper = false;
    end

    methods%% Class for UR3 robot simulation
        function self = LinearLBRiiwa14R820(useGripper)
        self.useGripper = useGripper;
        self.GetRobotModel();
        self.PlotAndColourRobot();
        end

         %% GetRobotModel
        function GetRobotModel(self)                                        %> Reference from https://www.oir.caltech.edu/twiki_oir/pub/Palomar/ZTF/KUKARoboticArmMaterial/Spez_LBR_iiwa_en.pdf, page 17-21
            pause(0.001);
            name = ['LBRiiwa14R820@',datestr(now,'yyyymmddTHHMMSSFFF')];
            L(1) = Link('prismatic','theta',deg2rad(180),'a',0,'alpha', deg2rad(90)); % Rail parameter
            L(2) = Link(            'd',0.268          ,'a',0,'alpha', deg2rad(90));
%             L(3) = Link('d',0.210,'a',0,'alpha', deg2rad(90));
%             L(4) = Link('d',0    ,'a',0,'alpha',-deg2rad(90));
%             L(5) = Link('d',0.200,'a',0,'alpha', deg2rad(90));
%             L(6) = Link('d',0    ,'a',0,'alpha',-deg2rad(90));
%             L(7) = Link('d',0.063,'a',0,'alpha', deg2rad(0));

            % Joint Coordinate Offset
%             L(1).offset = deg2rad(90);

            % Joint Coordinate Limits
            L(1).qlim = [-0.8,0];
            L(2).qlim = deg2rad([-170,170]);
%             L(3).qlim = deg2rad([-170,170]);
%             L(4).qlim = deg2rad([-120,120]);
%             L(5).qlim = deg2rad([-170,170]);
%             L(6).qlim = deg2rad([-120,120]);
%             L(7).qlim = deg2rad([-175,175]);

            self.model = SerialLink(L,'name',name);
            % Rotate robot to the correct orientation
            self.model.base = self.model.base * trotx(pi/2) * troty(pi/2);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self)
            mpath = strrep(which(mfilename),[mfilename '.m'],'');
            
            for linkIndex = 0:self.model.n
                [faceData,vertexData,plyData{linkIndex + 1}] = plyread([mpath,'ply/LinKUKAlink_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display Robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
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