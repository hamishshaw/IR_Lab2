%% Kuka LBR iiwa 14 R820 Robot
classdef LBRiiwa14R820 < handle

    properties (Constant)

        %> Kuka LBR iiwa 14 R820 joint count
        jointCount = 7;

        %> Joint angles
        qz = zeros(1,LBRiiwa14R820.jointCount);
        qDefault = [0,0.5236,-2.0944,-1.3963,-1.5708,0.3491,0];
    end
        

    properties
        %> Robot model
        model;

        %> Workspace
        workspace = [-1.5 1.5 -1.5 1.5 -0.2 1.5];

    end

    methods
        %% Class for LBR iiwa 14 R820 robot simulation
        function self = LBRiiwa14R820()
            self.GetRobotModel();
            self.PlotAndColourRobot();
        end
        
        %% GetRobotModel
        function GetRobotModel(self)                                        %> Reference from https://www.oir.caltech.edu/twiki_oir/pub/Palomar/ZTF/KUKARoboticArmMaterial/Spez_LBR_iiwa_en.pdf, page 17-21
            pause(0.001);
            name = ['LBRiiwa14R820@',datestr(now,'yyyymmddTHHMMSSFFF')];
            L(1) = Link('d',0.360,'a',0,'alpha', deg2rad(90));
            L(2) = Link('d',0    ,'a',0,'alpha',-deg2rad(90));
            L(3) = Link('d',0.420,'a',0,'alpha', deg2rad(90));
            L(4) = Link('d',0    ,'a',0,'alpha',-deg2rad(90));
            L(5) = Link('d',0.400,'a',0,'alpha', deg2rad(90));
            L(6) = Link('d',0    ,'a',0,'alpha',-deg2rad(90));
            L(7) = Link('d',0.126,'a',0,'alpha', deg2rad(0));

            % Joint Coordinate Offset
            L(1).offset = deg2rad(90);

            % Joint Coordinate Limits
            L(1).qlim = deg2rad([-170,170]);
            L(2).qlim = deg2rad([-120,120]);
            L(3).qlim = deg2rad([-170,170]);
            L(4).qlim = deg2rad([-120,120]);
            L(5).qlim = deg2rad([-170,170]);
            L(6).qlim = deg2rad([-120,120]);
            L(7).qlim = deg2rad([-175,175]);

            self.model = SerialLink(L,'name',name);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self)
            mpath = strrep(which(mfilename),[mfilename '.m'],'');
            
            for linkIndex = 0:self.model.n
                [faceData,vertexData,plyData{linkIndex + 1}] = plyread([mpath,'ply/LinkN',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display Robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
%             self.model.plot(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
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