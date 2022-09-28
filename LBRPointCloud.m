clf; clear; close all; clc;

%% Test Kuka LBR iiwa 14 R820 Robot
robot = LBRiiwa14R820;
hold on

qlim = robot.model.qlim
stepRads = deg2rad(45);
stepTrans = 0.1;
pointCloudeSize = prod(floor((qlim(2:6,2)-qlim(2:6,1))/stepRads + 1));
pointCloud = zeros(pointCloudeSize,3);
counter = 1;
tic

for q1 = qlim(1,1):stepTrans:qlim(1,2)
    for q2 = 0:stepRads:qlim(2,2)
        for q3 = 0:stepRads:qlim(3,2)
            for q4 = 0:stepRads:qlim(4,2)
                for q5 = 0:stepRads:qlim(5,2)
                    for q6 = 0:stepRads:qlim(6,2)
                        q7 = 0; % end effector only rotates
                        q = [q1,q2,q3,q4,q5,q6,q7];
                        tr = robot.model.fkine(q);                        
                        pointCloud(counter,:) = tr(1:3,4)';
                        counter = counter + 1; 
                        if mod(counter/pointCloudeSize * 100,1) == 0
                            display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                        end
                     end
                end
            end
        end
    end
end

cloudPlot = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
% reach is approx 0.8 m on x-y plane