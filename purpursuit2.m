clear all; close all; clc;

% % define path
path = 1; 
fprintf("path : %d번 path\n",path)
if path == 1
    wp_X = 0:5:360;
    wp_Y = 20*sind(wp_X );
    wp(:,1) = wp_X;
    wp(:,2) = wp_Y;
elseif path ==2
    T = readmatrix('TrackData.xlsx');
    wp_X = T(1800:164000,2);
    wp_Y = T(1800:164000,3);
end

wp__(:,1) = wp_X;
wp__(:,2) = wp_Y;

% 광역경로점들 간의 간격을 4.5미터로 지정
% dist = 0 ; index_ =1 ;
% for time_index = 1 : length(wp__(:,1))-1
%     dist = dist + sqrt((wp__(time_index,1)-wp__(time_index+1,1) )^2+(wp__(time_index,2)-wp__(time_index+1,2) )^2) ;
%     if dist > 1
%         wp(index_,:) = [wp__(time_index,1) wp__(time_index,2) ] ;
%         dist = 0 ;
%         index_ = index_ + 1 ;
%     end
%     if index_ > 1
%         dist_ = sqrt((wp__(time_index,1)-wp__(1,1) )^2+(wp__(time_index,2)-wp__(1,2) )^2) ;
%         if dist_ < 1
%             break
%         end
%     end
%     
% end



[~, track] = min(vecnorm(wp(:,1:2)' - [0,0]'));
% wp(:,3) = 

% plot(wp(:,1),wp(:,2),'b')
% grid on
% legend('path','FontSize',15,'Location','southeast') 
% xlabel('x-axis','Interpreter','latex','FontSize',25)
% ylabel('y-axis','Interpreter','latex','FontSize',25)
% title('path','Interpreter','latex','FontSize',30)




%% define parameter
param.L = 2.9; % length of wheelbase
param.Ld = 5; % Look-ahead distance
param.lim_delta = 30;

lookahead_dist = param.Ld;
wheelbase = param.L;

start_pos = [0 -5];
goal_pos = [];
delt = 0.1; % time of computaion
V = 20; % Velocity
psi = deg2rad(90);
delta = 0;
distance = 0;
steering = 0;
state = [start_pos psi steering];
i = 1;
Goal = 0;
while Goal == 0
    % find closest waypoint
    distance = vecnorm(wp(:,1:2)' - state(1:2)');
    
    [~, min_index] = min(distance);
 
    % find goal point
    for j = min_index:length(wp)
        dist = norm(wp(j, 1:2) - state(:,1:2));
        if dist > lookahead_dist
            break
        end
        
    end
    lookahead_pt = wp(j,:);

    alpha =  atan2(lookahead_pt(2)-state(2),lookahead_pt(1)-state(1))-state(3);
    R = lookahead_dist/(2*sin(alpha));
    kappa = 1/R;


    % define bicycle kinematics
    x_dot = V*cos(psi);
    y_dot = V*sin(psi);
    steering = atan2(wheelbase*2*sin(alpha),lookahead_dist);
%     steering = atan2(wheelbase/R);
%     if R < 0
%         steering = atan2(wheelbase,R) - pi;
%     else
%         steering = atan2(wheelbase,R);
%     end
    
    psi_dot = (V/wheelbase)*tan(steering);
    
%     limit steering
    if steering > deg2rad(30)
        steering = deg2rad(30);
    elseif steering < deg2rad(-30)
        steering = deg2rad(-30);
    end

    % update state
    state(1) = state(1) + x_dot*delt;
    state(2) = state(2) + y_dot*delt;
    state(3) = state(3) + psi_dot*delt;
    state(4) = steering*0.7;
    



    ego_status(i,1) = state(1); % x
    ego_status(i,2) = state(2); % y
    ego_status(i,3) = state(3); % psi
    ego_status(i,4) = state(4); % steering
    ego_status(i,5) = state(4);
    ego_status(i,6) = R;
    ego_status(i,7) = alpha;

    psi = ego_status(i,3);
    

%     scatter(state(1),state(2),10,'filled') 
%     pause(0.01)
    i = i + 1;

    if norm(wp(end,1:2)-state(1:2)) < lookahead_dist/2
        Goal = 1;
        fprintf('===== Goal =====\n')
    end


%     % plotting
%     figure(1);
%     plot(wp(:,1),wp(:,2),'b')
%     hold on
%     plot(ego_status(:,1),ego_status(:,2),'r') % xy graph
%     grid on
%     legend('Path','vehicle','FontSize',15) 
%     xlabel('X-axis','Interpreter','latex','FontSize',25)
%     ylabel('Y-axis','Interpreter','latex','FontSize',25)
%     title('Tracking','Interpreter','latex','FontSize',30)
%     set(gca,'Box','On')  
%     axis auto
%     
%     figure(2)
%     plot(rad2deg(ego_status(:,4)),'b') % steering angle
% %     hold on
% %     plot(rad2deg(ego_status(:,5)),'r'); % filter steering angle
%     grid on
%     legend('Steering angle','Filtered Steering angle','FontSize',15);
%     xlabel('Time','Interpreter','latex','FontSize',25)
%     ylabel('Steering angle','Interpreter','latex','FontSize',25)
%     title('Steering table','Interpreter','latex','FontSize',30)
%     set(gca,'Box','On')  
%     subplot(2,1,2)
%     grid on
%     plot(rad2deg(ego_status(:,5)))
%     pause(0.001)

% 

end


hold off

%% Video

% filter steering angle
windowSize = 20; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;

% ego_status(:,5) = filter(b,a,ego_status(:,5));

mode = 1;

if mode == 1
    video = VideoWriter("2022-06-13-path_3.avi");
    video.FrameRate = 60;
    open(video)
    
    Map1 = figure(1);
    pos = get(Map1, 'Position');
    scrsz = get(0,'ScreenSize');
    set(Map1,'Color','White','Position',[61 100 1407*0.8 864*0.8])

    figure(1)
    plot(wp(:,1),wp(:,2),'b')
    grid on
    hold on
    for kk =1:length(ego_status(:,1))  
        plot(ego_status(1:kk,1),ego_status(1:kk,2),'r') % xy graph
        grid on
        legend('Path','vehicle','FontSize',15,'Location','southeast') 
        xlabel('X-axis','Interpreter','latex','FontSize',25)
        ylabel('Y-axis','Interpreter','latex','FontSize',25)
        title('Tracking','Interpreter','latex','FontSize',30)
        set(gca,'Box','On')  
    
        currFrame = getframe(Map1);
        writeVideo(video,currFrame)
    end
    hold off
    close(video)
    
end
% delta video
if mode == 2
    video2 = VideoWriter("2022-06-13-delta_3.avi");
    video2.FrameRate = 60;
    open(video2)
    
    Map2 = figure(2);
    pos2 = get(Map2, 'Position');
    scrsz2 = get(0,'ScreenSize');
    set(Map2,'Color','White','Position',[61 100 1407*0.8 864*0.8])
    
    
    for kk = 1:length(ego_status(:,1))
        figure(2)
        plot(rad2deg(ego_status(1:kk,4)),'b') % steering angle
        grid on
        legend('Steering angle','Filtered Steering angle','FontSize',15,'Location','southeast');
        xlabel('Time','Interpreter','latex','FontSize',25)
        ylabel('Steering angle','Interpreter','latex','FontSize',25)
        title('Steering table','Interpreter','latex','FontSize',30)
        set(gca,'Box','On')  
        
    
        currFrame = getframe(Map2);
        writeVideo(video2,currFrame)
    end
    hold off
    close(video2)
end
% plotting

figure(1)
plot(ego_status(:,7),'r')
grid on
legend('Alpha','Fontsize',15,'Location','southeast')
xlabel('Time','Interpreter','latex','FontSize',25)
ylabel('rad','Interpreter','latex','FontSize',25)
title('Alpha','Interpreter','latex','FontSize',30)

figure(2)
plot(wp(:,1),wp(:,2),'b')
hold on
plot(ego_status(:,1),ego_status(:,2),'r') % xy graph
grid on
legend('Path','vehicle','FontSize',15,'Location','southeast') 
xlabel('X-axis','Interpreter','latex','FontSize',25)
ylabel('Y-axis','Interpreter','latex','FontSize',25)
title('Tracking','Interpreter','latex','FontSize',30)
set(gca,'Box','On') 

figure(3)
plot(rad2deg(ego_status(:,4)),'b') % steering angle
grid on
legend('Steering angle','FontSize',15,'Location','southeast');
xlabel('Time','Interpreter','latex','FontSize',25)
ylabel('Steering angle','Interpreter','latex','FontSize',25)
title('Steering table','Interpreter','latex','FontSize',30)
set(gca,'Box','On')  
ylim([-5 5])

fprintf('===== Finish =====')