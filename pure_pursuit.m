clear all; close all; clc;
global delt V
start_t = cputime;
% pos
% wp_X = [100 100 0 0];
% wp_Y = [0 100 100 0];
wp_X = 0:5:360;
wp_Y = 20*sind(wp_X);
% point = 5:5:100;
% wp_X = [point]
wp = [wp_X;wp_Y];

current_X = -5;
current_Y = -5;
psi = 1.57;

current_pos = [current_X;current_Y;psi];
init_pos = current_pos;
% init
lf = 1.5;
lr = 1.2;

L = lf+lr;
V = 5;
des_psi = 0;
delta_r = 0;

count = 1;
n = 1;

delt = 0.05;

goal = 0;

% circle

while true
% % 
%     ld = PSO(current_pos(1),current_pos(2),current_pos(3),wp(1),wp(2))
    ld = 1;
%     ld = 10;
    dist = sqrt(power(wp(1,count)-current_pos(1),2)+power(wp(2,count)-current_pos(2),2));
    if dist < 3
        count = count + 1;
    end

    if count > length(wp(1,:))
        count = 1;
        goal = goal + 1
        if goal == 1
            break
        end
    end
    Yr = abs(wp(2,count)-current_pos(2));

    des_psi = atan2(wp(2,count)-current_pos(2), wp(1,count)-current_pos(1));
        
    
    alpha = abs(-asin(Yr/ld)-psi);
%     alpha = (psi - );
    k = 2*sin(alpha)/ld;
    
    R = 1/k;
%     delta_f = k*atan2(2*(lf+lr)*sin(alpha),ld);
    delta_f = atan2(2*(lf+lr)*sin(alpha),ld);

    if delta_f > 0.52
        delta_f = 0.52;
    elseif delta_f < -0.52
        delta_f = -0.52;
    else
        delta_f = delta_f;
    end

    beta = atan2(lf*tan(delta_r)+lr*tan(delta_f),lf+lr);

    current_X = current_pos(1) + V*cos(psi+beta)*delt;
    current_Y = current_pos(2) + V*sin(psi+beta)*delt;
    psi = current_pos(3) + V*cos(beta)*tan(delta_f)*delt/(lf+lr) ;
%     psi_err = des_psi - psi;
    psi_err = -alpha;

    if abs(psi_err) > pi
        if psi_err > 0
            psi_err = psi_err - 2*pi;
        else
            psi_err = psi_err + 2*pi;
        end
    end
%     psi = current_pos(3) + psi_err*0.7*delt;
%     
%     mok = abs(fix(psi/(2*pi)));
%     if psi > 0
%         psi = psi-2*pi*mok;
%     elseif psi<0
%         psi = psi+2*pi*mok;
%     end

    if psi > pi 
        psi = psi - 2 * pi ;
    elseif psi < -pi
        psi = psi + 2 *pi ; 
    end
    current_pos = [current_X; current_Y; psi];
    state(1:5,n) = [current_pos; des_psi; delta_f];
    n = n+1;
%     hold on
%     scatter(current_pos(1),current_pos(2),25,'filled')
%     pause(0.0001)

end
end_t = cputime;
tt= end_t-start_t;