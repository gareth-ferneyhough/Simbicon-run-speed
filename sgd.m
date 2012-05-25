function [ final_alpha ] = sgd()
%SGD Summary of this function goes here
%   Detailed explanation goes here
    
    %%% Initial running params          
    torso0 = 0;
    torso1 = 0;
    torso2 = 0;		
    rhip0 = 0.8;
    rhip1 = 0;
    rhip2 = 0.2;
    rknee0 = -1.84;
    rknee1 = 0;
    rknee2 = 0;
    %lhip0 = 0; dont need these, must be 0
    %lhip1 = 0;
    %lhip2 = 0;
    lknee0 = -0.05;
    lknee1 = 0;
    lknee2 = 0;
    rankle0 = 0.2;
    rankle1 = 0;
    rankle2 = 0;
    lankle0 = 0.27;
    lankle1 = 0;
    lankle2 = 0;
    transTime = 0.21; 
    %%%
    
%     %%% Initial running params          
%     torso0 =  -0.2349 ;
%     torso1 = 0.0034 ;
%     torso2 = -0.0667;		
%     rhip0 = 0.7908  ;
%     rhip1 = -0.0033 ;
%     rhip2 = 0.1774 ;
%     rknee0 = -1.8435;
%     rknee1 = -0.0054 ;
%     rknee2 = -0.0254  ;
%     %lhip0 = 0; dont need these, must be 0
%     %lhip1 = 0;
%     %lhip2 = 0;
%     lknee0 = -0.0530;
%     lknee1 = -0.0012 ;
%     lknee2 = -0.0136 ;
%     rankle0 = 0.2130 ;
%     rankle1 = -0.0039;
%     rankle2 = 0.0104  ;
%     lankle0 = 0.2884  ;
%     lankle1 = -0.0059  ;
%     lankle2 = 0.0394  ;
%     transTime = 0.2169;
%     %%%
    
         
    alpha = zeros(1, 19);
    alpha(1) = torso0;
    alpha(2) = torso1;
    alpha(3) = torso2;
    alpha(4) = rhip0;
    alpha(5) = rhip1;
    alpha(6) = rhip2;
    alpha(7) = rknee0;
    alpha(8) = rknee1;
    alpha(9) = rknee2;
    alpha(10) = lknee0;
    alpha(11) = lknee1;
    alpha(12) = lknee2;
    alpha(13) = rankle0;
    alpha(14) = rankle1;
    alpha(15) = rankle2;
    alpha(16) = lankle0;
    alpha(17) = lankle1;
    alpha(18) = lankle2;
    alpha(19) = transTime;  
   
        
    d_a = 0;
    global i;
    global best_cost;
    global best_alpha;
    global last_alpha;
    
    best_cost = 99999999;
    last_alpha = alpha;
    
    %s = RandStream('mt19937ar','Seed', 0);
    %RandStream.setGlobalStream(s);
    
    i = 1;
    while( i < 2000)
        %J(alpha)
        fprintf('loop\n');
        d_a = delta_alpha(alpha);
        alpha = alpha + d_a;
       
        i = i + 1;
    end
    
    final_alpha = d_a;
end

function j = J(alpha)
    % Simple 1D case
    %x = alpha(1);
    %j = -3*x^3 + x^2 + x + 1;
    
    command =['java TheOne ' num2str(alpha)];
    [status, result] = system(command)
    j = status;
end

function d_a = delta_alpha( alpha )
    global best_cost;
    global best_alpha;
    global last_alpha;
    
    eta = 0.001;
    mu = 0.005;
    z = normrnd(0, mu, 1, length(alpha));
    
    J_alpha = J(alpha);
    if J_alpha == -1 % bad alpha. restore last good alpha
        alpha = last_alpha;
        J_alpha = J(alpha);
        if J_alpha == -1
            fprintf('fucked\n')
        end
            
        fprintf('bad J_alpha\n')
    else
        last_alpha = alpha %save good alpha
    end
    
    %%% Plot J(alpha)
    global i;
    plot (i, J_alpha, 'blue.-', 'markersize', 15);
    hold on
      
    while true
        alpha_plus_z = alpha + z;
        J_alpha_plus_z = J(alpha_plus_z);       
        
        if J_alpha_plus_z == -1
            fprintf('bad J_alpha_plus_z\n')
            z = normrnd(0, mu, 1, length(alpha)); % re-get new z vector
        else
            break
        end
    end
        
        d_a = -eta * ( J_alpha_plus_z - J_alpha) * z;
        if J_alpha_plus_z < best_cost
            best_cost = J_alpha_plus_z
            best_alpha = alpha_plus_z
        end
    
    %plot(alpha, J(alpha), 'blue.-', 'markersize', 20);
    %hold on
end
