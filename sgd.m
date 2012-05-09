function [ final_alpha ] = sgd()
%SGD Summary of this function goes here
%   Detailed explanation goes here
    
    %%% Initial running params          
    torso0 = -0.2;
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
    i = 1;
    global best_cost;
    global best_alpha;
    best_cost = 99999999;
    
    while( i < 2000)
        %J(alpha)
        d_a = delta_alpha(alpha);
        alpha = alpha + d_a;
        
        plot (i, J(alpha), 'blue.-', 'markersize', 15);
        hold on
        i = i + 1;
    end
    
    final_alpha = d_a;
end

function j = J(alpha)
    % Simple 1D case
    %x = alpha(1);
    %j = -3*x^3 + x^2 + x + 1;
    
    command =['java TheOne ' num2str(alpha)];
    [status, result] = system(command);
    j = status;
end

function d_a = delta_alpha( alpha )
    global best_cost;
    global best_alpha;
    
    eta = 0.0005;
    mu = 0.001;
    z = normrnd(0, mu, 1, length(alpha));
   
    alpha_plus_z = alpha + z;
    J_alpha_plus_z = J(alpha_plus_z);
    
    if J_alpha_plus_z ~= -1
        d_a = -eta * ( J_alpha_plus_z - J(alpha)) * z;
        if J_alpha_plus_z < best_cost
            best_cost = J_alpha_plus_z
            best_alpha = alpha_plus_z
        end
    else
        d_a = zeros(1, length(z));
    end
    
    %plot(alpha, J(alpha), 'blue.-', 'markersize', 20);
    %hold on
end
