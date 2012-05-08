function [ final_alpha ] = sgd( alpha )
%SGD Summary of this function goes here
%   Detailed explanation goes here
    
    d_a = 0;
    i = 1;
    while( i < 100)
        d_a = delta_alpha(alpha);
        alpha = alpha + d_a;
        J(alpha)
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
    
    command =['java TheOne ' num2str(alpha)]
    [status,result] = system(command, '-echo');
    j = status;
end

function d_a = delta_alpha( alpha )
    eta = 1;
    mu = 1;
    z = normrnd(0, mu, 1, length(alpha));
    
    alpha_plus_z = alpha + z;
    d_a = -eta * (J(alpha_plus_z) - J(alpha)) * z;
    
    %plot(alpha, J(alpha), 'blue.-', 'markersize', 20);
    %hold on
end
