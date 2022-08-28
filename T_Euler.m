function Output = T_Euler(Euler)
    alpha = Euler(:, 1);
    beta = Euler(:, 2); 
    gamma = Euler(:, 3);
    Output = [ cos(alpha).*cos(beta), cos(alpha).*sin(beta).*sin(gamma) - sin(alpha).*cos(gamma), ... 
               sin(alpha).*sin(gamma) + cos(alpha).*sin(beta).*cos(gamma);
               sin(alpha).*cos(beta), sin(alpha).*sin(beta).*sin(gamma) + cos(alpha).*cos(gamma), ... 
              -cos(alpha).*sin(gamma) + sin(alpha).*sin(beta).*cos(gamma);
              -sin(beta),    cos(beta).*sin(gamma),    cos(beta).*cos(gamma);];

end