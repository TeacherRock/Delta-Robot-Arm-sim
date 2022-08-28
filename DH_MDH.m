function Output = DH_MDH(type)
switch(type)
    case 'DH'
        % DH
        theta_DH = [     0,  -pi/2,      0,      0,      0,      0];
        d_DH =     [  37.4,      0,      0,   34.5,      0,    7.4];
        a_DH =     [   2.5,     34,      4,      0,      0,      0];
        alpha_DH = [ -pi/2,      0,  -pi/2,   pi/2,  -pi/2,      0];
        Output = [theta_DH', d_DH', a_DH', alpha_DH'];
    case 'MDH' 
        % MDH
        theta_MDH = [ 0,      0,  -pi/2,      0,      0,      0,      0];
        d_MDH =     [ 0,   37.4,      0,      0,   34.5,      0,    7.4];
        b_MDH =     [ 0,    2.5,     34,      4,      0,      0,      0];
        alpha_MDH = [ 0,  -pi/2,      0,  -pi/2,   pi/2,  -pi/2,      0];
        Output =       [theta_MDH', d_MDH',  b_MDH', alpha_MDH'];
end
    
end
