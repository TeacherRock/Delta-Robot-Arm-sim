function Delta_Limitation = Delta_Constraint

    PosLimit = [ [-170    170] ;
                 [-105    133] ;
                 [-205     65] ;
                 [-190    190] ;
                 [-120    120] ;
                 [-360    360] ] * pi / 180;
                
    GearRatio = [ ( 2835 / 32 ) , 121 , 81 , ( 2091 / 31 ) , ( 2244 / 30 ) , ( 2244 / 53 ) ] ;
    
    RatedSpeed  =  3000 / 9.55 ;  % 馬達額定轉速 (rad/s) , 單位轉換: (rad/s) = (rpm) / 9.55
    
    VelLimit    =  ( RatedSpeed ./ GearRatio ) * 0.7 ;   % [rad/s]

    AccLimit    =  VelLimit * 3 ; % [rad/(s^2)]

    JerkLimit   =  AccLimit * 3 ; % [rad/(s^3)]

    PosCLimit   =  [ -66    71  ;    % X
                     -66    71  ;    % Y
                     -20.6  105.9;  ] ;  % Z
    
    VelCLimit   =  [  100  ;  100  ;  100  ] ;
    
    AccCLimit   =  [  1000   ;  1000   ;  1000  ] ;
    
    JerkCLimit  =  [  1000   ;  1000   ;  1000  ] ;
    
    % >>>> package
    Joint     = struct('Pos',  PosLimit,...
                       'Vel',  VelLimit,...
                       'Acc',  AccLimit,...
                       'Jerk', JerkLimit);
    
    Cartesian = struct('Pos',  PosCLimit,...
                       'Vel',  VelCLimit,...
                       'Acc',  AccCLimit,...
                       'Jerk', JerkLimit);
    
    Delta_Limitation = struct('Joint', Joint, 'Cartesian', Cartesian);

end