%%Init 

x0 = [0 ; 0.9239; 2.2253; 3.0107; 0.5236; 0.8653; 0.3584; -1.0957; -2.3078; 2.0323] ;
alpha_0 = [pi/6; 0; 0; 0] ; 
beta_0 = zeros(4,1) ; 
z0 = [x0; alpha_0; beta_0] ; 

%% Linear Constraints 

% C1: Closed Loop Dynamics 
% C2: Hybrid Periodic 
% C3: Physcial Limitations 
% C4: Slipping Feasability 