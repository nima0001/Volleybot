clear
name = 'Arms';

% Define variables for time, generalized coordinates + derivatives, controls, and parameters 
syms t th1 th2 dth1 dth2 ddth1 ddth2 real
syms m1 m2 I1 I2 l_A_m1 l_B_m2  g real
syms l_OA l_AB l_BC real  %scalar distances (offset and link lengths)
syms tau1 tau2 Fx Fy real
syms Ir N real

% Group them
q   = [th1  ; th2 ];      % generalized coordinates
dq  = [dth1 ; dth2];    % first time derivatives
ddq = [ddth1;ddth2];  % second time derivatives
u   = [tau1 ; tau2];     % controls
F   = [Fx ; Fy];

p   = [m1 m2 I1 I2 Ir N l_A_m1 l_B_m2  l_OA  l_AB l_BC g]';        % parameters
% Generate Vectors and Derivativess
ihat = [1; 0; 0];
jhat = [0; 1; 0];

khat = cross(ihat,jhat);
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; % a handy anonymous function for taking time derivatives


AB = [l_AB * cos(th1);  l_AB * sin(th1); 0];
BC = [l_BC * cos(th1 + th2);  l_BC * sin(th1 + th2); 0];

th1p = pi- th1; th2p = -th2;
ApBp = [l_AB * cos(th1p);  l_AB * sin(th1p); 0];
BpCp = [l_BC * cos(th1p + th2p);  l_BC * sin(th1p + th2p); 0];

rA = l_OA* ihat;
rB = rA + AB;
rC = rB + BC;

rAp = -rA; 
rBp = [ApBp(1)+rAp(1); ApBp(2)+rAp(2); 0];
rCp = rBp + BpCp;




r_m1 = rA + AB*l_A_m1/l_AB;
r_m2 = rB + BC*l_B_m2/l_BC;



drB = ddt(rB);
drC = ddt(rC);


dr_m1 = ddt(r_m1);
dr_m2 = ddt(r_m2);


% Calculate Kinetic Energy, Potential Energy, and Generalized Forces
F2Q = @(F,r) simplify(jacobian(r,q)'*(F));    % force contributions to generalized forces
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));   % moment contributions to generalized forces


T1 = (1/2)*m1 * dot(dr_m1,dr_m1) + (1/2) * I1 * dth1^2;
T2 = (1/2)*m2 * dot(dr_m2,dr_m2) + (1/2) * I2 * (dth1 + dth2)^2;
T1r = (1/2)*Ir*(N*dth1)^2;
T2r = (1/2)*Ir*(dth1 + N*dth2)^2;

Vg1 = m1*g*dot(r_m1, jhat);
Vg2 = m2*g*dot(r_m2, jhat);


T = simplify(T1 + T2 + T1r + T2r);
Vg = Vg1 + Vg2;

Q_tau1 = M2Q(tau1*khat,dth1*khat);
Q_tau2 = M2Q(tau2*khat,(dth1 + dth2)*khat); 
Q_tau = Q_tau1+Q_tau2;


Q = Q_tau;

% Assemble the array of cartesian coordinates of the key points
keypoints = [rA(1:2) rB(1:2) rC(1:2) rAp(1:2) rBp(1:2) rCp(1:2)];

%% All the work is done!  Just turn the crank...
% Derive Energy Function and Equations of Motion
E = T+Vg;
L = T-Vg;
eom = ddt(jacobian(L,dq).') - jacobian(L,q).' - Q;




% Rearrange Equations of Motion
A = simplify(jacobian(eom,ddq));
b = A*ddq - eom;

% Equations of motion are
% eom = A *ddq + (coriolis term) + (gravitational term) - Q = 0
Mass_Joint_Sp = A
Grav_Joint_Sp = simplify(jacobian(Vg, q)');
Corr_Joint_Sp = simplify( eom + Q - Grav_Joint_Sp - A*ddq);

% Compute foot jacobian
J = jacobian(rC,q);

% Compute ddt( J )
dJ= reshape( ddt(J(:)) , size(J) );

% Write Energy Function and Equations of Motion
z  = [q ; dq];

rEnd = rC(1:2);
drEnd = drC(1:2);
J  = J(1:2,1:2)
dJ = dJ(1:2,1:2);
% 
% matlabFunction(A,'file',['A_' name],'vars',{z p});
% matlabFunction(b,'file',['b_' name],'vars',{z u p});
% matlabFunction(E,'file',['energy_' name],'vars',{z p});
% matlabFunction(rEnd,'file',['position_Endeffector'],'vars',{z p});
% matlabFunction(drEnd,'file',['velocity_Endeffector'],'vars',{z p});
% matlabFunction(J ,'file',['jacobian_Endeffector'],'vars',{z p});
% matlabFunction(dJ ,'file',['jacobian_dot_Endeffector'],'vars',{z p});
% 
% matlabFunction(Grav_Joint_Sp ,'file', ['Grav_Arms'] ,'vars',{z p});
% matlabFunction(Corr_Joint_Sp ,'file', ['Corr_Arms']     ,'vars',{z p});
% matlabFunction(keypoints,'file',['keypoints_' name],'vars',{z p});
% 
% 
% 
