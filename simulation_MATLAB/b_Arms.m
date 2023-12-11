function b = b_Arms(in1,in2,in3)
%b_Arms
%    B = b_Arms(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    04-Dec-2023 11:37:47

dth1 = in1(3,:);
dth2 = in1(4,:);
g = in3(12,:);
l_AB = in3(10,:);
l_A_m1 = in3(7,:);
l_B_m2 = in3(8,:);
m1 = in3(1,:);
m2 = in3(2,:);
tau1 = in2(1,:);
tau2 = in2(2,:);
th1 = in1(1,:);
th2 = in1(2,:);
t2 = cos(th1);
t3 = sin(th2);
t4 = th1+th2;
t5 = cos(t4);
b = [tau1+tau2+dth2.*(dth1.*l_AB.*l_B_m2.*m2.*t3.*2.0+dth2.*l_AB.*l_B_m2.*m2.*t3)-g.*m2.*(l_AB.*t2+l_B_m2.*t5)-g.*l_A_m1.*m1.*t2;tau2-g.*l_B_m2.*m2.*t5-dth1.^2.*l_AB.*l_B_m2.*m2.*t3];
