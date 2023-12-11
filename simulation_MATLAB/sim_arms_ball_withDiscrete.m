   
function main()
    % Generate Ky, 100 points from 1 to 1000
%     n = 10;
%     Ky = linspace(1,600, n);
% 
%     y_max = zeros(n, n);
%     for i = 1:n
%         for j = 1:n
%             y_max(i, j) = simulate_Arms(Ky(i), Ky(j));
%     
%         end
%     end
% 
% 
%     % Plotting the y_max array
%     [Ky_down, Ky_up] = meshgrid(Ky, Ky);
%     surf(Ky_down, Ky_up, y_max);
%     xlabel('Ky_{ascending}');
%     ylabel('Ky_{descending}');
%     zlabel('y_{max}');
%     title('3D Surface Plot of y_{max} as a Function of Ky_{down} and Ky_{up}');
%     zlim([-0.05 0.6]);
% 
% %     figure(2)
% %     plot(Ky_down, y_max(5,:))
% %     figure(3); clf; 
% %     plot(Ky_down, y_max(:,5 ))
     for i = 1:1000
        y_max = simulate_Arms(150,1000,  true);
     end
    
end

function y_max = simulate_Arms(Ky_down, Ky_up, anim)
    %% Definte fixed paramters

    
    m1 = 0.2684;         m2 = 0.0338; 
    I1 =  0.0002189;      I2 = 0.00006130;
    l_OA=0.05;              l_AB= 0.093;  l_BC = 0.107;   %TODO: Change l_OA depending on where we mount the base
    l_A_m1=0.0836;           l_B_m2=0.0484; 
    N = 18.75;
    Ir = 0.0035/N^2;
    g = 9.81; 
   
    

    %% Parameter vector
    p   = [m1 m2  I1 I2  Ir N l_A_m1 l_B_m2 l_OA l_AB l_BC g]';
       
    %% Simulation Parameters Set 2 -- Operational Space Control
 
    p_traj.x_0   = 0.005;
    p_traj.y_0   = 0.12;
  
    
    % points that work (m, k), (0.5, 10), (1, 1), (2, 1)
    % with k=1, drops it at 4
    ball_params.mass = .2; % units tbd
    ball_params.radius = 0.02; %To be changed 
    ball_params.position = .6; %To be changed 
    ball_params.velocity = 0;



    k = 10; % Note: k * ball mass = 1 for best results

    control_params.Kxe = 5000; % make arm hard to move from side to side
    control_params.Dxe = 1000; 
    control_params.Kye = 100;
    control_params.Dye = 20; 

   

    phase.ascending = false; 
    phase.descending = false;

    
    
    
   

    restitution_coeff = 0;
 

    %% Perform Dynamic simulation
    dt = 0.0003;
    tf = 2.0;
    num_step = floor(tf/dt);
    tspan = linspace(0, tf, num_step); 
    z0 = [pi/3; 2*pi/3; 0; 0];
    z_out = zeros(4,num_step);
    
    ball0 = [ball_params.position, ball_params.velocity]; % y height and velocity only because x-forces cancel out here
    zball_out = zeros(2, num_step);

    z_out(:,1) = z0;
    zball_out(:,1) = ball0;
    ddy = 0;
    
    Fc_arr = zeros(1, num_step);
    Fc_arr(1,1) = 0;
    


    
    

    for i=1:num_step-1
        

        %Let the controller settle down before ball falls down| 
        % We start ball descending phase when change in y of end-effector is greater than some threshold 
        % but we don't want to detect initial movement caused be controller  as descending phase
    
        if tspan(i) >= 0.5
            ddy = -g;
        end

        
     
        
        %Acceleration update:
        dz = dynamics(tspan(i), z_out(:,i), p, p_traj, control_params, phase, Ky_down, Ky_up);
        
        %Velocity Update Arm
        z_out(:,i+1) = z_out(:,i) + dz*dt;
        
        
        %Velocity update Ball
        zball_out(2, i+1) = zball_out(2, i) + ddy * dt;
        zball_out(1, i+1) = zball_out(1,i) + zball_out(2, i)*dt;
        

        %Check for Impact
        [contact,Fc, vy_ball, dq] = discrete_impact_contact(z_out(:,i+1), p, zball_out(1, i+1), zball_out(2, i+1), ball_params, restitution_coeff);
        Fc_arr(:,i+1) = Fc;
        
      
        z_out(3:4, i+1) = dq;

        zball_out(2, i+1) = vy_ball;

        %Position update
        z_out(1:2,i+1) = z_out(1:2,i) + z_out(3:4,i+1)*dt;
        
        
        zball_out(1, i+1) = zball_out(1, i) + zball_out(2, i+1)*dt;
        %Task space Constraint: 
%         y_min = 0.04; y_max = 0.175;
%         qdot = task_limit(z_out(:, i+1),p, [y_min  y_max]);
%         z_out(3:4, i+1) = qdot;

       
    end

    
    %% Compute Energy
    E = energy_Arms(z_out,p);
%     figure(1); clf
%     plot(tspan,E);xlabel('Time (s)'); ylabel('Energy (J)');
    
    %% Compute foot position over time
    rE = zeros(2,length(tspan));
    vE = zeros(2,length(tspan));
    for i = 1:length(tspan)
        rE(:,i) = position_Endeffector(z_out(:,i),p);
        vE(:,i) = velocity_Endeffector(z_out(:,i),p);
    end
    
%     figure(2); clf;
%     plot(tspan,zball_out(1,:),'r','LineWidth',2)
%     hold on
%     plot(tspan,rE(2,:),'b','LineWidth',2)
%     xlabel('Time (s)'); ylabel('Position (m)'); legend({'ball height','y_E'});
    
   

    
%     figure(3); clf;
%     plot(tspan,vE(1,:),'r','LineWidth',2)
%     hold on
%     plot(tspan,vE(2,:),'b','LineWidth',2)
%     
%     xlabel('Time (s)'); ylabel('Velocity (m)'); legend({'vel_x','vel_y'});
%     
%     figure(4)
%     plot(tspan,z_out(1:2,:)*180/pi)
%     legend('q1','q2');
%     xlabel('Time (s)');
%     ylabel('Angle (deg)');
%     % 
%     figure(5)
%     plot(tspan,z_out(3,:)*180/pi)
%     legend('q1dot');
%     xlabel('Time (s)');
%     ylabel('Angular Velocity (deg/sec)');
%     figure(6)
%     plot(tspan,z_out(4,:)*180/pi)
%     legend('q2dot');
%     xlabel('Time (s)');
%     ylabel('Angular Velocity (deg/sec)');
% 
%     
    
    
    
    
    
%     figure(7); clf;
%     plot(tspan, Fc_arr(1,:))
%     legend('Impact Force', 'r','LineWidth',2);
%     xlabel('Time (s)');
    
% 
%         
%     % Find the time indices where the ball hits the ground (or near zero height)
    % Find the collision points where the ball is below the threshold of y_0
    collisionPt = find(zball_out(1,:) < p_traj.y_0);
    % Find the index of the first bounce
    first_bounce_index = collisionPt(1);
    % Initialize y_max to zero
    y_max = 0;
    % Check if any point in zball_out is less than constant c
    if any(zball_out(1,:) < ball_params.radius)
        % If such a point exists, set y_max to 0
        y_max = 0;
    else
        % Otherwise, find the next maximum height after the first bounce
        [next_max_height, idx] = max(zball_out(1, first_bounce_index:end));
        % Set y_max to the next maximum height
        y_max = next_max_height;
    end

    % Display the next maximum height
%     disp(Ky_down)
%     disp(Ky_up)
%     disp(['y_{max}  ', num2str(next_max_height), ' meters']);
 
  
    

%% Animate Solution
    if anim == true 
        figure(8); clf;
        hold on
        animateSol(tspan, z_out, zball_out, p, ball_params);
    end
    

   
    
 


    
    
end

function tau = control_law(t, z, p, p_traj,  control_params, phase)
    
    
   

    % Controller gains
    K_x = control_params.Kxe ; % Spring stiffness X
    D_x = control_params.Dxe ;  % Damping X

    K_y = control_params.Kye ; % Spring stiffness Y
    D_y = control_params.Dye;  % Damping Y




   % Desired Position
    rEd = [p_traj.x_0 p_traj.y_0 0]';
    % Compute desired velocity of foot
    vEd = [0 0 0]';
    % Desired acceleration
    aEd =[0 0 0]';
   
    % Actual position and velocity 
    rE = position_Endeffector(z,p);
    vE = velocity_Endeffector(z,p);

    % Khatib's Operational space controller setup
    J  = jacobian_Endeffector(z,p);
    dJ = jacobian_dot_Endeffector(z,p);
   
    M = A_Arms(z,p);
    V = Corr_Arms(z,p);
    G = Grav_Arms(z,p);
    K = [K_x 0; 0 K_y];
    D = [D_x 0; 0 D_y];
    dq = z(3:4);
    
    lambda = inv(J*inv(M)*J');
    mu = lambda*J*inv(M)*V -  lambda* dJ* dq;
    rho = lambda*J*inv(M)*G;
    %Control law
    tau = J' * (lambda*( aEd(1:2) + K*(rEd(1:2)  - rE) + D*(vEd(1:2) - vE) )  + rho+ mu) ;

    %Set torque limit (-2,tau,2)
    tau_max =10;
    tau = [max(-tau_max, min(tau(1,1), tau_max)); max(-tau_max, min(tau(2,1), tau_max))];

end




function dz = dynamics(t,z,p,p_traj, control_params, phase, Ky_down, Ky_up)
    % Get mass matrix
    A = A_Arms(z,p);

    rE = position_Endeffector(z,p);
    vE = velocity_Endeffector(z,p);

    if (t> 0.5) && (p_traj.y_0 - rE(2) > 0.005) &&  vE(2) < 0
        phase.descending = true;
    end

    if (t> 1) && vE(2) >= 0 &&   rE(2) < 0.2
        phase.ascending = true;
    end



    

%     p_traj.y_0 = 0.13; %Default y set point for controller at 0.1m 

	if   phase.descending
	    % Descending Phase (end-effector is below desired position by some delta and  ball is moving downwards)
	    control_params.Kye = Ky_down;
        control_params.Dye =(Ky_down)^(1/3);
% p_traj.y0 = 0.05;
    

	elseif  phase.ascending
	    % Ascending Phase (end-effector moving upward and we are at low y-value OR  end-effector reached physical task-space limit)
        control_params.Kye = Ky_up;
        control_params.Dye = (Ky_up)^(1/3);
% p_traj.y_0 = 0.18; %change set point to highest point
    end

    

    % Compute Controls
    tau = control_law(t,z,p,p_traj,control_params, phase);
    
    % Get b = Q - V(q,qd) - G(q)
    b = b_Arms(z,tau,p);

    
    % Solve for qdd.
    qdd = A\(b);

    
    dz = 0*z;

    

    % Form dz
    dz(1:2) = z(3:4);
    dz(3:4) = qdd;
end




function animateSol(tspan, x, zball_out, p, ball_params)
    % Prepare plot handles
    hold on
    h_AB = plot([0],[0],'LineWidth',4);
    h_BC = plot([0],[0],'LineWidth',4);
    h_ApBp = plot([0],[0],'LineWidth',4);
    h_BpCp = plot([0],[0],'LineWidth',4);
    
    h_circleA = plot(0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    h_circleB = plot(0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    h_circleC = plot(0, 0, 'o', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    h_circleAp = plot(0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    h_circleBp = plot(0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    h_circleCp = plot(0, 0, 'o', 'MarkerSize', 12, 'MarkerFaceColor', 'r');


    center = [0, zball_out(1, 1)];
    
    R = ball_params.radius; % radius of circle
    ball = rectangle('Position', [center(1) - R, center(2) - R, 2 * R, 2 * R], 'Curvature', 1, 'FaceColor',	"#EDB120");
  
    xlabel('x'); ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    axis([-.2 .2 -.1 0.6]);
    
    %Step through and update animation
    for i = 1:length(tspan)
        % skip frame.
        if mod(i,10)
            continue;
        end
        t = tspan(i);
        z = x(:,i); 
        keypoints = keypoints_Arms(z,p);

        rA =  keypoints(:,1);
        rB = keypoints(:,2);
        rC = keypoints(:,3); % Vector to tip of pendulum
        rAp = keypoints(:,4);
        rBp = keypoints(:,5);
        rCp = keypoints(:,6);

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        set(h_AB,'XData',[rA(1) rB(1)]);
        set(h_AB,'YData',[rA(2) rB(2)]);
        
        set(h_BC,'XData',[rB(1) rC(1)]);
        set(h_BC,'YData',[rB(2) rC(2)]);

        set(h_ApBp,'XData',[rAp(1) rBp(1)]);
        set(h_ApBp ,'YData',[rAp(2) rBp(2)]);

        set(h_BpCp,'XData',[rBp(1) rCp(1)]);
        set(h_BpCp,'YData',[rBp(2) rCp(2)]);
        
        center = [0, zball_out(1, i)];
        ball.Position = [center(1) - R, center(2) - R, 2 * R, 2 * R];
        
        set(h_circleA, 'XData', rA(1), 'YData', rA(2));
        set(h_circleB, 'XData', rB(1), 'YData', rB(2));
        set(h_circleC, 'XData', rC(1), 'YData', rC(2));

        set(h_circleAp, 'XData', rAp(1), 'YData', rAp(2));
        set(h_circleBp, 'XData', rBp(1), 'YData', rBp(2));
        set(h_circleCp, 'XData', rCp(1), 'YData', rCp(2));
    
        

        
        pause(.01)
    end
end



function qdot = task_limit(z, p, y_lim)
    rE = position_Endeffector(z,p);
    vE = velocity_Endeffector(z,p);
    qdot1 = z(3); qdot2 = z(4);
   
    y_min = y_lim(1); y_max = y_lim(2);
    
    if (rE(2)<y_min && vE(2) <0) ||  (rE(2) > y_max && vE(2) > 0)
        qdot1 = 0; qdot2 = 0;   
    end
    qdot = [qdot1; qdot2];
end



function [contact,Fc_to_foot, vb_post, qdot] = discrete_impact_contact(z,p,y_ball, dy_ball, ball_params, rest_coeff)
    rE = position_Endeffector(z,p);
	vE = velocity_Endeffector(z,p);
    C_y = y_ball - ball_params.radius- rE(2) ;
    dC_y = dy_ball-  vE(2);
    J = jacobian_Endeffector(z,p);
    M = A_Arms(z,p);
    dq = z(3:4);
    Fcy_hat = 0;
    vb_post = dy_ball;
    
    
    contact = false;
    	%In case of collision: 
    if  (C_y < 0 && dC_y <0 && rE(1) < ball_params.radius)
        %Collision with end-effector
        contact = true;
        J_y= J(2,:);
	    lambda_y = (J_y*inv(M)*J_y'); 
	    m_e = 1/(lambda_y); 
        m_b = ball_params.mass;
	    vb_pre = dy_ball; 
        ve_pre = vE(2);

	    eta = - m_b/m_e; 
	    alpha = (m_b/m_e)* vb_pre + ve_pre;
	    beta = rest_coeff*(vb_pre - ve_pre);
	    vb_post = (alpha - beta)/(1- eta);
	    ve_post = eta*vb_post + alpha; 
	    
	    Fcy_hat = m_e*(ve_post - ve_pre);

	    dq = dq+ M\(J_y'*Fcy_hat);
  

    else  
        %Collision with ground
        if y_ball - ball_params.radius < 0
            vb_post = -rest_coeff*dy_ball;
        end
    end


	qdot = dq;
	Fc_to_foot = Fcy_hat; 
end