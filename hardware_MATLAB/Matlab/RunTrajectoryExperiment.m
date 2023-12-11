function output_data = RunTrajectoryExperiment( angle1_init, angle2_init, pts_foot, traj_time, pre_buffer_time, post_buffer_time, gains, duty_max)
    
    % Figure for plotting motor data
    figure(1);  clf;       
    a1 = subplot(421);
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Angle 1 (rad)');  
    
    a2 = subplot(423);
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    ylabel('Velocity 1 (rad/s)');
    
    a3 = subplot(425);
    h3 = plot([0],[0]);
    h3.XData = []; h3.YData = [];
    ylabel('Current 1 (A)');
    hold on;
    subplot(425);
    h4 = plot([0],[0],'r');
    h4.XData = []; h4.YData = [];
    hold off;
    
    a4 = subplot(427);
    h5 = plot([0],[0]);
    h5.XData = []; h5.YData = [];
    ylabel('Duty Cycle 1');
    

    a5 = subplot(422);
    h21 = plot([0],[0]);
    h21.XData = []; h21.YData = [];
    ylabel('Angle 2 (rad)');
    
    a6 = subplot(424);
    h22 = plot([0],[0]);
    h22.XData = []; h22.YData = [];
    ylabel('Velocity 2 (rad/s)');
    
    a7 = subplot(426);
    h23 = plot([0],[0]);
    h23.XData = []; h23.YData = [];
    ylabel('Current 2 (A)');
    hold on;
    subplot(426);
    h24 = plot([0],[0],'r');
    h24.XData = []; h24.YData = [];
    hold off;
    
    a8 = subplot(428);
    h25 = plot([0],[0]);
    h25.XData = []; h25.YData = [];
    ylabel('Duty Cycle 2');
    
    % Figure for plotting state of the leg
    figure(2)
    clf
    hold on
    axis equal
    axis([-.25 .25 -.25 .25]);

    
    %TODO: Edit this to make it initialize plots for current system: Look
    %at how it is set in animate function in simulate_armWithDiscrete.m
    % old 
    % h_OB = plot([0],[0],'LineWidth',2);
    % h_AC = plot([0],[0],'LineWidth',2);
    % h_BD = plot([0],[0],'LineWidth',2);
    % h_CE = plot([0],[0],'LineWidth',2);
    % 
    % for two armsfcgsfgzfd
    h_AB = plot([0],[0],'LineWidth',2);
    h_BC = plot([0],[0],'LineWidth',2);
    h_ApBp = plot([0],[0],'LineWidth',2);
    h_BpCp = plot([0],[0],'LineWidth',2);
    %h_ellip = plot([0],[0],'g','LineWidth',1.5);
    

    h_foot= plot([0],[0],'Color',[0.7,0.7,0.7]);
    h_des = plot([0],[0],'--','Color',[0.5,0.5,0.5]);
    h_des.XData=[];
    h_des.YData=[];
    h_foot.XData=[];
    h_foot.YData=[];
   

    % Define leg length parameters 
    % TODO
    m1 =.2684;         m2 =.0338; 
    I1 = 0.0002189;      I2 = 0.00006130;
    l_OA=0.045;              l_AB=0.093; l_BC = 0.107;              
    l_A_m1=0.0836;         l_B_m2=0.0484; 
    Nmot = 18.75;
    Ir = 0.0035/Nmot^2;
    g = 9.81;
    

    p   = [m1 m2  I1 I2 Ir Nmot l_A_m1 l_B_m2 l_OA l_AB l_BC g]';
    % p   = [m1 m2  I1 I2 Ir N l_A_m1 l_B_m2 l_OA l_AB l_BC g]';
    


    % This function will get called any time there is new data from
    % the Nucleo board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        % Parse new data
        t = new_data(:,1);          % time
        pos1 = new_data(:,2);       % position
        vel1 = new_data(:,3);       % velocity
        cur1 = new_data(:,4);       % current
        dcur1 = new_data(:,5);      % desired current
        duty1 = new_data(:,6);      % command
        
        pos2 = new_data(:,7);       % position
        vel2 = new_data(:,8);       % velocity
        cur2 = new_data(:,9);       % current
        dcur2 = new_data(:,10);     % desired current
        duty2 = new_data(:,11);     % command
        
        x = new_data(:,12);         % actual foot position (negative due to direction motors are mounted)
        y = new_data(:,13);         % actual foot position
        xdes = -new_data(:,16);      % desired foot position (negative due to direction motors are mounted)
        ydes = new_data(:,17);      % desired foot position         
        
        N = length(pos1);
        % Update motor data plots
        h1.XData(end+1:end+N) = t;   
        h1.YData(end+1:end+N) = -pos1; % switch sign on all plotted values due to direction motors are mounted
        h2.XData(end+1:end+N) = t;   
        h2.YData(end+1:end+N) = -vel1;
        h3.XData(end+1:end+N) = t;   
        h3.YData(end+1:end+N) = -cur1;
        h4.XData(end+1:end+N) = t;   
        h4.YData(end+1:end+N) = -dcur1;
        h5.XData(end+1:end+N) = t;   
        h5.YData(end+1:end+N) = -duty1;
        
        h21.XData(end+1:end+N) = t;   
        h21.YData(end+1:end+N) = -pos2;
        h22.XData(end+1:end+N) = t;   
        h22.YData(end+1:end+N) = -vel2;
        h23.XData(end+1:end+N) = t;   
        h23.YData(end+1:end+N) = -cur2;
        h24.XData(end+1:end+N) = t;   
        h24.YData(end+1:end+N) = -dcur2;
        h25.XData(end+1:end+N) = t;   
        h25.YData(end+1:end+N) = -duty2;
        
        % Calculate leg state and update plots
        z = [pos1(end) pos2(end) vel1(end) vel2(end)]';
        keypoints = keypoints_leg(z,p); %TODO: Edit keypoints file
        %inertia_ellipse = inertia_ellipse_leg(z,p); %TODO: Edit keypoints file
        
        % TODO: Edit Keypoints based on above keypoints call
        rA = keypoints(:,1); 
        rB = keypoints(:,2);
        rC = keypoints(:,3);

        set(h_AB,'XData',[rA(1) rB(1)],'YData',[rA(2) rB(2)]);
        set(h_BC,'XData',[rB(1) rC(1)],'YData',[rB(2) rC(2)]);
        
        R = .05;
        rectangle("Position", [-pts_foot(1, 1)-R, -pts_foot(2,1)-R,  .1, .1], 'LineWidth', 0.05, 'Curvature', 1.0);
        %ellipse_x = inertia_ellipse(1,:) + rE(1);
        %ellipse_y = inertia_ellipse(2,:) + rE(2);
        %set(h_ellip,'XData',ellipse_x,'YData',ellipse_y);
        
        h_foot.XData(end+1:end+N) = x;
        h_foot.YData(end+1:end+N) = -y;
        h_des.XData(end+1:end+N) = xdes;
        h_des.YData(end+1:end+N) = ydes;
    end

    
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
    %params.timeout  = 2;            % end of experiment timeout
    
    % Parameters for tuning
    start_period                = pre_buffer_time;    % In seconds 
    end_period                  = post_buffer_time;   % In seconds
    
    K_xx                     = gains.K_xx; % Stiffness
    K_xy                     = gains.K_xy; % Stiffness


    D_xx                     = gains.D_xx; % Damping
    D_xy                     = gains.D_xy; % Damping
    


     K_yy                 = gains.K_yy; % Stiffness 
     D_yy                = gains.D_yy; % Damping



    
    



    
    % Specify inputs
    input = [start_period traj_time end_period];
    input = [input angle1_init angle2_init];
    input = [input K_xx K_yy K_xy D_xx D_yy  D_xy];
    input = [input duty_max];
    input = [input pts_foot(:)']; % final size of input should be 28x1
    
    params.timeout  = (start_period+traj_time+end_period);  
    
    output_size = 20;    % number of outputs expected
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    linkaxes([a1 a2 a3 a4],'x')
    
end