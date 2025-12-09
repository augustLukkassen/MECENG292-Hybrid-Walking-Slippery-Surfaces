% Animation of 3-link bipedal walker
% Simplified version similar to animate_two_link_walker.m
% Inputs:
%  t_sol: Array of time obtained from multi-step simulation
%  x_sol: Array of states obtained from multi-step simulation
%         x_sol(j, :)' is the state at time t_sol(j)
%         State: [x; y; q1; q2; q3; dx; dy; dq1; dq2; dq3]
%  t_I   : End of step indices into the time array (optional)
%         t_sol(t_I(1)) = end time of first step
%         t_sol(t_I(2)) = end time of second step
%         t_sol(t_I(end)) = end time of last step
function animate_three_link_walker(t_sol, x_sol, t_I)
    
    if nargin < 3 || isempty(t_I)
        t_I = length(t_sol);  % Animate entire trajectory if t_I not provided
    end
    
    % Robot parameters (should match dynamics_code_final_project.m)
    lL = 1.0;   % Leg length
    lT = 0.5;   % Torso length
    
    % Create figure
    f = figure(11);
    axis manual;
    
    % Position of stance foot at start of simulation
    % This gets updated at end of each step to enable continuous animation
    xst = 0;
    yst = 0;
    
    % Track which leg is stance (1 or 2)
    % Leg 1 starts as stance, switches after each impact
    current_stance_leg = 1;
    
    % Animation loop - similar structure to animate_two_link_walker
    for j = 1:length(t_I)
        if j == 1
            ind_start = 1;
        else
            ind_start = t_I(j-1) + 1;
        end
        
        for k = ind_start:t_I(j)
            if k > size(x_sol, 1)
                break;
            end
            
            % Extract state variables
            % State x, y is the hip position in global coordinates
            xH = x_sol(k, 1);   % Hip x position
            yH = x_sol(k, 2);   % Hip y position
            q1 = x_sol(k, 3);   % Leg 1 angle (relative to torso)
            q2 = x_sol(k, 4);   % Leg 2 angle (relative to torso)
            q3 = x_sol(k, 5);   % Torso angle
            
            % Compute foot positions from hip
            % Leg 1 foot position (using dynamics convention)
            angle1 = q1 + q3 - 3*pi/2;  % Matches pLeg1_gen formula
            xLeg1 = xH - lL * cos(angle1);
            yLeg1 = yH + lL * sin(angle1);
            
            % Leg 2 foot position (using dynamics convention)
            angle2 = q2 + q3 - pi/2;  % Matches pLeg2_gen formula
            xLeg2 = xH + lL * cos(angle2);
            yLeg2 = yH - lL * sin(angle2);
            
            % Determine stance and swing foot based on current_stance_leg
            if current_stance_leg == 1
                % Leg 1 is stance, leg 2 is swing
                xst_foot = xLeg1;
                yst_foot = 0;  % Stance foot on ground
                xsw = xLeg2;
                ysw = yLeg2;   % Swing foot position
            else
                % Leg 2 is stance, leg 1 is swing
                xst_foot = xLeg2;
                yst_foot = 0;  % Stance foot on ground
                xsw = xLeg1;
                ysw = yLeg1;   % Swing foot position
            end
            
            % Torso endpoint
            xTorso = xH + lT * sin(q3);
            yTorso = yH + lT * cos(q3);
            
            % Plot points (similar to original animate_two_link_walker)
            pts = [xst_foot yst_foot;  % Stance foot (on ground)
                   xH yH;              % Hip
                   xsw ysw;            % Swing foot
                   xTorso yTorso];     % Torso endpoint
            
            clf;
            hold on;
            
            % Draw ground
            line([-1 10], [0 0], 'Color', [0 0 0], 'LineWidth', 2);
            
            % Draw robot
            % Stance leg (thicker line, blue)
            plot([pts(1,1), pts(2,1)], [pts(1,2), pts(2,2)], 'b-', 'LineWidth', 4);
            plot(pts(1,1), pts(1,2), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
            
            % Swing leg (thinner line, red)
            plot([pts(2,1), pts(3,1)], [pts(2,2), pts(3,2)], 'r-', 'LineWidth', 3);
            plot(pts(3,1), pts(3,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            
            % Torso (green)
            plot([pts(2,1), pts(4,1)], [pts(2,2), pts(4,2)], 'g-', 'LineWidth', 4);
            
            % Hip (black)
            plot(pts(2,1), pts(2,2), 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
            
            axis([-1 9 -1 2]);
            grid on;
            drawnow;
            pause(0.001);
        end
        
        % Update stance foot position after each step
        % The swing foot becomes the new stance foot
        if t_I(j) <= size(x_sol, 1)
            % Get final state
            k_end = t_I(j);
            xH_end = x_sol(k_end, 1);
            yH_end = x_sol(k_end, 2);
            q1 = x_sol(k_end, 3);
            q2 = x_sol(k_end, 4);
            q3 = x_sol(k_end, 5);
            
            % Compute swing foot position (which becomes new stance)
            if current_stance_leg == 1
                % Leg 2 was swing, becomes new stance
                angle2 = q2 + q3 - pi/2;
                xst = xH_end + lL * cos(angle2);
                yst = 0;  % On ground
            else
                % Leg 1 was swing, becomes new stance
                angle1 = q1 + q3 - 3*pi/2;
                xst = xH_end - lL * cos(angle1);
                yst = 0;  % On ground
            end
            
            % Switch stance leg
            current_stance_leg = 3 - current_stance_leg;  % 1->2 or 2->1
        end
    end
end
