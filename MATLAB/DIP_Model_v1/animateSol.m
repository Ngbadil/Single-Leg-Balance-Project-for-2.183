function animateSol(tspan, x, p)
    figure; clf;
    hold on;
    plot([-1 1],[0 0],'k-'); 

    % Prepare plot handles
    h_OA = plot([0 0],[0 0],'LineWidth',2);
    h_AB = plot([0 0],[0 0],'LineWidth',2);

    xlabel('x'); ylabel('y');
    h_title = title('t=0.0s');
    
    axis equal
    axis([-1.5 1.5 -0.2 1.8]);
    %Step through and update animation
    for i = 1:length(tspan)
        % skip frame.
        if mod(i,10)
            continue;
        end
        t = tspan(i);
        z = x(:,i); 
        r = r_DIP(z,p);

        rO = r(1:2);    % foot position
        rA = r(3:4);    % hip position
        rB = r(5:6);    % head position

        set(h_title,'String',  sprintf('t=%.2f',t) ); % update title
        
        set(h_OA,'XData',[rO(1) rA(1)]);
        set(h_OA,'YData',[rO(2) rA(2)]);
        
        set(h_AB,'XData',[rA(1) rB(1)]);
        set(h_AB,'YData',[rA(2) rB(2)]);

        pause(.001)
    end
end