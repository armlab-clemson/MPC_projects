function plot_car2(xx,xx1,xs,N,L,t,u_cl)
x_r_1 = [];
y_r_1 = [];

%% Parking Line
 
 y_p1= 25.25*ones(1,100);
 y_p2= 21.75*ones(1,100);
 y_p3= 18.25*ones(1,100);
 y_p4= 14.75*ones(1,100);
 x_p= linspace (19,21);
 y_p5= linspace(14.75,25.25);
 x_p2= 21*ones(1,100);
 
 %% Obstacles
 w= 1.34;
 x_obs= 20; y_obs1= 23.5; y_obs2= 16.5;
 
 x_obs_rect= [x_obs+w/2, x_obs+w/2, x_obs-w/2, x_obs-w/2];
 y_obs1_rect= [y_obs1-L/2, y_obs1+L/2, y_obs1+L/2, y_obs1-L/2];
 y_obs2_rect= [y_obs2-L/2, y_obs2+L/2, y_obs2+L/2, y_obs2-L/2];
 
%% Reference Point and Pose
 x1= xs(1); y1= xs(2); th1= xs(3);
 h_t = L/2; w_t=L/2.5;
 xref_rect = [x1 + w/2, x1 + w/2,x1 - w/2,x1 - w/2];
 yref_rect = [y1 - L/2, y1 + L/2, y1 + L/2, y1 - L/2];
 
 xref_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];
 yref_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];
 
 figure(1)
 set(gcf,'PaperPositionMode','auto')
 set(gcf, 'Color', 'w');
 set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);
 for k=1:size(xx,2)
    %% Ego Car
    x2= xx(1,k,1); y2= xx(2,k,1); th2= xx(3,k,1);
    xego_rect = [ - L/2,  L/2, L/2,- L/2];
    yego_rect = [w/2,  w/2, - w/2, - w/2];
    xego= xego_rect*cos(th2)-yego_rect*sin(th2);
    yego= xego_rect*sin(th2)+yego_rect*cos(th2);
    xego= xego+x2; yego= yego+y2;
    xego_tri = [ x2+h_t*cos(th2), x2+(w_t/2)*cos((pi/2)-th2), x2-(w_t/2)*cos((pi/2)-th2)];
    yego_tri = [ y2+h_t*sin(th2), y2-(w_t/2)*sin((pi/2)-th2), y2+(w_t/2)*sin((pi/2)-th2)];
    x_r_1 = [x_r_1 x2];
    y_r_1 = [y_r_1 y2];
    %% PLOT
    plot(x_p,y_p1,'-k',x_p,y_p2,'-k',x_p,y_p3,'-k',x_p,y_p4,'-k',x_p2,y_p5,'-k','LineWidth',5,'HandleVisibility','off');
    hold on
    fill(x_obs_rect,y_obs1_rect,'r','HandleVisibility','off');
    hold on
    fill(x_obs_rect,y_obs2_rect,'r','HandleVisibility','off');
    hold on
    fill(xref_rect,yref_rect,'k','HandleVisibility','off');
    
    hold on
    fill(xref_tri,yref_tri,'y','HandleVisibility','off');
   
    hold on
    fill(xego, yego,'b','HandleVisibility','off');
   
    hold on
    fill(xego_tri,yego_tri,'w','HandleVisibility','off');
    
    hold on
    plot(x_r_1,y_r_1,'-r','linewidth',1.5);
    hold on
        if k < size(xx,2) 
            plot(xx1(1:N,1,k),xx1(1:N,2,k),'r--*')
        end
    hold off
    axis([-5 25 -5 25])
    set(gca,'fontsize',10)
    title ('Parking Maneuver','FontSize',10);
    ylabel('Lateral Displacement (m)','FontSize',10); xlabel('Longitudinal Displacement (m)','FontSize',10);
    
    pause(0.1)
    box on;
    grid on
    drawnow
    F(k) = getframe(gcf);
 end
 close(gcf);
 


figure(2)
subplot(211)
stairs(t,u_cl(:,1),'k','linewidth',1.5); axis([0 t(end) -4.5 17])
ylabel('v (rad/s)')
grid on
subplot(212)
stairs(t,u_cl(:,2),'r','linewidth',1.5); axis([0 t(end) -1.5 1.5])
xlabel('time (seconds)')
ylabel('\delta (rad/s)')
grid on

end