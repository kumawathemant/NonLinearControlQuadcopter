function [figs] = parametric(t,x,args)
    figs = [];
    %try

        if args.AdaptiveLaw == 5
                figs(1)  = figure ;

                l = tiledlayout('flow');
                r = '';
                s = '';
                switch args.Reference
                    case 1
                        r = 'r_{scalar}';
                    case 2
                        r = 'r_{sinusodal}';
                    case 3
                        r = 'r_{steps}';
                end
                if args.Noise
                    s = ' With Noise';
                end
                a =''
                a = ' Non Linear system and non linear Adaptive';
                title(l,['Quadcopter Trajectory with',a,' Controller (',r,s,')'])
                l = nexttile;
                title(l,'Position')
                hold on
                plot(t,x(:,83));
                plot(t,x(:,84));
                plot(t,x(:,85));
                plot(t,x(:,13),'--');
                plot(t,x(:,14),'--');
                plot(t,x(:,15),'--');
                ylabel('Magnitude (m)');
                xlabel('Time (s)');
                hold off
                grid on
                legend('x','y','z','x_m','y_m','z_m','Interperter','latex');

                l = nexttile;
                title(l,'Position Error')
                hold on
                plot(t,x(:,83)-x(:,13));
                plot(t,x(:,84)-x(:,14));
                plot(t,x(:,85)-x(:,15));
                ylabel('Magnitude (m)');
                xlabel('Time (s)');
                hold off
                grid on
                legend('x-x_m','y-y_m','z-z_m','Interperter','latex');

    

                l = nexttile;
                title(l,'Kx')
                hold on
                plot(t,x(:,19:42));
      
                ylabel('Magnitude');
                xlabel('Time (s)');
                hold off
                grid on
                
      
                l = nexttile;
                title(l,'Kr')
                hold on
                plot(t,x(:,43:48));
                ylabel('Magnitude');
                xlabel('Time (s)');
                hold off
                grid on

        elseif args.AdaptiveLaw  == 1 | args.AdaptiveLaw  == 2 | args.AdaptiveLaw ==3 | args.AdaptiveLaw ==4
            
                figs(1)  = figure ;

                l = tiledlayout('flow');
                r = '';
                s = '';
                switch args.Reference
                    case 1
                        r = 'r_{scalar}';
                    case 2
                        r = 'r_{sinusodal}';
                    case 3
                        r = 'r_{steps}';
                end
                if args.Noise
                    s = ' With Noise';
                end
                a =''
                if args.AdaptiveLaw  == 2
                    a = ' Adaptive';
                    title(l,['Quadcopter Trajectory with',a,' Linear Controller (',r,s,')'])
                elseif args.AdaptiveLaw  == 3
                    a = ' Non Linear system and';
                    title(l,['Quadcopter Trajectory with',a,' Linear Controller (',r,s,')'])
                elseif args.AdaptiveLaw  == 4
                    a = ' Non Linear system and Adaptive';
                    title(l,['Quadcopter Trajectory with',a,' Linear Controller (',r,s,')'])
     
                end 

                
                
                l = nexttile;
                title(l,'Position')
                hold on
                plot(t,x(:,1));
                plot(t,x(:,2));
                plot(t,x(:,3));
                plot(t,x(:,13),'--');
                plot(t,x(:,14),'--');
                plot(t,x(:,15),'--');
                ylabel('Magnitude (m)');
                xlabel('Time (s)');
                hold off
                grid on
                legend('x','y','z','x_m','y_m','z_m','Interperter','latex');

                l = nexttile;
                title(l,'Position Error')
                hold on
                plot(t,x(:,1)-x(:,13));
                plot(t,x(:,2)-x(:,14));
                plot(t,x(:,3)-x(:,15));
                ylabel('Magnitude (m)');
                xlabel('Time (s)');
                hold off
                grid on
                legend('x-x_m','y-y_m','z-z_m','Interperter','latex');

                if args.AdaptiveLaw == 2

                end 

                l = nexttile;
                title(l,'Rotation')
                hold on
                plot(t,x(:,4));
                plot(t,x(:,5));
                plot(t,x(:,6));
                plot(t,x(:,16),'--');
                plot(t,x(:,17),'--');
                plot(t,x(:,18),'--');
                ylabel('Magnitude (rad)');
                xlabel('Time (s)');
                hold off
                grid on
                legend('$\phi$','$\theta$','$\psi$','$\phi_m$','$\theta_m$','$\psi_m$','Interpreter','latex');
                l = nexttile;
                
                title(l,'Rotational Error')
                hold on
                plot(t,x(:,4)-x(:,16));
                plot(t,x(:,5)-x(:,17));
                plot(t,x(:,6)-x(:,18));
                ylabel('Magnitude (rad)');
                xlabel('Time (s)');
                hold off
                grid on
                legend('$\phi$-$\phi_m$','$\theta$-$\theta_m$','$\psi$-$\psi_m$','Interpreter','latex');


                saveas(figs(1),['Parametric_',r,'_law',num2str(args.AdaptiveLaw),'.png']);
                
               
                if args.AdaptiveLaw  == 4
                figs(2) = figure
                l = tiledlayout('flow');
                nexttile
                hold on
                title(l,'Kx')
                plot(t,x(:,25:72))
                ylabel('Magnitude');
                xlabel('Time (s)');
                legend('$K_x$','Interpreter','latex');
                hold off
                grid on
                l = nexttile;
                hold on
                title(l,'Kr')
                plot(t,x(:,73:88))
                ylabel('Magnitude');
                xlabel('Time (s)');
                legend('$K_r$','Interpreter','latex');
                hold off
                grid on
                end
        



        end
    %catch ERR
    %    display(ERR)
    %end  
end 