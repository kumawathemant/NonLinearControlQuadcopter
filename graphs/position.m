function [figs] = position(t,x,args)
    figs = [];
    %try
        if args.AdaptiveLaw ==5
            figure;
            hold on;
            plot3(x(:,83),x(:,84),x(:,85),'r')
            plot3(x(:,13),x(:,14),x(:,15),'b--')
            plot3(x(end,83),x(end,84),x(end,85),'r*')
            plot3(x(end,13),x(end,14),x(end,15),'b*')
            
            title(['Quadcopter Trajectory for non linear system (',r,s,')'])
            legend('Mismatched','Model','Mismatched Final Position','Model Final Position')
            view(3)
            zlabel('y (m)','Interpreter','latex');
            ylabel('z (m)','Interpreter','latex');
            xlabel('x (m)','Interpreter','latex');
            hold off;


        elseif args.AdaptiveLaw == 1 || args.AdaptiveLaw == 2 || args.AdaptiveLaw ==3 || args.AdaptiveLaw ==4
                figs(1) = figure 
                hold on
                plot3(x(:,1),x(:,2),x(:,3),'r')
                plot3(x(:,13),x(:,14),x(:,15),'b--')
                plot3(x(end,1),x(end,2),x(end,3),'r*')
                plot3(x(end,13),x(end,14),x(end,15),'b*')
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
                elseif args.AdaptiveLaw  == 3
                    a = ' Non Linear system and';
                elseif args.AdaptiveLaw  == 4
                    a = ' Non Linear system and Adaptive';
                end 
                title(['Quadcopter Trajectory with',a,' Linear Controller (',r,s,')'])
                legend('Mismatched','Model','Mismatched Final Position','Model Final Position')
                view(3)
                zlabel('y (m)','Interpreter','latex');
                ylabel('z (m)','Interpreter','latex');
                xlabel('x (m)','Interpreter','latex');
                hold off
                grid on

                saveas(figs(1),['Position_',r,'_law',num2str(args.AdaptiveLaw),'.png']);
        end
    %catch ERR
    %    display(ERR)
    %end 
end 