function vis(lm, wp, xTrue, data, step, openVis)
    if openVis == 0
        return;
    end
    %%
    rob=[0 -4 -4; 0 2 -2];
    %% Initialize the figure for animation.
    figure('name','SLAM Demo','color','w','units','normalized',...
         'outerposition',[0 0 1 1]);
    hold on;axis equal;box on;
    axis([-20 250 -90 70]);
    xlabel('x(m)');
    ylabel('y(m)');
    %% Plot landmarks and waypoints   
    plot(wp(1, :), wp(2, :), 'g.', 'markersize', 10, 'linewidth',3);
    %%
    TrueLandmark= plot(lm(1,:),lm(2,:),'b*','linewidth', 1);
    trueRob     = patch(0,0,'b.','linewidth',2,'erasemode','normal');
    filterRob   = patch(0,0,'r.','markersize',3,'erasemode','normal'); 
    truePath    = plot(0,0,'g','linewidth',1,'erasemode','normal');
    filterPath  = plot(0,0,'m','linewidth',1,'erasemode','normal');
    rcovEllipse = plot(0,0,'r','linewidth',1,'erasemode','normal');
    fcovEllipse = plot(0,0,'r','linewidth',1,'erasemode','normal');
    obsFeature  = plot(0,0,'r+','linewidth',1,'erasemode','normal');
    obsLine     = plot(0,0,'y','linewidth',1,'erasemode','normal');
    label       = [truePath,filterPath,TrueLandmark,obsFeature];
    hc          =legend(label,'True Path','IEKF Location','True Landmarks','Observations');
    set(hc,'box','off','location', 'NorthOutside','orientation', 'horizontal');
    %% Animation
    path    = data.path; % Just for convenience.
    pos     = data.pos;
    cov     = data.cov;
    pcount  = 0; 
    %%
    for k = 1:step
        pcount = pcount+1;
        if pcount == 4      
            pcount      = 0;
            trueRobBody = compound(xTrue(:,k), rob);
            postRobBody = compound(pos(k).x(1:3), rob);
            %% 
            set(trueRob,  'xdata', trueRobBody(1,:),...
                'ydata', trueRobBody(2, :)); 
            set(filterRob,  'xdata', postRobBody(1,:), ...
                'ydata', postRobBody(2, :));
            set(truePath, 'xdata', xTrue(1,1:k),...
                'ydata', xTrue(2,1:k)); 
            set(filterPath,  'xdata', path(1,1:k), ...
                'ydata', path(2,1:k));
            %% Robot sigma ellipse
            xcov = getSigmaEllipse(pos(k).x,cov(k).Pr(1:2,1:2),3,10); 
            set(rcovEllipse,'xdata',xcov(1,:),'ydata',xcov(2,:));
            %% Observed features            
            if  ~isempty(pos(k).z) 
                set(obsFeature, 'xdata', pos(k).x(4:2:end),...
                    'ydata', pos(k).x(5:2:end));
                % Make laser lines between robot and features
                flines = makeLines(pos(k).z,path(:,k)); 
                set(obsLine, 'xdata', flines(1,:), 'ydata', flines(2,:));
                % Make the covariance between features
                fcov   = makeFcov(pos(k).x, cov(k).Pf, 3, 10);
                set(fcovEllipse,'xdata', fcov(1,:), 'ydata', fcov(2,:));
            end
        end
        drawnow;
    end
end