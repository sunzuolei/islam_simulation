function point = makeLines(z, x)
    % Use to compute sets of line segments for laser range-bearing measurements.
    % Inputs:
    %   z    - observations.
    %   x    - robot pose.
    % Outputs:
    %  point - points in line from robot to feature.
    %%
    if isempty(z), point = [];return,end
    %% Store range and bearing.
    % First, transform the polar coordinate to Cartesian coordinate.
    % Second, transform the position relative to robot  to global coordinate.
    lenf         = size(z,2);
    fline(1,:)   = zeros(1,lenf) + x(1); % Store robot position in x-axis
    fline(2,:)   = zeros(1,lenf) + x(2); % Store robot position in y-axis
    fline(3:4,:) = compound(x, [z(1,:).*cos(z(2,:)); z(1,:).*sin(z(2,:))]);

    %% Convert line to point
    % Convert a list of lines so that they will be plotted as a set of
    % unconnected lines but only require a single handle to do so. This
    % is performed by converting the lines to a set of points, where a
    % NaN point is inserted between every point-pair:
    %
    % l= [x1a x1b x1c;
    %     y1a y1b y1c;
    %     x2a x2b x2c;
    %     y2a y2b y2c];
    % becomes
    % p= [x1a x2a NaN x1b x2b NaN x1c x2c;
    %     y1a y2a NaN y1b y2b NaN y1c y2c];
    %%
    point = zeros(2,size(fline,2)*3 - 1);
    point(:,1:3:end) = fline(1:2,:);
    point(:,2:3:end) = fline(3:4,:);
    point(:,3:3:end) = NaN;
end