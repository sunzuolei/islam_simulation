function angle = piTopi(angle)
    % Input: 
    %   angle - array of angles.
    % Output:
    %   angle - normalised angles[-pi, pi].
    %%
    twopi = 2*pi;
    angle = angle - twopi*fix(angle/twopi); % This is a stripped-down version of rem(angle, 2*pi)

    i = find(angle >= pi);
    angle(i) = angle(i) - twopi;

    i = find(angle < -pi);
    angle(i) = angle(i) + twopi;
end