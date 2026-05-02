function S = smoothness_diff(x)
    x = x(:); % ensure column
    dx = diff(x); % first difference
    S = sqrt(mean(dx.^2)); % RMS of differences
end