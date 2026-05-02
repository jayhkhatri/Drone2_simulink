function S = smoothness_second_diff(x)
    x = x(:);
    d2x = diff(x,2); % second difference
    S = sqrt(mean(d2x.^2));
end