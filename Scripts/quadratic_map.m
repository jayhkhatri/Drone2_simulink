function y = quadratic_map(x, a, b, y_a, y_b, reverse)
    % Quadratic mapping from [a, b] to [y_a, y_b]
    % reverse = 0 -> Normal mapping (a -> y_a, b -> y_b)
    % reverse = 1 -> Reverse mapping (a -> y_b, b -> y_a)
    
    c = (y_b - y_a) / (b - a)^2;
    
    if reverse
        y = y_b - c * (x - a).^2;
    else
        y = c * (x - a).^2 + y_a;
    end
end
