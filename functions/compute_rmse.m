function rmse = compute_rmse(actual, predicted)
    % Ensure inputs are column vectors
    actual = actual(:);
    predicted = predicted(:);
    
    % Check same length
    if length(actual) ~= length(predicted)
        error('Vectors must be of same length');
    end
    
    % RMSE calculation
    rmse = sqrt(mean((actual - predicted).^2));
end