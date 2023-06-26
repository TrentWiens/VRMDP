inputMatrix = [1 8 3 4; 7 8 10 15; 3 2 60 7; 12 15 7 8];
% Step 1: Define the subset of locations
locations = [1 1; 2 1; 3 1; 2 2; 2 3; 3 1];

% Step 2: Create a new matrix with zeros
outputMatrix = zeros(max(locations(:, 1)), max(locations(:, 2)));

% Step 3 and 4: Find large differences and reconstruct the matrix
threshold = 3; % Set your desired threshold here

% Keep track of the locations already updated
updatedLocations = zeros(size(outputMatrix));

% Perform the updates
for k = 1:size(locations, 1)
    i = locations(k, 1);
    j = locations(k, 2);
    
    % Check difference with left adjacent cell
    if j > 1 && ismember([i, j-1], locations, 'rows') && abs(inputMatrix(i, j) - inputMatrix(i, j-1)) > threshold
        % Insert new location
        newLocation = [i, j-0.5];
        locations = [locations; newLocation];
        
        % Update the location being checked
        locations(k, :) = newLocation;
        
        % Mark the updated locations
        updatedLocations(i, j) = 1;
        updatedLocations(i, j-1) = 1;
    end
    
    % Check difference with above adjacent cell
    if i > 1 && ismember([i-1, j], locations, 'rows') && abs(inputMatrix(i, j) - inputMatrix(i-1, j)) > threshold
        % Insert new location
        newLocation = [i-0.5, j];
        locations = [locations; newLocation];
        
        % Update the location being checked
        locations(k, :) = newLocation;
        
        % Mark the updated locations
        updatedLocations(i, j) = 1;
        updatedLocations(i-1, j) = 1;
    end
end

% Apply the updates to the output matrix
outputMatrix(updatedLocations == 1) = inputMatrix(updatedLocations == 1);

% Display the output matrix
disp('Output Matrix:');
disp(outputMatrix);

