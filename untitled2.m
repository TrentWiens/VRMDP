% Example matrix
matrix = [1 2 3; 4 5 6; 7 8 9];

% Row indices to insert zeros
rowIndices = [1 2];

% Initialize the new matrix
newMatrix = [];

% Iterate over each row
for row = 1:size(matrix, 1)-1
    % Check if the current row needs to be replaced with zeros
        % Insert zeros in between the numbers
    % if
        newRow = [matrix(row, :); zeros(1, size(matrix, 2))];
    % else
    %     % Keep the original row
    %     newRow = matrix(row, :);
    % end
    
    % Concatenate the new row to the new matrix
    newMatrix = [newMatrix; newRow];
end

% Display the resulting matrix
disp(newMatrix);

% Example matrix
matrix = newMatrix

% Column indices to insert zeros
columnIndices = [1 2];

% Initialize the new matrix
newMatrix = [];

% Iterate over each column
for col = 1:size(matrix, 2)
    % Check if the current column needs to be replaced with zeros
    if any(columnIndices == col)
        % Insert zeros in between the numbers
        newColumn = [matrix(:, col), zeros(size(matrix, 1), 1)];
    else
        % Keep the original column
        newColumn = matrix(:, col);
    end
    
    % Concatenate the new column to the new matrix
    newMatrix = [newMatrix, newColumn];
end

% Display the resulting matrix
disp(newMatrix);

