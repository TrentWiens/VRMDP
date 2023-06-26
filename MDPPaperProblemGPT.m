% Example 3D matrix with zeros
matrix3D = zeros(3, 3, 5);
matrix3D(:,:,1) = [1 2 3; 4 5 6; 7 8 9];
matrix3D(:,:,3) = [2 3 4; 5 6 7; 8 9 10];
matrix3D(:,:,5) = [3 4 5; 4 5 6; 7 8 9];

% Example locations with zeros
zeroIndices = [2,4,5];

% Calculate distances between non-zero matrices and matrices with zeros
distances = zeros(size(matrix3D, 3), numel(zeroIndices));
for i = 1:size(matrix3D, 3)
    nonZeroMatrix = matrix3D(:,:,i);
    nonZeroIndices = nonZeroMatrix(:) ~= 0;
    zeroMatrix = nonZeroMatrix;
    zeroMatrix(nonZeroIndices) = 0;
    
    for j = 1:numel(zeroIndices)
        zeroIndex = zeroIndices(:, j);
        distances(i, j) = norm(zeroMatrix(zeroIndex) - nonZeroMatrix(zeroIndex));
    end
end

% Fill in the zeros with the average of the two closest matrices
for j = 1:numel(zeroIndices)
    [~, closestIndices] = sort(distances(:, j));
    closestIndices = closestIndices(1:2);
    
    averageMatrix = (matrix3D(:, :, closestIndices(1)) + matrix3D(:, :, closestIndices(2))) / 2;
    matrix3D(zeroIndices(:, j), j) = averageMatrix(zeroIndices(:, j));
end

disp(matrix3D)
