%build the state pixel matrix for the flying fish case
function [number_of_Landing_Spots, state_Pixel, Image] = FlyingFishMatrix()
    number_of_Landing_Spots = 23;
    Image = imread('FlyingFish2Map2.bmp');
    state_Pixel(:,:) = ones(number_of_Landing_Spots, 2); %build matrix of pixel locations. column 1 is x and 2 is y        
    state_Pixel(1,:) = [817, 292];
    state_Pixel(2,:) = [741, 354];
    state_Pixel(3,:) = [647, 445];
    state_Pixel(4,:) = [379, 436];
    state_Pixel(5,:) = [172, 529];
    state_Pixel(6,:) = [202, 488];
    state_Pixel(7,:) = [205, 452];
    state_Pixel(8,:) = [136, 313];
    state_Pixel(9,:) = [173, 323];
    state_Pixel(10,:) = [316, 300];
    state_Pixel(11,:) = [207, 223];
    state_Pixel(12,:) = [122, 259];
    state_Pixel(13,:) = [130, 227];
    state_Pixel(14,:) = [59, 218];
    state_Pixel(15,:) = [642, 86];
    state_Pixel(16,:) = [590, 114];
    state_Pixel(17,:) = [575, 69];
    state_Pixel(18,:) = [529, 87];
    state_Pixel(19,:) = [451, 43];
    state_Pixel(20,:) = [417, 30];
    state_Pixel(21,:) = [55, 81];
    state_Pixel(22,:) = [107, 15];
end