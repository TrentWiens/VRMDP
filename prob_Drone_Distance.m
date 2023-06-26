%Create a function that accepts average distance and standard deviation and
%a distance then outputs the probability the drone can make it that far. 
function Prob = prob_Drone_Distance(distance, avg, STD)
x = 0:.5:10;
dist_Position = round(distance/x(2)+1);
norm = normcdf(x,avg,STD);
max = 10/x(2) + 1;
Prob = norm(max) - norm(dist_Position);
end