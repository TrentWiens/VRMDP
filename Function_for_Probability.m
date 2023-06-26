%Create a function that accepts average distance and standard deviation and
%a distance then outputs the probability the drone can make it that far. 
clear all;
x = [0:.5:10];
distance = 0.7002;
avg = 5;
STD = 1;
dist_Position = round(distance/x(2)+1);
norm = normcdf(x,avg,STD);
max = 10/x(2) + 1;
prob_Function = norm(max) - norm(dist_Position);
