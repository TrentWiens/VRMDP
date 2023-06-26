%Create a function that accepts the average distance/charge, the distance,
%the charge,and the std. This function will return the probability you get 
%there with a specific charge. 
function [norm] = prob_Charge(AvgDpC, dist, STD,num)
    %charge_Dif = charge - final_Charge;
    x = linspace(0,100,num);
    avg_Charge = (dist/AvgDpC)*100;
    charge_STD = (STD/AvgDpC)*100; 
    %norm2 = normpdf(x,avg_Charge, charge_STD);
    norm = normcdf(x,avg_Charge, charge_STD);
    %charge_Dif_index = round((charge_Dif/5)+1,0);
    %max = 100/x(2) + 1;
    %plot(x,norm)
    %final_Charge_index = round((charge_Dif/5)+1,0);
    %Prob2 = norm2(max) - norm2(1);
    %Prob2
end