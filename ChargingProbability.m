%create a function to figure out probabilities for the amount of charge you
%can get from a solar cell.
function norm = ChargingProbability(avg_Charge, STD)
    x = [0:50:100];
    %avg_Charge = 50;
    %STD = 10;
    norm = normcdf(x, avg_Charge, STD);
   % plot(x,norm)
end
