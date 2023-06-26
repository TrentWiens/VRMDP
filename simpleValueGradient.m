%finds the places where the value gradient is large enough for the states
%to be split

function [splitSpots] = simpleValueGradient(horizonValue, horizonStateSpace)

    horizonStates = length(horizonStateSpace); %new number of states
   
    splitSpots = zeros(1,horizonStates-1); %matrix the is the size of the amount of pairs of neighboring states

    diffThreshhold = .3; %threshold for splittting the state space - can be changed to encourage or discourage splitting

    for i = 1:horizonStates-1 %loop through the new number of states minus one 
        valueDiff = abs(horizonValue(i) - horizonValue(i + 1)); %get the difference between the values for each pair of neighboring states
        if valueDiff > diffThreshhold 
            splitSpots(i) = 1; %change the zero to a one for each pair of neighboring states where the difference is larger than the threshold set
        end

    end

end