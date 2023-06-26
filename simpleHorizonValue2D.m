%finds the states that are within the horizon that is desired

function [horizonValue, horizonStateSpace] = simpleHorizonValue2D(Value, StateSpace, pos)


    NumOfStates = length(StateSpace); %find the number of states

    horizon = zeros(1,NumOfStates); %create empty matrix for all states

    a = 1;
    for i = 1:NumOfStates %loop through all states
        if i >= pos - 2 && i <= pos + 2 %horizon condition - if the space is two or less spots away. Can be changed to any condition if desired
            horizon(a) = 1; %set the horizon matrix number to one for the states that the condition is met
            horizonIndex(a) = i; %get the index of that number for the next loop
            a = a + 1;
        end
    end

    for i = 1:length(horizon) %loop through all the states again
        if horizon(i) == 1 %if the condition was met
            index = horizonIndex(i); 
            horizonStateSpace(i) = i; %get the state space for all the states that met the condition 
            horizonValue(i) = Value(index); %get all the values for the states that met the condition
        end 
    end


end