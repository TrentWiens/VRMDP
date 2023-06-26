%splits the states and outputs the new sets of values and the new state
%space

function [splitValues, splitStateSpace] = simpleSplitter(horizonValue, horizonStateSpace, splitSpots)

    NumOfStates = length(horizonStateSpace); %initial number of states

    a = 1;
    b = 1;
    for i = 1:NumOfStates %loop through all states
        splitStateSpace(a) = a; %get the state 'index' for an already existing state
        splitValues(a) = horizonValue(i); %get the value of the already existing state
        a = a + 1;
        if b < NumOfStates %makes sure that the splitSpots matrix is indexed correctly
            if splitSpots(b) == 1 %if the value in the splitSpots matrix is one, this means that the value difference between the two states is large enough to split
                splitStateSpace(a) = a; %get state index for the new split state 
                splitValues(a) = (horizonValue(i) + horizonValue(i+1))/2; %get the average value between the two neighboring values tht have the large enough difference - this is just to estimate the value
                a = a + 1;
            end
        end
        b = b + 1;
    end
end