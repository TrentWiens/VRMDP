%finds the states within the horizon that is desired

function [horizonValue, horizonStateSpace, horizonIndex] = recedingHorizonValue(Value, stateSpace, maxFlightDistance, landingSpots, pos)
    
    horizonValue = zeros(1,1,1);

    locStates = size(stateSpace,3); %find the number of states
    dayStates = size(stateSpace,2);
    chargeStates = size(stateSpace,1);

    horizonIndex = zeros(1,1,1);
    horizon = zeros(1,1,1);

    a = 1;
    for i = 1:locStates
        if landingSpots(i,1) <= landingSpots(pos,1) + maxFlightDistance && landingSpots(i,2) <= landingSpots(pos,2) + maxFlightDistance...
           && landingSpots(i,1) >= landingSpots(pos,1) - maxFlightDistance && landingSpots(i,2) >= landingSpots(pos,2) - maxFlightDistance
            horizon(a) = 1; %set the horizon matrix to one for the states that satisy the condition
            horizonIndex(a) = i; %get the index of that number for the next for loop
            a = a + 1;
        end
    end

    b = 1;
    for i = 1:length(horizon)
        if horizon(i) == 1 %if the horizon condition was met
            index = horizonIndex(i); %get index
            mappedIndex = (index-1)*dayStates*chargeStates + 1; %map the index to a the whole space
            for j = 1:size(stateSpace,2)*size(stateSpace,1) %go through all the charge and day states
                horizonStateSpace(j,1,i) = b; %make state space
                horizonValue(j,1,i) = Value(mappedIndex +(j - 1)); %get the values for the state space
                b = b + 1;
            end
        end
    end

    horizonValue = reshape(horizonValue,3,2,[]); %reshape to more better matrix
    horizonStateSpace = reshape(horizonStateSpace,3,2,[]);

end