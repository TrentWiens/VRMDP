%Simplest receding horizon problem 
clear all 

%Problem is a 1x5 grid of locations for a drone to move in - drone can move
%to the right or the left with a 80% chance to do the inputted action, 10%
%to stay in the same place, 10% to go the opposite direction

%takes outputs from MDP solver

Value = [2,5,3,2,1]; %Value output from MDP solver
Policy = [1,1,1,1,1]; %Policy output from MDP solver
StateSpace = [1,2,3,4,5]; %state space of the problem

%receding horizon decider - 2 spaces away from the current position

pos = 1;

horizonStateSpace = [1,2,3]; %only the states that are two spaces away from the current pos
horizonValue = [2,5,3]; %values from the states chosen

diffThreshold = -1; %the threshold where the difference is big enough to expand the state space

a = 1;
for i = 1:length(horizonValue)-1 %loop through each newValue

    val1 = horizonValue(i); %first value
    val2 = horizonValue(i+1); %next value
    valDiff = abs(val2-val1); %difference in value
    if valDiff > diffThreshold %if the difference is larger than the threshold
        horizonIndex(a) = 1;
    else
        horizonIndex(a) = 0;
    end
    a = a + 1;

end

splitStateSpace = [1,1.5,2,3]; %state space that is split with the horizon values
splitValue = [2,0,5,3]; %values with zero as a placeholder

%need to recalculate the value for the split state - shouldn't need to
%recalculate the whole problem
%could just assume the value is the average of the surrounding values

