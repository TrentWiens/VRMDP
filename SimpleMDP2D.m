%Another Simple MDP, 2D
clear all 
clc

%MDP is a 5x5 area of locations, where the drone is trying to go from the
%upper left location to the lower left location. There is a cost of going to
%each location (power/wear). The drone has an 80% chance of going the
%direction perscribed, a 10% chance of staying in place, and a 10% chance 
%of going the wrong direction. The drone can go in 4 directions, up, down, left, right

%% describe MDP to be solved

numOfStates = 25;

for i = 1:25

    StateSpace(i) = i;

end

states = [1  2  3  4  5 ;
          6  7  8  9  10;
          11 12 13 14 15;
          16 17 18 19 20;
          21 22 23 24 25];

actions = 4; %1 = up, 2 = down, 3 = right, 4 = left

%% create reward matrix

minusReward = -.4;
R = minusReward * ones(numOfStates,numOfStates,actions);
R(:,numOfStates,:) = 1;
R(numOfStates,:,:) = 0;

%% create transition probability matrix

f = .8; %chance the drone goes in the perscribed direction
o = .1; %chance the drone goes in the opposite direction
s = .1; %chance the drone does not move

T = zeros(numOfStates,numOfStates,actions);

for i = 1:numOfStates %state you are coming from 
    for j = 1:numOfStates %state you are going to 
        for k = 1:actions %actions
            if k == 1 %up action
                if i == j && i < 21 && i > 5 %prob you stay in the same spot, except first and last row
                    T(i,j,k) = s;
                elseif i == j && i <= 5 %prob you stay in the same spot, first row
                    T(i,j,k) = f + s;
                elseif i == j && i >= 21 %prob you stay in the same spot, last row 
                    T(i,j,k) = o + s;
                elseif i + 5 == j  %prob you go to the state below you 
                    T(i,j,k) = o;
                elseif j + 5 == i %prob you go to the state above you
                    T(i,j,k) = f;
                end
            elseif k == 2
                if i == j && i < 21 && i > 5 %prob you stay in the same spot, except first and last row
                    T(i,j,k) = s;
                elseif i == j && i <= 5 %prob you stay in the same spot, first row
                    T(i,j,k) = o + s;
                elseif i == j && i >= 21 %prob you stay in the same spot, last row 
                    T(i,j,k) = f + s;
                elseif i + 5 == j  %prob you go to the state below you 
                    T(i,j,k) = f;
                elseif j + 5 == i %prob you go to the state above you
                    T(i,j,k) = o;
                end
            elseif k == 3
                rEdge = [5,10,15,20,25]; %all states on the right edge
                lEdge = [1,6,11,16,21]; %all states on the left edge
                isrEdge = false;
                islEdge = false;
                if ismember(i,rEdge) %check if the state is on the right or left edge
                    isrEdge = true;
                elseif ismember(i,lEdge)
                    islEdge = true;
                end
                if i == j && ~isrEdge && ~islEdge %prob you stay in the same spot, except the left and right edge
                    T(i,j,k) = s;
                elseif i == j && isrEdge %prob you stay in the same spot, right edge
                    T(i,j,k) = o + s;
                elseif i == j && islEdge %prob you stay in the same spot, left edge 
                    T(i,j,k) = f + s;
                elseif i + 1 == j && ~isrEdge %prob you go to the state to the right
                    T(i,j,k) = f;
                elseif j + 1 == i && ~islEdge %prob you go to the state to the left
                    T(i,j,k) = o;
                end
            elseif k == 4
                rEdge = [5,10,15,20,25]; %all states on the right edge
                lEdge = [1,6,11,16,21]; %all states on the left edge
                isrEdge = false;
                islEdge = false;
                if ismember(i,rEdge) %check if the state is on the right or left edge
                    isrEdge = true;
                elseif ismember(i,lEdge)
                    islEdge = true;
                end
                if i == j && ~isrEdge && ~islEdge %prob you stay in the same spot, except the left and right edge
                    T(i,j,k) = s;
                elseif i == j && isrEdge %prob you stay in the same spot, right edge
                    T(i,j,k) = f + s;
                elseif i == j && islEdge %prob you stay in the same spot, left edge 
                    T(i,j,k) = o + s;
                elseif i + 1 == j && ~isrEdge %prob you go to the state to the right
                    T(i,j,k) = o;
                elseif j + 1 == i && ~islEdge %prob you go to the state to the left
                    T(i,j,k) = f;
                end
            end

            T(:,numOfStates,:) = 0; %make the goal state an absorbing state
            T(numOfStates,numOfStates,:) = 1;
        end
    end
end

%% solve the MDP 

discount = 0.95;
epsilon = 0.01;

[Value, Policy, iter] = MDPvalue(T,R,discount,epsilon);

fprintf('Original Policy: ')
disp(Policy)
fprintf('Original Value: ')
disp(Value)

%% get the values for the receding horizon

pos = 5; %position that the drone is in currently - could put this whole process in a loop to go through all starting positions - probably would want to make its own function then 

[horizonValue, horizonStateSpace] = simpleHorizonValue(Value, StateSpace, pos);

%% decide where the state space should be split from the horizon values using gradient

[splitSpots] = simpleValueGradient(horizonValue, horizonStateSpace);

%% split the state space according to the split spots decided

[splitValue, splitStateSpace] = simpleSplitter(horizonValue, horizonStateSpace, splitSpots);

%% keep splitting the state space until it does not want to split anymore
%this could split too much in a larger MDP, might need to  find a better
%criteria to stop the splitting

%this will take an increasing long time to run, the more values there are
%the longer it will take to check all the values and this will keep
%building upon itself

count = 0;
j = 1;
while j == 1 && count < 100 

    %should not run simpleHorizonValue again because we are already within
    %the horizon

    [splitSpots] = simpleValueGradient(splitValue, splitStateSpace); 
    [splitValueCurrent, splitStateSpaceCurrent] = simpleSplitter(splitValue, splitStateSpace, splitSpots);

    if isequal(splitValueCurrent,splitValue) %check if the split state from the current loop is the same as the split state from the previous state
        j = 0; %set j to zero, stopping the while loop 
    end

    splitValue = splitValueCurrent; %set the current loop's state space and value matrices to the "previous" loop's state and value matrices
    splitStateSpace = splitStateSpaceCurrent;

    count = count + 1;
end

fprintf('Splitting Iterations: %d \n',count+1) %print the amount of times that the states were split - could have been split multiple times in one iteration

numOfSplits = length(splitStateSpace) - length(horizonStateSpace); %the difference in the legnth of the splitStateSpace and the original horizonStateSpace is the amount of new states that were added 

fprintf('The state space was split %d times \n', numOfSplits) %print the amount of splits
fprintf('\n')
fprintf('Split State Space: ') %print the new splitStateSpace
disp(splitStateSpace)
fprintf('Split Value: ')
disp(splitValue)
