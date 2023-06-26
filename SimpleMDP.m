%% Simple MDP Problem
clear all 
clc

%MDP is a 1x5 area of locations where a drone is trying to go from the
%left-most location to the right-most location. There is a cost of going to
%each location (power/wear). The drone has an 80% chance of going the
%direction perscribed, 10% chance to go the opposite direction and 10%
%chance of not moving. 

%% describe the MDP to be solved

StateSpace = [1,2,3,4,5]; %each state
NumOfStates = length(StateSpace);
Actions = 2; %number of actions

%% create reward matrix

minusReward = -.4; %negative reward for moving
R = minusReward * ones(NumOfStates, NumOfStates, Actions);
R(:,5,:) = 1;
R(5,:,:) = 0;


%% create transition probability matrix

f = .8; %chance the drone goes in perscribed direction
o = .1; %chance the drone goes in the opposite direction
s = .1; %chance the drone does not move

T(:,:,1) = [o + s, f, 0, 0, 0;  %transition probability matrix taking action 1 (moving right) 
                o, s, f, 0, 0;
                0, o, s, f, 0;
                0, 0, o, s, 0;
                0, 0, 0, o, 1];

T(:,:,2) = [f + s, o, 0, 0, 0;  %transition probability matrix taking action 1 (moving left) 
                f, s, o, 0, 0;
                0, f, s, o, 0;
                0, 0, f, s, 0;
                0, 0, 0, f, 1];

%% solve the MDP 

discount = 0.95;
epsilon = 0.01;

[Value, Policy, iter] = MDPvalue(T,R,discount,epsilon);

fprintf('Original Policy: ')
disp(Policy)
fprintf('Original Value: ')
disp(Value)


%% get the values for the receding horizon

pos = 1; %position that the drone is in currently - could put this whole process in a loop to go through all starting positions - probably would want to make its own function then 

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


%% create a new transition matrix of split state space

newNumOfStates = length(splitStateSpace); %find how many new states there are

newT = zeros(newNumOfStates, newNumOfStates, Actions); %create the empty new transition matrix 

for i = 1:newNumOfStates %loop through all states 
    for j = 1:newNumOfStates %loop through all states
        for k = 1:Actions %loop through all actions
            if k == 1 %the first action (go right) 
                if i == 1 && j == 1 %if its the left most state, both the chance you hit the left wall and the chance that you stay in the same spot should be added together
                    newT(i,j,k) = o + s;
                elseif i == j && i ~= 1 && j ~= 1 %if the state you are going to and the state you are leaving are the same - set to the chace of staying in place (except the left-most state)  
                    newT(i,j,k) = s;
                elseif i + 1 == j %if the state you are going to is one larger than the state you are leaving - set to the chance of going the perscribed direction 
                    newT(i,j,k) = f;
                elseif j + 1 == i %if the state you are going to is one smaller than the state you are leaving - set to the chance of going the opposite direction
                    newT(i,j,k) = o;
                end
            elseif k == 2 %the second action (go left) 
                if i == 1 && j == 1 %same as the first action, but in this situation, the perdcribed direction will make you hit the wall so you stay in the same spot so, chance of perscribed direction is added to chance of staying in the same spot
                    newT(i,j,k) = f + s; 
                elseif i == j && i ~= 1 && j ~= 1 %if the state you are going to and the state you are leaving are the same - set to the chace of staying in place (except the left-most state) 
                    newT(i,j,k) = s;
                elseif i + 1 == j %if the state you are going to is one larger than the state you are leaving - set to the chance of going the opposite direction 
                    newT(i,j,k) = o;
                elseif j + 1 == i %if the sate you are going to is one smaller than the state you are leaving - set to the chance of going to perscribed direction
                    newT(i,j,k) = f;
                end
            end
        end
    end
end

newT(:,newNumOfStates,:) = 0; %assume the right-most state is the goal/end - the split could have not included the goal
newT(newNumOfStates,newNumOfStates,:) = 1; %make the goal an absorbing state 

%% create new reward matrix, assuming that the right most value is the goal/end

newR = ones(newNumOfStates, newNumOfStates, Actions); %create the new empty reward matrix 
newR = minusReward * newR; %set all the values to the negative reward for moving 

newR(:,newNumOfStates,:) = 1; %make the right-most state an absorbing state 
newR(newNumOfStates,:,:) = 0;

%% solve the MDP again, with newT and newR and 'warm start' it with the splitValue numbers

[newValue1, newPolicy1, iter1] = MDPvalueWARM(newT,newR,discount,.01,splitValue);

fprintf('New Value: ')
disp(newValue1) 
fprintf('Sub policy: ')
disp(newPolicy1) 

