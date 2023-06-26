%VRMDP Paper Problem - Low Resolution

%changing to a 4x4 landing grid (16 landing spots)
%3 charge states
%2 day states

%[1 ] [2 ] [3 ] [4 ]
%[5 ] [6 ] [7 ] [8 ]
%[9 ] [10] [11] [12]
%[13] [14] [15] [16] 

%% variable 
goal = 12; %goal location state
pos = 1; %current position of the drone
maxFlightDistance = 1; %max locations that the drone can fly

%% Set up conditions 

tic %start timer

landingStates = 16; %number of landing spots
l1 = 4; %lengths of the sides of the grid
l2 = 4;
landingSpots = zeros(landingStates,2);
chargeStates = 3; %number of charge states
dayStates = 2; %numbers of day states
actions = landingStates + 1; %can go to each landing state, or charge
numOfStates = landingStates * chargeStates * dayStates; %total number of states

allGoalState1 = goal * chargeStates * dayStates - (chargeStates * dayStates - 1);
allGoalState2 = goal * chargeStates * dayStates;

allGoalStates = allGoalState1:allGoalState2; %get all the states that are at the location;

stateSpace = zeros(chargeStates,dayStates,landingStates);
StateSpace = 1:numOfStates;

%build the map for the drone - 4x4 grid
[X, Y] = meshgrid(1:l1, 1:l2);
landingSpots = [X(:), Y(:)];

%find distance between the landing spots
dist = abs(landingSpots(:,1) - landingSpots(:,1)').' + abs(landingSpots(:,2) - landingSpots(:,2)');


%% Build Probability Matrix
%this is a matrix that tells you the probability of getting to anther state
%based on the charge in the battery

Prob = zeros(landingStates, landingStates, chargeStates);

for fLand = 1:landingStates %location coming from 
    for gLand = 1:landingStates %location going to
        distribution = prob_Charge(maxFlightDistance, dist(fLand, gLand), 1,chargeStates); %use prob_Charge to get a normal distribution
        Prob(fLand, gLand, :) = distribution(3:-1:1); %get the probability at each coming from, going to combo
        if fLand == gLand %set to 1 if you are going to the location you are coming from
            Prob(fLand, gLand, 1:3) = 1;
        end
        
    end
end

%% Build Transition Matrix

T = zeros(numOfStates, numOfStates, actions);

%'f' stands for 'from', i.e. 'fLand' means landing state you are coming from 
%'g' stands for 'going', i.e. 'gLand' means landing state you are going to

a = 1; %action counter
for action = 1:actions %actions
    b = 1; %from state counter
    for fLand = 1:landingStates %landing state you are coming from 
        for fDay = 1:dayStates %charge state you are coming from
            for fCharge = 1:chargeStates %day state you are coming from
                c = 1; %going state counter
                for gLand = 1:landingStates %landing state you are going to
                    for gDay = 1:dayStates %charge state you are going to
                        for gCharge = 1:chargeStates %day state you are going to
                            if fDay == gDay && action ~= actions %&& dist(fLand, gLand) <= 1 %day state does not change within an action, and you do not move in action 17
                                if fLand == gLand && gLand == action%% if you are staying in the same place
                                    if fCharge == 1 && gCharge == 2 %full charge, must decrease charge
                                        T(b,c,a) = 1;
                                    elseif fCharge > 1 && gCharge == 3 %not full charge, must decrease charge to lowest
                                        T(b,c,a) = 1;
                                    end
                                elseif gLand == action && fLand ~= gLand %if you are not staying in the same place
                                    if fCharge == 1 && gCharge == 2 %full charge, must decrease charge
                                        T(b,c,a) = Prob(fLand, gLand, fCharge); %Prob matrix from earlier
                                    elseif fCharge > 1 && gCharge == 3 %not full charge, must decrease charge to lowest possible
                                        T(b,c,a) = Prob(fLand, gLand, fCharge); %Prob matrix from earlier
                                    end
                                elseif fLand == gLand && gLand ~= action %if you are staying in the same place, but not at the goal
                                    if fCharge == 1 && gCharge == 2 %full charge, must decrease charge
                                        T(b,c,a) = 1 - Prob(fLand, action, fCharge); %prob matrix from earlier
                                    elseif fCharge > 1 && gCharge == 3 %not full charge, must decrease to lowest possible
                                        T(b,c,a) = 1 - Prob(fLand, action, fCharge);
                                    end
                                end
                            elseif fDay == gDay && action == actions && fLand == gLand %stay and charge action
                                if fDay == 1 %early in the day
                                    if fCharge < 3 && gCharge == 1 %charge is highest or second from highest, go to one level up
                                        T(b,c,a) = 1;
                                    elseif fCharge == 3 && gCharge == 1 %charge is lowest, go to one level up
                                        T(b,c,a) = 1;
                                    end
                                else %late in the day
                                    if fCharge == 1 && gCharge == 1 %full charge, does not change
                                        T(b,c,a) = 1;
                                    elseif fCharge > 1 && gCharge == 1 %chance to increase charge later in the day
                                        T(b,c,a) = .8;
                                    elseif fCharge == gCharge && fCharge ~= 1
                                        T(b,c,a) = .2;
                                    end
                                end
                            end
                            c = c + 1;
                        end
                    end
                end
                b = b + 1;
            end
        end
    end
    a = a + 1;
end

%% Build reward matrix

R = -.1*ones(numOfStates, numOfStates, actions); %negative reward for taking an action

R(allGoalStates,:,:) = 0; %create ending state
R(:,allGoalStates,:) = 1; %create reward states


%% solve flat MDP

discount = 0.95;
epsilon = 0.01;

[Value, Policy, iter] = MDPvalue(T,R,discount,epsilon); %solve MDP

Policy2 = reshape(Policy, [6,16]); %reshape policy and value for readability
Value2 = reshape(Value, [6,16]);

%print policy and value
fprintf('Goal is %i\nStay and charge action is %i \n \n',goal, actions)
fprintf('Original Policy: \n')
disp(1:16)
disp(Policy2)

fprintf('Original Value: \n')
disp(Value2)

%% receding horizon 

[horizonValue, horizonStateSpace, savedLocs] = recedingHorizonValue(Value, stateSpace, maxFlightDistance, landingSpots, pos);

%% find spots where the difference is large engough to split

savedLocsCoords = landingSpots(savedLocs,:);

[splitSpots] = splitValueGradient(horizonValue, savedLocs, savedLocsCoords);

%% split the state space according to the split spots decided

[splitValue, splitStateSpace, locSplitStates] = splitter(horizonValue, splitSpots);

%% get new conditions

newLandingStates = size(splitValue,3); %new number of states
newChargeStates = size(splitValue,1); 
newDayStates = size(splitValue,2); 
newNumOfStates = newLandingStates * newChargeStates * newDayStates;
newActions = newLandingStates + 1;

%% create new reward matrix
R = -.1*ones(newNumOfStates, newNumOfStates, newActions);

initSplitStates = zeros(1,1); %get the initial splitStates with the origional location numbers
a = 1;
for i = 1:length(locSplitStates)
    if locSplitStates(i) ~= 0
        initSplitStates(i) = savedLocs(a);
        a = a + 1;
    else
        initSplitStates(i) = 0;
    end
end

if any(ismember(initSplitStates,goal)) %if the goal is included in the receding horizon and make the ending reward state
    splitGoal = find(initSplitStates == goal);
    allGoalState1 = splitGoal * newChargeStates * newDayStates - (newChargeStates * newDayStates - 1);
    allGoalState2 = splitGoal * newChargeStates * newDayStates;
    allGoalStates = allGoalState1:allGoalState2;
    R(allGoalStates,:,:) = 0;
    R(:,allGoalStates,:) = 1;
    fprintf("Initial goal is included in horizon\nSplit goal is %i\nUnsplit goal is %i \n",splitGoal,goal)
else %otherwise, find the state that is closest to the goal and make the ending reward state
    splitDiff = 100;
    for i = 1:length(initSplitStates) %look at all the split states
        if initSplitStates(i) ~= 0 %check to make sure it is not a split state
            newSplitDiff = dist(initSplitStates(i),goal); %find the distance between the state and the goal
            if newSplitDiff < splitDiff %if the difference is smaller
                splitDiff = newSplitDiff; %set splitDiff to newSplitDiff
                splitGoal = i; %find the splitGoal
                origGoal = initSplitStates(i); %find the original equivalent for the splitGoal
            end
        end
    end
    allGoalState1 = splitGoal * newChargeStates * newDayStates - (newChargeStates * newDayStates - 1); %make the reward state
    allGoalState2 = splitGoal * newChargeStates * newDayStates;
    allGoalStates = allGoalState1:allGoalState2;
    R(allGoalStates,:,:) = 0;
    R(:,allGoalStates,:) = 1;
    fprintf("Initial goal is NOT included in horizon\nSplit goal is %i\nUnsplit goal is %i\n",splitGoal,origGoal) %publish info about the goal
end

%% new dist matrix 

newDist = zeros(newLandingStates, newLandingStates);

a = 1; %savedLocs counter 1
for i = 1:newLandingStates %for each landing state
    if locSplitStates(i) ~= 0 %if the state is not a split state
        b = 1; %savedLocs counter 2
        for j = 1:newLandingStates %for each landing state 
            if locSplitStates(j) ~= 0 %if the state is not a split state
                newDist(i,j) = dist(savedLocs(a),savedLocs(b)); %add the distance to the new matrix
                b = b + 1;
            end
        end
        a = a + 1;
    end
end

%get the averages for the split states
for i = 1:newLandingStates %states you are coming from 
    for j = 1:newLandingStates %states you are going to
        if i ~= j %the same location has zero distance
            if locSplitStates(i) == 0 %if the state youre coming from is a split state
                newDist(i,j) = (newDist(i+1,j) + newDist(i-1,j))/2; %get the average of the distance between two horizontal states
            elseif locSplitStates(j) == 0 %if the state youre going to is a split state 
                newDist(i,j) = (newDist(i,j+1) + newDist(i,j-1))/2; %get the average of the distance between two vertical states
            end
        end
    end
end

%% create new prob matrix
%same as 'Prob' matrix, just includes the new split states from the newDist
%matrix

newProb = zeros(newLandingStates, newLandingStates, newChargeStates);

for fLand = 1:newLandingStates 
    for gLand = 1:newLandingStates
        distribution = prob_Charge(maxFlightDistance, newDist(fLand, gLand), 1,newChargeStates);
        newProb(fLand, gLand, :) = distribution(newChargeStates:-1:1);
        if fLand == gLand
            newProb(fLand, gLand, 1:newChargeStates) = 1;
        end
    end
end

%% create new transition probability matrix

T = zeros(newNumOfStates, newNumOfStates, newActions);

a = 1; %action counter
for action = 1:newActions %actions
    b = 1; %from state counter
    for fLand = 1:newLandingStates %landing state you are coming from 
        for fDay = 1:newDayStates %charge state you are coming from
            for fCharge = 1:newChargeStates %day state you are coming from
                c = 1; %going state counter
                for gLand = 1:newLandingStates %landing state you are going to
                    for gDay = 1:newDayStates %charge state you are going to
                        for gCharge = 1:newChargeStates%day state you are going to
                            if fDay == gDay && action ~= newActions
                                if fLand == gLand && gLand == action %if you are staying in the same place
                                    if gCharge == fCharge + 1 %need to decrease charge
                                        T(b,c,a) = 1;
                                    elseif gCharge == newChargeStates && fCharge == newChargeStates %already at lowest charge
                                        T(b,c,a) = 1;
                                    end
                                elseif gLand == action && fLand ~= gLand
                                    if gCharge == fCharge + 1 %need to decrease charge
                                        T(b,c,a) = newProb(fLand, gLand, fCharge);
                                    elseif gCharge == newChargeStates && fCharge == newChargeStates %already at lowest charge
                                        T(b,c,a) = newProb(fLand, gLand,fCharge);
                                    end    
                                elseif fLand == gLand && gLand ~= newActions
                                    if gCharge == fCharge + 1 %need to decrease charge
                                        T(b,c,a) = 1 - newProb(fLand, action, fCharge);
                                    elseif gCharge == newChargeStates && fCharge == newChargeStates %already at lowest charge
                                        T(b,c,a) = 1 - newProb(fLand, action, fCharge);
                                    end
                                end

                            elseif fDay == gDay && action == newActions && fLand == gLand %charging action - stay in the same spot and increase charge
                                chanceOfCharge = linspace(1/newChargeStates,1,newDayStates); %chance of charging across the diferent day states
                                if fDay == 1 %earliest in the day, best time for charging
                                    if gCharge == 1 && fCharge < newChargeStates %not at max charge
                                        T(b,c,a) = chanceOfCharge(newDayStates + 1 - fDay);
                                    elseif gCharge == 1 && fCharge == newChargeStates %already at lowest charge
                                        T(b,c,a) = chanceOfCharge(newDayStates);
                                    end
                                else %some time other than earliest in the day
                                    if fCharge == gCharge && fCharge == 1
                                        T(b,c,a) = 1;
                                    elseif gCharge == fCharge
                                        T(b,c,a) = 1 - chanceOfCharge(newDayStates + 1 - fDay);
                                    elseif gCharge == 1 && fCharge ~= 1
                                        T(b,c,a) = chanceOfCharge(newDayStates + 1  - fDay);
                                    end
                                end
                            end
                            c = c + 1;
                        end
                    end
                end
                b = b + 1;
            end
        end
    end
    a = a + 1;
end

%% re-solve the MDP

discount = 0.95;
epsilon = 0.01;
splitValue = reshape(splitValue,1,[]);

[Value, Policy, iter] = MDPvalueWARM(T,R,discount,epsilon,splitValue); %solve MDP

Policy2 = reshape(Policy, [newChargeStates*newDayStates,newLandingStates]); %reshape policy and value for readability
Value2 = reshape(Value, [newChargeStates*newDayStates,newLandingStates]);

%print policy and value
fprintf('Stay and charge action is %i\n',newActions)
fprintf("\n")
fprintf('Split Policy: \n')
disp(initSplitStates)
disp(1:newLandingStates)
disp(Policy2)

fprintf('Split Value: \n')
disp(Value2)

toc %stop timer

%% export matrix to Excel (not required) - enter zero to not export - can help to visualize large matrices
%takes 15-30 seconds to run
%excel sheet must be closed

% actionNum = inputdlg('What Action?');
% num = str2num(actionNum{1});
% if num ~= 0
%     filename = 'ProbMat.xlsx';
%     writematrix(T(:,:,num),filename,'Sheet', 1);
%     fprintf('Done\n')
% end

