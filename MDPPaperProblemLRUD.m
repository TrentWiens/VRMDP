%VRMDP Paper Problem - Low Resolution

%changing to a 4x4 landing grid (16 landing spots)
%3 charge states
%2 day states

%[1 ] [2 ] [3 ] [4 ]
%[5 ] [6 ] [7 ] [8 ]
%[9 ] [10] [11] [12]
%[13] [14] [15] [16] 

%% Set up conditions 

tic %start timer

landingStates = 16; %number of landing spots
l1 = 4; %lengths of the sides of the grid
l2 = 4;
landingSpots = zeros(landingStates,2);
chargeStates = 3; %number of charge states
dayStates = 2; %numbers of day states
actions = 4 + 1; %can go up, down, left, right or stay
numOfStates = landingStates * chargeStates * dayStates; %total number of states

goal = 16; %goal location state

allGoalState1 = goal * chargeStates * dayStates - (chargeStates * dayStates - 1);
allGoalState2 = goal * chargeStates * dayStates;

allGoalStates = allGoalState1:allGoalState2; %get all the states that are at the location, it does not matter the ending charge or day state

stateSpace = zeros(dayStates,chargeStates,landingStates);
StateSpace = 1:numOfStates;

maxFlightDistance = 1; %max locations that the drone can fly over with full charge

%build the map for the drone - 4x4 grid
[X, Y] = meshgrid(1:l1, 1:l2);
landingSpots = [X(:), Y(:)];

%find distance between the landing spots
dist = abs(landingSpots(:,1) - landingSpots(:,1)').' + abs(landingSpots(:,2) - landingSpots(:,2)');

%% Build Probability Matrix
%this is a matrix that tells you the probability of getting to anther state
%based on the charge in the battery

Prob = zeros(landingStates, landingStates, chargeStates);

for fLand = 1:landingStates
    for gLand = 1:landingStates
        distribution = prob_Charge(maxFlightDistance, dist(fLand, gLand), 1,chargeStates);
        Prob(fLand, gLand, :) = distribution(3:-1:1);
        if fLand == gLand
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
                            if fDay == gDay && action ~= actions
                                if fLand == gLand
                                    if fCharge == gCharge - 1
                                        T(b,c,a) = 1;
                                    elseif gCharge == chargeStates && fCharge == chargeStates
                                        T(b,c,a) = 1; 
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

actionNum = inputdlg('What Action?');
num = str2num(actionNum{1});
if num ~= 0
    filename = 'ProbMat.xlsx';
    writematrix(T(:,:,num),filename,'Sheet', 1);
end

toc %stop timer