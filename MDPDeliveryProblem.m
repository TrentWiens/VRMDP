%% MDP Delivery Problem

%Drone is solar powered, and wants to deliver packages to multiple places.
%There are 16 locations that are available for the drone to land.
%The drone can stay in the same spot and charge or go to a different place.
%The drone must manage its battery and deliver the packages. The 
%packages are delviered to specific goal locations that are defined before
%the MDP is made. The battery is initially split into 3 states - 0%, 50%,
%100% 


%% initialize the MDP parameters

locationStates = 16; %number of locations the drone can land
chargeStates = 3; %states of charge the battery can be in
dayStates = 2; %states of the day, day or night 
dims = sqrt(locationStates); %dimensions of the grid

goal1 = [2,4]; %goal locations (row, column)
goal2 = [3,1];

goal1Num = 8;
goal2Num = 9;

%create state space 
a = 1;
for i = 1:locationStates
    for j = 1:chargeStates
        stateSpace(a) = a;
        a = a + 1;
    end
end

%map for the drone - combos of [row,column]
a = 1;
for i = 1:dims
    for j = 1:dims
        landingSpots(a,:) = [i,j]; 
        a = a + 1;
    end
end




%find the distace between the landing spots
%number locations starting at the top left and going right then next
%row on the left to the end of the grid
%(1,1) is the distance between location 1 and location 1
%(1,2) is the distance between location 1 and location 2
%continue for all 16 locations
for i=1:1:locationStates
    for j=1:1:locationStates
        dist(i,j) = abs(landingSpots(i,1) - landingSpots(j,1)) + abs(landingSpots(i,2) - landingSpots(j,2));
    end
end

a = 1; %state you are coming from count
for g1c = 1:2 %goal 1 coming from 
    for g2c = 1:2 %goal 2 coming from 
        for lc = 1:locationStates %state you are coming from 
            charge = 0; %charge set to zero initially
            for cc = 1:chargeStates %state you are coming from 
                for dc = 1:dayStates %state you are coming from
                    b = 1; %state you are going to count
                    for g1g = 1:2 %goal 1 going to 
                        for g2g = 1:2 %goal 2 going to
                            for lg = 1:locationStates %state you are going to
                                final_Charge = 0; %final charge set to zero
                                distribution = prob_Charge(5, dist(lc, lg), 1);
                                for cg = 1:chargeStates %state you are going to
                                    for dg = 1:dayStates %state you are going to

                                        if a == b %if the state you are in is the state you are going to
                                            Prob(a,b) = 1; %you have 100% chance of getting there
                                        else
                                            if g1c == g1g && g2c == g2g 
                                                %Probability of going from
                                                %one landing spot to
                                                %another. Day state does
                                                %not change, flight time <<
                                                %day state time
                                                 if dc == dg %if you stay in the same day state
                                                     if charge < final_Charge %if you start with less charge than you end with
                                                         Prob(a,b) = 0; %cannot get to that state
                                                     else
                                                         chargeDiff = charge - final_Charge; %difference in charge
                                                         chargeIndex = round(chargeDiff/50,0) + 1;
                                                         if chargeIndex == 1
                                                             Prob(a,b) = distribution(chargeIndex);
                                                         else
                                                             Prob(a,b) = distribution(chargeIndex) - distribution(chargeIndex - 1);
                                                         end
                                                     end
                                                 else
                                                     Prob(a,b) = 0;
                                                 end

                                                 %Probability of charging.
                                                 %Stay in the same laning
                                                 %spot but increase charge.
                                                 %Uses 1 day state. 
                                                 if lc == lg %if you stay in the same landing state
                                                     if dc == 1
                                                         chargeTOD = 100; %charge you can get based on the time of day
                                                         chSTD = 0;
                                                     else 
                                                         chargeTOD = 0;
                                                         chSTD = 0;
                                                     end

                                                     probOfCharge = ChargingProbability(chargeTOD, chSTD); %probability of getting charge from a solar cell
                                                     chargeDiff = charge - final_Charge;
                                                     chargeIndex = -round(chargeDiff / 50, 0) + 1;

                                                     % nd = next day state
                                                     % if it is night, nd =
                                                     % 1
                                                     
                                                     if dc == dayStates %if the day state we are coming from is night
                                                         nd = 1; %set the next day to 1
                                                     else
                                                         nd = dc + 1; %otherwise just increase it by 1
                                                     end

                                                     if dg == nd %if the next day is the state we are going to
                                                         if chargeIndex == 1
                                                             Prob(a,b) = probOfCharge(chargeIndex);
                                                         elseif chargeIndex > 0 
                                                             Prob(a,b) = probOfCharge(chargeIndex) - probOfCharge(chargeIndex - 1);
                                                         else
                                                             Prob(a,b) = 0;
                                                         end
                                                     end 
                                                 end
                                            elseif g2c == 1 && g2g == 2 && lc == lg && lc == goal1Num
                                                Prob(a,b) = .8;
                                            elseif g1c == 1 && g1g == 2 && lc == lg && lc == goal2Num
                                                Prob(a,b) = .8;
                                            else 
                                                Prob(a,b) = 0;
                                            end
                                        end
                                        b = b + 1;

                                    end
                                    final_Charge = final_Charge + 50;
                                end
                            end
                        end
                    end
                    Q(a) = a;
                    stateSpace(g1c, g2c, lc, cc, dc) = a;
                    a = a + 1;
                end
                charge = charge + 50;
            end
        end
    end
end

%see if a given location state is on the way to every other location state.
%this assumes the drone does the longer direction first 
OTW(:,:,:) = zeros(locationStates, locationStates, locationStates);
for lg = 1:locationStates %location going to 
    for lg = 1:locationStates %location 
        a = abs(landingSpots(lg,1) - landingSpots(lc,1)); %distance between states, x
        b = abs(landingSpots(lg,2) - landingSpots(lc,2)); %distance between states, y
        for la = 1:locationStates %location that is being analyzed 
            if a > b
                if ((landingSpots(la,1) >= landingSpots(lg,1) && landingSpots(la,1) <= landingSpots(lc,1))...
                   ||(landingSpots(k,1) <= landingSpots()))
                end
            end
        end
    end
end



