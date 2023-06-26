%This itteration of the Flying fish MDP will consider the landing spots,
%charge and time of day in states. We do not consider the time used flying
%as it is small compared to the time to charge. This allows us to break the
%day into fewer states. 
clear all;
number_of_Landing_Spots = 3; %number of states to build the matrix. NOTE: THIS MUST NOT INCLUDE FAIL STATE WHICH WILL BE THE LAST STATE
charge_Intervals = 20; %must divide 100 without a remainder
charge_States = 100/charge_Intervals + 1; %Charge states, includes 0 and 100
time_Intervals = 4;
hours_per_Interval = 24/time_Intervals;
number_of_States = number_of_Landing_Spots * charge_States; %This includes the possible charge state. Assumes will always charge to atleast 50% and rounds to the nearest incrememnt of 5%
km = 120; %Set 1km as a specific number of pixels
avg_Flight_Distance_Full = 5; %mean flight distance and standard deviation in km
STD = 1;
OTW_distance = 0.3;
Rs = -0.04;
charge_Per_Day = 50; 
chSTD = 30;
speed = 0.5;
%When creating the state matrix the goal state much be the second last
%state and the fail state must be the last state
state_Pixel(:,:) = ones(number_of_Landing_Spots, 2); %build matrix of pixel locations. column 1 is x and 2 is y        
state_Pixel(1,:) = [817, 292];
state_Pixel(2,:) = [590, 114];
state_Pixel(3,:) = [417, 30];

%create a matrix (V) of vectors conncecting each state and a matrix of
%vector distances. 
V(:,:) = zeros(number_of_Landing_Spots, number_of_Landing_Spots);
dist(:,:) = zeros(number_of_Landing_Spots, number_of_Landing_Spots);
for i=1:1:number_of_Landing_Spots
    for j=1:1:number_of_Landing_Spots
        V(i,j) = sqrt((state_Pixel(i,1) - state_Pixel(j,1))^2 + (state_Pixel(i,2) - state_Pixel(j,2))^2);
        dist(i,j) = V(i,j)/km;
        Prob_Make_it(i,j) = prob_Drone_Distance(dist(i,j), avg_Flight_Distance_Full, STD);
     end
end
V(number_of_Landing_Spots, :) = 0;
dist(number_of_Landing_Spots, :) = 0;
%The following loop will create a matrix of probabilities. This considers
%the probability based on distance and current charge. It also considers
%what charge state you start in and will finish in at each landing spot.
%This therefore will create a different state for each pair of landing
%spots and charge. Notice we differentiate position (physical placement)
%and state (position and charge state)


a = 1;
for i=1:1:number_of_Landing_Spots
    charge = 0;
    for j=1:1:charge_States
        for k=1:1:time_Intervals
            b = 1;
            for m=1:1:number_of_Landing_Spots
                distribution(:) = prob_Charge(avg_Flight_Distance_Full, dist(i,m), STD);
                final_Charge = 0;
                for n=1:1:charge_States
                    for o=1:1:time_Intervals
                        charge_Dif = charge - final_Charge;
                        if final_Charge > charge
                            Prob(a,b) = 0;
                        else
                            if k == o
                                charge_Index = round(charge_Dif / charge_Intervals, 0) + 1; %what index a charge value will be found at in the distribution
                                if charge_Index == 1
                                    Prob(a,b) = distribution(charge_Index);
                                else
                                    Prob(a,b) = distribution(charge_Index) - distribution(charge_Index - 1);
                                end
                            else 
                                Prob(a,b) = 0;
                            end
                        end
                        if i == m
                            if k == 1
                                charging = charge_Per_Day * 0.5;
                            elseif k == 2 
                                charging = charge_Per_Day;
                            elseif k == 3
                                charging = charge_Per_Day * 0.5;
                            elseif k == 4
                                charging = charge_Per_Day * 0.0;
                            end
                            charge_dist = ChargingProbability(charging, chSTD);
                            charge_Index = -round(charge_Dif / charge_Intervals, 0) + 1;
                            if k == time_Intervals
                                c = 1;
                            else
                                c = k + 1;
                            end
                            if o == c
                                if charge_Index == 1
                                    Prob(a,b) = charge_dist(charge_Index);
                                elseif charge_Index > 0
                                    Prob(a,b) = charge_dist(charge_Index) - charge_dist(charge_Index-1);
                                else 
                                    Prob(a,b) = 0;
                                end
                            else
                                Prob(a,b) = 0;
                            end
                            
                        end
                        b = b + 1;
                    end
                    final_Charge = final_Charge + charge_Intervals;
                end
            end
            a = a + 1;
        end
        charge = charge + charge_Intervals;
    end
end

%Make the chance of leaving the goal state 0
Sz = size(Prob,1);
states_per_Location = charge_States*time_Intervals;
for i=1:1:states_per_Location
    a = Sz-i;
    Prob(:,Sz+1) = 1;
    Prob(Sz+1,:) = 0;
    Prob(Sz+1,Sz+1) = 1;
    Prob(a,:) = 0;
    Prob(a,Sz) = 1;
end

on_The_Way(:,:,:) = zeros(number_of_Landing_Spots, number_of_Landing_Spots, number_of_Landing_Spots); 
for i = 1: 1: number_of_Landing_Spots
    for j = 1: 1: number_of_Landing_Spots
       for k = 1: 1: number_of_Landing_Spots
           v1 = [state_Pixel(j,:),0];
           v2 = [state_Pixel(i,:),0];
           midpoint = round((v1 + v2)/2);
           pt = [state_Pixel(k,:),0];
           d = point_to_line(pt, midpoint, v2);
           D = d/km;
           if D < OTW_distance
                on_The_Way(j,k,i) = 1;
                prob_OTW(j,k,i) = Prob_Make_it(j,k); %Make a matrix of probabilities for the spots on the way
           else
                prob_OTW(j,k,i) = 0;
           end
           if k == i 
               on_The_Way(j,k,i) = 1;
               prob_OTW(j,k,i) = Prob_Make_it(j,k);
           end
       end    
    end
end

for i = 1: 1: number_of_Landing_Spots
    for j = 1: 1: number_of_Landing_Spots
       max_Prob = 0;
       summation_Prob = 0;
       destination_Prob = 0;
        for k = 1: 1: number_of_Landing_Spots
            summation_Prob = summation_Prob + prob_OTW(j,k,i);
            if prob_OTW(j,k,i) > max_Prob && i ~= k
                max_Prob = prob_OTW(j,k,i);
            end
            if i == k
                destination_Prob = prob_OTW(j,k,i);
            end 
        end
            fail_Prob(j,i) = 1 - max_Prob;
            land_OTW = max_Prob - destination_Prob;
            %Create a coefficient that will normalize all the remaining
            %probabilities. Since the probabilities of getting to each
            %individual landing spot are independent of each other we use
            %this to more accurately represent each probability and make
            %sure everything adds to 1.
            coefficient(j,i) = land_OTW/summation_Prob; 
    end
end

a = 1;
Sz = size(Prob,1);
P(:,:,:) = zeros(Sz,Sz,Sz);
for i=1:1:number_of_Landing_Spots
    for j=1:1:charge_States
        for k=1:1:time_Intervals
            b = 1;
            for m=1:1:number_of_Landing_Spots
                for n=1:1:charge_States
                    for o=1:1:time_Intervals   
                        c = 1;
                        for p=1:1:number_of_Landing_Spots
                            for q=1:1:charge_States
                                for r=1:1:time_Intervals
                                    if i == p
                                        P(b,c,a) = Prob(b,c);
                                    else
                                        if on_The_Way(m,p,i) == 1
                                            P(b,c,a) = Prob(b,c)*coefficient(m,i); %multiply coefficienct to normalize.
                                        else
                                            P(b,c,a) = 0; %if not on the way probability = 0
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
    end
end

P(:,:,Sz) = 0;
P(:,Sz,Sz) = 1;
for i=1:1:Sz
    for j=1:1:Sz
        A=0;
        for k=1:1:Sz
            A = A +P(j,k,i); 
        end
        %Dummy(i,j) = A;
        Fail = 1-A;
        P(j,Sz,i) = Fail;
        %FAIL(i,j) = Fail; 
    end
end
