%This itteration of the flying fish problem will consider the charge, the
%time of day and the distance. Time of day will be split into hours, day
%hours will have full charge potential, dusk and dawn hours will have
%partial and night hours will have none.
clear all;
number_of_Landing_Spots = 3; %number of states to build the matrix. NOTE: THIS MUST INCLUDE FAIL STATE WHICH WILL BE THE LAST STATE
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
b = 1; %counter for take off state
for i=1:1:number_of_Landing_Spots %position we take off from
   charge = 0; %start with charge = 0 and increment by 5
    for l=1:1:charge_States %charge state per position
        a = 1; %counter for landing state.
        for j=1:1:number_of_Landing_Spots %position we land at
            final_Charge = 0; %start with final charge = 0 and increment by 5
            
            %this function will accept the average flight distance, the
            %distance we wish to travel, and the standard deviation of the 
            %average flight distance. It returns a distribution of the of
            %probability we make the flight using a specific amount of
            %charge
            distribution(:) = prob_Charge(avg_Flight_Distance_Full, dist(i,j), STD);
            for k=1:1:charge_States %charge state we land at
                charge_Dif = charge - final_Charge;
                if charge_Dif < 0 %Can't end a flight with more charge than you started
                    Prob(b,a) = 0;
                else
                    charge_Index = round(charge_Dif / 5, 0) + 1; %what index a charge value will be found at in the distribution
                    if charge_Index == 1
                        Prob(b,a) = distribution(charge_Index);
                    else
                        Prob(b,a) = distribution(charge_Index) - distribution(charge_Index - 1);
                    end
                end
                %If you are staying at a position then you will end up at 
                %the same position but a charge state higher by the amount 
                %you can charge in a day
                if i == j    
                   charge_dist = ChargingProbability(charge_Per_Day, chSTD);
                   charge_Index = -round(charge_Dif / 5, 0) + 1;
                   if charge_Index == 1
                       Prob(b,a) = charge_dist(charge_Index);
                   elseif charge_Index > 0
                       Prob(b,a) = charge_dist(charge_Index) - charge_dist(charge_Index-1);
                   else 
                       Prob(b,a) = 0;
                   end
                end
                
                a = a + 1;
                final_Charge = final_Charge + 5;
            end
        end
        charge = charge + 5;
        b = b +1;
    end
end

%Probability of leaving the goal state is 0
for i=1:1:charge_States
    a = 441 + i;
    Prob(a,:) = 0;
    Prob(a,462) = 1;
end

%Prob(number_of_Landing_Spots, :, :) = 0;
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

%Figure out the fail probability of going between every posible state and
%the probabilities of ending up at another state. 
%This first for loop group will create a matrix of probabilities on the way
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
x = (number_of_Landing_Spots-1)*charge_States +1;
P(:,:,:) = zeros(x,x,x);
a = 1;
%create the probability matrix to be used in the MDP. This matrix considers
%the probability of getting from one state to another and then calculates
%the probabilities that you end up in a different state then the one
%intended using the on the way calculations earlier to decide other viable
%landing spots. 
for i=1:1:number_of_Landing_Spots-1
    for l=1:1:charge_States
        b = 1;
        for j=1:1:number_of_Landing_Spots-1
            for m=1:1:charge_States
              c=1;
                for k=1:1:number_of_Landing_Spots-1
                    succeed = 0;
                    for n=1:1:charge_States
                        if i == k
                            P(b,c,a) = Prob(b,c);
                        else
                            if on_The_Way(j,k,i) == 1
                                P(b,c,a) = Prob(b,c)*coefficient(j,i); %multiply coefficienct to normalize.
                            else
                                P(b,c,a) = 0; %if not on the way probability = 0
                            end
                        end
                        c = c+1;
                    end     
                end
                b = b+1;
            end
        end
        a = a+1;
    end
end
%Create the fail probability. Fail probability is the 1 minus the sum of
%all of the possible land probabilities along a route. 
Sz = size(P,1); 
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

y = (number_of_Landing_Spots - 1) * charge_States * 4 + 1;
P2 = zeros(y,y,y);
a = 1;
aa = 1;
TOD = 0;
zz = 7;
for i=1:1:number_of_Landing_Spots-1
    for j=1:1:charge_States
        for k=1:1:time_Intervals
            b = 1;
            bb = 1;
            for m=1:1:number_of_Landing_Spots-1
                for n=1:1:charge_States
                    for o=1:1:time_Intervals
                        c = 1;
                        cc = 1;
                        for q=1:1:number_of_Landing_Spots-1
                            if m == q
                                time = 6;
                            else
                                time = dist(m,q)/speed;
                            end
                            for r=1:1:charge_States 
                                for t=0:1:time_Intervals
                                    if time>(t*hours_per_Interval-hours_per_Interval/2) && time<(t*hours_per_Interval+hours_per_Interval/2)
                                        tdif = t; 
                                    end
                                end
                                for s=1:1:time_Intervals
                                    if o + tdif < time_Intervals
                                        if s == o + tdif
                                            P2(b,c,a) = P(bb,cc,aa);
                                        end
                                    else
                                        zz = o + tdif - time_Intervals;
                                        if s == zz
                                            P2(b,c,a) = P(bb,cc,aa);
                                        end
                                    end
                                    c = c + 1; 
                                end
                                cc = cc +1;
                            end
                        end
                        b = b + 1;
                    end
                    bb = bb + 1;
                end
            end
            a = a + 1;
        end
        aa = aa + 1;
    end
end

%Assign rewards, Rs for every action, assign a large negative for the fail
%state and then assign a high positive to success states. 
Reward(:,:,:) = Rs*ones(Sz,Sz,Sz);
Reward(:,Sz,:) = -5;

for i=0:1:charge_States-1;
    g = (number_of_Landing_Spots - 1) * charge_States - i;
    Reward(:,g,:) = 5;
end
discount = .95;
[Value,policy, iter, cpu_time] = mdp_policy_iteration(P, Reward, discount);
%[V,policy,iter] = MDPpolicy(P, Reward, discount);
l = 0;
%add a second column to the policy matrix that shows the landing spot and
%charge of each state
for i=1:1:number_of_Landing_Spots
    k = 0; 
    if i ~= number_of_Landing_Spots
        for j=1:1:charge_States
         l = l+1;
         policy1(l,2) = i + k;
         k = k + 0.005;
        end
    else
        l = l+1;
        policy1(l,2) = i;
    end
end
policy1(:,1) = ceil(policy(:)/charge_States); %make more readable, instead of showing charge and position state just show position state

%Divide the policy matrix. columns are charge states and rows are position
%states
for i=1:1:charge_States
    for j=1:1:number_of_Landing_Spots-1
        m = (j-1)*charge_States + i;
        split_Policy(j,i) = policy(m);
    end
end
split_Policy(:,:) = ceil(split_Policy(:,:)/charge_States);
PP(:,:) = P(:,:,4);
P2P(:,:) = P2(:,:,4);
