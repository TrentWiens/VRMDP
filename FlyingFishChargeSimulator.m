clear all;

km = 120; %Set 1km as a specific number of pixels
avg_Flight_Distance_Full = 5; %mean flight distance and standard deviation in km
STD = 1;
OTW_distance = 0.3;
Rs = -0.04;
charge_Per_Hour = 10; 
chSTD = 6;
%When creating the state matrix the goal state much be the second last
%state and the fail state must be the last state
[number_of_Landing_Spots, state_Pixel, Image] = FlyingFishMatrix();
number_of_States = number_of_Landing_Spots *21; %This includes the possible charge state. Assumes will always charge to atleast 50% and rounds to the nearest incrememnt of 5%

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
    for l=1:1:21 %charge state per position
        a = 1; %counter for landing state.
        for j=1:1:number_of_Landing_Spots %position we land at
            final_Charge = 0; %start with final charge = 0 and increment by 5
            
            %this function will accept the average flight distance, the
            %distance we wish to travel, and the standard deviation of the 
            %average flight distance. It returns a distribution of the of
            %probability we make the flight using a specific amount of
            %charge
            distribution(:) = prob_Charge(avg_Flight_Distance_Full, dist(i,j), STD);
           
            for k=1:1:21 %charge state we land at
                 
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
                   charge_dist = ChargingProbability(charge_Per_Hour, chSTD);
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
for i=1:1:21
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
x = (number_of_Landing_Spots-1)*21 +1;
P(:,:,:) = zeros(x,x,x);
a = 1;
%create the probability matrix to be used in the MDP. This matrix considers
%the probability of getting from one state to another and then calculates
%the probabilities that you end up in a different state then the one
%intended using the on the way calculations earlier to decide other viable
%landing spots. 
for i=1:1:number_of_Landing_Spots-1
    for l=1:1:21
        b = 1;
        for j=1:1:number_of_Landing_Spots-1
            for m=1:1:21
              c=1;
                for k=1:1:number_of_Landing_Spots-1
                    succeed = 0;
                    for n=1:1:21
                        if i == k
                            P(b,c,a) = Prob(b,c);
                        else
                            if on_The_Way(j,k,i) == 1
                                P(b,c,a) = Prob(b,c)*coefficient(j,i); %multiply coefficienct to normalize.
                            else
                                P(b,c,a) = 0; %if not on the way probability = 0
                            end
                            %succeed = succeed + P(b,c,a); 
                        end
                       
                        c = c+1;
                    end     
                end
                %fail = 1 - succeed;
                %P(b,c,a) = fail;
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
%failP(:,:,:) = P(:,Sz,:);
%Assign rewards, Rs for every action, assign a large negative for the fail
%state and then assign a high positive to success states. 
Reward(:,:,:) = Rs*ones(Sz,Sz,Sz);
Reward(:,Sz,:) = -5;

for i=0:1:20
    g = (number_of_Landing_Spots - 1) * 21 - i;
    Reward(:,g,:) = 5;
end

discount = .95;
[Value,policy, iter, cpu_time] = mdp_policy_iteration(P, Reward, discount);
l = 0;
%add a second column to the policy matrix that shows the landing spot and
%charge of each state
for i=1:1:number_of_Landing_Spots
    k = 0; 
    if i ~= number_of_Landing_Spots
        for j=1:1:21
         l = l+1;
         policy1(l,2) = i + k;
         k = k + 0.005;
        end
    else
        l = l+1;
        policy1(l,2) = i;
    end
end
policy1(:,1) = ceil(policy(:)/21); %make more readable, instead of showing charge and position state just show position state


%Divide the policy matrix. columns are charge states and rows are position
%states
for i=1:1:21
    for j=1:1:number_of_Landing_Spots-1
        m = (j-1)*21 + i;
        split_Policy(j,i) = policy(m);
    end
end
split_Policy(:,:) = ceil(split_Policy(:,:)/21);

%Draw on sepperate maps, one for each charge state

for j=1:1:21
    
    figure,imshow(Image)
    for i=1:1:number_of_Landing_Spots-2
        a = split_Policy(i,j); %assign a to be the state we should go to according to the policy from state i
        x1 = state_Pixel(i,1); 
        y1 = state_Pixel(i,2); %assign x1 and y1 to be the x and y co-ordinates of the initial state i 
        x2 = state_Pixel(a,1);
        y2 = state_Pixel(a,2); %assign x2 and y2 to be the x and y co-ordinates of the final state determined by the policy
        hold on;
        
        if x1 == x2 && y1 == y2
            x =[x1, x1+5];
            y =[y1, y1+5];
            line(x,y,'Color','red','LineStyle','-')
            x =[x1, x1-5];
            y =[y1, y1-5];
            line(x,y,'Color','red','LineStyle','-')
        else
            x =[x1, x2]; %x is the x co-ordinates
            y =[y1, y2]; %y is the y co-ordinates
            line(x,y,'Color','blue','LineStyle','-') %draw line from the initial state to the state determined by the policy
        end
   end
end

%Start Simulator
%set the speed of the drone and number of trials
speed = 15; %km/hr
trials = 100;
%Initialize the average time and distance and the number of times the 
%simulation fails to count during simulation
%Count number of states and initialize the starting state as position 1
%with a full charge
avg_Time = 0;
avg_Dist = 0;
number_Of_States = Sz;
starting_State = 21;
current_State = 21;
failed = 0;
%final represents where the goal and fail states start
final = (number_of_Landing_Spots-2)*21+1;
Fail_State = number_of_Landing_Spots*21 + 1;
for j=1:1:trials
    clear route; %clear so we can reuse every loop
    clear route1;
    %initialize the current state as the starting state and that we are
    %starting new trip
    current_State = starting_State; 
    End = false; 
    %initialize the matrixes for time and distance. Initialize counter n
    %for while loop
    n = 1; 
    time(j)= 0;
    dist_Traveled(j) = 0;
    
    %Run while you have not reached the end. This loop will look at the
    %policy to decide where to go. It will then simulate an attmept to go
    %there using the probabilities of making it to that state. It will
    %decide where we end up using a random sample with our probabilities.
    %It will then record the data and make the new location current
    %location and run again.
    while End == false
        goTo = policy(current_State);
        Probabilities(:) = P(current_State, :, goTo);
        Size1 = size(Probabilities(:));
        for i=1:1:Size1
            Pop(i) = i;
        end
        current_State = randsample(Pop,1,true,Probabilities);
        if current_State >= final
            End = true;
        end
        route(n)= current_State;
        n = n+1;
    end
    %Make route1 display position not state.
    route1(:) = ceil(route(:)/21);
    x = size(route1);
    routeSz = x(1,2);
    %This loop will figure out the time and distance per loop. If we stay
    %to charge it will add 1 hour, if we travel it will divide the distance
    %travled by the speed of the drone to decide time. Distance between
    %landing spots will be added up.
    for i=2:1:routeSz
        if route1(i) == route1(i-1)
            time(j) = time(j)+1;
        else
            t = dist(route1(i-1), route1(i))/speed;
            dist_Traveled(j) = dist_Traveled(j) + dist(route1(i-1), route1(i));
            time(j) = time(j) + t;
        end
    end
    %Add all of the times and distances up to average later
    avg_Time = avg_Time + time(j);
    avg_Dist = avg_Dist + dist_Traveled(j);
    %if the current state is the fail state then the drone failed. This
    %will count the number of failures. 
    if current_State >= Fail_State
        failed = failed + 1;
    end
end
%compute averages, minimums and maximums
avg_Time = avg_Time/trials;
avg_Dist = avg_Dist/trials;
min_Time = time(1);
max_Time = time(1);
max_Dist = dist_Traveled(1);
min_Dist = dist_Traveled(1);
for j=1:1:trials
    if dist_Traveled(j) <= min_Dist
        min_Dist = dist_Traveled(j);
    end
    if dist_Traveled(j) >= max_Dist
        max_Dist = dist_Traveled(j);
    end
    if time(j) <= min_Time
        min_Time = time(j);
    end
    if time(j) >= max_Time
        max_Time = time(j);
    end
end