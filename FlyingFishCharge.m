clear all;
number_of_Landing_Spots = 23; %number of states to build the matrix. NOTE: THIS MUST INCLUDE FAIL STATE WHICH WILL BE THE LAST STATE
charge_Intervals = 5; %must divide 100 without a remainder
charge_States = 100/charge_Intervals + 1; %Charge states, includes 0 and 100
number_of_States = number_of_Landing_Spots *21; %This includes the possible charge state. Assumes will always charge to atleast 50% and rounds to the nearest incrememnt of 5%
km = 120; %Set 1km as a specific number of pixels
avg_Flight_Distance_Full = 5; %mean flight distance and standard deviation in km
STD = 1;
OTW_distance = 0.3;
Rs = -0.04;
charge_Per_Day = 50; 
chSTD = 30;
%When creating the state matrix the goal state much be the second last
%state and the fail state must be the last state
state_Pixel(:,:) = ones(number_of_Landing_Spots, 2); %build matrix of pixel locations. column 1 is x and 2 is y        
state_Pixel(1,:) = [817, 292];
state_Pixel(2,:) = [741, 354];
state_Pixel(3,:) = [647, 445];
state_Pixel(4,:) = [379, 436];
state_Pixel(5,:) = [172, 529];
state_Pixel(6,:) = [202, 488];
state_Pixel(7,:) = [205, 452];
state_Pixel(8,:) = [136, 313];
state_Pixel(9,:) = [173, 323];
state_Pixel(10,:) = [316, 300];
state_Pixel(11,:) = [207, 223];
state_Pixel(12,:) = [122, 259];
state_Pixel(13,:) = [130, 227];
state_Pixel(14,:) = [59, 218];
state_Pixel(15,:) = [642, 86];
state_Pixel(16,:) = [590, 114];
state_Pixel(17,:) = [575, 69];
state_Pixel(18,:) = [529, 87];
state_Pixel(19,:) = [451, 43];
state_Pixel(20,:) = [417, 30];
state_Pixel(21,:) = [55, 81];
state_Pixel(22,:) = [107, 15];
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
            if b == 441 && j == 22
                Dummy(:) = distribution(:);
                dd = dist(i,j);
                de = charge;
            end
            for k=1:1:21 %charge state we land at
                 
                charge_Dif = charge - final_Charge;
                if charge_Dif < 0 %Can't end a flight with more charge than you started
                    Prob(b,a) = 0;
                    if b == 441 && j == 22
                        DDD(k) = 1;
                    end
                else
                    charge_Index = round(charge_Dif / 5, 0) + 1; %what index a charge value will be found at in the distribution
                    if b == 441 && j == 22
                        DDD(k) = charge_Index;
                    end
                    if charge_Index == 1
                        Prob(b,a) = distribution(charge_Index);
                    else
                        Prob(b,a) = distribution(charge_Index) - distribution(charge_Index - 1);
                    end
                    if b == 441 && j == 22
                        Dummy2(k) = Prob(b,a);
                    end
                end
                %If you are staying at a position then you will end up at 
                %the same position but a charge state higher by the amount 
                %you can charge in a day
                if i == j    
                   % if charge_Dif == -charge_Per_Day
                   %     Prob(b,a) = 1;
                   % else
                   %     Prob(b,a) = 0;
                   % end
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
DummyR(:,:) = Reward(:,:,22);
discount = .95;
[Value,policy, iter, cpu_time] = mdp_policy_iteration(P, Reward, discount);
%[policyV, iterV, cpu_timeV] = mdp_value_iteration_bound_iter(P, Reward, discount);
%[V,policy,iter] = MDPpolicy(P, Reward, discount);
%DD = FAIL(:,:) + Dummy(:,:);
l = 0;
%add a second column to the policy matrix that shows the landing spot and
%charge of each state
for i=1:1:number_of_Landing_Spots
    k = 0; 
    if i ~= number_of_Landing_Spots
        for j=1:1:21
         l = l+1;
         policy1(l,2) = i + k;
         policy2(l,2) = i + k;
         k = k + 0.005;
        end
    else
        l = l+1;
        policy1(l,2) = i;
        policy2(l,2) = i;
    end
end
policy1(:,1) = ceil(policy(:)/21); %make more readable, instead of showing charge and position state just show position state
%policy2(:,1) = ceil(policyV(:)/21);

%Divide the policy matrix. columns are charge states and rows are position
%states
for i=1:1:21
    for j=1:1:number_of_Landing_Spots-1
        m = (j-1)*21 + i;
        split_Policy(j,i) = policy(m);
 %       split_PolicyV(j,i) = policyV(m);
    end
end
split_Policy(:,:) = ceil(split_Policy(:,:)/21);
%split_PolicyV(:,:) = ceil(split_PolicyV(:,:)/21);
%Draw on sepperate maps, one for each charge state
%Image = imread('FlyingFish2Map2.bmp');
%for j=1:1:21
    
 %   figure,imshow(Image)
  %  for i=1:1:number_of_Landing_Spots-2
   %     a = split_Policy(i,j); %assign a to be the state we should go to according to the policy from state i
   %     x1 = state_Pixel(i,1); 
   %     y1 = state_Pixel(i,2); %assign x1 and y1 to be the x and y co-ordinates of the initial state i 
   %     x2 = state_Pixel(a,1);
   %     y2 = state_Pixel(a,2); %assign x2 and y2 to be the x and y co-ordinates of the final state determined by the policy
   %     hold on;
        
   %     if x1 == x2 && y1 == y2
   %         x =[x1, x1+5];
   %         y =[y1, y1+5];
   %         line(x,y,'Color','red','LineStyle','-')
   %         x =[x1, x1-5];
   %         y =[y1, y1-5];
   %         line(x,y,'Color','red','LineStyle','-')
   %     else
   %         x =[x1, x2]; %x is the x co-ordinates
   %         y =[y1, y2]; %y is the y co-ordinates
   %         line(x,y,'Color','blue','LineStyle','-') %draw line from the initial state to the state determined by the policy
   %     end
   %end
%end
%initialize drone speed, the number of trials we want the simulator to run
%and the starting state as position 1 with full battery. 
speed = 15;
trials = 100;
starting_State = 21;
[avg_Time, avg_Dist, min_Time, min_Dist, max_Time, max_Dist, failed] = MDP_Simulator(P, speed, trials, number_of_Landing_Spots, starting_State, dist, policy, Sz);
