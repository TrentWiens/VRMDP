
clear all;
number_of_States = 23; %number of states to build the matrix. NOTE: THIS MUST INCLUDE FAIL STATE WHICH WILL BE THE LAST STATE
km = 120; %Set 1km as a specific number of pixels
avg_Flight_Distance = 4; %mean flight distance and standard deviation in km
STD = 1;
OTW_distance = 0.5;
Rs = -3;
%When creating the state matrix the goal state much be the second last
%state and the fail state must be the last state
state_Pixel(:,:) = ones(number_of_States, 2); %build matrix of pixel locations. column 1 is x and 2 is y        
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
%create a matrix (V) of vectors conncecting each state.

for i=1:1:number_of_States
    for j=1:1:number_of_States
        V(i,j) = sqrt((state_Pixel(i,1) - state_Pixel(j,1))^2 + (state_Pixel(i,2) - state_Pixel(j,2))^2);
        dist = V(i,j)/km;
        Prob(i,j) = prob_Drone_Distance(dist, avg_Flight_Distance, STD); %create a matrix of probabilities between 1 state and another. function accepts the distance flown, the avg distance the drone can go and the STD.  
    end
end
V(number_of_States, :) = 0;
Prob(number_of_States, :) = 0;
%Create a matrix to decide whether a state is a realistic landing spot
%between two states you intend on traveling between
on_The_Way(:,:,:) = zeros(number_of_States, number_of_States, number_of_States); 
for i = 1: 1: number_of_States
    for j = 1: 1: number_of_States
       for k = 1: 1: number_of_States
           v1 = [state_Pixel(j,:),0];
           v2 = [state_Pixel(i,:),0];
           pt = [state_Pixel(k,:),0];
           d = point_to_line(pt, v1, v2);
           D = d/km;
           if D < OTW_distance
                on_The_Way(j,k,i) = 1;
           end
           
       end    
    end
end
%Build the probability matrix. The fail probability is the probability it
%can't land any of the places on the way. This matrix will only consider
%the probability of getting to a state with a full charge. It will consider
%other possible landing spots on the way.

P(:,:,:) = zeros(number_of_States, number_of_States, number_of_States); 
for i = 1: 1: number_of_States
    for j = 1: 1: number_of_States
     total_Prob = 0;
     T = 0;
        for k = 1: 1: number_of_States
            if k == 1 %set the total probability as the probability of making it to the goal state. This will allow us to calculate fail probability.
                total_Prob = Prob(j,i);
            end
            if on_The_Way(j,k,i) == 1
                R(k) = Prob(j,k);
            else 
                R(k) = 0;
            end
            %Any probability that is higher than the goal state and is on
            %the way will be considered a possible landing spot and a
            %probability assigned.
            if R(k) > total_Prob && j ~= k  %Take the difference of the total probability and the probability of the on the way spots and assign that probability to the state. 
                P(j,k,i) = (R(k) - total_Prob)/2; %This is far from a perfect solution but it should work well enough
                total_Prob = total_Prob + P(j,k,i);
            elseif j == k && k ~= i
                    P(j,k,i) = 0.00;
                    total_Prob = total_Prob + P(j,k,i);
            elseif k == i
                P(j,k,i) = Prob(j,i);
            elseif R(k) <= P(j,k,i)
                P(j,k,i) = 0;
            end
            if P(j,k,i)< 0
                P(j,k,i) = 0;
            end
        end
       prob_Fail = 1 - total_Prob;
       P(j, number_of_States, i) = prob_Fail;
    end
end

%This creates a policy using policy iteration. This uses the probability of
%landing matrix created above

Reward(:,:,:) = Rs*ones(number_of_States, number_of_States, number_of_States);
Reward(:,22,:) = 5;
Reward(:,23,:) = -5;
discount = .95;
[V,policy, iter, cpu_time] = mdp_policy_iteration(P, Reward, discount);
%[V,policy, iter] = mdp_policy_iteration(P, Reward, discount);
Image = imread('FlyingFish2Map2.bmp');
figure,imshow(Image)

%Draw lines from state to the state determined by the policy on the map

for i=1:1:number_of_States-2
    a = policy(i); %assign a to be the state we should go to according to the policy from state i
    x1 = state_Pixel(i,1); 
    y1 = state_Pixel(i,2); %assign x1 and y1 to be the x and y co-ordinates of the initial state i 
    x2 = state_Pixel(a,1);
    y2 = state_Pixel(a,2); %assign x2 and y2 to be the x and y co-ordinates of the final state determined by the policy
       hold on;
    x =[x1, x2]; %x is the x co-ordinates
    y =[y1, y2]; %y is the y co-ordinates
  
   line(x,y,'Color','blue','LineStyle','-') %draw line from the initial state to the state determined by the policy
end
