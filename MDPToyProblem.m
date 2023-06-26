%MDP toy problem written for mdp research
%Written by Marc Lussier


tic

%stateSpace(:,:,:,:,:,:,:) = zeros(100,5,4,2,2,2); %Landing spots, charge states,time of day, goal 1, goal 2, goal 3
dimensions = 4;
landing_Spots = dimensions^2;
charge_States = 3;
day_States = 2;
landingSpots(:,:) = zeros(landing_Spots,2);
avg_Flight_Distance_Full = 3;
STD = 1;
reward1 = 7;
reward2 = 12;

%Build the map the drone will traverse. It will be a 16x16 grid.
i = 1;
for j=1:1:dimensions
    for k=1:1:dimensions
        landingSpots(i,:) = [j,k];
        i = i + 1;
    end
end 

%Establish the distances between different landing spots. 
for i=1:1:landing_Spots
    for j=1:1:landing_Spots
        V(i,j) = abs(landingSpots(i,1) - landingSpots(j,1)) + abs(landingSpots(i,2) - landingSpots(j,2));
    end
end


%Build the matrix of probabilities of getting from one state to another.
%In this matrix we consider the probability of completing 3 goals, (the for
%loops that have 2 iterations, completed or not. We consider the
%probability from getting from 1 landing spot with a specific charge to
%another and the probability of charging. 
%This is not the transition probability matrix.
a = 1;

%for i=1:1:2
    for j=1:1:2
        for k=1:1:2
            for m=1:1:landing_Spots
                charge = 0;
                for n=1:1:charge_States
                    for p=1:1:day_States
                        b = 1;
                        %for q=1:1:2
                            for r=1:1:2
                                for s=1:1:2
                                    for t=1:1:landing_Spots
                                        final_Charge = 0;
                                        distribution = prob_Chadistrge(5, V(m,t), 1);
                                        for u=1:1:charge_States
                                            for v=1:1:day_States
                                                %if i == q && j == r && k == s
                                                
                                                if a == b
                                                    Prob(a,b) = 1;
                                                else
                                                    if j == r && k ==s
                                                    %Probability of getting
                                                    %from one landing spot and
                                                    %charge state to another
                                                    %You will not leave a day
                                                    %state when flying as the
                                                    %flight time << than the
                                                    %time of a day state. 
                                                        if v == p
                                                            if charge < final_Charge
                                                                Prob(a,b) = 0;
                                                            else
                                                                charge_Dif = charge - final_Charge;
                                                                charge_Index = round(charge_Dif / 50, 0) + 1;
                                                                if charge_Index == 1
                                                                    Prob(a,b) = distribution(charge_Index);
                                                                else
                                                                    Prob(a,b) = distribution(charge_Index) - distribution(charge_Index - 1);
                                                                end
                                                            end
                                                        else
                                                            Prob(a,b) = 0;
                                                        end
                                                
                                                %Probability of charging.
                                                %Stay at same landing spot
                                                %but increase charge. This
                                                %assumes using 1 full day
                                                %state. Set the ability to
                                                %charge depending on the
                                                %time of day starting with
                                                %the morning being 1. 
                                                        if m == t
                                                        %if p == 1 | p == 3
                                                        %    chargeTOD = 50;
                                                        %    chSTD = 20;
                                                        %elseif p == 2
                                                        %    chargeTOD = 80;
                                                        %    chSTD = 10;
                                                        %else
                                                        %    chageTOD = 0;
                                                        %    chSTD = 0;
                                                        %end
                                                            if p == 1
                                                                chargeTOD = 100;
                                                                chSTD = 0;
                                                            else 
                                                                chargeTOD = 0;
                                                                chSTD = 0;
                                                            end
                                                            prob_of_Charge = ChargingProbability(chargeTOD, chSTD);
                                                            charge_Dif = charge - final_Charge;
                                                            charge_Index = -round(charge_Dif / 50, 0) + 1;
                                                    
                                                        %c is the next day
                                                        %state. If it is night
                                                        %(4) c = 1
                                                            if p == day_States
                                                                c = 1;
                                                            else
                                                                c = p + 1;
                                                            end
                                                    
                                                    %If the arrival day
                                                    %state is = c, or 1 day
                                                    %state after deciding
                                                    %to charge.
                                                            if v == c
                                                                if charge_Index == 1
                                                                    Prob(a,b) = prob_of_Charge(charge_Index);
                                                                elseif charge_Index > 0
                                                                    Prob(a,b) = prob_of_Charge(charge_Index) - prob_of_Charge(charge_Index-1);
                                                                else 
                                                                    Prob(a,b) = 0;
                                                                end
                                                            end
                                                        end
                                                    elseif k == 1 && s == 2 && m == reward1 && t == m
                                                        Prob(a,b) = 0.8;
                                                    elseif j == 1 && r == 2 && m == reward2 && t == m
                                                        Prob(a,b) = 0.8;
                                                %elseif i == 1 && q == 2 && m == 64 && t == m
                                                %    Prob(a,b) = 0.8;
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
                        %end 
                        Q(a) =  a;
                        TheStateSpace(j,k,m,n,p) = a;
                        a = a + 1;
                    end
                    charge = charge + 50;
                end
            end
        end
    end
%end

a = 1;
for i=1:1:2
    for j=1:1:2
        for k=1:1:landing_Spots
            for m=1:1:charge_States
                for n=1:1:day_States
                    Q(a) = a;
                end
            end
        end
    end
end

OTW(:,:,:) = zeros(landing_Spots, landing_Spots, landing_Spots);
for i=1:1:landing_Spots
    for j=1:1:landing_Spots
        a = abs(landingSpots(j,1) - landingSpots(i,1));
        b = abs(landingSpots(j,2) - landingSpots(i,2));
        for k=1:1:landing_Spots
            %This program only considers traveling in the x or y direction
            %at a time. 
            %The route: The drone will go the longer direction first, i.e.
            %if the destination is 3 steps in the y direction and 2 in the
            %x the drone will travel in the y direction first. If they are
            %the same the drone will travel in x first. 
            %The following determines which spots are on the way. If the
            %drone directly flies over a spot it is determined to be on the
            %way. 
            if a > b
                if ((landingSpots(k,1) >= landingSpots(j,1) && landingSpots(k,1) <= landingSpots(i,1)) || (landingSpots(k,1) <= landingSpots(j,1) && landingSpots(k,1) >= landingSpots(i,1))) && (landingSpots(k,2) == landingSpots(i,2))
                    OTW(j,k,i) = 1;
                elseif ((landingSpots(k,2) >= landingSpots(j,2) && landingSpots(k,2) <= landingSpots(i,2)) || (landingSpots(k,2) <= landingSpots(j,2) && landingSpots(k,2) >= landingSpots(i,2))) && (landingSpots(j,1) == landingSpots(k,1))
                    OTW(j,k,i) = 1;
                end
            else 
                if ((landingSpots(k,2) >= landingSpots(j,2) && landingSpots(k,2) <= landingSpots(i,2)) || (landingSpots(k,2) <= landingSpots(j,2) && landingSpots(k,2) >= landingSpots(i,2))) && (landingSpots(i,1) == landingSpots(k,1))
                    OTW(j,k,i) = 1;
                elseif ((landingSpots(k,1) >= landingSpots(j,1) && landingSpots(k,1) <= landingSpots(i,1)) || (landingSpots(k,1) <= landingSpots(j,1) && landingSpots(k,1) >= landingSpots(i,1))) && (landingSpots(k,2) == landingSpots(j,2))
                    OTW(j,k,i) = 1;
                end
            end
        end
    end
end


a = 1;
%for i=1:1:2
    for j=1:1:2
        for k=1:1:2
            for m=1:1:landing_Spots
                for n=1:1:charge_States
                    for o=1:1:day_States
                        b = 1;
                        %for p=1:1:2
                            for q=1:1:2
                                for r=1:1:2
                                    for s=1:1:landing_Spots
                                        for t=1:1:charge_States
                                            for u=1:1:day_States
                                                c = 1;
                                                totProb = 0;
                                                OTWProb = 0;
                                                %for v=1:1:2
                                                    for w=1:1:2
                                                        for x=1:1:2
                                                            for y=1:1:landing_Spots
                                                                for z=1:1:charge_States
                                                                    for yz=1:1:day_States
                                                                        if m == y
                                                                            totProb = totProb + Prob(b,c);
                                                                        else
                                                                            if OTW(s,y,m) == 1
                                                                                OTWProb = OTWProb + Prob(b,c);
                                                                            end
                                                                        end
                                                                        c = c + 1;
                                                                    end
                                                                end
                                                            end
                                                        end
                                                        RProb = 1 - totProb;
                                                        coefficient(b,a) = RProb/OTWProb;
                                                    end
                                                b = b + 1;    
                                                %end
                                            end
                                        end
                                    end
                                end    
                            end
                        a = a + 1;    
                        %end
                    end
                end
            end
        end
    end
%end



sz = size(Prob,1);
P = zeros(sz,sz,sz);
R = -0.04*ones(sz,sz,sz);
a = 1;
d = 1;
%for i=1:1:2
    for j=1:1:2
        for k=1:1:2
            for m=1:1:landing_Spots
                for n=1:1:charge_States
                    for o=1:1:day_States
                        b = 1;
                        %for p=1:1:2
                            for q=1:1:2
                                for r=1:1:2
                                    for s=1:1:landing_Spots
                                        for t=1:1:charge_States
                                            for u=1:1:day_States
                                                c = 1;
                                                %for v=1:1:2
                                                    for w=1:1:2
                                                        for x=1:1:2
                                                            for y=1:1:landing_Spots
                                                                for z=1:1:charge_States
                                                                    for yz=1:1:day_States
                                                                        if j == 2 && k == 2
                                                                            if c == b
                                                                                P(b,c,a) = 1;
                                                                            else
                                                                                P(b,c,a) = 0;
                                                                            end
                                                                        else
                                                                            if s == m
                                                                                if y == m
                                                                                    P(b,c,a) = Prob(b,c);
                                                                                else
                                                                                    P(b,c,a) = 0;
                                                                                end
                                                                            else
                                                                                if m == y
                                                                                    P(b,c,a) = Prob(b,c);
                                                                                else
                                                                                    if OTW(s,y,m) == 1
                                                                                        P(b,c,a) = coefficient(b,a) * Prob(b,c);
                                                                                    else
                                                                                        P(b,c,a) = 0;
                                                                                    end
                                                                                end
                                                                            if (y == reward1 && w == 2) || (y == reward2 && x == 2)
                                                                                R(b,c,a) = 1;
                                                                            end
                                                                            d = d + 1;
                                                                            c = c + 1;
                                                                            end
                                                                        end
                                                                    end
                                                                end
                                                            end
                                                        end
                                                    end
                                                b = b + 1;
                                                %end
                                            end
                                        end
                                    end
                                end
                            end
                            a = a + 1;
                        %end
                    end
                end
            end
        end
    end
    
%end

% PPP = zeros(sz,sz);
% for i=1:1:384
%     for k=1:1:384
%         for j=1:1:384
%             PPP(i,k) = PPP(i,k) + P(i,j,k);
%         end
%     end
% end

discount = 0.95;
%Value = zeros(1,sz);

[Value,policy,iter] = MDPvalue(P,R,discount,.01);

toc 
% 
% pos = 1;
% goal1=1;
% goal2=1;
% [StateSpace, horizonValue] = recedingHorizon(pos, Value, V, Q, landing_Spots, charge_States, day_States, goal1, goal2);
% [StateSpace2, ValueGrad] = ValueGradient(horizonValue, StateSpace);
% [StateSpace3, ValueS] = ValueSplitting(Value, StateSpace2, ValueGrad);