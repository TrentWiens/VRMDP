%Receding Horizon Code for the MDP Toy Problem

function [StateSpace, horizonValue] = recedingHorizon(position, Val, V, Q, L, C, D, goal1, goal2, StateMap)
AvgDpC = 2;
STD = 1;

Sz = size(V, 1);

%Create Matrix W to show which landing spots whould be considered
for i=1:1:Sz
    prob = prob_Drone_Distance(V(position,i), AvgDpC, STD);
    if prob > 0.2
        W(i) = 1;
    else
        W(i) = 0;
    end
end

a = 1;
b = 1;
for i=1:1:2
    for j=1:1:2
        for k=1:1:L
            for m=1:1:C
                for n=1:1:D
                    if W(k) == 1
                        if goal1 == i && goal2 == j
                            StateSpace(b) = Q(a);
                            horizonValue(b) = Val(a);
                            b = b + 1;
                        end
                    end
                    a = a + 1;
                end
            end
        end
    end
end

for i=1:1:2
    for j=1:1:2
        for k=1:1:L
            for m=1:1:C
                for n=1:1:Dfor i=1:1:2
                    for j=1:1:2
                        for k=1:1:L
                            for m=1:1:C
                                for n=1:1:Dfor i=1:1:2
                                    for j=1:1:2
                                        for k=1:1:L
                                            for m=1:1:C
                                                for n=1:1:D
                                                   if  
                                                    
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end
end