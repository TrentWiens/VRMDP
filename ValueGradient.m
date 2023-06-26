%Value Gradient

function [StateSpace, Value] = ValueGradient(Val, states)

Sz = size(states,2);
totVal = 0;
for i=1:1:Sz
    totVal = totVal + Val(i);
end

avgVal=totVal/Sz;

a = 1;

for i=1:1:Sz
    if Val(i) >= avgVal
        StateSpace(a) = states(i);
        Value(a)= Val(i);
        a = a + 1;
    end
end

end