function [StateSpace,ValueS] = ValueSplitting3(value, states, L, C, D, V)
    
splitDay = false;
splitCharge =  false;
% L = 2;
% D = 2;
% C = 3;
% Value = [2 1 3 2 4 4 4 3 5 4 6 6];
% states = [1 2 3 4 5 6];

c = 1;
for i=1:1:L
    for j=1:1:C
        for k=1:1:D
            F(j,k,i) = Value(c);
            c = c + 1;
        end
    end
end

for i=1:1:L
    Sza = size(F,1);
    Szb = size(F,2);
    for j=1:1:Sza
        for k=1:1:Szb
            if k > 1
                dif = F(j,k,i) - F(j,k-1,i);
                Dif = abs(dif);
                if Dif > 0.5
                    splitDay = true;
                end
            end
            if j > 1
                dif = F(j,k,i) - F(j-1,k,i);
                Dif = abs(dif);
                if Dif > 0.5
                    splitCharge = true;
                end
            end
        end

    end
end


if splitDay == true
    for i=1:1:L
        for j=1:1:Sza
            a = 1;
            for k=1:1:Szb
                F1(j,a,i) = F(j,k,i);
                a = a + 1;
                F1(j,a,i) = 0;
                a = a + 1;
            end
        end
    end
end

Szc = size(F1,2);
if splitCharge == true
    for i=1:1:L
        a = 1;
        for j=1:1:Sza
            for k=1:1:Szc
                F2(a,k,i) = F1(j,k,i);
            end
            a = a + 1;
            for k=1:1:Szc
                F2(a,k,i) = 0;
            end
            a = a + 1;
        end
    end
end

Sza = size(F2,1);
Szb = size(F2,2);

for i=1:1:L
    for j=1:1:Sza
        for k=1:1:Szb
            if F2(j,k,i) == 0 && k ~= 1
                F2(j,k,i) = F2(j,k-1,i);
            end
            if F2(j,k,i) == 0
               F2(j,k,i) = F2(j-1,k,i); 
            end
        end
    end
end

ValueS = reshape(F2,[],1);
ValueS = transpose(ValueS);
ChargeStates = Sza;
DayStates = Szb;


%end