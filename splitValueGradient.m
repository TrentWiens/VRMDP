%find places where the value gradiane is large enough for the states to be
%split

function [splitSpots] = splitValueGradient(horizonValue, savedLocs, savedLocsCoords)

    locationStates = size(horizonValue, 3); %find the amount of states per value
    chargeStates = size(horizonValue, 1);
    dayStates = size(horizonValue, 2);
    
    locDiffThreshold = 2; %thresholds for the different values
    chargeDiffThreshold = .4;
    dayDiffThreshold = .6;
    
    %% check for location splitting
    
    checked = zeros(1,2,2);
    
    a = 1;
    for k = 1:length(savedLocs)
        i = savedLocsCoords(k,1);
        j = savedLocsCoords(k,2);
    
        if j > 1 && ismember([i,j-1], savedLocsCoords,'rows') %check the adjacent to the left
            checked(a,:,1) = [i,j];
            checked(a,:,2) = [i,j-1];
            a = a + 1;
        end
    
        if i > 1 && ismember([i-1, j], savedLocsCoords,'rows') %check the adjacent above
            checked(a,:,1) = [i,j];
            checked(a,:,2) = [i-1,j];
            a = a + 1;
        end
    
    end
    
    locSplit = zeros(1,2,1);
    for i = 1:size(checked,1)
        loc1 = find(ismember(savedLocsCoords,checked(i,:,1),"rows")); %find the location equivalent
        loc2 = find(ismember(savedLocsCoords,checked(i,:,2),"rows"));
        if abs(horizonValue(:,:,loc1)-horizonValue(:,:,loc2)) > locDiffThreshold %see if the difference between the values is large enough
            locSplit(i,:,1) = [loc1,loc2]; %put into locSplit matrix
            locSplit(i,:,2) = 1;
        else
            locSplit(i,:,1) = [loc1,loc2]; %still put into locSplit matrix but zero instead
            locSplit(i,:,2) = 0;
        end
    
    end
    
    %% check for charge splitting
    
    %find the difference between all the values of each charge states
    valueDiffCharge = zeros(1,1,1);
    for k = 1:locationStates
        for i = 1:chargeStates - 1
            for j = 1:dayStates
                valueDiffCharge(i,j,k) = abs(horizonValue(i,j,k) - horizonValue(i + 1,j,k));
            end
        end
    end
    
    %find the places where the difference is larger than the threshold
    chargeSplit = zeros(1,1,1);
    for i = 1:locationStates
        for j = 1:chargeStates-1
            for k = 1:dayStates
                if valueDiffCharge(k,j,i) > chargeDiffThreshold
                    chargeSplit(k,j,i) = 1;
                else
                    chargeSplit(k,j,i) = 0;
                end
            end
        end
    end
    
    %% check for day splitting
    
    %find the difference between all the values of each day state
    valueDiffDay = zeros(1,1,1);
    for k = 1:locationStates
        for i = 1:dayStates - 1
            for j = 1:chargeStates
                valueDiffDay(j,i,k) = abs(horizonValue(j,i,k) - horizonValue(j,i + 1,k));
            end
        end
    end
    
    %find the plsaces where the difference is larger than the threshold
    daySplit = zeros(1,1,1);
    for i = 1:length(valueDiffDay)
        for j = 1:dayStates - 1
            for k = 1:chargeStates
                if valueDiffDay(k,j,i) > dayDiffThreshold
                    daySplit(k,j,i) = 1;
                else
                    daySplit(k,j,i) = 0;
                end
            end
        end
    end
    
    %% output
    
    splitSpots = {locSplit, chargeSplit, daySplit};
end