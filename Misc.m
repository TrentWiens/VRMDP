%% miscelaneous portions of code that could be useful 

    % from 'splitter' function - splits the states from the charge, per
    % specific location. ie; location 1 does not need to split charge but
    % location 2 does so it gives [1 2 ; 3 4; 5 6; 0 0] and [7 8; 9 10; 11
    % 12; 13 14] 

    %not used because of the zeros if it was not a tensor, you could split
    %differently

    % a = 1;
    % for i = 1:locStates
    %     b = 1;
    %     c = 1;
    %     for k = 1:chargeStates
    %         chargeSplitStates(c,1,i) = a;
    %         a = a + 1;
    %         chargeSplitStates(c,2,i) = a;
    %         a = a + 1;
    %         c = c + 1;
    %         if b < chargeStates && (chargeSpots(b,1,i) == 1 || chargeSpots(b,2,1) == 1)
    %             chargeSplitStates(c,1,i) = a;
    %             a = a + 1;
    %             chargeSplitStates(c,2,i) = a;
    %             a = a + 1;
    %             c = c + 1;
    %         end
    %         b = b + 1;
    %     end
    % end