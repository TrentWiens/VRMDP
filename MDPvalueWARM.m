% Value iteration algorithm for MDPs

% T = S x S' x A

% R = S x S' x A

% discount = discount factor [0,1]

% epsilon = convergence factor epsilon*(1-discount)/discount

function [Value,policy,iter] = MDPvalueWARM(T,R,discount,epsilon,Value)

debug = 0;

% the probability tensor

% S x S' x A

% reward matrix

% S x S' x A

% discount = 0.9;

% convergence value

% epsilon = 0.01;

max_iterations = 1000;



num_states = size(T,1); % number of states

num_actions = size(T,3); % number of actions

% ensure the matrix has the right dimensions

Tsize = size(T);

if(~isequal(Tsize,[num_states num_states num_actions]))

    msg = sprintf('Size of T is incorrect, must be %d x %d x %d\n',num_states,num_state,num_actions);

    disp(msg)

end



Rsize = size(R);

if(~isequal(Rsize,[num_states num_states num_actions]))

    msg = sprintf('Size of R is incorrect, must be %d x %d x %d\n',num_states,num_states,num_actions);

    disp(msg)

end



% initialize the value with action 1 always chosen

% Value = zeros(1,num_states);

Value_diff = epsilon + 1; % arbitrarily pick a value diff

% loop until value converges

if(debug)

    fprintf('Iteration\tV_variation\n')

end

num_iterations = 0;

converged = epsilon*(1-discount)/discount;

while(Value_diff > converged && num_iterations < max_iterations)

    for s=1:num_states

        for a=1:num_actions

            expected_future_reward = 0;

            for sp=1:num_states

                expected_future_reward = expected_future_reward + T(s,sp,a)*(R(s,sp,a)+discount*Value(sp));

            end

            Q(s,a) = expected_future_reward;

        end

        Value_prev = Value;

        % find the best action

        [Value(s), policy(s)] = max(Q(s,:));

    end

    % ensure we've converged for each state

    Value_diff = max(abs(Value - Value_prev));

    num_iterations = num_iterations + 1;

    if(debug)

        fprintf('%d\t\t\t%G\n',num_iterations,Value_diff)

    end

end

iter = num_iterations;

end











