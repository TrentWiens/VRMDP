% Policy Iteration method for Solving MDPs
% T = S x S' x A
% R = S x S' x A
% discount = discount factor [0,1]
function [Value,policy,iter,cpu_time] = mdp_policy_iteration(T,R,discount)
tic
debug = 1;
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
Tsize = size(T,1,1,3);
state = [num_states num_states num_actions];
if(~isequal(Tsize,[num_states, num_states, num_actions]))
    msg = sprintf('Size of T is incorrect, must be %d x %d x %d\n',num_states,num_states,num_actions);
    disp(msg)
end

Rsize = size(R,1,1,3);
if(~isequal(Rsize,[num_states num_states num_actions]))
    msg = sprintf('Size of R is incorrect, must be %d x %d x %d\n',num_states,num_states,num_actions);
    disp(msg)
end

if(debug)
    fprintf('Iteration\tPolicy\n')
end

% choose arbitrary policy (1's)
policy_prime = ones(1,num_states);
policy = policy_prime+1;
Value = zeros(1,num_states);
num_iterations = 0;
%Value = calc_value_policy(policy_prime,Value);
while(~isequal(policy,policy_prime) && num_iterations < max_iterations)
    policy = policy_prime;
    % calculate value of the current policy
    Value = calc_value_policy(policy,Value);
    % now loop over the actions for each state and pick the best one
    for stemp=1:num_states
        policy_prime(stemp) = improve_policy(stemp,Value);
    end
    % Value = NewValue;
    num_iterations = num_iterations + 1;
    if(debug)
        disp([sprintf('%d\t\t\t',num_iterations),num2str(policy)])
    end
end
Value = calc_value_policy(policy,Value);
iter = num_iterations;

% nested functions for readability in the while loop above
    function val = calc_value_policy(policy,Value)
        epsilon = 0.01; % just a good number
        converged = epsilon*(1-discount)/discount;
        max_count = 1000; % another good number that should never be reached
        count = 0;
        Value_diff = epsilon + 1; % arbitrarily pick a value diff
        val = Value;
        Value_prev = Value;
        while(Value_diff > converged && count < max_count)
            for s=1:num_states
                expected_future_reward = 0;
                for sp=1:num_states
                    expected_future_reward = expected_future_reward + T(s,sp,policy(s))*(R(s,sp,policy(s))+discount*val(sp));
                end
                val(s) = expected_future_reward;
            end
            Value_diff = max(abs(val - Value_prev));
            Value_prev = val;
            % increase the iterator
            count = count + 1;
        end
        %disp([sprintf('%d',count)])
    end

    function new_policy = improve_policy(state,Value)
        for a=1:num_actions
            expected_future_reward = 0;
            for sp=1:num_states
%                expected_future_reward = expected_future_reward + T(state,a,sp)*Value(sp);
                expected_future_reward = expected_future_reward + T(state,sp,a)*(R(state,sp,a)+discount*Value(sp));
            end
            val_for_action(a) = expected_future_reward;
        end
        [temp,new_policy] = max(val_for_action);
    end

cpu_time = toc;

end