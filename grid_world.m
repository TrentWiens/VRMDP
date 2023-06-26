% example from Russell and Norvig grid world
clear variables;close all;clc;

%  _____________
% 3| 8| 9|10|+1 11|
%  _____________
% 2| 5|XX| 6|-1 7|
%  _____________
% 1| 1| 2| 3| 4|
%  _____________
%    1  2  3  4

% number of states: 11
% number of actions: 4 (UP, LEFT, RIGHT, DOWN)

%% form Transition probabilities P as
% P(s,s',a)
% Taking action UP
%           1   2   3   4   5   6   7   8   9   10  11
P(:,:,1) = [0.1 0.1 0   0   0.8 0   0   0   0   0   0  ;...
            0.1 0.8 0.1 0   0   0   0   0   0   0   0  ;...
            0   0.1 0   0.1 0   0.8 0   0   0   0   0  ;...
            0   0   0.1 0.1 0   0   0.8 0   0   0   0  ;...
            0   0   0   0   0.2 0   0   0.8 0   0   0  ;...
            0   0   0   0   0   0.1 0.1 0   0   0.8 0  ;...
            0   0   0   0   0   0   1   0   0   0   0  ;...
            0   0   0   0   0   0   0   0.9 0.1 0   0  ;...
            0   0   0   0   0   0   0   0.1 0.8 0.1 0  ;...
            0   0   0   0   0   0   0   0   0.1 0.8 0.1;...
            0   0   0   0   0   0   0   0   0   0   1  ];
        
% Taking action LEFT
%           1   2   3   4   5   6   7   8   9   10  11
P(:,:,2) = [0.9 0   0   0   0.1 0   0   0   0   0   0  ;...
            0.8 0.2 0   0   0   0   0   0   0   0   0  ;...
            0   0.8 0.1 0   0   0.1 0   0   0   0   0  ;...
            0   0   0.8 0.1 0   0   0.1 0   0   0   0  ;...
            0.1 0   0   0   0.8 0   0   0.1 0   0   0  ;...
            0   0   0.1 0   0   0.8 0   0   0   0.1 0  ;...
            0   0   0   0   0   0   1   0   0   0   0  ;...
            0   0   0   0   0.1 0   0   0.9 0   0   0  ;...
            0   0   0   0   0   0   0   0.8 0.2 0   0  ;...
            0   0   0   0   0   0.1 0   0   0.8 0.1 0  ;...
            0   0   0   0   0   0   0   0   0   0   1  ];
        
% Taking action RIGHT
%           1   2   3   4   5   6   7   8   9   10  11
P(:,:,3) = [0.1 0.8 0   0   0.1 0   0   0   0   0   0  ;...
            0   0.2 0.8 0   0   0   0   0   0   0   0  ;...
            0   0   0.1 0.8 0   0.1 0   0   0   0   0  ;...
            0   0   0   0.9 0   0   0.1 0   0   0   0  ;...
            0.1 0   0   0   0.8 0   0   0.1 0   0   0  ;...
            0   0   0.1 0   0   0   0.8 0   0   0.1 0  ;...
            0   0   0   0   0   0   1   0   0   0   0  ;...
            0   0   0   0   0.1 0   0   0.1 0.8 0   0  ;...
            0   0   0   0   0   0   0   0   0.2 0.8 0  ;...
            0   0   0   0   0   0.1 0   0   0   0.1 0.8;...
            0   0   0   0   0   0   0   0   0   0   1  ];

% Taking action DOWN
%           1   2   3   4   5   6   7   8   9   10  11
P(:,:,4) = [0.9 0.1 0   0   0   0   0   0   0   0   0  ;...
            0.1 0.8 0.1 0   0   0   0   0   0   0   0  ;...
            0   0.1 0.8 0.1 0   0   0   0   0   0   0  ;...
            0   0   0.1 0.9 0   0   0   0   0   0   0  ;...
            0.8 0   0   0   0.2 0   0   0   0   0   0  ;...
            0   0   0.8 0   0   0.1 0.1 0   0   0   0  ;...
            0   0   0   0   0   0   1   0   0   0   0  ;...
            0   0   0   0   0.8 0   0   0.1 0.1 0   0  ;...
            0   0   0   0   0   0   0   0.1 0.8 0.1 0  ;...
            0   0   0   0   0   0.1 0   0   0   0.1 0.8;...
            0   0   0   0   0   0   0   0   0   0   1  ];
        
        
%% Now define rewards
% R(s,s',a)
% set up a mask matrix
%reward = -0.04;
%reward = -1.7;
%reward = -0.001;
reward = -0.1;
%reward = 0.1;
R = reward*ones(11,11,4);

% now fix the rewards for terminating states 7 and 11
% first, transitions from states into absorbing states
% landing in a terminal state gives a reward
R(:,7,:) = (-1)*ones(size(R(:,7,:)));
R(:,11,:) = (1)*ones(size(R(:,11,:)));

% next, transitions once in an absorbing states...
% 0 reward for absorbing states
R(7,:,:) = (0)*ones(size(R(:,7,:)));
R(11,:,:) = (0)*ones(size(R(:,11,:)));

%R(:,7,:) = (-1) + R(:,7,:);
%R(:,11,:) = (1) + R(:,11,:);

%% set the discount factor

% NOTE: In Russell and Norvig, "additive rewards" are used in the above
% gridworld problem. Additive rewards are equivalent to discounted rewards
% where gamma=1. However, only Dr. Bradley's methods work for gamma = 1.
%gamma = 0.999;
gamma = 1;

%% now solve the MDP
% turn on debug
%mdp_verbose;
% try policy iteration
%disp('MDPtoolbox policy iteration')
%[V, policy, iter, cpu_time] = mdp_policy_iteration(P, R, gamma)

% try linear programming
%disp('MDPtoolbox linear programming')
%[V, policy, cpu_time] = mdp_LP(P, R, gamma) 

disp('Dr. Bradley''s MDPvalue.m')
% try my value iteration
[Value,policy,iter] = MDPvalue(P,R,gamma,0.01)

disp('Dr. Bradley''s MDPpolicy.m')
% try my value iteration
[Value,policy,iter] = MDPpolicy(P,R,gamma)

%% Optimal Policy

% Plot a pretty policy
policy = reshape(policy,1,11);
actionstrings = {'   UP', ' LEFT', 'RIGHT', ' DOWN'};
newpolicy = [policy(1:5) 0 policy(6:end)]
newpolicy = reshape(newpolicy,4,3)';
cellpolicy = {actionstrings{policy(1:4)};
    actionstrings{policy(5)} '' actionstrings{policy(6)} '   -1';
    actionstrings{policy(8:end-1)} '   +1'};

% plot the policy
x = 1:5;
y = 1:4;
[X Y] = meshgrid(x,y);
% insert labels
plot(X,Y,'k','LineWidth',2)
hold on
plot([1 5],[1 1],'k','LineWidth',2)
plot([1 5],[2 2],'k','LineWidth',2)
plot([1 5],[3 3],'k','LineWidth',2)
plot([1 5],[4 4],'k','LineWidth',2)
%axis off
hold on
for n=1:4
    for p=1:3
        if ~((n==2 && p==2) || (n==4 && p==2) || (n==4 && p==3))
            switch newpolicy(p,n)
                case 1 % UP
                    X = [n+0.2 n+0.2];
                    Y = [p+0.2 p+0.7];
                    a = myarrow(X,Y);
                    a.Color = 'blue';
                    a.LineWidth = 3;
                case 2 % LEFT
                    X = [n+0.7 n+0.2];
                    Y = [p+0.3 p+0.3];
                    a = myarrow(X,Y);
                    a.Color = 'blue';
                    a.LineWidth = 3;
                case 3 % RIGHT
                    X = [n+0.2 n+0.7];
                    Y = [p+0.3 p+0.3];
                    a = myarrow(X,Y);
                    a.Color = 'blue';
                    a.LineWidth = 3;
                case 4 % DOWN
                    X = [n+0.9 n+0.9];
                    Y = [p+0.7 p+0.2];
%                     X = [x+0.05 x+0.05];
%                     Y = [y y-0.07];
                    a = myarrow(X,Y);
                    a.Color = 'blue';
                    a.LineWidth = 3;
            end
        end
        text(n+0.2,p+0.5,cellpolicy{p,n},'FontSize',18,'FontWeight','bold');
        % color in cell 2,2
        
    end
end
