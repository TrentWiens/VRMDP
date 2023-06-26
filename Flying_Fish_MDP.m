%Example from map
clear all;

r = .5;
b = 0.33;
g = 0.17;

          % 1 2 3 4 5 6 7 8 9 10
P(:,:,1) = [r r b 0 0 0 b 0 0 0;
            r r r b g g g 0 0 0;
            b r r r r b 0 0 0 0;
            0 b r r r r 0 0 g g;
            0 g r r r r g g r b;
            0 g b r r r r b r r;
            b g 0 0 g r r r 0 g;
            0 0 0 0 g b r r b b;
            0 0 0 g r r 0 b r r;
            0 0 0 0 0 0 0 0 0 r]
Rs = 0.1;
R(:,:,:) = Rs*ones(10,10,1);
R(:,10,:) = 1;
discount = .9;
[V,policy, iter, cpu_time] = mdp_policy_iteration(P, R, discount);
V
policy

