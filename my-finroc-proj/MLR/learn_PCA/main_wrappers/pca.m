I = [2 3;3 4;4 5;5 6;5 7;2 1;3 2;4 2;4 3;6 4;7 6];
c = [  1;  1;  1;  1;  1;  2;  2;  2;  2;  2;  2]; 

c1 = I(c==1,:);
c2 = I(c==2,:);

figure;

p1 = scatter(c1(:,1), c1(:,2)); hold on;
p2 = scatter(c2(:,1), c2(:,2));

xlim([0 8])
ylim([0 8])

mu = mean(I);
Fi = bsxfun(@minus, I, mu);

A = Fi' * Fi;
[V, D] = eig(A); 
%eigshow(A);

% sort eigenvectors descending manner
[D, i] = sort(diag(D), 'descend');
V = V(:, i);


scale = 5;
pc1 = line([mu(1) - scale * V(1,1) mu(1) + scale * V(1,1)], [mu(2) - scale * V(2,1) mu(2) + scale * V(2,1)]);
pc2 = line([mu(1) - scale * V(1,2) mu(1) + scale * V(1,2)], [mu(2) - scale * V(2,2) mu(2) + scale * V(2,2)]);

set(pc1, 'color', [1 0 0])
set(pc2, 'color', [0 1 0])


cumsum(D)/sum(D);

% project on pc1
z = Fi*V(:,1);
% and reconstruct it
p = z*V(:,1)';
p = bsxfun(@plus, p, mu);


% delete old plots
%delete(p1);delete(p2);

figure;

y1 = p(c==1,:);
y2 = p(c==2,:);



p1 = scatter(y1(:,1),y1(:,2)); hold on;
p2 = scatter(y2(:,1), y2(:,2)); 


scale = 5;
pc1 = line([mu(1) - scale * V(1,1) mu(1) + scale * V(1,1)], [mu(2) - scale * V(2,1) mu(2) + scale * V(2,1)]);
pc2 = line([mu(1) - scale * V(1,2) mu(1) + scale * V(1,2)], [mu(2) - scale * V(2,2) mu(2) + scale * V(2,2)]);

set(pc1, 'color', [1 0 0])
set(pc2, 'color', [0 1 0])

% project on pc1
z = Fi*V(:,2);
% and reconstruct it
p = z*V(:,2)';
p = bsxfun(@plus, p, mu);


% delete old plots
%delete(p1);delete(p2);

figure;

y1 = p(c==1,:);
y2 = p(c==2,:);



p1 = scatter(y1(:,1),y1(:,2)); hold on;
p2 = scatter(y2(:,1), y2(:,2)); 


scale = 5;
pc1 = line([mu(1) - scale * V(1,1) mu(1) + scale * V(1,1)], [mu(2) - scale * V(2,1) mu(2) + scale * V(2,1)]);
pc2 = line([mu(1) - scale * V(1,2) mu(1) + scale * V(1,2)], [mu(2) - scale * V(2,2) mu(2) + scale * V(2,2)]);

set(pc1, 'color', [1 0 0])
set(pc2, 'color', [0 1 0])
