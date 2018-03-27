I = [2 3;3 4;4 5;5 6;5 7;2 1;3 2;4 2;4 3;6 4;7 6];
c = [  1;  1;  1;  1;  1;  2;  2;  2;  2;  2;  2];

c1 = I((c==1),:);
c2 = I((c==2),:);



figure;

p1 = scatter(c1(:,1), c1(:,2)); hold on;
p2 = scatter(c2(:,1), c2(:,2));

xlim([0 8]);
ylim([0 8]);

classes = max(c);
mu_total = mean(I);
mu = [mean(c1); mean(c2)];
FI_within = (I - mu(c,:));
A_within = FI_within' * FI_within;

Fi_between = [mean(c1) - mu_total;
              mean(c2) - mu_total];
A_between = Fi_between' * Fi_between;

[V, D] = eig(A_within\A_between);


% sort eigenvectors desc
[D, i] = sort(diag(D), 'descend');
V = V(:,i);


scale = 5;
pc1 = line([mu_total(1) - scale * V(1,1) mu_total(1) + scale * V(1,1)], [mu_total(2) - scale * V(2,1) mu_total(2) + scale * V(2,1)]);

set(pc1, 'color', [1 0 0]);


Xm = bsxfun(@minus, I, mu_total);

z = Xm*V(:,1);
% and reconstruct it
p = z*V(:,1)';
p = bsxfun(@plus, p, mu_total);

figure;

% delete old plots
% delete(p1);delete(p2);

y1 = p((c==1),:);
y2 = p((c==2),:);

p1 = scatter(y1(:,1),y1(:,2)); hold on;
p2 = scatter(y2(:,1), y2(:,2));

scale = 5;
pc1 = line([mu_total(1) - scale * V(1,1) mu_total(1) + scale * V(1,1)], [mu_total(2) - scale * V(2,1) mu_total(2) + scale * V(2,1)]);

set(pc1, 'color', [1 0 0]);