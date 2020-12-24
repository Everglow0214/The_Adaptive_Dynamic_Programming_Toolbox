function p = quaternion_prod(q,r)
% Quaternion product.

if numel(r) == 3
    r = [0;r];
end
q_matrix = [q(1) -q(2) -q(3) -q(4);
            q(2) q(1) -q(4) q(3);
            q(3) q(4) q(1) -q(2);
            q(4) -q(3) q(2) q(1)];
p = q_matrix * r;
end