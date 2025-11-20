% function dist = quaternion_geodesic_distance(q1, q2)
%     % Ensure input quaternions are column vectors
%     q1 = q1(:);
%     q2 = q2(:);
% 
%     % Conjugate of q2
%     q2_conj = [q2(1); -q2(2:4)];
% 
%     % Quaternion product q = q1 * q2_conj
%     q = quaternion_multiply(q1, q2_conj);
% 
%     % Logarithmic map of q (assumes q is unit quaternion)
%     v = q(2:4);           % vector part
%     w = q(1);             % scalar part
% 
%     theta = acos(max(min(w, 1), -1)); % clamp w for numerical stability
% 
% 
%         log_q = 2 * theta * v / (norm(v)+1e-13);
% 
%     % Geodesic distance
%     dist = norm(log_q);
% end

function dist = quaternion_geodesic_distance(q1, q2)
    % % Normalize the input quaternions to ensure unit norm
    % q1 = q1(:) / norm(q1);
    % q2 = q2(:) / norm(q2);

    % Compute the absolute value of the dot product
    dot_product = abs(dot(q1, q2));

    % Clamp for numerical stability
    dot_product = max(min(dot_product, 1.0), -1.0);

    % Compute geodesic distance (2 * angle)
    %dist = 2 * acos(dot_product);
    dist = acos(dot_product);
end
