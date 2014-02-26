function qGoal = inverse_kinematics(rGoal,q0)

    % given are the functions 
    %   r_BF_inB(alpha,beta,gamma) and
    %   J_BF_inB(alpha,beta,gamma) 
    % for the foot positon respectively Jacobian

    r_BF_inB = @(alpha,beta,gamma)[
        - sin(beta + gamma) - sin(beta);
      sin(alpha)*(cos(beta + gamma) + cos(beta) + 1) + 1;
      -cos(alpha)*(cos(beta + gamma) + cos(beta) + 1)];

    J_BF_inB = @(alpha,beta,gamma)[0,- cos(beta + gamma) - cos(beta), -cos(beta + gamma);
     cos(alpha)*(cos(beta + gamma) + cos(beta) + 1),-sin(alpha)*(sin(beta + gamma) + sin(beta)), -sin(beta + gamma)*sin(alpha);
     sin(alpha)*(cos(beta + gamma) + cos(beta) + 1), cos(alpha)*(sin(beta + gamma) + sin(beta)),  sin(beta + gamma)*cos(alpha)];


    qGoal = q0;
    
    attempts=0;
    q=q0;

    J = J_BF_inB(q(1),q(2),q(3));
    Ji = pinv(J);

    r0 = r_BF_inB(q(1),q(2),q(3));
    dr = rGoal - r0;
    dist = norm(dr);
    prev_dist = dist+1;
    
    while(attempts<1000 && dist<prev_dist)
        qGoal = q;
        
        dq = Ji*dr;
        q = q +dq;
        
        r0 = r_BF_inB(q(1),q(2),q(3));
        dr = rGoal - r0;
        prev_dist = dist;
        dist = norm(dr);
        attempts = attempts+1;
    end
    
    