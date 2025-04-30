function [q_leg1, q_leg2, q_leg3, q_leg4] = literal_inverse_kinematics(pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4)

    load('constants.mat', 'L1', 'L2', 'L3', 'leg1', 'leg2', 'leg3', 'leg4')   
    
    % Agrupamento das pernas e posições
    legs = {leg1, leg2, leg3, leg4};
    positions = {pos_leg_1, pos_leg_2, pos_leg_3, pos_leg_4};
    
    q_all = cell(1,4); % para armazenar os vetores de juntas das 4 pernas

    for i = 1:4
        leg = legs{i};
        pos = positions{i};
        
        % Origem da base da perna no frame do corpo (CM)
        p0 = transl(leg.base)';
        
        % Cálculo no plano Y-Z (visão frontal)
        n = abs(pos(2) - p0(2)); % y - y0
        m = abs(pos(3) - p0(3)); % z - z0
        p = sqrt(n^2 + m^2);     % hipotenusa
        
        beta = atan(n/m);
        tau = asin(L1 / p);
        theta_1 = beta - tau;
        
        % Cálculo no plano X-Z (visão lateral)
        R = sqrt(p^2 - L1^2);
        b = pos(1) - p0(1);
        c = R;
        a = sqrt(b^2 + c^2);
        phi = asin(b / a);
        alpha = acos((a^2 + L2^2 - L3^2) / (2 * a * L2));
        
        theta_2 = alpha - phi;
        theta_3 = -pi + acos((L2^2 + L3^2 - a^2) / (2 * L2 * L3));
        
        q = [theta_1; theta_2; theta_3];
        
        % Invertemos o sinal das juntas para as pernas 3 e 4
        if i == 3 || i == 4
            q = -q;
        end
        
        q_all{i} = q;
    end
    
    % Atribuição às variáveis de saída
    q_leg1 = q_all{1};
    q_leg2 = q_all{2};
    q_leg3 = q_all{3};
    q_leg4 = q_all{4};
end
