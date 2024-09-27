function animate_leg_frames(TIC, TC_0N, T0_1, T1_2, T2_3)
    current_T = eye(4);
    tranimate(current_T, eye(4), 'rgb');
    disp('Pressione uma tecla para animar Base -> Link 1...');
    pause
    tranimate(current_T, current_T * TIC, 'rgb');
    disp('Pressione uma tecla para animar Link 1 -> Link 2...');
    pause
    current_T = current_T * TIC;
    tranimate(current_T, current_T * TC_0N, 'rgb');
    disp('Pressione uma tecla para animar Link 2 -> Link 3...');
    pause
    current_T = current_T * TC_0N;
    tranimate(current_T, current_T * T0_1, 'rgb');
    disp('Pressione uma tecla para animar Link 2 -> Link 3...');
    pause
    current_T = current_T * T0_1;
    tranimate(current_T, current_T * T1_2, 'rgb');
    disp('Pressione uma tecla para animar Link 2 -> Link 3...');
    pause
    current_T = current_T * T1_2;
    tranimate(current_T, current_T * T2_3, 'rgb');
    disp('Pressione uma tecla para animar Link 2 -> Link 3...');
end
