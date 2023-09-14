function x_opt = tf_output(output,N)
x_opt = [];
for k=1:N
    x = num2str(k);
    if k<10
        st = strcat('x00',x);
    elseif k<100
        st = strcat('x0',x);
    else
        st = strcat('x',x);
    end
    x_opt(:,k) = getfield(output,st); 
end
end   