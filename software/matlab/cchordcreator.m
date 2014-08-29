t = 1/2000 : 1/2000 : 3;
c = (1 + sin(2*pi*523.251*t))/2;
e = (1 + sin(2*pi*659.255*t))/2;
g = (1 + sin(2*pi*783.991*t))/2;

chord = (c + e + g)/3;
newchord = chord;

for i = size(chord,2)*3/5 : size(chord,2)
        newchord(i) = chord(i)*(size(chord,2) - i)*2.5/size(chord,2);
end

fprintf('{ ');
for i = 1: size(chord,2)    
    fprintf('%5.0f, ',1023*newchord(i));
    if(mod(i,1000) == 0)
        fprintf('\n')
    end
end

fprintf('};\n');