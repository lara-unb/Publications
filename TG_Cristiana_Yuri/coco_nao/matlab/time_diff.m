filename = resultados.txt;
m = csvread(filename);

mt = m';

%
%
%do plotting here%
%
%

j1 = [mt(11, :); mt(1,:)]
j2 = [mt(11, :); mt(6,:)]

threshold = 0.08;

time_diff = [];

for i = 1:424
    for k = 1:424
        if abs(j1(2, i) - j2(2, k)) < threshold
            time_diff = [time_diff (j2(1,k) - j1(1,i))];
            break;
        end
    end
end

            