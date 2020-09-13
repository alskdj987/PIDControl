clear all;clc;delete(instrfind)
t = tcpip('192.168.0.20', 23);
fopen(t);
pause(0.5);
disp('GO')

N = 100;
x = 1:N;
y = zeros(N,1);
fig = figure(1);
hp = plot(x,y,'o-');
ylim([-40 40])

while(ishandle(fig))
    v0 = fgetl(t);
    y(1:(end-1)) = y(2:end);
    y(end) = str2double(v0);
    hp.YData = y;
    pause(0.01);     

end

fclose(t)
delete(t)
clear t
disp('End')
