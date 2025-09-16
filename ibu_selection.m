clc;
clear all;
close all;

alpha = 19*21*pi;
beta = 20*2*pi;

us = linspace(10000, 20000);
bs = linspace(1, 100);

for i = 1:100
    Is(i) = (us(i) - bs(i)*beta)/alpha;
end

plot(us,Is)