close all;
clear;
clc;

%Simulation stuff
tf = 2;
delta_t = 0.01;


x_d = 0; y_d = 0;
x_r = 0.5; y_r = 0.1;

%Controller coefficients
k_p = 0.2;
k_a=0.5;
k_b = -0.05;

V = 0.1; %Velocity
theta = pi; 
w = 0.01;

ro = sqrt((y_d(1)-x_r(1))^2+(x_d(1)-x_r(1))^2);
alpha = theta(1)+atan2(y_d(1)-y_r(1),x_d(1)-x_r(1));
beta = theta(1)-alpha(1);



for i=1:tf/delta_t
    
    %Desired position of robot
    x_d(i)=i*delta_t;
    y_d(i)=x_d(i).*x_d(i);
    
    %Current postition of robot
    y_r(i+1) = y_r(i)+k_p*ro(i)*sin(theta(i));
    x_r(i+1) = x_r(i)+k_p*ro(i)*cos(theta(i));
    
    ro(i+1) = sqrt((y_d(i)-x_r(i))^2+(x_d(i)-x_r(i))^2);
    alpha(i+1) = theta(i)+atan2(y_d(i)-y_r(i),x_d(i)-x_r(i));
    
    %Orientation of robot
    theta(i+1)=theta(i)+(k_a*alpha(i)+k_b*beta(i));
    
    beta(i+1) = theta(i)-alpha(i);
    
    plot(x_d,y_d,'bo'); hold on;
    plot(x_r, y_r, 'ro'); hold on;
    
    pause(0.05)
end
