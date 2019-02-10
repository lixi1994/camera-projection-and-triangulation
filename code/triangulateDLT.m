function [ predicted, distances ] = triangulateDLT( K1,R1,C1,Pimagepts1,K2,R2,C2,Pimagepts2 )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
num = size(Pimagepts1,2);
predicted = zeros(3,num);
distances = zeros(1,num);

for i=1:num
    u1 = R1'/K1*[Pimagepts1(:,i);1];
    u1 = u1/norm(u1);
    u2 = R2'/K2*[Pimagepts2(:,i);1];
    u2 = u2/norm(u2);
    u3 = cross(u1,u2);
    u3 = u3/norm(u3);
    x = inv([u1 -u2 u3])*(C2-C1);
    a = x(1,1);
    b = x(2,1);
    distances(1,i) = abs(x(3,1));
    p1 = C1+a*u1;
    p2 = C2+b*u2;
    p = (p1+p2)/2;
    predicted(:,i) = p;
end

end

