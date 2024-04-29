

a = Verticies();
a.x = [1,2,3,4,5];
a.y = [11,12,13,14,15];
b = Verticies();
b.x = [-1,-2,-3,-4,-5];
b.y = [-11,-12,-13,-14,-15];


comb = Verticies.empty;
comb(1,1) = a;
comb(20,1) = b;

comb(19)
comb(20)