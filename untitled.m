obsState1 = [0 5 0];
obsState2 = [0 -5 0];
obsList = dynamicCapsuleList;
geom = struct("Length",7,"Radius",0.5,"FixedTransform",eye(3));

obsCapsule1 = struct('ID',1,'States',obsState1,'Geometry',geom);
obsCapsule2 = struct('ID',2,'States',obsState2,'Geometry',geom);

addObstacle(obsList,obsCapsule1);
addObstacle(obsList,obsCapsule2);

test(obsList)

show(obsList);
ylim([-20 20])


function test(obsList)
    geom = struct("Length",7,"Radius",0.5,"FixedTransform",eye(3));
    obsState1 = [5 0 0];
    obsCapsule1 = struct('ID',1,'States',obsState1,'Geometry',geom);
    updateObstaclePose(obsList,1,obsCapsule1);
end