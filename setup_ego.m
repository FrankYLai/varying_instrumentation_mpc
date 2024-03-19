function car = setup_ego(configs)
    [x,y,theta] = configs("start");
    states = configs("start");
    car = Car(x,y);

    %reference only for collision logic
    egoID1 = 1;
    geom = struct("Length",0.01,"Radius",1,"FixedTransform",eye(3));
    egoCapsule1 = struct('ID',egoID1,'States',states,'Geometry',geom);
    addEgo(configs("obsList"),egoCapsule1);
end

