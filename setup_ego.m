function car = setup_ego(configs)
    states = configs("start");
    configs("car_b") = 0.5;
    configs("car_r") = 0.1;
    
    car = Car(states(1),states(2),states(3),configs("car_b"),configs("car_r"));
    
    %reference only for collision logic
    egoID1 = 1;
    geom = struct("Length",0.01,"Radius",1,"FixedTransform",eye(3));
    egoCapsule1 = struct('ID',egoID1,'States',states,'Geometry',geom);
    addEgo(configs("obsList"),egoCapsule1);
end

