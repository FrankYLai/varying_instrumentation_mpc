function car = setup_ego(configs)
    states = configs("start");
    configs("car_b") = 0.5;
    configs("car_r") = 0.1;
    
    car = Car(states(1),states(2),states(3),configs("car_b"),configs("car_r"));
    
    %reference only for collision logic

    addEgo(configs("obsList"),car.egoCapsule);
end

