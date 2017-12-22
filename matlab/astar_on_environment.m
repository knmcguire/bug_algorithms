function optimal_path = astar_on_environment(environment,goal_coordinates,starting_position)

environment_large = environment;
%environment_large(find(environment_large==0))

environment_large(find(environment_large==255))=1;
goal_coordinate_env = int8(zeros(size(environment_large)));
goal_coordinate_env(floor(goal_coordinates(1)*10),floor(goal_coordinates(2)*10)) =1;
startX = starting_position(1)*10;
startY = starting_position(2)*10;
optimal_path=ASTARPATH(startX,startY,environment_large,goal_coordinate_env,1);


end

