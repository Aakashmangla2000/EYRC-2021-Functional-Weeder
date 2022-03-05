defmodule Task4CClientRobotA do

  @moduledoc """
  This module implements the client-A functionality of the theme.
  This includes moving and controlling the robot on arena and sending updates to liveview. It performs seeding part of the theme.
  """
  # max x-coordinate of table top
  @table_top_x 6
  # max y-coordinate of table top
  @table_top_y :f
  # mapping of y-coordinates
  @robot_map_y_atom_to_num %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5, :f => 6}

  @doc """
  ALl the place functions puts the robot to the provided position of (x, y, facing), this is used in start function.
  Prevents it to be placed outside of the table and facing invalid direction.
  Examples:
      iex> Task4CClientRobotA.place(1, :b, :south)
      {:ok, %Task4CClientRobotA.Position{facing: :south,x: 1, y: :b}}

      iex> Task4CClientRobotA.place(-1, :f, :north)
      {:failure, "Invalid position"}

      iex> Task4CClientRobotA.place(3, :c, :north_east)
      {:failure, "Invalid facing direction"}
  """
  def place do
    {:ok, %Task4CClientRobotA.Position{}}
  end

  def place(x, y, _facing) when x < 1 or y < :a or x > @table_top_x or y > @table_top_y do
    {:failure, "Invalid position"}
  end

  def place(_x, _y, facing) when facing not in [:north, :east, :south, :west] do
    {:failure, "Invalid facing direction"}
  end

  def place(x, y, facing) do
    {:ok, %Task4CClientRobotA.Position{x: x, y: y, facing: facing}}
  end

  @doc """
  Provide START position to the robot as given location of (x, y, facing) and place it.
  """
  def start(x, y, facing) do
    place(x, y, facing)
  end

  @doc """
  Main function to initiate the sequence of tasks to achieve by the Client Robot A,
  such as connect to the Phoenix server, get the robot A's start and goal locations to be traversed.
  Call the respective functions from this module and others as needed.
  """
  def main do

    ###########################
    ## complete this funcion ##
    ###########################
    {:ok, _response,channel} = Task4CClientRobotA.PhoenixSocketClient.connect_server()
    start = repss(channel,0)
    {x,y,facing} = change_start(start)
    goal_locs = Task4CClientRobotA.PhoenixSocketClient.get_goals(channel)
    {:ok, robot} = start(x,y,facing)
    _obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
    {_,robot} = stop(robot,goal_locs,channel)
    Task4CClientRobotA.PhoenixSocketClient.get_bot_position(false,channel,robot)
    Task4CClientRobotA.PhoenixSocketClient.get_bot_position(false,channel,robot)
    Task4CClientRobotA.PhoenixSocketClient.done(channel)
  end

  def change_start(str) do
    [str] = str
    str = String.replace(str, " ", "")
    pattern = :binary.compile_pattern([" ", ","])
    ls = String.split(str, pattern)

    {x, ls} = List.pop_at(ls,0)
    x = String.to_integer(x)
    {y, ls} = List.pop_at(ls,0)
    y = String.to_atom(y)
    {facing, _ls} = List.pop_at(ls,0)
    facing = String.to_atom(facing)
    {x,y,facing}
  end

  def repss(channel,start) do
    if(start == 0) do
      start = Task4CClientRobotA.PhoenixSocketClient.get_start(channel)
      repss(channel,start)
    else
      start
    end
  end

  @doc """
  stop function is called in the main fucntion. It provide GOAL positions to the robot as given location of [(x1, y1),(x2, y2),..]
  and plan the path from START to these locations.
  """
  def stop(robot, goal_locs,channel) do

    ###########################
    ## complete this funcion ##
    ###########################

    mp = %{"1" => 1, "2" => 2, "3" => 3, "4" => 4, "5" => 5, "6" => 6}
    mp2 = %{"a" => :a, "b" => :b, "c" => :c, "d" => :d, "e" => :e, "f" => :f}

    {motor_ref,pwm_ref} = Task4CClientRobotA.LineFollower.open_motor_pwm_pins()
    count = Enum.count(goal_locs)
    robot = goal_div(motor_ref,robot, goal_locs, channel,count,mp,mp2)

    {:ok, robot}

  end

  @doc """
  goal_select, min and min2 functions calls the nearby function to determine the nodes of the goal and
  then select the closest node to the robot's current position.
  """
  def goal_select(robot, parent, goal_locs,channel,min, index, count,mp,mp2) when count > 0 do
    mp3 = %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5, :f => 6}
    val = Enum.at(goal_locs,count)
    ls = nearby(val)
    min2 = abs(robot.x - Enum.at(Enum.at(ls,0),0)) + abs(Map.get(mp3, robot.y) -  Enum.at(Enum.at(ls,0),1))
    {_,[goal_x, goal_y]} = min(robot,ls,1,min2,0)
    distance = abs(robot.x - goal_x) + abs(Map.get(mp3, robot.y) -  goal_y)

    {index,min} = if(min >= distance) do
      {count, distance}
    else
      {index,min}
    end
    count = count - 1
    goal_select(robot, parent, goal_locs,channel, min,index, count,mp,mp2)
  end

  def goal_select(_robot, _parent, _goal_locs, _channel,_min, index, _count,_mp,_mp2) do
    index
  end

  @doc """
  nearby function takes the goal and applies a formula so as to find the four nodes surrounding it.
  """
  def nearby(goal) do
    goal = String.to_integer(goal)
    x = rem(goal,5)
    x = if(x == 0) do
      5
    else
      x
    end
    y = if(x == 5) do
      div(goal,5)
    else
      div(goal,5) + 1
    end
    [[x,y],[x+1,y],[x,y+1],[x+1,y+1]]
  end

  def min(robot,ls,count,min,index) when count < 4 do
    mp3 = %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5, :f => 6}
    distance = abs(robot.x - Enum.at(Enum.at(ls,count),0)) + abs(Map.get(mp3, robot.y) -  Enum.at(Enum.at(ls,count),1))
    {index,min} = if(min >= distance) do
      {count, distance}
    else
      {index,min}
    end
    min(robot,ls,count+1,min,index)
  end

  def min(_robot,ls,_count,_min,index) do
    Enum.fetch(ls,index)
  end

  @doc """
  goal_div functions select the best goal out of all the goals, based on the robot's current position.
  It calls goal_select to find the nearest node to the selected goal plant. Then it calls get_value to do the further process.
  """
  def goal_div(motor_ref,robot, goal_locs,channel,count,mp,mp2) when count > 0 do
    mp3 = %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5, :f => 6}
    parent = self()
      count = Enum.count(goal_locs)
      {robot,goal_locs,_count} = if(count == 0) do
        {robot,goal_locs,count}
      else
        count = Enum.count(goal_locs)
        index = 0
        {goal,goal_locs,goal_x, goal_y} = if(count > 0) do
          {val,index} = if (count > 1) do
            val = Enum.at(goal_locs,0)
            ls = nearby(val)
            min = abs(robot.x - Enum.at(Enum.at(ls,0),0)) + abs(Map.get(mp3, robot.y) -  Enum.at(Enum.at(ls,0),1))
            {_,[goal_x, goal_y]} = min(robot,ls,1,min,0)
            min = abs(robot.x - goal_x) + abs(Map.get(mp3, robot.y) -  goal_y)
            index = 0
            index = goal_select(robot, parent, goal_locs,channel,min, index, count-1,mp,mp2)
            {Enum.at(goal_locs,index),index}
          else
            {Enum.at(goal_locs,0),index}
          end
            {goal,goal_locs} = List.pop_at(goal_locs,index)
            mp4 = %{1 => :a, 2 => :b, 3 => :c, 4 => :d, 5 => :e, 6 => :f}
            ls = nearby(val)
            min = abs(robot.x - Enum.at(Enum.at(ls,0),0)) + abs(Map.get(mp3, robot.y) -  Enum.at(Enum.at(ls,0),1))
            {_,[goal_x, goal_y]} = min(robot,ls,1,min,0)
          {goal,goal_locs,goal_x, Map.get(mp4,goal_y)}
        else
          {0,goal_locs,robot.x, robot.y}
        end
        count = Enum.count(goal_locs)
        {robot} = get_value(motor_ref,goal,robot,goal_x, goal_y,channel)
        {robot,goal_locs,count}
      end
      count = Enum.count(goal_locs)

    if(Enum.count(goal_locs) == 2) do
      goal_div(motor_ref,robot, goal_locs,channel,0,mp,mp2)
    else
      goal_div(motor_ref,robot, goal_locs,channel,count,mp,mp2)
    end
  end

  def goal_div(motor_ref,robot, _goal_locs, _channel, _count,_mp,_mp2) do
   robot
  end

  @doc """
  get_value function defines the queue and visited queue for the implementation of deapth first search algorithm
  for finding the best possible path. The function calls another function rep and calls the seeding function
  from Task4CClientRobotA.ArmMechanismTest module to perform sowing of the seed
  """
  def get_value(motor_ref,goal,robot,goal_x, goal_y,channel) do
      len = 1
      q = :queue.new()
      visited = :queue.new()
      el = decide_dir(robot.facing, robot.x,robot.y,goal_x,goal_y)
      dir = [el]
      q = :queue.in({robot.x,robot.y,dir},q)
      visited = :queue.in({robot.x,robot.y},visited)
      IO.puts("Goal position: #{goal_x} #{goal_y}")
      {robot} = if(robot.x == goal_x and robot.y == goal_y) do
        {robot}
      else
        Task4CClientRobotA.rep(motor_ref,goal,dir,q,visited,robot,goal_x,goal_y,channel,len)
      end
      #sowing action to server
      IO.puts("Sowing the seed")
      Task4CClientRobotA.PhoenixSocketClient.sowing2(channel,String.to_integer(goal))
      robot = Task4CClientRobotA.ArmMechanismTest.seeding(channel,motor_ref,robot,goal)
      Task4CClientRobotA.PhoenixSocketClient.sowing(channel,String.to_integer(goal))
      {robot}
  end

  def rec_value() do
    receive do
      {:flag_value, flag} -> flag
    end
  end

  @doc """
  plus and minus functions are used to increment or decrement of y position coordinate of the robot
  """
  def plus(y) do
    Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)
  end
  def minus(y) do
    Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)
  end

  @doc """
  decide_dir defines the first step of the robot in terms of x and y coordinate
  along with the facing based on the next goal it has to reach
  """
  def decide_dir(facing,x,y,goal_x,goal_y) do
    mp2 = %{:a => 0, :b => 1, :c => 2, :d => 3, :e => 4, :f => 5}
    goal_y = Map.get(mp2, goal_y)
    y = Map.get(mp2, y)
    dir = cond do
      goal_x > x and goal_y > y ->
        dir = if(goal_x - x > goal_y - y) do
          if(facing == :south) do
            3
          else
            0
          end
        else
          if(facing == :west) do
            0
          else
            3
          end
        end
        dir
      goal_x > x and goal_y < y ->
        dir = if(goal_x - x > y - goal_y and facing != :north) do
          2
        else
          if(facing == :west) do
            2
          else
            3
          end
        end
        dir
      goal_x < x and goal_y > y ->
        dir = if(x - goal_x < goal_y - y and facing != :east) do
          1
        else
          0
        end
        dir
      goal_x < x and goal_y < y ->
        dir = if(x - goal_x > y - goal_y) do
          if(facing == :south or facing == :north) do
            1
          else
            2
          end
        else
          if(facing == :east) do
            2
          else
            1
          end
        end
        dir
      goal_x == x and goal_y == y ->
        cond do
          facing == :north ->
            0
          facing == :south ->
            2
          facing == :west ->
            1
          facing == :east ->
            3
        end
      goal_x == x ->
        dir = if(goal_y > y) do
          0
        else
          2
        end
        dir
      goal_y == y ->
        dir = if(goal_x > x) do
          3
        else
          1
        end
        dir
    end
    dir
  end

  @doc """
  dir_select decides the next step of the robot if it encounters an obstacle in its path
  """
  def dir_select(facing,x, y, goal_x, goal_y, dir) do
    mp2 = %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5, :f => 6}
    goal_y = Map.get(mp2, goal_y)
    y = Map.get(mp2, y)

    cond do
      goal_x > x and goal_y > y ->
        if (goal_x - x) < (goal_y - y)  do
            cond do
              Enum.member?(dir, 3) == :false ->
                List.insert_at(dir,-1,3)
              Enum.member?(dir, 0) == :false ->
                List.insert_at(dir,-1,0)
              Enum.member?(dir,1) == :false ->
                List.insert_at(dir,-1,1)
              Enum.member?(dir,2) == :false ->
                List.insert_at(dir,-1,2)
              true ->
                List.insert_at(dir,-1,4)
            end

          else
          #up
            cond do
              Enum.member?(dir, 0) == :false ->
                List.insert_at(dir,-1,0)
              Enum.member?(dir, 3) == :false ->
                List.insert_at(dir,-1,3)
              Enum.member?(dir,1) == :false ->
                List.insert_at(dir,-1,1)
              Enum.member?(dir,2) == :false ->
                List.insert_at(dir,-1,2)
              true ->
                List.insert_at(dir,-1,4)
            end
        end

      goal_x < x and goal_y < y ->
         cond do
          (x - goal_x) < (y - goal_y) ->
            cond do
              Enum.member?(dir, 1) == :false ->
                List.insert_at(dir,-1,1)
              Enum.member?(dir, 2) == :false ->
                List.insert_at(dir,-1,2)
              Enum.member?(dir,3) == :false ->
                List.insert_at(dir,-1,3)
              Enum.member?(dir,0) == :false ->
                List.insert_at(dir,-1,0)
              true ->
                List.insert_at(dir,-1,4)
            end

          true ->
            cond do
              Enum.member?(dir, 2) == :false ->
                List.insert_at(dir,-1,2)
              Enum.member?(dir, 1) == :false ->
                List.insert_at(dir,-1,1)
              Enum.member?(dir,3) == :false ->
                List.insert_at(dir,-1,3)
              Enum.member?(dir,0) == :false ->
                List.insert_at(dir,-1,0)
              true ->
                List.insert_at(dir,-1,4)
            end
        end

      goal_x > x and goal_y < y ->
         cond do
          (goal_x - x) < (y - goal_y) ->
            cond do
              Enum.member?(dir, 3) == :false ->
                List.insert_at(dir,-1,3)
              Enum.member?(dir, 2) == :false ->
                List.insert_at(dir,-1,2)
              Enum.member?(dir,1) == :false ->
                List.insert_at(dir,-1,1)
              Enum.member?(dir,0) == :false ->
                List.insert_at(dir,-1,0)
              true ->
                List.insert_at(dir,-1,4)
            end

          true ->
            cond do
              Enum.member?(dir, 2) == :false ->
                List.insert_at(dir,-1,2)
              Enum.member?(dir, 3) == :false ->
                List.insert_at(dir,-1,3)
              Enum.member?(dir,1) == :false ->
                List.insert_at(dir,-1,1)
              Enum.member?(dir,0) == :false ->
                List.insert_at(dir,-1,0)
              true ->
                List.insert_at(dir,-1,4)
            end
        end

      goal_x < x and goal_y > y ->
           cond do
            (x - goal_x) < (goal_y - y) ->
              cond do
                (Enum.member?(dir, 1) or facing == :east) == :false ->
                  List.insert_at(dir,-1,1)
                Enum.member?(dir, 0) == :false ->
                  List.insert_at(dir,-1,0)
                Enum.member?(dir,3) == :false ->
                  List.insert_at(dir,-1,3)
                Enum.member?(dir,2) == :false ->
                  List.insert_at(dir,-1,2)
                true ->
                  List.insert_at(dir,-1,4)
              end

            true ->
              cond do
                Enum.member?(dir, 0) == :false ->
                  List.insert_at(dir,-1,0)
                Enum.member?(dir, 1) == :false ->
                  List.insert_at(dir,-1,1)
                Enum.member?(dir,3) == :false ->
                  List.insert_at(dir,-1,3)
                Enum.member?(dir,2) == :false ->
                  List.insert_at(dir,-1,2)
                true ->
                  List.insert_at(dir,-1,4)
              end
          end

          goal_x == x ->
            cond do
             (goal_y > y) ->
               cond do
                 Enum.member?(dir, 0) == :false ->
                  List.insert_at(dir,-1,0)
                 (Enum.member?(dir, 1) or (x == 1) or facing == :east) == :false ->
                  List.insert_at(dir,-1,1)
                 (Enum.member?(dir,3) or (x == 6)) == :false ->
                  List.insert_at(dir,-1,3)
                  (Enum.member?(dir,2)) or (y == 1) == :false ->
                    List.insert_at(dir,-1,2)
                 true ->
                  List.insert_at(dir,-1,4)
               end

             true ->
               cond do
                 Enum.member?(dir, 2) == :false ->
                  List.insert_at(dir,-1,2)
                 (Enum.member?(dir, 1) or (x == 1) or facing == :east) == :false ->
                  List.insert_at(dir,-1,1)
                 (Enum.member?(dir,3) or (x == 6)) == :false ->
                  List.insert_at(dir,-1,3)
                 (Enum.member?(dir,0)) or (y == 6) == :false ->
                  List.insert_at(dir,-1,0)
                 true ->
                  List.insert_at(dir,-1,4)
               end
           end

           goal_y == y ->
            cond do
             (goal_x > x) ->
               cond do
                 Enum.member?(dir, 3) == :false ->
                  List.insert_at(dir,-1,3)
                 (Enum.member?(dir,2) or (y == 1) or facing == :north) == :false ->
                  List.insert_at(dir,-1,2)
                 (Enum.member?(dir,0) or (y == 6)) == :false ->
                  List.insert_at(dir,-1,0)
                 (Enum.member?(dir, 1)) or (x == 1) == :false ->
                  List.insert_at(dir,-1,1)
                 true ->
                  List.insert_at(dir,-1,4)
               end

             true ->
               cond do
                 Enum.member?(dir, 1) == :false ->
                  List.insert_at(dir,-1,1)
                 (Enum.member?(dir, 2) or (y == 1) or facing == :north) == :false ->
                  List.insert_at(dir,-1,2)
                 (Enum.member?(dir,0) or (y == 6)) == :false ->
                  List.insert_at(dir,-1,0)
                 (Enum.member?(dir,3)) or (x == 6) == :false ->
                  List.insert_at(dir,-1,3)
                 true ->
                  List.insert_at(dir,-1,4)
               end
           end
    end
  end

  def min2(x,y,robot,ls,count,min,index) when count < 4 do
    mp3 = %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5, :f => 6}
    distance = abs(robot.x - Enum.at(Enum.at(ls,count),0)) + abs(Map.get(mp3, robot.y) -  Enum.at(Enum.at(ls,count),1))
    {index,min} = if(min >= distance and Enum.at(Enum.at(ls,count),0) != x and Enum.at(Enum.at(ls,count),1) != y) do
      {count, distance}
    else
      {index,min}
    end
    min(robot,ls,count+1,min,index)
  end

  def min2(_x,_y,_robot,ls,_count,_min,index) do
    Enum.fetch(ls,index)
  end

  @doc """
  rep function is responsible for the deciding the entire path of the robot. This functions uses depth-first-search approach
  to determine the next node the robot has to visit. This function check the presence of obstacle with every movement and changes
  the course of the traversal path if one is encountered. It calls multiple functions defined in this module like, decide_dir, dir_select, goX,
  goY, forGoal_x, forGoal_y
  """

  def rep(motor_ref,goal,dir,q,visited,robot,goal_x,goal_y,channel, len) when len != 0 do
    Task4CClientRobotA.PhoenixSocketClient.timer(channel)
    #getting next block
    {{:value, value3}, q} = :queue.out_r(q)
    {x,y, dirs} = value3
    new_goal_x = x
    new_goal_y = y

    #if reached destination
    len = if(x == goal_x and y == goal_y or (robot.x == goal_x and robot.y == goal_y)) do
      0
    end

    obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
    [bx,by,bfacing,b_alive] = Task4CClientRobotA.PhoenixSocketClient.get_bot_position(true,channel,robot)
    mp4 = %{1 => :a, 2 => :b, 3 => :c, 4 => :d, 5 => :e, 6 => :f}
    mp3 = %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5, :f => 6}


    {goal_x,goal_y} = if(bx == goal_x and by == goal_y and b_alive == false) do
      ls = nearby(goal)
      min2 = abs(robot.x - Enum.at(Enum.at(ls,0),0)) + abs(Map.get(mp3, robot.y) -  Enum.at(Enum.at(ls,0),1))
      {_,[g_x, g_y]} = min2(goal_x,Map.get(mp3,goal_y),robot,ls,1,min2,0)
      {g_x,Map.get(mp4,g_y)}
    else
      {goal_x,goal_y}
    end

    first = 0
    first = if(new_goal_x == bx and new_goal_y == by and (robot.x != new_goal_x or robot.y != new_goal_y)) do
      x = cond do
        b_alive == false ->
          1
        bfacing == :east and robot.facing == :west ->
          1
        bfacing == :west and robot.facing == :east ->
          1
        bfacing == :north and robot.facing == :south ->
          1
        bfacing == :south and robot.facing == :north ->
          1
        true -> first
      end
      x
    else
      0
    end
    {q,visited,robot,len,dir} = cond do
      new_goal_x == bx and new_goal_y == by and first == 0 ->
      _obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
      [_bx,_by,_bfacing,_b_alive] = Task4CClientRobotA.PhoenixSocketClient.get_bot_position(true,channel,robot)
      q = :queue.in({x,y,dirs},q)
      len = :queue.len(q)
      {q,visited,robot,len,dir}
    true ->
      {q,robot,dir,visited,obs,len,x,y} = if(first == 0) do
        #travelling to new goals
        {robot,obs} = Task4CClientRobotA.forGoal_x(motor_ref,obs,robot,new_goal_x,channel)
        {robot,obs} = Task4CClientRobotA.goX(robot,new_goal_x,new_goal_y,channel,obs,motor_ref)
        {robot,obs} = Task4CClientRobotA.forGoal_y(motor_ref,obs,robot,new_goal_y,channel)
        {robot,obs} = Task4CClientRobotA.goY(robot,new_goal_x,new_goal_y,channel,obs,motor_ref)

        [_bx,_by,_bfacing,_b_alive] = Task4CClientRobotA.PhoenixSocketClient.get_bot_position(true,channel,robot)

        #putting back with changed dir
        dir = List.last(dirs)
        new_dir = dir_select(robot.facing,x,y,goal_x,goal_y,dirs)
        q = :queue.in({x,y,new_dir},q)

        #setting robot's direction based on dir
        both = if(robot.x == goal_x and robot.y == goal_y) do
          {robot,obs}
        else
        both = cond do
          dir == 0 ->
            both = cond do
              robot.facing == :east ->
                robot = left(channel,robot,motor_ref)
                obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                {robot,obs}

              robot.facing == :south ->
                robot = left(channel,robot,motor_ref)
                Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                robot = left(channel,robot,motor_ref)
                obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                {robot,obs}

              robot.facing == :west ->
                robot = right(channel,robot,motor_ref)
                obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                {robot,obs}

              robot.facing == :north ->
                {robot,obs}

            end
            both
          dir == 1 ->
            both = cond do
              robot.facing == :north ->
                robot = left(channel,robot,motor_ref)
                obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                {robot,obs}

              robot.facing == :east ->
                robot = left(channel,robot,motor_ref)
                Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                robot = left(channel,robot,motor_ref)
                obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                {robot,obs}

              robot.facing == :south ->
                robot = right(channel,robot,motor_ref)
                obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                {robot,obs}

              robot.facing == :west ->
                {robot,obs}

            end
            both
          dir == 2 ->
            both = cond do
              robot.facing == :west ->
                robot = left(channel,robot,motor_ref)
                obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                {robot,obs}

              robot.facing == :north ->
                robot = left(channel,robot,motor_ref)
                Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                robot = left(channel,robot,motor_ref)
                obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                {robot,obs}

              robot.facing == :east ->
                robot = right(channel,robot,motor_ref)
                obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                {robot,obs}

              robot.facing == :south ->
                {robot,obs}

            end
            both
          dir == 3 ->
            both = cond do
              robot.facing == :south ->
                robot = left(channel,robot,motor_ref)
                obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                {robot,obs}

              robot.facing == :west ->
                robot = left(channel,robot,motor_ref)
                Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                robot = left(channel,robot,motor_ref)
                obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                {robot,obs}

              robot.facing == :north ->
                robot = right(channel,robot,motor_ref)
                obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
                {robot,obs}

              robot.facing == :east ->
                {robot,obs}

            end
            both
          dir > 3 ->
            {robot,obs}
        end
        both
        end
        {robot,obs} = both
        [_bx,_by,_bfacing,_b_alive] = Task4CClientRobotA.PhoenixSocketClient.get_bot_position(true,channel,robot)
        {q,robot,dir,visited,obs,len,x,y}
      else
        {visited,q} = if(:queue.len(visited) != 0) do
            {{:value, _val},visited} = :queue.out_r(visited)
            {visited,q}
        else
          {visited,q}
        end

        obs = cond do
          dir == 0 and robot.facing == :north -> true
          dir == 1 and robot.facing == :west -> true
          dir == 2 and robot.facing == :south -> true
          dir == 3 and robot.facing == :east -> true
          true -> false
        end
        len = if((robot.x == goal_x and robot.y == goal_y)) do
          0
        else
          :queue.len(q)
        end
        {q,robot,dir,visited,obs,len,robot.x,robot.y}
      end
        {q,visited} = cond do

          dir == 0 ->
            #up
            check = if(y == :f or obs == true) do
              false
            else
              !(:queue.member({x,plus(y)},visited))
            end
              struc4 = if check do
                  el = decide_dir(robot.facing,x,plus(y),goal_x,goal_y)
                  dirs = [el]
                  q = :queue.in({x,plus(y),dirs}, q)
                  visited = :queue.in({x,plus(y)},visited)
                  {q,visited}
                else
                  {q,visited}
              end
              struc4

          dir == 1 ->
            #left
            check = if(x == 1 or obs == true) do
              false
            else
              !(:queue.member({x-1,y},visited))
            end
              struc1 = if check do
                  el = decide_dir(robot.facing,x-1,y,goal_x,goal_y)
                  dirs = [el]
                  q = :queue.in({x-1,y,dirs}, q)
                  visited = :queue.in({x-1,y},visited)
                  {q,visited}
                else
                  {q,visited}
              end
              struc1

          dir == 2 ->
            #down
            check = if(y == :a or obs == true) do
              false
            else
              !(:queue.member({x,minus(y)},visited))
            end
            struc2 = if check do
                el = decide_dir(robot.facing,x,minus(y),goal_x,goal_y)
                dirs = [el]
                q = :queue.in({x,minus(y),dirs}, q)
                visited = :queue.in({x,minus(y)},visited)
                {q,visited}
              else
              {q,visited}
            end
            struc2

          dir == 3 ->
            #right
            check = if(x == 6 or obs == true) do
              false
            else
              !(:queue.member({x+1,y},visited))
            end
            struc3 = if check do
                el = decide_dir(robot.facing,x+1,y,goal_x,goal_y)
                dirs = [el]
                q = :queue.in({x+1,y,dirs}, q)
                visited = :queue.in({x+1,y},visited)
                {q,visited}
              else
                {q,visited}
            end
            struc3

          dir > 3 ->
            #backtracking
            {{:value, _val}, q} = :queue.out_r(q)
            {{:value, _val},visited} = :queue.out_r(visited)
            struc = {q,visited}
            struc
        end

        len = if(len == 0) do
          0
        else
          :queue.len(q)
        end
        {q,visited,robot,len,dir}
    end

    rep(motor_ref,goal,dir,q,visited,robot,goal_x,goal_y,channel, len)
  end

  def rep(motor_ref,_goal,_dir,_q,_visited,robot,_goal_x,_goal_y,_channel, _len) do
    {robot}
  end

@doc """
  forGoal_x function sets the facing or direction of the robot
"""

def forGoal_x(motor_ref,_obs,robot,goal_x,channel) when robot.x < goal_x and robot.facing != :east do
  obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
    [_obs,robot] = cond do
      robot.facing == :north ->
        robot = right(channel,robot,motor_ref)
        [obs,robot]
      robot.facing == :west ->
        robot = left(channel,robot,motor_ref)
        obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
        robot = left(channel,robot,motor_ref)
        [obs,robot]
      robot.facing == :south ->
        robot = left(channel,robot,motor_ref)
        [obs,robot]
  end
  obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
  {robot,obs}
end

def forGoal_x(motor_ref,_obs,robot,goal_x,channel) when robot.x > goal_x and robot.facing != :west do
obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
  [_obs,robot] = cond do
    robot.facing == :south ->
      robot = right(channel,robot,motor_ref)
      [obs,robot]
    robot.facing == :east ->
      robot = left(channel,robot,motor_ref)
      obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
      robot = left(channel,robot,motor_ref)
      [obs,robot]
    robot.facing == :north ->
      robot = left(channel,robot,motor_ref)
      [obs,robot]
  end
  obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
  {robot,obs}
end

def forGoal_x(motor_ref,obs,robot,_goal_x, _channel) do
  {robot,obs}
end

@doc """
  forGoal_y function sets the facing or direction of the robot
"""
def forGoal_y(motor_ref,_obs,robot,goal_y,channel) when robot.y < goal_y and robot.facing != :north do
obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
  [_obs,robot] = cond do
    robot.facing == :west ->
      robot = right(channel,robot,motor_ref)
      [obs,robot]
    robot.facing == :south ->
      robot = left(channel,robot,motor_ref)
      obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
      robot = left(channel,robot,motor_ref)
      [obs,robot]
    robot.facing == :east ->
      robot = left(channel,robot,motor_ref)
      [obs,robot]
  end
  obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)

{robot,obs}
end

def forGoal_y(motor_ref,_obs,robot,goal_y,channel) when robot.y > goal_y and robot.facing != :south do
  obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)

  [_obs,robot] = cond do
  robot.facing == :east ->
    robot = right(channel,robot,motor_ref)
    [obs,robot]
  robot.facing == :north ->
    robot = left(channel,robot,motor_ref)
    obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
    robot = left(channel,robot,motor_ref)
    [obs,robot]
  robot.facing == :west ->
    robot = left(channel,robot,motor_ref)
    [obs,robot]
  end
  obs = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)

  {robot,obs}
end

def forGoal_y(motor_ref,obs,robot,_goal_y, _channel) do
  {robot,obs}
end

@doc """
goX function moves the bot along x axis
"""
def goX(%Task4CClientRobotA.Position{facing: _facing,x: x, y: _y} = robot, goal_x, goal_y,channel, _ob, motor_ref) when x != goal_x do
  robot = move(channel,robot,motor_ref)
  ob = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
  goX(robot,goal_x,goal_y,channel, ob,motor_ref)
end

def goX(robot, _goal_x, _goal_y, _channel, ob, _motor_ref) do
  {robot,ob}
end

@doc """
goY function moves the bot along y axis
"""
def goY(%Task4CClientRobotA.Position{facing: _facing,x: _x, y: y} = robot, goal_x, goal_y, channel, _ob, motor_ref) when y != goal_y do
  robot = move(channel,robot,motor_ref)
  ob = Task4CClientRobotA.PhoenixSocketClient.send_robot_status(channel,robot)
  goY(robot,goal_x,goal_y,channel,ob,motor_ref)
end

def goY(robot, _goal_x, _goal_y, _channel,ob,_motor_ref) do
  {robot,ob}
end

  @doc """
  Provides the report of the robot's current position

  Examples:

      iex> {:ok, robot} = Task4CClientRobotA.place(2, :b, :west)
      iex> Task4CClientRobotA.report(robot)
      {2, :b, :west}
  """
  def report(%Task4CClientRobotA.Position{x: x, y: y, facing: facing} = _robot) do
    {x, y, facing}
  end

  @directions_to_the_right %{north: :east, east: :south, south: :west, west: :north}
  @doc """
  Rotates the robot to the right
  """
  def right(channel,%Task4CClientRobotA.Position{facing: facing} = robot,motor_ref) do
    Task4CClientRobotA.LineFollower.right(channel,motor_ref,0)
    %Task4CClientRobotA.Position{robot | facing: @directions_to_the_right[facing]}
  end

  @directions_to_the_left Enum.map(@directions_to_the_right, fn {from, to} -> {to, from} end)
  @doc """
  Rotates the robot to the left
  """
  def left(channel,%Task4CClientRobotA.Position{facing: facing} = robot,motor_ref) do
    Task4CClientRobotA.LineFollower.left(channel,motor_ref,0)
    %Task4CClientRobotA.Position{robot | facing: @directions_to_the_left[facing]}
  end

  @doc """
  Moves the robot to the north, but prevents it to fall
  """
  def move(channel,%Task4CClientRobotA.Position{x: _, y: y, facing: :north} = robot, motor_ref) when y < @table_top_y do
    maximum = 110
    # Task4CClientRobotA.LineFollower.forward(1,1,0,motor_ref,maximum,0,0)
    Task4CClientRobotA.LineFollower.pid(channel,motor_ref)
    %Task4CClientRobotA.Position{ robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0) }
  end

  @doc """
  Moves the robot to the east, but prevents it to fall
  """
  def move(channel,%Task4CClientRobotA.Position{x: x, y: _, facing: :east} = robot, motor_ref) when x < @table_top_x do
    maximum = 110
    # Task4CClientRobotA.LineFollower.forward(1,1,0,motor_ref,maximum,0,0)
    Task4CClientRobotA.LineFollower.pid(channel,motor_ref)
    %Task4CClientRobotA.Position{robot | x: x + 1}
  end

  @doc """
  Moves the robot to the south, but prevents it to fall
  """
  def move(channel,%Task4CClientRobotA.Position{x: _, y: y, facing: :south} = robot, motor_ref) when y > :a do
    maximum = 110
    # Task4CClientRobotA.LineFollower.forward(1,1,0,motor_ref,maximum,0,0)
    Task4CClientRobotA.LineFollower.pid(channel,motor_ref)
    %Task4CClientRobotA.Position{ robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the west, but prevents it to fall
  """
  def move(channel,%Task4CClientRobotA.Position{x: x, y: _, facing: :west} = robot, motor_ref) when x > 1 do
    maximum = 110
    # Task4CClientRobotA.LineFollower.forward(1,1,0,motor_ref,maximum,0,0)
    Task4CClientRobotA.LineFollower.pid(channel,motor_ref)
    %Task4CClientRobotA.Position{robot | x: x - 1}
  end

  @doc """
  Does not change the position of the robot.
  This function used as fallback if the robot cannot move outside the table
  """
  def move(channel,robot), do: robot

  def failure do
    raise "Connection has been lost"
  end
end
