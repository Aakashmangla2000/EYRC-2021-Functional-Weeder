defmodule Task4CClientRobotB do
  # max x-coordinate of table top
  @table_top_x 6
  # max y-coordinate of table top
  @table_top_y :f
  # mapping of y-coordinates
  @robot_map_y_atom_to_num %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5, :f => 6}

  @doc """
  Places the robot to the default position of (1, A, North)

  Examples:

      iex> Task4CClientRobotB.place
      {:ok, %Task4CClientRobotB.Position{facing: :north, x: 1, y: :a}}
  """
  def place do
    {:ok, %Task4CClientRobotB.Position{}}
  end

  def place(x, y, _facing) when x < 1 or y < :a or x > @table_top_x or y > @table_top_y do
    {:failure, "Invalid position"}
  end

  def place(_x, _y, facing) when facing not in [:north, :east, :south, :west] do
    {:failure, "Invalid facing direction"}
  end

  @doc """
  Places the robot to the provided position of (x, y, facing),
  but prevents it to be placed outside of the table and facing invalid direction.

  Examples:

      iex> Task4CClientRobotB.place(1, :b, :south)
      {:ok, %Task4CClientRobotB.Position{facing: :south, x: 1, y: :b}}

      iex> Task4CClientRobotB.place(-1, :f, :north)
      {:failure, "Invalid position"}

      iex> Task4CClientRobotB.place(3, :c, :north_east)
      {:failure, "Invalid facing direction"}
  """
  def place(x, y, facing) do
    {:ok, %Task4CClientRobotB.Position{x: x, y: y, facing: facing}}
  end

  @doc """
  Provide START position to the robot as given location of (x, y, facing) and place it.
  """
  def start(x, y, facing) do
    place(x, y, facing)
  end

  @doc """
  Main function to initiate the sequence of tasks to achieve by the Client Robot B,
  such as connect to the Phoenix server, get the robot B's start and goal locations to be traversed.
  Call the respective functions from this module and others as needed.
  You may create extra helper functions as needed.
  """
  def main do

    ###########################
    ## complete this funcion ##
    ###########################
    {:ok, _response, channel} = Task4CClientRobotB.PhoenixSocketClient.connect_server()
    start = repss(channel,0)
    {x,y,facing} = change_start(start)
    {:ok, robot} = start(x,y,facing)
    # Process.sleep(2000)
    [_ax,_ay,_afacing,goal_locs,_obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
    # Process.sleep(2000)
    # x = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
    # Process.sleep(2000)
    # x = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
    # x = 1
    # IO.puts("#{ax} #{ay} #{afacing} #{inspect(goal_locs)}")
    stop(robot, goal_locs,channel)
  end

  def repss(channel,start) do
    if(start == 0) do
      start = Task4CClientRobotB.PhoenixSocketClient.get_start(channel)
      repss(channel,start)
    else
      start
    end
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
    IO.puts("#{x} #{inspect(y)} #{inspect(facing)}")
    {x,y,facing}
  end

  @doc """
  Provide GOAL positions to the robot as given location of [(x1, y1),(x2, y2),..] and plan the path from START to these locations.
  Make a call to ToyRobot.PhoenixSocketClient.send_robot_status/2 to get the indication of obstacle presence ahead of the robot.
  """
  def stop(robot, goal_locs, channel) do

    ###########################
    ## complete this funcion ##
    ###########################
    parent = self()
    require Integer
    mp = %{"1" => 1, "2" => 2, "3" => 3, "4" => 4, "5" => 5, "6" => 6}
    mp2 = %{"a" => :a, "b" => :b, "c" => :c, "d" => :d, "e" => :e, "f" => :f}
    first = 0

    [_ax,_ay,_afacing,_goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
    count = Enum.count(goal_locs)
    goal_div(obs, robot, parent, goal_locs, channel,count,mp,mp2,first)

    {:ok, robot}

  end

   def rec_value() do
    receive do
      {:flag_value, flag} -> flag
    end
  end


  def goal_select(robot, parent, goal_locs, channel,min, index, count,mp,mp2) when count > 0 do
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
    goal_select(robot, parent, goal_locs, channel, min,index, count,mp,mp2)
  end

  def goal_select(_robot, _parent, _goal_locs, _channel,_min, index, _count,_mp,_mp2) do
    index
  end

  def nearby(goal) do
    # mp = %{1 => :a, 2 => :b, 3 => :c, 4 => :d, 5 => :e, 6 => :f}
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
    # IO.puts("#{x} #{y}")
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

  def goal_div(obs, robot, _parent, goal_locs, channel,count,mp,mp2,first) when count > 0 do
    mp3 = %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5, :f => 6}
    parent = self()
    pid = spawn_link(fn ->
      count = Enum.count(goal_locs)
      # IO.puts(count)
      {robot,obs,goal_locs,_count} = if(count == 0) do
        {robot,obs,goal_locs,count}
      else
        [ax,ay,afacing,_goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)

        # IO.puts("b #{inspect(goal_locs)}")
        all = {ax,ay,afacing}
        count = Enum.count(goal_locs)
        index = 0
        {goal_locs,goal_x, goal_y} = if(count > 0) do
          {val,index} = if (count > 1) do
            val = Enum.at(goal_locs,0)
            ls = nearby(val)
            min = abs(robot.x - Enum.at(Enum.at(ls,0),0)) + abs(Map.get(mp3, robot.y) -  Enum.at(Enum.at(ls,0),1))
            {_,[goal_x, goal_y]} = min(robot,ls,1,min,0)
            min = abs(robot.x - goal_x) + abs(Map.get(mp3, robot.y) -  goal_y)
            index = 0
            index = goal_select(robot, parent, goal_locs, channel,min, index, count-1,mp,mp2)
            {Enum.at(goal_locs,index),index}
          else
            {Enum.at(goal_locs,0),index}
          end
            {_,goal_locs} = List.pop_at(goal_locs,index)
            mp4 = %{1 => :a, 2 => :b, 3 => :c, 4 => :d, 5 => :e, 6 => :f}
            ls = nearby(val)
            min = abs(robot.x - Enum.at(Enum.at(ls,0),0)) + abs(Map.get(mp3, robot.y) -  Enum.at(Enum.at(ls,0),1))
            {_,[goal_x, goal_y]} = min(robot,ls,1,min,0)
          {goal_locs,goal_x, Map.get(mp4,goal_y)}
        else
          {goal_locs,robot.x, robot.y}
        end
        # IO.inspect(goal_locs)
        count = Enum.count(goal_locs)
        {robot,obs,_goal_locs} = get_value(all,goal_locs,obs, robot,goal_x, goal_y,channel, parent,first)
        {robot,obs,goal_locs,count}
      end
      # IO.puts("b #{inspect(goal_locs)}")
      count = Enum.count(goal_locs)
      send(parent, {:flag_value, {robot,obs,goal_locs,count}})
    end)
    Process.register(pid, :client_toyrobotB)
    first = 1
    {robot,obs,goal_locs,count} = rec_value()
    goal_div(obs, robot, parent, goal_locs, channel,count,mp,mp2,first)
  end

  def goal_div(_obs, robot, _parent, _goal_locs, _channel, _count,_mp,_mp2,_first) do
   robot
  end

  def get_value(all,goal_locs,obs, robot,goal_x, goal_y,channel, _parent,_first) do
    IO.puts("#{goal_x} #{goal_y}")
      {ax,ay,afacing} = all
      len = 1
      q = :queue.new()
      visited = :queue.new()
      el = decide_dir(robot.facing,robot.x,robot.y,goal_x,goal_y)
      dir = [el]
      q = :queue.in({robot.x,robot.y,dir},q)
      visited = :queue.in({robot.x,robot.y},visited)

      {robot,obs,goal_locs} = if(robot.x == goal_x and robot.y == goal_y) do
        {robot,obs,goal_locs}
      else
      Task4CClientRobotB.rep(goal_locs, obs,ax,ay,afacing,q,visited,robot,goal_x,goal_y,channel,len)
      end
      {robot,obs,goal_locs}
  end

  def plus(y) do
    Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)
  end

  def minus(y) do
    Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)
  end

  def decide_dir(facing, x,y,goal_x,goal_y) do
    # IO.puts("#{x} #{y}  #{facing}")
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
    # IO.puts(dir)
    dir
  end

  def dir_select(facing,x, y, goal_x, goal_y, dir) do
    mp2 = %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5, :f => 6}
    goal_y = Map.get(mp2, goal_y)
    y = Map.get(mp2, y)
    # IO.puts("list is #{inspect(dir)}")

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

  def rep(goal_locs, obs,ax,ay,afacing, q,visited,robot,goal_x,goal_y,channel, len) when len != 0 do
    #getting next block
    {{:value, value3}, q} = :queue.out_r(q)
    {x,y, dirs} = value3
    new_goal_x = x
    new_goal_y = y

    #if reached destination
    len = if(x == goal_x and y == goal_y) do
      # IO.puts("B Reached #{x} #{y}")
      0
    end

    {q,visited,robot,len,ax,ay,afacing, goal_locs,obs} = cond do
      new_goal_x == ax and new_goal_y == ay->
      # IO.puts("B crash into A")
      [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)

      q = :queue.in({x,y,dirs},q)
      len = :queue.len(q)
      {q,visited,robot,len,ax,ay,afacing, goal_locs,obs}
      true ->
      #travelling to new goals
        {robot,ax,ay,afacing, goal_locs} = Task4CClientRobotB.forGoal_x(ax,ay,afacing, goal_locs,robot,new_goal_x, channel)
        {robot,obs,ax,ay,afacing, goal_locs} = Task4CClientRobotB.goX(ax,ay,afacing, goal_locs,robot,new_goal_x,new_goal_y,channel,obs)
        {robot,ax,ay,afacing, goal_locs} = Task4CClientRobotB.forGoal_y(ax,ay,afacing, goal_locs,robot,new_goal_y, channel)
        {robot,obs,ax,ay,afacing, goal_locs} = Task4CClientRobotB.goY(ax,ay,afacing, goal_locs,robot,new_goal_x,new_goal_y,channel,obs)

        #putting back with changed dir
        dir = List.last(dirs)
        new_dir = dir_select(robot.facing, x,y,goal_x,goal_y,dirs)
        q = :queue.in({x,y,new_dir},q)
        #setting robot's direction based on dir
        both = if(robot.x == goal_x and robot.y == goal_y) do
          {robot, obs, ax, ay, afacing, goal_locs}
        else

        both = cond do
          dir == 0 ->
            both = cond do
              robot.facing == :east ->
                robot = left(robot)
                [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)


                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :south ->

                robot = left(robot)
                [_ax,_ay,_afacing,_goal_locs,_obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
                robot = left(robot)
                [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)


                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :west ->

                robot = right(robot)
                [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)


                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :north ->
                {robot, obs, ax, ay, afacing, goal_locs}
            end
            both
          dir == 1 ->
            both = cond do
              robot.facing == :north ->

                robot = left(robot)
                [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)



                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :east ->

                robot = left(robot)
                [_ax,_ay,_afacing,_goal_locs,_obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)



                robot = left(robot)
                [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)


                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :south ->

                robot = right(robot)
                [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)


                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :west ->
                {robot, obs, ax, ay, afacing, goal_locs}
            end
            both
          dir == 2 ->
            both = cond do
              robot.facing == :west ->

                robot = left(robot)
                [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)


                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :north ->

                robot = left(robot)
                [_ax,_ay,_afacing,_goal_locs,_obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)



                robot = left(robot)
                [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)


                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :east ->

                robot = right(robot)
                [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)


                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :south ->
                {robot, obs, ax, ay, afacing, goal_locs}
            end
            both
          dir == 3 ->
            both = cond do
              robot.facing == :south ->

                robot = left(robot)
                [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)


                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :west ->

                robot = left(robot)
                [_ax,_ay,_afacing,_goal_locs,_obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)



                robot = left(robot)
                [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)


                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :north ->

                robot = right(robot)
                [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)


                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :east ->
                {robot, obs, ax, ay, afacing, goal_locs}
            end
            both
          dir > 3 ->
            {robot, obs, ax, ay, afacing, goal_locs}
        end

        both
        end
        {robot, obs, ax, ay, afacing, goal_locs} = both
        # struc = {q,visited}

        {q, visited} = cond do

          dir == 0 ->
            #up
            check = if(y == :e or obs == true) do
              false
            else
              !(:queue.member({x,plus(y)}, visited))
            end
              struc4 = if check do
                  el = decide_dir(robot.facing, x,plus(y),goal_x,goal_y)
                  dirs = [el]
                  q = :queue.in({x,plus(y),dirs}, q)
                  visited = :queue.in({x,plus(y)}, visited)
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
              !(:queue.member({x-1,y}, visited))
            end
              struc1 = if check do
                  el = decide_dir(robot.facing, x-1,y,goal_x,goal_y)
                  dirs = [el]
                  q = :queue.in({x-1,y,dirs}, q)
                  visited = :queue.in({x-1,y}, visited)
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
              !(:queue.member({x,minus(y)}, visited))
            end
            struc2 = if check do
                el = decide_dir(robot.facing, x,minus(y),goal_x,goal_y)
                dirs = [el]
                q = :queue.in({x,minus(y),dirs}, q)
                visited = :queue.in({x,minus(y)}, visited)
                {q,visited}
              else
              {q,visited}
            end
            struc2

          dir == 3 ->
            #right
            check = if(x == 5 or obs == true) do
              false
            else
              !(:queue.member({x+1,y}, visited))
            end
            struc3 = if check do
                el = decide_dir(robot.facing, x+1,y,goal_x,goal_y)
                dirs = [el]
                q = :queue.in({x+1,y,dirs}, q)
                visited = :queue.in({x+1,y}, visited)
                {q,visited}
              else
                {q,visited}
            end
            struc3

          dir > 3 ->
            #backtracking
            {{:value, _val}, q} = :queue.out_r(q)
            {{:value, _val}, visited} = :queue.out_r(visited)
            struc = {q,visited}
            struc
        end

        len = if(len == 0) do
          0
        else
          :queue.len(q)
        end
    {q,visited,robot,len,ax,ay,afacing, goal_locs,obs}
    end

    rep(goal_locs, obs,ax,ay,afacing, q,visited,robot,goal_x,goal_y,channel, len)
  end

  def rep(goal_locs, obs,_ax,_ay,_afacing, _q,_visited,robot,_goal_x,_goal_y,_channel, _len) do
    {robot,obs,goal_locs}
  end

  def forGoal_x(_ax,_ay,_afacing, _goal_locs,robot,goal_x, channel) when robot.x < goal_x and robot.facing != :east do
    [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
     [ax,ay,afacing,goal_locs,_obs,robot] = cond do
      robot.facing == :north ->
        robot = right(robot)
        [ax,ay,afacing,goal_locs,obs,robot]
      robot.facing == :west ->
        robot = left(robot)
        [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
        robot = left(robot)
        [ax,ay,afacing,goal_locs,obs,robot]
      robot.facing == :south ->
        robot = left(robot)
        [ax,ay,afacing,goal_locs,obs,robot]
    end
    # [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)

  {robot,ax,ay,afacing, goal_locs}
end

def forGoal_x(_ax,_ay,_afacing, _goal_locs,robot,goal_x,channel) when robot.x > goal_x and robot.facing != :west do
[ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
  [ax,ay,afacing,goal_locs,_obs,robot] = cond do
    robot.facing == :south ->
      robot = right(robot)
      [ax,ay,afacing,goal_locs,obs,robot]
    robot.facing == :east ->
      robot = left(robot)
      [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
      robot = left(robot)
      [ax,ay,afacing,goal_locs,obs,robot]
    robot.facing == :north ->
      robot = left(robot)
      [ax,ay,afacing,goal_locs,obs,robot]
  end
  # [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)

{robot,ax,ay,afacing, goal_locs}
end

def forGoal_x(ax,ay,afacing, goal_locs,robot, _goal_x, _channel) do
  {robot,ax,ay,afacing, goal_locs}
end

def forGoal_y(_ax,_ay,_afacing, _goal_locs,robot,goal_y, channel) when robot.y < goal_y and robot.facing != :north do
[ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
  [ax,ay,afacing,goal_locs,_obs,robot] = cond do
    robot.facing == :west ->
      robot = right(robot)
      [ax,ay,afacing,goal_locs,obs,robot]
    robot.facing == :south ->
      robot = left(robot)
      [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
      robot = left(robot)
      [ax,ay,afacing,goal_locs,obs,robot]
    robot.facing == :east ->
      robot = left(robot)
      [ax,ay,afacing,goal_locs,obs,robot]
  end
  # [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)

{robot,ax,ay,afacing,goal_locs}
end

def forGoal_y(_ax,_ay,_afacing, _goal_locs,robot,goal_y, channel) when robot.y > goal_y and robot.facing != :south do
[ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
  [ax,ay,afacing,goal_locs,_obs,robot] = cond do
  robot.facing == :east ->
    robot = right(robot)
    [ax,ay,afacing,goal_locs,obs,robot]
  robot.facing == :north ->
    robot = left(robot)
    [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
    robot = left(robot)
    [ax,ay,afacing,goal_locs,obs,robot]
  robot.facing == :west ->
    robot = left(robot)
    [ax,ay,afacing,goal_locs,obs,robot]
end
# [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)

{robot,ax,ay,afacing,goal_locs}
end

def forGoal_y(ax,ay,afacing, goal_locs,robot, _goal_y, _channel) do
{robot,ax,ay,afacing,goal_locs}
end


def goX(_ax,_ay,_afacing, _goal_locs,%Task4CClientRobotB.Position{facing: _facing, x: x, y: _y} = robot, goal_x, goal_y, channel,_ob) when x != goal_x do
  robot = move(robot)
  [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
  goX(ax,ay,afacing,goal_locs,robot,goal_x,goal_y,channel,obs)
end

def goX(ax,ay,afacing, goal_locs,robot, _goal_x, _goal_y, _channel,ob) do
  {robot,ob,ax,ay,afacing,goal_locs}
end

def goY(_ax,_ay,_afacing, _goal_locs,%Task4CClientRobotB.Position{facing: _facing, x: _x, y: y} = robot, goal_x, goal_y, channel,_ob) when y != goal_y do
  robot = move(robot)
  [ax,ay,afacing,goal_locs,obs] = Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
  goY(ax,ay,afacing,goal_locs,robot,goal_x,goal_y,channel,obs)
end

def goY(ax,ay,afacing, goal_locs,robot, _goal_x, _goal_y, _channel,ob) do
  {robot,ob,ax,ay,afacing,goal_locs}
end


  @doc """
  Provides the report of the robot's current position

  Examples:

      iex> {:ok, robot} = Task4CClientRobotB.place(2, :b, :west)
      iex> Task4CClientRobotB.report(robot)
      {2, :b, :west}
  """
  def report(%Task4CClientRobotB.Position{x: x, y: y, facing: facing} = _robot) do
    {x, y, facing}
  end

  @directions_to_the_right %{north: :east, east: :south, south: :west, west: :north}
  @doc """
  Rotates the robot to the right
  """
  def right(%Task4CClientRobotB.Position{facing: facing} = robot) do
    %Task4CClientRobotB.Position{robot | facing: @directions_to_the_right[facing]}
  end

  @directions_to_the_left Enum.map(@directions_to_the_right, fn {from, to} -> {to, from} end)
  @doc """
  Rotates the robot to the left
  """
  def left(%Task4CClientRobotB.Position{facing: facing} = robot) do
    %Task4CClientRobotB.Position{robot | facing: @directions_to_the_left[facing]}
  end

  @doc """
  Moves the robot to the north, but prevents it to fall
  """
  def move(%Task4CClientRobotB.Position{x: _, y: y, facing: :north} = robot) when y < @table_top_y do
    %Task4CClientRobotB.Position{ robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)
    }
  end

  @doc """
  Moves the robot to the east, but prevents it to fall
  """
  def move(%Task4CClientRobotB.Position{x: x, y: _, facing: :east} = robot) when x < @table_top_x do
    %Task4CClientRobotB.Position{robot | x: x + 1}
  end

  @doc """
  Moves the robot to the south, but prevents it to fall
  """
  def move(%Task4CClientRobotB.Position{x: _, y: y, facing: :south} = robot) when y > :a do
    %Task4CClientRobotB.Position{ robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the west, but prevents it to fall
  """
  def move(%Task4CClientRobotB.Position{x: x, y: _, facing: :west} = robot) when x > 1 do
    %Task4CClientRobotB.Position{robot | x: x - 1}
  end

  @doc """
  Does not change the position of the robot.
  This function used as fallback if the robot cannot move outside the table
  """
  def move(robot), do: robot

  def failure do
    raise "Connection has been lost"
  end
end
