defmodule CLI.ToyRobotB do
  # max x-coordinate of table top
  @table_top_x 5
  # max y-coordinate of table top
  @table_top_y :e
  # mapping of y-coordinates
  @robot_map_y_atom_to_num %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5}

  @doc """
  Places the robot to the default position of (1, A, North)

  Examples:

      iex> CLI.ToyRobotB.place
      {:ok, %CLI.Position{facing: :north, x: 1, y: :a}}
  """
  def place do
    {:ok, %CLI.Position{}}
  end

  def place(x, y, _facing) when x < 1 or y < :a or x > @table_top_x or y > @table_top_y do
    {:failure, "Invalid position"}
  end

  def place(_x, _y, facing)
  when facing not in [:north, :east, :south, :west]
  do
    {:failure, "Invalid facing direction"}
  end

  @doc """
  Places the robot to the provided position of (x, y, facing),
  but prevents it to be placed outside of the table and facing invalid direction.

  Examples:

      iex> CLI.ToyRobotB.place(1, :b, :south)
      {:ok, %CLI.Position{facing: :south, x: 1, y: :b}}

      iex> CLI.ToyRobotB.place(-1, :f, :north)
      {:failure, "Invalid position"}

      iex> CLI.ToyRobotB.place(3, :c, :north_east)
      {:failure, "Invalid facing direction"}
  """
  def place(x, y, facing) do
    # IO.puts String.upcase("B I'm placed at => #{x},#{y},#{facing}")
    {:ok, %CLI.Position{x: x, y: y, facing: facing}}
  end

  @doc """
  Provide START position to the robot as given location of (x, y, facing) and place it.
  """
  def start(x, y, facing) do
    ###########################
    ## complete this funcion ##
    ###########################
    CLI.ToyRobotB.place(x, y, facing)
  end

  def stop(_robot, goal_x, goal_y, _cli_proc_name) when goal_x < 1 or goal_y < :a or goal_x > @table_top_x or goal_y > @table_top_y do
    {:failure, "Invalid STOP position"}
  end

  @doc """
  Provide GOAL positions to the robot as given location of [(x1, y1),(x2, y2),..] and plan the path from START to these locations.
  Passing the CLI Server process name that will be used to send robot's current status after each action is taken.
  Spawn a process and register it with name ':client_toyrobotB' which is used by CLI Server to send an
  indication for the presence of obstacle ahead of robot's current position and facing.
  """
  def stop(robot, goal_locs, cli_proc_name) do
    ###########################
    ## complete this funcion ##
    ###########################
    parent = self()
    require Integer
    mp = %{"1" => 1, "2" => 2, "3" => 3, "4" => 4, "5" => 5}
    mp2 = %{"a" => :a, "b" => :b, "c" => :c, "d" => :d, "e" => :e}
    first = 0
    obs = false

    robot = if Enum.count(goal_locs) == 1 do
      robot
    else
      count = Enum.count(goal_locs)
      goal_div(obs, robot, parent, goal_locs, cli_proc_name,count,mp,mp2,first)
    end

    {:ok, robot}
  end

  def goal_select(robot, parent, goal_locs, cli_proc_name,min, index, count,mp,mp2) when count > 0 do
    mp3 = %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5}
    val = Enum.at(goal_locs,count)
    # IO.puts("andar #{count} #{inspect(val)}")
  # IO.puts(inspect(goal_locs))
    goal_x = Enum.at(val,0)
    goal_x = Map.get(mp, goal_x)
    goal_y = Enum.at(val,1)
    goal_y = Map.get(mp2, goal_y)
    goal_y = Map.get(mp3, goal_y)
    distance = abs(robot.x - goal_x) + abs(Map.get(mp3, robot.y) -  goal_y)

    {index,min} = if(min >= distance) do
      {count, distance}
    else
      {index,min}
    end
    count = count - 1
    goal_select(robot, parent, goal_locs, cli_proc_name, min,index, count,mp,mp2)
  end

  def goal_select(_robot, _parent, _goal_locs, _cli_proc_name,_min, index, _count,_mp,_mp2) do
    index
  end

  def goal_div(obs, robot, _parent, goal_locs, cli_proc_name,count,mp,mp2,first) when count > 0 do
    mp3 = %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5}
    parent = self()
    pid = spawn_link(fn ->
      count = Enum.count(goal_locs)
      # IO.puts(count)
      {robot,obs,goal_locs,count} = if(count == 0) do
        {robot,obs,goal_locs,count}
      else
        if (Process.whereis(:cli_robotB_state) == nil and first == 0) do
          %CLI.Position{x: px, y: py, facing: pfacing} = robot
          pid = spawn_link(fn -> listen_from_cli(px,py,pfacing,goal_locs) end)
          Process.register(pid, :cli_robotB_state)
        end

        {ax,ay,afacing,goal_locs} = receiving_coor(goal_locs)
        obs = send_robot_status(robot, cli_proc_name)

        # IO.puts("b #{inspect(goal_locs)}")
        all = {ax,ay,afacing}
        count = Enum.count(goal_locs)

        {goal_locs,goal_x, goal_y} = if(count > 0) do
          {:ok,val} = if (count > 1) do
            val = Enum.at(goal_locs,0)
            goal_x = Enum.at(val,0)
            goal_x = Map.get(mp, goal_x)
            goal_y = Enum.at(val,1)
            goal_y = Map.get(mp2, goal_y)
            goal_y = Map.get(mp3, goal_y)

            min = abs(robot.x - goal_x) + abs(Map.get(mp3, robot.y) -  goal_y)
            index = 0
            index = goal_select(robot, parent, goal_locs, cli_proc_name,min, index, count-1,mp,mp2)
            Enum.fetch(goal_locs,index)
          else
            {:ok,Enum.at(goal_locs,0)}
          end

          goal_x = Enum.at(val,0)
          goal_x = Map.get(mp, goal_x)
          goal_y = Enum.at(val,1)
          goal_y = Map.get(mp2, goal_y)

          goal_locs = Enum.filter(goal_locs, fn val ->
            g_x = Enum.at(val,0)
            g_x = Map.get(mp, g_x)
            g_y = Enum.at(val,1)
            g_y = Map.get(mp2, g_y)
            g_x != goal_x or g_y != goal_y
          end)
          {goal_locs,goal_x, goal_y}
        else
          {goal_locs,robot.x, robot.y}
        end

        sending_coor(goal_locs, robot)
        count = Enum.count(goal_locs)
        {robot,obs,goal_locs} = get_value(all,goal_locs,obs, robot,goal_x, goal_y,cli_proc_name, parent,first)
        {robot,obs,goal_locs,count}
      end
      # IO.puts("b #{inspect(goal_locs)}")
      count = Enum.count(goal_locs)
      send(parent, {:flag_value, {robot,obs,goal_locs,count}})
    end)
    Process.register(pid, :client_toyrobotB)
    first = 1
    {robot,obs,goal_locs,count} = rec_value()
    goal_div(obs, robot, parent, goal_locs, cli_proc_name,count,mp,mp2,first)
  end

  def goal_div(_obs, robot, _parent, _goal_locs, _cli_proc_name, _count,_mp,_mp2,_first) do
   robot
  end

  def get_value(all,goal_locs,obs, robot,goal_x, goal_y,cli_proc_name, _parent,_first) do
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
      CLI.ToyRobotB.rep(goal_locs, obs,ax,ay,afacing,q,visited,robot,goal_x,goal_y,cli_proc_name,len)
      end
      {robot,obs,goal_locs}
  end

  def rec_value() do
    receive do
      {:flag_value, flag} -> flag
    end
  end

  def plus(y) do
    Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)
  end

  def minus(y) do
    Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)
  end

  def listen_from_cli(px,py,pfacing,goal_locs) do
    receive do
      {:toyrobotA} ->
        send(:get_botB, {:positions, {px,py,pfacing,goal_locs}})
      end
  end

    def send_robot_stat() do
      send(:cli_robotA_state, {:toyrobotB})
      rec_botA()
    end

    def rec_botA() do
      receive do
        {:positions, pos} -> pos
      end
    end

    def wait_till_over() do
      if (Process.whereis(:cli_robotB_state) != nil) do
        # IO.puts("waiting4")
        Process.sleep(100)
        wait_till_over()
      end
    end

    def wait_until_received() do
      if (Process.whereis(:cli_robotA_state) != nil and Process.whereis(:get_botB) == nil) do
        else
          # IO.puts("waiting3")
        Process.sleep(100)
        wait_until_received()
      end
    end

  def receiving_coor(goal_locs) do

    parent = self()

    if(Process.whereis(:client_toyrobotA) != nil) do
    wait_until_received()
    pid2 = spawn_link(fn ->
      coor = send_robot_stat()
      # {x,y,facing} = coor
      # IO.puts("Robot A: #{x} #{y} #{facing}")
      send(parent, {coor})
    end)
    Process.register(pid2, :get_botA)
  end
  if (Process.whereis(:client_toyrobotA) != nil) do
    receive do
      {coor} -> coor
    end
  else
    {0,0,0,goal_locs}
  end
  end

  def sending_coor(goal_locs, robot) do

    if(Process.whereis(:client_toyrobotA) != nil) do
    wait_till_over()
    # IO.puts("B sent")
    %CLI.Position{x: px, y: py, facing: pfacing} = robot
    pid = spawn_link(fn -> listen_from_cli(px,py,pfacing,goal_locs) end)
    Process.register(pid, :cli_robotB_state)
    end
  end

  def decide_dir(facing, x,y,goal_x,goal_y) do
    # IO.puts("#{x} #{y}  #{facing}")
    mp2 = %{:a => 0, :b => 1, :c => 2, :d => 3, :e => 4}
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
    mp2 = %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5}
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
                 (Enum.member?(dir,3) or (x == 5)) == :false ->
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
                 (Enum.member?(dir,3) or (x == 5)) == :false ->
                  List.insert_at(dir,-1,3)
                 (Enum.member?(dir,0)) or (y == 5) == :false ->
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
                 (Enum.member?(dir,0) or (y == 5)) == :false ->
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
                 (Enum.member?(dir,0) or (y == 5)) == :false ->
                  List.insert_at(dir,-1,0)
                 (Enum.member?(dir,3)) or (x == 5) == :false ->
                  List.insert_at(dir,-1,3)
                 true ->
                  List.insert_at(dir,-1,4)
               end
           end
    end
  end

  def rep(goal_locs, obs,ax,ay,afacing, q,visited,robot,goal_x,goal_y,cli_proc_name, len) when len != 0 do
    #getting next block
    {{:value, value3}, q} = :queue.out_r(q)
    {x,y, dirs} = value3
    new_goal_x = x
    new_goal_y = y

    #if reached destination
    len = if(x == goal_x and y == goal_y) do
      IO.puts("B Reached #{x} #{y}")
      0
    end

    {q,visited,robot,len,ax,ay,afacing, goal_locs,obs} = cond do
      new_goal_x == ax and new_goal_y == ay->
      # IO.puts("B crash into A")
      {ax,ay,afacing, goal_locs} = receiving_coor(goal_locs)
      obs = send_robot_status(robot,cli_proc_name)
      sending_coor(goal_locs, robot)
      q = :queue.in({x,y,dirs},q)
      len = :queue.len(q)
      {q,visited,robot,len,ax,ay,afacing, goal_locs,obs}
      true ->
      #travelling to new goals
        {robot,ax,ay,afacing, goal_locs} = CLI.ToyRobotB.forGoal_x(ax,ay,afacing, goal_locs,robot,new_goal_x, cli_proc_name)
        {robot,obs,ax,ay,afacing, goal_locs} = CLI.ToyRobotB.goX(ax,ay,afacing, goal_locs,robot,new_goal_x,new_goal_y,cli_proc_name,obs)
        {robot,ax,ay,afacing, goal_locs} = CLI.ToyRobotB.forGoal_y(ax,ay,afacing, goal_locs,robot,new_goal_y, cli_proc_name)
        {robot,obs,ax,ay,afacing, goal_locs} = CLI.ToyRobotB.goY(ax,ay,afacing, goal_locs,robot,new_goal_x,new_goal_y,cli_proc_name,obs)

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
                {ax,ay,afacing, goal_locs } = receiving_coor(goal_locs)
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :south ->
                {_ax,_ay,_afacing, goal_locs} = receiving_coor(goal_locs)
                robot = left(robot)
                send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {ax,ay,afacing, goal_locs } = receiving_coor(goal_locs)
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :west ->
                {ax,ay,afacing, goal_locs } = receiving_coor(goal_locs)
                robot = right(robot)
                obs = send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :north ->
                # IO.puts("b khali")
                {robot, obs, ax, ay, afacing, goal_locs}
            end
            both
          dir == 1 ->
            both = cond do
              robot.facing == :north ->
                {ax,ay,afacing, goal_locs } = receiving_coor(goal_locs)
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)


                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :east ->
                {_ax,_ay,_afacing, goal_locs} = receiving_coor(goal_locs)
                robot = left(robot)
                send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {ax,ay,afacing, goal_locs } = receiving_coor(goal_locs)
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :south ->
                {ax,ay,afacing, goal_locs } = receiving_coor(goal_locs)
                robot = right(robot)
                obs = send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :west ->
                # IO.puts("b khali")
                {robot, obs, ax, ay, afacing, goal_locs}
            end
            both
          dir == 2 ->
            both = cond do
              robot.facing == :west ->
                {ax,ay,afacing, goal_locs } = receiving_coor(goal_locs)
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :north ->
                {_ax,_ay,_afacing, goal_locs} = receiving_coor(goal_locs)
                robot = left(robot)
                send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {ax,ay,afacing, goal_locs } = receiving_coor(goal_locs)
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :east ->
                {ax,ay,afacing, goal_locs} = receiving_coor(goal_locs)
                robot = right(robot)
                obs = send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :south ->
                # IO.puts("b khali")
                {robot, obs, ax, ay, afacing, goal_locs}
            end
            both
          dir == 3 ->
            both = cond do
              robot.facing == :south ->
                {ax,ay,afacing, goal_locs } = receiving_coor(goal_locs)
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :west ->
                {_ax,_ay,_afacing, goal_locs} = receiving_coor(goal_locs)
                robot = left(robot)
                send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {ax,ay,afacing, goal_locs } = receiving_coor(goal_locs)
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :north ->
                {ax,ay,afacing, goal_locs } = receiving_coor(goal_locs)
                robot = right(robot)
                obs = send_robot_status(robot,cli_proc_name)
                sending_coor(goal_locs, robot)

                {robot, obs, ax, ay, afacing, goal_locs}
              robot.facing == :east ->
                # IO.puts("b khali")
                {robot, obs, ax, ay, afacing, goal_locs}
            end
            both
          dir > 3 ->
            {robot, obs, ax, ay, afacing, goal_locs}
        end

        # sending_coor(goal_locs, robot)
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

    #sends coordinates to A
    # sending_coor(goal_locs, robot)
    rep(goal_locs, obs,ax,ay,afacing, q,visited,robot,goal_x,goal_y,cli_proc_name, len)
  end

  def rep(goal_locs, obs,_ax,_ay,_afacing, _q,_visited,robot,_goal_x,_goal_y,_cli_proc_name, _len) do
    {robot,obs,goal_locs}
  end

  def forGoal_x(_ax,_ay,_afacing, goal_locs,robot,goal_x, cli_proc_name) when robot.x < goal_x and robot.facing != :east do

    {ax,ay,afacing, goal_locs} = receiving_coor(goal_locs)
     {ax,ay,afacing, goal_locs,robot} = cond do
      robot.facing == :north ->
        robot = right(robot)
        {ax,ay,afacing, goal_locs,robot}
      robot.facing == :west ->
        robot = left(robot)
        send_robot_status(robot,cli_proc_name)
        sending_coor(goal_locs, robot)
        {ax,ay,afacing, goal_locs} = receiving_coor(goal_locs)
        robot = left(robot)
        {ax,ay,afacing, goal_locs,robot}
      robot.facing == :south ->
        robot = left(robot)
        {ax,ay,afacing, goal_locs,robot}
    end
    send_robot_status(robot,cli_proc_name)
    sending_coor(goal_locs, robot)
  {robot,ax,ay,afacing, goal_locs}
end

def forGoal_x(_ax,_ay,_afacing, goal_locs,robot,goal_x,cli_proc_name) when robot.x > goal_x and robot.facing != :west do
{ax,ay,afacing, goal_locs} = receiving_coor(goal_locs)
  {ax,ay,afacing, goal_locs,robot} = cond do
    robot.facing == :south ->
      robot = right(robot)
      {ax,ay,afacing, goal_locs,robot}
    robot.facing == :east ->
      robot = left(robot)
      send_robot_status(robot,cli_proc_name)
      sending_coor(goal_locs, robot)
      {ax,ay,afacing, goal_locs} = receiving_coor(goal_locs)
      robot = left(robot)
      {ax,ay,afacing, goal_locs,robot}
    robot.facing == :north ->
      robot = left(robot)
      {ax,ay,afacing, goal_locs,robot}
  end
  send_robot_status(robot,cli_proc_name)
  sending_coor(goal_locs, robot)
{robot,ax,ay,afacing, goal_locs}
end

def forGoal_x(ax,ay,afacing, goal_locs,robot, _goal_x, _cli_proc_name) do
  {robot,ax,ay,afacing, goal_locs}
end

def forGoal_y(_ax,_ay,_afacing, goal_locs,robot,goal_y, cli_proc_name) when robot.y < goal_y and robot.facing != :north do
  {ax,ay,afacing, goal_locs} = receiving_coor(goal_locs)
  {ax,ay,afacing, goal_locs,robot} = cond do
    robot.facing == :west ->
      robot = right(robot)
      {ax,ay,afacing, goal_locs,robot}
    robot.facing == :south ->
      robot = left(robot)
      send_robot_status(robot,cli_proc_name)
      sending_coor(goal_locs, robot)
      {ax,ay,afacing, goal_locs} = receiving_coor(goal_locs)
      robot = left(robot)
      {ax,ay,afacing, goal_locs,robot}
    robot.facing == :east ->
      robot = left(robot)
      {ax,ay,afacing, goal_locs,robot}
  end
  send_robot_status(robot,cli_proc_name)
  sending_coor(goal_locs, robot)
{robot,ax,ay,afacing, goal_locs}
end

def forGoal_y(_ax,_ay,_afacing, goal_locs,robot,goal_y, cli_proc_name) when robot.y > goal_y and robot.facing != :south do
{ax,ay,afacing, goal_locs } = receiving_coor(goal_locs)
  {ax,ay,afacing, goal_locs,robot} = cond do
  robot.facing == :east ->
    robot = right(robot)
    {ax,ay,afacing, goal_locs,robot}
  robot.facing == :north ->
    robot = left(robot)
    send_robot_status(robot,cli_proc_name)
    sending_coor(goal_locs, robot)
    {ax,ay,afacing, goal_locs} = receiving_coor(goal_locs)

    robot = left(robot)
    {ax,ay,afacing, goal_locs,robot}
  robot.facing == :west ->
    robot = left(robot)
    {ax,ay,afacing, goal_locs,robot}
end
send_robot_status(robot,cli_proc_name)
sending_coor(goal_locs, robot)

{robot,ax,ay,afacing, goal_locs}
end

def forGoal_y(ax,ay,afacing, goal_locs,robot, _goal_y, _cli_proc_name) do
{robot,ax,ay,afacing, goal_locs}
end


def goX(_ax,_ay,_afacing, goal_locs,%CLI.Position{facing: _facing, x: x, y: _y} = robot, goal_x, goal_y, cli_proc_name,_ob) when x != goal_x do
  {ax,ay,afacing, goal_locs} = receiving_coor(goal_locs)
  robot = move(robot)
  # IO.puts("hello")
  ob = send_robot_status(robot,cli_proc_name)
  sending_coor(goal_locs, robot)

  goX(ax,ay,afacing, goal_locs,robot,goal_x,goal_y,cli_proc_name,ob)
end

def goX(ax,ay,afacing, goal_locs,robot, _goal_x, _goal_y, _cli_proc_name,ob) do
  {robot,ob,ax,ay,afacing, goal_locs}
end

def goY(_ax,_ay,_afacing, goal_locs,%CLI.Position{facing: _facing, x: _x, y: y} = robot, goal_x, goal_y, cli_proc_name,_ob) when y != goal_y do
  {ax,ay,afacing, goal_locs} = receiving_coor(goal_locs)
  robot = move(robot)
  ob = send_robot_status(robot,cli_proc_name)
  sending_coor(goal_locs, robot)

  goY(ax,ay,afacing, goal_locs,robot,goal_x,goal_y,cli_proc_name,ob)
end

def goY(ax,ay,afacing, goal_locs,robot, _goal_x, _goal_y, _cli_proc_name,ob) do
  {robot,ob,ax,ay,afacing, goal_locs}
end


  @doc """
  Send Toy Robot's current status i.e. location (x, y) and facing
  to the CLI Server process after each action is taken.
  Listen to the CLI Server and wait for the message indicating the presence of obstacle.
  The message with the format: '{:obstacle_presence, < true or false >}'.
  """
  def send_robot_status(%CLI.Position{x: x, y: y, facing: facing} = _robot, cli_proc_name) do
    send(cli_proc_name, {:toyrobotB_status, x, y, facing})
    # IO.puts("Sent by Toy Robot Client: #{x}, #{y}, #{facing}")
    listen_from_server()
  end

  @doc """
  Listen to the CLI Server and wait for the message indicating the presence of obstacle.
  The message with the format: '{:obstacle_presence, < true or false >}'.
  """
  def listen_from_server() do
    receive do
      {:obstacle_presence, is_obs_ahead} ->
        is_obs_ahead
    end
  end

  @doc """
  Provides the report of the robot's current position

  Examples:

      iex> {:ok, robot} = CLI.ToyRobotB.place(2, :b, :west)
      iex> CLI.ToyRobotB.report(robot)
      {2, :b, :west}
  """
  def report(%CLI.Position{x: x, y: y, facing: facing} = _robot) do
    {x, y, facing}
  end

  @directions_to_the_right %{north: :east, east: :south, south: :west, west: :north}
  @doc """
  Rotates the robot to the right
  """
  def right(%CLI.Position{facing: facing} = robot) do
    %CLI.Position{robot | facing: @directions_to_the_right[facing]}
  end

  @directions_to_the_left Enum.map(@directions_to_the_right, fn {from, to} -> {to, from} end)
  @doc """
  Rotates the robot to the left
  """
  def left(%CLI.Position{facing: facing} = robot) do
    %CLI.Position{robot | facing: @directions_to_the_left[facing]}
  end

  @doc """
  Moves the robot to the north, but prevents it to fall
  """
  def move(%CLI.Position{x: _, y: y, facing: :north} = robot) when y < @table_top_y do
    %CLI.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the east, but prevents it to fall
  """
  def move(%CLI.Position{x: x, y: _, facing: :east} = robot) when x < @table_top_x do
    %CLI.Position{robot | x: x + 1}
  end

  @doc """
  Moves the robot to the south, but prevents it to fall
  """
  def move(%CLI.Position{x: _, y: y, facing: :south} = robot) when y > :a do
    %CLI.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the west, but prevents it to fall
  """
  def move(%CLI.Position{x: x, y: _, facing: :west} = robot) when x > 1 do
    %CLI.Position{robot | x: x - 1}
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
