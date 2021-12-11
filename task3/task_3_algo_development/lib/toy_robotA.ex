defmodule CLI.ToyRobotA do
  # max x-coordinate of table top
  @table_top_x 5
  # max y-coordinate of table top
  @table_top_y :e
  # mapping of y-coordinates
  @robot_map_y_atom_to_num %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5}

  @doc """
  Places the robot to the default position of (1, A, North)

  Examples:

      iex> CLI.ToyRobotA.place
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

      iex> CLI.ToyRobotA.place(1, :b, :south)
      {:ok, %CLI.Position{facing: :south, x: 1, y: :b}}

      iex> CLI.ToyRobotA.place(-1, :f, :north)
      {:failure, "Invalid position"}

      iex> CLI.ToyRobotA.place(3, :c, :north_east)
      {:failure, "Invalid facing direction"}
  """
  def place(x, y, facing) do
    # IO.puts String.upcase("A I'm placed at => #{x},#{y},#{facing}")
    {:ok, %CLI.Position{x: x, y: y, facing: facing}}
  end

  @doc """
  Provide START position to the robot as given location of (x, y, facing) and place it.
  """
  def start(x, y, facing) do
    ###########################
    ## complete this funcion ##
    ###########################
    CLI.ToyRobotA.place(x, y, facing)
  end

  def stop(_robot, goal_x, goal_y, _cli_proc_name) when goal_x < 1 or goal_y < :a or goal_x > @table_top_x or goal_y > @table_top_y do
    {:failure, "Invalid STOP position"}
  end

  @doc """
  Provide GOAL positions to the robot as given location of [(x1, y1),(x2, y2),..] and plan the path from START to these locations.
  Passing the CLI Server process name that will be used to send robot's current status after each action is taken.
  Spawn a process and register it with name ':client_toyrobotA' which is used by CLI Server to send an
  indication for the presence of obstacle ahead of robot's current position and facing.
  """
  def stop(robot, goal_locs, cli_proc_name) do
    ###########################
    ## complete this funcion ##
    ###########################

    mp = %{"1" => 1, "2" => 2, "3" => 3, "4" => 4, "5" => 5}
    mp2 = %{"a" => :a, "b" => :b, "c" => :c, "d" => :d, "e" => :e}
    val = Enum.at(goal_locs,0)
    goal_x = Enum.at(val,0)
    goal_x = Map.get(mp, goal_x)
    goal_y = Enum.at(val,1)
    goal_y = Map.get(mp2, goal_y)

    parent = self()

    get_value(robot,goal_x, goal_y,cli_proc_name, parent)
    robot = rec_value()

    {:ok, robot}
  end

  # def repeat_process(robot) do

  #   wait_until_received()
  #   pid2 = spawn_link(fn ->
  #     coor = send_robot_stat()
  #     {x,y,facing} = coor
  #     IO.puts(x)
  #     IO.puts(y)
  #     IO.puts(facing)
  #     end)
  #   Process.register(pid2, :get_botB)

  #   #robot

  #   wait_till_over()
  #   %CLI.Position{x: px, y: py, facing: pfacing} = robot
  #   pid = spawn_link(fn -> listen_from_cli(px,py,pfacing) end)
  #   Process.register(pid, :cli_robotA_state)
  # end

  def wait_until_received() do
    if (Process.whereis(:cli_robotB_state) == nil) do
      # IO.puts("waiting1")
      Process.sleep(100)
      wait_until_received()
    end
  end

  def wait_till_over() do
    if (Process.whereis(:cli_robotA_state) != nil) do
      # IO.puts("waiting2")
      Process.sleep(100)
      wait_till_over()
    end
  end

  def get_value(robot,goal_x, goal_y,cli_proc_name, parent) do
    pid = spawn_link(fn ->
      len = 1
      q = :queue.new()
      visited = :queue.new()
      q = :queue.in({robot.x,robot.y,0},q)
      visited = :queue.in({robot.x,robot.y},visited)
      send_robot_status(robot,cli_proc_name)

      %CLI.Position{x: px, y: py, facing: pfacing} = robot
        pid = spawn_link(fn -> listen_from_cli(px,py,pfacing) end)
        Process.register(pid, :cli_robotA_state)

      robot = if(robot.x == goal_x and robot.y == goal_y) do
      else
      CLI.ToyRobotA.rep( q,visited,robot,goal_x,goal_y,cli_proc_name,len)
      end
      send(parent, {:flag_value, robot})
    end)
    Process.register(pid, :client_toyrobotA)
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

  def listen_from_cli(px,py,pfacing) do
    receive do
      {:toyrobotB} ->
        send(:get_botA, {:positions, {px,py,pfacing}})
      end
  end

  def send_robot_stat() do
    send(:cli_robotB_state, {:toyrobotA})
    rec_botB()
  end

  def rec_botB() do
    receive do
      {:positions, pos} -> pos
    end
  end

  def receiving_coor(parent) do
    if(Process.whereis(:client_toyrobotB) != nil) do
    wait_until_received()
    pid2 = spawn_link(fn ->
      coor = send_robot_stat()
      # {x,y,facing} = coor
      # IO.puts("Robot B: #{x} #{y} #{facing}")
      send(parent, {coor})
      end)
    Process.register(pid2, :get_botB)
    else
      send(parent,{0,0,0})
    end
  end

  def sending_coor(robot) do
    if(Process.whereis(:client_toyrobotB) != nil) do
    wait_till_over()
    %CLI.Position{x: px, y: py, facing: pfacing} = robot
    pid = spawn_link(fn -> listen_from_cli(px,py,pfacing) end)
    Process.register(pid, :cli_robotA_state)
    end
  end

  def rep( q,visited,robot,goal_x,goal_y,cli_proc_name, len) when len != 0 do
    #getting next block
    {{:value, value3}, q} = :queue.out_r(q)
    {x,y, dir} = value3
    new_goal_x = x
    new_goal_y = y

    #if reached destination
    len = if(x == goal_x and y == goal_y) do
      IO.puts("Reached #{x} #{y}")
      0
    end
    parent = self()
    #receiving coordinates from B
    receiving_coor(parent)
    {bx,by,_bfacing} = if(Process.whereis(:client_toyrobotB) != nil) do
    receive do
      {coor} -> coor
    end
    else
      {0,0,0}
    end

    {q,visited,robot,len} = if(new_goal_x == bx and new_goal_y == by) do
      IO.puts("A crash into B")
      send_robot_status(robot,cli_proc_name)
      q = :queue.in({x,y,dir},q)
      {q,visited,robot,len}
    else
      #travelling to new goals
        robot = CLI.ToyRobotA.forGoal_x(robot,new_goal_x, cli_proc_name)
        robot = CLI.ToyRobotA.goX(robot,new_goal_x,new_goal_y,cli_proc_name)
        robot = CLI.ToyRobotA.forGoal_y(robot,new_goal_y, cli_proc_name)
        {robot,obs} = CLI.ToyRobotA.goY(robot,new_goal_x,new_goal_y,cli_proc_name,false)

        #putting back with changed dir
        q = :queue.in({x,y,dir+1},q)

        #setting robot's direction based on dir
        both = cond do
          dir == 0 ->
            both = cond do
              robot.facing == :east ->
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                {robot, obs}
              robot.facing == :south ->
                robot = left(robot)
                send_robot_status(robot,cli_proc_name)
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                {robot, obs}
              robot.facing == :west ->
                robot = right(robot)
                obs = send_robot_status(robot,cli_proc_name)
                {robot, obs}
              robot.facing == :north ->
                {robot, obs}
            end
            both
          dir == 1 ->
            both = cond do
              robot.facing == :north ->
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                {robot, obs}
              robot.facing == :east ->
                robot = left(robot)
                send_robot_status(robot,cli_proc_name)
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                {robot, obs}
              robot.facing == :south ->
                robot = right(robot)
                obs = send_robot_status(robot,cli_proc_name)
                {robot, obs}
              robot.facing == :west ->
                {robot, obs}
            end
            both
          dir == 2 ->
            both = cond do
              robot.facing == :west ->
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                {robot, obs}
              robot.facing == :nprth ->
                robot = left(robot)
                send_robot_status(robot,cli_proc_name)
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                {robot, obs}
              robot.facing == :east ->
                robot = right(robot)
                obs = send_robot_status(robot,cli_proc_name)
                {robot, obs}
              robot.facing == :south ->
                {robot, obs}
            end
            both
          dir == 3 ->
            both = cond do
              robot.facing == :south ->
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                {robot, obs}
              robot.facing == :west ->
                robot = left(robot)
                send_robot_status(robot,cli_proc_name)
                robot = left(robot)
                obs = send_robot_status(robot,cli_proc_name)
                {robot, obs}
              robot.facing == :north ->
                robot = right(robot)
                obs = send_robot_status(robot,cli_proc_name)
                {robot, obs}
              robot.facing == :east ->
                {robot, obs}
            end
            both
          dir > 3 ->
            {robot, obs}
        end

        {robot, obs} = both
        struc = {q,visited}

        {q, visited} = cond do

          dir == 0 ->
            #up
            check = if(y == :e or obs == true) do
              false
            else
              !(:queue.member({x,plus(y)}, visited))
            end
              struc4 = if check do
                  q = :queue.in({x,plus(y),0}, q)
                  visited = :queue.in({x,plus(y)}, visited)
                  {q,visited}
                else
                  {q,visited}
              end

              if(is_nil(struc4)) do
                struc
              else
                struc4
              end

          dir == 1 ->
            #left
            check = if(x == 1 or obs == true) do
              false
            else
              !(:queue.member({x-1,y}, visited))
            end
              struc1 = if check do
                  q = :queue.in({x-1,y,0}, q)
                  visited = :queue.in({x-1,y}, visited)
                  {q,visited}
                else
                  {q,visited}
              end

              if(is_nil(struc1)) do
                struc
              else
                struc1
              end

          dir == 2 ->
            #down
            check = if(y == :a or obs == true) do
              false
            else
              !(:queue.member({x,minus(y)}, visited))
            end
            struc2 = if check do
                q = :queue.in({x,minus(y),0}, q)
                visited = :queue.in({x,minus(y)}, visited)
                {q,visited}
              else
              {q,visited}
            end


            if(is_nil(struc2)) do
              struc
            else
              struc2
            end

          dir == 3 ->
            #right
            check = if(x == 5 or obs == true) do
              false
            else
              !(:queue.member({x+1,y}, visited))
            end
            struc3 = if check do
                q = :queue.in({x+1,y,0}, q)
                visited = :queue.in({x+1,y}, visited)
                {q,visited}
              else
                {q,visited}
            end

            if(is_nil(struc3)) do
              struc
            else
              struc3
            end

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
        {q,visited,robot,len}
    end

    #sends coordinates to B
    sending_coor(robot)
    rep( q,visited,robot,goal_x,goal_y,cli_proc_name, len)
  end

  def rep( _q,_visited,robot,_goal_x,_goal_y,_cli_proc_name, _len) do
    robot
  end

  def forGoal_x(robot,goal_x, cli_proc_name) when robot.x < goal_x and robot.facing != :east do
    robot = cond do
      robot.facing == :north ->
        right(robot)
      robot.facing == :west ->
        robot = left(robot)
        send_robot_status(robot,cli_proc_name)
        left(robot)
      robot.facing == :south ->
        left(robot)
    end
    send_robot_status(robot,cli_proc_name)
  robot
end

def forGoal_x(robot,goal_x, cli_proc_name) when robot.x > goal_x and robot.facing != :west do
  robot = cond do
    robot.facing == :south ->
      right(robot)
    robot.facing == :east ->
      robot = left(robot)
      send_robot_status(robot,cli_proc_name)
      left(robot)
    robot.facing == :north ->
      left(robot)
  end
  send_robot_status(robot,cli_proc_name)
robot
end

def forGoal_x(robot,_goal_x, _cli_proc_name) do
  robot
end

def forGoal_y(robot,goal_y, cli_proc_name) when robot.y < goal_y and robot.facing != :north do
  robot = cond do
    robot.facing == :west ->
      right(robot)
    robot.facing == :south ->
      robot = left(robot)
      send_robot_status(robot,cli_proc_name)
      left(robot)
    robot.facing == :east ->
      left(robot)
  end
  send_robot_status(robot,cli_proc_name)
robot
end

def forGoal_y(robot,goal_y, cli_proc_name) when robot.y > goal_y and robot.facing != :south do
robot = cond do
  robot.facing == :east ->
    right(robot)
  robot.facing == :north ->
    robot = left(robot)
    send_robot_status(robot,cli_proc_name)
    left(robot)
  robot.facing == :west ->
    left(robot)
end
send_robot_status(robot,cli_proc_name)
robot
end

def forGoal_y(robot,_goal_y, _cli_proc_name) do
robot
end


def goX(%CLI.Position{facing: _facing, x: x, y: _y} = robot, goal_x, goal_y, cli_proc_name) when x != goal_x do
  robot = move(robot)
  send_robot_status(robot,cli_proc_name)
  goX(robot,goal_x,goal_y,cli_proc_name)
end

def goX(robot, _goal_x, _goal_y, _cli_proc_name) do
  robot
end

def goY(%CLI.Position{facing: _facing, x: _x, y: y} = robot, goal_x, goal_y, cli_proc_name,_ob) when y != goal_y do
  robot = move(robot)
  ob = send_robot_status(robot,cli_proc_name)
  goY(robot,goal_x,goal_y,cli_proc_name,ob)
end

def goY(robot, _goal_x, _goal_y, _cli_proc_name,ob) do
  {robot,ob}
end

  @doc """
  Send Toy Robot's current status i.e. location (x, y) and facing
  to the CLI Server process after each action is taken.
  Listen to the CLI Server and wait for the message indicating the presence of obstacle.
  The message with the format: '{:obstacle_presence, < true or false >}'.
  """
  def send_robot_status(%CLI.Position{x: x, y: y, facing: facing} = _robot, cli_proc_name) do
    send(cli_proc_name, {:toyrobotA_status, x, y, facing})
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

      iex> {:ok, robot} = CLI.ToyRobotA.place(2, :b, :west)
      iex> CLI.ToyRobotA.report(robot)
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
