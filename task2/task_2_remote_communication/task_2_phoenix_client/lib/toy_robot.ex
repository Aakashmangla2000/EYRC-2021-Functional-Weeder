defmodule ToyRobot do
  # max x-coordinate of table top
  @table_top_x 5
  # max y-coordinate of table top
  @table_top_y :e
  # mapping of y-coordinates
  @robot_map_y_atom_to_num %{:a => 1, :b => 2, :c => 3, :d => 4, :e => 5}

  @doc """
  Places the robot to the default position of (1, A, North)

  Examples:

      iex> ToyRobot.place
      {:ok, %ToyRobot.Position{facing: :north, x: 1, y: :a}}
  """
  def place do
    {:ok, %ToyRobot.Position{}}
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

      iex> ToyRobot.place(1, :b, :south)
      {:ok, %ToyRobot.Position{facing: :south, x: 1, y: :b}}

      iex> ToyRobot.place(-1, :f, :north)
      {:failure, "Invalid position"}

      iex> ToyRobot.place(3, :c, :north_east)
      {:failure, "Invalid facing direction"}
  """
  def place(x, y, facing) do
    {:ok, %ToyRobot.Position{x: x, y: y, facing: facing}}
  end

  @doc """
  Provide START position to the robot as given location of (x, y, facing) and place it.
  """
  def start(x, y, facing) do
    ###########################
    ## complete this funcion ##
    ###########################
    ToyRobot.place(x,y,facing)
  end

  def stop(_robot, goal_x, goal_y, _channel) when goal_x < 1 or goal_y < :a or goal_x > @table_top_x or goal_y > @table_top_y do
    {:failure, "Invalid STOP position"}
  end

  @doc """
  Provide STOP position to the robot as given location of (x, y) and plan the path from START to STOP.
  Passing the channel PID on the Phoenix Server that will be used to send robot's current status after each action is taken.
  Make a call to ToyRobot.PhoenixSocketClient.send_robot_status/2
  to get the indication of obstacle presence ahead of the robot.
  """
  def stop(robot, goal_x, goal_y, channel) do

    ###########################
    ## complete this funcion ##
    ###########################
    parent = self()

    get_value(channel, robot,goal_x, goal_y,cli_proc_name, parent)
    robot = rec_value()

    {:ok, robot}
  end


  def get_value(channel, robot,goal_x, goal_y,cli_proc_name, parent) do
    pid = spawn_link(fn ->
      len = 1
      q = :queue.new()
      visited = :queue.new()
      q = :queue.in({robot.x,robot.y,0},q)
      visited = :queue.in({robot.x,robot.y},visited)
      ToyRobot.PhoenixSocketClient.send_robot_status(channel, robot)
      robot = if(robot.x == goal_x and robot.y == goal_y) do
      else
      ToyRobot.rep(channel,q,visited,robot,goal_x,goal_y,cli_proc_name,len)
      end
      send(parent, {:flag_value, robot})
    end)
    Process.register(pid, :client_toyrobot)
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

  def rep(channel,q,visited,robot,goal_x,goal_y,cli_proc_name, len) when len != 0 do

    #getting next block
    {{:value, value3}, q} = :queue.out_r(q)
    {x,y, dir} = value3
    new_goal_x = x
    new_goal_y = y

    #if reached destination
    len = if(x == goal_x and y == goal_y) do
      0
    end

    #travelling to new goals
    robot = ToyRobot.forGoal_x(channel, robot,new_goal_x)
    robot = ToyRobot.goX(channel,robot,new_goal_x,new_goal_y,cli_proc_name)
    robot = ToyRobot.forGoal_y(channel, robot,new_goal_y)
    {robot,obs} = ToyRobot.goY(robot,new_goal_x,new_goal_y,cli_proc_name,false)

    #putting back with changed dir
    q = :queue.in({x,y,dir+1},q)

    #setting robot's direction based on dir
    both = cond do
      dir == 0 ->
        both = cond do
          robot.facing == :east ->
            robot = left(robot)
            obs = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            {robot, obs}
          robot.facing == :south ->
            robot = left(robot)
            ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            robot = left(robot)
            obs = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            {robot, obs}
          robot.facing == :west ->
            robot = right(robot)
            obs = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            {robot, obs}
          robot.facing == :north ->
            {robot, obs}
        end
        both
      dir == 1 ->
        both = cond do
          robot.facing == :north ->
            robot = left(robot)
            obs = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            {robot, obs}
          robot.facing == :east ->
            robot = left(robot)
            ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            robot = left(robot)
            obs = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            {robot, obs}
          robot.facing == :south ->
            robot = right(robot)
            obs = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            {robot, obs}
          robot.facing == :west ->
            {robot, obs}
        end
        both
      dir == 2 ->
        both = cond do
          robot.facing == :west ->
            robot = left(robot)
            obs = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            {robot, obs}
          robot.facing == :nprth ->
            robot = left(robot)
            ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            robot = left(robot)
            obs = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            {robot, obs}
          robot.facing == :east ->
            robot = right(robot)
            obs = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            {robot, obs}
          robot.facing == :south ->
            {robot, obs}
        end
        both
      dir == 3 ->
        both = cond do
          robot.facing == :south ->
            robot = left(robot)
            obs = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            {robot, obs}
          robot.facing == :west ->
            robot = left(robot)
            ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            robot = left(robot)
            obs = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
            {robot, obs}
          robot.facing == :north ->
            robot = right(robot)
            obs = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
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
        {{:value, val}, q} = :queue.out_r(q)
        {{:value, val}, visited} = :queue.out_r(visited)
        struc = {q,visited}
        struc
    end

    len = if(len == 0) do
      0
    else
      :queue.len(q)
    end
    rep(q,visited,robot,goal_x,goal_y,cli_proc_name, len)
  end

  def rep(channel,q,visited,robot,goal_x,goal_y,cli_proc_name, len) do
    robot
  end

  def forGoal_x(channel, robot,goal_x) when robot.x < goal_x and robot.facing != :east do
    robot = cond do
      robot.facing == :north ->
        right(robot)
      robot.facing == :west ->
        robot = left(robot)
        ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
        left(robot)
      robot.facing == :south ->
        left(robot)
    end
    ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
  robot
end

def forGoal_x(channel, robot,goal_x) when robot.x > goal_x and robot.facing != :west do
  robot = cond do
    robot.facing == :south ->
      right(robot)
    robot.facing == :east ->
      robot = left(robot)
      ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
      left(robot)
    robot.facing == :north ->
      left(robot)
  end
  ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
robot
end

def forGoal_x(channel, robot,goal_x) do
  robot
end

def forGoal_y(channel, robot,goal_y) when robot.y < goal_y and robot.facing != :north do
  robot = cond do
    robot.facing == :west ->
      right(robot)
    robot.facing == :south ->
      robot = left(robot)
      ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
      left(robot)
    robot.facing == :east ->
      left(robot)
  end
  ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
robot
end

def forGoal_y(channel, robot,goal_y) when robot.y > goal_y and robot.facing != :south do
robot = cond do
  robot.facing == :east ->
    right(robot)
  robot.facing == :north ->
    robot = left(robot)
    ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
    left(robot)
  robot.facing == :west ->
    left(robot)
end
ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
robot
end

def forGoal_y(channel, robot,goal_y) do
robot
end


def goX(channel,%ToyRobot.Position{facing: facing, x: x, y: y} = robot, goal_x, goal_y, cli_proc_name) when x != goal_x do
  robot = move(robot)
  ob = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
  # IO.puts(ob)
  goX(channel,robot,goal_x,goal_y,cli_proc_name)
end

def goX(channel,%ToyRobot.Position{facing: facing, x: x, y: y} = robot, goal_x, goal_y, cli_proc_name) do
  robot
end

def goY(%ToyRobot.Position{facing: facing, x: x, y: y} = robot, goal_x, goal_y, cli_proc_name,ob) when y != goal_y do
  robot = move(robot)
  ob = ToyRobot.PhoenixSocketClient.send_robot_status(channel,robot)
  goY(robot,goal_x,goal_y,cli_proc_name,ob)
end

def goY(%ToyRobot.Position{facing: facing, x: x, y: y} = robot, goal_x, goal_y, cli_proc_name,ob) do
  {robot,ob}
end

  @doc """
  Provides the report of the robot's current position

  Examples:

      iex> {:ok, robot} = ToyRobot.place(2, :b, :west)
      iex> ToyRobot.report(robot)
      {2, :b, :west}
  """
  def report(%ToyRobot.Position{x: x, y: y, facing: facing} = _robot) do
    {x, y, facing}
  end

  @directions_to_the_right %{north: :east, east: :south, south: :west, west: :north}
  @doc """
  Rotates the robot to the right
  """
  def right(%ToyRobot.Position{facing: facing} = robot) do
    %ToyRobot.Position{robot | facing: @directions_to_the_right[facing]}
  end

  @directions_to_the_left Enum.map(@directions_to_the_right, fn {from, to} -> {to, from} end)
  @doc """
  Rotates the robot to the left
  """
  def left(%ToyRobot.Position{facing: facing} = robot) do
    %ToyRobot.Position{robot | facing: @directions_to_the_left[facing]}
  end

  @doc """
  Moves the robot to the north, but prevents it to fall
  """
  def move(%ToyRobot.Position{x: _, y: y, facing: :north} = robot) when y < @table_top_y do
    %ToyRobot.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) + 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the east, but prevents it to fall
  """
  def move(%ToyRobot.Position{x: x, y: _, facing: :east} = robot) when x < @table_top_x do
    %ToyRobot.Position{robot | x: x + 1}
  end

  @doc """
  Moves the robot to the south, but prevents it to fall
  """
  def move(%ToyRobot.Position{x: _, y: y, facing: :south} = robot) when y > :a do
    %ToyRobot.Position{robot | y: Enum.find(@robot_map_y_atom_to_num, fn {_, val} -> val == Map.get(@robot_map_y_atom_to_num, y) - 1 end) |> elem(0)}
  end

  @doc """
  Moves the robot to the west, but prevents it to fall
  """
  def move(%ToyRobot.Position{x: x, y: _, facing: :west} = robot) when x > 1 do
    %ToyRobot.Position{robot | x: x - 1}
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
