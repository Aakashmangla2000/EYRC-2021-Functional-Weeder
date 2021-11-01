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

  def stop(_robot, goal_x, goal_y, _cli_proc_name) when goal_x < 1 or goal_y < :a or goal_x > @table_top_x or goal_y > @table_top_y do
    {:failure, "Invalid STOP position"}
  end

  @doc """
  Provide STOP position to the robot as given location of (x, y) and plan the path from START to STOP.
  Passing the CLI Server process name that will be used to send robot's current status after each action is taken.
  Spawn a process and register it with name ':client_toyrobot' which is used by CLI Server to send an
  indication for the presence of obstacle ahead of robot's current position and facing.
  """
  def stop(robot, goal_x, goal_y, cli_proc_name) do
    ###########################
    ## complete this funcion ##
    ###########################
    # {:ok, pid} = ToyRobot.listen_from_server
    # pid = spawn(ToyRobot, :listen_from_server, [])
    # pid = spawn_link(fn ->
    #   result = receive do
    #     :true -> true
    #     :false -> false
    #     _ -> :none
    #   end
    #  end)
  #   pid = spawn_link(fn ->
  #     x = rec()
  #     IO.puts(x)
  #  end)
    # pid = spawn_link(fn -> rec() end)
    pid = spawn_link(fn ->
      x = send_robot_status(robot,cli_proc_name)
      IO.puts(x)
      robot = forGoal_x(robot, goal_x)
      robot = ToyRobot.goX(robot,goal_x,goal_y,cli_proc_name)
      robot = if(robot.x == goal_x) do
        robot = forGoal_y(robot, goal_y)
        ToyRobot.goY(robot,goal_x,goal_y,cli_proc_name)
      else
        robot = forGoal_x(robot, goal_x)
        ToyRobot.goX(robot,goal_x,goal_y,cli_proc_name)
      end

    end)
    Process.register(pid, :client_toyrobot)

    {:ok, robot}
  end

  def forGoal_x(robot,goal_x) when robot.x < goal_x and robot.facing != :east do
    robot = cond do
      robot.facing == :north ->
        right(robot)
      robot.facing == :west ->
        robot = left(robot)
        send_robot_status(robot,:cli_robot_state)
        left(robot)
      robot.facing == :south ->
        left(robot)
    end
    send_robot_status(robot,:cli_robot_state)
  robot
end

def forGoal_x(robot,goal_x) when robot.x > goal_x and robot.facing != :west do
  robot = cond do
    robot.facing == :south ->
      right(robot)
    robot.facing == :east ->
      robot = left(robot)
      send_robot_status(robot,:cli_robot_state)
      left(robot)
    robot.facing == :north ->
      left(robot)
  end
  send_robot_status(robot,:cli_robot_state)
robot
end

def forGoal_x(robot,goal_x) do
  robot
end

def forGoal_y(robot,goal_y) when robot.y < goal_y and robot.facing != :north do
  robot = cond do
    robot.facing == :west ->
      right(robot)
    robot.facing == :south ->
      robot = left(robot)
      send_robot_status(robot,:cli_robot_state)
      left(robot)
    robot.facing == :east ->
      left(robot)
  end
  send_robot_status(robot,:cli_robot_state)
robot
end

def forGoal_y(robot,goal_y) when robot.y > goal_y and robot.facing != :south do
robot = cond do
  robot.facing == :east ->
    right(robot)
  robot.facing == :north ->
    robot = left(robot)
    send_robot_status(robot,:cli_robot_state)
    left(robot)
  robot.facing == :west ->
    left(robot)
end
send_robot_status(robot,:cli_robot_state)
robot
end

def forGoal_y(robot,goal_y) do
robot
end

  def goX(%ToyRobot.Position{facing: facing, x: x, y: y} = robot, goal_x, goal_y, cli_proc_name) when x != goal_x do
    robot = move(robot)
    x = send_robot_status(robot,:cli_robot_state)
    IO.puts(x)
    robot = if(x == true) do
      IO.puts("yup")
      robot = ToyRobot.forGoal_y(robot,goal_y)
      ToyRobot.goY(robot,goal_x,goal_y,cli_proc_name)
    else
    goX(robot,goal_x,goal_y,cli_proc_name)
    end
    # robot
  end

  def goX(%ToyRobot.Position{facing: facing, x: x, y: y} = robot, goal_x, goal_y, cli_proc_name) do
    robot
  end

  def goY(%ToyRobot.Position{facing: facing, x: x, y: y} = robot, goal_x, goal_y, cli_proc_name) when y != goal_y do
    robot = move(robot)
    x = send_robot_status(robot,:cli_robot_state)
    IO.puts(x)
    robot = if(x == true) do
      IO.puts("yup")
      robot = ToyRobot.forGoal_x(robot,goal_x)
      ToyRobot.goX(robot,goal_x,goal_y,cli_proc_name)
    else
    goY(robot,goal_x,goal_y,cli_proc_name)
    end
    # robot
  end

  def goY(%ToyRobot.Position{facing: facing, x: x, y: y} = robot, goal_x, goal_y, cli_proc_name) do
    robot
  end

  @doc """
  Send Toy Robot's current status i.e. location (x, y) and facing
  to the CLI Server process after each action is taken.
  Listen to the CLI Server and wait for the message indicating the presence of obstacle.
  The message with the format: '{:obstacle_presence, < true or false >}'.
  """
  def send_robot_status(%ToyRobot.Position{x: x, y: y, facing: facing} = _robot, cli_proc_name) do
    send(cli_proc_name, {:toyrobot_status, x, y, facing})
    # IO.puts("Sent by Toy Robot Client: #{x}, #{y}, #{facing}")
    listen_from_server()
  end

  @doc """
  Listen to the CLI Server and wait for the message indicating the presence of obstacle.
  The message with the format: '{:obstacle_presence, < true or false >}'.
  """
  def listen_from_server() do
    receive do
      {:obstacle_presence, is_obs_ahead} -> is_obs_ahead
    end
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
