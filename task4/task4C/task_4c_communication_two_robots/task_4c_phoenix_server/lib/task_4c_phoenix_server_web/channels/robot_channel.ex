defmodule Task4CPhoenixServerWeb.RobotChannel do
  use Phoenix.Channel

  @doc """
  Handler function for any Client joining the channel with topic "robot:status".
  Subscribe to the topic named "robot:update" on the Phoenix Server using Endpoint.
  Reply or Acknowledge with socket PID received from the Client.
  """
  def join("robot:status", _params, socket) do
    Task4CPhoenixServerWeb.Endpoint.subscribe("robot:update")
    {:ok, socket}
  end

  @doc """
  Callback function for messages that are pushed to the channel with "robot:status" topic with an event named "new_msg".
  Receive the message from the Client, parse it to create another Map strictly of this format:
  %{"client" => < "robot_A" or "robot_B" >,  "left" => < left_value >, "bottom" => < bottom_value >, "face" => < face_value > }

  These values should be pixel locations for the robot's image to be displayed on the Dashboard
  corresponding to the various actions of the robot as recevied from the Client.

  Broadcast the created Map of pixel locations, so that the ArenaLive module can update
  the robot's image and location on the Dashboard as soon as it receives the new data.

  Based on the message from the Client, determine the obstacle's presence in front of the robot
  and return the boolean value in this format {:ok, < true OR false >}.

  If an obstacle is present ahead of the robot, then broadcast the pixel location of the obstacle to be displayed on the Dashboard.
  """
  def handle_in("new_msg", message, socket) do

    # determine the obstacle's presence in front of the robot and return the boolean value
    is_obs_ahead = Task4CPhoenixServerWeb.FindObstaclePresence.is_obstacle_ahead?(message["x"], message["y"], message["face"])

    # file object to write each action taken by each Robot (A as well as B)
    {:ok, out_file} = File.open("task_4c_output.txt", [:append])
    # write the robot actions to a text file
    IO.binwrite(out_file, "#{message["client"]} => #{message["x"]}, #{message["y"]}, #{message["face"]}\n")

    ###########################
    ## complete this funcion ##
    ###########################
    mp = %{"a" => 0, "b" => 1, "c" => 2, "d" => 3, "e" => 4, "f" => 5, "g" => 6}
    # IO.inspect(message)

    msg2 = if(message["x"] < 7) do
      msg2 = if(is_obs_ahead == false) do
      %{"client" => message["client"], "left" => (message["x"]-1)*150, "bottom" => Map.get(mp, message["y"])*150, "face" => message["face"],  "obs" => is_obs_ahead, "x" => 0, "y" => 0}
    else
      facing = message["face"]
      {x,y} = cond do
        facing == "north" ->
          {(message["x"]-1)*150,(Map.get(mp, message["y"])*150)+75}
        facing == "south" ->
          {(message["x"]-1)*150,(Map.get(mp, message["y"])*150)-75}
        facing == "east" ->
          {((message["x"]-1)*150)+75,(Map.get(mp, message["y"])*150)}
        facing == "west" ->
          {((message["x"]-1)*150)-75,(Map.get(mp, message["y"])*150)}
      end
      %{"client" => message["client"], "left" => (message["x"]-1)*150, "bottom" => Map.get(mp, message["y"])*150, "face" => message["face"], "obs" => is_obs_ahead, "x" => x, "y" => y}
    end
    msg2
    else
      nil
    end
    _msg3 = if(is_obs_ahead == false) do
      %{client: message["client"], x: message["x"], y: Map.get(mp, message["y"]), face: message["face"]}
    else
      %{client: "obs", x: 0, y: 0}
    end

    if(msg2 !=  nil ) do
      Phoenix.PubSub.broadcast(Task4CPhoenixServer.PubSub, "robot:update", msg2)
    else
    end

    # is_obs_ahead = if(message["client"] == "robot_A") do
    #   if(ax == 7 and ay == "g") do
    #     true
    #   else
    #     is_obs_ahead
    #   end
    # else
    #   if(bx == 7 and by == "g") do
    #     true
    #   else
    #     is_obs_ahead
    #   end
    # end

    # rep = [ax,ay,afacing,bx,by,bfacing,is_obs_ahead]
    {:reply, {:ok, is_obs_ahead}, socket}
  end

  def handle_in("get_bots",message,socket) do
    if (Process.whereis(:cli_robot_states) == nil and message["client"] == "robot_A") do
      IO.puts("jojo")
      pid = spawn_link(fn -> listen_from_cli("0","0","0","0","0","0") end)
      Process.register(pid, :cli_robot_states)
    end

    {ax,ay,afacing,bx,by,bfacing} = receiving_coors()
    robots = if(message["client"] == "robot_A") do
      %{ax: message["x"], ay: message["y"], afacing: message["face"], bx: bx, by: by, bfacing: bfacing}
    else
      %{ax: ax, ay: ay, afacing: afacing, bx: message["x"], by: message["y"], bfacing: message["face"]}
    end
    sending_coors(robots)

    IO.puts("a:#{ax} #{ay} b:#{bx} #{by}")
    if ax == 7 and ay == "g" and bx == 7 and by == "g" do
      IO.puts("over")
      Task4CPhoenixServerWeb.Endpoint.broadcast("timer:stop", "stop_timer", %{})
    end

    rep = [ax,ay,afacing,bx,by,bfacing]
    {:reply, {:ok, rep}, socket}
  end

  def handle_in("goals", message, socket) do
    y = File.stream!("Plant_Positions.csv")
    |> CSV.decode
    |> Enum.map(fn {_,x} ->
      x
    end)

    {_,y} = List.pop_at(y,0)
    sow = Enum.map(y,fn [a,_b] ->
      a
    end)
    weed = Enum.map(y,fn [_a,b] ->
      b
    end)
    goals = if(message["client"] == "robot_A") do
      sow
    else
      weed
    end
    {:reply, {:ok, goals}, socket}
  end

  def handle_in("start_pos", message, socket) do

    socket = if(message["client"] == "robot_A") do
      if(Process.whereis(:cli_robotA_start) != nil) do
        data = receiving_coors_start()
        assign(socket, :robotA_start, data)
      else
        assign(socket, :robotA_start, 0)
      end
    else
      if(Process.whereis(:cli_robotB_start) != nil) do
        data = receiving_coor_b_start()
        assign(socket, :robotB_start, data)
      else
        assign(socket, :robotB_start, 0)
      end
    end

    x = if(message["client"] == "robot_A") do
      if(socket.assigns.robotA_start == 0) do
        0
      else
        {x} = socket.assigns.robotA_start
        [x]
      end
    else
      if(socket.assigns.robotB_start == 0) do
        0
      else
        {x} = socket.assigns.robotB_start
        [x]
      end
    end

    {:reply, {:ok, x}, socket}
  end

  #########################################
  ## define callback functions as needed ##
  #########################################

  def handle_info(%{robotA_start: a, robotB_start: b} = _data, socket) do
    IO.puts("inside")
    # IO.inspect(data)
    # IO.inspect(Process.whereis(:cli_robotB_start))
    # IO.inspect(Process.whereis(:cli_robotA_start))
    if(Process.whereis(:cli_robotB_start) == nil) do
      pid = spawn_link(fn -> listen_from_cli_b_start(b) end)
      Process.register(pid, :cli_robotB_start)
    else
      Process.unregister(:cli_robotB_start)
      pid = spawn_link(fn -> listen_from_cli_b_start(b) end)
      Process.register(pid, :cli_robotB_start)
    end
    if(Process.whereis(:cli_robotA_start) == nil) do
      pid = spawn_link(fn -> listen_from_cli_a_start(a) end)
      Process.register(pid, :cli_robotA_start)
    else
      Process.unregister(:cli_robotA_start)
      pid = spawn_link(fn -> listen_from_cli_a_start(a) end)
      Process.register(pid, :cli_robotA_start)
    end
    {:noreply, socket, 1000}
  end

  # def handle_info(%{client: _cli, x: _x, y: _y, face: _face} = _data, socket) do
  #   IO.puts("inside")
  #   # IO.inspect(face)
  #   # IO.inspect(socket)
  #   {:noreply, socket}
  # end

  def handle_info(_data, socket) do
    # IO.puts("in handle_info")
    # IO.inspect(data)
    # socket = cond do
    #   data["client"] == "robot_A" ->
    #       socket = assign(socket, :robotA_x, data["left"])
    #       socket = assign(socket, :robotA_y, data["bottom"])
    #       assign(socket, :robotA_facing, data["face"])

    #   data["client"] == "robot_B" ->
    #       socket = assign(socket, :robotB_x, data["left"])
    #       socket = assign(socket, :robotB_y, data["bottom"])
    #       assign(socket, :robotB_facing, data["face"])

    #   true ->
    #     socket
    # end

    {:noreply, socket}
  end

  def send_robot_stat_b_start() do
    send(:cli_robotB_start, {:toyrobotB})
    rec_botB_start()
  end

  def rec_botB_start() do
    receive do
      {:positions, pos} -> pos
    end
  end

  def receiving_coor_b_start() do
    parent = self()
    pid2 = spawn_link(fn ->
      coor = send_robot_stat_b_start()
      send(parent, {coor})
    end)
    Process.register(pid2, :b_start)
    receive do
      {coor} -> coor
    end
  end

  def listen_from_cli_b_start(b) do
    receive do
      {:toyrobotB} ->
        send(:b_start, {:positions, {b}})
      end
  end

  def send_robot_stat_a_start() do
    send(:cli_robotA_start, {:toyrobotA})
    rec_botA_start()
  end

  def rec_botA_start() do
    receive do
      {:positions, pos} -> pos
    end
  end

  def receiving_coors_start() do
    parent = self()
    pid2 = spawn_link(fn ->
      coor = send_robot_stat_a_start()
      send(parent, {coor})
    end)
    Process.register(pid2, :a_start)
    receive do
      {coor} -> coor
    end
  end

  def listen_from_cli_a_start(a) do
    receive do
      {:toyrobotA} ->
        send(:a_start, {:positions, {a}})
      end
  end

  def listen_from_cli(ax,ay,afacing,bx,by,bfacing) do
    receive do
      {:toyrobots} ->
        send(:get_bots, {:positions, {ax,ay,afacing,bx,by,bfacing}})
      end
  end

  def send_robot_stats() do
    send(:cli_robot_states, {:toyrobots})
    rec_bots()
  end

  def rec_bots() do
    receive do
      {:positions, pos} -> pos
    end
  end

  def wait_until_received() do
    if (Process.whereis(:cli_robot_states) != nil and Process.whereis(:get_bots) == nil) do
      else
      # IO.puts("waiting 1")
      Process.sleep(100)
      wait_until_received()
    end
  end

  def wait_till_over() do
    if (Process.whereis(:cli_robot_states) != nil) do
      # IO.puts("waiting 2")
      Process.sleep(100)
      wait_till_over()
    end
  end

  def receiving_coors() do
    wait_until_received()
    parent = self()
    pid2 = spawn_link(fn ->
      coor = send_robot_stats()
      send(parent, {coor})
      end)
    Process.register(pid2, :get_bots)

    receive do
      {coor} -> coor
    end
  end

  def sending_coors(robots) do
    wait_till_over()
    %{ax: ax, ay: ay, afacing: afacing, bx: bx, by: by, bfacing: bfacing} = robots
    pid = spawn_link(fn -> listen_from_cli(ax,ay,afacing,bx,by,bfacing) end)
    Process.register(pid, :cli_robot_states)
  end

end
