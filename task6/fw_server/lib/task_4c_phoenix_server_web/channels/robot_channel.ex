defmodule Task4CPhoenixServerWeb.RobotChannel do
  use Phoenix.Channel

  @doc """
  Handler function for any Client joining the channel with topic "robot:status".
  Subscribe to the topic named "robot:update" on the Phoenix Server using Endpoint.
  Reply or Acknowledge with socket PID received from the Client.
  """
  def join("robot:status", _params, socket) do
    Task4CPhoenixServerWeb.Endpoint.subscribe("robot:update")
    Task4CPhoenixServerWeb.Endpoint.subscribe("timer:update")
    socket = assign(socket, :timer_tick, 180)
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
    # is_obs_ahead = Task4CPhoenixServerWeb.FindObstaclePresence.is_obstacle_ahead?(message["x"], message["y"], message["face"])
    is_obs_ahead = message["obs"]

    # file object to write each action taken by each Robot (A as well as B)
    {:ok, out_file} = File.open("task_4c_output.txt", [:append])
    # write the robot actions to a text file
    IO.binwrite(out_file, "#{message["client"]} => #{message["x"]}, #{message["y"]}, #{message["face"]}\n")

    ###########################
    ## complete this funcion ##
    ###########################
    mp = %{"a" => 0, "b" => 1, "c" => 2, "d" => 3, "e" => 4, "f" => 5, "g" => 6}

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

    if(msg2 !=  nil ) do
      Phoenix.PubSub.broadcast(Task4CPhoenixServer.PubSub, "robot:update", msg2)
    else
    end

    {:reply, {:ok, true}, socket}
  end

  def handle_in("get_bots",message,socket) do
    resource_id = {User, {:id, 1}}
    update_user = fn(parent,message) ->
      lock = Mutex.await(MyMutex, resource_id)
      {ax,ay,afacing,bx,by,bfacing,a_start,b_start,a_alive,b_alive} = GenServer.call(Positions, :pop)
      robots = if(message["client"] == "robot_A") do
        %{ax: message["x"], ay: message["y"], afacing: message["face"], bx: bx, by: by, bfacing: bfacing, a_alive: message["alive"], b_alive: b_alive}
      else
        %{ax: ax, ay: ay, afacing: afacing, bx: message["x"], by: message["y"], bfacing: message["face"], a_alive: a_alive, b_alive: message["alive"]}
      end
      %{ax: ax, ay: ay, afacing: afacing, bx: bx, by: by, bfacing: bfacing ,a_alive: a_alive, b_alive: b_alive} = robots
      GenServer.cast(Positions,{:push, {ax,ay,afacing,bx,by,bfacing,a_start,b_start,a_alive,b_alive}})
      Mutex.release(MyMutex, lock)
      send(parent,{:pos,[ax,ay,afacing,bx,by,bfacing,a_alive,b_alive]})
    end
    parent = self()
    spawn(fn -> update_user.(parent,message) end)
    [ax,ay,afacing,bx,by,bfacing,a_alive,b_alive] = receive do
      {:pos, value} -> value
    end

    if a_alive == false and b_alive == false do
      Task4CPhoenixServerWeb.Endpoint.broadcast("timer:stop", "stop_timer", %{})
    end

    rep = [ax,ay,afacing,bx,by,bfacing,a_alive,b_alive]
    {:reply, {:ok, rep}, socket}
  end

  def handle_in("goals", message, socket) do
    y = File.stream!("Plant_Positions.csv")
    |> CSV.decode
    |> Enum.map(fn {_,x} ->
      x
    end)

    {_,y} = List.pop_at(y,0)
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
    Phoenix.PubSub.broadcast(Task4CPhoenixServer.PubSub, "robot:update", %{sow: sow, weed: weed})
    {:reply, {:ok, goals}, socket}
  end

  def handle_in("start_pos", message, socket) do

    if(GenServer.whereis(Positions) == nil) do
      {:ok, _} = GenServer.start_link(Task4CPhoenixServer.Stack, [{"0","0","0","0","0","0",0,0,false,false}], name: Positions)
    end

    resource_id = {User, {:id, 1}}
    update_user = fn(parent,message) ->
      lock = Mutex.await(MyMutex, resource_id)
      {ax,ay,afacing,bx,by,bfacing,a_start,b_start,a_alive,b_alive} = GenServer.call(Positions, :pop)
      socket = if(message["client"] == "robot_A") do
        assign(socket, :robotA_start, a_start)
      else
        assign(socket, :robotB_start, b_start)
      end
      GenServer.cast(Positions,{:push, {ax,ay,afacing,bx,by,bfacing,a_start,b_start,a_alive,b_alive}})
      Mutex.release(MyMutex, lock)
      send(parent,{:socket,socket})
    end
    parent = self()
    spawn(fn -> update_user.(parent,message) end)
    socket = receive do
      {:socket, socket} -> socket
    end

    x = if(message["client"] == "robot_A") do
      if(socket.assigns.robotA_start == 0) do
        0
      else
        x = socket.assigns.robotA_start
        [x]
      end
    else
      if(socket.assigns.robotB_start == 0) do
        0
      else
        x = socket.assigns.robotB_start
        [x]
      end
    end

    {:reply, {:ok, x}, socket}
  end

  def handle_in("event_msg", message, socket) do
    message = Map.put(message, "timer", socket.assigns[:timer_tick])
    Task4CPhoenixServerWeb.Endpoint.broadcast_from(self(), "robot:status", "event_msg", message)
    {:reply, {:ok, true}, socket}
  end

  def handle_in("time", message, socket) do
    val = kill_bots(socket.assigns[:timer_tick],message["sender"])
    {:reply, {:ok, {val,socket.assigns[:timer_tick]}}, socket}
  end

  def handle_in("sowing", message, socket) do
    IO.inspect(message)
    {:reply, {:ok, true}, socket}
  end

   def handle_in("weeding", message, socket) do
    {:reply, {:ok, true}, socket}
  end

   def handle_in("deposition", message, socket) do
    {:reply, {:ok, true}, socket}
  end

  #########################################
  ## define callback functions as needed ##
  #########################################

  def kill_bots(time,sender) do
    y = File.stream!("Robots_handle.csv")
      |> CSV.decode
      |> Enum.map(fn {_,x} ->
        x
      end)

    {_,y} = List.pop_at(y,0)
    # count = Enum.count(y)
    vals = Enum.map(y,fn [_a,b,c,d] ->
     %{bot: b, stop: c, start: d}
    end)
    vals = Enum.filter(vals, fn %{bot: _b, stop: c, start: _d} = _x -> 300-c < time end)
    val = Enum.find(vals, fn %{bot: b, stop: _c, start: _d} = _x -> b == sender end)
    IO.inspect(vals)
    IO.inspect(val)
    val
  end

  def handle_info(%{event: "update_timer_tick", payload: timer_data, topic: "timer:update"}, socket) do
    socket = assign(socket, :timer_tick, timer_data.time)
    {:noreply, socket}
  end

  def handle_info(%{robotA_start: a, robotB_start: b} = _data, socket) do

    resource_id = {User, {:id, 1}}
    update_user = fn(a,b) ->
      lock = Mutex.await(MyMutex, resource_id)
      {ax,ay,afacing,bx,by,bfacing,_a_start,_b_start,a_alive,b_alive} = GenServer.call(Positions, :pop)
      GenServer.cast(Positions,{:push, {ax,ay,afacing,bx,by,bfacing,a,b,a_alive,b_alive}})
      Mutex.release(MyMutex, lock)
    end
    spawn(fn -> update_user.(a,b) end)
    {:noreply, socket, 1000}
  end

  def handle_info(_data, socket) do

    {:noreply, socket}
  end

end
