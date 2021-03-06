defmodule Task4CPhoenixServerWeb.ArenaLive do
  use Task4CPhoenixServerWeb,:live_view
  require Logger

  @doc """
  Mount the Dashboard when this module is called with request
  for the Arena view from the client like browser.
  Subscribe to the "robot:update" topic using Endpoint.
  Subscribe to the "timer:update" topic as PubSub.
  Assign default values to the variables which will be updated
  when new data arrives from the RobotChannel module.
  """
  def mount(_params, _session, socket) do

    Task4CPhoenixServerWeb.Endpoint.subscribe("robot:update")
    :ok = Phoenix.PubSub.subscribe(Task4CPhoenixServer.PubSub, "timer:update")

    socket = assign(socket, :img_robotA, "robot_facing_north_a.png")
    socket = assign(socket, :bottom_robotA, 0)
    socket = assign(socket, :left_robotA, 0)
    socket = assign(socket, :robotA_start, "")
    socket = assign(socket, :robotA_goals, [])
    socket = assign(socket, :seeds, [])
    socket = assign(socket, :depos, [])

    socket = assign(socket, :img_robotB, "robot_facing_south_b.png")
    socket = assign(socket, :bottom_robotB, 750)
    socket = assign(socket, :left_robotB, 750)
    socket = assign(socket, :robotB_start, "")
    socket = assign(socket, :robotB_goals, [])
    socket = assign(socket, :weeds, [])

    socket = assign(socket, :obstacle_pos, MapSet.new())
    socket = assign(socket, :timer_tick, 300)

    {:ok,socket}

  end

  @doc """
  Render the Grid with the coordinates and robot's location based
  on the "img_robotA" or "img_robotB" variable assigned in the mount/3 function.
  This function will be dynamically called when there is a change
  in the values of any of these variables =>
  "img_robotA", "bottom_robotA", "left_robotA", "robotA_start", "robotA_goals",
  "img_robotB", "bottom_robotB", "left_robotB", "robotB_start", "robotB_goals",
  "obstacle_pos", "timer_tick"
  """
  def render(assigns) do

    ~H"""
    <div id="dashboard-container">

    <div id="deposition2">
          <div> Deposition Zone </div>
        </div>
        <br><br>
      <div class="grid-container">
       <div id="deposition1">
          <div> Deposition Zone </div>
        </div>

        <div id="alphabets">
          <div> A </div>
          <div> B </div>
          <div> C </div>
          <div> D </div>
          <div> E </div>
          <div> F </div>
        </div>

        <div class="board-container">
          <div class="game-board">
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
            <div class="box"></div>
          </div>

          <%= for obs <- @obstacle_pos do %>
            <img  class="obstacles"  src="/images/stone.png" width="50px" style={"bottom: #{elem(obs,1)}px; left: #{elem(obs,0)}px"}>
          <% end %>

          <div class="robot-container" style={"bottom: #{@bottom_robotA}px; left: #{@left_robotA}px"}>
            <img id="robotA" src={"/images/#{@img_robotA}"} style="height:70px;">
          </div>

          <div class="robot-container" style={"bottom: #{@bottom_robotB}px; left: #{@left_robotB}px"}>
            <img id="robotB" src={"/images/#{@img_robotB}"} style="height:70px;">
          </div>

          <%= for i <- @depos do %>
            <div class="plant2" style={"bottom: 800px; left: #{80+((Enum.find_index(@depos, fn x -> x == i end))+1)*150}px"}>
              <img id={"pl#{i}"} src={"/images/weed.jpeg"} style="height:50px;">
            </div>
          <% end %>

          <%= for i <- 0..4 do %>
            <div class="plant" style={"bottom: 660px; left: #{80+i*150}px"}>
              <img id={"plant_1_#{i}"} src={"/images/plant.png"} style="height:71px;">
            </div>
          <% end %>

          <%= for i <- 0..4 do %>
            <div class="plant" style={"bottom: 510px; left: #{80+i*150}px"}>
              <img id={"plant_2_#{i}"} src={"/images/plant.png"} style="height:71px;">
            </div>
          <% end %>

          <%= for i <- 0..4 do %>
            <div class="plant" style={"bottom: 360px; left: #{80+i*150}px"}>
              <img id={"plant_3_#{i}"} src={"/images/plant.png"} style="height:71px;">
            </div>
          <% end %>

          <%= for i <- 0..4 do %>
            <div class="plant" style={"bottom: 210px; left: #{80+i*150}px"}>
              <img id={"plant_4_#{i}"} src={"/images/plant.png"} style="height:71px;">
            </div>
          <% end %>

          <%= for i <- 0..4 do %>
            <div class="plant" style={"bottom: 60px; left: #{80+i*150}px"}>
              <img id={"plant_5_#{i}"} src={"/images/plant.png"} style="height:71px;">
            </div>
          <% end %>

          <%= for i <- @robotA_goals do %>
              <div class="plant" style={"bottom: #{(div(String.to_integer(i)-1,5)*150)+60}px; left: #{(rem(String.to_integer(i)+4,5))*150+80}px"}>
              <img id={"pot_#{i}"} src={"/images/pot.png"} style="height:70px;">
            </div>
          <% end %>

          <%= for i <- @seeds do %>
              <div class="seed" style={"bottom: #{(div(i-1,5)*150)+60}px; left: #{(rem(i+5,5)-1)*150+80}px"}>
              <img id={"seed_#{i}"} src={"/images/seed.png"} style="height:80px;">
            </div>
          <% end %>

          <%= for i <- @robotB_goals do %>
              <div class="plant" style={"bottom: #{(div(String.to_integer(i)-1,5)*150)+60}px; left: #{(rem(String.to_integer(i)+4,5))*150+80}px"}>
              <img id={"weed_#{i}"} src={"/images/weed.jpeg"} style="height:70px;">
            </div>
          <% end %>

          <%= for i <- @weeds do %>
              <div class="plant" style={"bottom: #{(div(i-1,5)*150)+60}px; left: #{(rem(i+4,5))*150+80}px"}>
              <img id={"soil_#{i}"} src={"/images/soil.png"} style="height:70px;">
            </div>
          <% end %>

        </div>

        <div id="numbers">
          <div> 1 </div>
          <div> 2 </div>
          <div> 3 </div>
          <div> 4 </div>
          <div> 5 </div>
          <div> 6 </div>
        </div>

      </div>
      <div id="right-container">

        <div class="timer-card">
          <label style="text-transform:uppercase;width:100%;font-weight:bold;text-align:center" >Timer</label>
            <p id="timer" ><%= @timer_tick %></p>
        </div>

        <div class="goal-card">
          <div style="text-transform:uppercase;width:100%;font-weight:bold;text-align:center" > Goal positions </div>
          <div style="display:flex;flex-flow:wrap;width:100%">
            <div style="width:50%">
              <label>Robot A</label>
              <%= for i <- @robotA_goals do %>
                <div><%= i %></div>
              <% end %>
            </div>
            <div  style="width:50%">
              <label>Robot B</label>
              <%= for i <- @robotB_goals do %>
              <div><%= i %></div>
              <% end %>
            </div>
          </div>
        </div>

        <div class="position-card">
          <div style="text-transform:uppercase;width:100%;font-weight:bold;text-align:center"> Start Positions </div>
          <form phx-submit="start_clock" style="width:100%;display:flex;flex-flow:row wrap;">
            <div style="width:100%;padding:10px">
              <label>Robot A</label>
              <input name="robotA_start" style="background-color:white;" value={"#{@robotA_start}"}>
            </div>
            <div style="width:100%; padding:10px">
              <label>Robot B</label>
              <input name="robotB_start" style="background-color:white;" value={"#{@robotB_start}"}>
            </div>

            <button  id="start-btn" type="submit">
              <svg xmlns="http://www.w3.org/2000/svg" style="height:30px;width:30px;margin:auto" viewBox="0 0 20 20" fill="currentColor">
                <path fill-rule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM9.555 7.168A1 1 0 008 8v4a1 1 0 001.555.832l3-2a1 1 0 000-1.664l-3-2z" clip-rule="evenodd" />
              </svg>
            </button>

            <button phx-click="stop_clock" id="stop-btn" type="button">
              <svg xmlns="http://www.w3.org/2000/svg" style="height:30px;width:30px;margin:auto" viewBox="0 0 20 20" fill="currentColor">
                <path fill-rule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8 7a1 1 0 00-1 1v4a1 1 0 001 1h4a1 1 0 001-1V8a1 1 0 00-1-1H8z" clip-rule="evenodd" />
              </svg>
            </button>
          </form>
        </div>

      </div>

    </div>
    """

  end

  @doc """
  Handle the event "start_clock" triggered by clicking
  the PLAY button on the dashboard.
  """
  def handle_event("start_clock", data, socket) do
    socket = assign(socket, :robotA_start, data["robotA_start"])
    socket = assign(socket, :robotB_start, data["robotB_start"])
    Task4CPhoenixServerWeb.Endpoint.broadcast("timer:start", "start_timer", %{})
    #################################
    ## edit the function if needed ##
    #################################
    Phoenix.PubSub.broadcast!(Task4CPhoenixServer.PubSub, "robot:update", %{robotA_start: data["robotA_start"],robotB_start: data["robotB_start"]})

    {:noreply, socket}

  end

  @doc """
  Handle the event "stop_clock" triggered by clicking
  the STOP button on the dashboard.
  """
  def handle_event("stop_clock", _data, socket) do

    Task4CPhoenixServerWeb.Endpoint.broadcast("timer:stop", "stop_timer", %{})

    #################################
    ## edit the function if needed ##
    #################################

    {:noreply, socket}

  end

  @doc """
  Callback function to handle incoming data from the Timer module
  broadcasted on the "timer:update" topic.
  Assign the value to variable "timer_tick" for each countdown.
  """
  def handle_info(%{event: "update_timer_tick", payload: timer_data, topic: "timer:update"}, socket) do

    Logger.info("Timer tick: #{timer_data.time}")
    socket = assign(socket, :timer_tick, timer_data.time)

    {:noreply, socket}

  end

  @doc """
  Handle info functions for sowing weeding and deposition to depict them on Live View
  """

  def handle_info(%{seed: sow} = _data, socket) do
    socket = assign(socket, :seeds,socket.assigns[:seeds]++[sow])
    IO.inspect(socket.assigns[:seeds])
    {:noreply, socket}
  end

  def handle_info(%{wee: weed} = _data, socket) do
    socket = assign(socket, :weeds,socket.assigns[:weeds]++[weed])
    IO.inspect(socket.assigns[:weeds])
    {:noreply, socket}
  end

  def handle_info(%{depos: depos} = _data, socket) do
    socket = assign(socket, :depos,depos)
    IO.inspect(socket.assigns[:depos])
    {:noreply, socket}
  end

  @doc """
  This assigns the goals decided by both the robots and shows them on live view
  """

  def handle_info(%{sow: sow, weed: weed} = _data, socket) do
    socket = if(sow != nil) do
        assign(socket, :robotA_goals,sow)
      else
        socket
      end
    socket = if(weed != nil) do
        assign(socket, :robotB_goals,weed)
      else
        socket
      end
    {:noreply, socket}
  end

  @doc """
  Callback function to handle any incoming data from the RobotChannel module
  broadcasted on the "robot:update" topic.
  Assign the values to the variables => "img_robotA", "bottom_robotA", "left_robotA",
  "img_robotB", "bottom_robotB", "left_robotB" and "obstacle_pos" as received.
  Make sure to add a tuple of format: { < obstacle_x >, < obstacle_y > } to the MapSet object "obstacle_pos".
  These values msut be in pixels. You may handle these variables in separate callback functions as well.
  """
  def handle_info(data, socket) do

    ###########################
    ## complete this funcion ##
    ###########################

    socket = if(data["obs"] == true) do
      assign(socket, :obstacle_pos,MapSet.put(socket.assigns.obstacle_pos,{data["x"],data["y"]}))
    else
      assign(socket, :obstacle_pos,socket.assigns.obstacle_pos)
    end
    socket = if(data["client"] == "robot_A") do
      facing = data["face"]
      socket = cond do
        facing == "north" ->
          assign(socket, :img_robotA, "robot_facing_north_a.png")
        facing == "south" ->
          assign(socket, :img_robotA, "robot_facing_south_a.png")
        facing == "east" ->
          assign(socket, :img_robotA, "robot_facing_east_a.png")
        facing == "west" ->
          assign(socket, :img_robotA, "robot_facing_west_a.png")
        true ->
          assign(socket, :img_robotA, "robot_facing_west_a.png")
      end
      socket = assign(socket, :bottom_robotA,data["bottom"])
      assign(socket, :left_robotA,data["left"])

    else
      facing = data["face"]
      socket = cond do
        facing == "north" ->
          assign(socket, :img_robotB, "robot_facing_north_b.png")
        facing == "south" ->
          assign(socket, :img_robotB, "robot_facing_south_b.png")
        facing == "east" ->
          assign(socket, :img_robotB, "robot_facing_east_b.png")
        facing == "west" ->
          assign(socket, :img_robotB, "robot_facing_west_b.png")
        true ->
          assign(socket, :img_robotB, "robot_facing_west_b.png")
      end
      socket = assign(socket, :bottom_robotB,data["bottom"])
      assign(socket, :left_robotB,data["left"])
    end

    {:noreply, socket}

  end

  ######################################################
  ## You may create extra helper functions as needed  ##
  ## and update remaining assign variables.           ##
  ######################################################

end
