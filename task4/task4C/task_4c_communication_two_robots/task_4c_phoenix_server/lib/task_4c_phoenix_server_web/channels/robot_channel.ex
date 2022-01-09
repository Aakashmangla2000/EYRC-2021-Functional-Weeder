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
    IO.puts("message #{inspect(message)} #{is_obs_ahead} #{inspect(socket)}")
    mp = %{"a" => 0, "b" => 1, "c" => 2, "d" => 3, "e" => 4, "f" => 5}

    msg2 = if(is_obs_ahead == false) do
      %{"client" => message["client"], "left" => (message["x"]-1)*150, "bottom" => Map.get(mp, message["y"])*150, "face" => message["face"]}
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
      %{"client" => "obs", "x" => x, "y" => y}
    end

    Phoenix.PubSub.subscribe(Task4CPhoenixServer.PubSub, "robot:update")
    Phoenix.PubSub.broadcast(Task4CPhoenixServer.PubSub, "robot:update", msg2)

    {:reply, {:ok, is_obs_ahead}, socket}
  end

  #########################################
  ## define callback functions as needed ##
  #########################################

end
