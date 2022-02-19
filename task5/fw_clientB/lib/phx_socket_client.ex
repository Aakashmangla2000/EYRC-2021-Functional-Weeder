defmodule Task4CClientRobotB.PhoenixSocketClient do

  alias PhoenixClient.{Socket, Channel, Message}

  @doc """
  Connect to the Phoenix Server URL (defined in config.exs) via socket.
  Once ensured that socket is connected, join the channel on the server with topic "robot:status".
  Get the channel's PID in return after joining it.

  NOTE:
  The socket will automatically attempt to connect when it starts.
  If the socket becomes disconnected, it will attempt to reconnect automatically.
  Please note that start_link is not synchronous,
  so you must wait for the socket to become connected before attempting to join a channel.
  Reference to above note: https://github.com/mobileoverlord/phoenix_client#usage

  You may refer: https://github.com/mobileoverlord/phoenix_client/issues/29#issuecomment-660518498
  """
  def connect_server do

    ###########################
    ## complete this funcion ##
    ###########################

    url = Application.get_env(:task_4c_client_robotb, :phoenix_server_url)
    socket_opts = [
      url: url
    ]
    {:ok, socket} = PhoenixClient.Socket.start_link(socket_opts)
    wait_until_connected(socket)
    PhoenixClient.Channel.join(socket,"robot:status")

  end

   def wait_until_connected(socket) do
    if !PhoenixClient.Socket.connected?(socket) do
      Process.sleep(100)
      wait_until_connected(socket)
    end
  end

  @doc """
  Send Toy Robot's current status i.e. location (x, y) and facing
  to the channel's PID with topic "robot:status" on Phoenix Server with the event named "new_msg".

  The message to be sent should be a Map strictly of this format:
  %{"client": < "robot_A" or "robot_B" >,  "x": < x_coordinate >, "y": < y_coordinate >, "face": < facing_direction > }

  In return from Phoenix server, receive the boolean value < true OR false > indicating the obstacle's presence
  in this format: {:ok, < true OR false >}.
  Create a tuple of this format: '{:obstacle_presence, < true or false >}' as a return of this function.
  """
  def send_robot_status(channel, %Task4CClientRobotB.Position{x: x, y: y, facing: facing} = _robot) do

    ###########################
    ## complete this funcion ##
    ###########################
    # Process.sleep(1000)
    tup = PhoenixClient.Channel.push(channel,"new_msg",%{"client" => "robot_B","x" => x, "y" => y, "face" => facing},5000)
    {:ok, is_obs_ahead} = tup
    is_obs_ahead
  end

  ######################################################
  ## You may create extra helper functions as needed. ##
  ######################################################

  def get_start(channel) do
    tup = PhoenixClient.Channel.push(channel,"start_pos",%{"client" => "robot_B"},5000)
    {:ok, start} = tup
    start
  end

  def get_bot_position(bool,channel,robot) do
    %Task4CClientRobotB.Position{facing: facing,x: x, y: y} = robot
    tup = PhoenixClient.Channel.push(channel,"get_bots",%{"client" => "robot_B","x" => x, "y" => y, "face" => facing, "alive" => bool},5000)
    {:ok, rep} = tup
    [ax,ay,afacing,_bx,_by,_bfacing,a_alive,_b_alive] = rep
    [ax,String.to_atom(ay),String.to_atom(afacing),a_alive]
  end

  def get_goals(channel) do
    tup = PhoenixClient.Channel.push(channel,"goals",%{"client" => "robot_B"},5000)
    {:ok, weed} = tup
    weed
  end

end
