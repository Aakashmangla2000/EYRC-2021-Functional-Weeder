defmodule Task4CClientRobotB.ArmMechanismTest do
  @moduledoc """
  This module implements Weeding the plant functionality for the weeder robot.
  """

  ## importing necessary libraries and setting pins
  require Logger
  use Bitwise
  alias Circuits.GPIO

  @sensor_pins [cs: 5, clock: 25, address: 24, dataout: 23]
  @ir_pins [dr: 16, dl: 19]
  @motor_pins [lf: 12, lb: 13, rf: 20, rb: 21]
  @pwm_pins [enl: 6, enr: 26]
  @servo_a_pin 27
  @servo_b_pin 22

  @ref_atoms [:cs, :clock, :address, :dataout]
  @lf_sensor_data %{sensor0: 0, sensor1: 0, sensor2: 0, sensor3: 0, sensor4: 0, sensor5: 0}
  @lf_sensor_map %{0 => :sensor0, 1 => :sensor1, 2 => :sensor2, 3 => :sensor3, 4 => :sensor4, 5 => :sensor5}

  @forward [0, 1, 1, 0]
  @backward [1, 0, 0, 1]
  @left [0, 1, 0, 1]
  @right [1, 0, 1, 0]
  @stop [0, 0, 0, 0]
  @leftback [0, 0, 0, 1]
  @rightback [1, 0, 0, 0]
  @leftgo [0, 0, 1, 0]
  @rightgo [0, 1, 0, 0]

  @duty_cycles [70]
  @pwm_frequency 50

  @doc """
  test_ir function reads and returns the ir sensor values detecting the obstacles or the plants.
  The presence of an object is indicated by 0 and the absence is indicated by 1.
  """
  def test_ir do
    ir_ref = Enum.map(@ir_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :input, pull_mode: :pullup) end)
    ir_values = Enum.map(ir_ref,fn {_, ref_no} -> GPIO.read(ref_no) end)
  end

  @doc """
  weeding function calls the find_plant function defined below.
  Once the plant is located and the robot is positioned accordingly,
  the weeding is initiated.
  """
  def weeding(channel,motor_ref,robot, goal) do
    {robot,dir} = find_plant(channel,robot,goal,motor_ref) #finding the plant and setting the bot position

    Process.sleep(500)
    IO.puts("Opening the Claws...")
    test_servo_b(0)  #opening claws
    Process.sleep(500)

    IO.puts("Positioning the arm...")
    test_servo_a(40)
    Process.sleep(500)
    test_servo_a(30)  #setting arm at height
    Process.sleep(500)
    test_servo_a(10)  #setting arm at height
    Process.sleep(500)

    IO.puts("Weeding Begins...")
    test_servo_b(20)
    Process.sleep(500)
    test_servo_b(40)
    Process.sleep(500)
    test_servo_b(60)
    Process.sleep(500)
    test_servo_b(90)
    Process.sleep(500)
    test_servo_a(50)
    Process.sleep(500)

    motor_action(motor_ref,@backward)
    pwm(50)
    Process.sleep(100)
    motor_action(motor_ref,@stop)
    Process.sleep(100)

    #This condition sets the robot back to its original position on the line,once the weed is picked up
    if(dir == 0) do
      Task4CClientRobotB.LineFollower.left(channel,motor_ref,1)
    else
      Task4CClientRobotB.LineFollower.right(channel,motor_ref,1)
    end
    robot
  end

  @doc """
  deposition function implements dropping of weed at the deposition zones.
  Once the robot reaches to a node near the deposition zone, the facing of the robot is checked.
  Now, based on the facing, it is rotated such that the backside of the robot with the weeder arm is just above the zone.
  Finally, the weed is dropped and the weeding is completed.
  """
   def deposition(channel,motor_ref,robot, goal) do
    #setting up the robot position just over the deposition zone
    robot = cond do
      robot.x == 6 and robot.y == :f ->
        cond do
          robot.facing == :north ->
            robot = Task4CClientRobotB.left(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            robot
          robot.facing == :south ->
            robot
          robot.facing == :east ->
            robot = Task4CClientRobotB.right(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            robot
          robot.facing == :west ->
            robot
        end
      robot.y == :f ->
        cond do
          robot.facing == :north ->
            robot = Task4CClientRobotB.right(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            robot = Task4CClientRobotB.right(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            robot
          robot.facing == :south ->
            robot
          robot.facing == :east ->
            robot = Task4CClientRobotB.right(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            robot
          robot.facing == :west ->
            robot = Task4CClientRobotB.left(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            robot
        end
      robot.x == 6 ->
        cond do
          robot.facing == :east ->
            robot = Task4CClientRobotB.right(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            robot = Task4CClientRobotB.right(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            robot
          robot.facing == :west ->
            robot
          robot.facing == :north ->
            robot = Task4CClientRobotB.left(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            robot
          robot.facing == :south ->
            robot = Task4CClientRobotB.right(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            robot
        end
    end
    IO.puts("Weed Depositing...")
    Process.sleep(200)
    test_servo_a(50)
    Process.sleep(500)
    test_servo_a(30)
    Process.sleep(500)
    test_servo_b(0)
    Process.sleep(500)
    test_servo_a(60)
    IO.puts("Weeding Ends...")
    robot
   end

  @doc """
  find_plant function finds the plant to be weeded and position the robot for weeding using
  find_on_right and find_on_left functions. This function checks, if the goal plant is at the top, bottom,
  right or left of the plant on basis of the robot current position and facing. Once this is done the robot's backside
  is positioned just in front of the plant.
  """
  def find_plant(channel,robot,goal,motor_ref) do
    p3 = %{:a => 0, :b => 1, :c => 2, :d => 3, :e => 4, :f => 5}
    Process.sleep(1000)
    goal = String.to_integer(goal)

    IO.puts("Find plant")
    val = cond do
      robot.x == 6 ->
        cond do
          goal == (robot.x + (5*Map.get(p3,robot.y)) - 1) -> 1
          goal == (robot.x + (5*Map.get(p3,robot.y)) - 6) -> 3
        end
      robot.y == :f ->
        cond do
          goal == (robot.x + (5*Map.get(p3,robot.y)) - 5) -> 4
          goal == (robot.x + (5*Map.get(p3,robot.y)) - 6) -> 3
        end
      robot.x == 6 and robot.y == :f ->
        cond do
          goal == (robot.x + (5*Map.get(p3,robot.y)) - 6) -> 3
        end
      true ->
        cond do
          goal == (robot.x + (5*Map.get(p3,robot.y)) - 1) -> 1
          goal == (robot.x + (5*Map.get(p3,robot.y)) ) ->     2
          goal == (robot.x + (5*Map.get(p3,robot.y)) - 6) -> 3
          goal == (robot.x + (5*Map.get(p3,robot.y)) - 5) -> 4
        end
    end

    {robot,dir} = cond do
      robot.facing == :north ->
        cond do
          val == 1 ->
            robot = Task4CClientRobotB.right(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            find_on_left(motor_ref,0)
            {robot,0}
          val == 2 ->
            robot = Task4CClientRobotB.left(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            find_on_right(motor_ref,0)
            {robot,1}
          val == 3 ->
            find_on_left(motor_ref,0)
            {robot,0}
          val == 4 ->
            find_on_right(motor_ref,0)
            {robot,1}
        end
      robot.facing == :east ->
        cond do
          val == 1 ->
            find_on_left(motor_ref,0)
            {robot,0}
          val == 2 ->
            robot = Task4CClientRobotB.right(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            find_on_left(motor_ref,0)
            {robot,0}
          val == 3 ->
            find_on_right(motor_ref,0)
            {robot,1}
          val == 4 ->
            robot = Task4CClientRobotB.left(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            find_on_right(motor_ref,0)
            {robot,1}
        end
      robot.facing == :west ->
        cond do
          val == 1 ->
            robot = Task4CClientRobotB.left(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            find_on_right(motor_ref,0)
            {robot,1}
          val == 2 ->
            find_on_right(motor_ref,0)
            {robot,1}
          val == 3 ->
            robot = Task4CClientRobotB.right(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            find_on_left(motor_ref,0)
            {robot,0}
          val == 4 ->
            find_on_left(motor_ref,0)
            {robot,0}
        end
      robot.facing == :south ->
        cond do
          val == 1 ->
            find_on_right(motor_ref,0)
            {robot,1}
          val == 2 ->
            find_on_left(motor_ref,0)
            {robot,0}
          val == 3 ->
            robot = Task4CClientRobotB.left(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            find_on_right(motor_ref,0)
            {robot,1}
          val == 4 ->
            robot = Task4CClientRobotB.right(channel,robot,motor_ref)
            Task4CClientRobotB.PhoenixSocketClient.send_robot_status(channel,robot)
            find_on_left(motor_ref,0)
            {robot,0}
        end
    end
    {robot,dir}
  end

  @doc """
  find_on_left rotates the backside of the robot to the left i.e.
  the robot to the right to find the plant on the left and
  accordingly position the arm mechanism just above the plant.
  """
  def find_on_left(motor_ref,count) do
    IO.puts("find on left")
    Process.sleep(300)
    count = count + 1
    [a,b] = test_ir()
    IO.inspect("a: #{a} b: #{b} count #{count}")

    # If ir sensor at the arm indicates 1 then the bot will rotate further,else it will stop and position the arm
    if(b == 1) do
      pwm(50)
      motor_action(motor_ref,@right)
      Process.sleep(1)
      motor_action(motor_ref,@stop)
      Process.sleep(200)
      find_on_left(motor_ref,count)
    else
      motor_action(motor_ref,@stop)
      Process.sleep(200)
      motor_action(motor_ref,@leftback)
      pwm(50)
      Process.sleep(70)
      motor_action(motor_ref,@stop)
      Process.sleep(50)
      motor_action(motor_ref,@forward)
      pwm(50)
      Process.sleep(120)
      motor_action(motor_ref,@stop)
      Process.sleep(100)
    end
  end

  @doc """
  find_on_right rotates the backside of the robot to the right i.e.
  the robot to the left to find the plant on the right and
  accordingly position the arm mechanism just above the plant.
  """
  def find_on_right(motor_ref,count) do
    IO.puts("find on right")
    Process.sleep(200)
    count = count + 1
    [a,b] = test_ir()
    IO.inspect("a: #{a} b: #{b} count #{count}")

    # If ir sensor at the arm indicates 1 then the bot will rotate further,else it will stop and position the arm
    if(b == 1) do
      motor_action(motor_ref,@left)
      pwm(50)
      Process.sleep(10)
      motor_action(motor_ref,@stop)
      Process.sleep(200)
      find_on_right(motor_ref,count)
    else

      motor_action(motor_ref,@stop)
      Process.sleep(100)
      motor_action(motor_ref,@leftback)
      pwm(50)
      Process.sleep(50)
      motor_action(motor_ref,@stop)
      Process.sleep(50)
      motor_action(motor_ref,@forward)
      pwm(50)
      Process.sleep(150)
      motor_action(motor_ref,@stop)
      Process.sleep(100)
    end
  end

  @doc """
  test_servo_a sets the angle for the servo controlling up and down
  movement of the arm.
  """
  def test_servo_a(angle) do
    val = trunc(((2.5 + 10.0 * angle / 180) / 100 ) * 255)
    Pigpiox.Pwm.set_pwm_frequency(@servo_a_pin, @pwm_frequency)
    Pigpiox.Pwm.gpio_pwm(@servo_a_pin, val)
  end

  @doc """
  test_servo_b sets the angle for the servo controlling opening and
  closing of the claws.
  """
  def test_servo_b(angle) do
    val = trunc(((2.5 + 10.0 * angle / 180) / 100 ) * 255)
    Pigpiox.Pwm.set_pwm_frequency(@servo_b_pin, @pwm_frequency)
    Pigpiox.Pwm.gpio_pwm(@servo_b_pin, val)
  end

  @doc """
  Function defining pin values for various movements.
  """
  defp motor_action(motor_ref,motion) do
    motor_ref |> Enum.zip(motion) |> Enum.each(fn {{_, ref_no}, value} -> GPIO.write(ref_no, value) end)
    pwm(100)
    Process.sleep(200)
  end

  @doc """
  Function defining pwm values to motor pins
  Note: "duty" variable can take value from 0 to 255. Value 255 indicates 100% duty cycle

  """
  defp pwm(duty) do
    Enum.each(@pwm_pins, fn {_atom, pin_no} -> Pigpiox.Pwm.gpio_pwm(pin_no, duty) end)
  end

end
