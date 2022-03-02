defmodule Task4CClientRobotA.ArmMechanismTest do
  @moduledoc """
  Documentation for `ArmMechanismTest'
  """

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

  @duty_cycles [70]
  @pwm_frequency 50

  def test_ir do
    # Logger.debug("Testing IR Proximity Sensors")
    ir_ref = Enum.map(@ir_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :input, pull_mode: :pullup) end)
    ir_values = Enum.map(ir_ref,fn {_, ref_no} -> GPIO.read(ref_no) end)
  end

  def seeding(motor_ref,robot,goal) do
    robot2 = robot
    {robot,dir} = find_plant(robot,goal,motor_ref)
    motor_action(motor_ref,@backward)
    pwm(100)
    if(robot == robot2) do
      Process.sleep(100)
    else
      IO.puts("boop")
      Process.sleep(200)
    end
    motor_action(motor_ref,@stop)
    Process.sleep(100)

    change_angle2(20)
    Process.sleep(1000)
    change_angle(140)

    motor_action(motor_ref,@forward)
    pwm(100)
    if(robot == robot2) do
      Process.sleep(50)
    else
      IO.puts("boop2")
      Process.sleep(150)
    end
    motor_action(motor_ref,@stop)
    Process.sleep(100)
    if(dir == 0) do
      Task4CClientRobotA.LineFollower.left(motor_ref,1)
    else
      Task4CClientRobotA.LineFollower.right(motor_ref,1)
    end
    robot
  end

  def change_angle(angle) do
    if(angle > 0) do
      test_servo_b(angle-4)
      change_angle(angle-4)
    else
    end
  end

  def change_angle2(angle) do
    if(angle < 140) do
      test_servo_b(angle+4)
      change_angle2(angle+4)
    else
    end
  end

  def find_plant(robot,goal,motor_ref) do
    p3 = %{:a => 0, :b => 1, :c => 2, :d => 3, :e => 4, :f => 5}
    Process.sleep(1000)
    goal = String.to_integer(goal)

    IO.puts("Find plant")
    IO.inspect(goal)
    IO.inspect(robot.x)
    IO.inspect(robot.y)
    IO.inspect(Map.get(p3,robot.y))

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
            robot = Task4CClientRobotA.right(robot,motor_ref)
            find_on_left(motor_ref,0)
            {robot,0}
          val == 2 ->
            robot = Task4CClientRobotA.left(robot,motor_ref)
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
            robot = Task4CClientRobotA.right(robot,motor_ref)
            find_on_left(motor_ref,0)
            {robot,0}
          val == 3 ->
            find_on_right(motor_ref,0)
            {robot,1}
          val == 4 ->
            robot = Task4CClientRobotA.left(robot,motor_ref)
            find_on_right(motor_ref,0)
            {robot,1}
        end
      robot.facing == :west ->
        cond do
          val == 1 ->
            robot = Task4CClientRobotA.left(robot,motor_ref)
            find_on_right(motor_ref,0)
            {robot,1}
          val == 2 ->
            find_on_right(motor_ref,0)
            {robot,1}
          val == 3 ->
            robot = Task4CClientRobotA.right(robot,motor_ref)
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
            robot = Task4CClientRobotA.left(robot,motor_ref)
            find_on_right(motor_ref,0)
            {robot,1}
          val == 4 ->
            robot = Task4CClientRobotA.right(robot,motor_ref)
            find_on_left(motor_ref,0)
            {robot,0}
        end
    end
    {robot,dir}
  end

  def find_on_left(motor_ref,count) do
    IO.puts("find on left")
    Process.sleep(200)
    count = count + 1
    [a,b] = test_ir()
    # IO.inspect("a: #{a} b: #{b} count #{count}")
    if(a == 1 or count < 2) do
      motor_action(motor_ref,@right)
      pwm(100)
      Process.sleep(10)
      motor_action(motor_ref,@stop)
      Process.sleep(100)
      find_on_left(motor_ref,count)
    else
      motor_action(motor_ref,@stop)
      Process.sleep(100)
    end
  end

  def find(val) do
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    if(val == 0) do
      find_on_right(motor_ref,0)
    else
      find_on_left(motor_ref,0)
    end
    motor_action(motor_ref,@backward)
    pwm(100)
    Process.sleep(10)
    motor_action(motor_ref,@stop)
    Process.sleep(100)
  end

  def find_on_right(motor_ref,count) do
    IO.puts("find on right")
    Process.sleep(200)
    count = count + 1
    [a,b] = test_ir()
    # IO.inspect("a: #{a} b: #{b} count #{count}")
    if(a == 1 or count < 2) do
      motor_action(motor_ref,@left)
      pwm(100)
      Process.sleep(50)
      motor_action(motor_ref,@stop)
      Process.sleep(100)
      find_on_right(motor_ref,count)
    else
      motor_action(motor_ref,@stop)
      Process.sleep(100)
    end
  end

  def rep() do
    [a,b] = test_ir()
    IO.inspect("a: #{a} b: #{b}")
    if(a == 1) do
      rep()
    else
      [a,b]
    end
  end

  def test_servo_a(angle) do
    val = trunc(((2.5 + 10.0 * angle / 180) / 100 ) * 255)
    Pigpiox.Pwm.set_pwm_frequency(@servo_a_pin, @pwm_frequency)
    Pigpiox.Pwm.gpio_pwm(@servo_a_pin, val)
  end

  def test_servo_b(angle) do
    val = trunc(((2.5 + 10.0 * angle / 180) / 100 ) * 255)
    Pigpiox.Pwm.set_pwm_frequency(@servo_b_pin, @pwm_frequency)
    Pigpiox.Pwm.gpio_pwm(@servo_b_pin, val)
  end

  defp motor_action(motor_ref,motion) do
    # IO.puts("a #{inspect(motor_ref)}")
    motor_ref |> Enum.zip(motion) |> Enum.each(fn {{_, ref_no}, value} -> GPIO.write(ref_no, value) end)
    pwm(100)
    Process.sleep(200)
  end

  defp pwm(duty) do
    Enum.each(@pwm_pins, fn {_atom, pin_no} -> Pigpiox.Pwm.gpio_pwm(pin_no, duty) end)
  end

end
