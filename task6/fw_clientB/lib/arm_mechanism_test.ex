defmodule Task4CClientRobotB.ArmMechanismTest do
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
  @leftback [0, 0, 0, 1]
  @rightback [1, 0, 0, 0]
  @leftgo [0, 0, 1, 0]
  @rightgo [0, 1, 0, 0]

  @duty_cycles [70]
  @pwm_frequency 50


  def test_motion do
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
    motion_list = [@forward,@forward,@forward,@forward,@forward,@forward,@stop]
    Enum.each(motion_list, fn motion -> motor_action(motor_ref,motion) end)

  end

  def test_ir do
    ir_ref = Enum.map(@ir_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :input, pull_mode: :pullup) end)
    ir_values = Enum.map(ir_ref,fn {_, ref_no} -> GPIO.read(ref_no) end)
    IO.inspect(ir_values)
  end

  def weeding do
    Process.sleep(5000)
    IO.puts("Opening the Claws...")
    test_servo_b(0)  #opening claws
    Process.sleep(1000)

    IO.puts("Positioning the arm...")
    test_servo_a(40)
    Process.sleep(500)
    test_servo_a(30)  #setting arm at height
    Process.sleep(500)
    test_servo_a(10)  #setting arm at height
    Process.sleep(500)

    IO.puts("Weeding Begins...")

    test_servo_b(20)
    Process.sleep(1000)
    test_servo_b(40)
    Process.sleep(1000)
    test_servo_b(60)
    Process.sleep(1000)
    test_servo_b(90)
    Process.sleep(1000)
    test_servo_a(50)
    Process.sleep(1000)

  end

   def deposition do
    IO.puts("Weed Depositing...")
    Process.sleep(1000)
    test_servo_a(30)
    Process.sleep(1000)
    test_servo_b(0)
    Process.sleep(1000)
    test_servo_a(40)
    IO.puts("Weeding Ends...")
   end


  # 1 - top left 2-top right 3- bottom left 4- bottom right

  def find_plant(robot,goal) do
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    p3 = %{:a => 0, :b => 1, :c => 2, :d => 3, :e => 4, :f => 5}
    Process.sleep(1000)

    val = cond do
      robot.x == 6 ->
        cond do
          goal == robot.x + 5*Map.get(p3,robot.y) - 1 -> 1
          goal == robot.x + 5*Map.get(p3,robot.y) - 6 -> 3
        end
      robot.y == :f ->
        cond do
          goal == robot.x + 5*Map.get(p3,robot.y) - 5 -> 4
          goal == robot.x + 5*Map.get(p3,robot.y) - 6 -> 3
        end
      robot.x == 6 and robot.y == :f ->
        cond do
          goal == robot.x + 5*Map.get(p3,robot.y) - 6 -> 3
        end
      true ->
        cond do
          goal == robot.x + 5*Map.get(p3,robot.y) - 1 -> 1
          goal == robot.x + 5*Map.get(p3,robot.y) ->     2
          goal == robot.x + 5*Map.get(p3,robot.y) - 6 -> 3
          goal == robot.x + 5*Map.get(p3,robot.y) - 5 -> 4
        end
    end

    cond do
      robot.facing == :north ->
        cond do
          val == 1 ->
            Task4CClientRobotB.LineFollower.right(motor_ref,0)
            find_on_left(motor_ref,0)
          val == 2 ->
            Task4CClientRobotB.LineFollower.left(motor_ref,0)
            find_on_right(motor_ref,0)
          val == 3 ->
            find_on_left(motor_ref,0)
          val == 4 ->
            find_on_right(motor_ref,0)
        end
      robot.facing == :east ->
        cond do
          val == 1 ->
            find_on_left(motor_ref,0)
          val == 2 ->
            Task4CClientRobotB.LineFollower.right(motor_ref,0)
            find_on_left(motor_ref,0)
          val == 3 ->
            find_on_right(motor_ref,0)
          val == 4 ->
            Task4CClientRobotB.LineFollower.left(motor_ref,0)
            find_on_right(motor_ref,0)
        end
      robot.facing == :west ->
        cond do
          val == 1 ->
            Task4CClientRobotB.LineFollower.left(motor_ref,0)
            find_on_right(motor_ref,0)
          val == 2 ->
            find_on_right(motor_ref,0)
          val == 3 ->
            Task4CClientRobotB.LineFollower.right(motor_ref,0)
            find_on_left(motor_ref,0)
          val == 4 ->
            find_on_left(motor_ref,0)
        end
      robot.facing == :south ->
        cond do
          val == 1 ->
            find_on_right(motor_ref,0)
          val == 2 ->
            find_on_left(motor_ref,0)
          val == 3 ->
            Task4CClientRobotB.LineFollower.left(motor_ref,0)
            find_on_right(motor_ref,0)
          val == 4 ->
            Task4CClientRobotB.LineFollower.right(motor_ref,0)
            find_on_left(motor_ref,0)
        end
    end


  end

  def find_on_left(motor_ref,count) do
    save = motor_ref
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    count = count + 1
    [a,b] = test_ir()
    IO.inspect("a: #{a} b: #{b} count #{count}")
    if(b == 1) do
      pwm(50)
      motor_action(motor_ref,@right)
      Process.sleep(50)
      motor_action(motor_ref,@stop)
      Process.sleep(100)
      find_on_left(save,count)
    else
      motor_action(motor_ref,@stop)
      Process.sleep(200)
      motor_action(motor_ref,@rightgo)
      Process.sleep(8)
      motor_action(motor_ref,@stop)
      Process.sleep(200)
      # motor_action(motor_ref,@forward)
      # Process.sleep(1)
      # motor_action(motor_ref,@stop)
      # Process.sleep(100)

    end
    # [a,b] = test_ir()
    # rep()
    motor_action(motor_ref,@stop)
    Process.sleep(50)
  end

  def find_on_right(motor_ref,count) do
    save = motor_ref
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    count = count + 1
    [a,b] = test_ir()
    IO.inspect("a: #{a} b: #{b} count #{count}")
    if(b == 1) do
      pwm(50)
      motor_action(motor_ref,@left)
      Process.sleep(50)
      motor_action(motor_ref,@stop)
      Process.sleep(100)
      find_on_right(save, count)
    else
      motor_action(motor_ref,@stop)
      Process.sleep(200)
      motor_action(motor_ref,@leftback)
      Process.sleep(8)
      motor_action(motor_ref,@stop)
      Process.sleep(200)
      motor_action(motor_ref,@forward)
      Process.sleep(1)
      motor_action(motor_ref,@stop)
      Process.sleep(100)


    end
    # [a,b] = test_ir()
    # rep()
    motor_action(motor_ref,@stop)
    Process.sleep(50)
  end

  def rep() do
    [a,b] = test_ir()
    IO.inspect("a: #{a} b: #{b}")
    if(b == 1) do
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
