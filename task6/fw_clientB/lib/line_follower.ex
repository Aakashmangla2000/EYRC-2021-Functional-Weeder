defmodule Task4CClientRobotB.LineFollower do
  @moduledoc """
  Documentation for `LineFollower`.
  """

  require Logger
  use Bitwise
  alias Integer
  alias Decimal
  alias Circuits.GPIO

  @sensor_pins [cs: 5, clock: 25, address: 24, dataout: 23]
  @motor_pins [lf: 12, lb: 13, rf: 20, rb: 21]
  @pwm_pins [enl: 6, enr: 26]
  @ir_pins [dr: 16, dl: 19]

  @ref_atoms [:cs, :clock, :address, :dataout]
  @lf_sensor_data %{sensor0: 0, sensor1: 0, sensor2: 0, sensor3: 0, sensor4: 0, sensor5: 0}
  @lf_sensor_map %{0 => :sensor0, 1 => :sensor1, 2 => :sensor2, 3 => :sensor3, 4 => :sensor4, 5 => :sensor5}

  @left [1, 0, 1, 0]
  @right [0, 1, 0, 1]
  @forward [0, 1, 1, 0]
  @backward [1, 0, 0, 1]
  @stop [0, 0, 0, 0]

  @duty_cycles [150, 70, 0]
  @pwm_frequency 50

  @doc """
  Tests white line sensor modules reading

  Example:

      iex> FW_DEMO.test_wlf_sensors
      [0, 958, 851, 969, 975, 943]  // on white surface
      [0, 449, 356, 312, 321, 267]  // on black surface
  """
  def test_wlf_sensors do
    # Logger.debug("Testing white line sensors connected ")
    sensor_ref = Enum.map(@sensor_pins, fn {atom, pin_no} -> configure_sensor({atom, pin_no}) end)
    sensor_ref = Enum.map(sensor_ref, fn{_atom, ref_id} -> ref_id end)
    sensor_ref = Enum.zip(@ref_atoms, sensor_ref)
    get_lfa_readings([0,1,2,3,4], sensor_ref)
  end

  def test_ir do
    ir_ref = Enum.map(@ir_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :input, pull_mode: :pullup) end)
    ir_values = Enum.map(ir_ref,fn {_, ref_no} -> GPIO.read(ref_no) end)
    # IO.inspect(ir_values)
  end

  def ir_sensors do
    proximity = test_ir()
    front  =  Enum.at(proximity, 0)
    back = Enum.at(proximity, 1)
    # IO.puts("front: #{front}")
    # IO.puts("back: #{back}")
    [front,back]
  end

  def obs_detect do
    obs = false
    [front,back] = ir_sensors()
    # IO.inspect(ir_sensors())

    obs = if front == 0 do
      true
    else
      false
    end
    obs
  end

  def open_motor_pwm_pins() do
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
    {motor_ref,pwm_ref}
  end


  def pid(channel,motor_ref) do
    # Task4CClientRobotB.PhoenixSocketClient.timer(channel)
    Logger.debug("Going Forward")
    Process.sleep(100)
    # {motor_ref,pwm_ref} = open_motor_pwm_pins()

    maximum = 100;
    integral = 0;
    last_proportional = 0
    motor_action(motor_ref,@stop)
    Task4CClientRobotB.PhoenixSocketClient.timer(channel)
    pwm(10)
    # motor_action(motor_ref,@forward)
    Process.sleep(5)

    count = 1
    nodes = 1
    stop = 0
    proportional = 0
    x = 1
    y = 1
    filter = 1

    sensor_vals = test_wlf_sensors()
    [s1,s2,s3,s4,s5] = set_vals(sensor_vals)
    # IO.inspect(sensor_vals)
    # IO.inspect(set_vals(sensor_vals))

    if(s1 == 0 and s2 == 0 and s3 == 0 and s4 == 0 and s5 == 0) do
      find_line(motor_ref)
    end
    motor_action(motor_ref,@forward)
    forward(channel,count,filter,nodes,stop,motor_ref,maximum,integral,last_proportional)
    Task4CClientRobotB.PhoenixSocketClient.timer(channel)
  end


  def right(channel,motor_ref,count) do
    Task4CClientRobotB.PhoenixSocketClient.timer(channel)
    IO.puts("going right")
    count = count + 1
    IO.inspect(count)
    sensor_vals = test_wlf_sensors()
    [s1,s2,s3,s4,s5] = set_vals(sensor_vals)
    IO.inspect(sensor_vals)
    IO.inspect(set_vals(sensor_vals))
    pwm(120)
    Process.sleep(100)
    motor_action(motor_ref,@left)
    Process.sleep(100)
    motor_action(motor_ref,@stop)
    Process.sleep(100)
    # if(count > 5 and (s1 == 0 or s2 == 0 or s3 == 1 or s4 == 1 or s5 == 0)) do
    # if(s1 == 0 and s2 == 0 and s3 == 0 and s4 == 1 and s5 == 0) do
    # if(count > 4 and (s1 == 0 and s2 == 0 and s3 == 1 and s4 == 1 or s5 == 1)) do
    if(count > 2 and (s1 == 1 or s2 == 1 or s3 == 1 or s4 == 1 or s5 == 1)) do
    else
      right(channel,motor_ref,count)
    end
  end

  def left(channel,motor_ref,count) do
    Task4CClientRobotB.PhoenixSocketClient.timer(channel)
    IO.puts("going left")
    count = count + 1
    IO.inspect(count)
    sensor_vals = test_wlf_sensors()
    [s1,s2,s3,s4,s5] = set_vals(sensor_vals)
    IO.inspect(sensor_vals)
    IO.inspect(set_vals(sensor_vals))
    pwm(120)
    Process.sleep(100)
    motor_action(motor_ref,@right)
    Process.sleep(100)
    motor_action(motor_ref,@stop)
    Process.sleep(100)
    # if(count > 4 and (s1 == 1 or s2 == 1 or s3 == 1 and s4 == 0 and s5 == 0)) do
    if(count > 2 and (s1 == 1 or s2 == 1 or s3 == 1 or s4 == 1 or s5 == 1)) do
    else
      left(channel,motor_ref,count)
    end
  end


  def twice(motor_ref) do
    pwm(150)
    Process.sleep(230)
    motor_action(motor_ref,@left)
    Process.sleep(500)
    motor_action(motor_ref,@stop)
    Process.sleep(500)
  end

  def set_motors(_motor_ref,r,l) do
    pwml(l)
    pwmr(r)
  end

  def loop(j,i,sensor_vals) when j < 5 do
    sensor_vals = List.replace_at(sensor_vals,j,1000*i*Enum.at(sensor_vals,j))
    loop(j+1,i-1,sensor_vals)
  end

  def loop(_j,_i,sensor_vals) do
    sensor_vals
  end

  def read_line2(sensor_vals) do
    sensor_vals = set_vals(sensor_vals)
    denominator = Enum.sum(sensor_vals)
    i = 4
    sensor_vals = loop(0,i,sensor_vals)
    sum = Enum.sum(sensor_vals)
    if(denominator != 0) do
      Kernel.div(sum,denominator)
    else
      0
    end
  end

  def forward(channel,count,filter,nodes,stop,motor_ref,maximum,integral,last_proportional) when stop == 0 do
    # Task4CClientRobotB.PhoenixSocketClient.timer(channel)
    IO.puts("count: #{count}")
    count = count + 1
    IO.puts("nodes: #{nodes}")
    filter = filter + 1

    #Simple ReadLine
    sensor_vals = test_wlf_sensors()
    position = read_line2(sensor_vals)
    IO.inspect(sensor_vals)

    [s1,s2,s3,s4,s5] = set_vals(sensor_vals)

    IO.inspect(set_vals(sensor_vals))


    if(filter > 6) do
      motor_action(motor_ref,@forward)
      filter = 1
    else
      motor_action(motor_ref,@stop)
    end

    # [s1,s2,s3,s4,s5] = if(s1 == 0 and s2 == 0 and s3 == 0 and s4 == 0 and s5 == 0) do
    #   [0,0,1,0,0]
    # else
    #   [s1,s1,s3,s4,s5]
    # end

    proportional = position - 2000

		# Compute the derivative (change) and integral (sum) of the position.
		derivative = proportional - last_proportional
		integral = integral + proportional

		# Remember the last position.
		last_proportional = proportional

		power_difference = proportional*0.015 + derivative*0.020 #+ integral*0.005;
    power_difference = Kernel.round(power_difference)
    # IO.puts("Power Difference: #{power_difference}")
		power_difference = if (power_difference > maximum) do
			maximum
    else
      power_difference
    end

		power_difference = if (power_difference < (maximum*(-1))) do
      maximum*(-1)
    else
      power_difference
    end

		if (power_difference < 0) do
      IO.puts("r #{maximum + power_difference} l #{maximum}")
      set_motors(motor_ref,maximum,maximum + power_difference)
		else
      IO.puts("r #{maximum} l #{maximum - power_difference}")
      set_motors(motor_ref,maximum - power_difference,maximum)
    end

    sensor_vals = test_wlf_sensors()
    [s1,s2,s3,s4,s5] = set_vals(sensor_vals)

    {nodes,count} = if(count >= 12 or ((s1 == 1 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 1) or (s1 == 1 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 0) or (s1 == 0 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 1) or (s1 == 1 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 0) or (s1 == 0 and s2 == 0 and s3 == 1 and s4 == 1 and s5 == 1) or (s1 == 1 and s2 == 1 and s3 == 1 and s4 == 0 and s5 == 0) or (s1 == 0 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 0) and count > 5)) do
        nodes = nodes + 1
        count = 1
        # IO.puts(nodes)
        # IO.puts("Node")
        # IO.puts(nodes)
        motor_action(motor_ref,@stop)
        Process.sleep(100)
        pwm(100)
        {nodes,count}
    else
      {nodes,count}
    end

    # if((s1 == 0 and s2 == 0 and s3 == 0 and s4 == 0 and s5 == 0) or
    if(nodes == 2) do #6
      forward(channel,count,filter,nodes,1,motor_ref,maximum,integral,last_proportional)
    else
      forward(channel,count,filter,nodes,0,motor_ref,maximum,integral,last_proportional)
    end
  end

  def forward(channel,_count,_filter,_nodes,_stop,motor_ref,_maximum,_integral,_last_proportional) do
    motor_action(motor_ref,@stop)
  end


  def find_line(motor_ref) do
    # motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    sensor_vals = test_wlf_sensors()
    [s1,s2,s3,s4,s5] = set_vals(sensor_vals)
    # IO.inspect(set_vals(sensor_vals))
    if(s1+s2+s3+s4+s5 == 0) do
      motor_action(motor_ref,@right)
      pwm(100)
      Process.sleep(150)
      motor_action(motor_ref,@stop)
      sensor_vals = test_wlf_sensors()
      [s1,s2,s3,s4,s5] = set_vals(sensor_vals)
      # IO.inspect(set_vals(sensor_vals))
      if(s1+s2+s3+s4+s5 == 0) do
        motor_action(motor_ref,@left)
        pwm(100)
        Process.sleep(150)
        motor_action(motor_ref,@stop)
      end
    end

    sensor_vals = test_wlf_sensors()
    [s1,s2,s3,s4,s5] = set_vals(sensor_vals)
    # IO.inspect(set_vals(sensor_vals))
    if(s1+s2+s3+s4+s5 == 0) do
      motor_action(motor_ref,@left)
      pwm(100)
      Process.sleep(150)
      motor_action(motor_ref,@stop)
      sensor_vals = test_wlf_sensors()
      [s1,s2,s3,s4,s5] = set_vals(sensor_vals)
      # IO.inspect(set_vals(sensor_vals))
      if(s1+s2+s3+s4+s5 == 0) do
        motor_action(motor_ref,@right)
        pwm(100)
        Process.sleep(150)
        motor_action(motor_ref,@stop)
      end
    end
    Process.sleep(1000)
  end

  def set_vals(vals) do
    {_s0, vals} = List.pop_at(vals,0)
    # List.replace_at(vals,1,Enum.at(vals,1)+100)
    Enum.map(vals, fn x -> if(x > 800) do
        1
      else
        0
      end
    end)
  end

  def test_motion do
    Logger.debug("Testing Motion of the Robot ")
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
    motion_list = [@forward,@stop]
    Enum.each(motion_list, fn motion -> motor_action(motor_ref,motion) end)
  end

  @doc """
  Supporting function for test_wlf_sensors
  Configures sensor pins as input or output

  [cs: output, clock: output, address: output, dataout: input]
  """
  defp configure_sensor({atom, pin_no}) do
    if (atom == :dataout) do
      GPIO.open(pin_no, :input, pull_mode: :pullup)
    else
      GPIO.open(pin_no, :output)
    end
  end

  @doc """
  Supporting function for test_wlf_sensors
  Reads the sensor values into an array. "sensor_list" is used to provide list
  of the sesnors for which readings are needed


  The values returned are a measure of the reflectance in abstract units,
  with higher values corresponding to lower reflectance (e.g. a black
  surface or void)
  """
  defp get_lfa_readings(sensor_list, sensor_ref) do
    append_sensor_list = sensor_list ++ [5]
    temp_sensor_list = [5 | append_sensor_list]
    vals = append_sensor_list
        |> Enum.with_index
        |> Enum.map(fn {sens_num, sens_idx} ->
              analog_read(sens_num, sensor_ref, Enum.fetch(temp_sensor_list, sens_idx))
              end)
    Enum.each(0..5, fn n -> provide_clock(sensor_ref) end)
    GPIO.write(sensor_ref[:cs], 1)
    # IO.inspect(vals)
    Process.sleep(50)
    # get_lfa_readings(sensor_list, sensor_ref)
    vals
  end

  @doc """
  Supporting function for test_wlf_sensors
  """
  defp analog_read(sens_num, sensor_ref, {_, sensor_atom_num}) do

    GPIO.write(sensor_ref[:cs], 0)
    %{^sensor_atom_num => sensor_atom} = @lf_sensor_map
    Enum.reduce(0..9, @lf_sensor_data, fn n, acc ->
                                          read_data(n, acc, sens_num, sensor_ref, sensor_atom_num)
                                          |> clock_signal(n, sensor_ref)
                                        end)[sensor_atom]
  end

  @doc """
  Supporting function for test_wlf_sensors
  """
  defp read_data(n, acc, sens_num, sensor_ref, sensor_atom_num) do
    if (n < 4) do

      if (((sens_num) >>> (3 - n)) &&& 0x01) == 1 do
        GPIO.write(sensor_ref[:address], 1)
      else
        GPIO.write(sensor_ref[:address], 0)
      end
      Process.sleep(1)
    end

    %{^sensor_atom_num => sensor_atom} = @lf_sensor_map
    if (n <= 9) do
      Map.update!(acc, sensor_atom, fn sensor_atom -> ( sensor_atom <<< 1 ||| GPIO.read(sensor_ref[:dataout]) ) end)
    end
  end

  @doc """
  Supporting function for test_wlf_sensors used for providing clock pulses
  """
  defp provide_clock(sensor_ref) do
    GPIO.write(sensor_ref[:clock], 1)
    GPIO.write(sensor_ref[:clock], 0)
  end

  @doc """
  Supporting function for test_wlf_sensors used for providing clock pulses
  """
  defp clock_signal(acc, n, sensor_ref) do
    GPIO.write(sensor_ref[:clock], 1)
    GPIO.write(sensor_ref[:clock], 0)
    acc
  end

  @doc """
  Supporting function for test_motion
  """
  defp motor_action(motor_ref,motion) do
    motor_ref |> Enum.zip(motion) |> Enum.each(fn {{_, ref_no}, value} -> GPIO.write(ref_no, value) end)
  end

  @doc """
  Supporting function for test_pwm

  Note: "duty" variable can take value from 0 to 255. Value 255 indicates 100% duty cycle
  """
  defp pwm(duty) do
    Enum.each(@pwm_pins, fn {_atom, pin_no} -> Pigpiox.Pwm.gpio_pwm(pin_no, duty) end)
  end

  def pwml(duty) do
    Pigpiox.Pwm.gpio_pwm(6, duty)
    Process.sleep(1)
  end

  def pwmr(duty) do
    Pigpiox.Pwm.gpio_pwm(26, duty)
    Process.sleep(1)
  end
end
