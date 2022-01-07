defmodule LineFollower do
  @moduledoc """
  Documentation for `LineFollower`.
  """

  require Logger
  use Bitwise
  alias Integer
  alias Decimal
  alias Circuits.GPIO

  @doc """
  Hello world.

  ## Examples

      iex> LineFollower.hello()
      :world

  """
  def hello do
    :world
  end

  @sensor_pins [cs: 5, clock: 25, address: 24, dataout: 23]
  @ir_pins [dr: 16, dl: 19]
  @motor_pins [lf: 12, lb: 13, rf: 20, rb: 21]
  @pwm_pins [enl: 6, enr: 26]
  @servo_a_pin 27
  @servo_b_pin 22

  @ref_atoms [:cs, :clock, :address, :dataout]
  @lf_sensor_data %{sensor0: 0, sensor1: 0, sensor2: 0, sensor3: 0, sensor4: 0, sensor5: 0}
  @lf_sensor_map %{0 => :sensor0, 1 => :sensor1, 2 => :sensor2, 3 => :sensor3, 4 => :sensor4, 5 => :sensor5}

  @left [1, 0, 0, 0]
  @right [0, 0, 0, 1]
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


  @doc """
  Tests IR Proximity sensor's readings

  Example:

      iex> FW_DEMO.test_ir
      [1, 1]     // No obstacle
      [1, 0]     // Obstacle in front of Right IR Sensor
      [0, 1]     // Obstacle in front of Left IR Sensor
      [0, 0]     // Obstacle in front of both Sensors

  Note: You can adjust the potentiometer provided on the IR sensor to get proper results
  """
  def test_ir do
    Logger.debug("Testing IR Proximity Sensors")
    ir_ref = Enum.map(@ir_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :input, pull_mode: :pullup) end)
    ir_values = Enum.map(ir_ref,fn {_, ref_no} -> GPIO.read(ref_no) end)
  end

  def calibrate_400(cal_min,cal_max,val) when val < 10 do
    IO.puts("Calibrating #{val} time")
    {min,max} = calibrate()

    # IO.puts("max in calibrate #{inspect(max)}")
    # IO.puts("min in calibrate #{inspect(min)}")

    {maxs1,maxs2,maxs3,maxs4,maxs5} = max
    {mins1,mins2,mins3,mins4,mins5} = min

    {cal_maxs1,cal_maxs2,cal_maxs3,cal_maxs4,cal_maxs5} = cal_max
    {cal_mins1,cal_mins2,cal_mins3,cal_mins4,cal_mins5} = cal_min

    cal_maxs1 = if(maxs1 < cal_maxs1) do
      maxs1
    else
      cal_maxs1
    end

    cal_mins1 = if(mins1 > cal_mins1) do
      mins1
    else
      cal_mins1
    end

    cal_maxs2 = if(maxs2 < cal_maxs2) do
      maxs2
    else
      cal_maxs2
    end

    cal_mins2 = if(mins2 > cal_mins2) do
      mins2
    else
      cal_mins2
    end

    cal_maxs3 = if(maxs3 < cal_maxs3) do
      maxs3
    else
      cal_maxs3
    end

    cal_mins3 = if(mins3 > cal_mins3) do
      mins3
    else
      cal_mins3
    end

    cal_maxs4 = if(maxs4 < cal_maxs4) do
      maxs4
    else
      cal_maxs4
    end

    cal_mins4 = if(mins4 > cal_mins4) do
      mins4
    else
      cal_mins4
    end

    cal_maxs5 = if(maxs5 < cal_maxs5) do
      maxs5
    else
      cal_maxs5
    end

    cal_mins5 = if(mins5 > cal_mins5) do
      mins5
    else
      cal_mins5
    end

    cal_max = {cal_maxs1,cal_maxs2,cal_maxs3,cal_maxs4,cal_maxs5}
    cal_min = {cal_mins1,cal_mins2,cal_mins3,cal_mins4,cal_mins5}

    # IO.puts("cal max in calibrate #{inspect(cal_max)}")
    # IO.puts("cal min in calibrate #{inspect(cal_min)}")

    calibrate_400(cal_min,cal_max,val+1)
  end

  def calibrate_400(cal_min,cal_max,val) do
    {cal_min,cal_max}
  end

  def calibrate do
    max = {0,0,0,0,0}
    min = {0,0,0,0,0}
    IO.puts("in calibrate")

    {max,min} = run_ten_times(max,min,0)
    {maxs1,maxs2,maxs3,maxs4,maxs5} = max
    {mins1,mins2,mins3,mins4,mins5} = min
    {min,max}
  end

  def run_ten_times(max,min,val) when val < 10 do
    # IO.puts("Running #{val}th time")
    vals = test_wlf_sensors()
    # IO.puts("sensor vals#{inspect(vals)}")

    {maxs1,maxs2,maxs3,maxs4,maxs5} = max
    {mins1,mins2,mins3,mins4,mins5} = min
    # IO.puts("max old #{inspect(max)}")
    {s0, vals} = List.pop_at(vals,0)
    {s1, vals} = List.pop_at(vals,0)
    {s2, vals} = List.pop_at(vals,0)
    {s3, vals} = List.pop_at(vals,0)
    {s4, vals} = List.pop_at(vals,0)
    {s5, vals} = List.pop_at(vals,0)

    maxs1 = if(maxs1 < s1 or val == 0) do
      s1
    else
      maxs1
    end

    mins1 = if(mins1 > s1 or val == 0) do
      s1
    else
      mins1
    end

    maxs2 = if(maxs2 < s2 or val == 0) do
      s2
    else
      maxs2
    end

    mins2 = if(mins2 > s2 or val == 0) do
      s2
    else
      mins2
    end

    maxs3 = if(maxs3 < s3 or val == 0) do
     s3
    else
      maxs3
    end

    mins3 = if(mins3 > s3 or val == 0) do
      s3
    else
      mins3
    end

    maxs4 = if(maxs4 < s4 or val == 0) do
      s4
    else
      maxs4
    end

    mins4 = if(mins4 > s4 or val == 0) do
      s4
    else
      mins4
    end

    maxs5 = if(maxs5 < s5 or val == 0) do
      s5
    else
      maxs5
    end

    mins5 = if(mins5 > s5 or val == 0) do
      s5
    else
      mins5
    end


    max = {maxs1,maxs2,maxs3,maxs4,maxs5}
    min = {mins1,mins2,mins3,mins4,mins5}
    # IO.puts("max new #{inspect(max)}")
    run_ten_times(max,min,val+1)
  end

  def run_ten_times(max,min,val) do
    {max,min}
  end

  def pid() do
    Logger.debug("PID")
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)

    maximum = 70;
    integral = 0;
    last_proportional = 0
    max = {1023,1023,1023,1023,1023}
    min = {0,0,0,0,0}
    motor_action(motor_ref,@stop)
    pwm(70)
    {cal_min,cal_max} = calibrate_400(min,max,0)

    {cal_maxs1,cal_maxs2,cal_maxs3,cal_maxs4,cal_maxs5} = cal_max
    {cal_mins1,cal_mins2,cal_mins3,cal_mins4,cal_mins5} = cal_min

    last_value = 0

    cal_max = [cal_maxs1,cal_maxs2,cal_maxs3,cal_maxs4,cal_maxs5]
    cal_min = [cal_mins1,cal_mins2,cal_mins3,cal_mins4,cal_mins5]

    # IO.puts("cal max in calibrate #{inspect(cal_max)}")
    # IO.puts("cal min in calibrate #{inspect(cal_min)}")


    motor_action(motor_ref,@forward)
    # pwm(30)
    forward(cal_min,cal_max,last_value,maximum,integral,last_proportional,0)
  end

  def forward(cal_min,cal_max,last_value,maximum,integral,last_proportional,proportional) do
    # IO.puts("Going Forward")
    sensor_vals = read_calibrated(cal_min,cal_max)
    # IO.puts("after calibration sensor vals #{inspect(sensor_vals)}")

    position = read_line(sensor_vals,last_value,0,0,0)
    IO.puts("position #{inspect(position)}")


    proportional = position - 2000

		# Compute the derivative (change) and integral (sum) of the position.
		derivative = proportional - last_proportional
		integral = integral + proportional

		# Remember the last position.
		last_proportional = proportional

		power_difference = proportional/30 + derivative/110 #+ integral/1000;
    power_difference = Kernel.round(power_difference)

		power_difference = if (power_difference > maximum) do
			maximum
    else
      power_difference
    end

		power_difference = if (power_difference < (0 - maximum)) do
      0	- maximum
    else
      power_difference
    end
    IO.puts("power #{inspect(power_difference)}")

		if (power_difference < 0) do
      IO.puts("r #{maximum + power_difference} l #{maximum}")
			pwmr(maximum + power_difference)
			pwml(maximum)
		else
      IO.puts("r #{maximum} l #{maximum - power_difference}")
			pwmr(maximum)
			pwml(maximum - power_difference)
    end
    forward(cal_min,cal_max,last_value,maximum,integral,last_proportional,proportional)
  end

  def read_calibrated(cal_min,cal_max) do
    # IO.puts("in read cali")
    vals = test_wlf_sensors()
    # IO.inspect(vals)
    {s0, vals} = List.pop_at(vals,0)
    sens_vals(0,cal_min,cal_max,0,0,vals)
  end

  def sens_vals(val,cal_min,cal_max,denominator,value,sensor_vals) when val < 5 do
    # IO.puts("in sens vals")
    # IO.inspect(cal_max)
    # IO.inspect(cal_min)

    denominator = Enum.at(cal_max,val) - Enum.at(cal_min,val)
    # IO.puts("denominator #{inspect(denominator)}")


    value = if(denominator != 0) do
      Kernel.div(((Enum.at(sensor_vals,val) - Enum.at(cal_min,val))* 1000), denominator)
    else
      value
    end

    # IO.puts("value before #{inspect(value)}")

    value = if(value < 0) do
      0
    else
      value
    end
    value = if(value > 1000) do
      1000
    else
      value
    end
    # IO.puts("value after #{inspect(value)}")
    sensor_vals = List.replace_at(sensor_vals,val,value)
    # IO.puts("sensor-vals in sensvals #{inspect(sensor_vals)}")
    sens_vals(val+1,cal_min,cal_max,denominator,value,sensor_vals)
  end

  def sens_vals(val,cal_min,cal_max,denominator,value,sensor_vals) do
    sensor_vals
  end

  def read_line(sensor_vals,last_value,avg,sum,on_line) do
    {avg,sum,on_line} = set_on_line(0,sensor_vals,avg,sum,on_line)

    x = if(on_line != 1) do
      if(last_value < ((5 - 1)*1000)/2) do
        0
      else
        (5 - 1)*1000
      end
    else
      Kernel.div(avg, sum)
    end
    x

  end

  def set_on_line(val,sensor_vals,avg,sum,on_line) when val < 5 do
    # IO.puts("Sensor value #{inspect(sensor_vals)}")
    value = Enum.at(sensor_vals,val)
    # IO.puts("set-on-line-value #{inspect(value)}")
    value = 1000-value
    on_line = if(value > 200) do
      1
    else
      on_line
    end

    # only average in values that are above a noise threshold
    # IO.puts("value #{inspect(value)}")
    {avg,sum} = if(value > 50) do
      avg = avg + value * (val * 1000)  # this is for the weighted total,
      sum = sum + value
      {avg,sum}
    else
      {avg,sum}
    end
    set_on_line(val+1,sensor_vals,avg,sum,on_line)
  end

  def set_on_line(val,sensor_vals,avg,sum,on_line) do
    {avg,sum,on_line}
  end


  @doc """
  Tests motion of the Robot

  Example:

      iex> FW_DEMO.test_motion
      :ok

  Note: On executing above function Robot will move forward, backward, left, right
  for 500ms each and then stops
  """
  def test_motion(nodes,stop) when stop == 0 do
    Logger.debug("Testing Motion of the Robot ")
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
    # motion_list = [@forward,@stop]
    # {cal_min,cal_max} = calibrate_400(min,max,0)
    vals = test_wlf_sensors()
    {s1,s2,s3,s4,s5} = set_vals(vals)
    IO.puts("#{s1} #{s2} #{s3} #{s4} #{s5}")
    # Enum.each(motion_list, fn motion -> motor_action(motor_ref,motion) end)
    {stop,motion_list,nodes} = cond do
      s1 == 0 and s2 == 0 and s3 == 0 and s4 == 0 and s5 == 0 ->
        {1,[@stop],nodes}
      s1 == 0 and s2 == 0 and s3 == 1 and s4 == 0 and s5 == 0 ->
        # motor_action(motor_ref,@forward)
        {0,[@forward,@stop],nodes}
      s1 == 1 and s2 == 1 and s3 == 0 and s4 == 0 and s5 == 0 ->
        # motor_action(motor_ref,@right)
        {0,[@backward,@stop,@right,@stop],nodes}
      s1 == 1 and s2 == 0 and s3 == 0 and s4 == 0 and s5 == 0 ->
        # motor_action(motor_ref,@right)
        {0,[@backward,@stop,@right,@stop],nodes}
      s1 == 0 and s2 == 1 and s3 == 1 and s4 == 0 and s5 == 0 ->
        # motor_action(motor_ref,@right)
        {0,[@backward,@stop,@right,@stop],nodes}
      s1 == 0 and s2 == 1 and s3 == 0 and s4 == 0 and s5 == 0 ->
        # motor_action(motor_ref,@right)
        {0,[@backward,@stop,@right,@stop],nodes}
      s1 == 1 and s2 == 1 and s3 == 1 and s4 == 0 and s5 == 0 ->
        # motor_action(motor_ref,@right)
        {0,[@backward,@stop,@right,@stop],nodes}
      s1 == 0 and s2 == 0 and s3 == 1 and s4 == 1 and s5 == 1 ->
        # motor_action(motor_ref,@left)
        {0,[@backward,@stop,@left,@stop],nodes}
      s1 == 0 and s2 == 0 and s3 == 0 and s4 == 1 and s5 == 1 ->
        # motor_action(motor_ref,@left)
        {0,[@backward,@stop,@left,@stop],nodes}
      s1 == 0 and s2 == 0 and s3 == 0 and s4 == 0 and s5 == 1 ->
        # motor_action(motor_ref,@left)
        {0,[@backward,@stop,@left,@stop],nodes}
      s1 == 0 and s2 == 0 and s3 == 1 and  s4 == 1 and s5 == 0 ->
        # motor_action(motor_ref,@left)
        {0,[@backward,@stop,@left,@stop],nodes}
      s1 == 0 and s2 == 0 and s3 == 0 and s4 == 1 and s5 == 0 ->
        # motor_action(motor_ref,@left)
        {0,[@backward,@stop,@left,@stop],nodes}
      s1 == 0 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 1 ->
        # motor_action(motor_ref,@left)
        {0,[@forward,@stop],nodes}
      s1 == 0 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 0 ->
        # motor_action(motor_ref,@left)
        {0,[@forward,@stop],nodes}
      s1 == 0 and s2 == 1 and s3 == 0 and s4 == 1 and s5 == 0 ->
        # motor_action(motor_ref,@left)
        {0,[@forward,@stop],nodes}
      s1 == 1 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 1 ->
        # motor_action(motor_ref,@left)
        {0,[@forward,@stop],nodes+1}
      s1 == 0 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 1 ->
        # motor_action(motor_ref,@left)
        {0,[@backward,@stop],nodes}
      s1 == 1 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 0 ->
        # motor_action(motor_ref,@left)
        {0,[@backward,@stop],nodes}
      s1 == 1 and s2 == 0 and s3 == 0 and s4 == 0 and s5 == 1 ->
        # motor_action(motor_ref,@left)
        {0,[@backward,@stop],nodes}
      true ->
        IO.puts("last")
        {0,[@forward,@stop],nodes}
    end
    Enum.each(motion_list, fn motion -> motor_action(motor_ref,motion) end)
    IO.puts(nodes)
    test_motion(nodes,stop)
  end

  def test_motion(nodes,stop) do
    Logger.debug("Testing Motion of the Robot ")
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
    # motion_list = [@stop]
    # vals = test_wlf_sensors()
    # {s1,s2,s3,s4,s5} = set_vals(vals)
    # IO.puts("#{s1} #{s2} #{s3} #{s4} #{s5}")
    motor_action(motor_ref,@stop)
    # Enum.each(motion_list, fn motion -> motor_action(motor_ref,motion) end)
  end

  def set_vals(vals) do
    {s0, vals} = List.pop_at(vals,0)
    {s1, vals} = List.pop_at(vals,0)
    {s2, vals} = List.pop_at(vals,0)
    {s3, vals} = List.pop_at(vals,0)
    {s4, vals} = List.pop_at(vals,0)
    {s5, vals} = List.pop_at(vals,0)

    IO.puts("#{s1} #{s2} #{s3} #{s4} #{s5}")
    s1 = if(s1 > 900) do
      1
    else
      0
    end

    s2 = if(s2 > 900) do
      1
    else
      0
    end

    s3 = if(s3 > 900) do
      1
    else
      0
    end

    s4 = if(s4 > 900) do
      1
    else
      0
    end

    s5 = if(s5 > 900) do
      1
    else
      0
    end

    {s1,s2,s3,s4,s5}
  end


  @doc """
  Controls speed of the Robot

  Example:

      iex> FW_DEMO.test_pwm
      Forward with pwm value = 150
      Forward with pwm value = 70
      Forward with pwm value = 0
      {:ok, :ok, :ok}

  Note: On executing above function Robot will move in forward direction with different velocities
  """
  def test_pwm do
    Logger.debug("Testing PWM for Motion control")
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    motor_action(motor_ref, @forward)
    Enum.map(@duty_cycles, fn value -> motion_pwm(value) end)
  end


  @doc """
  Controls angle of serve motor A

  Example:

      iex> FW_DEMO.test_servo_a(90)
      :ok

  Note: On executing above function servo motor A will rotate by 90 degrees. You can provide
  values from 0 to 180
  """
  def test_servo_a(angle) do
    Logger.debug("Testing Servo A")
    val = trunc(((2.5 + 10.0 * angle / 180) / 100 ) * 255)
    Pigpiox.Pwm.set_pwm_frequency(@servo_a_pin, @pwm_frequency)
    Pigpiox.Pwm.gpio_pwm(@servo_a_pin, val)
  end


  @doc """
  Controls angle of serve motor B

  Example:

      iex> FW_DEMO.test_servo_b(90)
      :ok

  Note: On executing above function servo motor B will rotate by 90 degrees. You can provide
  values from 0 to 180
  """
  def test_servo_b(angle) do
    Logger.debug("Testing Servo B")
    val = trunc(((2.5 + 10.0 * angle / 180) / 100 ) * 255)
    Pigpiox.Pwm.set_pwm_frequency(@servo_b_pin, @pwm_frequency)
    Pigpiox.Pwm.gpio_pwm(@servo_b_pin, val)
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

    # IO.puts("sensor list #{inspect(sensor_list)}")

    append_sensor_list = sensor_list ++ [5]

    # IO.puts("append sensor list #{inspect(append_sensor_list)}")

    temp_sensor_list = [5 | append_sensor_list]

    # IO.puts("temp sensor list #{inspect(temp_sensor_list)}")
    vals = append_sensor_list
        |> Enum.with_index
        |> Enum.map(fn {sens_num, sens_idx} ->
              analog_read(sens_num, sensor_ref, Enum.fetch(temp_sensor_list, sens_idx))
              end)
    # IO.inspect(IEx.Info.info(vals))
    # IO.puts("value #{inspect(vals)}")
    Enum.each(0..5, fn n -> provide_clock(sensor_ref) end)
    GPIO.write(sensor_ref[:cs], 1)
    Process.sleep(250)
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


    # cond do
    #   motion == @right ->
    #     pwm(110)
    #     Process.sleep(80)
    #   motion == @left ->
    #     pwm(110)
    #     Process.sleep(80)
    #   motion == @forward ->
    #     pwm(150)
    #     Process.sleep(150)
    #   motion == @backward ->
    #     pwm(100)
    #     Process.sleep(50)
    #   true ->
    #     pwm(150)
    #     Process.sleep(100)
    # end

    # pwm(80)

  end

  @doc """
  Supporting function for test_pwm
  """
  defp motion_pwm(value) do
    IO.puts("Forward with pwm value = #{value}")
    pwm(value)
    Process.sleep(2000)
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
  end

  def pwmr(duty) do
    Pigpiox.Pwm.gpio_pwm(26, duty)
  end
end
