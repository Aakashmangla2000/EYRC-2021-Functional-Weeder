defmodule Task4CClientRobotB.LineFollower do
  @moduledoc """
  This module implements line following functionality for the weeder robot.
  """

  ## importing necessary libraries and setting pins
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
  test_wlf_sensors function reads and returns the 5-array ir sensor values detecting white line and black surface.
  """
  def test_wlf_sensors do
    # Logger.debug("Testing white line sensors connected ")
    sensor_ref = Enum.map(@sensor_pins, fn {atom, pin_no} -> configure_sensor({atom, pin_no}) end)
    sensor_ref = Enum.map(sensor_ref, fn{_atom, ref_id} -> ref_id end)
    sensor_ref = Enum.zip(@ref_atoms, sensor_ref)
    get_lfa_readings([0,1,2,3,4], sensor_ref)
  end

  @doc """
  test_ir function reads and returns the ir sensor values detecting the obstacles or the plants.
  The presence of an object is indicated by 0 and the absence is indicated by 1.
  """
  def test_ir do
    ir_ref = Enum.map(@ir_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :input, pull_mode: :pullup) end)
    ir_values = Enum.map(ir_ref,fn {_, ref_no} -> GPIO.read(ref_no) end)
  end

  @doc """
  ir_sensors function calls test_ir() function to store and return the ir sensor values placed at the front and at the back of
  the robot. The value is returned in the form of an array with first element as the front sensor value and the second element as the back sensor value.
  """
  def ir_sensors do
    proximity = test_ir()
    front  =  Enum.at(proximity, 0)
    back = Enum.at(proximity, 1)
    [front,back]
  end

  @doc """
  obs_detect function calls ir_sensors function to check the presence of obstacles in the front ir sensor.
  In case the front value is 0 the function returns true else it return false.
  """
  def obs_detect do
    obs = false
    [front,back] = ir_sensors()
    obs = if front == 0 do
      true
    else
      false
    end
    obs
  end

  @doc """
  open_motor_pwm_pins function is setting up motor and pwm pins for further use.
  """
  def open_motor_pwm_pins() do
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
    {motor_ref,pwm_ref}
  end

  @doc """
  pid function implements forward movement of the robot along the white grid lines.
  This function sets up the default values of the variables and calls find_line function which helps the robot locate the line.
  Once the line is located, the function calls the forward function to move the robot along the line.
  """
  def pid(channel,motor_ref) do
    Logger.debug("Going Forward")
    Process.sleep(100)

    maximum = 100;  #maximum power that can be given to the wheels
    integral = 0;   #default integral value
    last_proportional = 0   #default last propotional value
    motor_action(motor_ref,@stop)
    Task4CClientRobotB.PhoenixSocketClient.timer(channel)
    pwm(10)  #setting up pwm value
    Process.sleep(5)

    count = 1  #Keeps the track of the number of times a particular function has been called
    nodes = 1  #tracks the number of nodes covered
    stop = 0
    proportional = 0 #default proportional value
    x = 1  #variable used in set_vals
    y = 1
    filter = 1 #default filter value

    sensor_vals = test_wlf_sensors() #storing sensor value
    [s1,s2,s3,s4,s5] = set_vals(sensor_vals) #setting sensor values in the form of 0s and 1s
    IO.inspect(sensor_vals)

    #If the 5-array ir sensor is entirely on the black surface detecting no line, find_line function will be called
    if(s1 == 0 and s2 == 0 and s3 == 0 and s4 == 0 and s5 == 0) do
      find_line(motor_ref)
    end

    motor_action(motor_ref,@forward)
    forward(channel,count,filter,nodes,stop,motor_ref,maximum,integral,last_proportional)
    Task4CClientRobotB.PhoenixSocketClient.timer(channel)
  end

  @doc """
  right function implements the right turn of the robot by reading the 5-array ir sensor.
  The robot will turn until it detects a white line under the sensors.
  """
  def right(channel,motor_ref,count) do
    Task4CClientRobotB.PhoenixSocketClient.timer(channel)
    IO.puts("going right")
    count = count + 1
    sensor_vals = test_wlf_sensors()
    [s1,s2,s3,s4,s5] = set_vals(sensor_vals)
    IO.inspect(set_vals(sensor_vals))
    pwm(120)
    Process.sleep(100)
    motor_action(motor_ref,@left)
    Process.sleep(100)
    motor_action(motor_ref,@stop)
    Process.sleep(100)
    if(count > 2 and (s1 == 1 or s2 == 1 or s3 == 1 or s4 == 1 or s5 == 1)) do
    else
      right(channel,motor_ref,count)
    end
  end

  @doc """
  left function implements the left turn of the robot by reading the 5-array ir sensor.
  The robot will turn until it detects a white line under the sensors.
  """
  def left(channel,motor_ref,count) do
    Task4CClientRobotB.PhoenixSocketClient.timer(channel)
    IO.puts("going left")
    count = count + 1
    sensor_vals = test_wlf_sensors()
    [s1,s2,s3,s4,s5] = set_vals(sensor_vals)
    IO.inspect(set_vals(sensor_vals))
    pwm(120)
    Process.sleep(100)
    motor_action(motor_ref,@right)
    Process.sleep(100)
    motor_action(motor_ref,@stop)
    Process.sleep(100)
    if(count > 2 and (s1 == 1 or s2 == 1 or s3 == 1 or s4 == 1 or s5 == 1)) do
    else
      left(channel,motor_ref,count)
    end
  end

  @doc """
  set_motors provide pwm value for left and right wheels of the robot
  """
  def set_motors(_motor_ref,r,l) do
    pwml(l)
    pwmr(r)
  end

  @doc """
  loop function recieves the sensor values in the form of 0s and 1s and applies a formula.
  """
  def loop(j,i,sensor_vals) when j < 5 do
    sensor_vals = List.replace_at(sensor_vals,j,1000*i*Enum.at(sensor_vals,j))
    loop(j+1,i-1,sensor_vals)
  end

  def loop(_j,_i,sensor_vals) do
    sensor_vals
  end

   @doc """
  read_line2 function calls the loop function to recieve the tuned values of the sensor and calculates the average to find the position.
  """
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

  @doc """
  forward function implements pid- proportional,integral,derivative to set the pwm value for the right and lef wheels.
  pid requires position value which is calculated with read_line2. Here, the pid calculates the error in position of the robot with
  the help of sensor value to set pwm value in the left and right value and keep the robot along the line.
  """
  def forward(channel,count,filter,nodes,stop,motor_ref,maximum,integral,last_proportional) when stop == 0 do
    count = count + 1
    IO.puts("nodes: #{nodes}")
    filter = filter + 1

    #Simple ReadLine
    sensor_vals = test_wlf_sensors()
    position = read_line2(sensor_vals)
    [s1,s2,s3,s4,s5] = set_vals(sensor_vals)
    IO.inspect(set_vals(sensor_vals))

    #The condition here allows the robot to move after 6 ir-sensor readings to reduce intial errors in the sensor value.
    if(filter > 6) do
      motor_action(motor_ref,@forward)
      filter = 1
    else
      motor_action(motor_ref,@stop)
    end

    proportional = position - 2000

		# Compute the derivative (change) and integral (sum) of the position.
		derivative = proportional - last_proportional
		integral = integral + proportional

		# Remember the last position.
		last_proportional = proportional

    # Calculating the power difference
		power_difference = proportional*0.015 + derivative*0.020
    power_difference = Kernel.round(power_difference)

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
      set_motors(motor_ref,maximum,maximum + power_difference)
		else
      set_motors(motor_ref,maximum - power_difference,maximum)
    end

    sensor_vals = test_wlf_sensors()
    [s1,s2,s3,s4,s5] = set_vals(sensor_vals)

    #Check if the next node is reached, if yes then the robot will be stopped
    {nodes,count} = if(count >= 13 or ((s1 == 1 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 1) or (s1 == 1 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 0) or (s1 == 0 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 1) or (s1 == 1 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 0) or (s1 == 0 and s2 == 0 and s3 == 1 and s4 == 1 and s5 == 1) or (s1 == 1 and s2 == 1 and s3 == 1 and s4 == 0 and s5 == 0) or (s1 == 0 and s2 == 1 and s3 == 1 and s4 == 1 and s5 == 0) and count > 5)) do
        nodes = nodes + 1
        count = 1
        motor_action(motor_ref,@stop)
        Process.sleep(100)
        pwm(100)
        {nodes,count}
    else
      {nodes,count}
    end

    if(nodes == 2) do
      forward(channel,count,filter,nodes,1,motor_ref,maximum,integral,last_proportional)
    else
      forward(channel,count,filter,nodes,0,motor_ref,maximum,integral,last_proportional)
    end
  end

  def forward(channel,_count,_filter,_nodes,_stop,motor_ref,_maximum,_integral,_last_proportional) do
    motor_action(motor_ref,@stop)
  end

  @doc """
  find_line function is used in the case where robot is completely on the black surface indicating no line.
  This function shifts the robot left and right accordingly in order to search for the line.
  """
  def find_line(motor_ref) do
    sensor_vals = test_wlf_sensors()
    [s1,s2,s3,s4,s5] = set_vals(sensor_vals)

    if(s1+s2+s3+s4+s5 == 0) do
      motor_action(motor_ref,@right)
      pwm(100)
      Process.sleep(150)
      motor_action(motor_ref,@stop)
      sensor_vals = test_wlf_sensors()
      [s1,s2,s3,s4,s5] = set_vals(sensor_vals)

      if(s1+s2+s3+s4+s5 == 0) do
        motor_action(motor_ref,@left)
        pwm(100)
        Process.sleep(150)
        motor_action(motor_ref,@stop)
      end
    end

    sensor_vals = test_wlf_sensors()
    [s1,s2,s3,s4,s5] = set_vals(sensor_vals)

    if(s1+s2+s3+s4+s5 == 0) do
      motor_action(motor_ref,@left)
      pwm(100)
      Process.sleep(150)
      motor_action(motor_ref,@stop)
      sensor_vals = test_wlf_sensors()
      [s1,s2,s3,s4,s5] = set_vals(sensor_vals)

      if(s1+s2+s3+s4+s5 == 0) do
        motor_action(motor_ref,@right)
        pwm(100)
        Process.sleep(150)
        motor_action(motor_ref,@stop)
      end
    end
    Process.sleep(1000)
  end
  @doc """
  set_vals function converts every individual sensor value to 0 and 1 depending on the threshold value defined.
  """
  def set_vals(vals) do
    {_s0, vals} = List.pop_at(vals,0)
    Enum.map(vals, fn x -> if(x > 800) do
        1
      else
        0
      end
    end)
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
    Process.sleep(50)
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
  Supporting function for test_pwm is pwm
  Note: "duty" variable can take value from 0 to 255. Value 255 indicates 100% duty cycle
  pwml and pwmr spefically sets pwm value for left and right wheels
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
