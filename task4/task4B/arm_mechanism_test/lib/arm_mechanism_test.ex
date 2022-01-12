defmodule ArmMechanismTest do
  @moduledoc """
  Documentation for `FW_DEMO`.

  Different functions provided for testing components of Alpha Bot.
  test_wlf_sensors  - to test white line sensors
  test_ir           - to test IR proximity sensors
  test_motion       - to test motion of the Robot
  test_pwm          - to test speed of the Robot
  test_servo_a      - to test servo motor A
  test_servo_b      - to test servo motor B
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

  # @doc """
  # Tests white line sensor modules reading


  #     iex> FW_DEMO.test_wlf_sensors
  #     [0, 958, 851, 969, 975, 943]  // on white surface
  #     [0, 449, 356, 312, 321, 267]  // on black surface
  # """
  # def test_wlf_sensors do
  #   Logger.debug("Testing white line sensors connected ")
  #   sensor_ref = Enum.map(@sensor_pins, fn {atom, pin_no} -> configure_sensor({atom, pin_no}) end)
  #   sensor_ref = Enum.map(sensor_ref, fn{_atom, ref_id} -> ref_id end)
  #   sensor_ref = Enum.zip(@ref_atoms, sensor_ref)
  #   get_lfa_readings([1,2,3,4,5], sensor_ref)
  # end


  # @doc """
  # Tests IR Proximity sensor's readings

  # Example:

  #     iex> FW_DEMO.test_ir
  #     [1, 1]     // No obstacle
  #     [1, 0]     // Obstacle in front of Right IR Sensor
  #     [0, 1]     // Obstacle in front of Left IR Sensor
  #     [0, 0]     // Obstacle in front of both Sensors

  # Note: You can adjust the potentiometer provided on the IR sensor to get proper results
  # """
  # def test_ir do
  #   Logger.debug("Testing IR Proximity Sensors")
  #   ir_ref = Enum.map(@ir_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :input, pull_mode: :pullup) end)
  #   ir_values = Enum.map(ir_ref,fn {_, ref_no} -> GPIO.read(ref_no) end)
  # end


  @doc """
  Tests motion of the Robot

  Example:

      iex> FW_DEMO.test_motion
      :ok

  Note: On executing above function Robot will move forward, backward, left, right
  for 500ms each and then stops
  """
  def test_motion do
    # Logger.debug("Testing Motion of the Robot ")
    motor_ref = Enum.map(@motor_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    pwm_ref = Enum.map(@pwm_pins, fn {_atom, pin_no} -> GPIO.open(pin_no, :output) end)
    Enum.map(pwm_ref,fn {_, ref_no} -> GPIO.write(ref_no, 1) end)
    motion_list = [@forward,@forward,@forward,@forward,@forward,@forward,@stop]
    Enum.each(motion_list, fn motion -> motor_action(motor_ref,motion) end)
    # Enum.map(@duty_cycles, fn value -> motion_pwm(value) end)

  end

  # def arm_up(angle) do
  #   if (angle < 50) do
  #     test_servo_b(angle)
  #     angle = angle + 1
  #     Process.sleep(70)
  #     arm_up(angle)
  #   else
  #     IO.puts(angle)
  #   end
  # end

  # def arm_down(angle) do
  #   if (angle > 15) do
  #     test_servo_b(angle)
  #     angle = angle - 1
  #     Process.sleep(70)
  #     arm_down(angle)
  #   else
  #     IO.puts(angle)
  #   end
  # end

  def weeding do
    IO.puts("Positioning the arm...")
    test_servo_b(45)  #setting arm at height
    Process.sleep(250)
    IO.puts("Opening the Claws...")
    test_servo_a(90)  #opening claws
    Process.sleep(250)
    IO.puts("Weeding Begins...")
    test_servo_b(20)
    Process.sleep(500)
    test_servo_a(0)
    Process.sleep(500)
    test_servo_b(50)
    Process.sleep(1000)

    IO.puts("Change position...")
    test_motion

    IO.puts("Weed Depositing...")
    Process.sleep(500)
    test_servo_b(30)

    Process.sleep(500)
    test_servo_a(90)

    Process.sleep(500)
    test_servo_b(50)
    IO.puts("Weeding Ends...")




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
    # Enum.map(@duty_cycles, fn value -> motion_pwm(value) end)
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
    # Logger.debug("Testing Servo A")
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
    # Logger.debug("Testing Servo B")
    val = trunc(((2.5 + 10.0 * angle / 180) / 100 ) * 255)
    Pigpiox.Pwm.set_pwm_frequency(@servo_b_pin, @pwm_frequency)
    Pigpiox.Pwm.gpio_pwm(@servo_b_pin, val)



  end

  # @doc """
  # Supporting function for test_wlf_sensors
  # Configures sensor pins as input or output

  # [cs: output, clock: output, address: output, dataout: input]
  # """
  # defp configure_sensor({atom, pin_no}) do
  #   if (atom == :dataout) do
  #     GPIO.open(pin_no, :input, pull_mode: :pullup)
  #   else
  #     GPIO.open(pin_no, :output)
  #   end
  # end

  @doc """
  Supporting function for test_wlf_sensors
  Reads the sensor values into an array. "sensor_list" is used to provide list
  of the sesnors for which readings are needed


  The values returned are a measure of the reflectance in abstract units,
  with higher values corresponding to lower reflectance (e.g. a black
  surface or void)
  """
  # defp get_lfa_readings(sensor_list, sensor_ref) do
  #   append_sensor_list = sensor_list ++ [5]
  #   temp_sensor_list = [5 | append_sensor_list]
  #   IO.inspect(append_sensor_list
  #       |> Enum.with_index
  #       |> Enum.map(fn {sens_num, sens_idx} ->
  #             analog_read(sens_num, sensor_ref, Enum.fetch(temp_sensor_list, sens_idx))
  #             end))
  #   Enum.each(0..5, fn n -> provide_clock(sensor_ref) end)
  #   GPIO.write(sensor_ref[:cs], 1)
  #   Process.sleep(250)
  #   get_lfa_readings(sensor_list, sensor_ref)
  # end

  @doc """
  Supporting function for test_wlf_sensors
  """
  # defp analog_read(sens_num, sensor_ref, {_, sensor_atom_num}) do

  #   GPIO.write(sensor_ref[:cs], 0)
  #   %{^sensor_atom_num => sensor_atom} = @lf_sensor_map
  #   Enum.reduce(0..9, @lf_sensor_data, fn n, acc ->
  #                                         read_data(n, acc, sens_num, sensor_ref, sensor_atom_num)
  #                                         |> clock_signal(n, sensor_ref)
  #                                       end)[sensor_atom]
  # end

  @doc """
  Supporting function for test_wlf_sensors
  """
  # defp read_data(n, acc, sens_num, sensor_ref, sensor_atom_num) do
  #   if (n < 4) do

  #     if (((sens_num) >>> (3 - n)) &&& 0x01) == 1 do
  #       GPIO.write(sensor_ref[:address], 1)
  #     else
  #       GPIO.write(sensor_ref[:address], 0)
  #     end
  #     Process.sleep(1)
  #   end

  #   %{^sensor_atom_num => sensor_atom} = @lf_sensor_map
  #   if (n <= 9) do
  #     Map.update!(acc, sensor_atom, fn sensor_atom -> ( sensor_atom <<< 1 ||| GPIO.read(sensor_ref[:dataout]) ) end)
  #   end
  # end

  @doc """
  Supporting function for test_wlf_sensors used for providing clock pulses
  """
  # defp provide_clock(sensor_ref) do
  #   GPIO.write(sensor_ref[:clock], 1)
  #   GPIO.write(sensor_ref[:clock], 0)
  # end

  # @doc """
  # Supporting function for test_wlf_sensors used for providing clock pulses
  # """
  # defp clock_signal(acc, n, sensor_ref) do
  #   GPIO.write(sensor_ref[:clock], 1)
  #   GPIO.write(sensor_ref[:clock], 0)
  #   acc
  # end

  @doc """
  Supporting function for test_motion
  """
  defp motor_action(motor_ref,motion) do
    # IO.puts("a #{inspect(motor_ref)}")
    motor_ref |> Enum.zip(motion) |> Enum.each(fn {{_, ref_no}, value} -> GPIO.write(ref_no, value) end)
    pwm(100)

    Process.sleep(200)
  end

  @doc """
  Supporting function for test_pwm
  """
  defp motion_pwm(value) do
    IO.puts("Forward with pwm value = #{value}")
    pwm(value)
    # Process.sleep(2000)
  end

  @doc """
  Supporting function for test_pwm

  Note: "duty" variable can take value from 0 to 255. Value 255 indicates 100% duty cycle
  """
  defp pwm(duty) do
    Enum.each(@pwm_pins, fn {_atom, pin_no} -> Pigpiox.Pwm.gpio_pwm(pin_no, duty) end)
  end

end
