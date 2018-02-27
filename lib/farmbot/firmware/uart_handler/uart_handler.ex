defmodule Farmbot.Firmware.UartHandler do
  @moduledoc """
  Handles communication between farmbot and uart devices
  """

  use GenStage
  alias Farmbot.Firmware
  use Farmbot.Logger
  alias Farmbot.System.ConfigStorage
  import ConfigStorage, only: [update_config_value: 4, get_config_value: 3]
  @behaviour Firmware.Handler

  def start_link do
    GenStage.start_link(__MODULE__, [])
  end

  def move_absolute(handler, pos, x_speed, y_speed, z_speed) do
    GenStage.call(handler, {:move_absolute, pos, x_speed, y_speed, z_speed})
  end

  def calibrate(handler, axis) do
    GenStage.call(handler, {:calibrate, axis})
  end

  def find_home(handler, axis) do
    GenStage.call(handler, {:find_home, axis})
  end

  def home(handler, axis) do
    GenStage.call(handler, {:home, axis})
  end

  def home_all(handler) do
    GenStage.call(handler, :home_all)
  end

  def zero(handler, axis) do
    GenStage.call(handler, {:zero, axis})
  end

  def update_param(handler, param, val) do
    GenStage.call(handler, {:update_param, param, val})
  end

  def read_param(handler, param) do
    GenStage.call(handler, {:read_param, param})
  end

  def read_all_params(handler) do
    GenStage.call(handler, :read_all_params)
  end

  def emergency_lock(handler) do
    GenStage.call(handler, :emergency_lock)
  end

  def emergency_unlock(handler) do
    GenStage.call(handler, :emergency_unlock)
  end

  def set_pin_mode(handler, pin, mode) do
    GenStage.call(handler, {:set_pin_mode, pin, mode})
  end

  def read_pin(handler, pin, pin_mode) do
    GenStage.call(handler, {:read_pin, pin, pin_mode})
  end

  def write_pin(handler, pin, pin_mode, value) do
    GenStage.call(handler, {:write_pin, pin, pin_mode, value})
  end

  def request_software_version(handler) do
    GenStage.call(handler, :request_software_version)
  end

  def set_servo_angle(handler, pin, number) do
    GenStage.call(handler, {:set_servo_angle, pin, number})
  end

  def init(_) do
    update_config_value(:bool, "settings", "firmware_input_log", false)
    hw = get_config_value(:string, "settings", "firmware_hardware")
    tty = Application.get_env(:farmbot, :uart_handler)[:tty] |> to_charlist()
    {:ok, res} = start_handler(tty)
    :ok = poll(res)
    gen_stage_opts = [
      dispatcher: GenStage.BroadcastDispatcher,
      subscribe_to: [ConfigStorage.Dispatcher]
    ]
    {:producer_consumer, %{res: res}, gen_stage_opts}
  end

  @doc false
  def terminate(_, state) do
    if state.res do
      stop_handler(state.res)
    end
  end

  def handle_events(events, _, state) do
    # state = Enum.reduce(events, state, fn(event, state_acc) ->
    #   handle_config(event, state_acc)
    # end)
    #
    # case state do
    #   %State{} = state ->
    #     {:noreply, [], state}
    #   _ -> state
    # end
    {:noreply, [], state}
  end

  def handle_info({:select, res, _ref, :ready_input}, state) do
    {time, raw_input} = :timer.tc(fn -> receive_input(res) end)
    case raw_input do
      {:error, reason} -> {:stop, {:input_error, reason}, state}
      input when is_binary(input) ->
        :ok = poll(res)
        Logger.debug 1, "read #{byte_size(input)} bytes in #{time}µs: #{inspect input}"
        {:noreply, [], state}
      _ -> {:noreply, [], state}
    end
  end

  def handle_call(_, _, state) do
    {:reply, {:error, :not_implemented}, state}
  end

  @on_load :load_nif
  @doc false
  def load_nif do
    require Elixir.Logger
    nif_file = '#{:code.priv_dir(:farmbot)}/firmware_nif'
    case :erlang.load_nif(nif_file, 0) do
      :ok -> :ok
      {:error, {:reload, _}} -> :ok
      {:error, reason} -> Elixir.Logger.warn "Failed to load nif: #{inspect reason}"
    end
  end

  ## These functions get replaced by the nif.
  @doc false
  def start_handler(_device), do: do_exit_no_nif()
  @doc false
  def stop_handler(_handler), do: do_exit_no_nif()
  @doc false
  def poll(_handler), do: do_exit_no_nif()
  @doc false
  def receive_input(_handler), do: do_exit_no_nif()

  defp do_exit_no_nif, do: exit("nif not loaded.")
end
