#include "core/impl/obstacle_hcsr04_impl.hpp"

namespace
{
  constexpr std::size_t k_max_sensors = 16U;
  constexpr std::uint32_t k_trigger_pulse_width_us = 10U;
  constexpr std::uint32_t k_default_echo_timeout_us = 60000U;
  constexpr std::uint32_t k_min_measurement_interval_us = 60000U;
  constexpr std::uint32_t k_min_distance_mm = 20U;
  constexpr std::uint32_t k_min_valid_pulse_us = (2U * k_min_distance_mm * 1000U) / 343U;
  constexpr std::uint32_t k_max_distance_mm = 4000U;
  constexpr std::uint32_t k_max_valid_pulse_us = (2U * k_max_distance_mm * 1000U) / 343U;
  constexpr std::uint32_t k_distance_numerator = 343U;
  constexpr std::uint32_t k_distance_denominator = 2000U;
  constexpr std::uint8_t k_invalid_sensor_id = 0xFFU;

  enum class scheduler_phase : std::uint8_t
  {
    stopped = 0U,
    trigger_high,
    waiting_for_echo,
    waiting_for_next_measurement
  };

  struct sensor_runtime
  {
    std::uint32_t latest_distance_mm = 0U;
    std::uint32_t latest_time_ms = 0U;
    obstacle_hcsr04_impl::sample_status latest_status = obstacle_hcsr04_impl::sample_status::no_signal;
    bool has_completed_measurement = false;
  };

  struct measurement_scheduler
  {
    scheduler_phase phase = scheduler_phase::stopped;
    std::uint8_t active_sensor_id = k_invalid_sensor_id;
    std::uint8_t scheduled_sensor_id = k_invalid_sensor_id;
  };

  const obstacle_api::obstacle_input *global_input = nullptr;
  std::size_t global_input_count = 0U;
  sensor_runtime global_sensor_runtime[k_max_sensors] = {};
  measurement_scheduler global_scheduler = {};

  void reset_sensor_runtime(sensor_runtime &selected_sensor_runtime)
  {
    selected_sensor_runtime = {};
    selected_sensor_runtime.latest_status = obstacle_hcsr04_impl::sample_status::no_signal;
  }

  void stop_scheduler(void)
  {
    global_scheduler = {};
    global_scheduler.phase = scheduler_phase::stopped;
    global_scheduler.active_sensor_id = k_invalid_sensor_id;
    global_scheduler.scheduled_sensor_id = k_invalid_sensor_id;
  }

  void clear_global_state(void)
  {
    global_input = nullptr;
    global_input_count = 0U;
    stop_scheduler();

    for (std::size_t sensor_index = 0U; sensor_index < k_max_sensors; ++sensor_index)
    {
      reset_sensor_runtime(global_sensor_runtime[sensor_index]);
    }
  }

  std::uint32_t select_echo_timeout_us(const obstacle_api::obstacle_input &selected_sensor)
  {
    if (selected_sensor.echo_timeout_us == 0U)
    {
      return k_default_echo_timeout_us;
    }

    return selected_sensor.echo_timeout_us;
  }

  bool convert_pulse_us_to_distance_mm(std::uint32_t pulse_width_us, std::uint32_t &distance_mm_out)
  {
    if (pulse_width_us < k_min_valid_pulse_us || pulse_width_us > k_max_valid_pulse_us)
    {
      return false;
    }

    const std::uint32_t scaled_pulse = pulse_width_us * k_distance_numerator;
    const std::uint32_t rounded_pulse = scaled_pulse + (k_distance_denominator / 2U);
    distance_mm_out = rounded_pulse / k_distance_denominator;
    return true;
  }

  bool validate_input(const obstacle_api::obstacle_input *input, std::size_t count)
  {
    if (input == nullptr || count == 0U || count > k_max_sensors)
    {
      return false;
    }

    for (std::size_t sensor_index = 0U; sensor_index < count; ++sensor_index)
    {
      const obstacle_api::ultrasonic_operations *selected_operations = input[sensor_index].platform_operations;

      if (selected_operations == nullptr)
      {
        return false;
      }

      if (selected_operations->register_event_callback == nullptr || selected_operations->set_trigger_level == nullptr || selected_operations->schedule_alarm_us == nullptr || selected_operations->cancel_alarm == nullptr || selected_operations->read_time_ms == nullptr)
      {
        return false;
      }
    }

    return true;
  }

  bool read_current_time_ms(const obstacle_api::obstacle_input &selected_sensor, std::uint32_t &current_time_ms_out)
  {
    return selected_sensor.platform_operations->read_time_ms(selected_sensor.platform_handle, current_time_ms_out);
  }

  bool set_trigger_level(const obstacle_api::obstacle_input &selected_sensor, bool is_high)
  {
    return selected_sensor.platform_operations->set_trigger_level(selected_sensor.platform_handle, selected_sensor.channel, is_high);
  }

  bool schedule_alarm_us(const obstacle_api::obstacle_input &selected_sensor, std::uint32_t delay_us)
  {
    return selected_sensor.platform_operations->schedule_alarm_us(selected_sensor.platform_handle, delay_us);
  }

  void cancel_alarm(const obstacle_api::obstacle_input &selected_sensor)
  {
    selected_sensor.platform_operations->cancel_alarm(selected_sensor.platform_handle);
  }

  std::uint8_t select_next_sensor_id(std::uint8_t previous_sensor_id)
  {
    if (global_input_count <= 1U)
    {
      return previous_sensor_id;
    }

    return static_cast<std::uint8_t>((previous_sensor_id + 1U) % global_input_count);
  }

  std::uint8_t find_sensor_id(void *platform_handle, std::uint8_t channel)
  {
    if (global_input == nullptr)
    {
      return k_invalid_sensor_id;
    }

    for (std::size_t sensor_index = 0U; sensor_index < global_input_count; ++sensor_index)
    {
      const obstacle_api::obstacle_input &selected_sensor = global_input[sensor_index];

      if (selected_sensor.platform_handle == platform_handle && selected_sensor.channel == channel)
      {
        return static_cast<std::uint8_t>(sensor_index);
      }
    }

    return k_invalid_sensor_id;
  }

  void store_no_signal(sensor_runtime &selected_sensor_runtime)
  {
    selected_sensor_runtime.latest_status = obstacle_hcsr04_impl::sample_status::no_signal;
  }

  void store_timeout(sensor_runtime &selected_sensor_runtime, std::uint32_t current_time_ms)
  {
    selected_sensor_runtime.latest_time_ms = current_time_ms;
    selected_sensor_runtime.latest_status = obstacle_hcsr04_impl::sample_status::timeout;
    selected_sensor_runtime.has_completed_measurement = true;
  }

  void store_completed_measurement(sensor_runtime &selected_sensor_runtime, std::uint32_t pulse_width_us, std::uint32_t current_time_ms)
  {
    std::uint32_t distance_mm = 0U;

    selected_sensor_runtime.latest_time_ms = current_time_ms;
    selected_sensor_runtime.has_completed_measurement = true;

    if (!convert_pulse_us_to_distance_mm(pulse_width_us, distance_mm))
    {
      selected_sensor_runtime.latest_status = obstacle_hcsr04_impl::sample_status::out_of_range;
      return;
    }

    selected_sensor_runtime.latest_distance_mm = distance_mm;
    selected_sensor_runtime.latest_status = obstacle_hcsr04_impl::sample_status::ok;
  }

  bool schedule_next_measurement(std::uint8_t previous_sensor_id)
  {
    const std::uint8_t next_sensor_id = select_next_sensor_id(previous_sensor_id);
    const obstacle_api::obstacle_input &selected_sensor = global_input[next_sensor_id];

    if (!schedule_alarm_us(selected_sensor, k_min_measurement_interval_us))
    {
      stop_scheduler();
      return false;
    }

    global_scheduler.phase = scheduler_phase::waiting_for_next_measurement;
    global_scheduler.active_sensor_id = k_invalid_sensor_id;
    global_scheduler.scheduled_sensor_id = next_sensor_id;
    return true;
  }

  bool begin_measurement(std::uint8_t sensor_id)
  {
    if (sensor_id >= global_input_count)
    {
      return false;
    }

    const obstacle_api::obstacle_input &selected_sensor = global_input[sensor_id];

    if (!set_trigger_level(selected_sensor, false) || !set_trigger_level(selected_sensor, true) || !schedule_alarm_us(selected_sensor, k_trigger_pulse_width_us))
    {
      return false;
    }

    global_scheduler.phase = scheduler_phase::trigger_high;
    global_scheduler.active_sensor_id = sensor_id;
    global_scheduler.scheduled_sensor_id = k_invalid_sensor_id;
    return true;
  }

  void handle_trigger_alarm(void *platform_handle)
  {
    if (global_scheduler.active_sensor_id == k_invalid_sensor_id || global_scheduler.active_sensor_id >= global_input_count)
    {
      stop_scheduler();
      return;
    }

    const obstacle_api::obstacle_input &selected_sensor = global_input[global_scheduler.active_sensor_id];

    if (selected_sensor.platform_handle != platform_handle)
    {
      return;
    }

    if (!set_trigger_level(selected_sensor, false) || !schedule_alarm_us(selected_sensor, select_echo_timeout_us(selected_sensor)))
    {
      store_no_signal(global_sensor_runtime[global_scheduler.active_sensor_id]);
      schedule_next_measurement(global_scheduler.active_sensor_id);
      return;
    }

    global_scheduler.phase = scheduler_phase::waiting_for_echo;
  }

  void handle_timeout_alarm(void *platform_handle)
  {
    if (global_scheduler.active_sensor_id == k_invalid_sensor_id || global_scheduler.active_sensor_id >= global_input_count)
    {
      stop_scheduler();
      return;
    }

    const obstacle_api::obstacle_input &selected_sensor = global_input[global_scheduler.active_sensor_id];

    if (selected_sensor.platform_handle != platform_handle)
    {
      return;
    }

    std::uint32_t current_time_ms = 0U;

    if (!read_current_time_ms(selected_sensor, current_time_ms))
    {
      store_no_signal(global_sensor_runtime[global_scheduler.active_sensor_id]);
    }
    else
    {
      store_timeout(global_sensor_runtime[global_scheduler.active_sensor_id], current_time_ms);
    }

    schedule_next_measurement(global_scheduler.active_sensor_id);
  }

  void handle_next_measurement_alarm(void *platform_handle)
  {
    if (global_scheduler.scheduled_sensor_id == k_invalid_sensor_id || global_scheduler.scheduled_sensor_id >= global_input_count)
    {
      stop_scheduler();
      return;
    }

    const std::uint8_t scheduled_sensor_id = global_scheduler.scheduled_sensor_id;
    const obstacle_api::obstacle_input &selected_sensor = global_input[scheduled_sensor_id];

    if (selected_sensor.platform_handle != platform_handle)
    {
      return;
    }

    if (!begin_measurement(scheduled_sensor_id))
    {
      store_no_signal(global_sensor_runtime[scheduled_sensor_id]);
      schedule_next_measurement(scheduled_sensor_id);
    }
  }

  void handle_alarm_elapsed(void *platform_handle)
  {
    if (global_input == nullptr || global_input_count == 0U)
    {
      return;
    }

    if (global_scheduler.phase == scheduler_phase::trigger_high)
    {
      handle_trigger_alarm(platform_handle);
      return;
    }

    if (global_scheduler.phase == scheduler_phase::waiting_for_echo)
    {
      handle_timeout_alarm(platform_handle);
      return;
    }

    if (global_scheduler.phase == scheduler_phase::waiting_for_next_measurement)
    {
      handle_next_measurement_alarm(platform_handle);
    }
  }

  void handle_capture_ready(const obstacle_api::ultrasonic_platform_event &event)
  {
    const std::uint8_t sensor_id = find_sensor_id(event.platform_handle, event.channel);

    if (sensor_id == k_invalid_sensor_id || sensor_id >= global_input_count)
    {
      return;
    }

    if (global_scheduler.phase != scheduler_phase::waiting_for_echo || global_scheduler.active_sensor_id != sensor_id)
    {
      return;
    }

    const obstacle_api::obstacle_input &selected_sensor = global_input[sensor_id];
    cancel_alarm(selected_sensor);
    store_completed_measurement(global_sensor_runtime[sensor_id], event.pulse_width_us, event.time_ms);
    schedule_next_measurement(sensor_id);
  }

  void handle_platform_event(void *event_context, const obstacle_api::ultrasonic_platform_event *event)
  {
    (void)event_context;

    if (event == nullptr)
    {
      return;
    }

    if (event->type == obstacle_api::ultrasonic_platform_event_type::alarm_elapsed)
    {
      handle_alarm_elapsed(event->platform_handle);
      return;
    }

    if (event->type == obstacle_api::ultrasonic_platform_event_type::capture_ready)
    {
      handle_capture_ready(*event);
    }
  }

  bool register_platform_callbacks(const obstacle_api::obstacle_input *input, std::size_t count)
  {
    for (std::size_t sensor_index = 0U; sensor_index < count; ++sensor_index)
    {
      bool already_registered = false;

      for (std::size_t previous_index = 0U; previous_index < sensor_index; ++previous_index)
      {
        if (input[previous_index].platform_handle == input[sensor_index].platform_handle)
        {
          already_registered = true;
          break;
        }
      }

      if (already_registered)
      {
        continue;
      }

      if (!input[sensor_index].platform_operations->register_event_callback(input[sensor_index].platform_handle, handle_platform_event, nullptr))
      {
        return false;
      }
    }

    return true;
  }
}

namespace obstacle_hcsr04_impl
{
  void init(const obstacle_api::obstacle_input *input, std::size_t count)
  {
    clear_global_state();

    if (!validate_input(input, count))
    {
      return;
    }

    global_input = input;
    global_input_count = count;

    if (!register_platform_callbacks(input, count))
    {
      clear_global_state();
      return;
    }

    begin_measurement(0U);
  }

  bool read_sample(std::uint8_t sensor_id, sample &out)
  {
    out = {};

    if (global_input == nullptr || global_input_count == 0U)
    {
      out.status = sample_status::no_signal;
      return false;
    }

    if (sensor_id >= global_input_count)
    {
      out.status = sample_status::invalid_id;
      return false;
    }

    const sensor_runtime &selected_sensor_runtime = global_sensor_runtime[sensor_id];

    if (!selected_sensor_runtime.has_completed_measurement)
    {
      out.status = sample_status::no_signal;
      return false;
    }

    out.distance_mm = selected_sensor_runtime.latest_distance_mm;
    out.time_ms = selected_sensor_runtime.latest_time_ms;
    out.status = selected_sensor_runtime.latest_status;

    if (global_scheduler.active_sensor_id == sensor_id && selected_sensor_runtime.latest_status == sample_status::ok)
    {
      out.status = sample_status::stale;
    }

    return out.status == sample_status::ok;
  }
}
