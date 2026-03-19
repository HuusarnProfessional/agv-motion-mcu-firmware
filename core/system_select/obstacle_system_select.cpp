#include "core/system_select/system_select.hpp"

#include "core/api/obstacle_api.hpp"
#include "core/impl/obstacle_hcsr04_impl.hpp"

namespace
{
  void init_obstacle_hcsr04(const obstacle_api::obstacle_input *inputs, std::size_t count)
  {
    obstacle_hcsr04_impl::init(inputs, count);
  }

  obstacle_api::obstacle_status map_obstacle_status(obstacle_hcsr04_impl::sample_status status)
  {
    switch (status)
    {
      case obstacle_hcsr04_impl::sample_status::ok:
        return obstacle_api::obstacle_status::ok;
      case obstacle_hcsr04_impl::sample_status::timeout:
        return obstacle_api::obstacle_status::timeout;
      case obstacle_hcsr04_impl::sample_status::stale:
        return obstacle_api::obstacle_status::stale;
      case obstacle_hcsr04_impl::sample_status::no_signal:
        return obstacle_api::obstacle_status::no_signal;
      case obstacle_hcsr04_impl::sample_status::out_of_range:
        return obstacle_api::obstacle_status::out_of_range;
      case obstacle_hcsr04_impl::sample_status::invalid_id:
        return obstacle_api::obstacle_status::invalid_id;
      default:
        return obstacle_api::obstacle_status::stale;
    }
  }

  bool read_obstacle_hcsr04(std::uint8_t sensor_id, obstacle_api::obstacle_sample &out)
  {
    obstacle_hcsr04_impl::sample impl_sample = {};
    const bool ok = obstacle_hcsr04_impl::read_sample(sensor_id, impl_sample);

    out.distance_mm = impl_sample.distance_mm;
    out.time_ms = impl_sample.time_ms;
    out.status = map_obstacle_status(impl_sample.status);

    return ok;
  }
}
