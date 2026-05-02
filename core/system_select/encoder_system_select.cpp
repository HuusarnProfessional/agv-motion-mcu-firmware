#include "core/system_select/system_select.hpp"

#include "core/api/encoder_api.hpp"
#include "core/impl/encoder_as5600_pwm_impl.hpp"

namespace
{
  void init_encoder_as5600_pwm(const encoder_api::encoder_input *encoders, std::size_t count)
  {
    encoder_as5600_pwm_impl::init(encoders, count);
  }

  encoder_api::encoder_status map_encoder_status(encoder_as5600_pwm_impl::sample_status status)
  {
    switch (status)
    {
      case encoder_as5600_pwm_impl::sample_status::ok:
        return encoder_api::encoder_status::ok;
      case encoder_as5600_pwm_impl::sample_status::no_signal:
        return encoder_api::encoder_status::no_signal;
      case encoder_as5600_pwm_impl::sample_status::stale:
        return encoder_api::encoder_status::stale;
      case encoder_as5600_pwm_impl::sample_status::invalid_duty:
        return encoder_api::encoder_status::invalid_duty;
      case encoder_as5600_pwm_impl::sample_status::invalid_id:
        return encoder_api::encoder_status::invalid_id;
      default:
        return encoder_api::encoder_status::stale;
    }
  }

  bool read_encoder_as5600_pwm(std::uint8_t encoder_id, encoder_api::encoder_sample &out)
  {
    encoder_as5600_pwm_impl::sample impl_sample{};
    const bool ok = encoder_as5600_pwm_impl::read_sample(encoder_id, impl_sample);

    out.angle_raw_12bit = impl_sample.angle_raw_12bit;
    out.sample_id = impl_sample.sample_id;
    out.time_ms = impl_sample.time_ms;
    out.status = map_encoder_status(impl_sample.status);
    return ok;
  }
}

