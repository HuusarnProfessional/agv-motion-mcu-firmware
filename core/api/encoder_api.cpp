#include "core/api/encoder_api.hpp"
#include "core/system_select/system_select.hpp"

// Controller-owned calculations:
// - continuous angle
// - angular velocity
// - reset/rebase policy
namespace
{
  encoder_api::backend_operation g_backend{};
  bool g_backend_ready = false;

  std::uint32_t raw12_to_mdeg(std::uint16_t raw)
  {
    return (raw * 360000U + 2047U) / 4095U;
  }

  std::uint32_t raw12_to_mrad(std::uint16_t raw)
  {
    // 2*pi in urad = 6,283,185
    // Split to keep 32-bit math safe:
    // 6,283,185 = 4,095 * 1,534 + 1,455
    const std::uint32_t r = raw;

    const std::uint32_t urad =
        (r * 1534U) +
        ((r * 1455U + 2047U) / 4095U); // rounded remainder term

    return (urad + 500U) / 1000U; // rounded urad -> mrad
  }
}

namespace encoder_api
{
  void init(const encoder_input *encoders, std::size_t count)
  {
    g_backend = {};
    g_backend_ready = false;

    if (encoders == nullptr || count == 0U)
    {
      return;
    }

    system_select::select_encoder_backend(g_backend);
    if (g_backend.init_fn == nullptr || g_backend.read_sample_fn == nullptr)
    {
      return;
    }

    g_backend.init_fn(encoders, count);
    g_backend_ready = true;
  }

  bool read_sample(std::uint8_t encoder_id, encoder_sample &out)
  {
    if (!g_backend_ready || g_backend.read_sample_fn == nullptr)
    {
      out.angle_raw_12bit = 0U;
      out.angle_mdeg = 0U;
      out.angle_mrad = 0U;
      out.time_ms = 0U;
      out.status = encoder_status::stale;
      return false;
    }

    const bool got_sample = g_backend.read_sample_fn(encoder_id, out);
    out.angle_mdeg = raw12_to_mdeg(out.angle_raw_12bit);
    out.angle_mrad = raw12_to_mrad(out.angle_raw_12bit);
    return got_sample;
  }
}
