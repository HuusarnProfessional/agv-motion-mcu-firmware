#include "core/api/button_api.hpp"

#include "core/system_select/system_select.hpp"

namespace
{
  button_api::backend_operation global_backend = {};
  bool global_backend_ready = false;
}

namespace button_api
{
  void init(const button_input *inputs, std::size_t count)
  {
    global_backend = {};
    global_backend_ready = false;

    if (inputs == nullptr || count == 0U)
    {
      return;
    }

    system_select::select_button_backend(global_backend);

    if (global_backend.init_fn == nullptr || global_backend.read_sample_fn == nullptr)
    {
      return;
    }

    global_backend.init_fn(inputs, count);
    global_backend_ready = true;
  }

  bool read_sample(std::uint8_t button_id, button_sample &out)
  {
    if (!global_backend_ready || global_backend.read_sample_fn == nullptr)
    {
      out = {};
      out.status = button_status::no_signal;
      return false;
    }

    return global_backend.read_sample_fn(button_id, out);
  }
}
