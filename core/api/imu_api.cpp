#include "core/api/imu_api.hpp"
#include "core/system_select/system_select.hpp"

namespace
{
    imu_api::backend_operation g_backend = {};
    bool g_backend_ready = false;
}

namespace imu_api
{
    void init(const imu_input *input, std::size_t count)
    {
        g_backend = {};
        g_backend_ready = false;

        if (input == nullptr || count == 0u)
        {
            return;
        }
        system_select::select_imu_backend(g_backend);

        if(g_backend.init_fn == nullptr || g_backend.read_sample_fn == nullptr)
        {
            return;
        }

        g_backend.init_fn(input, count);
        g_backend_ready = true;
    }

    bool read_sample(std::uint8_t imu_id, imu_sample &out)
    {
        if(!g_backend_ready || g_backend.read_sample_fn == nullptr)
        {
            return false;
        }

        return g_backend.read_sample_fn(imu_id, out);
    }
}
