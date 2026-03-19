#include "core/impl/encoder_as5600_pwm_impl.hpp"

namespace encoder_as5600_pwm_impl
{
  
  static const encoder_api::encoder_input *global_encoders = nullptr; //pointer to encoders adress
  static  std::size_t global_encoders_count = 0U; //number of encoders 


  void init(const encoder_api::encoder_input *encoders, std::size_t count)
  {
    if(encoders == nullptr || count == 0U) //check if we get anything
    {
      //if we get nothing but still have stuff from last run
      global_encoders = nullptr; 
      global_encoders_count = 0U;

      return;
    } 

    //assigns the values to global variables
    global_encoders = encoders;  
    global_encoders_count = count;
    
  }

  bool read_sample(std::uint8_t encoder_id, sample &out)
  {
    if(global_encoders == nullptr || global_encoders_count == 0U) //check if we have any encoders
    {
      out.angle_raw_12bit = 0U;
      out.time_ms = 0U;
      out.status = sample_status::no_signal;
      return false;
    }

    if(encoder_id >= global_encoders_count) // if you ask for more encoders then we have on the agv
    {
      out.angle_raw_12bit = 0U;
      out.time_ms = 0U;
      out.status = sample_status::invalid_id;
      return false;
    }

    


    const encoder_api::encoder_input &encoder = global_encoders[encoder_id]; // gets wrapped encoder operations
    if (encoder.platform_operations == nullptr)
    {
      out.angle_raw_12bit = 0U;
      out.time_ms = 0U;
      out.status = sample_status::no_signal;
      return false;
    }
    if (encoder.platform_operations->read_capture == nullptr)
    {
      out.angle_raw_12bit = 0U;
      out.time_ms = 0U;
      out.status = sample_status::no_signal;
      return false;
    }

    std::uint32_t high_ticks = 0U;
    std::uint32_t period_ticks = 0U;
    std::uint32_t time_ms = 0U;

    const bool got_capture = encoder.platform_operations->read_capture(encoder.platform_handle, encoder.channel, high_ticks, period_ticks, time_ms);


    if(got_capture == false)
    {
      out.time_ms = 0U;
      out.angle_raw_12bit = 0U;
      out.status = sample_status::no_signal;
      return false;
    }
    if(period_ticks == 0U || high_ticks > period_ticks) 
    {
      out.angle_raw_12bit = 0U;
      out.time_ms = time_ms;
      out.status = sample_status::invalid_duty;
      return false;
    }

    const std::uint32_t raw =(high_ticks * 4095U) / period_ticks; //calculate duty cycle, scaled to 4095(12bit)
    out.angle_raw_12bit = static_cast<std::uint16_t>(raw);
    out.time_ms = time_ms;
    out.status = sample_status::ok;
    return true;

  }




}
