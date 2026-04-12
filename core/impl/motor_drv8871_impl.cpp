#include "core/impl/motor_drv8871_impl.hpp"



namespace motor_drv8871_impl
{

	static const motor_api::motor_pwm2 *g_motors = nullptr; //local pointer for motor adress
	static std::size_t g_count =0; //number of motors

	static constexpr std::size_t k_max_motors = 16; // known value before compl and max 16 motors
	static std::int16_t g_last_u[k_max_motors] = {}; // // zero from start, g_last_u is for rotational change

	static std::int16_t clamp_u(std::int16_t u) //max sure u is between 1000(100% forward) and -1000(100% backward)
	{
		if(u > 1000)
		{
			return 1000;
		}
		else if(u < -1000)
		{
			return -1000;
		}
		else
		{
			return u;
		}
	}




	static bool sign_changed(std::int16_t a, std::int16_t b) //check if we go from forward to backward or the reverse
	{
		return (a > 0 && b < 0) || (a < 0 && b > 0);
	}

	static void pwm_start(const motor_api::pwm_out &out)
	{
		if (!out.platform_operations) //no backend
		{
			return;
		}
	    if (!out.platform_operations->start) // no start function
	    {
	    	return;
	    }
	    out.platform_operations->start(out.platform_handle, out.channel); // call the board pwm start function
	}

	static void pwm_set_compare(const motor_api::pwm_out &out, std::uint32_t ccr)
	{
		if (!out.platform_operations) //no backend
		{
			return;
		}
		if (!out.platform_operations->set_compare) // no set_compare
		{
			return;
		}
		out.platform_operations->set_compare(out.platform_handle, out.channel, ccr); // set compare (duty)
	}

	static std::uint32_t pwm_get_period_top(const motor_api::pwm_out &out)
	{
		if (!out.platform_operations) //no backend
		{
			return 0;
		}
		if (!out.platform_operations->get_period) //no get_period
		{
			return 0;
		}

		return out.platform_operations->get_period(out.platform_handle); // return timer top (period in ticks)
	}


	static std::uint32_t u_to_ccr(std::uint32_t top, std::uint16_t abs_u)
	{
		if (top == 0) // invalid period
		{
			return 0;
		}
		if (abs_u >= 1000) // full scale
		{
			return top;
		}

		const std::uint64_t num = static_cast<std::uint64_t>(abs_u) * static_cast<std::uint64_t>(top);

		return static_cast<std::uint32_t>(num / 1000ULL); // scale to CCR (ULL = 64bit)
	}

	static void coast(const motor_api::motor_pwm2 &m) // both low = coast
	{
		pwm_set_compare(m.pwm_a, 0);
		pwm_set_compare(m.pwm_b, 0);
	}

	//get motor address and number from board
		void init(const motor_api::motor_pwm2 *motors, std::size_t count)
		{
			g_motors = motors;

			if (count > k_max_motors) // clip to our storage
			{
				g_count = k_max_motors;
			}
			else
			{
				g_count = count;
			}
			for(std::size_t i = 0; i < g_count; ++i)
			{
				g_last_u[i] = 0;

				pwm_start(g_motors[i].pwm_a); //tart both PWM:s
				pwm_start(g_motors[i].pwm_b);

				coast(g_motors[i]); //safe
			}
		}

	void set_u(std::uint8_t motor_id, std::int16_t u)
	{
		//safety check
		if(!g_motors)
		{
			return;
		}
		if(motor_id >= g_count)
		{
			return;
		}

		u = clamp_u(u);

		std::int16_t &last = g_last_u[motor_id];
		if (sign_changed(last, u)) // force a stop between direction changes
		{
			coast(g_motors[motor_id]);
			last = 0;
			return;
		}
		const motor_api::motor_pwm2 &m = g_motors[motor_id]; //pick motor

		const std::uint32_t top_a = pwm_get_period_top(m.pwm_a); //period tick for a
		const std::uint32_t top_b = pwm_get_period_top(m.pwm_b); //period tick for b


		if (top_a == 0 || top_b == 0) // invalid timer period
		{
		  coast(m);
		  last = 0;
		  return;
		}
		//check that a and b are on same timer(forward and reverse would scale differently)
		if (top_a != top_b)
		{
		  coast(m);
		  last = 0;
		  return;
		}

		const std::uint32_t top = top_a;
		std::int16_t signed_abs_u = u;
		if (u < 0)
		{
			signed_abs_u = static_cast<std::int16_t>(-u);
		}

		const std::uint16_t abs_u = static_cast<std::uint16_t>(signed_abs_u); //make it to abs value as +- is rotation direction value
		const std::uint32_t ccr = u_to_ccr(top, abs_u); // compare value(~duty)

		if (u > 0) // forward: pwm_a active, pwm_b low
		{
			pwm_set_compare(m.pwm_a, ccr);
			pwm_set_compare(m.pwm_b, 0);
		}
		else if (u < 0) // reverse: pwm_b active, pwm_a low
		{
			pwm_set_compare(m.pwm_a, 0);
			pwm_set_compare(m.pwm_b, ccr);
		}
		else // stop: coast
		{
			coast(m);
		}

		last = u;

	}
}

