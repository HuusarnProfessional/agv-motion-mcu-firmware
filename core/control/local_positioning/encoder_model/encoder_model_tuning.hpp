#pragma once

#include <cstdint>

namespace encoder_model_tuning
{
  struct motion_model_tuning
  {
    // status-based weighting
    std::int64_t weight_ok_per_mille = 1000;
    std::int64_t weight_stale_per_mille = 500;
    std::int64_t weight_bad_per_mille = 0;

    // per-wheel base scaling
    std::int64_t weight_fl = 1000;
    std::int64_t weight_fr = 1000;
    std::int64_t weight_rl = 1000;
    std::int64_t weight_rr = 1000;

    // unit scaling
    std::int64_t um_per_mm = 1000;
    std::int64_t urad_per_rad = 1000000;
    std::int64_t d_ref_um = 1000;

    // fallback confidence per case (per-mille)
    std::int64_t confidence_translation_all_ok = 1000;
    std::int64_t confidence_translation_one_bad = 800;
    std::int64_t confidence_translation_two_bad_same_axle = 650;
    std::int64_t confidence_translation_two_bad_mixed_axle = 550;
    std::int64_t confidence_translation_two_bad_same_side = 500;
    std::int64_t confidence_translation_three_bad = 300;
    std::int64_t confidence_translation_four_bad = 0;
    

    std::int64_t confidence_rotation_all_ok = 1000;
    std::int64_t confidence_rotation_one_bad = 700;
    std::int64_t confidence_rotation_two_bad_same_axle = 500;
    std::int64_t confidence_rotation_two_bad_mixed_axle = 400;
    std::int64_t confidence_rotation_two_bad_same_side = 0;
    std::int64_t confidence_rotation_three_bad = 0;
    std::int64_t confidence_rotation_four_bad = 0;

  };
}
