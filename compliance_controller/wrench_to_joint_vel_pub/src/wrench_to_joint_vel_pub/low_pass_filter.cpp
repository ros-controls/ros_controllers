#include <wrench_to_joint_vel_pub/low_pass_filter.h>

namespace wrench_to_joint_vel_pub
{
LowPassFilter::LowPassFilter(double filter_param) : filter_param_(filter_param)
{
}

double LowPassFilter::filter(const double new_msrmt)
{
  // Push in the new measurement
  prev_msrmts_[2] = prev_msrmts_[1];
  prev_msrmts_[1] = prev_msrmts_[0];
  prev_msrmts_[0] = new_msrmt;

  double new_filtered_msrmt = (1 / (1 + filter_param_ * filter_param_ + 1.414 * filter_param_)) *
                              (prev_msrmts_[2] + 2 * prev_msrmts_[1] + prev_msrmts_[0] -
                               (filter_param_ * filter_param_ - 1.414 * filter_param_ + 1) * prev_filtered_msrmts_[1] -
                               (-2 * filter_param_ * filter_param_ + 2) * prev_filtered_msrmts_[0]);

  // Store the new filtered measurement
  prev_filtered_msrmts_[1] = prev_filtered_msrmts_[0];
  prev_filtered_msrmts_[0] = new_filtered_msrmt;

  return new_filtered_msrmt;
}

void LowPassFilter::reset(double data)
{
  prev_msrmts_ = { data, data, data };
  prev_filtered_msrmts_ = { data, data };
}
}  // namespace wrench_to_joint_vel_pub
