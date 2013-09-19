/*
 *  Copyright (c) 2010 PAL Robotics sl. All Rights Reserved
 *  Created on: 10 Mar 2010
 *      Author: luca
 *			 email: luca.marchionni@pal-robotics.com
 *  Refactored & changed on: 18 Sept 2013
 *      Author: Bence Magyar
 *			 email: bence.magyar@pal-robotics.com
 */
#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <ros/time.h>
#include <geometry_msgs/Point32.h>
#include <cmath>

namespace diff_drive_controller{
  /**
   * @brief The Odometry class handles odometry readings
   *  (2D oriented position with related timestamp)
   */
  class Odometry
  {
  public:
    /**
      * Timestamp will get the current time value
      * Value will be set to zero
      */
    Odometry():
      timestamp_(0),
      wheel_separation_(0.0),
      wheel_radius_(0.0),
      left_wheel_old_pos_(0.0),
      right_wheel_old_pos_(0.0),
      linear_est_speed_(0.0),
      angular_est_speed_(0.0)
    {
      value_.x = value_ .y = value_.z = 0.0;
    }

    /**
     * @brief update the odometry class with latest wheels position
     * @param left_pos
     * @param right_pos
     * @param time
     * @return
     */
    bool update(double left_pos, double right_pos, const ros::Time &time)
    {
      // get current wheel joint positions
      const double left_wheel_cur_pos_ = left_pos*wheel_radius_;
      const double right_wheel_cur_pos_ = right_pos*wheel_radius_;
      // estimate velocity of wheels using old and current position
      const double left_wheel_est_vel_ = left_wheel_cur_pos_ - left_wheel_old_pos_;
      const double right_wheel_est_vel_ = right_wheel_cur_pos_ - right_wheel_old_pos_;
      // update old position with current
      left_wheel_old_pos_ = left_wheel_cur_pos_;
      right_wheel_old_pos_ = right_wheel_cur_pos_;

      // compute linear and angular diff
      const double linear = (left_wheel_est_vel_ + right_wheel_est_vel_) * 0.5 ;
      const double angular = (right_wheel_est_vel_ - left_wheel_est_vel_) / wheel_separation_;

      if((fabs(angular) < 1e-15) && (fabs(linear) < 1e-15)) // when both velocities are ~0
      {
        return false;
      }

      // integrate
      const double deltaTime = (time - timestamp_).toSec();
      if(deltaTime < 0.0001)
        return false; // interval too small to integrate with

      timestamp_ = time;
      integrationByRungeKutta(linear, angular);

      // estimate speeds
      speedEstimation(linear/deltaTime, angular/deltaTime);

      return true;
    }

    double getHeading() const
    {
      return value_.z;
    }

    geometry_msgs::Point32 getPos() const
    {
      return value_;
    }

    void setPos(const geometry_msgs::Point32 &pos )
    {
      value_ = pos;
    }

    ros::Time getTimestamp() const
    {
      return timestamp_;
    }

    void setTimestamp(const ros::Time &time )
    {
      timestamp_ = time;
    }

    double getLinearEstimated() const
    {
      return linear_est_speed_;
    }

    double getAngularEstimated() const
    {
      return angular_est_speed_;
    }

    void setWheelParams(double wheel_separation, double wheel_radius)
    {
      wheel_separation_ = wheel_separation;
      wheel_radius_ = wheel_radius;
    }

  private:

    /**
     * @brief Function to update the odometry based on the velocities of the robot
     * @param linear : linear velocity m/s * DT (linear desplacement) computed by encoders
     * @param angular   : angular velocity rad/s * DT (angular desplacement) computed by encoders
     * @param time  : timestamp of the measured velocities
     *
     */
    void integrationByRungeKutta(const double& linear, const double& angular)
    {
      double direction = value_.z + angular/2.0;

      /// Normalization of angle between -Pi and Pi
      /// @attention the assumption here is that between two integration step the total angular cannot be bigger than Pi
      //direction = atan2(sin(direction),cos(direction));

      /// RUNGE-KUTTA 2nd ORDER INTEGRATION
      value_.x += linear * cos(direction);
      value_.y += linear * sin(direction);
      value_.z += angular;

      /// Normalization of angle between -Pi and Pi
      value_.z = atan2(sin(value_.z),cos(value_.z));
    }

    /**
     * @brief Other possible integration method provided by the class
     * @param linear
     * @param angular
     */
    void integrationExact(double linear, double angular)
    {
      if(fabs(angular) < 10e-3)
        integrationByRungeKutta(linear, angular);
      else
      {
        ///EXACT INTEGRATION (should be resolved problems when angularForDelta is zero)
        double thetaOld = value_.z;
        value_.z += angular;
        value_.x +=   (linear/angular )*( sin(value_.z) - sin(thetaOld) );
        value_.y +=  -(linear/angular )*( cos(value_.z) - cos(thetaOld) );
      }
    }

    /**
     * @brief Estimate speed based on averaging last 10 measurements
     * @param linear
     * @param angular
     */
    void speedEstimation(double linear, double angular)
    {
      if(lastSpeeds.size()> 10)
        lastSpeeds.pop_front();

      geometry_msgs::Point speed;
      speed.x = linear;
      speed.y = angular;
      lastSpeeds.push_back(speed);

      double averageLinearSpeed = 0.0, averageAngularSpeed = 0.0;
      for(speeds_it it=lastSpeeds.begin();it!= lastSpeeds.end(); ++it)
      {
        averageLinearSpeed += it->x;
        averageAngularSpeed += it->y;
      }

      linear_est_speed_ = averageLinearSpeed/lastSpeeds.size();
      angular_est_speed_ = averageAngularSpeed/lastSpeeds.size();
    }

    ros::Time timestamp_;
    ///(X,Y,Z) : Z is the orientation
    geometry_msgs::Point32 value_;

    double wheel_separation_;
    double wheel_radius_;
    double left_wheel_old_pos_, right_wheel_old_pos_;
    double linear_est_speed_, angular_est_speed_;
    std::list<geometry_msgs::Point> lastSpeeds;
    typedef std::list<geometry_msgs::Point>::const_iterator speeds_it;
  };
}
#endif /* ODOMETRY_H_ */
