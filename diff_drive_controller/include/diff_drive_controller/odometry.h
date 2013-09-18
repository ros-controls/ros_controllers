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

/**
 * Class to handle odometry reading (2D oriented position with related timestamp)
 */
namespace diff_drive_controller{
  class Odometry
  {
  public:
    /**
      * Default contructor
      * Timestamp will get the current time value
      * Value will be set to zero
      */
    Odometry():
      _timestamp(0),
      _linear(0.0),
      _angular(0.0)
    {
      _value.x = _value .y = _value.z = 0.0;
    }

    Odometry(Odometry const &odo) :
      _timestamp(odo._timestamp),
      _value(odo._value),
      _linear(odo._linear),
      _angular(odo._angular)
    { }

    Odometry(geometry_msgs::Point32 const &p) :
      _timestamp(ros::Time::now()),
      _value(p),
      _linear(0.0),
      _angular(0.0)
    { }

    Odometry& operator=(Odometry const &rho)
    {
      _timestamp = rho._timestamp;
      _value     = rho._value;
      _linear     = rho._linear;
      _angular     = rho._angular;
      return *this;
    }

    bool operator==(Odometry const &rht)
    {
      return (_value.x == rht._value.x) &&
          (_value.y == rht._value.y) &&
          (_value.z == rht._value.z) &&
          fabs((_timestamp - rht._timestamp).toSec()) < 0.0001;
    }

    /**
      * Function to update the odometry based on the velocities of the robot
      * @param linear : linear velocity m/s * DT (linear desplacement) computed by encoders
      * @param angular   : angular velocity rad/s * DT (angular desplacement) computed by encoders
      * @param time  : timestamp of the measured velocities
      */
    inline bool integrate(const double& linear, const double& angular, const ros::Time &time)
    {
      double deltaTime = (time - _timestamp).toSec();
      if(deltaTime > 0.0001)
      {
        _timestamp = time;
        integrationByRungeKutta(linear, angular);
        _linear = linear / deltaTime;
        _angular = angular / deltaTime;
        return true;
      }
      else
        return false;
    }

    inline double getHeading() const
    {
      return _value.z;
    }

    /** Retrieves the odometry value **/
    inline geometry_msgs::Point32 getPos() const
    {
      return _value;
    }

    /** Sets the odometry value **/
    inline void setPos(const geometry_msgs::Point32 &pos )
    {
      _value = pos;
    }

    /** Retrieves the timestamp value **/
    inline ros::Time getTimestamp() const
    {
      return _timestamp;
    }

    /** Sets the odometry value **/
    inline void setTimestamp(const ros::Time &time )
    {
      _timestamp = time;
    }

    /** Retrieves the linear velocity value **/
    inline double getLinear() const
    {
      return _linear;
    }

    /** Retrieves the angular velocity value **/
    inline double getAngular() const
    {
      return _angular;
    }

    inline void setSpeeds (double linear, double angular)
    {
      _linear = linear;
      _angular = angular;
    }

  private:

    /**
     * One possible integration method provided by the class
     * @param linear
     * @param angular
     */
    inline void integrationByRungeKutta(const double& linear, const double& angular)
    {
      double direction = _value.z + angular/2.0;

      /// Normalization of angle between -Pi and Pi
      /// @attention the assumption here is that between two integration step the total angular cannot be bigger than Pi
      //direction = atan2(sin(direction),cos(direction));

      /// RUNGE-KUTTA 2nd ORDER INTEGRATION
      _value.x += linear * cos(direction);
      _value.y += linear * sin(direction);
      _value.z += angular;

      /// Normalization of angle between -Pi and Pi
      _value.z = atan2(sin(_value.z),cos(_value.z));
    }

    /**
     * Other possible integration method provided by the class
     * @param linear
     * @param angular
     */
    inline void integrationExact(const double& linear, const double& angular)
    {
      if(fabs(angular) < 10e-3)
        integrationByRungeKutta(linear, angular);
      else
      {
        ///EXACT INTEGRATION (should be resolved problems when angularForDelta is zero)
        double thetaOld = _value.z;
        _value.z += angular;
        _value.x +=   (linear/angular )*( sin(_value.z) - sin(thetaOld) );
        _value.y +=  -(linear/angular )*( cos(_value.z) - cos(thetaOld) );
      }
    }

    ros::Time _timestamp;

    ///(X,Y,Z) : Z is the orientation
    geometry_msgs::Point32 _value;

    ///last linear velocity used to update odometry
    double _linear;

    ///last angular velocity used to update odometry
    double _angular;
  };
}
#endif /* ODOMETRY_H_ */
