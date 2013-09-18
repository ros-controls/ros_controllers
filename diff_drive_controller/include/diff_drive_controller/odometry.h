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

#include <cmath>
#include <vector>
#include <boost/concept_check.hpp>

#include <ros/time.h>
#include <geometry_msgs/Point32.h>

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
      * Function to update the odometry based on the speed and jog of the robot
      * @param speed : linear velocity m/s * DT (linear desplacement) computed by encoders
      * @param jog   : angular velocity rad/s * DT (angular desplacement) computed by encoders
      * @param time  : timestamp of the measured speed and jog
      */
    inline bool integrate(const double& speed, const double& jog, const ros::Time &time)
    {
      double deltaTime = (time - _timestamp).toSec();
      if( deltaTime > 0.0001)
      {
        _timestamp = time;
        integrationByRungeKutta(speed, jog);
        _linear = speed / deltaTime;
        _angular = jog / deltaTime;
        return true;
      }
      else
        return false;
    }

    //    inline void setHeading(double angle)
    //    {
    //      _value.set( _value.x, _value.y, angle );
    //    }

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

    /** Retrieves the speed value **/
    inline double getLinear() const
    {
      return _linear;
    }

    /** Retrieves the jog value **/
    inline double getAngular() const
    {
      return _angular;
    }

    inline void setSpeeds (double speed, double jog)
    {
      _linear = speed;
      _angular = jog;
    }

  private:

    /**
     * One possible integration method provided by the class
     * @param speed
     * @param jog
     */
    inline void integrationByRungeKutta(const double& speed, const double& jog)
    {
      double direction = _value.z + jog/2.0;

      /// Normalization of angle between -Pi and Pi
      /// @attention the assumption here is that between two integration step the total jog cannot be bigger than Pi
      //direction = atan2(sin(direction),cos(direction));

      /// RUNGE-KUTTA 2nd ORDER INTEGRATION
      _value.x += speed * cos(direction);
      _value.y += speed * sin(direction);
      _value.z += jog;

      /// Normalization of angle between -Pi and Pi
      _value.z = atan2(sin(_value.z),cos(_value.z));
    }

    /**
     * Other possible integration method provided by the class
     * @param speed
     * @param jog
     */
    inline void integrationExact(const double& speed, const double& jog)
    {
      if(fabs(jog) < 10e-3)
        integrationByRungeKutta(speed, jog);
      else
      {
        ///EXACT INTEGRATION (should be resolved problems when jogForDelta is zero)
        double thetaOld = _value.z;
        _value.z += jog;
        _value.x +=   (speed/jog )*( sin(_value.z) - sin(thetaOld) );
        _value.y +=  -(speed/jog )*( cos(_value.z) - cos(thetaOld) );
      }
    }

    ros::Time _timestamp;

    ///(X,Y,Z) : Z is the orientation
    geometry_msgs::Point32   _value;

    ///last speed used to update odometry
    double _linear;

    ///last jog used to update odometry
    double _angular;
  };
}
#endif /* ODOMETRY_H_ */
