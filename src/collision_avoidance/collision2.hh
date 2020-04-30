#ifndef _GAZEBO_COLLISION_HH_
#define _GAZEBO_COLLISION_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class Collision : public SensorPlugin
  {
    int numberOfCollision=0;
    std::list<std::string> DroCon; //droni con i quali e' avvenuto un contatto

    /// \brief Constructor.
    public: Collision();

    /// \brief Destructor.
    public: virtual ~Collision();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;
  };
}
#endif