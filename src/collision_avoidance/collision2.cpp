#include "collision2.hh"

//for logging
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>

std::ofstream myFile;

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(Collision)

/////////////////////////////////////////////////
Collision::Collision() : SensorPlugin()
{
}

/////////////////////////////////////////////////
Collision::~Collision()
{
}

/////////////////////////////////////////////////
void Collision::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "Collision requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&Collision::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void Collision::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    bool newContact = true;
    for(std::string s : DroCon){
        if(s == contacts.contact(i).collision2()){
            newContact = false;
        }
    }
    if(newContact){
        DroCon.push_back(contacts.contact(i).collision2());
        std::cout << this->parentSensor->Name() << " Collisioni totali "  << DroCon.size()<<std::endl;
    }
    //std::cout << "Collision between[" << contacts.contact(i).collision1()
    //          << "] and [" << contacts.contact(i).collision2() << "]\n";

  }
}