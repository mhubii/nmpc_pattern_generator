#include "AICubPlugin.h"
#include <iostream>

namespace gazebo
{

// Register the plugin.
GZ_REGISTER_MODEL_PLUGIN(AICubPlugin)

AICubPlugin::AICubPlugin() {

}

void AICubPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {

      // Store the pointer to the model.
      this->model_ = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&AICubPlugin::OnUpdate, this));
}

void AICubPlugin::OnUpdate() {

	std::cout << "updating.." << std::endl;
}

}
