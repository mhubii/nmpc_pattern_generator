#ifndef GAZEBO_HEICUB_PLUGIN_H_
#define GAZEBO_HEICUB_PLUGIN_H_

#include <gazebo/gazebo.hh>

namespace gazebo 
{

class AICubPlugin : public ModelPlugin
{

public:

	AICubPlugin();

	virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

	void OnUpdate();

private:

	physics::ModelPtr model_;
	math::Pose original_pose_;
	event::ConnectionPtr update_connection_;

};
}

#endif
