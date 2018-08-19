#include <algorithm>
#include <string>
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "uw_dyn/underwater_dynamics.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(UWDynamicsPlugin)

//*********************** Default values ******************************//

UWDynamicsPlugin::UWDynamicsPlugin() : rho(999.1026)
{}

//##################################################################//

//*********************** Load values ******************************//

void UWDynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	GZ_ASSERT(_model, "UWDynamicsPlugin _model pointer is NULL");
	GZ_ASSERT(_sdf, "UWDynamicsPlugin _sdf pointer is NULL");
	this->model = _model;
	this->modelName = _model->GetName();
	this->sdf = _sdf;
	this->world = this->model->GetWorld();
	GZ_ASSERT(this->world, "UWDynamicsPlugin world pointer is NULL");
	this->physics = this->world->GetPhysicsEngine();
	GZ_ASSERT(this->physics, "UWDynamicsPlugin physics pointer is NULL");
	GZ_ASSERT(_sdf, "UWDynamicsPlugin _sdf pointer is NULL");

	//*********************** Load values from SDF ******************************//

	if (_sdf->HasElement("fluid_density"))
		this->rho = _sdf->Get<double>("fluid_density");

	//##################################################################//

	//*********************** Compute values for all links ******************************//
	int i = 0;
	for (auto link : this->model->GetLinks())
	{
		int id = link->GetId();
		//ROS_INFO_NAMED("Hello", "this is link :%d", id);
		if (this->volPropsMap.find(id) == this->volPropsMap.end())
		{
			double volumeSum = 0;
			ignition::math::Vector3d weightedPosSum = ignition::math::Vector3d::Zero;
			math::Vector3 weightedPosSumCOP = math::Vector3::Zero;
			this->volPropsMap[id].size = link->GetCollisionBoundingBox().GetSize();

			if(i == this->model->GetJointCount())
			{
				for (auto joint : link->GetParentJoints())
				{
					int Jid = joint->GetChild()->GetId();
					//ROS_INFO_NAMED("joint retrieved","joint retrieved: %d", Jid);
					math::Vector3 local_axis = math::Vector3::Zero;
					math::Vector3 new_local_axis = math::Vector3::Zero;
					double a[3]={0.0, 0.0, 0.0}, b[3]={0.0, 0.0, 0.0};
					local_axis = joint->GetLocalAxis(0);
					for(int i=0;i<3;i++)
					{
						if(abs(local_axis[i])>0.0)
							if(i<2)
								a[i+1]=1.0;
							else
								a[i-1]=1.0;
					}
					new_local_axis = math::Vector3(a[0], a[1], a[2]);

					for(int i=0;i<3;i++)
					{
						if(abs(new_local_axis[i])>0.0)
							if(i<2)
								b[i+1]=1.0;
							else
								b[i-1]=1.0;
					}

					this->volPropsMap[id].upward = math::Vector3(a[0], a[1], a[2]);
					this->volPropsMap[id].forward = local_axis.Cross(this->volPropsMap[id].upward).GetAbs();
					this->volPropsMap[id].area = this->volPropsMap[id].size.Dot(new_local_axis) * this->volPropsMap[id].size.Dot(new_local_axis.Cross(this->volPropsMap[id].upward).GetAbs());
				}
			}
			else
			{
				for (auto joint : link->GetChildJoints())
				{
					int Jid = joint->GetParent()->GetId();
					//ROS_INFO_NAMED("joint retrieved","joint retrieved: %d", Jid);
					math::Vector3 local_axis = math::Vector3::Zero;
					double a[3]={0.0, 0.0, 0.0};
					local_axis = joint->GetLocalAxis(0);
					for(int i=0;i<3;i++)
					{
						if(abs(local_axis[i])>0.0)
							if(i<2)
								a[i+1]=1.0;
							else
								a[i-1]=1.0;
					}
					this->volPropsMap[id].upward = math::Vector3(a[0], a[1], a[2]);
					this->volPropsMap[id].forward = local_axis.Cross(this->volPropsMap[id].upward).GetAbs();
					this->volPropsMap[id].area = this->volPropsMap[id].size.Dot(local_axis) * this->volPropsMap[id].size.Dot(local_axis.Cross(this->volPropsMap[id].upward).GetAbs());
				}				
			}

			for (auto collision : link->GetCollisions())
			{
				double volume = collision->GetShape()->ComputeVolume();
				volumeSum += volume;
				weightedPosSumCOP += volume*collision->GetWorldPose().pos;
				weightedPosSum += volume*collision->GetWorldPose().pos.Ign();
			}

			this->volPropsMap[id].cov = weightedPosSum/volumeSum - link->GetWorldPose().pos.Ign();
			this->volPropsMap[id].cop = math::Vector3(0, 0, 0);
			this->volPropsMap[id].volume = volumeSum;
			this->volPropsMap[id].Cdrift = 1.47;
			this->volPropsMap[id].CMass = 0.0;
			this->volPropsMap[id].Clift = 0.0;
			
			ROS_INFO_NAMED("length", "length: %0.7lf",this->volPropsMap[id].size[0]);
			ROS_INFO_NAMED("breadth", "breadth: %0.7lf",this->volPropsMap[id].size[1]);
			ROS_INFO_NAMED("height", "height: %0.7lf",this->volPropsMap[id].size[2]);
			ROS_INFO_NAMED("area", "area: %0.7lf",this->volPropsMap[id].area);
			ROS_INFO_NAMED("Volume", "volume: %0.7lf",this->volPropsMap[id].volume);
			ROS_INFO_NAMED("up", "up %0.7lf, %0.7lf, %0.7lf",this->volPropsMap[id].upward.x,this->volPropsMap[id].upward.y,this->volPropsMap[id].upward.z);
			ROS_INFO_NAMED("forw", "forw %0.7lf, %0.7lf, %0.7lf",this->volPropsMap[id].forward.x,this->volPropsMap[id].forward.y,this->volPropsMap[id].forward.z);
			ROS_INFO_NAMED("Hello", "***********%d************",i);

		}
		i++;
	}
}

//##################################################################//

//*********************** Init ******************************//

void UWDynamicsPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&UWDynamicsPlugin::OnUpdate, this));
}

//###########################################################//

//*********************** Update forces ******************************//

void UWDynamicsPlugin::OnUpdate()
{
	//*********************** Buoyant force ******************************//

	for (auto link : this->model->GetLinks())
	{
	    VolumeProperties volumeProperties = this->volPropsMap[link->GetId()];
	    double volume = volumeProperties.volume;
	    GZ_ASSERT(volume > 0, "Nonpositive volume found in volume properties!");
	    ignition::math::Vector3d buoyancy = -this->rho * volume * this->model->GetWorld()->Gravity();
	    ignition::math::Pose3d linkFrame = link->GetWorldPose().Ign();
	    ignition::math::Vector3d buoyancyLinkFrame = linkFrame.Rot().Inverse().RotateVector(buoyancy);
	  	link->AddLinkForce(buoyancyLinkFrame, volumeProperties.cov);
	}

	//###########################################################//

	for (auto link : this->model->GetLinks())
	{
		VolumeProperties volumeProperties = this->volPropsMap[link->GetId()];
		math::Vector3 vel = link->GetWorldLinearVel(volumeProperties.cop);
		if (vel.GetLength() <= 0.01)
			return;

		math::Pose pose = link->GetWorldPose();
		// rotate forward and upward vectors into inertial frame
		math::Vector3 forwardI = pose.rot.RotateVector(volumeProperties.forward);
		//ROS_INFO_NAMED("forward", "Forward: x: %0.7lf, y: %0.7lf, z: %0.7lf", volumeProperties.forward[0], volumeProperties.forward[1], volumeProperties.forward[2]);
		//ROS_INFO_NAMED("forward", "ForwardINV: x: %0.7lf, y: %0.7lf, z: %0.7lf", forwardI[0], forwardI[1], forwardI[2]);
		//ROS_INFO_NAMED("Hello", "***********************");

		math::Vector3 upwardI = pose.rot.RotateVector(volumeProperties.upward);
		// ldNormal vector to lift-drag-plane described in inertial frame
		math::Vector3 ldNormal = forwardI.Cross(upwardI).Normalize();
		//ROS_INFO_NAMED("forward", "Normal: x: %0.7lf, y: %0.7lf, z: %0.7lf", ldNormal[0], ldNormal[1], ldNormal[2]);
		//ROS_INFO_NAMED("forward", "Velocity: x: %0.7lf, y: %0.7lf, z: %0.7lf", vel[0], vel[1], vel[2]);

		// angle of attack is the angle between vel projected into lift-drag plane and forward vector
		// projected = ldNormal Xcross ( vector Xcross ldNormal)
		// so, velocity in lift-drag plane (expressed in inertial frame) is:
		math::Vector3 velInLDPlane = ldNormal.Cross(vel.Cross(ldNormal));
		//ROS_INFO_NAMED("forward", "velInLDPlane: x: %0.7lf, y: %0.7lf, z: %0.7lf", velInLDPlane[0], velInLDPlane[1], velInLDPlane[2]);

		// get direction of drag
		math::Vector3 dragDirection = -velInLDPlane;
		dragDirection.Normalize();

		// get direction of lift
		math::Vector3 liftDirection = ldNormal.Cross(velInLDPlane);
		liftDirection.Normalize();
		//ROS_INFO_NAMED("forward", "lift: x: %0.7lf, y: %0.7lf, z: %0.7lf", dragDirection[0], dragDirection[1], dragDirection[2]);


		// get direction of moment
		math::Vector3 momentDirection = ldNormal;

		// compute dynamic pressure
		double speedInLDPlane = velInLDPlane.GetLength();
		double q = 0.333333333 * this->rho * speedInLDPlane * speedInLDPlane;
		// drag at cp
		
		//ROS_INFO_NAMED("angle", "Cdrift: %0.7lf", cd);
		math::Vector3 lift = volumeProperties.Clift * q * volumeProperties.area * liftDirection;
		math::Vector3 drag = volumeProperties.Cdrift * q * volumeProperties.area * dragDirection;
		// compute cm at cp, check for stall, correct for sweep

		// compute moment (torque) at cp
		math::Vector3 moment = volumeProperties.CMass * q * volumeProperties.area * momentDirection;

		// moment arm from cg to cp in inertial plane
		math::Vector3 momentArm = pose.rot.RotateVector(
		volumeProperties.cop - link->GetInertial()->GetCoG());
		// gzerr << this->cp << " : " << this->link->GetInertial()->GetCoG() << "\n";

		// force and torque about cg in inertial frame
		math::Vector3 force = lift + drag;
		// + moment.Cross(momentArm);

		math::Vector3 torque = moment;
		// - lift.Cross(momentArm) - drag.Cross(momentArm);

		if (0)
		{
			gzerr << "=============================\n";
			gzerr << "Link: [" << this->link->GetName()<< "] pose: [" << pose << "] dynamic pressure: [" << q << "]\n";
			gzerr << "spd: [" << vel.GetLength() << "] vel: [" << vel << "]\n";
			gzerr << "spd sweep: [" << velInLDPlane.GetLength()<< "] vel in LD: [" << velInLDPlane << "]\n";
			gzerr << "forward (inertial): " << forwardI << "\n";
			gzerr << "upward (inertial): " << upwardI << "\n";
			gzerr << "lift dir (inertial): " << liftDirection << "\n";
			gzerr << "LD Normal: " << ldNormal << "\n";
			gzerr << "moment: " << moment << "\n";
			gzerr << "cp momentArm: " << momentArm << "\n";
			gzerr << "force: " << force << "\n";
			gzerr << "torque: " << torque << "\n";
		}

		link->AddForceAtRelativePosition(force, volumeProperties.cop);
		link->AddTorque(torque);
	}
	//ROS_INFO_NAMED("Hello", "***********************");
}

//##################################################################//