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

UWDynamicsPlugin::UWDynamicsPlugin() : rho(999.1026){}

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
	math::Vector3 y_axis = math::Vector3(0, 1, 0);
	math::Vector3 z_axis = math::Vector3(0, 0, 1);

	for (auto link : this->model->GetLinks())
	{
		int id = link->GetId();
		if (this->propsMap.find(id) == this->propsMap.end())
		{
			double volumeSum = 0;
			ignition::math::Vector3d weightedPosSum = ignition::math::Vector3d::Zero;
			math::Vector3 weightedPosSumCOP = math::Vector3::Zero;
			this->propsMap[id].size = link->GetCollisionBoundingBox().GetSize();

			if(i == this->model->GetJointCount())
				for (auto joint : link->GetParentJoints())
					getProperties(joint, this->propsMap[id], y_axis, z_axis);

			else
				for (auto joint : link->GetChildJoints())
					getProperties(joint, this->propsMap[id], y_axis, z_axis);

			for (auto collision : link->GetCollisions())
			{
				double volume = collision->GetShape()->ComputeVolume();
				volumeSum += volume;
				weightedPosSumCOP += volume*collision->GetWorldPose().pos;
				weightedPosSum += volume*collision->GetWorldPose().pos.Ign();
			}

			this->propsMap[id].cov = weightedPosSum/volumeSum - link->GetWorldPose().pos.Ign();
			this->propsMap[id].cop = math::Vector3(0, 0, 0);
			this->propsMap[id].volume = volumeSum;
			this->propsMap[id].cDrift = 1.47;
			this->propsMap[id].cMass = 0.0;
			this->propsMap[id].cLift = 0.0;
			
			ROS_INFO_NAMED("link", "***********( %d )************",i+1);
			ROS_INFO_NAMED("ID", "linkID: %d", id);
			ROS_INFO_NAMED("length", "length: %0.7lf",this->propsMap[id].size[0]);
			ROS_INFO_NAMED("breadth", "breadth: %0.7lf",this->propsMap[id].size[1]);
			ROS_INFO_NAMED("height", "height: %0.7lf",this->propsMap[id].size[2]);
			ROS_INFO_NAMED("area", "area: %0.7lf",this->propsMap[id].area);
			ROS_INFO_NAMED("volume", "volume: %0.7lf",this->propsMap[id].volume);
			ROS_INFO_NAMED("upward", "upward: %0.7lf, %0.7lf, %0.7lf",this->propsMap[id].upward.x,this->propsMap[id].upward.y,this->propsMap[id].upward.z);
			ROS_INFO_NAMED("forward", "forward: %0.7lf, %0.7lf, %0.7lf",this->propsMap[id].forward.x,this->propsMap[id].forward.y,this->propsMap[id].forward.z);
			ROS_INFO_NAMED("cop", "cop: %0.7lf, %0.7lf, %0.7lf",this->propsMap[id].cop.x,this->propsMap[id].cop.y,this->propsMap[id].cop.z);
			ROS_INFO_NAMED("cov", "cov: %0.7lf, %0.7lf, %0.7lf",this->propsMap[id].cov.X(),this->propsMap[id].cov.Y(),this->propsMap[id].cov.Z());
			ROS_INFO_NAMED("params", "cDrift: %0.7lf; cMass: %0.7lf; cLift: %0.7lf;",this->propsMap[id].cDrift,this->propsMap[id].cMass,this->propsMap[id].cLift);
			ROS_INFO_NAMED("end", "******************************\n");

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
	    properties properties = this->propsMap[link->GetId()];
	    double volume = properties.volume;
	    GZ_ASSERT(volume > 0, "Nonpositive volume found in volume properties!");
	    ignition::math::Vector3d buoyancy = -this->rho * volume * this->model->GetWorld()->Gravity();
	    ignition::math::Pose3d linkFrame = link->GetWorldPose().Ign();
	    ignition::math::Vector3d buoyancyLinkFrame = linkFrame.Rot().Inverse().RotateVector(buoyancy);
	  	link->AddLinkForce(buoyancyLinkFrame, properties.cov);
	}

	//###########################################################//

	for (auto link : this->model->GetLinks())
	{
		properties properties = this->propsMap[link->GetId()];
		math::Vector3 vel = link->GetWorldLinearVel(properties.cop);
		if (vel.GetLength() <= 0.01)
			return;

		math::Pose pose = link->GetWorldPose();
		// rotate forward and upward vectors into inertial frame
		math::Vector3 forwardI = pose.rot.RotateVector(properties.forward);
		math::Vector3 upwardI = pose.rot.RotateVector(properties.upward);

		// ldNormal vector to lift-drag-plane described in inertial frame
		math::Vector3 ldNormal = upwardI.Normalize();

		// angle of attack is the angle between vel projected into lift-drag plane and forward vector
		// projected = ldNormal Xcross ( vector Xcross ldNormal)
		// so, velocity in lift-drag plane (expressed in inertial frame) is:
		double magVelNorm = vel.Dot(ldNormal);
		math::Vector3 velInLDPlane = ldNormal * magVelNorm;
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
		double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;
		// drag at cp
		
		//ROS_INFO_NAMED("angle", "cDrift: %0.7lf", cd);
		math::Vector3 lift = properties.cLift * q * properties.area * liftDirection;
		math::Vector3 drag = properties.cDrift * q * properties.area * dragDirection;
		// compute cm at cp, check for stall, correct for sweep

		// compute moment (torque) at cp
		math::Vector3 moment = properties.cMass * q * properties.area * momentDirection;

		// moment arm from cg to cp in inertial plane
		math::Vector3 momentArm = pose.rot.RotateVector(
		properties.cop - link->GetInertial()->GetCoG());
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

		link->AddForceAtRelativePosition(force, properties.cop);
		link->AddTorque(torque);
	}
}

//##################################################################//

void UWDynamicsPlugin::getProperties(physics::JointPtr joint, properties& ptr, math::Vector3 y_axis, math::Vector3 z_axis)
{
	math::Vector3 local_axis = math::Vector3::Zero;
	local_axis = joint->GetLocalAxis(0);

	if(local_axis == z_axis)
		ptr.upward = y_axis;
	else
		ptr.upward = z_axis;

	ptr.forward = local_axis.Cross(ptr.upward).GetAbs();
	ptr.area = ptr.size.Dot(local_axis) * ptr.size.Dot(local_axis.Cross(ptr.upward).GetAbs());
}
