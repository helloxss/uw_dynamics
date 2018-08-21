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
			this->propsMap[id].cF = 0.01;
			this->propsMap[id].cD = 0.001;
			this->propsMap[id].cA = 0.001;
			
			ROS_INFO_NAMED("link", "***********( %d )************",i+1);
			ROS_INFO_NAMED("ID", "linkID: %d", id);
			ROS_INFO_NAMED("length", "length: %0.7lf",this->propsMap[id].size[0]);
			ROS_INFO_NAMED("breadth", "breadth: %0.7lf",this->propsMap[id].size[1]);
			ROS_INFO_NAMED("height", "height: %0.7lf",this->propsMap[id].size[2]);
			ROS_INFO_NAMED("area", "area: %0.7lf",this->propsMap[id].area);
			ROS_INFO_NAMED("volume", "volume: %0.7lf",this->propsMap[id].volume);
			ROS_INFO_NAMED("normal", "normal: %0.7lf, %0.7lf, %0.7lf",this->propsMap[id].normal.x,this->propsMap[id].normal.y,this->propsMap[id].normal.z);
			ROS_INFO_NAMED("tangential", "tangential: %0.7lf, %0.7lf, %0.7lf",this->propsMap[id].tangential.x,this->propsMap[id].tangential.y,this->propsMap[id].tangential.z);
			ROS_INFO_NAMED("cop", "cop: %0.7lf, %0.7lf, %0.7lf",this->propsMap[id].cop.x,this->propsMap[id].cop.y,this->propsMap[id].cop.z);
			ROS_INFO_NAMED("cov", "cov: %0.7lf, %0.7lf, %0.7lf",this->propsMap[id].cov.X(),this->propsMap[id].cov.Y(),this->propsMap[id].cov.Z());
			ROS_INFO_NAMED("params", "cF: %0.7lf; cD: %0.7lf; cA: %0.7lf;",this->propsMap[id].cF,this->propsMap[id].cD,this->propsMap[id].cA);
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

	int j = 0;
	for (auto link : this->model->GetLinks())
	{
		properties properties = this->propsMap[link->GetId()];
		math::Vector3 vel = link->GetWorldLinearVel();
		math::Vector3 acc = link->GetRelativeLinearAccel();

		if (vel.GetLength() <= 0.01)
			return;

		math::Pose pose = link->GetWorldPose();

		// rotate tangential and normal vectors into inertial frame
		math::Vector3 tangentialI = pose.rot.RotateVector(properties.tangential).Normalize();
		math::Vector3 normalI = pose.rot.RotateVector(properties.normal).Normalize();

		math::Vector3 velT = tangentialI * vel.Dot(tangentialI);
		math::Vector3 velN = normalI * vel.Dot(normalI);
		math::Vector3 accT = tangentialI * acc.Dot(tangentialI);
		math::Vector3 accN = normalI * acc.Dot(normalI);

		double magVelT = velT.GetLength();
		double magVelN = velN.GetLength();
		double magaccT = accT.GetLength();
		double magaccN = accN.GetLength();

		double cT = 0.25 * this->rho * 3.1415926535 * properties.cF * properties.length * properties.breadth;
		double cN = 0.5 * this->rho * properties.cD * properties.length * properties.breadth;
		double uT = 0.0;
		double uN = 0.25 * this->rho * 3.1415926535 * properties.cA * properties.length * properties.breadth * properties.breadth;

		double forceT = -1.0 * cT * sgn(magVelT) * magVelT * magVelT;
		double forceN = -1.0 * ((uN * magaccN) + (cN * sgn(magVelN) * magVelN * magVelN));

		math::Vector3 tangentialForce = tangentialI * forceT;
		math::Vector3 normalForce = normalI * forceN;
		math::Vector3 force = tangentialForce + normalForce;

		if (0)
		{
			gzerr << "=============================\n";
			gzerr << "tangential (inertial): " << tangentialI << "\n";
			gzerr << "normal (inertial): " << normalI << "\n";
			gzerr << "LD Normal: " << normalI << "\n";
			gzerr << "force: " << force << "\n";
		}

		link->AddForceAtRelativePosition(force, properties.cop);
		j++;
	}
}

//##################################################################//

void UWDynamicsPlugin::getProperties(physics::JointPtr joint, properties& ptr, math::Vector3 y_axis, math::Vector3 z_axis)
{
	math::Vector3 local_axis = math::Vector3::Zero;
	local_axis = joint->GetLocalAxis(0);

	if(local_axis == z_axis)
		ptr.normal = y_axis;
	else
		ptr.normal = z_axis;

	ptr.tangential = local_axis.Cross(ptr.normal).GetAbs();
	ptr.length = ptr.size.Dot(ptr.tangential);
	ptr.breadth = ptr.size.Dot(local_axis);
	ptr.area = ptr.size.Dot(local_axis) * ptr.size.Dot(ptr.tangential);
}

double UWDynamicsPlugin::sgn(double k)
{
	if(k < 0) return -1.0;
	else if (k > 0) return 1.0;
	else return 0.0;
}