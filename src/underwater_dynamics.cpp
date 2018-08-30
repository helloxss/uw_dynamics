#include <stdio.h>
#include <stdlib.h>
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
	lid = (int*) malloc(this->model->GetJointCount()+1 * sizeof(int));
	math::Vector3 y_axis = math::Vector3(0, 1, 0);
	math::Vector3 z_axis = math::Vector3(0, 0, 1);

	for (auto link : this->model->GetLinks())
	{
		lid[i] = link->GetId();
		if (this->propsMap.find(lid[i]) == this->propsMap.end())
		{
			double volumeSum = 0;
			ignition::math::Vector3d weightedPosSum = ignition::math::Vector3d::Zero;
			math::Vector3 weightedPosSumCOP = math::Vector3::Zero;
			this->propsMap[lid[i]].size = link->GetCollisionBoundingBox().GetSize();

			if(i == this->model->GetJointCount())
				for (auto joint : link->GetParentJoints())
					getProperties(joint, this->propsMap[lid[i]], y_axis, z_axis);

			else
				for (auto joint : link->GetChildJoints())
					getProperties(joint, this->propsMap[lid[i]], y_axis, z_axis);

			for (auto collision : link->GetCollisions())
			{
				double volume = collision->GetShape()->ComputeVolume();
				volumeSum += volume;
			}

			this->propsMap[lid[i]].cop = math::Vector3(0, 0, 0);
			this->propsMap[lid[i]].cob = link->GetInertial()->GetCoG();
			this->propsMap[lid[i]].cog = this->propsMap[lid[i]].tangential * this->propsMap[lid[i]].length/2;
			this->propsMap[lid[i]].volume = volumeSum;
			this->propsMap[lid[i]].velocity = math::Vector3(0, 0, 0);
			this->propsMap[lid[i]].acceleration = math::Vector3(0, 0, 0);
			this->propsMap[lid[i]].omega = math::Vector3(0, 0, 0);

			this->propsMap[lid[i]].cF = 0.01;
			this->propsMap[lid[i]].cD = 0.42;
			this->propsMap[lid[i]].cA = 0.001;
			this->propsMap[lid[i]].cM = 0.3;
			
			ROS_INFO_NAMED("link", "***********( %d )************",i+1);
			ROS_INFO_NAMED("ID", "linkID: %d", lid[i]);
			ROS_INFO_NAMED("length", "length: %0.7lf",this->propsMap[lid[i]].size[0]);
			ROS_INFO_NAMED("breadth", "breadth: %0.7lf",this->propsMap[lid[i]].size[1]);
			ROS_INFO_NAMED("height", "height: %0.7lf",this->propsMap[lid[i]].size[2]);
			ROS_INFO_NAMED("area", "area: %0.7lf",this->propsMap[lid[i]].area);
			ROS_INFO_NAMED("volume", "volume: %0.7lf",this->propsMap[lid[i]].volume);
			ROS_INFO_NAMED("normal", "normal: %0.7lf, %0.7lf, %0.7lf",this->propsMap[lid[i]].normal.x,this->propsMap[lid[i]].normal.y,this->propsMap[lid[i]].normal.z);
			ROS_INFO_NAMED("tangential", "tangential: %0.7lf, %0.7lf, %0.7lf",this->propsMap[lid[i]].tangential.x,this->propsMap[lid[i]].tangential.y,this->propsMap[lid[i]].tangential.z);
			ROS_INFO_NAMED("cop", "cop: %0.7lf, %0.7lf, %0.7lf",this->propsMap[lid[i]].cop.x,this->propsMap[lid[i]].cop.y,this->propsMap[lid[i]].cop.z);
			ROS_INFO_NAMED("cog", "cog: %0.7lf, %0.7lf, %0.7lf",this->propsMap[lid[i]].cog.x,this->propsMap[lid[i]].cog.y,this->propsMap[lid[i]].cog.z);			
			ROS_INFO_NAMED("params", "cF: %0.7lf; cD: %0.7lf; cA: %0.7lf;",this->propsMap[lid[i]].cF,this->propsMap[lid[i]].cD,this->propsMap[lid[i]].cA);
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
	  	link->AddLinkForce(buoyancyLinkFrame, properties.cob);
	}

	//###########################################################//

	int j = 0;
	for (auto link : this->model->GetLinks())
	{
		this->propsMap[link->GetId()].omega = link->GetWorldAngularVel();
		this->propsMap[link->GetId()].alpha = link->GetWorldAngularAccel();

		properties properties = this->propsMap[link->GetId()];
		math::Vector3 vel = link->GetWorldLinearVel(properties.cog);
		math::Vector3 acc = link->GetWorldLinearAccel();

		if (vel.GetLength() <= 0.01)
			return;

		math::Pose pose = link->GetWorldPose();
		math::Vector3 cogI = -pose.rot.RotateVector(properties.cog);

		this->propsMap[link->GetId()].velocity = properties.omega.Cross(cogI);
		this->propsMap[link->GetId()].acceleration = properties.alpha.Cross(cogI);

		math::Vector3 tangentialI = pose.rot.RotateVector(properties.tangential).Normalize();
		math::Vector3 normalI = pose.rot.RotateVector(properties.normal).Normalize();

		double magVelT = properties.velocity.Dot(tangentialI);
		double magVelN = properties.velocity.Dot(normalI);
		double magaccT = properties.acceleration.Dot(tangentialI);
		double magaccN = properties.acceleration.Dot(normalI);

		double cT = 0.25 * this->rho * 3.1415926535 * properties.cF * properties.length * properties.breadth;
		double cN = 0.5 * this->rho * properties.cD * properties.length * properties.breadth;
		double uT = 0.0;
		double uN = 0.25 * this->rho * 3.1415926535 * properties.cA * properties.length * properties.breadth * properties.breadth;

		double forceT = 1.0 * ((uN * magaccT) + (cN * magVelT) + (cN * sgn(magVelT) * magVelT * magVelT));
		double forceN = 1.0 * ((cT * magVelN) + (cT * sgn(magVelN) * magVelN * magVelN));

		math::Vector3 tangentialForce = -tangentialI * forceT;
		math::Vector3 normalForce = -normalI * forceN;
		math::Vector3 force = tangentialForce + normalForce;


		double lambda1 = 0.08333333333 * this->rho * 3.1415926535 * properties.cM * pow(properties.length/2, 2) * 0.0;
		double lambda2 = 0.16666666666 * this->rho * 3.1415926535 * properties.cF * properties.breadth * pow(properties.length/2, 3);
		double lambda3 = 0.125 * this->rho * 3.1415926535 * properties.cF * properties.breadth * pow(properties.length/2, 4);

		double torqueT = 1.0 * (lambda1 * magaccT) + (lambda2 * magVelT) + (lambda3 * magVelT * properties.velocity.GetLength());
		double torqueN = 1.0 * (lambda1 * magaccN) + (lambda2 * magVelN) + (lambda3 * magVelN * properties.velocity.GetLength());
		math::Vector3 torque = -1.0 * (torqueT + torqueN) * properties.localAxis;

		//link->AddForceAtRelativePosition(force, properties.cob);
		link->AddLinkForce(force);
		link->AddRelativeTorque(torque);

		if(j==0)
		{
			//ROS_INFO_NAMED("link", "***********( %d )************",j+1);
			//ROS_INFO_NAMED("ID", "linkID: %d", link->GetId());
			//ROS_INFO_NAMED("tangentialI", "force: %0.7lf, %0.7lf, %0.7lf",force.x,force.y,force.z);
			//ROS_INFO_NAMED("normalI", "normalI: %0.7lf, %0.7lf, %0.7lf",normalI.x,normalI.y,normalI.z);
			//ROS_INFO_NAMED("params", "cT: %0.7lf; cN: %0.7lf; uN: %0.7lf;",cT,cN,uN);
			//ROS_INFO_NAMED("end", "******************************\n");			
		}
		j++;
	}

}

//##################################################################//

void UWDynamicsPlugin::getProperties(physics::JointPtr joint, properties& ptr, math::Vector3 y_axis, math::Vector3 z_axis)
{
	ptr.localAxis = joint->GetLocalAxis(0);

	if(ptr.localAxis == z_axis)
		ptr.normal = y_axis;
	else
		ptr.normal = z_axis;

	ptr.tangential = ptr.localAxis.Cross(ptr.normal).GetAbs();
	ptr.length = ptr.size.Dot(ptr.tangential);
	ptr.breadth = ptr.size.Dot(ptr.localAxis);
	ptr.area = ptr.size.Dot(ptr.localAxis) * ptr.size.Dot(ptr.tangential);
}

double UWDynamicsPlugin::sgn(double k)
{
	if(k < 0) return -1.0;
	else if (k > 0) return 1.0;
	else return 0.0;
}