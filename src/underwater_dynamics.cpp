#include <stdio.h>
#include <math.h>
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

			/*
			this->propsMap[lid[i]].Mct = 0.0;
			this->propsMap[lid[i]].LDct = 0.1;
			this->propsMap[lid[i]].NLDct = 0.1;
			this->propsMap[lid[i]].Mcn = 0.1;
			this->propsMap[lid[i]].LDcn = 0.35;
			this->propsMap[lid[i]].NLDcn = 0.1;

			*/
			this->propsMap[lid[i]].Mct = 0.1; //this->rho * pow(this->propsMap[lid[i]].breadth, 2) * this->propsMap[lid[i]].length;
			this->propsMap[lid[i]].LDct = 0.2 * 3.1415926535 / (log(2.0 * this->propsMap[lid[i]].length/this->propsMap[lid[i]].breadth) - 0.807);
			this->propsMap[lid[i]].NLDct = pow(0.954919498, this->propsMap[lid[i]].length/this->propsMap[lid[i]].breadth);

			this->propsMap[lid[i]].Mcn = 0.1;//this->rho * pow(this->propsMap[lid[i]].breadth, 2);
			this->propsMap[lid[i]].LDcn = 0.4 * 3.1415926535 / (log(2.0 * this->propsMap[lid[i]].length/this->propsMap[lid[i]].breadth) + 0.193); 
			this->propsMap[lid[i]].NLDcn = pow(0.5130060177, this->propsMap[lid[i]].breadth/this->propsMap[lid[i]].length);
			/*
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
			ROS_INFO_NAMED("end", "******************************\n");
			*/
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
	double mU = 0.0000010533;
	double staticMu = 0.0010518;
	//double reynolds = 0.0;
	for (auto link : this->model->GetLinks())
	{
		properties properties = this->propsMap[link->GetId()];

		math::Pose pose = link->GetWorldPose();
		math::Vector3 cogI = -1.0 * pose.rot.RotateVector(properties.cog).Normalize();
		math::Vector3 localI = pose.rot.RotateVector(properties.localAxis).Normalize();
		math::Vector3 tangentialI = pose.rot.RotateVector(properties.tangential).Normalize();
		math::Vector3 normalI = pose.rot.RotateVector(properties.normal).Normalize();

		math::Vector3 normalVelocity = vectorize(link->GetWorldLinearVel()).Dot(normalI) * normalI;
		math::Vector3 tangentialVelocity = vectorize(link->GetWorldLinearVel()).Dot(tangentialI) * tangentialI;

		double Re = properties.length * link->GetWorldLinearVel().GetLength() / mU;
		double cdx = 1.0;
		if (Re>0) cdx = 37/Re;
		double cdy = 5.46 / fabs(log(7.4/Re));

		math::Vector3 normalAcceleration = vectorize(link->GetWorldLinearAccel()).Dot(normalI) * normalI;
		math::Vector3 tangentialAcceleration = vectorize(link->GetWorldLinearAccel()).Dot(tangentialI) * tangentialI;

		double addedMassT = 0.25 * properties.Mct * this->rho * 3.1415926535 * pow(properties.breadth, 2) * properties.length * tangentialAcceleration.GetLength();
		double nonLinearDragT = 0.5 * properties.NLDct * this->rho * properties.breadth * pow(tangentialVelocity.GetLength(), 2);
		double linearDragT = cdx * tangentialVelocity.GetLength();

		math::Vector3 tangentialForce = -1.0 * (addedMassT  + linearDragT  + nonLinearDragT) * tangentialVelocity.Normalize();
		//reynolds = (this->rho * normalVelocity.GetLength() * properties.breadth) / mU;

		double addedMassN = 0.25 * properties.Mcn * this->rho * 3.1415926535 * pow(properties.breadth, 2) * properties.length * normalAcceleration.GetLength();
		double nonLinearDragN = 0.5 * properties.NLDcn * this->rho * properties.breadth * pow(normalVelocity.GetLength(), 2);
		double linearDragN = cdy * normalVelocity.GetLength();

		math::Vector3 normalForce = -1.0 * (addedMassN  + linearDragN + nonLinearDragN) * normalVelocity.Normalize();

		link->AddForce(normalForce);
		link->AddForce(tangentialForce);

		if(j==0)
		{
			//ROS_INFO_NAMED("log", "coeffT: %0.7lf",(log(2.0 * properties.length/properties.breadth) - 0.807));
			//ROS_INFO_NAMED("log", "coeffN: %0.7lf",(log(2.0 * properties.length/properties.breadth) + 0.193));
			//ROS_INFO_NAMED("Velocity", "Mcn: %0.7lf",properties.Mct);
			//ROS_INFO_NAMED("eacclerationlocity", "Mtn: %0.7lf",properties.Mcn);			
			//ROS_INFO_NAMED("rey", "Re: %0.7lf",Re);
			//ROS_INFO_NAMED("rey", "ReT: %0.7lf",ReT);
			//ROS_INFO_NAMED("cdx", "linearDragT: %0.7lf",linearDragT);
			//ROS_INFO_NAMED("cdy", "linearDragN: %0.7lf",linearDragN);
			//ROS_INFO_NAMED("link", "***********( %d )************",j+1);
			//ROS_INFO_NAMED("ID", "linkID: %d", link->GetId());
			//ROS_INFO_NAMED("force", "force: %0.7lf, %0.7lf, %0.7lf",force.x,force.y,force.z);
			//ROS_INFO_NAMED("cogI", "cogI: %0.7lf, %0.7lf, %0.7lf",cogI.x,cogI.y,cogI.z);
			//ROS_INFO_NAMED("normalI", "normalI: %0.7lf, %0.7lf, %0.7lf",normalI.x,normalI.y,normalI.z);
			//ROS_INFO_NAMED("Velocity", "Velocity: %0.7lf",link->GetWorldLinearVel().Dot(normalI));
			//ROS_INFO_NAMED("eacclerationlocity", "Acceleration: %0.7lf",tangentialVelocity.GetLength());			
			//ROS_INFO_NAMED("tangentialI", "tangentialI: %0.7lf, %0.7lf, %0.7lf",tangentialI.x,tangentialI.y,tangentialI.z);
			//ROS_INFO_NAMED("normalVelocity", "normalVelocity: %0.7lf, %0.7lf, %0.7lf",normalVelocity.x,normalVelocity.y,normalVelocity.z);
			//ROS_INFO_NAMED("tangentialVelocity", "tangentialVelocity: %0.7lf, %0.7lf, %0.7lf",tangentialVelocity.x,tangentialVelocity.y,tangentialVelocity.z);	
			//ROS_INFO_NAMED("forceN", "forceN: %0.7lf, %0.7lf, %0.7lf",normalForce.x,normalForce.y,normalForce.z);
			//ROS_INFO_NAMED("forceT", "forceT: %0.7lf, %0.7lf, %0.7lf",tangentialForce.x,tangentialForce.y,tangentialForce.z);
			//ROS_INFO_NAMED("end", "******************************\n");			
		}

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

	ptr.localAxis = local_axis;
	ptr.tangential = local_axis.Cross(ptr.normal).GetAbs();
	ptr.length = ptr.size.Dot(ptr.tangential);
	ptr.breadth = ptr.size.Dot(local_axis);
	ptr.area = ptr.size.Dot(local_axis) * ptr.size.Dot(ptr.tangential);
}

 math::Vector3 UWDynamicsPlugin::vectorize(math::Vector3 vector)
 {
 	if(vector.GetLength() < 0.001)
 		return math::Vector3(0, 0, 0);
 	else
 		return vector;
}