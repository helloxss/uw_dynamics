/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <algorithm>
#include <string>
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "uw_dyn/underwater_dynamics.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LiftDragPlugin)

//*********************** Default values ******************************//

LiftDragPlugin::LiftDragPlugin() : cla(1.0), cda(0.01), cma(0.01), rho(1.2041), fluidDensity(999.1026)
{
	this->alpha0 = 0.0;
	this->alphaStall = 0.5*M_PI;
	this->claStall = 0.0;
	this->cdaStall = 1.0;
	this->cmaStall = 0.0;
}

//##################################################################//

//*********************** Load values ******************************//

void LiftDragPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	GZ_ASSERT(_model, "LiftDragPlugin _model pointer is NULL");
	GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");
	this->model = _model;
	this->modelName = _model->GetName();
	this->sdf = _sdf;
	this->world = this->model->GetWorld();
	GZ_ASSERT(this->world, "LiftDragPlugin world pointer is NULL");
	this->physics = this->world->GetPhysicsEngine();
	GZ_ASSERT(this->physics, "LiftDragPlugin physics pointer is NULL");
	GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");

	//*********************** Load values from SDF ******************************//
	if (_sdf->HasElement("a0"))
		this->alpha0 = _sdf->Get<double>("a0");
	if (_sdf->HasElement("cla"))
		this->cla = _sdf->Get<double>("cla");
	if (_sdf->HasElement("cda"))
		this->cda = _sdf->Get<double>("cda");
	if (_sdf->HasElement("cma"))
		this->cma = _sdf->Get<double>("cma");
	if (_sdf->HasElement("alpha_stall"))
		this->alphaStall = _sdf->Get<double>("alpha_stall");
	if (_sdf->HasElement("cla_stall"))
		this->claStall = _sdf->Get<double>("cla_stall");
	if (_sdf->HasElement("cda_stall"))
		this->cdaStall = _sdf->Get<double>("cda_stall");
	if (_sdf->HasElement("cma_stall"))
		this->cmaStall = _sdf->Get<double>("cma_stall");
	if (_sdf->HasElement("fluid_density"))
		this->rho = _sdf->Get<double>("fluid_density");

	//##################################################################//

	//*********************** Compute values for all links ******************************//

	for (auto link : this->model->GetLinks())
	{
		int id = link->GetId();
		ROS_INFO_NAMED("Hello", "this is link :%d", id);
		if (this->volPropsMap.find(id) == this->volPropsMap.end())
		{
			double volumeSum = 0;
			ignition::math::Vector3d weightedPosSum = ignition::math::Vector3d::Zero;
			math::Vector3 size = math::Vector3::Zero;
			math::Vector3 weightedPosSumCOP = math::Vector3::Zero;
			size = link->GetBoundingBox().GetSize();

			for (auto joint : link->GetChildJoints())
			{
				ROS_INFO_NAMED("joint retrieved","joint retrieved");
				math::Vector3 local_axis = math::Vector3::Zero;
				double a[3];
				local_axis = joint->GetLocalAxis(0);
				for(int i=0;i<3;i++)
				{
					if(abs(local_axis[i])>0.0)
						if(i<2)
							a[i+1]=local_axis[i];
						else
							a[0]=local_axis[i];
				}
				this->volPropsMap[id].upward = math::Vector3(a[0], a[1], a[2]);
				this->volPropsMap[id].forward = local_axis.Cross(this->volPropsMap[id].upward);
				this->volPropsMap[id].area = size.Dot(local_axis) * size.Dot(this->volPropsMap[id].forward);
				ROS_INFO_NAMED("length", "area: %0.7lf",this->volPropsMap[id].area);
				ROS_INFO_NAMED("loc", "local %0.7lf, %0.7lf, %0.7lf",local_axis.x,local_axis.y,local_axis.z);
				ROS_INFO_NAMED("up", "up %0.7lf, %0.7lf, %0.7lf",this->volPropsMap[id].upward.x,this->volPropsMap[id].upward.y,this->volPropsMap[id].upward.z);
				ROS_INFO_NAMED("forw", "forw %0.7lf, %0.7lf, %0.7lf",this->volPropsMap[id].forward.x,this->volPropsMap[id].forward.y,this->volPropsMap[id].forward.z);

			}

			for (auto collision : link->GetCollisions())
			{
				double volume = collision->GetShape()->ComputeVolume();
				volumeSum += volume;
				weightedPosSumCOP += volume*collision->GetWorldPose().pos;
				weightedPosSum += volume*collision->GetWorldPose().pos.Ign();
			}

			this->volPropsMap[id].cov = weightedPosSum/volumeSum - link->GetWorldPose().pos.Ign();
			this->volPropsMap[id].cop = weightedPosSumCOP/volumeSum - link->GetWorldPose().pos;
			this->volPropsMap[id].volume = volumeSum;
		}
	}
}

//##################################################################//

//*********************** Init ******************************//

void LiftDragPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&LiftDragPlugin::OnUpdate, this));
}

//###########################################################//

//*********************** Update forces ******************************//

void LiftDragPlugin::OnUpdate()
{

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

	for (auto link : this->model->GetLinks())
	{
		VolumeProperties volumeProperties = this->volPropsMap[link->GetId()];
		math::Vector3 vel = link->GetWorldLinearVel(volumeProperties.cop);
		if (vel.GetLength() <= 0.01)
			return;

		math::Pose pose = link->GetWorldPose();
		// rotate forward and upward vectors into inertial frame
		math::Vector3 forwardI = pose.rot.RotateVector(volumeProperties.forward);
		math::Vector3 upwardI = pose.rot.RotateVector(volumeProperties.upward);
		// ldNormal vector to lift-drag-plane described in inertial frame
		math::Vector3 ldNormal = forwardI.Cross(upwardI).Normalize();
		// check sweep (angle between vel and lift-drag-plane)
		double sinSweepAngle = ldNormal.Dot(vel) / vel.GetLength();
		// get cos from trig identity
		double cosSweepAngle2 = (1.0 - sinSweepAngle * sinSweepAngle);
		this->sweep = asin(sinSweepAngle);
		// truncate sweep to within +/-90 deg
		while (fabs(this->sweep) > 0.5 * M_PI)
		this->sweep = this->sweep > 0 ? this->sweep - M_PI : this->sweep + M_PI;

		// angle of attack is the angle between vel projected into lift-drag plane and forward vector
		// projected = ldNormal Xcross ( vector Xcross ldNormal)
		// so, velocity in lift-drag plane (expressed in inertial frame) is:
		math::Vector3 velInLDPlane = ldNormal.Cross(vel.Cross(ldNormal));
		// get direction of drag
		math::Vector3 dragDirection = -velInLDPlane;
		dragDirection.Normalize();

		// get direction of lift
		math::Vector3 liftDirection = ldNormal.Cross(velInLDPlane);
		liftDirection.Normalize();

		// get direction of moment
		math::Vector3 momentDirection = ldNormal;

		double cosAlpha = math::clamp(
		forwardI.Dot(velInLDPlane) /
		(forwardI.GetLength() * velInLDPlane.GetLength()), -1.0, 1.0);

		// gzerr << "ca " << forwardI.Dot(velInLDPlane) /(forwardI.GetLength() * velInLDPlane.GetLength()) << "\n";
		// get sign of alpha take upwards component of velocity in lift-drag plane.
		// if sign == upward, then alpha is negative

		double alphaSign = -upwardI.Dot(velInLDPlane)/
		(upwardI.GetLength() + velInLDPlane.GetLength());

		// double sinAlpha = sqrt(1.0 - cosAlpha * cosAlpha);
		if (alphaSign > 0.0)
		this->alpha = this->alpha0 + acos(cosAlpha);
		else
		this->alpha = this->alpha0 - acos(cosAlpha);

		// normalize to within +/-90 deg
		while (fabs(this->alpha) > 0.5 * M_PI)
		this->alpha = this->alpha > 0 ? this->alpha - M_PI
		                              : this->alpha + M_PI;

		// compute dynamic pressure
		double speedInLDPlane = velInLDPlane.GetLength();
		double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

		// compute cl at cp, check for stall, correct for sweep
		double cl;
		if (this->alpha > this->alphaStall)
		{
			cl = (this->cla * this->alphaStall + this->claStall * (this->alpha - this->alphaStall)) * cosSweepAngle2;
			// make sure cl is still great than 0
			cl = std::max(0.0, cl);
		}
		else if (this->alpha < -this->alphaStall)
		{
			cl = (-this->cla * this->alphaStall + this->claStall * (this->alpha + this->alphaStall)) * cosSweepAngle2;
			// make sure cl is still less than 0
			cl = std::min(0.0, cl);
		}
		else
			cl = this->cla * this->alpha * cosSweepAngle2;

		// compute lift force at cp
		math::Vector3 lift = cl * q * volumeProperties.area * liftDirection;
		// compute cd at cp, check for stall, correct for sweep
		double cd;

		if (this->alpha > this->alphaStall)
		{
			cd = (this->cda * this->alphaStall + this->cdaStall * (this->alpha - this->alphaStall)) * cosSweepAngle2;
		}
		else if (this->alpha < -this->alphaStall)
		{
			cd = (-this->cda * this->alphaStall + this->cdaStall * (this->alpha + this->alphaStall)) * cosSweepAngle2;
		}
		else
			cd = (this->cda * this->alpha) * cosSweepAngle2;

		// make sure drag is positive
		cd = fabs(cd);
		// drag at cp
		math::Vector3 drag = cd * q * volumeProperties.area * dragDirection;
		// compute cm at cp, check for stall, correct for sweep
		double cm;
		if (this->alpha > this->alphaStall)
		{
			cm = (this->cma * this->alphaStall + this->cmaStall * (this->alpha - this->alphaStall)) * cosSweepAngle2;
			// make sure cm is still great than 0
			cm = std::max(0.0, cm);
		}
		else if (this->alpha < -this->alphaStall)
		{
			cm = (-this->cma * this->alphaStall + this->cmaStall * (this->alpha + this->alphaStall)) * cosSweepAngle2;
			// make sure cm is still less than 0
			cm = std::min(0.0, cm);
		}
		else
			cm = this->cma * this->alpha * cosSweepAngle2;

		// reset cm to zero, as cm needs testing
		cm = 0.0;

		// compute moment (torque) at cp
		math::Vector3 moment = cm * q * volumeProperties.area * momentDirection;

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
			gzerr << "sweep: " << this->sweep << "\n";
			gzerr << "alpha: " << this->alpha << "\n";
			gzerr << "lift: " << lift << "\n";
			gzerr << "drag: " << drag << " cd: "<< cd << " cda: " << this->cda << "\n";
			gzerr << "moment: " << moment << "\n";
			gzerr << "cp momentArm: " << momentArm << "\n";
			gzerr << "force: " << force << "\n";
			gzerr << "torque: " << torque << "\n";
		}

		link->AddForceAtRelativePosition(force, volumeProperties.cop);
		link->AddTorque(torque);
	}
}

//##################################################################//