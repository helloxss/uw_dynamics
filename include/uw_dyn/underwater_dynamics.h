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
#ifndef _GAZEBO_UNDERWATER_DYNAMICS_HH_
#define _GAZEBO_UNDERWATER_DYNAMICS_HH_

#include <string>
#include <vector>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{

  /// \brief A class for storing the volume properties of a link.
  class VolumeProperties
  {
    /// \brief Default constructor.
    public: VolumeProperties() : volume(0) {}

    /// \brief Center of volume in the link frame.
    public: ignition::math::Vector3d cov;
    public: math::Vector3 cop;
    /// \brief forward flight direction in link local coordinates
    public: math::Vector3 forward;

    /// \brief A vector in the lift/drag plane, anything orthogonal to it
    /// is considered wing sweep.
    public: math::Vector3 upward;

    /// \brief Volume of this link.
    public: double volume;

    /// \brief effective planeform surface area
    public: double area;

    /// \brief angle of sweep
    public: double sweep;

    /// \brief angle of attach when airfoil stalls
    public: double alphaStall;

    /// \brief initial angle of attack
    public: double alpha0;

    /// \brief angle of attack
    public: double alpha;

    /// \brief Cl-alpha rate after stall
    public: double claStall;

    /// \brief Cd-alpha rate after stall
    public: double cdaStall;

    /// \brief Cm-alpha rate after stall
    public: double cmaStall;

    /// \brief Coefficient of Lift / alpha slope.
    /// Lift = C_L * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    public: double cla;

    /// \brief Coefficient of Drag / alpha slope.
    /// Drag = C_D * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    public: double cda;

    /// \brief Coefficient of Moment / alpha slope.
    /// Moment = C_M * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    public: double cma;
  };

  /// \brief A plugin that simulates lift and drag.
  class LiftDragPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: LiftDragPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    protected: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    protected: physics::ModelPtr model;

    /// \brief Name of model containing plugin.
    protected: std::string modelName;

    /// \brief: \TODO: make a stall velocity curve
    protected: double velocityStall;

    /// \brief air density
    /// at 25 deg C it's about 1.1839 kg/m^3
    /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
    protected: double rho;

    /// \brief Smooth velocity
    protected: math::Vector3 velSmooth;

    /// \brief Pointer to link currently targeted by mud joint.
    protected: physics::LinkPtr link;

    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;

    /// \brief The density of the fluid in which the object is submerged in
    /// kg/m^3. Defaults to 1000, the fluid density of water.
    protected: double fluidDensity;

    /// \brief Map of <link ID, point> pairs mapping link IDs to the CoV (center
    /// of volume) and volume of the link.
    protected: std::map<int, VolumeProperties> volPropsMap;

  };
}
#endif