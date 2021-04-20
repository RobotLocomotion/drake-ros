/*****************************************************************************
 * Copyright (c) 2017-2021, Massachusetts Institute of Technology.
 * Copyright (c) 2017-2021, Toyota Research Institute.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <iostream>

#include <drake/geometry/drake_visualizer.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>

int main(int argc, char** argv) {
  drake::systems::DiagramBuilder<double> builder;
  auto [pendulum_plant, scene_graph] =
      drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  auto parser = drake::multibody::Parser(&pendulum_plant, &scene_graph);

  // Populate from AMENT_PREFIX_PATH environment variable to find URDF files and
  // their resources, such as meshes.
  parser.package_map().PopulateFromEnvironment("AMENT_PREFIX_PATH");
  const std::string pendulum_desc_package =
      "drake_example_pendulum_description";

  if (!parser.package_map().Contains(pendulum_desc_package))
  {
    std::cerr << "The package: '" << pendulum_desc_package
              << "' could not be found. Have you sourced your ROS workspace?"
              << std::endl;
    return -1;
  }

  const std::string package_path =
      parser.package_map().GetPath(pendulum_desc_package);
  auto model_instance = parser.AddModelFromFile(
      package_path + "/urdf/drake_example_pendulum.urdf");

  const auto& base_link =
      pendulum_plant.GetFrameByName("base_link", model_instance);
  // Weld the base_link to the world so the pendulum doesn't fall forever.
  pendulum_plant.WeldFrames(pendulum_plant.world_frame(), base_link, {});

  pendulum_plant.Finalize();

  // The following line can be commented out if Drake visualizer is not needed.
  drake::geometry::DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);

  drake::systems::Context<double>& pendulum_context =
      diagram->GetMutableSubsystemContext(pendulum_plant,
                                          &simulator.get_mutable_context());

  drake::VectorX<double> joint_position(1);
  joint_position << 0.5;
  pendulum_plant.SetPositions(&pendulum_context, joint_position);

  simulator.set_monitor([&pendulum = pendulum_plant](const auto& root_context) {
    auto &context = pendulum.GetMyContextFromRoot(root_context);
    std::cout << fmt::format("{:0.3f}: {}", context.get_time(),
                             pendulum.GetPositions(context).transpose())
              << std::endl;
    return drake::systems::EventStatus::Succeeded();
  });

  simulator.AdvanceTo(10);
  return 0;
}
