#ifndef LINKRESOLVER_H
#define LINKRESOLVER_H

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>

#include <string>
#include <vector>

// Model::LinkByName only searches an entity's direct children, but a link
// commonly lives inside a nested <include> (e.g. abv's base_link is a child
// of the "base" sub-model, not of abv itself). This does a breadth-first
// search of aRoot's whole descendant tree for a Link entity with a matching
// unscoped name, so plugins can target any link regardless of nesting depth.
inline ignition::gazebo::Entity resolveLinkEntity(
    ignition::gazebo::Entity aRoot,
    const ignition::gazebo::EntityComponentManager &ecm,
    const std::string &aLinkName)
{
  std::vector<ignition::gazebo::Entity> frontier{aRoot};

  while (!frontier.empty())
  {
    std::vector<ignition::gazebo::Entity> next;

    for (auto parent : frontier)
    {
      for (auto child : ecm.EntitiesByComponents(ignition::gazebo::components::ParentEntity(parent)))
      {
        if (ecm.Component<ignition::gazebo::components::Link>(child))
        {
          auto name = ecm.Component<ignition::gazebo::components::Name>(child);
          if (name && name->Data() == aLinkName)
          {
            return child;
          }
        }

        next.push_back(child);
      }
    }

    frontier = next;
  }

  return ignition::gazebo::kNullEntity;
}

#endif // LINKRESOLVER_H
