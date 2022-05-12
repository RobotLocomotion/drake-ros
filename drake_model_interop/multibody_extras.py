from pydrake.multibody.tree import (
    BodyIndex,
    FrameIndex,
    JointActuatorIndex,
    JointIndex,
    ModelInstanceIndex,
)


def _get_plant_aggregate(num_func, get_func, index_cls, model_instances=None):
    items = []
    for i in range(num_func()):
        item = get_func(index_cls(i))
        if model_instances is None or item.model_instance() in model_instances:
            items.append(item)
    return items


def get_model_instances(plant):
    # TODO(eric.cousineau): Hoist this somewhere?
    return _get_plant_aggregate(
        plant.num_model_instances, lambda x: x, ModelInstanceIndex
    )


def get_bodies(plant, model_instances=None):
    # TODO(eric.cousineau): Hoist this somewhere?
    return _get_plant_aggregate(
        plant.num_bodies, plant.get_body, BodyIndex, model_instances
    )


def get_frames(plant, model_instances=None):
    # TODO(eric.cousineau): Hoist this somewhere?
    return _get_plant_aggregate(
        plant.num_frames, plant.get_frame, FrameIndex, model_instances
    )


def get_frames_attached_to(plant, bodies):
    # TODO(eric.cousineau): Hoist this somewhere?
    frames = []
    for frame in get_frames(plant):
        if frame.body() in bodies:
            frames.append(frame)
    return frames


def get_joints(plant, model_instances=None):
    # TODO(eric.cousineau): Hoist this somewhere?
    return _get_plant_aggregate(
        plant.num_joints, plant.get_joint, JointIndex, model_instances
    )


def is_joint_solely_connected_to(joint, bodies):
    # TODO(eric.cousineau): Hoist this somewhere?
    parent = joint.parent_body()
    child = joint.child_body()
    return parent in bodies and child in bodies


def get_joints_solely_connected_to(plant, bodies):
    # TODO(eric.cousineau): Hoist this somewhere?
    return [
        joint
        for joint in get_joints(plant)
        if is_joint_solely_connected_to(joint, bodies)
    ]


def get_joint_actuators(plant, model_instances=None):
    # TODO(eric.cousineau): Hoist this somewhere?
    return _get_plant_aggregate(
        plant.num_actuators, plant.get_joint_actuator, JointActuatorIndex
    )


def get_joint_actuators_affecting_joints(plant, joints):
    # TODO(eric.cousineau): Hoist this somewhere?
    joint_actuators = []
    for joint_actuator in get_joint_actuators(plant):
        if joint_actuator.joint() in joints:
            joint_actuators.append(joint_actuator)
    return joint_actuators


def get_or_add_model_instance(plant, name):
    # TODO(eric.cousineau): Hoist this somewhere?
    if not plant.HasModelInstanceNamed(name):
        return plant.AddModelInstance(name)
    else:
        return plant.GetModelInstanceByName(name)


def get_geometries(plant, scene_graph, bodies):
    """Returns all GeometryId's attached to bodies. Assumes corresponding
    FrameId's have been added."""
    geometry_ids = []
    inspector = scene_graph.model_inspector()
    for geometry_id in inspector.GetAllGeometryIds():
        body = plant.GetBodyFromFrameId(inspector.GetFrameId(geometry_id))
        if body in bodies:
            geometry_ids.append(geometry_id)
    return sorted(geometry_ids, key=lambda x: x.get_value())


def get_joint_positions(plant, context, joint):
    # TODO(eric.cousineau): Hoist to C++ / pydrake.
    q = plant.GetPositions(context)
    start = joint.position_start()
    count = joint.num_positions()
    return q[start : start + count].copy()


def set_joint_positions(plant, context, joint, qj):
    # TODO(eric.cousineau): Hoist to C++ / pydrake.
    q = plant.GetPositions(context)
    start = joint.position_start()
    count = joint.num_positions()
    q[start : start + count] = qj
    plant.SetPositions(context, q)


def get_joint_velocities(plant, context, joint):
    # TODO(eric.cousineau): Hoist to C++ / pydrake.
    v = plant.GetVelocities(context)
    start = joint.velocity_start()
    count = joint.num_velocities()
    return v[start : start + count].copy()


def set_joint_velocities(plant, context, joint, vj):
    # TODO(eric.cousineau): Hoist to C++ / pydrake.
    v = plant.GetVelocities(context)
    start = joint.velocity_start()
    count = joint.num_velocities()
    v[start : start + count] = vj
    plant.SetVelocities(context, v)
