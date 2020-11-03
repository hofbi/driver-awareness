#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Scenario where the hero is doing a left turn additionally to the coming
vehicle there is a parking one and another driving on the second lane after
turning left
"""

# these lines suppress pylint warnings about the module file name
# without silencing other invalid-name errors
# pylint: disable=invalid-name
# pylint: enable=invalid-name

from six.moves.queue import Queue  # pylint: disable=relative-import

import py_trees
import carla
from agents.navigation.local_planner import RoadOption

from srunner.scenariomanager.carla_data_provider import (
    CarlaDataProvider,
    CarlaActorPool,
)
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter,
    ActorDestroy,
    ActorSource,
    ActorSink,
    KeepVelocity,
    WaypointFollower,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
    InTriggerDistanceToVehicle,
)
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import generate_target_waypoint


class LeftTurnTwoVehicles(BasicScenario):
    """
    Implementation class for Hero
    Vehicle turning left at signalized junction scenario,
    Traffic Scenario 08.

    This is a single ego vehicle scenario
    """

    timeout = 80  # Timeout of scenario in seconds

    def __init__(
        self,
        world,
        ego_vehicles,
        config,
        randomize=False,
        debug_mode=False,
        criteria_enable=True,
        timeout=120,
    ):
        """
        Setup all relevant parameters and create scenario
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._target_vel = 6.9
        self._brake_value = 0.5
        self._ego_distance = 140
        self._traffic_light = None
        self._other_actor_transform = None
        self._blackboard_queue_name = "LeftTurnTwoVehicles/actor_flow_queue"
        self._blackboard_queue_name2 = "LeftTurnTwoVehicles/actor_flow_queue2"
        self._queue = py_trees.blackboard.Blackboard().set(
            self._blackboard_queue_name, Queue()
        )
        self._queue2 = py_trees.blackboard.Blackboard().set(
            self._blackboard_queue_name2, Queue()
        )
        self._initialized = True
        super(LeftTurnTwoVehicles, self).__init__(
            "LeftTurnTwoVehicles",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

        self._traffic_light = CarlaDataProvider.get_next_traffic_light(
            self.ego_vehicles[0], False
        )
        traffic_light_other = CarlaDataProvider.get_next_traffic_light(
            self.other_actors[0], False
        )
        if self._traffic_light is None or traffic_light_other is None:
            raise RuntimeError("No traffic light for the given location found")
        self._traffic_light.set_state(carla.TrafficLightState.Green)
        self._traffic_light.set_green_time(self.timeout)
        # other vehicle's traffic light
        traffic_light_other.set_state(carla.TrafficLightState.Green)
        traffic_light_other.set_green_time(self.timeout)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        self._other_actor_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(
                config.other_actors[0].transform.location.x,
                config.other_actors[0].transform.location.y,
                config.other_actors[0].transform.location.z - 500,
            ),
            config.other_actors[0].transform.rotation,
        )
        try:
            first_vehicle = CarlaActorPool.request_new_actor(
                config.other_actors[0].model, self._other_actor_transform
            )
        except RuntimeError as r_error:
            raise r_error
        first_vehicle.set_transform(first_vehicle_transform)
        self.other_actors.append(first_vehicle)

        self._other_actor_transform2 = config.other_actors[1].transform
        first_vehicle_transform2 = carla.Transform(
            carla.Location(
                config.other_actors[1].transform.location.x,
                config.other_actors[1].transform.location.y,
                config.other_actors[1].transform.location.z,
            ),
            config.other_actors[1].transform.rotation,
        )
        try:
            first_vehicle2 = CarlaActorPool.request_new_actor(
                config.other_actors[1].model, self._other_actor_transform2
            )
        except RuntimeError as r_error:
            raise r_error
        first_vehicle2.set_transform(first_vehicle_transform2)
        self.other_actors.append(first_vehicle2)

        self._other_actor_transform3 = config.other_actors[2].transform
        first_vehicle_transform3 = carla.Transform(
            carla.Location(
                config.other_actors[2].transform.location.x,
                config.other_actors[2].transform.location.y,
                config.other_actors[2].transform.location.z,
            ),
            config.other_actors[2].transform.rotation,
        )
        try:
            first_vehicle3 = CarlaActorPool.request_new_actor(
                config.other_actors[2].model, self._other_actor_transform3
            )
        except RuntimeError as r_error:
            raise r_error
        first_vehicle3.set_transform(first_vehicle_transform3)
        self.other_actors.append(first_vehicle3)
        print("Init done.")

    def _create_behavior(self):
        """
        Hero vehicle is turning left in an urban area,
        at a signalized intersection, while other actor coming straight
        .The hero actor may turn left either before other actor
        passes intersection or later, without any collision.
        After 80 seconds, a timeout stops the scenario.
        """

        sequence = py_trees.composites.Sequence("Sequence Behavior")

        # Selecting straight path at intersection
        target_waypoint = generate_target_waypoint(
            CarlaDataProvider.get_map().get_waypoint(
                self.other_actors[0].get_location()
            ),
            0,
        )
        # Generating waypoint list till next intersection
        plan = []
        wp_choice = target_waypoint.next(1.0)
        while not wp_choice[0].is_intersection:
            target_waypoint = wp_choice[0]
            plan.append((target_waypoint, RoadOption.LANEFOLLOW))
            wp_choice = target_waypoint.next(1.0)
        # adding flow of actors
        actor_source = ActorSource(
            ["vehicle.tesla.model3"],
            self._other_actor_transform,
            15,
            self._blackboard_queue_name,
        )
        # destroying flow of actors
        actor_sink = ActorSink(plan[-1][0].transform.location, 10)
        # follow waypoints untill next intersection
        move_actor = WaypointFollower(
            self.other_actors[0],
            self._target_vel,
            plan=plan,
            blackboard_queue_name=self._blackboard_queue_name,
            avoid_collision=True,
        )
        # wait
        wait = DriveDistance(self.ego_vehicles[0], self._ego_distance)

        # actor_source2 = ActorSource(['vehicle.audi.tt'],
        #                             self._other_actor_transform2, 10,
        #                             self._blackboard_queue_name2)
        # # destroying flow of actors
        # actor_sink2 = ActorSink(carla.Location(x=-74.9, y=30), 20)
        # # follow waypoints untill next intersection
        # move_actor2 = WaypointFollower(
        #     self.other_actors[1],
        #     self._target_vel,
        #     blackboard_queue_name=self._blackboard_queue_name2,
        #     avoid_collision=True)

        distance_to_vehicle = InTriggerDistanceToVehicle(
            self.other_actors[1], self.ego_vehicles[0], 20
        )
        actor1_drive = KeepVelocity(self.other_actors[1], 8, distance=90)

        # Behavior tree
        root1 = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        # root1.add_child(wait)
        root1.add_child(actor_source)
        root1.add_child(actor_sink)
        root1.add_child(move_actor)
        # root2 = py_trees.composites.Parallel(
        #     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root2 = py_trees.composites.Sequence(
            "Sequence_dist", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
        )
        # root2.add_child(actor_source2)
        # root2.add_child(actor_sink2)
        # root2.add_child(move_actor2)
        root2.add_child(distance_to_vehicle)
        root2.add_child(actor1_drive)

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        root.add_child(root1)
        root.add_child(root2)
        root.add_child(wait)

        sequence.add_child(
            ActorTransformSetter(self.other_actors[0], self._other_actor_transform)
        )
        sequence.add_child(root)
        sequence.add_child(ActorDestroy(self.other_actors[0]))
        sequence.add_child(ActorDestroy(self.other_actors[1]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collison_criteria = CollisionTest(self.ego_vehicles[0])
        criteria.append(collison_criteria)

        return criteria

    def __del__(self):
        self._traffic_light = None
        self.remove_all_actors()
