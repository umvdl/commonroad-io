import copy
import unittest
import numpy as np
from commonroad.scenario.lanelet import Lanelet, LineMarking, LaneletNetwork, StopLine, LaneletType
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.state import InitialState
from commonroad.scenario.traffic_sign import TrafficSignElement, TrafficSign, TrafficSignIDGermany, TrafficLight, \
    TrafficLightCycleElement, TrafficLightState
from commonroad.scenario.intersection import Intersection, IntersectionIncomingElement


class TestLaneletNetwork(unittest.TestCase):

    def setUp(self):
        self.right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        self.left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        self.center_vertices = np.array(
                [[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, .5], [8, .5]])
        self.lanelet_id = 5
        self.predecessor = [1, 2]
        self.successor = [6, 7]
        self.adjacent_left = 12
        self.adjacent_right = 4
        self.adjacent_right_same_dir = True
        self.adjacent_left_same_dir = False
        self.line_marking_right = LineMarking.SOLID
        self.line_marking_left = LineMarking.DASHED
        traffic_sign_max_speed = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED.value, ["15"])
        self.traffic_sign = TrafficSign(1, [traffic_sign_max_speed], {5}, np.array([0.0, 0.0]))
        cycle = [TrafficLightCycleElement(TrafficLightState.GREEN, 2),
                 TrafficLightCycleElement(TrafficLightState.YELLOW, 3),
                 TrafficLightCycleElement(TrafficLightState.RED, 2)]
        self.traffic_light = TrafficLight(567, cycle, position=np.array([10., 10.]))
        self.stop_line = StopLine(self.left_vertices[-1], self.right_vertices[-1], LineMarking.SOLID,
                                  {self.traffic_sign.traffic_sign_id}, {self.traffic_light.traffic_light_id})

        incoming_1 = IntersectionIncomingElement(2, {self.lanelet_id, 11}, {12, 13}, {14, 15}, {16, 17}, 18)
        incoming_2 = IntersectionIncomingElement(3, {20, 21}, {22, 23}, {24, 25}, {26, 27}, 28)
        self.intersection = Intersection(1, [incoming_1, incoming_2], {30, 31})

        self.lanelet = Lanelet(self.left_vertices, self.center_vertices, self.right_vertices, self.lanelet_id,
                               self.predecessor, self.successor, self.adjacent_left, self.adjacent_left_same_dir,
                               self.adjacent_right, self.adjacent_right_same_dir, self.line_marking_left,
                               self.line_marking_right, traffic_signs={self.traffic_sign.traffic_sign_id},
                               traffic_lights={self.traffic_light.traffic_light_id}, stop_line=self.stop_line)

        self.lanelet_2 = Lanelet(np.array([[8, 1], [9, 1]]), np.array([[8, .5], [9, .5]]), np.array([[8, 0], [9, 0]]),
                                 6, [self.lanelet.lanelet_id], [678], 910, True, 750, False, LineMarking.UNKNOWN,
                                 LineMarking.UNKNOWN)

        self.lanelet_network = LaneletNetwork()
        self.lanelet_network.add_lanelet(self.lanelet)
        self.lanelet_network.add_lanelet(self.lanelet_2)
        self.lanelet_network.add_traffic_sign(self.traffic_sign, set())
        self.lanelet_network.add_traffic_light(self.traffic_light, set())
        self.lanelet_network.add_intersection(self.intersection)

        self.diagonal_lanelet_network = LaneletNetwork()
        lanelet_width = np.array([0.0, 3.0])
        right_vertices = np.array([[0., 0.], [6., 0.1], [12., 0.5]])
        left_vertices = copy.copy(right_vertices) + lanelet_width
        center_vertices = (right_vertices + left_vertices) * 0.5
        lanelet_id = 0
        self.diagonal_lanelet_network.add_lanelet(Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id))

        left_vertices = copy.copy(right_vertices)
        right_vertices = copy.copy(left_vertices) - lanelet_width
        center_vertices = (right_vertices + left_vertices) * 0.5
        lanelet_id = 1
        self.diagonal_lanelet_network.add_lanelet(Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id))

    def test_initialize_lanelets(self):
        s1 = np.sqrt(1.25)
        s2 = np.sqrt(2.0)
        desired_dist = [0.0, 1.0, 2.0, 2.0 + s1, 2.0 + 2 * s1, 3.0 + 2 * s1, 4.0 + 2 * s1, 4.0 + 2 * s1 + s2,
                        5.0 + 2 * s1 + s2]
        for i, dist in enumerate(self.lanelet.distance):
            self.assertAlmostEqual(dist, desired_dist[i])

        self.assertEqual(self.lanelet.lanelet_id, self.lanelet_id)
        np.testing.assert_array_almost_equal(self.lanelet.right_vertices, self.right_vertices)
        np.testing.assert_array_almost_equal(self.lanelet.left_vertices, self.left_vertices)
        np.testing.assert_array_almost_equal(self.lanelet.center_vertices, self.center_vertices)
        np.testing.assert_array_almost_equal(self.lanelet.predecessor, self.predecessor)
        np.testing.assert_array_almost_equal(self.lanelet.successor, self.successor)
        self.assertEqual(self.lanelet.adj_left, self.adjacent_left)
        self.assertEqual(self.lanelet.adj_right, self.adjacent_right)
        self.assertEqual(self.lanelet.adj_left_same_direction, self.adjacent_left_same_dir)
        self.assertEqual(self.lanelet.adj_right_same_direction, self.adjacent_right_same_dir)
        self.assertEqual(self.lanelet.line_marking_left_vertices, self.line_marking_left)
        self.assertEqual(self.lanelet.line_marking_right_vertices, self.line_marking_right)

        self.assertEqual(self.lanelet_network.lanelets[0].lanelet_id, self.lanelet.lanelet_id)

    def test_create_from_lanelet_network(self):
        lanelet_network = LaneletNetwork()

        right_vertices = np.array([[0, 0], [1, 0], [1.1, 0.1]])
        left_vertices = np.array([[0, 1], [1, 1], [1.1, 1.1]])
        center_vertices = np.array([[0, .5], [1, .5], [1.1, .6]])
        lanelet_id = 5
        lanelet1 = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id)
        lanelet_network.add_lanelet(lanelet1)
        lanelet_network.add_traffic_sign(self.traffic_sign, {lanelet1.lanelet_id})
        lanelet_network.add_traffic_light(self.traffic_light, {lanelet1.lanelet_id})

        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2]])
        center_vertices = np.array([[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5]])
        lanelet_id = 6
        lanelet_type = {LaneletType.URBAN}
        lanelet2 = Lanelet(left_vertices, right_vertices, center_vertices, lanelet_id, None, None, None, None, None,
                           None, None, None, None, lanelet_type, None, None, {self.traffic_sign.traffic_sign_id},
                           {self.traffic_light.traffic_light_id})
        lanelet_network.add_lanelet(lanelet2)

        right_vertices = np.array([[5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array([[5, 1.5], [6, 1.5], [7, .5], [8, .5]])
        lanelet_id = 7
        lanelet3 = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id)
        lanelet_network.add_lanelet(lanelet3)

        new_network = lanelet_network.create_from_lanelet_network(lanelet_network, Rectangle(2, 2))

        a = False
        for i in range(0, len(new_network.lanelets)):
            if lanelet1.lanelet_id == new_network.lanelets[i].lanelet_id:
                a = True
        self.assertTrue(a)

        a = False
        for i in range(0, len(new_network.lanelets)):
            if lanelet2.lanelet_id == new_network.lanelets[i].lanelet_id:
                a = True
        self.assertTrue(a)

        a = False
        for i in range(0, len(new_network.lanelets)):
            if lanelet3.lanelet_id == new_network.lanelets[i].lanelet_id:
                a = True
        self.assertFalse(a)

        self.assertEqual(lanelet2.traffic_signs, {1})
        self.assertEqual(lanelet2.traffic_lights, {567})
        self.assertEqual(lanelet1.traffic_signs, {1})
        self.assertEqual(lanelet1.traffic_lights, {567})

        lanelets_in_network = []
        for i in range(0, len(new_network.lanelets)):
            lanelets_in_network.append(new_network.lanelets[i])
        self.assertNotIn(lanelet3.lanelet_id, lanelets_in_network)

        new_network_lanelet_types = lanelet_network.create_from_lanelet_network(lanelet_network, Rectangle(2, 2),
                                                                                {LaneletType.URBAN})
        lanelets_in_network = []
        for i in range(0, len(new_network_lanelet_types.lanelets)):
            lanelets_in_network.append(new_network_lanelet_types.lanelets[i])

        self.assertNotIn(lanelet2.lanelet_id, lanelets_in_network)
        self.assertEquals(lanelet1.traffic_signs, new_network_lanelet_types.lanelets[0].traffic_signs)
        self.assertEquals(lanelet1.traffic_lights, new_network_lanelet_types.lanelets[0].traffic_lights)

    def create_from_lanelet_list(self):
        new_network = LaneletNetwork.create_from_lanelet_list([self.lanelet])

        for lanelet_act, lanelet_des in zip(new_network.lanelets, self.lanelet_network.lanelets):
            np.testing.assert_array_almost_equal(lanelet_act.right_vertices, lanelet_des.right_vertices)
            np.testing.assert_array_almost_equal(lanelet_act.center_vertices, lanelet_des.center_vertices)
            np.testing.assert_array_almost_equal(lanelet_act.left_vertices, lanelet_des.left_vertices)
            self.assertEqual(lanelet_act.lanelet_id, lanelet_des.lanelet_id)

    def test_find_lanelet_by_id(self):
        actual_lanelet_found = self.lanelet_network.find_lanelet_by_id(5)

        np.testing.assert_array_almost_equal(actual_lanelet_found.right_vertices, self.lanelet.right_vertices)
        np.testing.assert_array_almost_equal(actual_lanelet_found.center_vertices, self.lanelet.center_vertices)
        np.testing.assert_array_almost_equal(actual_lanelet_found.left_vertices, self.lanelet.left_vertices)
        self.assertEqual(actual_lanelet_found.lanelet_id, self.lanelet.lanelet_id)
        self.assertEqual(self.lanelet_network.find_lanelet_by_id(2), None)

    def test_add_lanelet(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array([[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, .5], [8, .5]])
        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 2)

        self.assertTrue(self.lanelet_network.add_lanelet(lanelet))

        np.testing.assert_array_almost_equal(self.lanelet_network.lanelets[0].right_vertices,
                                             self.lanelet.right_vertices)
        np.testing.assert_array_almost_equal(self.lanelet_network.lanelets[0].center_vertices,
                                             self.lanelet.center_vertices)
        np.testing.assert_array_almost_equal(self.lanelet_network.lanelets[0].left_vertices, self.lanelet.left_vertices)
        self.assertEqual(self.lanelet_network.lanelets[0].lanelet_id, self.lanelet.lanelet_id)

    def test_add_traffic_sign(self):
        traffic_sign_max_speed = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED.value, ["15"])
        traffic_sign = TrafficSign(123, [traffic_sign_max_speed], {5}, np.array([0.0, 0.0]))

        self.assertTrue(self.lanelet_network.add_traffic_sign(traffic_sign, {5}))

        self.assertEqual(self.lanelet_network.traffic_signs[1].traffic_sign_id, traffic_sign.traffic_sign_id)
        self.assertSetEqual(self.lanelet_network.lanelets[0].traffic_signs, {123, 1})

    def test_add_traffic_light(self):
        cycle = [TrafficLightCycleElement(TrafficLightState.GREEN, 2),
                 TrafficLightCycleElement(TrafficLightState.YELLOW, 3),
                 TrafficLightCycleElement(TrafficLightState.RED, 2)]
        traffic_light = TrafficLight(234, cycle, time_offset=5, position=np.array([10., 10.]))

        self.assertTrue(self.lanelet_network.add_traffic_light(traffic_light, {5}))

        self.assertEqual(self.lanelet_network.traffic_lights[1].traffic_light_id, traffic_light.traffic_light_id)
        self.assertSetEqual(self.lanelet_network.lanelets[0].traffic_lights, {234, 567})

    def test_add_lanelets_from_network(self):

        actual_network = LaneletNetwork()
        actual_network.add_lanelets_from_network(self.lanelet_network)

        for lanelet_act, lanelet_des in zip(actual_network.lanelets, self.lanelet_network.lanelets):
            np.testing.assert_array_almost_equal(lanelet_act.right_vertices, lanelet_des.right_vertices)
            np.testing.assert_array_almost_equal(lanelet_act.center_vertices, lanelet_des.center_vertices)
            np.testing.assert_array_almost_equal(lanelet_act.left_vertices, lanelet_des.left_vertices)
            self.assertEqual(lanelet_act.lanelet_id, lanelet_des.lanelet_id)

    def test_translate_rotate(self):

        self.lanelet_network.translate_rotate(np.array([2, -4]), np.pi / 2)

        desired_lanelet_center = np.array(
                [[3.5, 2], [3.5, 3], [3.5, 4], [3, 5], [2.5, 6], [2.5, 7], [2.5, 8], [3.5, 9], [3.5, 10]])
        desired_traffic_sign_position = np.array([4, 2])

        np.testing.assert_array_almost_equal(self.lanelet_network.lanelets[0].center_vertices, desired_lanelet_center)
        np.testing.assert_array_almost_equal(self.lanelet_network.traffic_signs[0].position,
                                             desired_traffic_sign_position)

    def test_translate_invalid(self):

        with self.assertRaises(AssertionError):
            self.lanelet_network.translate_rotate(np.array([2, -4]), 320)
        with self.assertRaises(AssertionError):
            self.lanelet_network.translate_rotate(np.array([3, 5, -7]), np.pi / 2)
        with self.assertRaises(AssertionError):
            self.lanelet_network.translate_rotate(np.array([3]), np.pi / 2)
        with self.assertRaises(AssertionError):
            self.lanelet_network.translate_rotate(0.0, np.pi / 2)

    def test_find_lanelet_by_position(self):
        additional_lanelet_network = LaneletNetwork.create_from_lanelet_network(self.lanelet_network)

        observed_lanelet = self.lanelet_network.find_lanelet_by_position([np.array([1, 1])])
        self.assertEqual(observed_lanelet[0][0], self.lanelet.lanelet_id)
        self.assertEqual(len(self.lanelet_network.find_lanelet_by_position([np.array([-5, -5])])[0]), 0)

        observed_lanelet = additional_lanelet_network.find_lanelet_by_position([np.array([1, 1])])
        self.assertEqual(observed_lanelet[0][0], self.lanelet.lanelet_id)
        self.assertEqual(len(additional_lanelet_network.find_lanelet_by_position([np.array([-5, -5])])[0]), 0)

        tolerance = 1e-14

        def assert_pos(vertex, lanelet_id_list):
            [ret_list] = self.diagonal_lanelet_network.find_lanelet_by_position([vertex])
            ret_list.sort()
            lanelet_id_list.sort()
            self.assertEqual(ret_list, lanelet_id_list)

        lanelet_0 = self.diagonal_lanelet_network.find_lanelet_by_id(0)
        lanelet_1 = self.diagonal_lanelet_network.find_lanelet_by_id(1)
        for dist_i in list(np.linspace(0.0, lanelet_0.distance[2], 1000)):
            center_vertex, right_vertex, left_vertex, _ = lanelet_0.interpolate_position(dist_i)
            assert_pos(left_vertex, [0])
            assert_pos(left_vertex + np.array([0.0, 0.5 * tolerance]), [])

            assert_pos(center_vertex, [0])

            assert_pos(right_vertex, [0, 1])
            assert_pos(right_vertex + np.array([0.0, 0.5 * tolerance]), [0])
            assert_pos(right_vertex - np.array([0.0, 0.5 * tolerance]), [1])

            center_vertex, right_vertex, left_vertex, _ = lanelet_1.interpolate_position(dist_i)
            assert_pos(left_vertex, [0, 1])
            assert_pos(left_vertex + np.array([0.0, 0.5 * tolerance]), [0])
            assert_pos(left_vertex - np.array([0.0, 0.5 * tolerance]), [1])

            assert_pos(center_vertex, [1])

            assert_pos(right_vertex, [1])
            assert_pos(right_vertex - np.array([0.0, 0.5 * tolerance]), [])

        assert_pos(np.array([-tolerance, 0.]), [])
        assert_pos(np.array([lanelet_0.center_vertices[-1][0] + tolerance, 0.]), [])

    def test_find_lanelet_by_shape(self):
        rectangle1 = Rectangle(2, 2)
        rectangle2 = Rectangle(2, 2, np.array([100.0, 100.0]))
        rectangle3 = Rectangle(2, 2, np.array([9.0, 0.0]))

        observed_lanelet = self.lanelet_network.find_lanelet_by_shape(rectangle1)
        self.assertEqual(observed_lanelet[0], self.lanelet.lanelet_id)
        observed_lanelet = self.lanelet_network.find_lanelet_by_shape(rectangle2)
        self.assertEqual(observed_lanelet, [])
        observed_lanelets = self.lanelet_network.find_lanelet_by_shape(rectangle3)
        self.assertEqual([self.lanelet_2.lanelet_id, self.lanelet.lanelet_id], observed_lanelets)

    def test_find_most_likely_lanelet_by_state(self):

        left_vertices = np.array([[0, 1], [2, 1], [8, 1]])
        center_vertices = np.array([[0, 0], [2, 0], [8, 0]])
        right_vertices = np.array([[0, -1], [2, -1], [8, -1]])

        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1)

        lanelet_2 = Lanelet(left_vertices, center_vertices, right_vertices, 2)
        lanelet_2.translate_rotate(translation=np.array([0., 0.]), angle=np.deg2rad(45))

        lanelet_network = LaneletNetwork()
        lanelet_network.add_lanelet(lanelet)
        lanelet_network.add_lanelet(lanelet_2)

        # from commonroad.visualization.mp_renderer import MPRenderer
        # cr_render = MPRenderer()
        # lanelet_network.draw(cr_render, draw_params={"lanelet": {"show_label": True}})
        # cr_render.render(show=True)

        state = InitialState(position=np.array([0., 0.]), orientation=0.)
        self.assertEqual(lanelet_network.find_most_likely_lanelet_by_state([state]), [1])

        state = InitialState(position=np.array([0., 0.]), orientation=np.deg2rad(45))
        self.assertEqual(lanelet_network.find_most_likely_lanelet_by_state([state]), [2])

    def test_filter_obstacles_in_network_positive_map_obstacles_to_lanelet_postive(self):
        initial_state = InitialState(**{'position': np.array([0, 0]), 'orientation': 0.0})
        rect_shape = Rectangle(2, 2)
        expected_obstacle = StaticObstacle(obstacle_id=1, obstacle_type=ObstacleType.CAR, obstacle_shape=rect_shape,
                                           initial_state=initial_state)

        self.lanelet_network.map_obstacles_to_lanelets([expected_obstacle])
        actual_obstacles = self.lanelet_network.filter_obstacles_in_network([expected_obstacle])

        self.assertEqual(len(actual_obstacles), 1)
        self.assertEqual(actual_obstacles[0].obstacle_id, 1)
        self.assertEqual(actual_obstacles[0].obstacle_type, ObstacleType.CAR)

    def test_filter_obstacles_in_network_positive_map_obstacles_to_lanelet_negative(self):
        initial_state = InitialState(**{'position': np.array([-50, -50]), 'orientation': 0.0})
        rect_shape = Rectangle(2, 2)
        expected_obstacle = StaticObstacle(obstacle_id=1, obstacle_type=ObstacleType.CAR, obstacle_shape=rect_shape,
                                           initial_state=initial_state)

        self.lanelet_network.map_obstacles_to_lanelets([expected_obstacle])
        actual_obstacles = self.lanelet_network.filter_obstacles_in_network([expected_obstacle])

        self.assertEqual(len(actual_obstacles), 0)

    def test_lanelets_in_proximity(self):
        right_vertices = np.array([[0, 0], [1, 0], [2, 0], [3, .5], [4, 1], [5, 1], [6, 1], [7, 0], [8, 0]])
        left_vertices = np.array([[0, 1], [1, 1], [2, 1], [3, 1.5], [4, 2], [5, 2], [6, 2], [7, 1], [8, 1]])
        center_vertices = np.array([[0, .5], [1, .5], [2, .5], [3, 1], [4, 1.5], [5, 1.5], [6, 1.5], [7, .5], [8, .5]])
        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, 1)
        lanelet_network = LaneletNetwork()
        lanelet_network.add_lanelet(lanelet)

        radius = 5.0
        point = np.array([0, 0])

        actual_in_proximity = lanelet_network.lanelets_in_proximity(point, radius)

        self.assertTrue(len(actual_in_proximity), 1)

    def test_remove_lanelet(self):
        self.lanelet_network.remove_lanelet(123456789)  # delete non-existing lanelet
        self.assertEqual(len(self.lanelet_network.lanelets), 2)

        self.lanelet_network.remove_lanelet(self.lanelet.lanelet_id)  # delete existing lanelet
        self.assertEqual(len(self.lanelet_network.lanelets), 1)

    def test_remove_traffic_sign(self):
        self.lanelet_network.remove_traffic_sign(123456789)  # delete non-existing traffic sign
        self.assertEqual(len(self.lanelet_network.traffic_signs), 1)

        self.lanelet_network.remove_traffic_sign(self.traffic_sign.traffic_sign_id)  # delete existing traffic sign
        self.assertEqual(len(self.lanelet_network.traffic_signs), 0)

    def test_remove_traffic_light(self):
        self.lanelet_network.remove_traffic_light(123456789)  # delete non-existing traffic light
        self.assertEqual(len(self.lanelet_network.traffic_lights), 1)

        self.lanelet_network.remove_traffic_light(self.traffic_light.traffic_light_id)  # delete existing traffic light
        self.assertEqual(len(self.lanelet_network.traffic_lights), 0)

    def test_remove_intersection(self):
        self.lanelet_network.remove_intersection(123456789)  # delete non-existing intersection
        self.assertEqual(len(self.lanelet_network.intersections), 1)

        self.lanelet_network.remove_intersection(self.intersection.intersection_id)  # delete existing traffic light
        self.assertEqual(len(self.lanelet_network.intersections), 0)

    def test_cleanup_lanelet_references(self):
        # intersection contains dummy references which will be deleted during cleanup
        self.assertEqual(len(self.intersection.incomings[0].incoming_lanelets), 2)
        self.lanelet_network.remove_lanelet(self.lanelet.lanelet_id)  # delete existing traffic sign
        self.assertEqual(len(self.lanelet_2.predecessor), 0)
        self.assertEqual(len(self.intersection.incomings[0].incoming_lanelets), 0)

    def test_cleanup_traffic_light_references(self):
        self.lanelet_network.remove_traffic_light(self.traffic_light.traffic_light_id)  # delete existing traffic light
        self.assertEqual(len(self.lanelet.traffic_lights), 0)
        self.assertEqual(len(self.lanelet.stop_line.traffic_light_ref), 0)

    def test_cleanup_traffic_sign_references(self):
        self.lanelet_network.remove_traffic_sign(self.traffic_sign.traffic_sign_id)  # delete existing traffic sign
        self.assertEqual(len(self.lanelet.traffic_signs), 0)
        self.assertEqual(len(self.lanelet.stop_line.traffic_sign_ref), 0)

    def test_equality_hash(self):
        left_vertices = np.array([[0, 1], [1, 1], [2, 1]])
        center_vertices = np.array([[0, 0], [1, 0], [2, 0]])
        right_vertices = np.array([[0, -1], [1, -1], [2, -1]])
        lanelet_id = 3
        lanelet = Lanelet(left_vertices, center_vertices, right_vertices, lanelet_id)

        incoming = IntersectionIncomingElement(2, {9, 11}, {12, 13}, {14, 15}, {16, 17}, 18)
        intersection = Intersection(1, [incoming], {30, 31})

        traffic_sign_max_speed = TrafficSignElement(TrafficSignIDGermany.MAX_SPEED, ["15"])
        traffic_sign = TrafficSign(1, [traffic_sign_max_speed], {3}, np.array([10.0, 7.0]))

        cycle = [TrafficLightCycleElement(TrafficLightState.GREEN, 2)]
        traffic_light = TrafficLight(234, cycle, np.array([10., 10.]), 5)

        lanelet_network_1 = LaneletNetwork()
        lanelet_network_2 = LaneletNetwork()
        for lanelet_network in [lanelet_network_1, lanelet_network_2]:
            lanelet_network.add_lanelet(lanelet)
            lanelet_network.add_intersection(intersection)
            lanelet_network.add_traffic_sign(traffic_sign, {3})
            lanelet_network.add_traffic_light(traffic_light, {3})
        self.assertTrue(lanelet_network_1 == lanelet_network_2)
        self.assertTrue(hash(lanelet_network_1), hash(lanelet_network_2))

        lanelet_network_2.remove_lanelet(3)
        self.assertFalse(lanelet_network_1 == lanelet_network_2)
        self.assertNotEqual(hash(lanelet_network_1), hash(lanelet_network_2))

        lanelet_network_2.add_lanelet(lanelet)
        lanelet_network_2.add_intersection(Intersection(2, [incoming], {30, 31}))
        self.assertFalse(lanelet_network_1 == lanelet_network_2)
        self.assertNotEqual(hash(lanelet_network_1), hash(lanelet_network_2))

        lanelet_network_2.remove_intersection(2)
        lanelet_network_2.add_traffic_sign(TrafficSign(4, [traffic_sign_max_speed], {3}, np.array([10.0, 7.0])), {3})
        self.assertFalse(lanelet_network_1 == lanelet_network_2)
        self.assertNotEqual(hash(lanelet_network_1), hash(lanelet_network_2))

        lanelet_network_2.remove_traffic_sign(4)
        lanelet_network_2.remove_traffic_light(234)
        self.assertFalse(lanelet_network_1 == lanelet_network_2)
        self.assertNotEqual(hash(lanelet_network_1), hash(lanelet_network_2))


if __name__ == '__main__':
    unittest.main()
