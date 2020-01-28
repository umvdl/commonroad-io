"""
File Writer for scenarios to commonroad xml-format
"""
import datetime
import enum
import pathlib
import io
import os
from typing import Union, List
import numpy as np
import decimal
import warnings
from lxml import etree, objectify

from commonroad.common.util import Interval
from commonroad.geometry.shape import Rectangle, Circle, Polygon, ShapeGroup
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.prediction.prediction import SetBasedPrediction, TrajectoryPrediction
from commonroad.scenario.intersection import Intersection
from commonroad.scenario.lanelet import Lanelet, LineMarking, StopLine
from commonroad.scenario.obstacle import (
    ObstacleRole,
    ObstacleType,
    DynamicObstacle,
    StaticObstacle,
    Obstacle,
    Occupancy,
    Shape,
)
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_sign import TrafficSign, TrafficLight, TrafficLightCycleElement
from commonroad.scenario.trajectory import Trajectory, State

__author__ = "Stefanie Manzinger, Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "2019.1"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad@in.tum.de"
__status__ = "Released"


SCENARIO_VERSION = '2020a'

# create a new context for this task
ctx = decimal.Context()


def float_to_str(f):
    """
    Convert the given float to a string,
    without resorting to scientific notation
    """
    d1 = ctx.create_decimal(repr(f))
    return '{}'.format(d1)


def create_exact_node_float(value: Union[int, float]) -> etree.Element:
    """
    creates element node for exact value
    :param value: exact value
    :return: node for exact value
    """
    node = etree.Element('exact')
    node.text = float_to_str(np.float64(value))
    return node


def create_exact_node_int(value: Union[int]) -> etree.Element:
    """
    creates element node for exact value
    :param value: exact value
    :return: node for exact value
    """
    assert np.issubdtype(type(value), np.integer), (
        '<util/create_exact_node_int> expected type int for value but'
        ' got %s' % (type(value))
    )
    node = etree.Element('exact')
    node.text = str(value)
    return node


def create_interval_node_float(interval: Interval) -> List[etree.Element]:
    """
    creates ElementTree.Element for an interval
    :param interval:
    :return: list of Element nodes with start and end of interval
    """
    node_lo = etree.Element('intervalStart')
    node_lo.text = float_to_str(np.float64(interval.start))
    node_hi = etree.Element('intervalEnd')
    node_hi.text = float_to_str(np.float64(interval.end))
    return [node_lo, node_hi]


def create_interval_node_int(interval: Interval) -> List[etree.Element]:
    """
    creates ElementTree.Element for an interval
    :param interval:
    :return: list of Element nodes with start and end of interval
    """
    assert np.issubdtype(type(interval.start), np.integer), (
        '<util/create_exact_node_int> expected type int for value but got %s'
        % (type(interval.start))
    )
    assert np.issubdtype(type(interval.end), np.integer), (
        '<util/create_exact_node_int> expected type int for value but got %s'
        % (type(interval.start))
    )

    node_lo = etree.Element('intervalStart')
    node_lo.text = str(interval.start)
    node_hi = etree.Element('intervalEnd')
    node_hi.text = str(interval.end)
    return [node_lo, node_hi]


class OverwriteExistingFile(enum.Enum):
    """
    Specifies whether an existing file will be overwritten or skipped
    """

    ASK_USER_INPUT = 0
    ALWAYS = 1
    SKIP = 2


class CommonRoadFileWriter:
    def __init__(
        self,
        scenario: Scenario,
        planning_problem_set: PlanningProblemSet,
        author: str = None,
        affiliation: str = None,
        source: str = None,
        tags: str = None,
        decimal_precision: int = 8,
    ):
        """
        Initialize the FileWriter with a scenario and tags for the xml-header

        :param scenario: scenario that should be written later
        :param planning_problem_set: corresponding planning problem to the scenario
        :param author: author's name
        :param affiliation: affiliation of the author
        :param source: source of dataset (d.h. database, handcrafted, etc.)
        :param tags: keywords describing the scenario (e.g. road type(one-lane road, multilane),
                required maneuver etc., see commonroad.in.tum.de for full list))
        :param decimal_precision: number of decimal places used when writing float values
        """
        assert not (author is None and scenario.author is None)
        assert not (affiliation is None and scenario.affiliation is None)
        assert not (source is None and scenario.source is None)
        assert not (tags is None and scenario.tags is None)

        self.scenario = scenario
        self.planning_problem_set = planning_problem_set
        self._root_node = etree.Element('commonRoad')
        self.author = author if author is not None else scenario.author
        self.affiliation = affiliation if affiliation is not None else scenario.affiliation
        self.source = source if source is not None else scenario.source
        self.tags = tags if tags is not None else scenario.tags

        # set decimal precision
        ctx.prec = decimal_precision

    @property
    def root_node(self):
        return self._root_node

    @root_node.setter
    def root_node(self, root_node):
        warnings.warn(
            '<CommonRoadFileWriter/root_node> root_node of CommonRoadFileWriter is immutable'
        )

    @property
    def author(self):
        return self._author

    @author.setter
    def author(self, author):
        assert isinstance(
            author, str
        ), '<CommonRoadFileWriter/author> author must be a string, but has type {}'.format(
            type(author)
        )
        self._author = author

    @property
    def affiliation(self):
        return self._affiliation

    @affiliation.setter
    def affiliation(self, affiliation):
        assert isinstance(
            affiliation, str
        ), '<CommonRoadFileWriter/affiliation> affiliation must be a string, but has type {}'.format(
            type(affiliation)
        )
        self._affiliation = affiliation

    @property
    def source(self):
        return self._source

    @source.setter
    def source(self, source):
        assert isinstance(
            source, str
        ), '<CommonRoadFileWriter/source> source must be a string, but has type {}'.format(
            type(source)
        )
        self._source = source

    @property
    def tags(self):
        return self._tags

    @tags.setter
    def tags(self, tags):
        assert isinstance(
            tags, str
        ), '<CommonRoadFileWriter/tags> tags must be a string, but has type {}'.format(
            type(tags)
        )
        self._tags = tags

    def _write_header(self):
        self._root_node.set('timeStepSize', str(self.scenario.dt))
        self._root_node.set('commonRoadVersion', SCENARIO_VERSION)
        self._root_node.set('author', self.author)
        self._root_node.set('affiliation', self.affiliation)
        self._root_node.set('source', self.source)
        self._root_node.set('tags', self.tags)

        try:
            if self.scenario.benchmark_id:
                self._root_node.set('benchmarkID', self.scenario.benchmark_id)
        except:
            self._root_node.set('benchmarkID', '-1')
            print('Warning: No benchmark id set.')

        self._root_node.set('date', datetime.datetime.today().strftime('%Y-%m-%d'))

    def _add_all_objects_from_scenario(self):
        for l in self.scenario.lanelet_network.lanelets:
            self._root_node.append(LaneletXMLNode.create_node(l))
        for sign in self.scenario.lanelet_network.traffic_signs:
            self._root_node.append(TrafficSignXMLNode.create_node(sign))
        for light in self.scenario.lanelet_network.traffic_lights:
            self._root_node.append(TrafficLightXMLNode.create_node(light))
        for intersection in self.scenario.lanelet_network.intersections:
            self._root_node.append(IntersectionXMLNode.create_node(intersection))
        for o in self.scenario.obstacles:
            self._root_node.append(ObstacleXMLNode.create_node(o))

    def _add_all_planning_problems_from_planning_problem_set(self):
        for (
            planning_problem
        ) in self.planning_problem_set.planning_problem_dict.values():
            self._root_node.append(PlanningProblemXMLNode.create_node(planning_problem))

    def _dump(self):
        rough_string = etree.tostring(
            self._root_node, pretty_print=True, encoding='unicode'
        )
        return rough_string
        # reparsed = minidom.parseString(rough_string)
        # return reparsed.toprettyxml(indent='  ')

    def write_to_file(
        self,
        filename: Union[str, None] = None,
        overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT,
    ):
        """
        Write a scenario including planning-problem. If file already exists, it will be overwritten of skipped

        :param filename: filename of the xml output file. If 'None', the Benchmark ID is taken
        :param overwrite_existing_file: Specify whether an already existing file should be overwritten or skipped
        :return:
        """
        if filename is None:
            filename = self.scenario.benchmark_id

        if pathlib.Path(filename).is_file():
            if overwrite_existing_file is OverwriteExistingFile.ASK_USER_INPUT:
                overwrite = input(
                    'File {} already exists, replace old file (or else skip)? (y/n)'.format(
                        filename
                    )
                )
            elif overwrite_existing_file is OverwriteExistingFile.SKIP:
                overwrite = 'n'
            else:
                overwrite = 'y'

            if overwrite is 'n':
                print('Writing of file {} skipped'.format(filename))
                return
            else:
                print('Replace file {}'.format(filename))

        file = open(filename, 'w')
        self._write_header()
        self._add_all_objects_from_scenario()
        self._add_all_planning_problems_from_planning_problem_set()
        file.write(self._dump())
        file.close()

    def write_scenario_to_file(
        self,
        filename: Union[str, None] = None,
        overwrite_existing_file: OverwriteExistingFile = OverwriteExistingFile.ASK_USER_INPUT,
    ):
        """
        Write a scenario without planning-problem. If file already exists, it will be overwritten of skipped.

        :param filename: filename of the xml output file. If 'None', the Benchmark ID is taken
        :param OverwriteExistingFile: Specify whether an already existing file should be overwritten or skipped
        :return: None
        """
        if filename is None:
            filename = self.scenario.benchmark_id

        if pathlib.Path(filename).is_file():
            if overwrite_existing_file is OverwriteExistingFile.ASK_USER_INPUT:
                overwrite = input(
                    'File {} already exists, replace old file (or else skip)? (y/n)'.format(
                        filename
                    )
                )
            elif overwrite_existing_file is OverwriteExistingFile.SKIP:
                overwrite = 'n'
            else:
                overwrite = 'y'

            if overwrite is 'n':
                print(
                    'Writing skipped for file, since it already exists {}'.format(
                        filename
                    )
                )
                return
            else:
                print('Replace file {}'.format(filename))

        with open(filename, 'w') as file_out:
            self._write_header()
            self._add_all_objects_from_scenario()
            file_out.write(self._dump())

    def write_scenario_to_file_io(self, file_io: io.IOBase):
        """Write a scenario without planning-problem to file_io.

        Args:
          file_io: File to write to.

        """
        self._write_header()
        self._add_all_objects_from_scenario()
        self._write_xml_output_to_file(file_io)

    def write_to_file_io(self, file_io: io.IOBase):
        """Write a scenario including planning-problem to file_io.

        Args:
          file_io: File to write to.

        """
        self._write_header()
        self._add_all_objects_from_scenario()
        self._add_all_planning_problems_from_planning_problem_set()
        self._write_xml_output_to_file(file_io)

    def _write_xml_output_to_file(self, file_io: io.IOBase):
        """Write the dump from self._dump() to file_io.

        Args:
          file_io: File to write to.

        """
        output_str = self._dump()
        CommonRoadFileWriter.check_validity_of_commonroad_file(output_str)
        file_io.write(output_str)

    @staticmethod
    def check_validity_of_commonroad_file(commonroad_str: str):
        """Check the validity of a generated xml_string in terms of
        commonroad with an existing XSD schema.
        Throw an error if it is not valid.

        Args:
          commonroad_str: XML formatted string which should be checked.

        """
        with open(
            os.path.dirname(os.path.abspath(__file__)) + '/commonroad_validity.xsd',
            'rb',
        ) as schema_file:
            schema = etree.XMLSchema(etree.parse(schema_file))

        parser = objectify.makeparser(schema=schema, encoding='utf-8')

        try:
            etree.fromstring(commonroad_str, parser)
        except etree.XMLSyntaxError as error:
            raise Exception(
                'Could not produce valid CommonRoad file! Error: {}'.format(error.msg)
            )


class LaneletXMLNode:
    @classmethod
    def create_node(cls, lanelet: Lanelet) -> etree.Element:
        """
        Create XML-Node for a Lanelet
        :param lanelet: lanelet for creating a node
        :return: node
        """
        lanelet_node = etree.Element('lanelet')
        lanelet_node.set('id', str(lanelet.lanelet_id))

        # left boundary
        left_boundary = etree.Element('leftBound')
        Pointlist.create_from_numpy_array(lanelet.left_vertices).add_points_to_node(
            left_boundary
        )

        if hasattr(lanelet, 'line_marking_left_vertices') and isinstance(
            lanelet.line_marking_left_vertices, LineMarking
        ):
            line_marking_left = etree.Element('lineMarking')
            if lanelet.line_marking_left_vertices is LineMarking.DASHED:
                line_marking_left.text = 'dashed'
            elif lanelet.line_marking_left_vertices is LineMarking.SOLID:
                line_marking_left.text = 'solid'
            left_boundary.append(line_marking_left)

        lanelet_node.append(left_boundary)

        # right boundary
        right_boundary = etree.Element('rightBound')
        Pointlist.create_from_numpy_array(lanelet.right_vertices).add_points_to_node(
            right_boundary
        )

        if hasattr(lanelet, 'line_marking_right_vertices') and isinstance(
            lanelet.line_marking_right_vertices, LineMarking
        ):
            line_marking_right = etree.Element('lineMarking')
            if lanelet.line_marking_right_vertices is LineMarking.DASHED:
                line_marking_right.text = 'dashed'
            elif lanelet.line_marking_right_vertices is LineMarking.SOLID:
                line_marking_right.text = 'solid'
            right_boundary.append(line_marking_right)

        lanelet_node.append(right_boundary)

        for l in lanelet.predecessor:
            predecessor = etree.Element('predecessor')
            predecessor.set('ref', str(l))
            lanelet_node.append(predecessor)

        for l in lanelet.successor:
            successor = etree.Element('successor')
            successor.set('ref', str(l))
            lanelet_node.append(successor)

        if lanelet.adj_left:
            adjacent_left = etree.Element('adjacentLeft')
            adjacent_left.set('ref', str(lanelet.adj_left))
            if lanelet.adj_left_same_direction:
                adjacent_left.set('drivingDir', 'same')
            else:
                adjacent_left.set('drivingDir', 'opposite')
            lanelet_node.append(adjacent_left)

        if lanelet.adj_right:
            adjacent_right = etree.Element('adjacentRight')
            adjacent_right.set('ref', str(lanelet.adj_right))
            if lanelet.adj_right_same_direction:
                adjacent_right.set('drivingDir', 'same')
            else:
                adjacent_right.set('drivingDir', 'opposite')
            lanelet_node.append(adjacent_right)

        if lanelet.stop_line:
            stop_line_node = LaneletStopLineXMLNode.create_node(lanelet.stop_line)
            lanelet_node.append(stop_line_node)

        if lanelet.lanelet_type:
            for lanelet_type_element in lanelet.lanelet_type:
                lanelet_type_node = etree.Element('type')
                lanelet_type_node.text = str(lanelet_type_element.value)
                lanelet_node.append(lanelet_type_node)

        if lanelet.user_one_way:
            for user_one_way in lanelet.user_one_way:
                user_one_way_node = etree.Element('userOneWay')
                user_one_way_node.text = str(user_one_way.value)
                lanelet_node.append(user_one_way_node)

        if lanelet.user_bidirectional:
            for user in lanelet.user_bidirectional:
                user_node = etree.Element('userBidirectional')
                user_node.text = str(user.value)
                lanelet_node.append(user_node)

        if lanelet.traffic_signs:
            for traffic_sign in lanelet.traffic_signs:
                traffic_sign_node = TrafficSignXMLNode.create_ref_node(traffic_sign)
                lanelet_node.append(traffic_sign_node)

        if lanelet.traffic_lights:
            for traffic_light in lanelet.traffic_lights:
                traffic_light_node = TrafficLightXMLNode.create_ref_node(traffic_light)
                lanelet_node.append(traffic_light_node)

        return lanelet_node


class ObstacleXMLNode:
    @classmethod
    def create_node(cls, obstacle: Obstacle) -> etree.Element:
        """
        Create XML-Node for an Obstacle
        :param obstacle: Obstacle for creating a node
        :return:
        """
        if type(obstacle) == DynamicObstacle:
            return DynamicObstacleXMLNode.create_node(obstacle)
        elif type(obstacle) == StaticObstacle:
            return StaticObstacleXMLNode.create_node(obstacle)
        else:
            raise Exception()

    @classmethod
    def _obstacle_type_enum_to_string(cls, obstacle_type: ObstacleType):
        """
        Create string for obstacle type
        """
        if obstacle_type == ObstacleType.CAR:
            return 'car'
        elif obstacle_type == ObstacleType.UNKNOWN:
            return 'unknown'
        elif obstacle_type == ObstacleType.BICYCLE:
            return 'bicycle'
        elif obstacle_type == ObstacleType.PEDESTRIAN:
            return 'pedestrian'
        elif obstacle_type == ObstacleType.PARKED_VEHICLE:
            return 'parkedVehicle'
        elif obstacle_type == ObstacleType.ROAD_BOUNDARY:
            return 'roadBoundary'
        elif obstacle_type == ObstacleType.TRUCK:
            return 'truck'
        elif obstacle_type == ObstacleType.BUS:
            return 'bus'
        elif obstacle_type == ObstacleType.PRIORITY_VEHICLE:
            return 'priorityVehicle'
        elif obstacle_type == ObstacleType.CONSTRUCTION_ZONE:
            return 'constructionZone'
        elif obstacle_type == ObstacleType.TRAIN:
            return 'train'

    @classmethod
    def _obstacle_role_enum_to_string(cls, obstacle_role: ObstacleRole):
        """
        Create string for obstalce role
        """
        if obstacle_role == ObstacleRole.STATIC:
            return 'static'
        elif obstacle_role == ObstacleRole.DYNAMIC:
            return 'dynamic'

    @classmethod
    def create_obstacle_node_header(
        cls, obstacle_id: int, obstacle_role: ObstacleRole, obstacle_type: ObstacleType
    ):
        obstacle_node = etree.Element(cls._obstacle_role_enum_to_string(obstacle_role)+'Obstacle')
        obstacle_node.set('id', str(obstacle_id))
        # role_node = etree.Element('role')
        # role_node.text = cls._obstacle_role_enum_to_string(obstacle_role)
        # obstacle_node.append(role_node)
        type_node = etree.Element('type')
        type_node.text = cls._obstacle_type_enum_to_string(obstacle_type)
        obstacle_node.append(type_node)
        return obstacle_node


class StaticObstacleXMLNode:
    @classmethod
    def create_node(cls, static_obstacle: StaticObstacle) -> etree.Element:
        """
        Create XML-Node for a StaticObstacle
        :param static_obstacle: static_obstacle for creating a node
        :return: node
        """
        node = ObstacleXMLNode.create_obstacle_node_header(
            static_obstacle.obstacle_id,
            static_obstacle.obstacle_role,
            static_obstacle.obstacle_type,
        )
        shape_node = etree.Element('shape')
        shape_node.extend(ShapeXMLNode.create_node(static_obstacle.obstacle_shape))
        node.append(shape_node)

        # write intial state
        initial_state_node = etree.Element('initialState')
        StateXMLNode.create_state_node(
            static_obstacle.initial_state,
            initial_state_node,
            time_step=static_obstacle.initial_state.time_step,
        )
        node.append(initial_state_node)

        return node


class DynamicObstacleXMLNode:
    @classmethod
    def create_node(cls, dynamic_obstacle: DynamicObstacle) -> etree.Element:
        """
        Create XML-Node for a DynamicObstacle
        :param dynamic_obstacle: dynamic_obstacle for creating a node
        :return: node
        """
        obstacle_node = ObstacleXMLNode.create_obstacle_node_header(
            dynamic_obstacle.obstacle_id,
            dynamic_obstacle.obstacle_role,
            dynamic_obstacle.obstacle_type,
        )
        shape_node = etree.Element('shape')
        shape_node.extend(
            ShapeXMLNode.create_node(
                dynamic_obstacle.obstacle_shape, dynamic_obstacle_shape=True
            )
        )
        obstacle_node.append(shape_node)

        # write intial state
        initial_state_node = etree.Element('initialState')
        StateXMLNode.create_state_node(
            dynamic_obstacle.initial_state,
            initial_state_node,
            time_step=dynamic_obstacle.initial_state.time_step,
        )
        obstacle_node.append(initial_state_node)

        # write prediction depending on type
        if isinstance(dynamic_obstacle.prediction, SetBasedPrediction):
            obstacle_node.append(
                cls._create_occupancy_node(dynamic_obstacle.prediction.occupancy_set)
            )
        elif isinstance(dynamic_obstacle.prediction, TrajectoryPrediction):
            obstacle_node.append(
                cls._create_trajectory_node(dynamic_obstacle.prediction.trajectory)
            )

        return obstacle_node

    @classmethod
    def _create_trajectory_node(cls, trajectory: Trajectory) -> etree.Element:
        """
        Create XML-Node for a Trajectory
        :param trajectory: trajectory for creating a node
        :return: node
        """
        traj_node = etree.Element('trajectory')
        for state in trajectory.state_list:
            state_node = etree.Element('state')
            traj_node.append(
                StateXMLNode.create_state_node(state, state_node, state.time_step)
            )
        return traj_node

    @classmethod
    def _create_occupancy_node(cls, occupancy_set: List[Occupancy]) -> etree.Element:
        """
        Create XML-Node for an occupancy_set
        :param occupancy_set: occupancy_set for creating a node
        :return: node
        """
        occupancy_set_node = etree.Element('occupancySet')
        for occupancy in occupancy_set:
            occupancy_set_node.append(OccupancyXMLNode.create_node(occupancy))
        return occupancy_set_node


class OccupancyXMLNode:
    @classmethod
    def create_node(cls, occupancy: Occupancy) -> etree.Element:
        """
        Create XML-Node for an Occupancy
        :param occupancy: occupancy for creating a node
        :return: node
        """
        occupancy_node = etree.Element('occupancy')

        shape_node = etree.Element('shape')
        shape_node.extend(ShapeXMLNode.create_node(occupancy.shape))
        occupancy_node.append(shape_node)

        time_node = etree.Element('time')
        time = occupancy.time_step
        if isinstance(occupancy.time_step, Interval):
            time_node.extend(create_interval_node_int(time))
        else:
            time_node.append(create_exact_node_int(time))
        occupancy_node.append(time_node)

        return occupancy_node


class ShapeXMLNode:
    @classmethod
    def create_node(cls, shape, dynamic_obstacle_shape=False) -> List[etree.Element]:
        """
        Create XML-Node for a shape
        :param shape: shape for creating a node
        :param dynamic_obstacle_shape: specify whether the shape belongs to an dynamic obstacle or not
        :return: node
        """
        if type(shape) == ShapeGroup:
            shape_node_list = []
            for s in shape.shapes:
                shape_node_list.append(
                    cls._create_single_element(s, dynamic_obstacle_shape)
                )
        else:
            shape_node = cls._create_single_element(shape, dynamic_obstacle_shape)
            shape_node_list = [shape_node]
        return shape_node_list

    @classmethod
    def _create_single_element(
        cls, shape: Shape, dynamic_obstacle_shape: bool
    ) -> etree.Element:
        """
        Create XML-Node for a single shape element
        :param shape: shape for creating a node
        :param dynamic_obstacle_shape: specify whether the shape belongs to an dynamic obstacle or not
        :return: node
        """
        if type(shape) == Rectangle:
            node = RectangleXMLNode.create_rectangle_node(shape, dynamic_obstacle_shape)
        elif type(shape) == Circle:
            node = CircleXMLNode.create_circle_node(shape, dynamic_obstacle_shape)
        elif type(shape) == Polygon:
            node = PolygonXMLNode.create_polygon_node(shape, dynamic_obstacle_shape)
        else:
            raise TypeError(
                '<ShapeXMLNode/_create_single_element> Expected type Polygon, Circle or Rectangle but got %s'
                % (type(shape))
            )
        return node


class RectangleXMLNode:
    @classmethod
    def create_rectangle_node(
        cls, rectangle: Rectangle, dynamic_obstacle_shape=False
    ) -> etree.Element:
        """
        Create XML-Node for a rectangle
        :param rectangle: rectangle for creating a node
        :param dynamic_obstacle_shape: specify whether the shape belongs to an dynamic obstacle or not
        :return: node
        """
        rectangle_node = etree.Element('rectangle')
        length_node = etree.Element('length')
        length_node.text = str(rectangle.length)
        rectangle_node.append(length_node)

        width_node = etree.Element('width')
        width_node.text = str(rectangle.width)
        rectangle_node.append(width_node)

        if not dynamic_obstacle_shape:
            orientation_node = etree.Element('orientation')
            orientation_node.text = str(np.float64(rectangle.orientation))
            rectangle_node.append(orientation_node)

            center_node = etree.Element('center')
            x_node = etree.Element('x')
            x_node.text = str(np.float64(rectangle.center[0]))
            center_node.append(x_node)
            y_node = etree.Element('y')
            y_node.text = str(np.float64(rectangle.center[1]))
            center_node.append(y_node)
            rectangle_node.append(center_node)
        return rectangle_node


class CircleXMLNode:
    @classmethod
    def create_circle_node(
        cls, circle: Circle, dynamic_obstacle_shape=False
    ) -> etree.Element:
        """
        Create XML-Node for a circle
        :param circle: circle for creating a node
        :param dynamic_obstacle_shape: specify whether the shape belongs to an dynamic obstacle or not
        :return: node
        """
        circle_node = etree.Element('circle')

        radius_node = etree.Element('radius')
        radius_node.text = str(np.float64(circle.radius))
        circle_node.append(radius_node)

        if not dynamic_obstacle_shape:
            center_node = etree.Element('center')
            x_node = etree.Element('x')
            x_node.text = str(np.float64(circle.center[0]))
            center_node.append(x_node)
            y_node = etree.Element('y')
            y_node.text = str(np.float64(circle.center[1]))
            center_node.append(y_node)
            circle_node.append(center_node)
        return circle_node


class PolygonXMLNode:
    @classmethod
    def create_polygon_node(
        cls, polygon: Polygon, dynamic_obstacle_shape: bool = False
    ) -> etree.Element:
        """
        Create XML-Node for a polygon
        :param polygon: polygon for creating a node
        :param dynamic_obstacle_shape: specify whether the shape belongs to an dynamic obstacle or not
        :return: node
        """
        polygon_node = etree.Element('polygon')
        for p in polygon.vertices:
            polygon_node.append(Point(p[0], p[1]).create_node())
        return polygon_node


class StateXMLNode:
    @classmethod
    def create_goal_state_node(
        cls, state: State, goal_lanelet_ids: List[int]
    ) -> etree.Element:
        """
        Create XML-Node for a polygon
        :param polygon: polygon for creating a node
        :param goal_lanelet_ids: contains a list of lanelet ids if a goal state's position is specified lanelet id(s)
        :return: node
        """
        state_node = etree.Element('goalState')
        if hasattr(state, 'position') or len(goal_lanelet_ids) > 0:
            position = etree.Element('position')
            position = cls._write_goal_position(position, state.position, goal_lanelet_ids)
            state_node.append(position)

        if hasattr(state, 'orientation'):
            orientation = etree.Element('orientation')
            orientation = cls._write_value_exact_or_interval(
                orientation, state.orientation
            )
            state_node.append(orientation)
        if hasattr(state, 'time_step'):
            time = etree.Element('time')
            time = cls._write_goal_time_exact_or_interval(time, state.time_step)
            state_node.append(time)
        if hasattr(state, 'velocity'):
            velocity = etree.Element('velocity')
            velocity = cls._write_value_exact_or_interval(velocity, state.velocity)
            state_node.append(velocity)
        if hasattr(state, 'acceleration'):
            acceleration = etree.Element('acceleration')
            acceleration = cls._write_value_exact_or_interval(
                acceleration, state.acceleration
            )
            state_node.append(acceleration)
        if hasattr(state, 'yaw_rate'):
            yaw_rate = etree.Element('yawRate')
            yaw_rate = cls._write_value_exact_or_interval(yaw_rate, state.yaw_rate)
            state_node.append(yaw_rate)
        if hasattr(state, 'slip_angle'):
            slip_angle = etree.Element('slipAngle')
            slip_angle = cls._write_value_exact_or_interval(
                slip_angle, state.slip_angle
            )
            state_node.append(slip_angle)
        return state_node

    @classmethod
    def _write_goal_position(
        cls, node: etree.Element, position: Union[Shape, int, list], goal_lanelet_ids: List[int],
    ) -> etree.Element:
        """
        Create XML-Node for a goal position
        :param node: node of the GoalState
        :param position: either (list of) shape elements or lanelet ids specifying the goal position
        :return: node
        """
        if len(goal_lanelet_ids) > 0:
            for id in goal_lanelet_ids:
                lanelet = etree.Element('lanelet')
                lanelet.set('ref', str(id))
                node.append(lanelet)
        elif isinstance(position, int):
            lanelet = etree.Element('lanelet')
            lanelet.set('ref', str(position))
            node.append(lanelet)
        elif(
            isinstance(position, Rectangle)
            or isinstance(position, Circle)
            or isinstance(position, Polygon)
            ):
            node.extend(ShapeXMLNode.create_node(position))
        elif isinstance(position, ShapeGroup):
            node.extend(ShapeXMLNode.create_node(position))
        elif type(position) is list:
            raise ValueError('A goal state cannot contain multiple items. Use a list of goal states instead.')
        else:
            raise ValueError('Case should not occur, position={}, goal_lanelet_ids={}.'.format(position,goal_lanelet_ids))
        return node

    @classmethod
    def _write_goal_time_exact_or_interval(
        cls, node: etree.Element, time_step: Union[Interval, float, int]
    ) -> etree.Element:
        """
        Create XML-Node for a goal time
        :param node: node of the GoalState
        :param time_step: contains time interval or time_step of goal time
        :return: node
        """
        if isinstance(time_step, int):
            node.append(create_exact_node_int(time_step))
        elif isinstance(time_step, Interval):
            node.extend(
                create_interval_node_int(Interval(time_step.start, time_step.end))
            )
        else:
            raise Exception()
        return node

    @classmethod
    def _write_value_exact_or_interval(
        cls, node: etree.Element, var: Union[Interval, float, int]
    ):
        """
        Create XML-Node for a goal value
        :param node: node of the GoalState
        :param var: contains interval or exact_value of goal state value
        :return: node
        """
        if isinstance(var, (float, int)):
            node.append(create_exact_node_float(var))
        elif isinstance(var, Interval):
            node.extend(create_interval_node_float(var))
        else:
            raise Exception()
        return node

    @classmethod
    def create_state_node(
        cls, state: State, state_node: etree.Element, time_step: int
    ) -> etree.Element:
        """
        Create XML-Node for a state
        :param state: value of the state
        :param state_node: node of the overlying state
        :return: node
        """

        if hasattr(state, 'position'):
            position = etree.Element('position')
            if type(state.position) in [np.ndarray, list]:
                position.append(
                    Point.create_from_numpy_array(state.position).create_node()
                )
                state_node.append(position)
            elif isinstance(state.position, Shape):
                position.extend(ShapeXMLNode.create_node(state.position))
                state_node.append(position)
            else:
                raise Exception()
        if hasattr(state, 'orientation'):
            orientation = etree.Element('orientation')
            orientation = cls._write_value_exact_or_interval(
                orientation, state.orientation
            )
            state_node.append(orientation)

        time_node = etree.Element('time')
        time_node.append(create_exact_node_int(time_step))
        state_node.append(time_node)

        if hasattr(state, 'velocity'):
            velocity = etree.Element('velocity')
            velocity = cls._write_value_exact_or_interval(velocity, state.velocity)
            state_node.append(velocity)

        if hasattr(state, 'acceleration'):
            acceleration = etree.Element('acceleration')
            acceleration = cls._write_value_exact_or_interval(
                acceleration, state.acceleration
            )
            state_node.append(acceleration)
        if hasattr(state, 'yaw_rate'):
            yaw_rate = etree.Element('yawRate')
            yaw_rate = cls._write_value_exact_or_interval(yaw_rate, state.yaw_rate)
            state_node.append(yaw_rate)
        if hasattr(state, 'slip_angle'):
            slip_angle = etree.Element('slipAngle')
            slip_angle = cls._write_value_exact_or_interval(
                slip_angle, state.slip_angle
            )
            state_node.append(slip_angle)
        return state_node


class PlanningProblemXMLNode:
    @classmethod
    def create_node(cls, planning_problem: PlanningProblem) -> etree.Element:
        """
        Create a xml-Node for a single planning_problem
        :param planning_problem: planning problem for creating the node
        :return:
        """
        planning_problem_node = etree.Element('planningProblem')
        planning_problem_node.set('id', str(planning_problem.planning_problem_id))
        initial_state_node = etree.Element('initialState')
        planning_problem_node.append(
            StateXMLNode.create_state_node(
                planning_problem.initial_state,
                initial_state_node,
                planning_problem.initial_state.time_step,
            )
        )

        for state_id, goal_state in enumerate(planning_problem.goal.state_list):
            if (
                planning_problem.goal.lanelets_of_goal_position is not None
                and state_id in planning_problem.goal.lanelets_of_goal_position
            ):
                goal_lanelet_ids: List[
                    int
                ] = planning_problem.goal.lanelets_of_goal_position[state_id]
            else:
                goal_lanelet_ids = []

            planning_problem_node.append(
                StateXMLNode.create_goal_state_node(goal_state, goal_lanelet_ids)
            )

        return planning_problem_node


class Point:
    def __init__(self, x: Union[int, float], y: Union[int, float]):
        self.x: Union[int, float] = x
        self.y: Union[int, float] = y

    def as_numpy_array(self):
        return np.array([self.x, self.y])

    @classmethod
    def create_from_numpy_array(cls, point: Union[np.array, list]):
        assert len(point) == 2
        return cls(point[0], point[1])

    def create_node(self):
        point_node = etree.Element('point')
        x = etree.Element('x')
        x.text = float_to_str(np.float64(self.x))
        point_node.append(x)
        y = etree.Element('y')
        y.text = float_to_str(np.float64(self.y))
        point_node.append(y)
        return point_node


class Pointlist:
    def __init__(self, points: List[Point]):
        self.points = points

    @classmethod
    def create_from_numpy_array(cls, points: np.array):
        point_list = []
        for point in points:
            point_list.append(Point.create_from_numpy_array(point))
        return cls(point_list)

    def add_points_to_node(self, xml_node: etree.Element):
        for point in self.points:
            xml_node.append(point.create_node())


class IntersectionXMLNode:
    @classmethod
    def create_node(cls, intersection: Intersection) -> etree.Element:
        """
        Create a xml node for an intersection
        :param intersection: intersection for the node
        :return:
        """
        intersection_node = etree.Element('intersection')
        intersection_node.set('id', str(intersection.intersection_id))

        for incoming in intersection.incomings:
            incoming_node = etree.Element('incoming')
            incoming_node.set('id', str(incoming.incoming_id))
            for incoming_lanelet in incoming.incoming_lanelets:
                incoming_lanelet_node = etree.Element('incomingLanelet')
                incoming_lanelet_node.set('ref', str(incoming_lanelet))
                incoming_node.append(incoming_lanelet_node)

            if incoming.successors_right:
                for successor_right in incoming.successors_right:
                    successor_right_node = etree.Element('successorsRight')
                    successor_right_node.set('ref', str(successor_right))
                    incoming_node.append(successor_right_node)

            if incoming.successors_straight:
                for successor_straight in incoming.successors_straight:
                    successor_straight_node = etree.Element('successorsStraight')
                    successor_straight_node.set('ref', str(successor_straight))
                    incoming_node.append(successor_straight_node)

            if incoming.successors_left:
                for successor_left in incoming.successors_left:
                    successor_left_node = etree.Element('successorsLeft')
                    successor_left_node.set('ref', str(successor_left))
                    incoming_node.append(successor_left_node)

            if incoming.incomings_left:
                for is_left_of in incoming.incomings_left:
                    is_left_of_node = etree.Element('isLeftOf')
                    is_left_of_node.set('ref', str(is_left_of))
                    incoming_node.append(is_left_of_node)
            intersection_node.append(incoming_node)

        if intersection.crossings is not None:
            crossing_node = etree.Element('crossing')
            for crossing_lanelet in intersection.crossings:
                crossing_lanelet_node = etree.Element('crossingLanelet')
                crossing_lanelet_node.set('ref', str(crossing_lanelet))
                crossing_node.append(crossing_lanelet_node)
            intersection_node.append(crossing_node)

        return intersection_node


class TrafficSignXMLNode:
    @classmethod
    def create_node(cls, traffic_sign: TrafficSign) -> etree.Element:
        traffic_sign_node = etree.Element('trafficSign')
        traffic_sign_node.set('id', str(traffic_sign.traffic_sign_id))
        for element in traffic_sign.traffic_sign_elements:
            element_node = etree.Element('trafficSignElement')
            sign_id_node = etree.Element('trafficSignID')
            sign_id_node.text = str(element.traffic_sign_element_id)
            element_node.append(sign_id_node)
            for value in element.additional_values:
                value_node = etree.Element('additionalValue')
                value_node.text = str(value)
                element_node.append(value_node)
            traffic_sign_node.append(element_node)

        if traffic_sign.position is not None:
            position_node = etree.Element('position')
            position_node.append(Point(traffic_sign.position[0],
                                       traffic_sign.position[1]).create_node())
            traffic_sign_node.append(position_node)

        if traffic_sign.virtual is not None:
            virtual_node = etree.Element('virtual')
            virtual_node.text = str(traffic_sign.virtual).lower()
            traffic_sign_node.append(virtual_node)
        return traffic_sign_node

    @classmethod
    def create_ref_node(cls, traffic_sign_ref) -> etree.Element:
        traffic_sign_ref_node = etree.Element('trafficSignRef')
        traffic_sign_ref_node.set('ref', str(traffic_sign_ref))
        return traffic_sign_ref_node


class TrafficLightXMLNode:
    @classmethod
    def create_node(cls, traffic_light: TrafficLight) -> etree.Element:
        traffic_light_node = etree.Element('trafficLight')
        traffic_light_node.set('id', str(traffic_light.traffic_light_id))
        cycle_node = etree.Element('cycle')
        for state in traffic_light.cycle:
            element_node = TrafficLightCycleElementXMLNode.create_node(state)
            cycle_node.append(element_node)
        if traffic_light.time_offset is not None:
            offset_node = etree.Element('timeOffset')
            offset_node.text = str(traffic_light.time_offset)
            cycle_node.append(offset_node)
        traffic_light_node.append(cycle_node)

        if traffic_light.position is not None:
            position_node = etree.Element('position')
            position_node.append(Point(traffic_light.position[0],
                                       traffic_light.position[0]).create_node())
            traffic_light_node.append(position_node)
        if traffic_light.active is not None:
            active_node = etree.Element('active')
            active_node.text = str(traffic_light.active).lower()
            traffic_light_node.append(active_node)
        return traffic_light_node

    @classmethod
    def create_ref_node(cls, traffic_light_ref) -> etree.Element:
        traffic_light_ref_node = etree.Element('trafficLightRef')
        traffic_light_ref_node.set('ref', str(traffic_light_ref))
        return traffic_light_ref_node


class TrafficLightCycleElementXMLNode:
    @classmethod
    def create_node(cls, cycle_element: TrafficLightCycleElement) -> etree.Element:
        element_node = etree.Element(str(cycle_element.state.value))
        element_node.text = str(cycle_element.duration)
        return element_node


class LineMarkingXMLNode:
    @classmethod
    def _line_marking_enum_to_string(cls, line_marking):
        return str(line_marking.name.lower())

    @classmethod
    def create_node(cls, line_marking) -> etree.Element:
        line_marking_node = etree.Element('lineMarking')
        line_marking_node.text = cls._line_marking_enum_to_string(line_marking)
        return line_marking_node


class LaneletStopLineXMLNode:
    @classmethod
    def create_node(cls, stop_line : StopLine) -> etree.Element:
        stop_line_node = etree.Element('stopLine')

        start_node = Point(stop_line.start.x, stop_line.start.y).create_node()
        stop_line_node.append(start_node)
        end_node = Point(stop_line.end.x, stop_line.end.y).create_node()
        stop_line_node.append(end_node)

        if stop_line.line_marking:
            line_marking_node = LineMarkingXMLNode.create_node(stop_line.line_marking)
            stop_line_node.append(line_marking_node)

        if stop_line.traffic_sign_ref:
            traffic_sign_ref_node = TrafficSignXMLNode.create_ref_node(stop_line.traffic_sign_ref)
            stop_line_node.append(traffic_sign_ref_node)

        if stop_line.traffic_light_ref:
            traffic_light_ref_node = TrafficLightXMLNode.create_ref_node(stop_line.traffic_light_ref)
            stop_line_node.append(traffic_light_ref_node)

        return stop_line_node


