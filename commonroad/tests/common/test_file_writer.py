import logging
import os
import unittest

from shapely.geometry import Point

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.geometry.shape import *
from lxml import etree
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet, GoalRegion
from commonroad.prediction.prediction import *
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork, LineMarking, LaneletType, StopLine
from commonroad.scenario.obstacle import *
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.traffic_rule import TrafficSign, TrafficSignElement, TrafficSignID
from commonroad.scenario.trajectory import *


class TestFileWriter(unittest.TestCase):
    def setUp(self):
        self.cwd_path = os.path.dirname(os.path.abspath(__file__))
        self.xsd_path = self.cwd_path + "/XML_commonRoad_XSD.xsd"
        self.out_path = self.cwd_path + "/../.pytest_cache"
        if not os.path.isdir(self.out_path):
            os.makedirs(self.out_path)
        else:
            for (dirpath, dirnames, filenames) in os.walk(self.out_path):
                for file in filenames:
                    if file.endswith('.xml'):
                        os.remove(os.path.join(dirpath, file))

    def test_writing_shapes(self):
        rectangle = Rectangle(4.3, 8.9, center=np.array([2.5, -1.8]), orientation=1.7)
        polygon = Polygon(np.array([np.array((0.0, 0.0)), np.array((0.0, 1.0)), np.array((1.0, 1.0)), np.array((1.0, 0.0))]))
        circ = Circle(2.0, np.array([10.0, 0.0]))
        sg = ShapeGroup([circ, rectangle])
        occupancy_list = list()
        occupancy_list.append(Occupancy(0, rectangle))
        occupancy_list.append(Occupancy(1, circ))
        occupancy_list.append(Occupancy(2, polygon))
        occupancy_list.append(Occupancy(3, circ))


        set_pred = SetBasedPrediction(0,occupancy_list)

        states = list()
        states.append(State(time_step=0, orientation=0, position=np.array([0, 0])))
        states.append(State(time_step=1, orientation=0, position=np.array([0, 1])))
        trajectory = Trajectory(0, states)

        init_state = State(time_step=0, orientation=0, position=np.array([0, 0]))

        traj_pred = TrajectoryPrediction(trajectory, rectangle)

        static_obs = StaticObstacle(3,ObstacleType(0), obstacle_shape=circ, initial_state=init_state)
        dyn_set_obs = DynamicObstacle(1,ObstacleType(0),
                                      initial_state=traj_pred.trajectory.state_at_time_step(0),
                                      prediction=set_pred, obstacle_shape=rectangle)
        dyn_traj_obs = DynamicObstacle(2, ObstacleType(0),
                                       initial_state=traj_pred.trajectory.state_at_time_step(0),
                                       prediction=traj_pred, obstacle_shape=rectangle)
        lanelet1_stopline = StopLine(Point(12345.12, 0.0), Point(0.0, 1), LineMarking.SOLID)
        lanelet1 = Lanelet(np.array([[12345.12, 0.0], [1.0,0.0],[2,0]]), np.array([[0.0, 1],[1.0,1],[2,1]]), np.array([[0.0, 2], [1.0,2],[2,2]]), 100,
                [101], [101], 101, False, 101, True,
                          LineMarking.DASHED, LineMarking.SOLID,lanelet1_stopline, {LaneletType.HIGHWAY}, None, None, {1})
        lanelet2 = Lanelet(np.array([[0.0, 0.0], [1.0, 0.0], [2, 0]]), np.array([[0.0, 1], [1.0, 1], [2, 1]]),
                           np.array([[0.0, 2], [1.0, 2], [2, 2]]), 101,
                           [100], [100], 100, False, 100, True,
                           LineMarking.SOLID, LineMarking.DASHED, None, {LaneletType.HIGHWAY}, None, None, {1})
        traffic_sign_max_speed = TrafficSignElement(TrafficSignID.MAXSPEED.value, [10.0])
        traffic_sign = TrafficSign(1, {traffic_sign_max_speed})

        lanelet_network = LaneletNetwork().create_from_lanelet_list(list([lanelet1, lanelet2]))
        lanelet_network.add_traffic_sign(traffic_sign)
        scenario = Scenario(0.1,'ZAM_test_0-0-1')
        scenario.add_objects([static_obs, lanelet_network])

        goal_region = GoalRegion([State(time_step=Interval(0,1),velocity=Interval(0.0,1),position=rectangle),
                                  State(time_step=Interval(1,2),velocity=Interval(0.0,1),position=circ)],{0:[100,101],1:list([101])})
        planning_problem = PlanningProblem(1000, State(velocity=0.1, position=np.array([[0],[0]]),  orientation=0,
                                                       yaw_rate=0, slip_angle=0, time_step=0), goal_region)
        planning_problem_set = PlanningProblemSet(list([planning_problem]))
        filename = self.out_path + '/test_writing_shapes.xml'
        CommonRoadFileWriter(scenario,planning_problem_set, 'PrinceOfZAM','TU Munich','unittest','original').\
            write_to_file(filename=filename,overwrite_existing_file=OverwriteExistingFile.ALWAYS)
        assert self.validate_with_xsd(self.out_path + '/test_writing_shapes.xml')
        # test overwriting
        CommonRoadFileWriter(scenario, planning_problem_set, 'PrinceOfZAM', 'TU Munich', 'unittest', 'should_not_appear'). \
            write_to_file(filename=filename,
                          overwrite_existing_file=OverwriteExistingFile.SKIP)
        CommonRoadFileWriter(scenario, planning_problem_set, 'PrinceOfZAM', 'TU Munich', 'unittest', 'overwritten'). \
            write_to_file(filename=filename,
                          overwrite_existing_file=OverwriteExistingFile.ALWAYS)


    def validate_with_xsd(self, xml_path: str) -> bool:
        xmlschema_doc = etree.parse(self.xsd_path)
        xmlschema = etree.XMLSchema(xmlschema_doc)

        xml_doc = etree.parse(xml_path)
        try:
            xmlschema.assert_(xml_doc)
            return True
        except Exception as e:
            logging.error('xml produced by file_writer not conformant with xsd-scheme: ' + str(e))
            return False

    # def test_all_scenarios(self):
    #     file_list=[]
    #     for (dirpath, dirnames, filenames) in os.walk(self.cwd_path + "/../../../../../../scenarios/NGSIM/Lankershim"):
    #         file_list.extend(filenames)
    #         break
    #     from commonroad.common.file_reader import CommonRoadFileReader
    #     for file in file_list:
    #         scenario_tmp, planning_problem_tmp = CommonRoadFileReader(os.path.join(dirpath, file)).open()
    #         CommonRoadFileWriter(scenario_tmp, planning_problem_tmp, 'PrinceOfZAM', 'TU Munich', 'unittest', 'overwritten'). \
    #             write_to_file(filename=self.out_path + '/' + file, overwrite_existing_file=OverwriteExistingFile.SKIP)
    #         assert self.validate_with_xsd(self.out_path + '/' + file)


if __name__ == '__main__':
    unittest.main()
