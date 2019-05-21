import warnings
from typing import List, Dict, Union

import matplotlib as mpl
import matplotlib.pyplot as plt
from commonroad.common.util import Interval
from commonroad.scenario.scenario import Scenario
from commonroad.visualization.draw_dispatch_cr import draw_object

__author__ = "Moritz Klischat"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = [""]
__version__ = "2019.1"
__maintainer__ = "Moritz Klischat"
__email__ = "commonroad@in.tum.de"
__status__ = "Released"

"""
Example for a fast updating of dynamic obstacles with matplotlib:
# 1. set backend for matplotlib (Qt5Agg or TkAgg recommended) directly after importing matplotlib for the first time
import matplotlib
matplotlib.use('Qt5Agg')
import os
from commonroad.common.file_reader import CommonRoadFileReader

# 2. example function:
def example_plot():        
    filename = os.getcwd() + '/scenarios/NGSIM/US101/USA_US101-4_1_T-1.xml'
    scenario, planning_problem_set = CommonRoadFileReader(filename).open()
    from commonroad.visualization.plot_helper import *
    
    set_non_blocking()  # ensures interactive plotting is activated
    plt.style.use('classic')
    inch_in_cm = 2.54
    figsize = [30, 8]
    
    fig = plt.figure(figsize=(figsize[0] / inch_in_cm, figsize[1] / inch_in_cm))
    fig.gca().axis('equal')
    handles = {}  # collects handles of obstacle patches, plotted by matplotlib
    
    # inital plot including the lanelet network
    draw_object(scenario, handles=handles)
    fig.canvas.draw()
    plt.gca().autoscale()
    
    # loop where obstacle positions are updated
    for i in range(0, nrun):
        # ...
        # change positions of obstacles
        # ...
        redraw_obstacles(scenario, handles=handles, figure_handle=fig, plot_limits=None, draw_params={'time_begin':i})
    
    print((time.time()-t1)/nrun)
        
"""

def set_non_blocking() -> None:
    """
    Ensures that interactive plotting is enabled for non-blocking plotting.

    :return: None
    """

    plt.ion()
    if not mpl.is_interactive():
        warnings.warn('The current backend of matplotlib does not support interactive mode: '
                      + str(mpl.get_backend()) + '. Select another backend with: \"matplotlib.use(\'TkAgg\')\"',
                      UserWarning, stacklevel=3)


def redraw_obstacles(scenario: Scenario, handles: Dict[int,List[mpl.patches.Patch]], figure_handle: mpl.figure.Figure,
                     draw_params=None, plot_limits: Union[List[Union[int,float]], None] = None) -> None:
    """
    This function is used for fast updating dynamic obstacles of an already drawn plot. Saves about 80% time compared to a complete plot.
    Deletes all dynamic obstacles which are specified in handles and draws dynamic obstacles of a scenario.

    :param scenario: scenario from which obstacle should be plotted
    :param handles: dict of obstacle_ids and corresponding patch handles (generated by draw_object function)
    :param plot_limits: axis limits for plot [x_min, x_max, y_min, y_max]
    :param figure_handle: figure handle of current plot
    :return: None
    """
    #remove dynamic obstacle from current plot
    for handles_i in handles.values():
        for handle in handles_i:
            handle.remove()
    handles.clear()

    # redraw dynamic obstacles
    if plot_limits is not None:
        draw_object(scenario.obstacles_by_position_intervals([Interval(plot_limits[0], plot_limits[1]),
                                                              Interval(plot_limits[2], plot_limits[3])]),
                    draw_params=draw_params, plot_limits=plot_limits, handles=handles)
    else:
        draw_object(scenario.obstacles, draw_params=draw_params, plot_limits=plot_limits, handles=handles)

    # update plot
    ax = figure_handle.gca()
    for handles_i in handles.values():
        for handle in handles_i:
            ax.draw_artist(handle)

    if mpl.get_backend() =='TkAgg':
        figure_handle.canvas.draw()

    elif mpl.get_backend() =='Qt5Agg':
        figure_handle.canvas.update()
    else:
        try:
            figure_handle.canvas.update()
        except:
            raise Exception('<plot_helper/redraw_dynamic_obstacles> Backend for matplotlib needs to be \'Qt5Agg\' or \'TkAgg\' but is'
                        '\'%s\'' % mpl.get_backend())

    figure_handle.canvas.flush_events()