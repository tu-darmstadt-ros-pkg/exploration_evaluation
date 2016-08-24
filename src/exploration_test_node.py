#!/usr/bin/env python
import csv
import time
from time import gmtime, strftime
from std_msgs.msg import *
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from flexbe_msgs.msg import BehaviorExecutionActionGoal
import rospy
import roslib; roslib.load_manifest('exploration_evaluation')


class ExplorationTester(object):

    '''Class that automatically starts a exploration behaviour on given world and
    creates reports abouts victims found and the created map.
    '''

    def __init__(self):
        '''Intializes variables, publishers, subscribers and files, 
        then starts autonomy.
        '''
        self._occupancy_grid = None
        self.num_victims_found = 0
        self.autonomy_start_time = rospy.Time.now()
        self._autonomous = False
        self._map_timer = None
        self.date_string = strftime("%Y_%m_%d_%H-%M-%S", gmtime())

        # File variables
        self._map_file = None
        self._map_file_writer = None

        # ROS Publishers & Subscribers
        self.sys_command_publisher = None
        self._behavior_publisher = None
        self._map_subscriber = None

        rospy.loginfo("Init ExplorationTester")
      
        self.init_map_file()

        self.init_publishers()
        time.sleep(0.2)    
        self.init_subscribers()
        time.sleep(30)
      
        self.send_autonomy_start()


    def init_map_file(self):
        '''Init file-writers.'''
        self._map_file = open(self.date_string + '_map_data.csv', 'wb')
        self._map_file_writer = csv.writer(self._map_file, delimiter=',')


    def init_publishers(self):
        '''Initializes publishers used during exploration.'''
        self.sys_command_publisher = rospy.Publisher("syscommand", String, queue_size=5)
        self._behavior_publisher = rospy.Publisher('/flexbe/execute_behavior/goal', BehaviorExecutionActionGoal, queue_size=5)


    def init_subscribers(self):
        '''Initializes subscribers used during exploration.'''
        self._map_subscriber = rospy.Subscriber('scanmatcher_map', OccupancyGrid, self.handle_occupancy_grid)


    def handle_occupancy_grid(self, msg):
        '''Callback method for the occupancy grid.'''
        self._occupancy_grid = msg


    def send_autonomy_start(self):
        '''Starts FlexBe exploration behavior and starts callback for logging
        map exploration progress.
        '''
        unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause_physics_client()
        rospy.loginfo("Gazebo sim activated.")

        rospy.sleep(10)
        self.start_exploration()

        self.autonomy_start_time = rospy.Time.now()
        self.write_map_exploration_progress_entry()
        
        #self._autonomy_publisher.publish(True)
        self._autonomous = True
        
        self._map_timer = rospy.Timer(rospy.Duration(10), self.map_timer_callback)


    def start_exploration(self):
        '''Starts FlexBe behavior according to param "mission_behavior".'''
        msg = BehaviorExecutionActionGoal()
        #msg.goal.behavior_name = 'Search Victims'
        mission_full_param_name = rospy.search_param('mission_behavior')
        msg.goal.behavior_name = rospy.get_param(mission_full_param_name)
        self._behavior_publisher.publish(msg)
        rospy.loginfo("FlexBe exploration behavior "+ rospy.get_param(mission_full_param_name) +" activated.")


    def map_timer_callback(self, event):
        '''Callback for writing map exploration progress.'''
        self.write_map_exploration_progress_entry()


    def write_map_exploration_progress_entry(self):
        '''Writes an entry with known cells and elapsed time in map-file.'''
        num_cells = self.compute_known_cells()
        elapsed_time_in_sec = str((rospy.Time.now() - self.autonomy_start_time).to_sec())
      
        self._map_file_writer.writerow([str(elapsed_time_in_sec), str(num_cells)])


    def compute_known_cells(self):
        '''Computes currently known cells in environment'''
        if not self._occupancy_grid:
            return 0

        num_unknown_cells = 0.0

        for cell in self._occupancy_grid.data:
            if cell == -1:
                num_unknown_cells += 1
        
        return int(len(self._occupancy_grid.data) - num_unknown_cells)


    def close_file_writers(self):
        '''Closes open file writers.'''
        self._map_file.close()


if __name__ == "__main__":
    '''Creates an instance of ExplorationTester for a specified time. 
    Afterwards prints statistics and saves map.
    '''
    rospy.init_node("exploration_evaluation")

    time.sleep(0.5)

    exploration_instance = ExplorationTester()

    r = rospy.Rate(10) # 10hz

    mission_time_full_param_name = rospy.search_param('mission_sim_time')
    mission_sim_time_in_sec = int(rospy.get_param(mission_time_full_param_name))
    rospy.loginfo("Sim will be executed for " + str(mission_sim_time_in_sec) + " seconds.")
    
    # mission_sim_time_in_sec seconds sim-time.
    while not ((rospy.Time.now() - exploration_instance.autonomy_start_time).to_sec() 
               > mission_sim_time_in_sec):
        r.sleep()
    
    exploration_instance.close_file_writers()

    time.sleep(10)
    rospy.loginfo("ExplorationTester done. Exiting.")

    rospy.loginfo("Experiment at " + exploration_instance.date_string + " ended.")
    rospy.loginfo("Robot found " + str(exploration_instance.num_victims_found) + " victims.")
        
    rospy.loginfo("Saving GeoTiff...")
    geotiff_string = String()
    geotiff_string = "savegeotiff"
    exploration_instance.sys_command_publisher.publish(geotiff_string)
