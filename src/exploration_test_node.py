#!/usr/bin/env python
import roslib; roslib.load_manifest('exploration_evaluation')
import math
import rospy
#import sensor_msgs.msg
#import actionlib
#from control_msgs.msg import *
#from trajectory_msgs.msg import *
from std_msgs.msg import *
from copy import deepcopy


from task_msgs.msg import TasksModel
from std_msgs.msg import Bool, String, Float32, Empty
from task_msgs.msg import SupervisionCommand, TaskState, TaskCost, SupervisionSettings
from task_msgs.srv import AddTask
#from sar_msgs.msg import SarTaskTypes
from monstertruck_msgs.msg import Status
from hector_move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist
from hector_worldmodel_msgs.srv import AddObject
from hector_worldmodel_msgs.msg import ObjectState


class ExplorationTester:

    def __init__(self):
      
      self._publisher = rospy.Publisher("victimAnswer", VictimAnswer)
      
      self._subscriber = rospy.Subscriber("victimFound", Victims, self._on_victim_found)
      
      
      
    def send_autonomy_start:
                  # re-send data for mapping or sar mission
            #if self._widget.sar_radiobutton.isChecked():
            #    self._on_sar_radiobutton_pressed()
            #else:
            #    self._on_mapping_radiobutton_pressed()
            
            # add exploration task if not done so far
            #if not self._initializedExploreTask:
                req = AddTask._request_class()
                req.task.state.state = TaskState.PENDING

                try:
                    rospy.wait_for_service('taskallocation/add_task',3)
                except rospy.exceptions.ROSException, e:
                    err_msg = "error: "
                    err_msg += str(e)
                    self._write_message(err_msg)
                    return
                    
                try:
                    add_task = rospy.ServiceProxy('taskallocation/add_task', AddTask)
                    
                    req.task.header.stamp = rospy.Time(0)
                    req.task.header.frame_id = "/map"
                    req.task.pose.orientation.w = 1
                    
                    req.task.details.task_type = SarTaskTypes.START
                    req.task.details.task_id = SarTaskTypes.START
                    req.task.details.boolParams = []
                    req.task.details.floatParams = []
                    req.task.details.floatParams.insert(SarTaskTypes.INDEX_START_DIRECTION, 5.0)
                    add_task(req)

                    req.task.details.task_type = SarTaskTypes.EXPLORE
                    req.task.details.task_id = "explore"
                    req.task.details.boolParams = []
                    req.task.details.floatParams = []
                    
                    #if self._widget.sar_radiobutton.isChecked():
                    req.task.details.boolParams.insert(0, True)
                    #elif self._widget.mapping_radiobutton.isChecked():
                    #    req.task.details.boolParams.insert(0, False)
                    #else:
                    #    print 'Warning: No mission selected!'

                    add_task(req)
                    
                except rospy.ServiceException, e:
                    err_msg = "error: "
                    err_msg += str(e)
                    self._write_message(err_msg)
                
            self._autonomyPublisher.publish(True)
            self._autonomous = True
            self._widget.autonomy_button.setText(self._stopAutonomyText)
            self._write_message('started autonomy')
            
            self._widget.start_mode_combobox.setCurrentIndex(4)
            
            self._on_add_test_object()
  


    # Velocity callback. Recomputes trajectory when new velocity command comes in
    def vel_cmd_callback(self, data):
        self._velocity_command = data.data
        if self._velocity_command > 0:
            self._trajectory_points = self.set_motion_properties(0, self._velocity_command, self._param_acceleration, self._param_min_angle, self._param_max_angle)
        else:
            self._trajectory_points = []
        self._client.cancel_all_goals()

    def run(self):
        next_start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
          rospy.sleep(0.1)
          
if __name__ == "__main__":
        rospy.init_node("exploration_evaluation")
        try:
                rospy.sleep(0.5)
        except rospy.exceptions.ROSInterruptException:
                pass
        exploration_tester = ExplorationTester()
        exploration_tester.run()