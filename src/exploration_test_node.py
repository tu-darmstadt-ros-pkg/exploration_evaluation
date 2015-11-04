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
from sar_msgs.msg import SarTaskTypes
from monstertruck_msgs.msg import Status
from hector_move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist
from hector_worldmodel_msgs.srv import AddObject
from hector_worldmodel_msgs.msg import ObjectState

from sar_msgs.msg import VictimAnswer, Victims


class ExplorationTester:

    def __init__(self):
      
      rospy.loginfo("Init")
      
      self._autonomy_start_time = rospy.Time.now()
      
      self._publisher = rospy.Publisher("victimAnswer", VictimAnswer, queue_size=5)
      self._autonomyPublisher = rospy.Publisher('taskallocation/startAutonomy', Bool, queue_size=5)
      
      rospy.sleep(0.2)
            
      self._subscriber = rospy.Subscriber("victimFound", Victims, self._on_victim_found)
      
      rospy.sleep(0.2)
      
      self.send_autonomy_start()
      
    def _confirm_victim(self):
        self._victimAnswer = VictimAnswer.CONFIRM
        
        #self._publish_answer()
        
        answer = VictimAnswer()
        answer.task_id = self._task_id
        answer.answer = self._victimAnswer
        self._publisher.publish(answer)
        
        #self._widget.confirm_push_button.setEnabled(False)
        #self._widget.get_closer_push_button.setEnabled(False)
        #self._widget.ignore_push_button.setEnabled(False)
        #self._widget.postpone_push_button.setEnabled(False)        
        
        
        status_msg = "confirmed victim, id = "
        status_msg += self._task_id
        rospy.loginfo(status_msg)  
      
    def _on_victim_found(self, message):
        if not len(message.victim_ids) == 0:
            #subprocess.call(['/usr/bin/canberra-gtk-play','--id','phone-incoming-call']) 
            
            victim_found_time = rospy.Time.now()
            
            status_msg = "Time " + str((victim_found_time - self._autonomy_start_time).to_sec())
            status_msg += " received victim found message, id = "
            status_msg += message.victim_ids[0]
            #self._print_message_signal.emit(status_msg)
            rospy.loginfo(status_msg)
            
            self._task_id = message.victim_ids[0]
            
            self._confirm_victim()
            #self._widget.confirm_push_button.setEnabled(True)
            #self._widget.confirm_push_button.setStyleSheet('QPushButton:enabled {background-color: lightgreen; border-style: outset; border-width: 2px; border-radius: 5px; border-color: black; padding: 6px;}')
            #self._widget.get_closer_push_button.setEnabled(True)
            #self._widget.get_closer_push_button.setStyleSheet('QPushButton:enabled {background-color: yellow; border-style: outset; border-width: 2px; border-radius: 5px; border-color: black; padding: 6px;}')
            #self._widget.ignore_push_button.setEnabled(True)
            #self._widget.ignore_push_button.setStyleSheet('QPushButton:enabled {background-color: red; border-style: outset; border-width: 2px; border-radius: 5px; border-color: black; padding: 6px;}')
            #self._widget.postpone_push_button.setEnabled(True)
            
        else:
            #self._widget.confirm_push_button.setEnabled(False)
            #self._widget.get_closer_push_button.setEnabled(False)
            #self._widget.ignore_push_button.setEnabled(False)
            #self._widget.postpone_push_button.setEnabled(False)    
            pass
      
    def send_autonomy_start(self):
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
                    #self._write_message(err_msg)
                    rospy.logerr(err_msg)
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
                    #print req
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

                    #print req
                    add_task(req)
                    
                except rospy.ServiceException, e:
                    err_msg = "error: "
                    err_msg += str(e)
                    self._write_message(err_msg)
                
                
                
                rospy.loginfo("Sent start request")
                self._autonomy_start_time = rospy.Time.now()
                
                self._autonomyPublisher.publish(True)
                self._autonomous = True
                #self._widget.autonomy_button.setText(self._stopAutonomyText)
                #self._write_message('started autonomy')
            
                #self._widget.start_mode_combobox.setCurrentIndex(4)
            
                #self._on_add_test_object()
  


    # Velocity callback. Recomputes trajectory when new velocity command comes in
    def vel_cmd_callback(self, data):
        self._velocity_command = data.data
        if self._velocity_command > 0:
            self._trajectory_points = self.set_motion_properties(0, self._velocity_command, self._param_acceleration, self._param_min_angle, self._param_max_angle)
        else:
            self._trajectory_points = []
        self._client.cancel_all_goals()

    def run(self):
        #next_start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
          rospy.sleep(0.1)
          
if __name__ == "__main__":
        rospy.init_node("exploration_evaluation")
        try:
                rospy.sleep(0.5)
        except rospy.exceptions.ROSInterruptException:
                pass
        exploration_tester = ExplorationTester()
        rospy.spin()