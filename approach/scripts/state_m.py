#!/usr/bin/env python 
import rospy
from gate import Gate
from approach_by_loc_main import Approach_by_loc
from rovering import  TurningAround
import smach
import smach_ros

class searching_state(smach.State):
      def __init__(self):
         smach.State.__init__(self, 
                              outcomes=['1','2'],
                              input_keys=['post_number'],
                              )

      def execute(self, userdata):
         k = Approach_by_loc()
         Approach_by_loc().run()
         del(k)
         if userdata.post_number == 4:
            return "2"
         else :
            return "1"


class one_marker_state(smach.State):
   def __init__(self):
      smach.State.__init__(self, 
                     outcomes=['Success_Post','Return_look'])
      
   def execute(self,userdata):
      result = TurningAround().run()
      if result == True:   
         return "Success_Post"
      else:
         return "Return_look"

class gate_state(smach.State):
   def __init__(self):
      smach.State.__init__(self, 
                        outcomes=['Success_Post','Return_look'])
      
   def execute(self,userdata):
      result = Gate().run()
      if result == True:
         return "Success_Post"
      else:
         return "Return_look"


def main_sub():
   rospy.init_node("Approach_state_m")
   
   sm_sub = smach.StateMachine(outcomes=['Success_Post'],
                          input_keys=['post_number'],
                          )
   sm_sub.userdata.post_number = 4 # if it is 4 do gate, otherwsie do one marker
   with sm_sub:
      smach.StateMachine.add('Searching',searching_state(),
                            transitions={'1':'One_marker',
                                         '2':'Gate'},
                            remapping={'post_number':'post_number'
                           })
      smach.StateMachine.add('One_marker', one_marker_state(),
                            transitions={'Success_Post':'Success_Post',
                                         'Return_look':'Searching'},
                           )
      smach.StateMachine.add('Gate', gate_state(),
                            transitions={'Success_Post':'Success_Post',
                                         'Return_look':'Searching'},
                           )
   outcome = sm_sub.execute()

if __name__ == '__main__':
    main_sub()