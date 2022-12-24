from distutils.log import ERROR
from enum import Enum
from locale import ERA
from velocity.run_w import *
import behavior
import _GoToPoint_
import _GoOnArc_
import _GoOnArc_intercept
import time

import rospy
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *
from krssg_ssl_msgs.srv import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from utils import config, functions
import rospy,sys

rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)

prev_state = None
try:
    prev_state = getState(prev_state).stateB
except rospy.ServiceException, e:
    print("Error ", e)


# class Goalie(behavior.Behavior):

#     class State(Enum):

#         setup = 1
#         centre_of_goal = 2
#         move_towards_ball = 3
#         centre_of_projection = 4

#     def __init__(self,max_vel = MAX_BOT_SPEED/1.2,min_vel = MIN_BOT_SPEED*1.2):

#         super(Goalie,self).__init__()
class Goalie(behavior.Behavior):

    class State(Enum):
        setup=1
        center_of_goal=2
        move_towards_ball=3
        center_of_projection=4

    def __init__(self):
        super(Goalie,self).__init__()

        self.Dbox_vertical_length = 200

        self.Dbox_horizontal_length = 100

        self.Field_horizontal_length = 6000

        self.p1 = Vector2D()
        self.p1.x = OUR_GOAL_MAXX
        self.p1.y = OUR_GOAL_MAXY
        self.p2 = Vector2D()
        self.p2.x = OUR_GOAL_MAXX
        self.p2.y = OUR_GOAL_MINY

        self.centre = (self.p1+self.p2)/2

        self.name = "Goalie"

        self.add_state(Goalie.State.setup,behavior.Behavior.State.running)

        self.add_state(Goalie.State.center_of_goal,behavior.Behavior.State.running)

        self.add_state(Goalie.State.move_towards_ball,behavior.Behavior.State.running)

        self.add_state(Goalie.State.center_of_projection,behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            Goalie.State.setup, lambda: True,
             'immediately')

        self.add_transition(Goalie.State.setup, 
            Goalie.State.center_of_goal, lambda: not self.Ball_inside_midfield(),
             'center of goal')

        self.add_transition(Goalie.State.setup,
            Goalie.State.center_of_projection, lambda: not self.Ball_inside_Dbox() and self.Ball_inside_midfield(),
             'center of projection')

        self.add_transition(Goalie.State.setup, 
            Goalie.State.move_towards_ball, lambda: self.Ball_inside_Dbox(),
            'Move towards ball')

        self.add_transition(Goalie.State.center_of_goal, 
            Goalie.State.move_towards_ball, lambda: self.Ball_inside_Dbox(),'From goal towards ball')

        self.add_transition(Goalie.State.center_of_goal, 
            Goalie.State.center_of_projection, lambda: not self.Ball_inside_Dbox() and self.Ball_inside_midfield(),
            'Goal towards projection')

        self.add_transition(Goalie.State.center_of_projection, 
            Goalie.State.move_towards_ball, lambda: self.Ball_inside_Dbox(),
            'ball came inside Dbox')

        self.add_transition(Goalie.State.center_of_projection,
            Goalie.State.center_of_goal, lambda: not self.Ball_inside_midfield(),
            'ball went out of midfield')

        self.add_transition(Goalie.State.move_towards_ball, 
            Goalie.State.center_of_projection, lambda: not self.Ball_inside_Dbox() and self.Ball_inside_midfield(),
            'ball went out of Dbox') 




    def Ball_inside_midfield(self):
        return self.kub.state.ballPos.x < 0

    def Ball_inside_Dbox(self):
        return self.kub.state.ballPos.x < self.Dbox_horizontal_length - self.Field_horizontal_length/2 and self.kub.state.ballPos.x > - self.Field_horizontal_length/2 and self.kub.state.ballPos.y < Dbox_vertical_length/2 and self.kub.state.ballPos.y > -Dbox_vertical_length/2






    def add_kub(self,kub):
        self.kub = kub

    def mid_point_of_projection(self):

        if magnitute(self.p1 - self.kub.ballPos) <= magnitute(self.p2 - self.kub.ballPos):
            # drop perpendicular from self.p1 to line joining ball and self.p2
            # find intersection of this perpendicular with line joining ball and self.p2
            # return this point
            line = Line(self.kub.ballPos, self.p2)
            perp = line.perpendicular(self.p1)
            temp = perp.intersection_with_line(line)
            return (temp+self.p1)/2
            pass
        else:
            # drop perpendicular from self.p2 to line joining ball and p1
            # find intersection of this perpendicular with line joining ball and p1
            # return this point
            line = Line(self.kub.ballPos, self.p1)
            perp = line.perpendicular(self.p2)
            temp = perp.intersection_with_line(line)
            return (temp+self.p2)/2

    def calcTheta(self):
        if  self.kub.state.ballPos.x - self.centre.x != 0 :
            theta = normalize_angle(atan2(self.kub.state.ballPos.y - self.centre.y,+self.kub.state.ballPos.x - self.centre.x))
        elif self.kub.state.ballPos.y > self.centre.y :
            theta = math.pi/2
        else:
            theta = -math.pi/2

            self.theta = theta

    def on_enter_setup(self):
        self.behavior_failed = False

    def execute_setup(self):
        ball_pos = Vector2D(self.kub.state.ballPos.x,self.kub.state.ballPos.y)
        pass

    def on_exit_setup(self):
        pass

    def on_enter_centre_of_goal(self):
        self.target_point = self.centre
        theta = 0
        _GoToPoint_.init(self.kub, self.target_point, theta)

    def execute_centre_of_goal(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh/5,True)
        #self.behavior_failed = False
        for gf in generatingfunction:
            self.kub,target_point = gf
            # self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
            if not vicinity_points(self.target_point,target_point,thresh=BOT_RADIUS):
                self.behavior_failed = True
                break

    def on_exit_centre_of_goal(self):
        pass

    def on_enter_move_towards_ball(self):
        self.calcTheta()
        self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
        _GoToPoint_.init(self.kub, self.target_point, self.theta)

    def execute_move_towards_ball(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh/5,True)
        #self.behavior_failed = False
        for gf in generatingfunction:
            self.kub,target_point = gf
            # self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
            if not vicinity_points(self.target_point,target_point,thresh=BOT_RADIUS):
                self.behavior_failed = True
                break

    def on_exit_move_towards_ball(self):
        pass

    def on_enter_center_of_projection(self):
        self.target_point = self.mid_point_of_projection()
        pass

    def execute_center_of_projection(self):
        pass

    def on_exit_center_of_projection(self):
        pass