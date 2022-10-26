from enum import Enum
import behavior
import _GoToPoint_
import rospy
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *
from utils.geometry import Vector2D
import time

class DribbleAndKick(behavior.Behavior):

    class State(Enum):
        setup = 1 
        course_approach = 2
        fine_approach = 3
        intercept = 4
        dribble_move = 5
        kick = 6

    def __init__(self,course_approch_thresh =  DISTANCE_THRESH/3,continuous=False):

        super(DribbleAndKick,self).__init__()

        self.name = "DribbleAndKick"

        self.power = 7.0

        self.target_point = None
        

        self.course_approch_thresh = 4*course_approch_thresh

        self.ball_dist_thresh = 1.4*BOT_BALL_THRESH #changed from 1.2 to 1.4

        self.behavior_failed = False
        

        self.add_state(DribbleAndKick.State.setup,
            behavior.Behavior.State.running)

        self.add_state(DribbleAndKick.State.course_approach,
            behavior.Behavior.State.running)
        
        self.add_state(DribbleAndKick.State.fine_approach,
            behavior.Behavior.State.running)

        # self.add_state(DribbleAndKick.State.intercept,
        #     behavior.Behavior.State.running)

        self.add_state(DribbleAndKick.State.dribble_move,
            behavior.Behavior.State.running)

        # self.add_state(DribbleAndKick.State.kick,
        #     behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            DribbleAndKick.State.setup,lambda:True,'immediately')


        self.add_transition(DribbleAndKick.State.setup,
            DribbleAndKick.State.course_approach,lambda:not self.at_target_point() and not self.ball_moving(),'course approach')

        self.add_transition(DribbleAndKick.State.setup,
            DribbleAndKick.State.fine_approach,lambda:self.at_target_point() and not self.ball_moving(),'fine approach')

        # self.add_transition(DribbleAndKick.State.setup,
        #     DribbleAndKick.State.intercept,lambda:self.ball_moving(), 'intercept')

        self.add_transition(DribbleAndKick.State.course_approach,
            DribbleAndKick.State.fine_approach,lambda:self.at_target_point() and not self.ball_moving(),'course-fine approach')

        self.add_transition(DribbleAndKick.State.fine_approach,
            DribbleAndKick.State.dribble_move,lambda: self.ball_connected(),'dribble')

        
        self.add_transition(DribbleAndKick.State.dribble_move,
            behavior.Behavior.State.completed,lambda:self.at_target_point(),'complete') #wrong

        # self.add_transition(DribbleAndKick.State.dribble_move,DribbleAndKick.State.kick,
        #     lambda:self.near_the_goal() and self.ball_connected(), 'kick')

    def at_target_point(self):
        return vicinity_points(self.target_point,self.kub.get_pos(),thresh= self.course_approch_thresh)

    def ball_moving(self):
        return False


    def at_ball_pos(self):
        return ball_in_front_of_bot(self.kub)

    def near_the_goal(self):        
        return False

    def ball_connected(self):
        return  kub_has_ball(self.kub.state, self.kub.kubs_id)

    def add_kub(self,kub):
        self.kub = kub

    def add_theta(self,theta):
        self.theta = theta
    
    
    def on_enter_setup(self):
        self.target_point = self.kub.state.ballPos
    
    def execute_setup(self):
        pass
        
    def on_exit_setup(self):
        pass

    def on_enter_course_approach(self):
        self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
        self.target_point = self.kub.state.ballPos
        _GoToPoint_.init(self.kub, self.target_point, self.theta)
        pass

    def execute_course_approach(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,True)
        #self.behavior_failed = False
        for gf in generatingfunction:
            self.kub,target_point = gf
            # self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
            self.target_point = self.kub.state.ballPos
            if not vicinity_points(self.target_point,target_point,thresh=BOT_RADIUS*2.0):
                self.behavior_failed = True
                break


    def on_exit_course_approach(self):
        time.sleep(5)
        print("waited for two seconds")
        pass

    def on_enter_fine_approach(self):
        #self.kub.dribble(True)
        theta = self.kub.get_pos().theta
        _GoToPoint_.init(self.kub, self.kub.state.ballPos, theta)
        pass

    def execute_fine_approach(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.ball_dist_thresh)

        for gf in generatingfunction:
            self.kub,ballPos = gf
            
            if not vicinity_points(ballPos,self.kub.state.ballPos,thresh=BOT_RADIUS):
                print(ballPos,self.kub.state.ballPos)
                print("failed fine approach")
                time.sleep(4)
                self.behavior_failed = True
                break

    def on_exit_fine_approach(self):
        time.sleep(1)
        print("waited for one second")
        pass

    def on_enter_dribble_move(self):
        self.kub.dribble(True)
        theta = self.kub.get_pos().theta
        self.target_point = self.kub.state.awayPos[4]
        _GoToPoint_.init(self.kub,self.target_point, theta)
        

    def execute_dribble_move(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.ball_dist_thresh)

        for gf in generatingfunction:
            print("HII")
            self.kub,ballPos = gf
            
            if not vicinity_points(ballPos,self.kub.state.awayPos[4],thresh=BOT_RADIUS):        #wrong as fuck
                self.behavior_failed = True
                print("I am a failure")
                break



    def on_exit_dribble_move(self):
        pass
        
    



        

        

               

        

        

        

        


          


