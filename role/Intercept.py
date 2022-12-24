from enum import Enum
import behavior
import rospy
import _GoOnArc_
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *

class Intercept(behavior.Behavior):

    class State(Enum):
        setup=1
        move_outside_circle=2
        move_inside_circle=1
        move_within_2alpha=3
        dribble_move=4
        kick=5

    def __init__(self):
        super(Intercept,self).__init__()

        self.name="Intercept"

        self.power = 7.0

        self.ball_dist_thresh = 1.3*BOT_BALL_THRESH

        self.behavior_failed = False

        for state in Intercept.State:
            self.add_state(state,behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            Intercept.State.setup,lambda:True,
            'immediately')

        self.add_transition(Intercept.State.setup,
            Intercept.State.move_outside_circle, lambda: not self.is_inside_circle() and not self.is_inside_2alpha(),
            'bot in circle')

        self.add_transition(Intercept.State.setup,
            Intercept.State.move_inside_circle, lambda: self.is_inside_circle() and not self.is_inside_2alpha(),
            'bot outside circle')

        self.add_transition(Intercept.State.setup,
            Intercept.State.move_within_2alpha, lambda: self.is_inside_2alpha(),
            'bot in 2alpha')

        self.add_transition(Intercept.State.move_inside_circle,
            Intercept.State.move_outside_circle, lambda: not self.is_inside_circle() and not self.is_inside_2alpha(),
            'bot went outside circle')

        self.add_transition(Intercept.State.move_outside_circle,
            Intercept.State.move_inside_circle, lambda: self.is_inside_circle() and not self.is_inside_2alpha(),
            'bot went inside circle')

        self.add_transition(Intercept.State.move_inside_circle,
            Intercept.State.move_within_2alpha, lambda: self.is_inside_2alpha(),
            'bot came into 2alpha')

        self.add_transition(Intercept.State.move_outside_circle,
            Intercept.State.move_within_2alpha, lambda: self.is_inside_2alpha(),
            'bot came into 2alpha')

        self.add_transition(Intercept.State.move_within_2alpha,
            Intercept.State.dribble_move, lambda: self.at_ball_pos() and self.ball_connected(),
            'bot reached ball')

    def is_inside_circle(self):
        pass

    def is_inside_2alpha(self):
        pass

    def at_ball_pos(self):
        return ball_in_front_of_bot(self.kub)

    def ball_connected(self):
        return  kub_has_ball(self.kub.state, self.kub.kubs_id)

    def add_kub(self,kub):
        self.kub = kub

    def add_theta(self,theta):
        self.theta = theta


