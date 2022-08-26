from enum import Enum
import behavior
import _GoToPoint_
try:
    _GoToPoint_ = reload(_GoToPoint_)
except:
    import _GoToPoint_
import rospy
from utils.functions import *
from utils.config import *

class GoToPoint(behavior.Behavior):
    """
    Goes to the ball from current position

    Parameters:
    behavior.Behavior(object): Inheriting from Behaviour.py file


    """
    ##
    ## @brief      Class for state.
    ##
    class State(Enum):
        """
        Two states in the program are setup and drive

        """
        setup = 1 
        drive = 2


    def __init__(self,continuous=False):
        """
        Initialize the attributes of the class

        Paramaters: self: present in every class, continuous: 

        """
        # print "gtp"
        #GoToPoint.behavior.Behavior()
        #g = behavior.Behavior()
        #print "gtp2"
        super(GoToPoint,self).__init__()
        """
        Used to initialize superclass attributes

        Parameters: Current class name, object name

        """
        #self.state = state

        self.name = "GoToPoint"

        #setting this to false initially as we have not failed yet
        self.behavior_failed = False
        
        self.DISTANCE_THRESH = DISTANCE_THRESH

        """
        Define two new states in the fsm, drive and setup.

        Parameters: add_state(self, state, parent_state=None)
        """
        #Start, running, completed, failed, cancelled are the five states inhereted from behaviour.py

        #transition to go from setup state to running state
        self.add_state(GoToPoint.State.setup,
            behavior.Behavior.State.running)

        #transition to go from drive to running state
        self.add_state(GoToPoint.State.drive,
            behavior.Behavior.State.running)

        """
        Define transitions involving setup and drive here

        Parameters: add_transition(self, from_state, to_state, condition, event_name)
        """

        
        #transition to go from start to setup state
        self.add_transition(behavior.Behavior.State.start,
            GoToPoint.State.setup,lambda: True,'immediately')

        #transition to go from setup to drive state
        self.add_transition(GoToPoint.State.setup,
            GoToPoint.State.drive,lambda: self.target_present(),'setup')

        #self.add_transition(GoToPoint.State.drive,
        #    GoToPoint.State.drive,lambda: not self.at_new_point(),'restart')

        #transition to go from drive to completed state
        self.add_transition(GoToPoint.State.drive,
            behavior.Behavior.State.completed,lambda:self.at_new_point(),'complete')

        #transition to go from setup to failed state
        self.add_transition(GoToPoint.State.setup,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')

        #transition to go from drive to failed state
        self.add_transition(GoToPoint.State.drive,
            behavior.Behavior.State.failed,lambda: self.behavior_failed,'failed')

    """
    The add_point function is used to define a position

    It is being used here to define a point and optionally an angle if 
    we want to reset the angle value to some arbitrary value. Else, 
    we are going to set it to whatever it's previous theta was

    Parameters: 1: point(an object with x and y coordinates) 
                2: theta is the optional parameter we can specify (orientation angle)
    """
    def add_point(self,point,orient=None):
        self.target_point = point
        if orient:
            self.theta = orient
        else:
            self.theta = self.kub.get_pos().theta
        
    #defining a new cub using this function
    def add_kub(self,kub):
        self.kub = kub
        
    # function that returns our current target position
    def target_present(self):
        return self.target_point is not None


    def at_new_point(self):
        #print (dist(self.target_point,self.new_point),210)
        return dist(self.target_point,self.new_point) < self.DISTANCE_THRESH

        
    def on_enter_setup(self):
        pass
    def execute_setup(self):
        _GoToPoint_.init(self.kub,self.target_point,self.theta)
        pass
        
    def on_exit_setup(self):
        pass

    def on_enter_drive(self):
        pass

    def terminate(self):
        super().terminate()

    def execute_drive(self):
        print("Execute drive")
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
        _GoToPoint_.init(self.kub,self.target_point,self.theta)
        generatingfunction = _GoToPoint_.execute(start_time,self.DISTANCE_THRESH)
        print("Datatype of gf:",type(generatingfunction))
        for gf in generatingfunction:
            self.kub,target_point = gf

            # print self.behavior_failed
            if not vicinity_points(self.target_point,target_point):
                # print 
                # print  (self.target_point.x,self.target_point.y)
                # print  (target_point.x,target_point.y)
                # print 
                self.behavior_failed = True
                # print self.behavior_failed
                break
        self.new_point = self.kub.get_pos()
        

    
    def on_exit_drive(self):
        pass




