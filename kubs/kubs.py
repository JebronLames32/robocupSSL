import cmd_node
import rospy
from krssg_ssl_msgs.msg import BeliefState
from utils.functions import kub_has_ball

"""
Class for kubs
"""
class kubs:
    """
    Constructs the object(kub), initializing class attributes
    
    Parameters:
        self: object
        kubs_id(int): id if the kub(number)
        Bstate(function_object): State of the kub
        pub(function_object): the ros publisher (will be initialized in test_role.py)
    """
    
    def __init__(self, kubs_id, BState, pub):
        self.kubs_id = kubs_id
        #velocities
        self.vx = 0
        self.vy = 0
        self.vw = 0
        
        self.isteamyellow = False
        self.dribbler = False
        self.power = False
        # self.state = state
        # self.kubsBelief()
        self.pub = pub
        self.c=0
    
    """
    Updates the state and position of the object(kub)
    
    Parameters:
        self: object
        state(function_object): state of the kub
    """
    def update_state(self,state):
        self.state = state
        self.pos = state.homePos[self.kubs_id]

    """
    Reset the velocities, angle and power of the object(kub). Turns off the dribbler.
    Parameter: 
        self: object
    """
    def reset(self):
        self.dribbler = False
        self.vx = 0.0
        self.vy = 0.0
        self.vw = 0.0
        self.power = 0.0
    
    """
    Setting the 2D velocity of the object(kub)
    
    Parameters:
        self: object
        vx(float): X-component of velocity
        vy(float): Y-component of velocity
    """    
    def move(self, vx, vy):
        self.vx = vx
        self.vy = vy

    """
    Set the state of the dribbler
    
    Parameters:
        self: object
        dribbler(bool): A boolean value to control the dribbler of the kub
    """
    def dribble(self, dribbler):
        self.dribbler = dribbler


    """
    Set the angular velocity
    
    Parameters:
        self: object
        vw(float): angular velociry
    """
    def turn(self, vw):
        self.vw = vw

    """
    Set the power of kick
    
    Parameters:
        self: object
        power(float): power of kick
    """
    def kick(self, power):
        self.power = power



    """
    The real deal that publishes any kind of motion
    
    Parameters:
        self: object
    """
    def execute(self):
        cmd_node.send_command(self.pub, self.isteamyellow, self.kubs_id, self.vx, self.vy, self.vw, self.power, self.dribbler)  
        self.reset()

    """
    Get the State and ID of the kub which has the ball
    
    Parameters:
        self: object
        
    Returns:
        (function_object,int): The state and the id of the kub
    """
    def has_ball(self):
        return kub_has_ball(self.state,self.kubs_id)

    """
    Get the velocity of the kub
    
    Parameters:
        self: object
        
    Returns:
        (dictionary): {(float,float),float}: {X and Y component of velocities, Angular velocity}
    """
    def get_vel(self):
        velo = {
                'magnitute':magnitute(self.state.homeVel[self.kubs_id]),
                'direction':direction(self.state.homeVel[self.kubs_id])
                }
        return velo

    """
    Get the linear position of the kub
    
    Parameters:
        self: object
        
    Returns:
        (tuple): (float,float): (X and Y component of position)
    """
    def get_pos(self):
        return self.state.homePos[self.kubs_id]

    """
    Get the angular position of the kub
    
    Parameters:
        self: object
        
    Returns:
        (float): Angle theta of the kub
    """    
    def get_theta(self):
        return self.state.homePos[self.kubs_id].theta

    # def bs_callback(self, data):
    #     self.state.isteamyellow                 = data.isteamyellow
    #     self.state.frame_number                 = data.frame_number
    #     self.state.t_capture                    = data.t_capture
    #     self.state.ballPos                      = data.ballPos
    #     self.state.ballVel                      = data.ballVel
    #     self.state.awayPos                      = data.awayPos
    #     self.state.homePos                      = data.homePos
    #     self.state.awayVel                      = data.awayVel
    #     self.state.homeVel                      = data.homeVel
    #     self.state.ballDetected                 = data.ballDetected
    #     self.state.homeDetected                 = data.homeDetected
    #     self.state.awayDetected                 = data.awayDetected
    #     self.state.our_bot_closest_to_ball      = data.our_bot_closest_to_ball
    #     self.state.opp_bot_closest_to_ball      = data.opp_bot_closest_to_ball
    #     self.state.our_goalie                   = data.our_goalie
    #     self.state.opp_goalie                   = data.opp_goalie
    #     self.state.opp_bot_marking_our_attacker = data.opp_bot_marking_our_attacker
    #     self.state.ball_at_corners              = data.ball_at_corners
    #     self.state.ball_in_our_half             = data.ball_in_our_half
    #     self.state.ball_in_our_possession       = data.ball_in_our_possession

    #     self.isteamyellow = data.isteamyellow
    #     self.pos = data.homePos[self.kubs_id]
    #     # self.vx = data.homeVel[self.kubs_id].x
    #     # self.vy = data.homeVel[self.kubs_id].y
    #     self.c=self.c+1
    #     print(self.state.ballPos.x, self.state.ballPos.y)
    #     print("shubham   "+str(self.c))

    #     #print(str(self.state.ballPos.x))
    #     #if data.homeDetected[self.kubs_id] == True:
    #         #print("kubs_id " + str(self.kubs_id) + "Detected")
    #     #else:
    #         #print("kubs_id " + str(self.kubs_id) + "Not Detected")

    # def kubsBelief(self):
    #     #rospy.init_node('kubs_node', anonymous=False)
    #     rospy.Subscriber('/belief_state',BeliefState,self.bs_callback,queue_size=1000)
       # rospy.spin()
