"""
Contains multiple versions of the consensus controller and several car following models as a baseline. 
"""
import math
import numpy as np

from flow.controllers.base_controller import BaseController

class BaselineController(BaseController):
    """A simple car-following model parameterized by the same IDM variables.
    Note that default arguments are overriden by experiment configuration.
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : sstr
        Vehicle ID for SUMO identification
    car_following_params : flow.core.param.SumoCarFollowingParams
        see parent class
    v0 : float
        desirable velocity, in m/s (default: 30)
    T : float
        safe time headway, in s (default: 1)
    a : float
        max acceleration, in m/s2 (default: 1)
    b : float
        comfortable deceleration, in m/s2 (default: 1.5)
    delta : float
        acceleration exponent (default: 4)
    s0 : float
        linear jam distance, in m (default: 2)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """
    def __init__(self,
                veh_id,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0

    def get_accel(self, env):
        """See parent class."""

        print('Max accel', self.a)
        print('Target velocity', self.v0)

        v = env.k.vehicle.get_speed(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)
        p = env.k.vehicle.get_position(self.veh_id)
        print('\n\n=============>')
        print('Self velocity', v)
        print('Self headway', h)
        print('Self position', p)
        
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        if lead_id != None:
            lv = env.k.vehicle.get_speed(lead_id)
            lh = env.k.vehicle.get_headway(lead_id)
            lp = env.k.vehicle.get_position(lead_id)  
            print('Lead velocity', lv)
            print('Lead headway', lh)
            print('Lead position', lp)
            distance = np.linalg.norm(lp - p)

        print('Max velocity ratio:', v/self.v0)
        print('Scaled mvr: ', self.a * (v/self.v0))

        desired_speed = (1-(v/self.v0))  
        
        if lead_id != None and distance < 40:
            slowing = (v-lv)
        else:
            slowing = 0

        myacc = self.a * (desired_speed + slowing)

        print('desired speed:', desired_speed)
        print('slowing', slowing)
        print('My acc:', myacc)

        print('===============>')
        return myacc

class ConsensusController(BaseController):
    """
    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.param.SumoCarFollowingParams
        see parent class
    v0 : float
        desirable velocity, in m/s (default: 30)
    a : float
        max acceleration, in m/s2 (default: 1)
    delta : float
        acceleration exponent (default: 4)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """
    def __init__(self,
                 veh_id,
                 v0=30,
                 c_headway=0,
                 c_velocity=0,
                 c_acceleration=0,
                 a=1,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.a = a
        self.delta = delta
        self.c_headway = c_headway
        self.c_velocity = c_velocity
        self.c_acceleration = c_acceleration

    def get_accel(self, env):
        """Employ consensus technique to output new acceleration command."""

        v = env.k.vehicle.get_speed(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)
        p = env.k.vehicle.get_position(self.veh_id)
        n_vehicles = len(env.k.vehicle.get_ids())
        print('\n\n==================>')
        print('Self id: ', self.veh_id)
        print('Self velocity', v)
        print('Self headway', h)
        print('Self position', p)
        print()
                
        head_sum = 0
        for cid in range(n_vehicles):
            c_head = env.k.vehicle.get_headway(f'idm_{cid}')
            head_sum += (h - c_head)
        
        vel_sum = 0
        for cid in range(n_vehicles):
            c_vel = env.k.vehicle.get_speed(f'idm_{cid}')
            vel_sum += (v - c_vel)
        
        acc_sum = 0
        pv = env.k.vehicle.get_previous_speed(self.veh_id)
        acc = (v - pv)/env.sim_step
        for cid in range(n_vehicles):
            c_vel = env.k.vehicle.get_speed(f'idm_{cid}')
            c_prev_vel = env.k.vehicle.get_previous_speed(f'idm_{cid}')
            c_acc = abs(c_vel - c_prev_vel) / env.sim_step
            acc_sum += (acc - c_acc) 

        # The own vehicle does not contribute to the average
        headway_term = self.c_headway*(head_sum/(n_vehicles-1))
        velocity_term = self.c_velocity*(vel_sum/(n_vehicles-1))
        acc_term = self.c_acceleration*(acc_sum/(n_vehicles-1))
        ###

        print(f'num vehicles: {n_vehicles}')
        print(f'headway term: {headway_term}')  
        print(f'velocity term: {velocity_term}')
        print(f'acceleration term:  {acc_term}')
        
        acc = self.a *((1 - (v / self.v0)**4) + headway_term - velocity_term - acc_term)
        
        print('acc out: ', acc)
        print('===========<<<')

        return acc

        
class ConsensusFailureController(BaseController):
    """
    Same design as the ConsensusCOntroller but with parameters for controlling failure conditions
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.param.SumoCarFollowingParams
        see parent class
    v0 : float
        desirable velocity, in m/s (default: 30)
    a : float
        max acceleration, in m/s2 (default: 1)
    delta : float
        acceleration exponent (default: 4)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """
    def __init__(self,
                 veh_id,
                 v0=30,
                 c_headway=0,
                 c_velocity=0,
                 c_acceleration=0,
                 a=1,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.a = a
        self.delta = delta
        self.c_headway = c_headway
        self.c_velocity = c_velocity
        self.c_acceleration = c_acceleration
        self.counter = 0
        self.update_interval = 1
        self.consensus_car_count = 20
        self.consensus_info = {}
    

    def get_vehicle_accel(self, env, veh_id):
        v = env.k.vehicle.get_speed(veh_id)
        pv = env.k.vehicle.get_previous_speed(veh_id)
        return (v - pv)/env.sim_step

    def get_accel(self, env):
        """Employ consensus technique to output new acceleration command."""

        v = env.k.vehicle.get_speed(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)
        p = env.k.vehicle.get_position(self.veh_id)
        p2d = env.k.vehicle.get_2d_position(self.veh_id)
        n_vehicles = len(env.k.vehicle.get_ids())

        print('\n\n==================>')
        print('Self id: ', self.veh_id)
        print('Self velocity', v)
        print('Self headway', h)
        print('Self counter:', self.counter)
        print()

        #Update consensus information every update interval
        if self.counter % self.update_interval == 0:
            other_ids = [v for v in env.k.vehicle.get_ids() if v != self.veh_id]
            other_positions = [env.k.vehicle.get_2d_position(v) for v in other_ids]
            other_distances = [np.linalg.norm(np.array(p2d) - np.array(op)) for op in other_positions]
            sorted_distances = sorted(zip(other_ids, other_distances), key=lambda tup: tup[1])

            consensus_info = [(   
                vinfo[0], 
                env.k.vehicle.get_headway(vinfo[0]),
                env.k.vehicle.get_speed(vinfo[0]),
                self.get_vehicle_accel(env, vinfo[0])
            ) for vinfo in sorted_distances[:self.consensus_car_count]]

            print(f'Counter: {self.counter}, Consensus info: ', consensus_info)
            self.consensus_info = consensus_info
        
        head_sum = 0
        vel_sum = 0
        acc_sum = 0
        a = self.get_vehicle_accel(env, self.veh_id)
        for (vid, c_head, c_vel, c_acc) in self.consensus_info:
            head_sum += (h-c_head)
            vel_sum += (v-c_vel)
            acc_sum += (a-c_acc)

        # Compute consensus terms
        headway_term = self.c_headway*(head_sum/len(self.consensus_info))
        velocity_term = self.c_velocity*(vel_sum/len(self.consensus_info))
        acc_term = self.c_acceleration*(acc_sum/len(self.consensus_info))

        print(f'headway term: {headway_term}')  
        print(f'velocity term: {velocity_term}')
        print(f'acceleration term:  {acc_term}')
        
        acc = self.a *((1 - (v / self.v0)**4) + headway_term - velocity_term - acc_term)
        
        print('acc out: ', acc)
        print('===========<<<')
        self.counter += 1
        return acc


class FigureEightController(BaseController):
    """
    Same design as the ConsensusCOntroller but with parameters for controlling failure conditions
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.param.SumoCarFollowingParams
        see parent class
    v0 : float
        desirable velocity, in m/s (default: 30)
    a : float
        max acceleration, in m/s2 (default: 1)
    delta : float
        acceleration exponent (default: 4)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """
    def __init__(self,
                 veh_id,
                 v0=30,
                 c_headway=0,
                 c_velocity=0,
                 c_acceleration=0,
                 a=1,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.a = a
        self.delta = delta
        self.c_headway = c_headway
        self.c_velocity = c_velocity
        self.c_acceleration = c_acceleration
        self.counter = 0
        self.update_interval = 1
        self.consensus_car_count = 15
        self.consensus_info = {}
    

    def get_vehicle_accel(self, env, veh_id):
        v = env.k.vehicle.get_speed(veh_id)
        pv = env.k.vehicle.get_previous_speed(veh_id)
        return (v - pv)/env.sim_step

    def get_accel(self, env):
        """Employ consensus technique to output new acceleration command."""

        v = env.k.vehicle.get_speed(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)
        p = env.k.vehicle.get_position(self.veh_id)
        p2d = env.k.vehicle.get_2d_position(self.veh_id)
        n_vehicles = len(env.k.vehicle.get_ids())

        print('\n\n==================>')
        print('Self id: ', self.veh_id)
        print('Self position: ', p)
        print('Self velocity', v)
        print('Self headway', h)
        print('Self counter:', self.counter)
        print()

        #Update consensus information every update interval
        if self.counter % self.update_interval == 0:
            other_ids = [v for v in env.k.vehicle.get_ids() if v != self.veh_id]
            other_positions = [env.k.vehicle.get_2d_position(v) for v in other_ids]
            other_distances = [np.linalg.norm(np.array(p2d) - np.array(op)) for op in other_positions]
            sorted_distances = sorted(zip(other_ids, other_distances), key=lambda tup: tup[1])

            consensus_info = [(   
                vinfo[0], 
                env.k.vehicle.get_headway(vinfo[0]),
                env.k.vehicle.get_speed(vinfo[0]),
                self.get_vehicle_accel(env, vinfo[0])
            ) for vinfo in sorted_distances[:self.consensus_car_count]]

            #print(f'Counter: {self.counter}, Consensus info: ', consensus_info)
            self.consensus_info = consensus_info
        
        head_sum = 0
        for (vid, c_head, c_vel, c_acc) in self.consensus_info:
            head_sum += (h-c_head)

        # Compute consensus terms
        headway_term = .006*(head_sum/len(self.consensus_info))
        print(f'headway term: {headway_term}')  

        carid = int(self.veh_id.split('_')[1])
        headway_term += -.005 if carid % 1 else +.005
        acc = self.a * ((1 - (v/self.v0)**4) + headway_term)

        print('acc out: ', acc)
        print('===========<<<')
        self.counter += 1
        return acc



class IDMController(BaseController):
    """Intelligent Driver Model (IDM) controller.

    For more information on this controller, see:
    Treiber, Martin, Ansgar Hennecke, and Dirk Helbing. "Congested traffic
    states in empirical observations and microscopic simulations." Physical
    review E 62.2 (2000): 1805.

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.param.SumoCarFollowingParams
        see parent class
    v0 : float
        desirable velocity, in m/s (default: 30)
    T : float
        safe time headway, in s (default: 1)
    a : float
        max acceleration, in m/s2 (default: 1)
    b : float
        comfortable deceleration, in m/s2 (default: 1.5)
    delta : float
        acceleration exponent (default: 4)
    s0 : float
        linear jam distance, in m (default: 2)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None,
                 display_warnings=True,
                 car_following_params=None):
        """Instantiate an IDM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise,
            display_warnings=display_warnings,
        )
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0

    def get_accel(self, env):
        """See parent class."""
        v = env.k.vehicle.get_speed(self.veh_id)
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)

        # in order to deal with ZeroDivisionError
        if abs(h) < 1e-3:
            h = 1e-3

        if lead_id is None or lead_id == '':  # no car ahead
            s_star = 0
        else:
            lead_vel = env.k.vehicle.get_speed(lead_id)
            s_star = self.s0 + max(
                0, v * self.T + v * (v - lead_vel) /
                (2 * np.sqrt(self.a * self.b)))

        return self.a * (1 - (v / self.v0)**self.delta - (s_star / h)**2)


class SimCarFollowingController(BaseController):
    """Controller whose actions are purely defined by the simulator.

    Note that methods for implementing noise and failsafes through
    BaseController, are not available here. However, similar methods are
    available through sumo when initializing the parameters of the vehicle.

    Usage: See BaseController for usage example.
    """

    def get_accel(self, env):
        """See parent class."""
        return None
