import logging
import logger as lg
import vrep
import math
import time
from sensors import ProximitySensor, Compass


class Robot:
    """
    Super class implementing robot
    """
    def __init__(self, name, simple_object_list, helper):
        self.h = helper

        # Robot objects
        self.name = name
        self.handle = self.h.get_object_handle_by_name(self.name)
        self.component_tree = self.set_object_tree()

        # Robot facing direction 1=forward, -1=back
        self.robot_dir_travel = 1

    def set_object_tree(self):
        """
        Get object tree for robot
        """
        self.h.get_object_tree(self.handle, 0)
        return self.h.object_tree

    def get_components(self, tag, simple_object_list):
        """
        Get component tree for vrep object
        """
        lg.message(logging.DEBUG, 'Getting V-REP components for ' + tag)
        components = {}
        for comp in self.component_tree:
            for child in self.component_tree[comp].children:
                if tag in simple_object_list[child]:
                    components[child] = simple_object_list[child]
        return components

    def set_joint_v(self, joint, velocity):
        """
        Set joint velocity
        """
        vrep.simxSetJointTargetVelocity(self.h.client_id, joint, velocity, vrep.simx_opmode_streaming)


class PioneerP3dx(Robot):
    """
    Pioneer P3DX robot implementation
    """
    def __init__(self, helper, simple_object_list):
        Robot.__init__(self, 'Pioneer_p3dx', simple_object_list, helper)
        lg.message(logging.INFO, 'Initialising ' + self.__class__.__name__)

        # Pioneer specific properties
        self.props = {'wheel_rad_m': 0.195,
                      'wheel_circ_m': 0.61261,
                      'chassis_l_mm': 455,
                      'chassis_w_mm': 381,
                      'chassis_h_mm': 237,
                      'weight_kg': 9,
                      'operating_payload_kg': 17,
                      'def_ms_v': 0.5,
                      'min_speed_ms': 0.12,
                      'max_speed_ms': 1.2,
                      'sensor_us_weights': [0.75 / 180.0 * math.pi, 0.55 / 180.0 * math.pi, 0.5 / 180.0 * math.pi,
                                            0.25 / 180.0 * math.pi, 0.25 / 180.0 * math.pi, 0.5 / 180.0 * math.pi,
                                            0.55 / 180.0 * math.pi, 0.75 / 180.0 * math.pi]}

        # Pioneer specific internal and external state
        self.state = {'int': {'motors': [],
                              'motor_l_v': 0.0,
                              'motor_r_v': 0.0,
                              'motor_all_v': [],
                              'jpos': {},
                              'prox_s_arr': [],
                              'prox_s': None,
                              'prox_is_less_min_dist_f': False,
                              'prox_is_less_min_dist_r': False,
                              'prox_min_dist_f': 0,
                              'prox_min_dist_r': 0,
                              'compass': None,
                              'err_corr_count': 0},
                      'ext': {'abs_pos_s': None,
                              'abs_pos_n': None,
                              'abs_pos_all': []}}

        # Initialise internal state for robot components
        self.state['int']['motors'] = self.get_components('Motor', simple_object_list)
        self.state['int']['prox_s_arr'] = self.get_components('ultrasonic', simple_object_list)
        self.state['int']['prox_s'] = ProximitySensor(helper.client_id, sensor_array=self.state['int']['prox_s_arr'])
        self.state['int']['compass'] = Compass(helper.client_id, handle=self.handle)
        self.set_joint_pos()

    def set_state_proximity(self, min_distance):
        """
        Boolean state indicating whether robot within specified minimum distance as detected front or rear array
        """
        self.state['int']['prox_min_dist_f'] = min(self.state['int']['prox_s'].last_read[0:8])
        self.state['int']['prox_min_dist_r'] = min(self.state['int']['prox_s'].last_read[8:16])

        if self.state['int']['prox_min_dist_f'] < min_distance:
            self.state['int']['prox_is_less_min_dist_f'] = True
        else:
            self.state['int']['prox_is_less_min_dist_f'] = False

        if self.state['int']['prox_min_dist_r'] < min_distance:
            self.state['int']['prox_is_less_min_dist_r'] = True
        else:
            self.state['int']['prox_is_less_min_dist_r'] = False

    def is_less_min_prox_dir_travel(self):
        """
        Returns boolean state indicating if robot proximity less than minimum distance for direction of travel
        """
        if self.robot_dir_travel == 1:
            return self.state['int']['prox_is_less_min_dist_f']
        else:
            return self.state['int']['prox_is_less_min_dist_r']

    def prox_dist_dir_travel(self):
        """
        Returns minimum distance between robot and collidable object for direction of travel
        """
        if self.robot_dir_travel:
            return self.state['int']['prox_min_dist_f']
        else:
            return self.state['int']['prox_min_dist_r']

    def update_state_proximity(self, props):
        """
        Read sensor array and update associated proximity states
        """
        self.state['int']['prox_s'].read(vrep.simx_opmode_buffer, sensor_array=self.state['int']['prox_s_arr'])
        if 'min_dist_enabled' in props:
            if props['min_dist_enabled']:
                self.set_state_proximity(props['min_dist'])

    def update_state_compass(self):
        """
        Get current object bearing and update state
        """
        self.state['int']['compass'].read(vrep.simx_opmode_buffer, handle=self.handle)

    def set_motor_v(self):
        """
        Set motor velocity for each motor joint after updating state with conversion m/s to rad/s
        """
        if self.state['int']['motor_l_v'] > self.props['max_speed_ms']:  # Cap velocity to top speed per data sheet
            self.state['int']['motor_l_v'] = self.props['max_speed_ms']
            lg.message(logging.WARNING, 'Max speed left motor limited to ' + str(self.props['max_speed_ms']))

        if self.state['int']['motor_r_v'] > self.props['max_speed_ms']:  # Cap velocity to top speed per data sheet
            self.state['int']['motor_r_v'] = self.props['max_speed_ms']
            lg.message(logging.WARNING, 'Max speed right motor limited to ' + str(self.props['max_speed_ms']))

        lg.message(logging.DEBUG, 'Setting motor left velocity (m/s) - ' + str(self.state['int']['motor_l_v']))
        lg.message(logging.DEBUG, 'Setting motor right velocity (m/s) - ' + str(self.state['int']['motor_r_v']))

        self.state['int']['motor_l_v'] = self.h.ms_to_radss(self.state['int']['motor_l_v'], self.props['wheel_rad_m'])
        self.state['int']['motor_r_v'] = self.h.ms_to_radss(self.state['int']['motor_r_v'], self.props['wheel_rad_m'])

        t = time.time()
        self.state['int']['motor_all_v'].append((t, self.state['int']['motor_l_v'], self.state['int']['motor_r_v']))

        for m in self.state['int']['motors']:
            if 'left' in self.state['int']['motors'][m]:
                self.set_joint_v(m, self.state['int']['motor_l_v'])
            else:
                self.set_joint_v(m, self.state['int']['motor_r_v'])

        lg.message(logging.DEBUG, 'Motor left velocity now set at (rad/s) - ' + str(self.state['int']['motor_l_v']))
        lg.message(logging.DEBUG, 'Motor right velocity now set at (rad/s) - ' + str(self.state['int']['motor_r_v']))

    def set_state_pos_start(self):
        """
        Set starting position as external, absolute position
        """
        res, self.state['ext']['abs_pos_s'] = vrep.simxGetObjectPosition(self.h.client_id, self.handle, -1,
                                                                         vrep.simx_opmode_buffer)
        lg.message(logging.DEBUG, 'Start point set ' + str(self.state['ext']['abs_pos_s']))

    def update_state_position(self):
        """
        Set current (now) position as external, absolute position
        """
        res, self.state['ext']['abs_pos_n'] = vrep.simxGetObjectPosition(self.h.client_id, self.handle, -1,
                                                                         vrep.simx_opmode_streaming)
        self.state['ext']['abs_pos_all'].append(self.state['ext']['abs_pos_n'][0:2])

    def stop(self, step_status, world_props, args):
        """
        Stop robot locomotion
        """
        self.state['int']['motor_l_v'] = 0
        self.state['int']['motor_r_v'] = 0
        self.set_motor_v()
        step_status['complete'] = True

    def move(self, step_status, world_props, args):
        """
        Move robot as locomotion task
        """
        if 'robot_dir_travel' in args:
            self.robot_dir_travel = args['robot_dir_travel']

        if step_status['complete'] is None:
            velocity = self.props['def_ms_v']  # Set default velocity

            # Direction travel - forward = 1, reverse = -1
            self.state['int']['motor_l_v'] = velocity * self.robot_dir_travel
            self.state['int']['motor_r_v'] = velocity * self.robot_dir_travel
            self.set_motor_v()
            step_status['complete'] = False

        # Velocity as a "dist" is applied for specified time. Continue moving for "dist" or stop if proximity detected
        if 'dist' in args:
            if (time.time() - step_status['start_t'] > args['dist']) or self.is_less_min_prox_dir_travel():
                step_status['complete'] = True
                lg.message(logging.INFO, 'Move event complete')

    def turn(self, step_status, world_props, args):
        """
        Turn robot task - calculate new bearing relative to current orientation and turn cw or ccw at constant speed
        """
        if step_status['complete'] is None:
            if 'degrees' in args:

                #
                self.state['int']['compass'].set_to_bearing(args['degrees'])
                lg.message(logging.DEBUG, 'Turn bearing from {} to {}'.format(self.state['int']['compass'].last_read,
                                                                              self.state['int']['compass'].to_bearing))

                # Turn cw or ccw at slow speed to prevent overshooting
                if args['degrees'] > 0:
                    self.state['int']['motor_l_v'] = -0.1
                    self.state['int']['motor_r_v'] = 0.1
                else:
                    self.state['int']['motor_l_v'] = 0.1
                    self.state['int']['motor_r_v'] = -0.1

                self.set_motor_v()
            step_status['complete'] = False

        # Subtract max from min between last compass state and target bearing. Turn threshold is 0.5 degrees
        radius_threshold = 0.5
        diff = max(self.state['int']['compass'].last_read, self.state['int']['compass'].to_bearing) - \
               min(self.state['int']['compass'].last_read, self.state['int']['compass'].to_bearing)
        if diff < radius_threshold:
            step_status['complete'] = True
            lg.message(logging.INFO, 'Turn event complete')

    def wall_follow(self, step_status, world_props, args):
        """
        Wall follow task
        """

        # Set default minimum distance
        min_dist = 0.22
        if 'min_dist' in world_props:
            min_dist = world_props['min_dist']

        max_dist = 0.25
        if 'max_dist' in world_props:
            max_dist = world_props['max_dist']

        coeff = 1
        if 'coeff' in args:
            coeff = args['coeff']

        if step_status['complete'] is None:
            self.set_state_pos_start()  # Set absolute position on start of wall follow task
            step_status['complete'] = False

        # After 5 seconds track euclidean distance between start point and current, to check if task completed (1 loop)
        if self.h.within_dist(self.state['ext']['abs_pos_s'], self.state['ext']['abs_pos_n']) and time.time() - \
                step_status['start_t'] > 10:
            step_status['complete'] = True
            lg.message(logging.INFO, 'Wall Follow event complete')

        # Set motor speed mapping index according to rotation (cw, ccw) and direction of travel (forward, reverse)
        if args['dir'] == 1 and self.robot_dir_travel == 1:
            motor_index = 0
        elif args['dir'] == 1 and self.robot_dir_travel == -1:
            motor_index = 1
        elif args['dir'] == -1 and self.robot_dir_travel == 1:
            motor_index = 1
        else:
            motor_index = 0

        # Use wall follow algorithm specified in scenario
        getattr(self, 'wall_follow_' + args['method'])(min_dist, max_dist, coeff, motor_index)

        # Set motor velocity from updated state
        self.set_motor_v()

    def wall_follow_min(self, min_dist, max_dist, coeff, motor_index):
        """
        This wall follow algorithm uses minimum sensor reading to determine left and right
        motor speeds to orientate and navigate the robot
        """
        distance = self.prox_dist_dir_travel()

        diff = 0
        if distance < min_dist:
            diff = min_dist - distance
        else:
            diff = distance - min_dist

        self.state['int']['err_corr_count'] += diff

        baseline_v = distance * coeff
        diff_map = [[math.sqrt(diff), math.sqrt(diff) * -1], [math.sqrt(diff) * -1, math.sqrt(diff)]]

        if distance < min_dist:
            self.state['int']['motor_l_v'] = baseline_v + diff_map[motor_index][0]
            self.state['int']['motor_r_v'] = baseline_v + diff_map[motor_index][1]
        else:
            self.state['int']['motor_l_v'] = baseline_v + diff_map[motor_index][1]
            self.state['int']['motor_r_v'] = baseline_v + diff_map[motor_index][0]

    def wall_follow_multi(self, min_dist, max_dist, coeff, motor_index):
        """
        This wall follow algorithm uses an approach where each sensor is weighted differently, thereby it's value input
        influencing it's contrubution to motor velocity differently
        """
        sensors = [s if s <= 1 else 1 for s in self.state['int']['prox_s'].last_read[0:8]]
        weighted_sensors_f = sum([s * w for s, w in zip(sensors, self.props['sensor_us_weights'][0:8])])

        distance = self.prox_dist_dir_travel()

        diff = 0
        if distance < min_dist:
            diff = min_dist - distance
        else:
            diff = distance - min_dist

        self.state['int']['err_corr_count'] += diff

        baseline_v = weighted_sensors_f * coeff
        diff_map = [[math.sqrt(diff), math.sqrt(diff) * -1], [math.sqrt(diff) * -1, math.sqrt(diff)]]

        if distance < min_dist:
            self.state['int']['motor_l_v'] = baseline_v + diff_map[motor_index][0]
            self.state['int']['motor_r_v'] = baseline_v + diff_map[motor_index][1]
        else:
            self.state['int']['motor_l_v'] = baseline_v + diff_map[motor_index][1]
            self.state['int']['motor_r_v'] = baseline_v + diff_map[motor_index][0]

    def set_joint_pos(self):
        """
        Set starting positions for revolute joints
        """
        for m in self.state['int']['motors']:
            res, current_pos = vrep.simxGetJointPosition(self.h.client_id, m, vrep.simx_opmode_streaming)
            self.state['int']['jpos'][str(self.state['int']['motors'][m] + '_prev')] = current_pos
            self.state['int']['jpos'][str(self.state['int']['motors'][m] + '_total')] = current_pos

    def update_state_odometry(self):
        """
        Update odometry state by reading the motor revolute joint positions
        """
        for m in self.state['int']['motors']:
            self.update_joint_pos(m)

    def update_joint_pos(self, handle):
        """
        Update motor revolute joint positions. Determine rotation amount since last read
        """

        res, current_pos = vrep.simxGetJointPosition(self.h.client_id, handle, vrep.simx_opmode_streaming)
        difference_pos = current_pos - self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_prev')]

        self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_prev')] = current_pos

        difference_pos = self.get_pos_ang(difference_pos)
        self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_total')] += difference_pos

        self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_turns')] = \
            self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_total')] / (math.pi * 2)

        self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_dist')] = \
            self.state['int']['jpos'][str(self.state['int']['motors'][handle] + '_turns')] * self.props['wheel_circ_m']

    @staticmethod
    def get_pos_ang(pos):
        """
        Get revolute position as turn angle
        """
        if pos >= 0:
            pos = math.fmod(pos + math.pi, 2 * math.pi) - math.pi
        else:
            pos = math.fmod(pos - math.pi, 2 * math.pi) + math.pi
        return pos
