import sys
import time
from helper import Helper
from robots import PioneerP3dx
import logging
import logger as lg
import csv


class Controller:
    """
    Controller managing connection to V-REP simulator, executing the physics loop etc
    """
    def __init__(self, world):
        lg.message(logging.INFO, 'Initialising ' + self.__class__.__name__)
        self.world_props = world['props']
        self.world_events = world['events']
        self.start_time = 0

        # Instantiate helper and connect to VREP
        self.h = Helper()
        if not self.h.connected:
            sys.exit()

        # Get all objects from running VREP scene
        self.full_scene_object_list, self.simple_object_list = self.h.get_objects()

        # Instantiate robot
        self.robot = PioneerP3dx(self.h, self.simple_object_list)

        self.loop()

        self.stats()

        # Disconnect from VREP
        self.h.disconnect_client()

    def loop(self):
        """
        Sequentially process planned events in a sense>think>act control cycle
        """
        lg.message(logging.INFO, 'Starting controller loop')

        self.start_time = time.time()

        stop_exceptions = ['wall_follow']
        step_status = {'start_t': time.time(),
                       'complete': None}

        for step in self.world_events:
            lg.message(logging.INFO, 'Starting event - ' + step['task'])

            task_args = None

            step_status['start_t'] = time.time()  # Log start, as some actions have lifetime defined in route plan
            step_status['complete'] = None  # Step tristate - not started (None), in progress (False), complete (True)
            while not step_status['complete']:
                # Sense
                self.sense()

                # Think
                # Possible impending collision - trigger stop unless excepted actions
                if self.robot.is_less_min_prox_dir_travel() and step['task'] not in stop_exceptions:
                    lg.message(logging.INFO, 'Stop task triggered')
                    self.robot.stop(step_status, self.world_props, task_args)
                    continue

                # Act
                if step_status['complete'] is None:
                    # Execute current route plan step
                    task_args = {arg: step[arg] for arg in step if arg not in 'task'}
                    lg.message(logging.DEBUG, 'Executing method ' + str(step['task']) + ' args ' + str(task_args))
                    getattr(self.robot, step['task'])(step_status, self.world_props, task_args)
                    continue

                getattr(self.robot, step['task'])(step_status, self.world_props, task_args)

                time.sleep(0.004)

    def sense(self):
        """
        Update state on how robot perceives world
        """
        self.robot.update_state_position()
        self.robot.update_state_proximity(self.world_props)
        self.robot.update_state_compass()
        self.robot.update_state_odometry()

    def stats(self):
        """
        Show simulation statistics
        """
        time_taken = round(time.time() - self.start_time, 2)

        avg_joint_dist = 0
        for m in self.robot.state['int']['motors']:
            avg_joint_dist += self.robot.state['int']['jpos'][str(self.robot.state['int']['motors'][m] + '_dist')]

        avg_joint_dist = round(avg_joint_dist / 2, 2)
        avg_speed_ms = round(avg_joint_dist / time_taken, 2)
        lg.message(logging.INFO, 'Distance travelled - {}m'.format(avg_joint_dist))
        lg.message(logging.INFO, 'Average speed - {}m/s'.format(avg_speed_ms))
        lg.message(logging.INFO, 'Nav dist diff error - {}cm'.format(round(self.robot.state['int']['err_corr_count'], 2)))
        lg.message(logging.INFO, 'Controller loop complete - time taken - {}s'.format(round(time.time() -
                                                                                            self.start_time, 2)))

        with open('output/abs_pos_all.csv', 'w', newline='') as out:
            csv_output = csv.writer(out)
            csv_output.writerows(self.robot.state['ext']['abs_pos_all'])

        with open('output/motor_all_v', 'w', newline='') as out:
            csv_output = csv.writer(out)
            csv_output.writerows(self.robot.state['int']['motor_all_v'])

