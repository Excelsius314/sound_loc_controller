import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
import math
import numpy as np
from sound_loc_controller.simplex import *
import threading

from epuck_driver_interfaces.action import SimpleMovement
from epuck_driver_interfaces.action import CollectSoundData
from rclpy.action import ActionClient

def vector_from_angle(ori):
     return np.array([math.cos(ori),math.sin(ori)])


def get_triangle_positions(element_distance):
        
        # Coordinates are relative anyway
        ori = 0.0
        center = np.array([0., 0.])

        directions = np.concatenate((vector_from_angle(ori).reshape(-1, 2),
                                     vector_from_angle(ori - 2*math.pi/3).reshape(-1, 2),
                                     vector_from_angle(ori + 2*math.pi/3).reshape(-1, 2)), axis=0)
        
        a = element_distance/(2 * math.sin(math.pi/3))
        return directions * a + center


def send_movement_goal(client, angle, distance):

    goal_msg = SimpleMovement.Goal()
    goal_msg.angle = angle
    goal_msg.distance = distance

    if not client.wait_for_server(5):
        raise Exception("Timeout on movement control action server")
    

    
    return client.send_goal_async(goal_msg)


class SoundLocController(Node):

    def __init__(self):
        super().__init__('TeamController')
        
        
        self.robot_ids = ["epuck"]
        self.robot_distance = 0.3  # in meters
        self.record_time = 3  # in seconds
        self.step_distance = 0.1  # in meter
        

        self.prev_mov_dir = None
        self.is_converged = False
        self.declare_parameter('average_mic_amplitudes_per_robot', True)

        self.movement_controller_action_clients = [ActionClient(self, SimpleMovement, "{}/movement_goal".format(id)) for id in self.robot_ids]
        self.get_audio_data_locations = get_triangle_positions if self.get_parameter('average_mic_amplitudes_per_robot') else self.get_individual_microphone_positions

        self.audio_data_collection_action_client = ActionClient(self, CollectSoundData, "sound_data_collection")



    def issue_triangle_translation(self, direction):

        d_theta = math.atan2(direction[1], direction[0])

        return [send_movement_goal(action_client, d_theta, self.step_distance) for action_client in self.movement_controller_action_clients]
 
    def init_audio_data_collection(self):

        goal_msg = CollectSoundData.Goal()
        goal_msg.recording_time = self.record_time
        return [self.audio_data_collection_action_client.send_goal_async(goal_msg)]



    # using averaging strategy for now
    def sound_localization(self, time_avg_dBs):

        
        #robot_avg_dBs = np.average(time_avg_dBs.reshape(-1, 3), axis=1)
        #robot_locs = get_triangle_positions(self.robot_distance)
        audio_locations = self.get_audio_data_locations(self.robot_distance)
        microphone_data = [{"pos":audio_locations[i], "dB":time_avg_dBs[i]} for i in range(len(time_avg_dBs))]

        separations = determine_sound_source_halfspaces(microphone_data, True)
        simplex = aggregate_separations_to_simplex(separations)
        mov_dir, sampled_outer_points = determine_direction(simplex, 
                                                            np.array([0., 0.]), 
                                                            self.robot_distance)
        # check if more than 90 degree direction change
        if self.prev_mov_dir is not None:
            if self.prev_mov_dir @ mov_dir / (np.linalg.norm(self.prev_mov_dir) * np.linalg.norm(self.prev_mov_dir)) < 0:
                # convergence
                self.is_converged = True
        self.prev_mov_dir = mov_dir

        return mov_dir
    
    
    
    def get_individual_microphone_positions(self, robot_distance):
        pass


    def wait_on_action_completion(self, goal_handle_futures):

        result_fts_and_g_handles = []
        for goal_handle_future in goal_handle_futures:
            rclpy.spin_until_future_complete(self, goal_handle_future)

            goal_handle = goal_handle_future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected :(')
            else:
                result_fts_and_g_handles.append((goal_handle.get_result_async(), goal_handle))

        results = []
        for result_future, g_handle in result_fts_and_g_handles:
            rclpy.spin_until_future_complete(self, result_future)

            results.append(result_future.result())

        return results



    def main_control_loop(self):

        # 1. Stage: Optimize Formation
        # 2. Stage: Do soundlocalizaton and check convergence
        # 3. Stage: translate Formation
        
                           #create_stage_executer(, self.formation_translation_done, rate)]
                    

        while not self.is_converged:
            
            print("Initiating Audio data collection")
            audio_data = self.wait_on_action_completion(self.init_audio_data_collection())

            print("Audio data collection done")
            print("Doing sound localization")
            mov_dir = self.sound_localization(audio_data)
            print("Issueing move command: {}".format(mov_dir))
            self.wait_on_action_completion(self.issue_triangle_translation(mov_dir))


             
             
def main(args=None):
    rclpy.init(args=args)

    team_controller = SoundLocController()

    #rclpy.spin(TeamController)

    #thread = threading.Thread(target=rclpy.spin, args=(team_controller, ), daemon=True)
    
    #thread.start()

    #rate = team_controller.create_rate(10)
    try:
        team_controller.main_control_loop()
    except KeyboardInterrupt:
        pass
   

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    team_controller.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
