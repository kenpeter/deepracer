

pure chase
--------------------
    def reward_function(self, on_track, x, y, distance_from_center, car_orientation, progress, steps,
                        throttle, steering, track_width, waypoints, closest_waypoints):
        
        # low reward
        reward = 1e-3
        
        # start: rabbit is there
        rabbit = [0,0]

        # start: pointing to rabbit
        pointing = [0,0]
            
        # Reward when yaw (car_orientation) is pointed to the next waypoint IN FRONT.
        
        # 1. closest_waypoints+1, next arr pair
        # 2. rabbit => [arr[0], arr[1]]
        rabbit = [waypoints[closest_waypoints+1][0],waypoints[closest_waypoints+1][1]]
        
        # e.g. 3^2 + 4^2 = 5^2 ==> 5
        radius = math.hypot(x - rabbit[0], y - rabbit[1])
        
        pointing[0] = x + (radius * math.cos(car_orientation))

        pointing[1] = y + (radius * math.sin(car_orientation))
        
        vector_delta = math.hypot(pointing[0] - rabbit[0], pointing[1] - rabbit[1])
        
        # Max distance for pointing away will be the radius * 2
        # Min distance means we are pointing directly at the next waypoint
        # We can setup a reward that is a ratio to this max.
        
        if vector_delta == 0:
            reward += 1
        else:
            reward += ( 1 - ( vector_delta / (radius * 2)))

        return reward



# self motivation
-----
def reward_function(params):
    # 1. stay on track
    # 2. steps is there
    # 3. progress precent * 100 + double the speed
    if params["all_wheels_on_track"] and params["steps"] > 0:
        reward = ((params["progress"] / params["steps"]) * 100) + (params["speed"]**2)
    else:
        # else no good
        reward = 0.01
        
    return float(reward)


# self motivation extreme
-----
def reward_function(params):
    # on track and only progress
    if params['all_wheels_on_track']:
        reward = params['progress']
    else:
        reward = 0.001

    return float(reward)



-----
import math
def reward_function(params):
    # track width
    track_width = params['track_width']

    # car to center
    distance_from_center = params['distance_from_center']

    # abs angle ++++
    steering = abs(params['steering_angle'])

    # angle - or +
    direction_stearing=params['steering_angle']

    # car speed
    speed = params['speed']

    # state
    steps = params['steps']

    # how much done a lap
    progress = params['progress']

    # all in track
    all_wheels_on_track = params['all_wheels_on_track']

    # only 15 steer ++++
    ABS_STEERING_THRESHOLD = 15

    # speed limit not use
    SPEED_TRESHOLD = 5

    # step limit no used
    TOTAL_NUM_STEPS = 85

    # all point list
    waypoints = params['waypoints']

    # prev, future index
    closest_waypoints = params['closest_waypoints']

    # car heading in degree
    heading = params['heading']

    # 1 reward
    reward = 1.0

    # done, reward 100
    if progress == 100:
        reward += 100

    # closest_waypoints[1] is future index
    # closest_waypoints[0] is prev index
    # waypoints[index] is the realy point
    next_point = waypoints[closest_waypoints[1]]

    # prev pt
    prev_point = waypoints[closest_waypoints[0]]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians

    # 1 is y, 0 is x
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])

    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)

    # Penalize the reward if the difference is too large
DIRECTION_THRESHOLD = 10.0
malus=1
if direction_diff > DIRECTION_THRESHOLD:
malus=1-(direction_diff/50)
if malus<0 or malus>1:
malus = 0
reward *= malus
return reward