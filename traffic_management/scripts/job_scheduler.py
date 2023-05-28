#! /usr/bin/env python3

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
import os

from transforms3d.euler import euler2quat
from random import randrange

from robot_navigator import BasicNavigator, NavigationResult


# # Shelf positions for picking
# shelf_positions = {
#     # "shelf_1": [[5.5, 8.0, 3.14], True],
#     # "shelf_2": [[8.0, 8.0, 3.14], True],
#     # "shelf_3": [[10.5, 8.0, 3.14], True],
#     # "shelf_4": [[17.5, 8.0, 3.14], True],
#     # "shelf_5": [[20.0, 8.0, 3.14], True],
#     # "shelf_6": [[22.5, 8.0, 3.14], True],
#     "shelf_7": [[5.5, 11.0, 0.0], True],
#     "shelf_8": [[8.0, 11.0, 0.0], True],
#     "shelf_9": [[10.5, 11.0, 0.0], True],
#     "shelf_10": [[17.5, 11.0, 0.0], True],
#     "shelf_12": [[20.0, 11.0, 0.0], True],
#     "shelf_12": [[22.5, 11.0, 0.0], True],
#     "shelf_13": [[5.5, 14.0, 3.14], True],
#     "shelf_14": [[8.0, 14.0, 3.14], True],
#     "shelf_15": [[10.5, 14.0, 3.14], True],
#     "shelf_16": [[17.5, 14.0, 3.14], True],
#     "shelf_17": [[20.0, 14.0, 3.14], True],
#     "shelf_18": [[22.5, 14.0, 3.14], True]
#     # "shelf_19": [[5.5, 17.0, 0.0], True],
#     # "shelf_20": [[8.0, 17.0, 0.0], True],
#     # "shelf_21": [[10.5, 17.0, 0.0], True],
#     # "shelf_22": [[17.5, 17.0, 0.0], True],
#     # "shelf_23": [[20.0, 17.0, 0.0], True],
#     # "shelf_24": [[22.5, 17.0, 0.0], True]
#     }

# Shelf positions for picking
# shelf_positions = {
#     "shelf_1": [[6.0, 10.0, 4.71], True],
#     "shelf_2": [[6.0, 12.0, 4.71], True],
#     "shelf_3": [[6.0, 14.0, 4.71], True],
#     "shelf_4": [[9.0, 10.0, 1.57], True],
#     "shelf_5": [[9.0, 12.0, 1.57], True],
#     "shelf_6": [[9.0, 14.0, 1.57], True]
#     }

shelf_positions = {
    "shelf_1": [[9.0, 12.0, 1.57], True],
    "shelf_2": [[9.0, 12.0, 1.57], True],
    "shelf_3": [[9.0, 12.0, 1.57], True]
    }

# Dropoff position for picked products
dropoff_positions = {
    "MIR_01": [[3.0, 4.0, 4.71]],
    "MIR_02": [[7.5, 4.0, 4.71]],
    "MIR_03": [[12.0, 4.0, 4.71]]
    }

def getNextPickingJob():
    key = ''
    value = ''

    while True:
        randomJob = randrange(0, len(shelf_positions.keys()))

        # The next job is randomly selected:
        key = list(shelf_positions.keys())[randomJob]
        value = shelf_positions[key]
        if value[1] == True:
            # We set the job as reserved:
            shelf_positions[key][1] = False
            break
    
    return (key, value)

def getDropoff(name):
    key = name
    value = dropoff_positions[name]
    
    return (key, value)

def createPoseFromJobProperties(navigator, jobProperties):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = jobProperties[0][0]
    pose.pose.position.y = jobProperties[0][1]
    orientation = euler2quat(0.0, 0.0, jobProperties[0][2], 'rxyz')
    pose.pose.orientation.z = orientation[3]
    pose.pose.orientation.w = orientation[0]
    print(pose)
    return pose

def timeToSeconds(time):
    sec, nsec = time.seconds_nanoseconds()
    return sec + nsec / 1000000000

def writeToFile(path, content):
    with open(path, 'a') as f:
        f.write(content)
    f.close()

robots = ['MIR_01', 'MIR_02', 'MIR_03']
robotInfo = dict()

def main():
    print("Starting job scheduler...")

    rclpy.init()

    file_path = os.path.abspath(__file__)
    base_dir = os.path.dirname(file_path)
    write_path = os.path.join(base_dir, 'results_' + str(randrange(9999)) + '.txt')

    for robot in robots:
        navigator = BasicNavigator("/" + robot)
        robotInfo[robot] = dict()
        robotInfo[robot]['name'] = robot
        robotInfo[robot]['navigator'] = navigator
        jobName, jobProperties = getNextPickingJob()
        jobPose = createPoseFromJobProperties(navigator, jobProperties)
        robotInfo[robot]['jobName'] = jobName
        robotInfo[robot]['jobProperties'] = jobProperties
        robotInfo[robot]['jobPose'] = jobPose
        robotInfo[robot]['jobType'] = 'picking'
        robotInfo[robot]['jobState'] = 'waiting'
        robotInfo[robot]['navigationState'] = ''
        robotInfo[robot]['started'] = False
        time.sleep(0.5)
        robotInfo[robot]['navigator'].publish_goal(robotInfo[robot]['name'], jobPose.pose.position.x, jobPose.pose.position.y)

        #for p in navigator.get_parameters(['use_sim_time']):
        #    print(p.get_parameter_value())
        
    while True:

        for robot in robots:

            # State 1: Selecting next job
            if robotInfo[robot]['jobName'] == '':
                if robotInfo[robot]['jobType'] == 'picking':
                    jobName, jobProperties = getNextPickingJob()
                    jobPose = createPoseFromJobProperties(navigator, jobProperties)
                    robotInfo[robot]['jobName'] = jobName
                    robotInfo[robot]['jobProperties'] = jobProperties
                    robotInfo[robot]['jobPose'] = jobPose
                    robotInfo[robot]['jobState'] = 'waiting'
                    robotInfo[robot]['navigator'].publish_goal(robotInfo[robot]['name'], jobPose.pose.position.x, jobPose.pose.position.y)

                elif robotInfo[robot]['jobType'] == 'dropoff':
                    jobName, jobProperties = getDropoff(robotInfo[robot]['name'])
                    jobPose = createPoseFromJobProperties(navigator, jobProperties)
                    robotInfo[robot]['jobName'] = jobName
                    robotInfo[robot]['jobProperties'] = jobProperties
                    robotInfo[robot]['jobPose'] = jobPose
                    robotInfo[robot]['jobState'] = 'waiting'
                    robotInfo[robot]['navigator'].publish_goal(robotInfo[robot]['name'], jobPose.pose.position.x, jobPose.pose.position.y)

            # State 2: Starting navigation
            if robotInfo[robot]['jobName'] != '' and robotInfo[robot]['navigator'].isNav2Active() and robotInfo[robot]['jobState'] == 'waiting':
                if robotInfo[robot]['started'] == True or robotInfo[robot]['navigator'].isNav2Active():
                    robotInfo[robot]['started'] = True
                    robotInfo[robot]['navigator'].info('STARTING: ' + robotInfo[robot]['jobType'] + ' to ' + robotInfo[robot]['jobName'])
                    robotInfo[robot]['navigator'].goToPose(robotInfo[robot]['jobPose'])
                    robotInfo[robot]['jobState'] = 'executing'
                    if robotInfo[robot]['jobType'] == 'picking':
                        robotInfo[robot]['jobStartTime'] = robotInfo[robot]['navigator'].get_clock().now()
                        robotInfo[robot]['navigator'].info('Job start: ' + str(timeToSeconds(robotInfo[robot]['jobStartTime'])))
        
            # Get current state: 
            if robotInfo[robot]['navigator'].isNavComplete() == True:
                robotInfo[robot]['navigationState'] = robotInfo[robot]['navigator'].getResult()
                robotInfo[robot]['navigator'].result_future = None

            # State 3: Check goal complete
            if robotInfo[robot]['jobName'] != '' and robotInfo[robot]['navigationState'] == NavigationResult.SUCCEEDED:
                robotInfo[robot]['navigator'].info('SUCCEEDED: ' + robotInfo[robot]['jobType'] + ' to ' + robotInfo[robot]['jobName'])
                if robotInfo[robot]['jobType'] == 'dropoff':
                    robotInfo[robot]['jobStopTime'] = robotInfo[robot]['navigator'].get_clock().now()
                    jobDuration = timeToSeconds(robotInfo[robot]['jobStopTime']) - timeToSeconds(robotInfo[robot]['jobStartTime'])
                    robotInfo[robot]['navigator'].info('Job end: ' +  str(timeToSeconds(robotInfo[robot]['jobStopTime'])))
                    robotInfo[robot]['navigator'].info('Job completed in ' + str(jobDuration))
                    writeToFile(write_path, robotInfo[robot]['name'] + ',' + str(timeToSeconds(robotInfo[robot]['jobStartTime'])) + ',' + str(timeToSeconds(robotInfo[robot]['jobStopTime'])) + ',' + str(jobDuration) + '\n')
                robotInfo[robot]['navigationState'] = NavigationResult.UNKNOWN

                if robotInfo[robot]['jobType'] == 'picking':
                    robotInfo[robot]['jobType'] = 'dropoff'
                    shelf_positions[robotInfo[robot]['jobName']][1] = True
                    robotInfo[robot]['jobName'] = ''
                    robotInfo[robot]['jobProperties'] = ''
                    robotInfo[robot]['jobPose'] = ''


                elif robotInfo[robot]['jobType'] == 'dropoff':
                    robotInfo[robot]['jobType'] = 'picking'
                    robotInfo[robot]['jobName'] = ''
                    robotInfo[robot]['jobProperties'] = ''
                    robotInfo[robot]['jobPose'] = ''

        time.sleep(1.0)

    """# Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    
    navigator.goToPose(pose)

    while True: 
        
        if (navigator.isNavComplete() == True):
            pose, key, destination = getNextJob(navigator)
            navigator.goToPose(pose)
        
        i = 0
        while not navigator.isNavComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival at ' + destination +
                    ' for worker: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

        result = navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Got product from ' + key)

        # elif result == NavigationResult.CANCELED:
        #     print('Task at ' + request_item_location + ' was canceled. Returning to staging point...')
        #     initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        #     navigator.goToPose(initial_pose)

        elif result == NavigationResult.FAILED:
            print('Task at ' + key + ' failed!')
            exit(-1)
    """


    exit(0)


if __name__ == '__main__':
    main()


