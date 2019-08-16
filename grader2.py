# Start Date - 20th June 2019
# @Author - Shankar Sridhar

# grading based on distances

import csv
from enum import Enum
from geopy.distance import geodesic
from datetime import datetime
import matplotlib.pyplot as plt
import math

log_files = [
    'log_LAPS_2019_07_14_15_35_54',
    'log_LAPS_2019_07_14_15_53_38',
    'log_LAPS_2019_07_14_16_10_42',
    'log_LAPS_2019_07_14_16_27_53',
    'log_LAPS_2019_07_14_16_45_15',
    'log_LAPS_2019_07_14_17_21_23'
]

time_idx = 1
rpm_idx = 6
speed_idx = 7
acc_idx = 8
lat_idx = 9
lon_idx = 10
dist_idx = 13
limits = [90.0, 150.0, 2005.0, 2100.0, 2317.0, 2427.0, 4300.0, 4350.0, 4745.0, 4805.0, 4841.0, 4975.0, 5038.0,
          5088.0, 9189.0, 9320.0, 10990.0, 11095.0, 11375.0, 11565.0, 111840.0, 11955.0, 12000.0, 12075.0]

# contains of tuple(starting, ending ) distance of a segment (stop sign with turns, stoplight with turns or roundabout)

segment_idx = 0

segment_limits = [
    (70, 150),
    (1800, 2200),
    (9100, 9350)
]

# segments sub-divided into zones. Distance in meters.
zone_limits = [
    [(70, 90), (91, 98), (99, 130), (131, 150)],  # south loop drive to airport road
    [(1800, 2015), (2016, 2085), (2086, 2200)] , # airport road - univ blvd stop light
    [(9100,9210), (9211,9250), (9251,9350)]
]

# speed limits for each zone in a segment. Speed in km/h
speed_limits = [
    (40.23, 40.23, 32.18, 72.41),  # (25, 25, 20, 45),  # segment 1
    (72.41, 72.41, 56.3),
    (56.3, 40.23, 72.41)# segment 2
]


# defines the states of driving
class State(Enum):
    stop = -1
    accelerating = 0
    decelerating = 1
    cruise = 2

    undefined = 3


# defines states of zone determined by distances
class Zone(Enum):
    Approaching = 0
    Special = 1
    Turn = 2
    Leaving = 3

    undefined = -1


# length of the turn in meters from its start point, turn_start[]
turn_dist = [
    25,  # S Loop Drive to Airport road - Stop sign
    70,  # Airport road to Univ Blvd - Stop light
    25,  # - Stop sign
    40  # newton drive Subway shop turn
]

# global variables
avg_speed = 0.0
cnt = 0.0
has_turn = False
has_stopSign = False
has_stopLight = False
complete_stop = False
bad_acc = 0
turn_entrySpeed = 0.0
turn_exitSpeed = 0.0
min_speed = 200
prev_dist = 0.0  # for noting distance from last GPS fix to target point
curr_dist = 0.0  # for noting distance from current GPS fix to target point
zone_dist = 0.0
total_event_dist = [0, 0, 0]  # to store total distance happened for an event in a segment
prev_gps_fix = (0, 0)
time_fmt = "%H:%M:%S.%f"  # to parse the timestamp from the log file
prev_time = 0
prev_state = State.undefined
state_now = State.undefined
current_segment = 0
current_zone = Zone.undefined  # current zone
prev_zone = Zone.undefined
speed_limit = 35
speed = 0
prev_speed = 0  # speed from previous second
jerk = 0
prev_jerk = 0
acc1 = 0.0
acc2 = 0.0

# Its a list of lists each containing tuples of (distance , excess acc, excess speed) pair
event_dist = [[], [], [], []]  # acceleration, deceleration, cruise and speed
turn_excess = []
thresh = [3, -3, 1, 2]  # acceleration/deceleration threshold levels for acc, dec, and cruise. jerk threshold for all.

# list of tuples containing (event, score, dist) for each type of event - acc, dec, cruise
segment_scores = [[], [], []]

segment_raw_data = []  # tuple of (dist(m), speed, acc, jerk)

# row params
acc = 0.0
row_speed = 0.0
lat = 0.0
lon = 0.0
dist = 0.0
row_time = " "

turn_start = [
    (42.000620, -93.633499),  # S Loop Drive to Airport road
    (42.001049, -93.610349),  # Airport road to Univ Blvd
    (42.005532, -93.586389),  # Stop sign to newton drive
    (42.009372, -93.586653)  # newton drive subway turn
]


# turn_start[2] = (42.005532,	-93.586389)
# turn_start[3] = (42.009372, -93.586653)
# turn_start[4] = (42.009566, -93.585268)
# turn_start[5] = (42.00838713, -93.5842956)
# turn_start[6] = (42.011378, -93.633668)
# turn_start[7] = (41.997891,	-93.639397)  # 1st roundabout start
# turn_start[8] = (41.993972,	-93.639316)
# turn_start[9] = (41.996908, -93.639255)
# turn_start[10] = (41.99747,	-93.637548)
#
# turn_end = []
# turn_end[0] = (42.00090357, -93.63290814)
# turn_end[1] = (42.001351, -93.609888)
# turn_end[2] = (42.005676, -93.586267)
# turn_end[3] = (42.009535, -93.586233)
# turn_end[4] = (42.008887, -93.584279)
# turn_end[5] = (42.008276, -93.584682)
# turn_end[6] = (42.011472, -93.634072)
# turn_end[7] = (41.996918, -93.639408)  # 1st roundabout end
# turn_end[8] = (41.993978, -93.639205)
# turn_end[9] = (41.997305, -93.638618)
# turn_end[10] = (41.997383, -93.637181)


# need for newton drive turns


def segment_report():
    global event_dist, thresh, speed_limits, current_segment, total_event_dist

    acc_score = 0
    dec_score = 0
    cruise_score = 0

    segment_score = 0

    if len(event_dist[State.accelerating.value]) is not 0:
        grade_event(State.accelerating)
        event_dist = [[], [], [], []]

    print("No.of accelerations: " + str(len(segment_scores[State.accelerating.value])))

    for event_score in segment_scores[State.accelerating.value]:
        # dist travelled in that event session / total distance of that event in this segment times the score for that session
        acc_score += (event_score[1] / total_event_dist[State.accelerating.value]) * event_score[0]

    print("Total acceleration score: " + str(acc_score) + "%")

    if len(event_dist[State.decelerating.value]) is not 0:
        grade_event(State.decelerating)
        event_dist = [[], [], [], []]

    print("No.of Decelerations: " + str(len(segment_scores[State.decelerating.value])))

    for event_score in segment_scores[State.decelerating.value]:
        # dist travelled in that event session / total distance of that event in this segment times the score for that session
        dec_score += (event_score[1] / total_event_dist[State.decelerating.value]) * event_score[0]

    print("Total Deceleration score: " + str(dec_score) + "%")

    if len(event_dist[State.cruise.value]) is not 0:
        grade_event(State.cruise)
        event_dist = [[], [], [], []]

    print("No.of cruise events: " + str(len(segment_scores[State.cruise.value])))

    for event_score in segment_scores[State.cruise.value]:
        cruise_score += (event_score[1] / total_event_dist[State.cruise.value] * event_score[0])

    print("Total cruising score: " + str(cruise_score) + "%")

    total_segment_distance = total_event_dist[0] + total_event_dist[1] + total_event_dist[2]

    segment_score += (total_event_dist[State.accelerating.value] / total_segment_distance) * acc_score
    segment_score += (total_event_dist[State.decelerating.value] / total_segment_distance) * dec_score
    segment_score += (total_event_dist[State.cruise.value] / total_segment_distance) * cruise_score

    print("Total segment " + str(segment_idx) + 'score: ' + str(segment_score))


def turn_report():
    global turn_exitSpeed, turn_entrySpeed, turn_excess, avg_speed

    total_speed = 0

    for t in turn_excess:  # t is a (dist,speed) tuple
        total_speed += t[1]

    avg_speed = total_speed / len(turn_excess)
    print("Average speed: " + str(avg_speed) + ' km/h')

    print("Turn entry speed: " + str(turn_entrySpeed) + ' km/h')

    print("Turn exit speed: " + str(turn_exitSpeed) + ' km/h')

    # print('Maximum turn speed reached:' + str(last_speed) + ' km/h')

    # print("No.of hard accelerations: " + str(bad_acc))
    # bad_acc = 0

    return


# converting required data from string to float
def update_params(row):
    global acc, row_speed, speed, lat, lon, dist
    global row_time, current_dist

    acc = float(row[acc_idx])
    row_speed = float(row[speed_idx])
    lat = float(row[lat_idx])
    lon = float(row[lon_idx])
    dist = float(row[dist_idx])
    row_time = row[time_idx]

    speed = row_speed
    current_dist = dist
    return


def gps_distance(zone_point):
    global lat, lon

    current_point = (lat, lon)
    # zone_point = turn_start[1]

    return geodesic(current_point, zone_point).meters


def is_point_crossed():
    global current_dist, prev_dist

    if current_dist - prev_dist <= 1.0:
        return False
    else:
        return True


# returns True if duration of a second has passed
def is_time():
    global row_time, prev_time

    current_time = datetime.strptime(row_time, time_fmt)
    time_diff = current_time - prev_time

    if time_diff.seconds == 1:
        prev_time = datetime.strptime(row_time, time_fmt)
        return True
    else:
        return False


# called every one second to calculate acc and jerk and update the global variables
def update_acc():
    global acc1, acc2, jerk, prev_jerk, speed, prev_speed, prev_state, state_now, is_first

    # convert speed from row in km/h to m/s for calculating acc and jerk

    # save prev acc of prev second
    acc1 = acc2

    # update acc if only there is valid speed
    acc2 = (0, speed - prev_speed)[prev_speed > 3]

    # save speed of current instance
    # prev_speed = speed already done in reset params

    # calc jerk from last two accelerations
    prev_jerk = jerk

    # update jerk if only there is valid acceleration (i.e. non-zero acceleration)
    jerk = (0, acc2 - acc1)[acc1 != 0]

    print(acc2, jerk, speed)


# identifies different event states. Called every second of update
def identify_state():
    # acc2 contains the latest acceleration updated at the rate of 1 second
    global acc2

    # sure acceleration
    if acc2 > 2:
        return State.accelerating
    # minute accelerations at higher speeds can be considered for cruise, but if
    # noted from a stopped position indicates acceleration
    if 0 < acc2 <= 2:
        if prev_state == State.stop:
            return State.accelerating
        else:
            return State.cruise
    # zero acc at higher speeds indicates cruising, but at lower speeds could be Stop
    if acc2 == 0:
        if speed > 5:
            return State.cruise
        else:
            return State.stop
    # if acc <0, might be decelerating at higher speeds or coming to a stop at lower speeds.
    if acc2 < 0:
        if speed <= 3:
            return State.stop
        else:
            if -2 <= acc2 < 0:
                return State.cruise
            else:
                return State.decelerating


# based on thresh[] for acceleration/deceleration thresholds, add excess acc/dec to event_dist list
# also adds speed exceed based on speed_limit variable.
def calc_excess():
    global event_dist, state_now, speed_limit, jerk, current_zone

    acc_thresh = 0

    if state_now != State.stop:
        delta_dist = current_dist - prev_dist
        if state_now is not State.cruise:
            acc_thresh = get_threshold()
        else:
            acc_thresh = thresh[State.cruise.value]
        excess_acc = abs(acc2) - abs(acc_thresh)
        excess_speed = speed - speed_limit  # both are in km/h

        # tuple of dist , excess acc, jerk and excess speed
        event_dist[state_now.value].append(
            (delta_dist,
             (0, excess_acc)[excess_acc > 0],
             jerk,
             (0, excess_speed)[excess_speed > 0],
             current_zone.value
             )
        )

        if current_zone is Zone.Turn:
            turn_excess.append(
                (delta_dist, speed)
            )

        # if speed > speed_limit:
        #   event_dist[3].append((delta_dist, speed - speed_limit))


def grade_event(event):
    global event_dist, thresh, speed_limits, current_segment, complete_stop, min_speed
    global speed, prev_speed, acc1, acc2, total_event_dist

    if (event is State.accelerating) or (event is State.decelerating) or (event is State.cruise):
        acc_dist = 0
        dec_dist = 0
        cruise_dist = 0
        acc_penalty = 0
        jerk_penalty = 0
        acc_score = 0
        jerk_score = 0
        speed_limit_score = 0
        total_jerk = 0
        # total distance for a event per zone
        dist_per_zone = [0, 0, 0, 0]  # per event
        speed_penalty_per_zone = [0, 0, 0, 0]

        # for action in segment_dist:

        for a in event_dist[event.value]:  # acceleration
            acc_dist += a[0]  # gives total distance accelerated in this event
            # if exceeded the threshold, calculate penalty
            # dist accelerated * excess accelerated * 0.28 for m/s^2 conversion
            acc_penalty += (0, a[0] * a[1] * 0.28)[a[1] > 0]
            excess_jerk = abs(a[2]) - thresh[3]  # 3rd parameter is the jerk
            jerk_penalty += (0, a[0] * excess_jerk * 0.28)[excess_jerk > 0]
            total_jerk += abs(a[2])
            # 1st element is the distance covered over last second. 5th element is that zone's value
            dist_per_zone[a[4]] += a[0]
            speed_penalty_per_zone[a[4]] += a[0] * a[3] * 0.28  # dist *excess speed saved for each zone num(a[4))

        print("Total distance " + event.name + ': ' + str(acc_dist) + ' m')
        total_event_dist[event.value] += acc_dist

        if acc_dist is not 0:
            acc_score = (1 - abs((acc_penalty / (acc_dist * thresh[event.value] * 0.28)))) * 100
            jerk_score = (1 - abs((jerk_penalty / (acc_dist * thresh[3] * 0.28)))) * 100
            # segment_scores[event.value].append((acc_score, acc_dist))

            print(event.name + ' event Score:' + str(acc_score) + ' %')
            print("Jerk score: " + str(jerk_score) + ' %')

        print("Average jerk: " + str(total_jerk / len(event_dist[event.value])))

        # Speed limit score calculation

        speed_score_perZone = [0, 0, 0, 0]

        for n in range(0, len(speed_limits[current_segment])):
            if dist_per_zone[n] is not 0:
                speed_score_perZone[n] = \
                    1 - (speed_penalty_per_zone[n]) / (dist_per_zone[n] * speed_limits[current_segment][n] * 0.28)

                speed_limit_score += (dist_per_zone[n] / acc_dist) * (speed_score_perZone[n])
            else:
                speed_score_perZone[n] = 0

            print(
                "Zone " + str(n + 1) + ": " + str(speed_score_perZone[n] * 100) + "% for " + str(
                    dist_per_zone[n]) + 'm')

        speed_limit_score *= 100  # in terms of percentage
        print("Total speed limit score: " + str(speed_limit_score) + "%")

        if event is State.cruise:
            total_event_score = 0.5 * jerk_score + 0.5 * speed_limit_score
        else:
            total_event_score = 0.33 * acc_score + 0.33 * jerk_score + 0.33 * speed_limit_score

        print("Complete " + event.name + " Score: " + str(total_event_score))
        segment_scores[event.value].append((acc_dist, total_event_score))

    if event is State.stop:
        if prev_zone is Zone.Special and has_stopSign:
            if prev_speed <= 3 and acc1 == 0 and prev_jerk == 0:
                complete_stop = True
                print("Complete stop detected!")
            else:
                min_speed = (min_speed, speed)[speed < min_speed]

    # if event is State.cruise:
    #     #
    #     print("Total distance " + event.name + ': ' + str(acc_dist) + ' m')
    #     # print("Average jerk: " + str(total_jerk / len(event_dist[event.value])))


# identifies zone based on distance. Sets has_turn flag and also the speed limit for the zone
def identify_zone():
    global has_turn, has_stopLight, has_stopSign, current_zone, speed_limit, current_segment

    if segment_limits[0][0] <= dist <= segment_limits[0][1]:  # segment number, start/end
        has_turn = True  # indicates that this segment has a turn
        has_stopSign = True
        current_segment = 0
        if zone_limits[0][0][0] <= dist <= zone_limits[0][0][1]:  # segment number,zone number ,start/end limit
            # update_zone(Zone.Approaching)
            current_zone = Zone.Approaching
            speed_limit = speed_limits[0][0]  # segment number,zone number
            return
        if zone_limits[0][1][0] <= dist <= zone_limits[0][1][1]:
            # update_zone(Zone.Special)
            current_zone = Zone.Special
            speed_limit = speed_limits[0][1]
            return
        if zone_limits[0][2][0] <= dist <= zone_limits[0][2][1]:
            # update_zone(Zone.Turn)
            current_zone = Zone.Turn
            speed_limit = speed_limits[0][2]
            return
        if zone_limits[0][3][0] <= dist <= zone_limits[0][3][1]:
            # update_zone(Zone.Leaving)
            current_zone = Zone.Leaving
            speed_limit = speed_limits[0][3]
            return

    if segment_limits[1][0] <= dist <= segment_limits[1][1]:
        has_turn = True
        has_stopLight = True
        current_segment = 1

        if zone_limits[1][0][0] <= dist <= zone_limits[1][0][1]:
            # update_zone(Zone.Approaching)
            current_zone = Zone.Approaching
            speed_limit = speed_limits[1][0]
            return
        if zone_limits[1][1][0] <= dist <= zone_limits[1][1][1]:
            # update_zone(Zone.Special)
            current_zone = Zone.Turn
            speed_limit = speed_limits[1][1]
            return
        if zone_limits[1][2][0] <= dist <= zone_limits[1][2][1]:
            # update_zone(Zone.Leaving)
            current_zone = Zone.Leaving
            speed_limit = speed_limits[1][2]
            return

    if segment_limits[2][0] <= dist <= segment_limits[2][1]:
        has_turn = True
        has_stopLight = True
        current_segment = 2

        if zone_limits[2][0][0] <= dist <= zone_limits[2][0][1]:
            # update_zone(Zone.Approaching)
            current_zone = Zone.Approaching
            speed_limit = speed_limits[2][0]
            return
        if zone_limits[2][1][0] <= dist <= zone_limits[2][1][1]:
            # update_zone(Zone.Special)
            current_zone = Zone.Turn
            speed_limit = speed_limits[2][1]
            return
        if zone_limits[2][2][0] <= dist <= zone_limits[2][2][1]:
            # update_zone(Zone.Leaving)
            current_zone = Zone.Leaving
            speed_limit = speed_limits[2][2]
            return


# saves the previous zone info and updates the current zone
def update_zone(zone_now):
    global prev_zone, current_zone

    prev_zone = current_zone
    current_zone = zone_now


# called after every segment is finished
def reset_params():
    global acc1, acc2, jerk, speed, prev_speed, prev_time, current_dist, prev_dist, zone_dist, event_dist
    global current_zone, prev_zone, row_speed, avg_speed, prev_state
    global has_stopLight, has_stopSign, has_turn, complete_stop, turn_entrySpeed, turn_exitSpeed
    global segment_scores, total_event_dist, segment_raw_data

    acc1 = 0
    acc2 = 0
    jerk = 0
    prev_speed = speed
    avg_speed = 0
    turn_entrySpeed = 0
    turn_exitSpeed = 0
    prev_time = datetime.strptime(row_time, time_fmt)

    event_dist = [[], [], [], []]
    segment_scores = [[], [], []]
    total_event_dist = [0, 0, 0]
    segment_raw_data = []

    prev_dist = dist
    zone_dist = 0.0
    prev_zone = Zone.undefined
    prev_state = State.undefined

    has_stopSign = False
    has_turn = False
    has_stopLight = False
    complete_stop = False


def save_data():
    global dist, speed, acc2, jerk, current_zone, segment_idx

    segment_raw_data.append((dist, speed, acc2, jerk, get_threshold(), speed_limits[segment_idx][current_zone.value]
                             , thresh[3], turn_regression(speed), normal_regression(speed)))


def plot_raw():
    global segment_raw_data, speed_limits, thresh

    dist_axis = []
    speed_axis = []
    acc_axis = []
    jerk_axis = []
    thresh_axis = []
    speed_limit_axis = []
    jerk_thresh_axis = []
    turn_thresh_axis = []
    normal_thresh_axis = []
    # convert each param in tuple to a list and units of m/s
    for s in segment_raw_data:
        dist_axis.append(s[0])
        speed_axis.append(s[1] * 0.28)
        acc_axis.append((s[2]) * 0.28)
        jerk_axis.append((s[3] * 0.28))
        thresh_axis.append(s[4]*0.28)
        speed_limit_axis.append((s[5]*0.28))
        jerk_thresh_axis.append((s[6]*0.28))
        turn_thresh_axis.append((s[7]*0.28))
        normal_thresh_axis.append((s[8]*0.28))

    plt.plot(dist_axis, speed_axis, label='Speed(m/s)', color='b', marker='.')
    plt.plot(dist_axis, acc_axis, label='Acceleration(m/s^2)', color='k', marker='.')
    plt.plot(dist_axis, jerk_axis, label='Jerk(m/s^3)', color='g', marker='.')

    plt.plot(dist_axis, thresh_axis, label='Threshold(m/s^2)', lw=1.0, color='r', linestyle='-.')
    plt.plot(dist_axis, speed_limit_axis, label = 'Speed Limit(m/s)', lw=1.0, color = 'r', linestyle = '--')
    plt.plot(dist_axis, jerk_thresh_axis, label = 'Jerk threshold(m/s^3)', lw=1.0, color='r', linestyle = ':')

    plt.xlabel("Distance(m)")
    plt.legend()
    plt.show()

    plt.figure()
    plt.plot(dist_axis, acc_axis, label='Acceleration(m/s^2)', color='k', marker='.')
    plt.plot(dist_axis, turn_thresh_axis, label='Turn threshold(m/s^2)', lw=1.0, color='r', linestyle = ':')
    plt.plot(dist_axis, thresh_axis, label='Threshold(m/s^2)', lw=1.0, color='g', linestyle='-.')
    plt.plot(dist_axis, normal_thresh_axis, label='Normal(m/s^2)', lw=1.0, color='r', linestyle=':')

    plt.xlabel("Distance(m)")
    plt.legend()
    plt.show()


def turn_regression(speed):

    turn_thresh =  7.7871 * pow(math.e, speed * (-0.051))
    return turn_thresh

def normal_regression(speed):
    normal_thresh = 14.477 * pow(math.e, speed * (-0.034))
    return normal_thresh

def get_threshold():
    global current_zone, speed

    if current_zone == Zone.Turn:
        #acc_thresh = 6.1875 * pow(math.e, speed * (-0.043))
        acc_thresh = 7.7871 * pow(math.e, speed * (-0.051))
        return acc_thresh

    else:
        acc_thresh = 14.477 * pow(math.e, speed * (-0.034))
        return acc_thresh


# with open('C:\\Users\\ssridhar\\Documents\\logs\\log_LAPS_2019_07_14_16_10_42.csv') as file:
with open('C:\\Users\\DELL\\Documents\\Smart Car\\final driving files\\' + log_files[1] + '.csv') as file:
    reader = csv.reader(file, delimiter=',')

    for row in reader:

        # wait till reaching start of segment 1
        while dist < segment_limits[segment_idx][0]:
            row = next(reader)
            update_params(row)

        print("Inside segment " + str(segment_idx))
        # sets the current speed from row
        reset_params()

        print(acc2, jerk, speed)
        #save_data()

        # while zone_dist <= turn_dist[0]:
        while dist <= segment_limits[segment_idx][1]:
            # keep moving in csv rows for 1 second
            while is_time() is False:
                row = next(reader)
                update_params(row)

            # use the above updated params to calc. acc and jerk
            update_acc()

            # identifies the current state based on the acc/speed values calculated for the last second
            state_now = identify_state()
            print(state_now)

            # identifies and sets segment num, zone inside this segment and the sets the speed limit for it.
            identify_zone()

            if state_now is not prev_state and prev_state is not State.undefined:
                grade_event(prev_state)
                event_dist = [[], [], [], []]

            # calculates excess of acceleration/deceleration values from its threshold and the distance covered
            # during it
            calc_excess()

            if current_zone is Zone.Special and has_stopSign:
                if state_now is State.stop:
                    if speed <= 3 and acc2 == 0 and jerk == 0:
                        complete_stop = True
                    else:
                        min_speed = (min_speed, speed)[speed < min_speed]

            elif current_zone is Zone.Turn:
                if prev_zone is not Zone.Turn:
                    print("Inside turn")
                    turn_entrySpeed = speed

                avg_speed += speed

            elif current_zone is Zone.Leaving and prev_zone is Zone.Turn:
                print("Exited turn")
                turn_exitSpeed = speed

            # saves dist, speed, acc and jerk in a tuple to a list for plotting.
            save_data()
            # row = next(reader)
            # update_params(row)
            prev_speed = speed
            prev_state = state_now
            prev_dist = current_dist
            current_dist = dist
            prev_zone = current_zone
            zone_dist += (current_dist - prev_dist)

        print('End of segment')

        print("Segment report:")
        segment_report()
        plot_raw()
        if has_turn is True:
            print("Turn report:")
            turn_report()
        segment_idx += 1

        # reset_params()

        # # wait till reaching segment 2
        # while dist < segment_limits[1][0]:
        #     row = next(reader)
        #     update_params(row)
        #
        # # Turn 2 - Stop Light - Left turn
        # # if (dist >= limits[2]) and (dist <= limits[3]):
        # print("Inside Turn 2")
        #
        # reset_params()
        #
        # while dist <= segment_limits[1][1]:
        #     # keep moving in csv rows for 1 second
        #     while is_time() is False:
        #         update_params(row)
        #         row = next(reader)
        #
        #     # after one second
        #     update_acc()
        #
        #     state_now = identify_state()
        #
        #     calc_excess()
        #
        #     print(state_now)
        #
        #     row = next(reader)
        #     update_params(row)
        #     prev_dist = current_dist
        #     current_dist = dist
        #     zone_dist += (current_dist - prev_dist)
        #
        # print("End of turn 2")
        #
        # segment_report()
        # # check = False
        # reset_params()
        #
        # # wait till reaching turn 3
        # while True:
        #     if lat != 0 and lon != 0:
        #         current_dist = gps_distance(turn_start[2])
        #         if current_dist < 5:
        #             break
        #     row = next(reader)
        #     update_params(row)
        #
        # print("Entering turn 3 zone - Stop sign left from highway")
        # reset_params()
        #
        # while zone_dist <= turn_dist[2]:
        #     # keep moving in csv rows for 1 second
        #     while is_time() is False:
        #         update_params(row)
        #         row = next(reader)
        #
        #     # after one second
        #     update_acc()
        #
        #     state_now = identify_state()
        #
        #     calc_excess()
        #
        #     print(state_now)
        #
        #     row = next(reader)
        #     update_params(row)
        #     prev_dist = current_dist
        #     current_dist = dist
        #     zone_dist += (current_dist - prev_dist)
        #
        # print('End of turn 3')
        #
        # segment_report()
        #
        # # Turn 4 - Kum n go Free Right turn
        #
        # # wait till reaching turn 4
        # while True:
        #     if lat != 0 and lon != 0:
        #         current_dist = gps_distance(turn_start[3])
        #         if current_dist < 5:
        #             break
        #     row = next(reader)
        #     update_params(row)
        #
        # print("Entering Turn 4 ")
        #
        # reset_params()
        #
        # while zone_dist <= turn_dist[3]:
        #     # keep moving in csv rows for 1 second
        #     while is_time() is False:
        #         update_params(row)
        #         row = next(reader)
        #
        #     # after one second
        #     update_acc()
        #
        #     state_now = identify_state()
        #
        #     print(state_now)
        #
        #     row = next(reader)
        #     update_params(row)
        #     prev_dist = current_dist
        #     current_dist = dist
        #     zone_dist += (current_dist - prev_dist)
        #
        # print('End of turn 4')
        #
        # if check:
        #     print("Turn 4 report:")
        #     segment_report()
        #     reset_params()
        #
        # # wait till reaching turn 5 - dairy queen curve
        # while True:
        #     if lat != 0 and lon != 0:
        #         current_dist = gps_distance(turn_start[4])
        #         if current_dist < 5:
        #             break
        #     row = next(reader)
        #     update_params(row)
        #
        # print("Entering Turn 5 ")
        # turn_entrySpeed = speed
        #
        # current_dist = gps_distance(turn_end[4])
        # prev_dist = current_dist
        # check = True
        #
        # while True:
        #
        #     if acc < -30.0 or acc > 30.0:
        #         bad_acc += 1
        #
        #     avg_speed += speed
        #     cnt += 1.0
        #
        #     if lat == 0 and lon == 0:
        #         row = next(reader)
        #         update_params(row)
        #         continue
        #     else:
        #         prev_dist = current_dist
        #         current_dist = gps_distance(turn_end[4])
        #         if (round(gps_distance(turn_end[4])) <= 5.0) or is_point_crossed() is True:
        #             turn_exitSpeed = speed
        #             break
        #         else:
        #             row = next(reader)
        #             update_params(row)
        #             continue
        #
        # if check:
        #     print("Turn 5 report:")
        #     segment_report()
        #     reset_params()
        #     check = False
        #
        # # wait till reaching turn 6: newton - S 16 Stop sign right turn
        # while True:
        #     if lat != 0 and lon != 0:
        #         current_dist = gps_distance(turn_start[5])
        #         if current_dist < 5:
        #             break
        #     row = next(reader)
        #     update_params(row)
        #
        # print("Entering Turn 6 ")
        #
        # # updating distance parameters to track turn end
        # current_dist = gps_distance(turn_end[5])
        # prev_dist = current_dist
        #
        # while True:
        #     if acc > 10.0 and check is False:  # indicates moving of vehicle
        #         check = True
        #
        #     if acc > 30.0:  # start checking after  first major acceleration in speed
        #         bad_acc += 1
        #
        #     if check:
        #         last_speed = speed
        #
        #     if lat == 0 and lon == 0:
        #         row = next(reader)
        #         update_params(row)
        #         continue
        #     else:
        #         prev_dist = current_dist
        #         current_dist = gps_distance(turn_end[5])
        #         if (round(gps_distance(turn_end[5])) <= 5.0) or is_point_crossed() is True:
        #             turn_exitSpeed = speed
        #             break
        #         else:
        #             row = next(reader)
        #             update_params(row)
        #             continue
        #
        # if check:
        #     print("Turn 6 report:")
        #     turn_stopsign_report()
        #     # check = False
        #     reset_params()
        #
        # # wait till reaching turn 7: S 16 - S Blvd Stop light left turn
        # while True:
        #     if lat != 0 and lon != 0:
        #         current_dist = gps_distance(turn_start[6])
        #         if current_dist < 5:
        #             break
        #     row = next(reader)
        #     update_params(row)
        #
        # exit()
        #
        # if row is None:
        #     print("End of file:")
        #     break
