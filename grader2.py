# Start Date - 20th June 2019
# @Author - Shankar Sridhar

# grading based on distances

import csv
from enum import Enum

from geopy.distance import geodesic
from datetime import datetime

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

segment_limits = [
    (70, 150),
    (1800, 2200)
]

# segments sub-divided into zones
zone_limits = [
    [(70, 90), (91, 105), (106, 130), (131, 150)],
    [(1800, 2015), (2016, 2085), (2086, 2200)]  # airport road - univ blvd stop light
]

speed_limits = [
    (40.23, 40.23, 32.18, 72.41),#(25, 25, 20, 45),  # segment 1
    (72.41, 72.41, 56.3)  # segment 2
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
check = False
avg_speed = 0.0
cnt = 0.0
has_turn = False
has_stopSign = False
has_stopLight = False
complete_stop = False
bad_acc = 0
last_speed = 0.0
turn_entrySpeed = 0.0
turn_exitSpeed = 0.0
min_speed = 200
prev_dist = 0.0  # for noting distance from last GPS fix to target point
curr_dist = 0.0  # for noting distance from current GPS fix to target point
zone_dist = 0.0

prev_gps_fix = (0, 0)
time_fmt = "%H:%M:%S.%f"
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
acc1 = 0.0
acc2 = 0.0
state_check = False

# Its a list of lists each containing tuples of (distance , excess acc, excess speed) pair
event_dist = [[], [], [], []]  # acceleration, deceleration, cruise and speed
turn_excess = []
thresh = [5, -5, 1]  # acceleration/deceleration threshold levels

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
    global event_dist, thresh, speed_limits, current_segment

    acc_dist = 0
    dec_dist = 0
    cruise_dist = 0
    penalty = 0
    score = 1
    total_jerk = 0
    dist_per_zone = [0,0,0,0] #per event
    speed_penalty_per_zone = [0,0,0,0]
    # for action in segment_dist:

    for a in event_dist[State.accelerating.value]:  # acceleration
        acc_dist += a[0]
        # if exceeded the threshold, calculate penalty
        penalty += (0, a[0] * a[1]*0.28)[a[1] > 0]  # dist accelerated * excess accelerated * 0.28 for m/s^2 conversion
        total_jerk += abs(a[2])
        # 1st element is the distance covered over last second. 5th element is that zone's value
        dist_per_zone[a[4]]+= a[0]
        speed_penalty_per_zone[a[4]] += a[0]*a[3]  #dist *excess speed saved for each zone num(a[4))

    print("Total distance accelerated:" + str(acc_dist) + ' m')
    if acc_dist is not 0:
        print(" Acceleration Score: " + str((1 - (penalty / (acc_dist * thresh[State.accelerating.value]*0.28))) * 100) + '%')

    print("Average jerk: " + str(total_jerk / len(event_dist[State.accelerating.value])))

    speed_score_perZone = [0,0,0,0]

    for n in range(0,len(speed_limits[current_segment])):
        speed_score_perZone[n] = (dist_per_zone[n]*speed_limits[current_segment][n] - speed_penalty_per_zone[n])/dist_per_zone[n]

    print("Speed limit")

    penalty = 0
    total_jerk = 0

    for d in event_dist[State.decelerating.value]:
        dec_dist += d[0]
        penalty += d[0] * d[1]
        total_jerk += abs(d[2])

    print("Total distance decelerated`:" + str(dec_dist) + ' m')
    if dec_dist is not 0:
        print(" Deceleration Score: " + str((1 - (penalty / (dec_dist * thresh[State.decelerating.value]))) * 100) + '%')

    print("Average jerk: " + str(total_jerk / len(event_dist[State.decelerating.value])))

    total_jerk = 0
    penalty = 0

    for c in event_dist[State.cruise.value]:
        cruise_dist += c[0]
        penalty += c[0] * c[1]
        total_jerk += abs(c[2])

    print("Total distance cruised: " + str(cruise_dist) + "m")
    if cruise_dist is not 0:
        print("Cruise score:" + str((1 - (penalty / (cruise_dist * thresh[State.cruise.value]))) * 100) + "%")

    print("Average jerk: " + str(total_jerk / len(event_dist[State.cruise.value])))

    if has_stopSign:
        if complete_stop:
            score = 1
            print("complete stop detected")
            print("Score:" + str(score * 100))
        else:
            print("incomplete stop / rolling stop detected!")
            print("score: " + str(100 - pow(min_speed, 2)))

    return


def turn_report():
    global turn_exitSpeed, turn_entrySpeed, turn_excess

    total_speed = 0

    for t in turn_excess: #t is a (dist,speed) tuple
        total_speed += t[1]

    avg_speed = total_speed / len(turn_excess)
    print("Average speed: " + str(avg_speed))


    #print('Maximum turn speed reached:' + str(last_speed) + ' km/h')

    #print("No.of hard accelerations: " + str(bad_acc))
    #bad_acc = 0

    return


# converting required data from string to float
def update_params(row):
    global acc, row_speed, speed, lat, lon, dist
    global row_time, prev_speed, prev_state

    acc = float(row[acc_idx])
    row_speed = float(row[speed_idx])
    lat = float(row[lat_idx])
    lon = float(row[lon_idx])
    dist = float(row[dist_idx])
    row_time = row[time_idx]
    prev_state = state_now

    speed = row_speed  # * 0.27 #convert speed from row in km/h to m/s for calculating acc and jerk

    return


def reset_params():
    global avg_speed, turn_entrySpeed, turn_exitSpeed, last_speed, current_dist, prev_dist, check, cnt
    global has_stopLight, has_stopSign, has_turn, complete_stop

    avg_speed = 0.0
    turn_exitSpeed = 0.0
    turn_entrySpeed = 0.0
    last_speed = 0.0
    cnt = 0.0
    current_dist = 0.0
    check = False
    has_stopSign = False
    has_turn = False
    has_stopLight = False
    complete_stop = False

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
    global acc1, acc2, jerk, speed, prev_speed

    # save prev acc of prev second
    acc1 = acc2
    # update latest second's acc
    acc2 = speed - prev_speed
    # save speed of current instance
    prev_speed = speed
    # calc jerk from last two accelerations
    jerk = acc2 - acc1

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
            if -2<= acc <0:
                return State.cruise
            else:
                return State.decelerating


# based on thresh[] for acceleration/deceleration thresholds, add excess acc/dec to event_dist list
# also adds speed exceed based on speed_limit variable.
def calc_excess():
    global event_dist, state_now, speed_limit, jerk, current_zone

    if state_now != State.stop:
        delta_dist = current_dist - prev_dist
        excess_acc = abs(acc2) - abs(thresh[state_now.value])
        excess_speed = speed - speed_limit

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

        #if speed > speed_limit:
         #   event_dist[3].append((delta_dist, speed - speed_limit))


# identifies zone based on distance. Sets has_turn flag and also the speed limit for the zone
def identify_zone():
    global has_turn, has_stopLight, has_stopSign, current_zone, speed_limit, current_segment

    if segment_limits[0][0] <= dist <= segment_limits[0][1]:  # segment number, start/end
        has_turn = True  # indicates that this segment has a turn
        has_stopSign = True
        current_segment = 0
        if zone_limits[0][0][0] <= dist <= zone_limits[0][0][1]:  # segment number,zone number ,start/end limit
            update_zone(Zone.Approaching)
            # current_zone = Zone.Approaching
            speed_limit = speed_limits[0][0]  # segment number,zone number
            return
        if zone_limits[0][1][0] <= dist <= zone_limits[0][1][1]:
            update_zone(Zone.Special)
            # current_zone = Zone.Special
            speed_limit = speed_limits[0][1]
            return
        if zone_limits[0][2][0] <= dist <= zone_limits[0][2][1]:
            update_zone(Zone.Turn)
            # current_zone = Zone.Turn
            speed_limit = speed_limits[0][2]
            return
        if zone_limits[0][3][0] <= dist <= zone_limits[0][3][1]:
            update_zone(Zone.Leaving)
            # current_zone = Zone.Leaving
            speed_limit = speed_limits[0][3]
            return

    if segment_limits[1][0] <= dist <= segment_limits[1][1]:
        has_turn = True
        has_stopLight = True
        current_segment = 1

        if zone_limits[1][0][0] <= dist <= zone_limits[1][0][1]:
            update_zone(Zone.Approaching)
            # current_zone = Zone.Approaching
            speed_limit = speed_limits[1][0]
            return
        if zone_limits[1][1][0] <= dist <= zone_limits[1][1][1]:
            update_zone(Zone.Special)
            # current_zone = Zone.Special
            speed_limit = speed_limits[1][1]
            return
        if zone_limits[1][2][0] <= dist <= zone_limits[1][2][1]:
            update_zone(Zone.Leaving)
            # current_zone = Zone.Leaving
            speed_limit = speed_limits[1][2]
            return


# saves the previous zone info and updates the current zone
def update_zone(zone_now):
    global prev_zone, current_zone

    prev_zone = current_zone
    current_zone = zone_now


# called after every segment is finished
def reset_params():
    global acc1, acc2, jerk, speed, prev_speed, prev_time, current_dist, prev_dist, zone_dist, event_dist
    global current_zone, prev_zone
    acc1 = 0
    acc2 = 0
    jerk = 0
    prev_speed = speed
    prev_time = datetime.strptime(row_time, time_fmt)

    event_dist = [[], [], [], []]
    # update current distance from the turn end point
    current_dist = dist
    prev_dist = dist
    zone_dist = 0.0
    current_zone = Zone.undefined
    prev_zone = Zone.undefined


# def update_state():

# with open('C:\\Users\\ssridhar\\Documents\\logs\\log_LAPS_2019_07_14_16_10_42.csv') as file:
with open('C:\\Users\\DELL\\Documents\\Smart Car\\final driving files\\log_LAPS_2019_07_14_16_10_42.csv') as file:
    reader = csv.reader(file, delimiter=',')

    for row in reader:

        # wait till reaching turn 1 stop sign
        while dist < segment_limits[0][0]:
            row = next(reader)
            update_params(row)

        print("Inside segment 1")
        reset_params()

        # wait till reaching turn 1
        # while lat < 42.00061 or lat == 0:
        #     row = next(reader)
        #     update_params(row)

        # Turn 1 - Stop Sign Right
        # while (dist >= limits[0]) and (dist <= limits[1]):
        # while 42.00061 < lat < 42.0008 and lat != 0:

        #while zone_dist <= turn_dist[0]:
        while dist <= segment_limits[0][1]:
            # keep moving in csv rows for 1 second
            while is_time() is False:
                update_params(row)
                row = next(reader)

            # after one second
            update_acc()

            # identifies the current state based on the acc/speed values calculated for the last second
            state_now = identify_state()

            # identifies the zone inside this segment and the sets the speed limit for it.
            identify_zone()

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
                    turn_entrySpeed = speed

                avg_speed += speed

            elif current_zone is Zone.Leaving and prev_zone is Zone.Turn:
                turn_exitSpeed = speed



            print(state_now)

            row = next(reader)
            update_params(row)
            prev_dist = current_dist
            current_dist = dist
            zone_dist += (current_dist - prev_dist)

        print('End of segment 1')

        print("Segment 1 report:")
        segment_report()
        if has_turn is True:
            print("Turn report:")
            turn_report()

        reset_params()

        # wait till reaching turn 2
        while True:
            if lat != 0 and lon != 0:
                current_dist = gps_distance(turn_start[1])
                if current_dist < 5:  # or is_point_crossed(turn_start[1]):
                    break
            row = next(reader)
            update_params(row)

        # Turn 2 - Stop Light - Left turn
        # if (dist >= limits[2]) and (dist <= limits[3]):
        print("Inside Turn 2")

        reset_params()

        while zone_dist <= turn_dist[1]:
            # keep moving in csv rows for 1 second
            while is_time() is False:
                update_params(row)
                row = next(reader)

            # after one second
            update_acc()

            state_now = identify_state()

            calc_excess()

            print(state_now)

            row = next(reader)
            update_params(row)
            prev_dist = current_dist
            current_dist = dist
            zone_dist += (current_dist - prev_dist)

        print("End of turn 2")

        segment_report()
        # check = False
        reset_params()

        # wait till reaching turn 3
        while True:
            if lat != 0 and lon != 0:
                current_dist = gps_distance(turn_start[2])
                if current_dist < 5:
                    break
            row = next(reader)
            update_params(row)

        print("Entering turn 3 zone - Stop sign left from highway")
        reset_params()

        while zone_dist <= turn_dist[2]:
            # keep moving in csv rows for 1 second
            while is_time() is False:
                update_params(row)
                row = next(reader)

            # after one second
            update_acc()

            state_now = identify_state()

            calc_excess()

            print(state_now)

            row = next(reader)
            update_params(row)
            prev_dist = current_dist
            current_dist = dist
            zone_dist += (current_dist - prev_dist)

        print('End of turn 3')

        segment_report()

        # Turn 4 - Kum n go Free Right turn

        # wait till reaching turn 4
        while True:
            if lat != 0 and lon != 0:
                current_dist = gps_distance(turn_start[3])
                if current_dist < 5:
                    break
            row = next(reader)
            update_params(row)

        print("Entering Turn 4 ")

        reset_params()

        while zone_dist <= turn_dist[3]:
            # keep moving in csv rows for 1 second
            while is_time() is False:
                update_params(row)
                row = next(reader)

            # after one second
            update_acc()

            state_now = identify_state()

            print(state_now)

            row = next(reader)
            update_params(row)
            prev_dist = current_dist
            current_dist = dist
            zone_dist += (current_dist - prev_dist)

        print('End of turn 4')

        if check:
            print("Turn 4 report:")
            segment_report()
            reset_params()

        # wait till reaching turn 5 - dairy queen curve
        while True:
            if lat != 0 and lon != 0:
                current_dist = gps_distance(turn_start[4])
                if current_dist < 5:
                    break
            row = next(reader)
            update_params(row)

        print("Entering Turn 5 ")
        turn_entrySpeed = speed

        current_dist = gps_distance(turn_end[4])
        prev_dist = current_dist
        check = True

        while True:

            if acc < -30.0 or acc > 30.0:
                bad_acc += 1

            avg_speed += speed
            cnt += 1.0

            if lat == 0 and lon == 0:
                row = next(reader)
                update_params(row)
                continue
            else:
                prev_dist = current_dist
                current_dist = gps_distance(turn_end[4])
                if (round(gps_distance(turn_end[4])) <= 5.0) or is_point_crossed() is True:
                    turn_exitSpeed = speed
                    break
                else:
                    row = next(reader)
                    update_params(row)
                    continue

        if check:
            print("Turn 5 report:")
            segment_report()
            reset_params()
            check = False

        # wait till reaching turn 6: newton - S 16 Stop sign right turn
        while True:
            if lat != 0 and lon != 0:
                current_dist = gps_distance(turn_start[5])
                if current_dist < 5:
                    break
            row = next(reader)
            update_params(row)

        print("Entering Turn 6 ")

        # updating distance parameters to track turn end
        current_dist = gps_distance(turn_end[5])
        prev_dist = current_dist

        while True:
            if acc > 10.0 and check is False:  # indicates moving of vehicle
                check = True

            if acc > 30.0:  # start checking after  first major acceleration in speed
                bad_acc += 1

            if check:
                last_speed = speed

            if lat == 0 and lon == 0:
                row = next(reader)
                update_params(row)
                continue
            else:
                prev_dist = current_dist
                current_dist = gps_distance(turn_end[5])
                if (round(gps_distance(turn_end[5])) <= 5.0) or is_point_crossed() is True:
                    turn_exitSpeed = speed
                    break
                else:
                    row = next(reader)
                    update_params(row)
                    continue

        if check:
            print("Turn 6 report:")
            turn_stopsign_report()
            # check = False
            reset_params()

        # wait till reaching turn 7: S 16 - S Blvd Stop light left turn
        while True:
            if lat != 0 and lon != 0:
                current_dist = gps_distance(turn_start[6])
                if current_dist < 5:
                    break
            row = next(reader)
            update_params(row)

        exit()

        if row is None:
            print("End of file:")
            break
