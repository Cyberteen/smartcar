# Start Date - 20th June 2019
# @Author - Shankar Sridhar

import csv
from geopy.distance import geodesic,great_circle

rpm_idx = 6
speed_idx = 7
acc_idx = 8
lat_idx = 9
lon_idx = 10
dist_idx = 13
limits = [90.0, 150.0, 2005.0, 2100.0, 2317.0, 2427.0, 4300.0, 4350.0, 4745.0, 4805.0, 4841.0, 4975.0, 5038.0,
          5088.0, 9189.0, 9320.0, 10990.0, 11095.0, 11375.0, 11565.0, 111840.0, 11955.0, 12000.0, 12075.0]

# global variables
check = False
avg_speed = 0.0
cnt = 0.0
turn = 1
bad_acc = 0
last_speed = 0.0
turn_entrySpeed = 0.0
turn_exitSpeed = 0.0
prev_dist = 0.0 # for noting distance from last GPS fix to target point
current_dist = 0.0 # for noting distance from current GPS fix to target point
is_stopped = False
prev_gps_fix = (0,0)
zone_speeds = []
zone_accs = []
zone_rpms = []
zone_dist = 0.0
hesitations = 0
hesitating = False
crucial_zone = False

# row params of interest
acc = 0.0
speed = 0.0
lat = 0.0
lon = 0.0
dist = 0.0

# tuple of latitude and longitude for turn start and turn ends.
turn_start = []
turn_start.append((42.000620, -93.633499)) #S Loop Drive to Airport road
turn_start[1] = (42.001049, -93.610349)
turn_start[2] = (42.005532,	-93.586389)
turn_start[3] = (42.009372, -93.586653)
turn_start[4] = (42.009566, -93.585268)
turn_start[5] = (42.00838713, -93.5842956)
turn_start[6] = (42.011378, -93.633668)
turn_start[7] = (41.997891,	-93.639397)  # 1st roundabout start
turn_start[8] = (41.993972,	-93.639316)
turn_start[9] = (41.996908, -93.639255)
turn_start[10] = (41.99747,	-93.637548)

turn_end = []
turn_end[0] = (42.00090357, -93.63290814)
turn_end[1] = (42.001351, -93.609888)
turn_end[2] = (42.005676, -93.586267)
turn_end[3] = (42.009535, -93.586233)
turn_end[4] = (42.008887, -93.584279)
turn_end[5] = (42.008276, -93.584682)
turn_end[6] = (42.011472, -93.634072)
turn_end[7] = (41.996918, -93.639408)  # 1st roundabout end
turn_end[8] = (41.993978, -93.639205)
turn_end[9] = (41.997305, -93.638618)
turn_end[10] = (41.997383, -93.637181)

def turn_report():
    global avg_speed, cnt, bad_acc, turn_entrySpeed, turn_exitSpeed
    print('Average turn speed:' + str(avg_speed / cnt) + ' km/h')
    cnt = 0.0
    avg_speed = 0.0
    print("No.of hard accelerations: " + str(bad_acc))
    bad_acc = 0

    return


def turn_stopsign_report():
    global last_speed, bad_acc, cnt, avg_speed
    print('Maximum turn speed reached:' + str(last_speed) + ' km/h')
    cnt = 0.0
    avg_speed = 0.0
    print("No.of hard accelerations: " + str(bad_acc))
    bad_acc = 0

    return


# converting required data from string to float
def update_params(row):
    global acc, speed, lat, lon, dist

    acc = float(row[acc_idx])
    speed = float(row[speed_idx])
    lat = float(row[lat_idx])
    lon = float(row[lon_idx])
    dist = float(row[dist_idx])

    return


def reset_params():
    global avg_speed, turn_entrySpeed, turn_exitSpeed, last_speed, current_dist, prev_dist, check, cnt

    avg_speed = 0.0
    turn_exitSpeed = 0.0
    turn_entrySpeed = 0.0
    last_speed = 0.0
    cnt = 0.0
    current_dist =0.0
    check = False

    return


def gps_distance(zone_point):
    global lat, lon

    current_point = (lat, lon)
    #zone_point = turn_start[1]

    return geodesic(current_point, zone_point).meters


def is_point_crossed():

    global current_dist, prev_dist

    if current_dist - prev_dist <= 1.0:
        return False
    else:
        return True


with open('C:\\Users\\DELL\\Documents\\Smart Car\\new logs\\log_LAPS_2019_06_01_13_45_49.csv') as file:
    reader = csv.reader(file, delimiter=',')

    for row in reader:

        # wait till reaching turn 1
        while True:
            if lat != 0 and lon != 0:
                current_dist = gps_distance(turn_start[0])
                if current_dist < 5: # or is_point_crossed(turn_start[0]):
                    break
            row = next(reader)
            update_params(row)

        print("Inside turn 1")

        # update current distance from the turn end point
        current_dist = gps_distance(turn_end[0])
        prev_dist = current_dist
        # wait till reaching turn 1
        # while lat < 42.00061 or lat == 0:
        #     row = next(reader)
        #     update_params(row)

        # Turn 1 - Stop Sign Right
        # while (dist >= limits[0]) and (dist <= limits[1]):
        # while 42.00061 < lat < 42.0008 and lat != 0:

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
            else:
                prev_dist = current_dist
                current_dist = gps_distance(turn_end[0])
                if (round(gps_distance(turn_end[0])) <= 5.0) or is_point_crossed() is True:
                    turn_exitSpeed = speed
                    break
                else:
                    prev_gps_fix = (lat,lon)
                    row = next(reader)
                    update_params(row)

                    continue

        if check:
            print("Turn 1 report:")
            turn_stopsign_report()
            #check = False
            reset_params()

    # wait till reaching turn 2
        while True:
            if lat != 0 and lon != 0:
                current_dist = gps_distance(turn_start[1])
                if current_dist < 5: # or is_point_crossed(turn_start[1]):
                    break
            row = next(reader)
            update_params(row)

        # Turn 2 - Stop Light - Left turn
        # if (dist >= limits[2]) and (dist <= limits[3]):
        print("Inside Turn 2")
        current_dist = gps_distance(turn_end[1])
        prev_dist = current_dist

        while True:
            if check is False and speed <= 10.0 and acc < 10.0:  # if vehicle was still before start of turn
                while acc < 10.0:
                    row = next(reader)
                    update_params(row)
                is_stopped = True
                check = True

            if check is False and speed > 10.0:
                check = True
                is_stopped = False

            if is_stopped is True and check is True:
                if acc > 10.0 and check is False:  # indicates moving of vehicle
                    check = True

                if acc > 30.0:  # start checking after  first major acceleration in speed
                    bad_acc += 1

                if check:
                    last_speed = speed

            elif is_stopped is False and check is True:
                last_speed = speed
                avg_speed += speed
                cnt += 1.0

                if acc > 30.0:
                    bad_acc += 1

            if lat == 0 and lon == 0:
                row = next(reader)
                update_params(row)
            else:
                prev_dist = current_dist
                current_dist = gps_distance(turn_end[1])
                if (round(gps_distance(turn_end[1])) <= 5.0) or is_point_crossed() is True:
                    turn_exitSpeed = speed
                    break
                else:
                    row = next(reader)
                    update_params(row)
                    continue

        if check:
            print("Turn 2 report")
            if is_stopped:
                turn_stopsign_report()
            else:
                turn_report()
            #check = False
            reset_params()

            # while dist <= limits[3]:
            #     if acc > 30.0:
            #         bad_acc += 1
            #     avg_speed += speed  # also check and note down average speed
            #     cnt += 1.0
            #     row = next(reader)
            #     update_params(row)

        # if cnt > 0:
        #     print("Average turn stability speed: " + str(avg_speed / cnt))
        #     print("No.of bad accelerations: " + str(bad_acc))
        #     avg_speed = 0.0
        #     bad_acc = 0.0
        #     cnt = 0

        # wait till reaching turn 3
        while True:
            if lat != 0 and lon != 0:
                current_dist = gps_distance(turn_start[2])
                if current_dist < 5:
                    break
            row = next(reader)
            update_params(row)

        print("Entering turn 3 zone - Stop sign left from highway")

        # updating distance parameters to track turn end
        current_dist = gps_distance(turn_end[2])
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

            prev_dist = current_dist
            current_dist = gps_distance(turn_end[2])

            if (gps_distance(turn_end[2]) < 5) or (is_point_crossed() is True):
                break
            else:
                row = next(reader)
                update_params(row)

        if check:
            print("Turn 3 report:")
            turn_stopsign_report()
            reset_params()

            # while lat < 42.0056:
            #     if acc > 30.0:  # start checking after  first major acceleration in speed
            #         if bad_acc == 0:  # if its the first acceleration for the stop-turn, and very high acc
            #             check = True
            #             if bad_acc > 40.0:  # first time, only consider very high acc.
            #                 bad_acc = 1
            #         else:
            #             if check:
            #                 bad_acc += 1
            #     if check:
            #         avg_speed += speed
            #         cnt = cnt + 1.0
            #     #  print('Right turn 1: ' + row[13] + 'm and GPS:' + row[lat] + ',' + row[lon])
            #     row = next(reader)
            #     update_params(row)

        # if cnt > 0:
        #     print('Turn 3 report')
        #     check = False
        #     turn_report()

        # while lat != fcnt += 1.0
        #     avg_speed += speed
        #     if -30.0 < acc < 30.0:
        #         bad_acc += 1
        #
        #     row = next(reader)
        #     update_params(row)
        #
        # if cnt > 0:
        #     print('Turn 3 stability report:')
        #     turn_report()

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
        turn_entrySpeed = speed

        current_dist = gps_distance(turn_end[3])
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
                current_dist = gps_distance(turn_end[3])

                if (round(gps_distance(turn_end[3])) <= 5.0) or is_point_crossed() is True:
                    turn_exitSpeed = speed
                    break
                else:
                    row = next(reader)
                    update_params(row)
                    continue

        if check:
            print("Turn 4 report:")
            turn_report()
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
            turn_report()
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

        # Turn 7 -
        # if (dist >= limits[2]) and (dist <= limits[3]):
        print("Inside Turn 7")
        current_dist = gps_distance(turn_end[6])
        prev_dist = current_dist

        while True:
            if check is False and speed <= 10.0 and acc < 10.0:  # if vehicle was still before start of turn
                while acc < 10.0:
                    row = next(reader)
                    update_params(row)
                is_stopped = True
                check = True

            if check is False and speed > 10.0:
                check = True
                is_stopped = False

            if is_stopped is True and check is True:
                if acc > 10.0 and check is False:  # indicates moving of vehicle
                    check = True

                if acc > 30.0:  # start checking after  first major acceleration in speed
                    bad_acc += 1

                if check:
                    last_speed = speed

            elif is_stopped is False and check is True:
                last_speed = speed
                avg_speed += speed
                cnt += 1.0

                if acc > 30.0:
                    bad_acc += 1

            if lat == 0 and lon == 0:
                row = next(reader)
                update_params(row)
            else:
                prev_dist = current_dist
                current_dist = gps_distance(turn_end[6])
                if (round(gps_distance(turn_end[6])) <= 5.0) or is_point_crossed() is True:
                    turn_exitSpeed = speed
                    break
                else:
                    row = next(reader)
                    update_params(row)
                    continue

        if check:
            print("Turn 7 report")
            if is_stopped:
                turn_stopsign_report()
            else:
                turn_report()
            # check = False
            reset_params()

        # wait till reaching turn 8: S Univ Blvd - Roundabout 2nd exit
        while True:
            if lat != 0 and lon != 0:
                current_dist = gps_distance(turn_start[8])
                if current_dist < 5:
                    break
            row = next(reader)
            update_params(row)

        current_dist = gps_distance(turn_end[8])
        prev_dist = current_dist
        zone_dist = 0
        check = True

        while True:
            zone_speeds.append(speed)
            zone_accs.append(acc)

            if acc < -20 or acc > 20:
                bad_acc += 1

            if speed <= 5:
                hesitating = True

            if lat == 0 and lon == 0:
                row = next(reader)
                update_params(row)
            else:
                prev_dist = current_dist
                current_dist = gps_distance(turn_end[7])
                if (round(gps_distance(turn_end[7])) <= 5.0) or is_point_crossed() is True:
                    turn_exitSpeed = speed
                    break
                else:
                    row = next(reader)
                    update_params(row)
                    continue










        exit()

        if row is None:
            print("End of file:")
            break
