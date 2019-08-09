from datetime import datetime

time1_str = "22:43.8"
time2_str =  "22:44.8"

format = '%M:%S.%f'
time1 = datetime.strptime(time1_str,format)
time2 = datetime.strptime(time2_str,format)
tdiff = time2-time1

print (time1, time2)
print (tdiff.seconds)