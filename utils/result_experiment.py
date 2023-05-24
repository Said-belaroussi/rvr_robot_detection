import csv

with open("./rosbags/session184_17_05_3-tf.csv", 'r') as file:
  csvreader = csv.reader(file)
  first_time = 0
  i=-9
  cam_list = []
  for row in csvreader:
    if (i+6)% 17 == 0 :
      if first_time == 0:
        first_time = int(row[0][10:])
      print(row)
      time_in_sec = int(row[0][10:]) - first_time
      print(time_in_sec)
    if i %17 == 0 :
      print(row)
      x = float(row[0][7:])
      print(x)
    if (i-1) %17 == 0 :
      print(row)
      y = float(row[0][7:])
      print(y)
      cam_list.append([time_in_sec, x, y])
    i+=1

with open("./rosbags/tycho184_17_05_3-tf.csv", 'r') as file:
  csvreader = csv.reader(file)
  first_time = 0
  i=-9
  tycho_list = []
  for row in csvreader:
    if (i+6)% 17 == 0 :
      if first_time == 0:
        first_time = int(row[0][10:])
      print(row)
      time_in_sec = int(row[0][10:]) - first_time
      print(time_in_sec)
    if i %17 == 0 :
      print(row)
      x = float(row[0][7:])
      print(x)
    if (i-1) %17 == 0 :
      print(row)
      y = float(row[0][7:])
      print(y)
      tycho_list.append([time_in_sec, x, y])
    i+=1

for i in range(len(tycho_list)-1,-1, -1):
  if i < 11:
    tycho_list[i][0] = tycho_list[0][0]
  else:
    tycho_list[i][0] = tycho_list[i-11][0]


j = 0
k = 0
for i in range(len(tycho_list)):
  if tycho_list[i][0] == j :
    k += 1
  else :
    k = 0
    j += 1
i = 0
j = 0
comparison_list = []
while i < len(cam_list) and j < len(tycho_list):
  if cam_list[i][0] == tycho_list[j][0] :
    comparison_list.append([cam_list[i][1], cam_list[i][2], tycho_list[j][1], tycho_list[j][2]])
    i += 1
    j += 1
  elif cam_list[i][0] < tycho_list[j][0] :
    i += 1
  elif cam_list[i][0] > tycho_list[j][0] :
    j += 1

average = 0
for i in comparison_list:
    average += ((i[0]-i[2])**2 + (i[1]-i[3])**2)**(1/2)
    print(((i[0]-i[2])**2 + (i[1]-i[3])**2)**(1/2))

average_error = (average /len(comparison_list))
average = 0
for i in comparison_list:
    average += ((i[0]-i[2])**2 + (i[1]-i[3])**2)
    print(((i[0]-i[2])**2 + (i[1]-i[3])**2)**(1/2))

RMSE = (average /len(comparison_list))**(1/2)
print("\nAverage error: ", average_error, "\nRMSE: ", RMSE)
