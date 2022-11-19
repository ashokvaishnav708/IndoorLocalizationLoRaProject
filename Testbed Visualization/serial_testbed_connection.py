# serial port connection and import data and print on map

import time
import serial
import re

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
#from scipy.misc import imread

addr = "COM3"
baud = 115200

ser = serial.Serial(addr, baud)
print("Connected to: " + ser.portstr)


NODE_POSITION = [[1530,  135], [2730,  135], [2730, 2005], [2730, 2705], [2295,  875], [135,  135], [135, 2415]]

# ser = serial.Serial(
#     port=addr,
#     baudrate=baud,
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE,
#     bytesize=serial.EIGHTBITS,
#     timeout=0)

map_path = 'floor_plan_kiel.png'


im = plt.imread(map_path)
fig, ax = plt.subplots()
#fig = plt.figure()


#point, = ax.plot([], [], marker='o', c='red')


patch = plt.Circle((0, 0), 300, fc='r', alpha=0.2)



# while True:
# 	data = str(ser.readline())
# 	#print(data)
# 	for word in re.finditer(r"Location\s:\(\d*", data):
# 		x = int(data[(word.start()+11):(word.end())])
# 	for word in re.finditer(r",\s\d*\)", data):
# 		y = int(data[(word.start()+2):(word.end() -1)])
# 	if (x and y):
# 		print('Location: ('+str(x)+', '+str(y)+')')
lx = 0
ly = 0

def update(i):
	global lx
	global ly
	x=''
	y=''
	data = str(ser.readline())
	#print(data)
	for word in re.finditer(r"Location\s:\(\d*", data):
		x = int(data[(word.start()+11):(word.end())])
	for word in re.finditer(r",\s\d*\)", data):
		y = int(data[(word.start()+2):(word.end() -1)])
	#ax.imshow(im)
	if (x and y):
		lx = x
		ly = y
		patch.center = (x, y)
		return patch,
		#print('Location: ('+str(x)+', '+str(y)+')')
		#point.set_data([x],[y])
	else:
		patch.center = (lx, ly)
		return patch,
		#point.set_data([], [])
	#return point,

def init():
	patch.center = (0, 0)
	ax.add_patch(patch)
	return patch,
	#point.set_data([], [])
	#return point,


anim = animation.FuncAnimation(fig, update, 
                               init_func=init, 
                               frames=360, 
                               interval=500,
                               blit=True,
                               repeat=True)

for i in range(len(NODE_POSITION)):
	if not i == 4:

plt.imshow(im)
plt.show()
# while True:
# 	x, y = getLocation(str(ser.readline()))
# 	print(x, y)
# 	# plt.imshow(im)
# 	# plt.scatter(x, y, 'o', color='red')
# 	# plt.show()
# 	# plt.clf(0.5)


	



