#!/usr/bin/env python
import sqlite3
import rospy
import sys
import os.path

# print str(sys.argv[0])
conn = sqlite3.connect(str(sys.argv[1]))
c = conn.cursor()
rospy.init_node('set_params')

if not os.path.isfile(sys.argv[1]) :
	rospy.logfatal("[load_item_params]: File %s doesn't exist.", sys.argv[1])
	sys.exit(0)

rospy.loginfo("[load_item_params]: Loading item parameters from %s", sys.argv[1])

row_names = []
try:
	c.execute('''PRAGMA table_info(items)''')
	for row in c:
		row_names.append(row[1])
except Exception as inst:
	rospy.logerr("[load_item_params]: Couldn't get table info: %s", inst)

try:
	c.execute('''SELECT * FROM items''')
	names = []
	for row in c:
		names.append(row[1])
		for i in xrange(2, len(row_names)):
			if row[i] is not None:
				rospy.set_param('items/'+str(row[1])+'/'+str(row_names[i]), row[i])
	rospy.set_param('items/list', names)
except Exception as inst:
	rospy.logerr("[load_item_params]: Couldn't load SQLite parameters: %s", inst)
finally:
	rospy.loginfo("[load_item_params]: Done loading SQLite parameters")
