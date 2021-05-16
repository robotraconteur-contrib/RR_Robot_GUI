#!/usr/bin/python3
from RobotRaconteur.Client import *
import sys, os, time, argparse, traceback, copy
from tkinter import *
from tkinter import messagebox
import numpy as np
from importlib import import_module
sys.path.append('toolbox')
from vel_emulate_sub import EmulatedVelocityControl

#Accept the names of the robots from command line
parser = argparse.ArgumentParser(description="RR plug and play client")
parser.add_argument("--robot-name",type=str)
args, _ = parser.parse_known_args()
robot_name=args.robot_name


#auto discovery for robot service
time.sleep(2)
res=RRN.FindServiceByType("com.robotraconteur.robotics.robot.Robot",
["rr+local","rr+tcp","rrs+tcp"])
url=None
for serviceinfo2 in res:
	if robot_name in serviceinfo2.NodeName:
		url=serviceinfo2.ConnectionURL
		break
if url==None:
	print('service not found')
	sys.exit()

#auto discovery for robot tool service
res=RRN.FindServiceByType("com.robotraconteur.robotics.tool.Tool",
["rr+local","rr+tcp","rrs+tcp"])
url_gripper=None
for serviceinfo2 in res:
	if robot_name in serviceinfo2.NodeName:
		url_gripper=serviceinfo2.ConnectionURL
		break
if url_gripper==None:
	print('gripper service not found')
else:
	#connect
	try:
		tool=RRN.ConnectService(url_gripper)
	except:
		traceback.print_exc()

#connect robot services
robot_sub=RRN.SubscribeService(url)
robot=robot_sub.GetDefaultClientWait(1)
state_w = robot_sub.SubscribeWire("robot_state")
#get params of robots
num_joints=len(robot.robot_info.joint_info)


##########Initialize robot constants
robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", robot)
state_flags_enum = robot_const['RobotStateFlags']
halt_mode = robot_const["RobotCommandMode"]["halt"]
position_mode = robot_const["RobotCommandMode"]["position_command"]
robot.command_mode = halt_mode
time.sleep(0.1)
robot.command_mode = position_mode
cmd_w = robot_sub.SubscribeWire("position_command")

vel_ctrl = EmulatedVelocityControl(robot,state_w, cmd_w)
#enable velocity mode
vel_ctrl.enable_velocity_mode()


top=Tk()
top.title(robot_name)
jobid = None
def gripper_ctrl(tool):

	if gripper.config('relief')[-1] == 'sunken':
		tool.open()
		gripper.config(relief="raised")
		gripper.configure(bg='red')
		gripper.configure(text='gripper off')

	else:
		tool.close()
		gripper.config(relief="sunken")
		gripper.configure(bg='green')
		gripper.configure(text='gripper on')
	return

def move(joint, vel_ctrl,vd):
	global jobid
	try:
		qdot=np.zeros(num_joints)
		qdot[joint]=vd
		
		vel_ctrl.set_velocity_command(qdot)

		jobid = top.after(10, lambda: move(joint, vel_ctrl,vd))
	except:
		traceback.print_exc()
	return

def stop(n,vel_ctrl):
	global jobid
	top.after_cancel(jobid)
	vel_ctrl.set_velocity_command(np.zeros((n,)))
	return

##RR part
# def update_label():
# 	robot_state=state_w.TryGetInValue()
# 	flags_text = "Robot State Flags:\n\n"
# 	if robot_state[0]:
# 		for flag_name, flag_code in state_flags_enum.items():
# 			if flag_code & robot_state[1].robot_state_flags != 0:
# 				flags_text += flag_name + "\n"
# 	else:
# 		flags_text += 'service not running'
		
# 	joint_text = "Robot Joint Positions:\n\n"
# 	for j in robot_state[1].joint_position:
# 		joint_text += "%.2f\n" % np.rad2deg(j)

# 	label.config(text = flags_text + "\n\n" + joint_text)

# 	label.after(250, update_label)

# top.title = "Robot State"

# label = Label(top, fg = "black", justify=LEFT)
# label.pack()
# label.after(250,update_label)

joints_ccw=[]
joints_cw=[]
for i in range(num_joints):
	Label(top, text="joint"+str(i+1)).grid(row=i,column=0)
	joints_ccw.append(Button(top,text='+'))
	joints_ccw[i].grid(row=i,column=1)
	joints_cw.append(Button(top,text='-'))
	joints_cw[i].grid(row=i,column=2)

	# joints_ccw[i].bind('<ButtonPress-1>', lambda event: move(i,vel_ctrl,0.2))
	# joints_cw[i].bind('<ButtonPress-1>', lambda event: move(i,vel_ctrl,-0.2))
	# joints_ccw[i].bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))
	# joints_cw[i].bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))


joints_ccw[0].bind('<ButtonPress-1>', lambda event: move(0,vel_ctrl,0.2))
joints_cw[0].bind('<ButtonPress-1>', lambda event: move(0,vel_ctrl,-0.2))
joints_ccw[0].bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))
joints_cw[0].bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))
joints_ccw[1].bind('<ButtonPress-1>', lambda event: move(1,vel_ctrl,0.2))
joints_cw[1].bind('<ButtonPress-1>', lambda event: move(1,vel_ctrl,-0.2))
joints_ccw[1].bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))
joints_cw[1].bind('<ButtonRelease-1>', lambda event: stop(num_joints,vel_ctrl))

gripper=Button(top,text='gripper off',command=lambda: gripper_ctrl(tool),bg='red')
gripper.grid(row=num_joints,column=0)


top.mainloop()
vel_ctrl.disable_velocity_mode()