import tkinter as tk
from tkinter import messagebox
from tkinter.filedialog import askopenfilename, asksaveasfilename
from tkinter.colorchooser import askcolor
import xml.etree.ElementTree as ET
import rospkg
import os
import subprocess
import signal
import atexit
#TODO REFACTOR FUNCTIONS IN A SWTICH CASE MANNER

def load_last_dir():
	try:
		with open("lastdir.txt", "r") as f:
			for line in f:
				last_dir.set(line.strip())
				break
	except:
		pass


def save_last_dir(path):
	last_dir.set(path)
	with open("lastdir.txt", "w+") as f:
		f.write(last_dir.get())

def open_file():
	"""Open a file for editing."""
	filepath = askopenfilename(
		filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
	)
	if not filepath:
		return
	print(filepath)
	window.title(filepath)

def save_file():
	"""Save the current file as a new file."""
	filepath = asksaveasfilename(
		defaultextension="txt",
		filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")],
	)
	if not filepath:
		return
	print(filepath)
	window.title(filepath)

def load_config():
	filepath = askopenfilename(
		initialdir=last_dir.get(),
		filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
	)
	if not filepath:
		return
	
	txt_config_lbl.set(filepath.split('/')[-2]+"/"+filepath.split('/')[-1])
	txt_config_path.set(filepath)
	save_last_dir('/'.join(filepath.split('/')[:-1]) + "/")

def load_pose():
	filepath = askopenfilename(
		initialdir=last_dir.get(),
		filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
	)
	if not filepath:
		return
	
	txt_pose_lbl.set(filepath.split('/')[-2]+"/"+filepath.split('/')[-1])
	txt_pose_path.set(filepath)
	save_last_dir('/'.join(filepath.split('/')[:-1]) + "/")

def gload_init(index, label):
	filepath = askopenfilename(
		initialdir=last_dir.get(),
		filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
	)
	if not filepath:
		return
	
	label.set(filepath.split('/')[-1])
	concat_list[index]['initFileName'] = filepath
	save_last_dir('/'.join(filepath.split('/')[:-1]) + "/")


def gload_end(index, label):
	filepath = askopenfilename(
		initialdir=last_dir.get(),
		filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
	)
	if not filepath:
		return
	
	label.set(filepath.split('/')[-1])
	concat_list[index]['endFileName'] = filepath
	save_last_dir('/'.join(filepath.split('/')[:-1]) + "/")

def checkbutton_callback(index, button):
	concat_list[index]['active'] = button.get()

def load_init():
	filepath = askopenfilename(
		initialdir=last_dir.get(),
		filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
	)
	if not filepath:
		return
	
	txt_init_lbl.set(filepath.split('/')[-1])
	txt_init_path.set(filepath)
	save_last_dir('/'.join(filepath.split('/')[:-1]) + "/")


def load_end():
	filepath = askopenfilename(
		initialdir=last_dir.get(),
		filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
	)
	if not filepath:
		return
	
	txt_end_lbl.set(filepath.split('/')[-1])
	txt_end_path.set(filepath)
	save_last_dir('/'.join(filepath.split('/')[:-1]) + "/")

def go_config():
	path = txt_config_path.get()
	if path is not "":
		p = subprocess.Popen(['rostopic', 'pub', '-1', '/principle_planner_command', 'std_msgs/String', '"go_to_config '+path+'"' ], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
		out, err = p.communicate()
	else:
		print("No input file selected")

def go_pose():
	path = txt_pose_path.get()
	if path is not "":
		p = subprocess.Popen(['rostopic', 'pub', '-1', '/principle_planner_command', 'std_msgs/String', '"go_to_pose '+path+'"'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
		out, err = p.communicate()
	else:
		print("No input file selected")

def scroll_function(event):
	global list_height
	canvas.configure(scrollregion=canvas.bbox("all"),width=520,height=300)

def complex_import():
	filepath = askopenfilename(
		initialdir=last_dir.get(),
		filetypes=[("Plans", "*.plan"), ("All Files", "*.*")]
	)
	if not filepath:
		return
	
	complex_reset()

	with open(filepath, "r") as f:
		for line in f:
			args = line.split(' ')
			active = args[0]
			init = args[1]
			end = args[2]
			principle_str_get = args[3]
			apply_principle = args[4]
			presence = args[5]

			principle_str = ""

			if principle_str_get == "drag":
				principle_str = "Drag"
			elif principle_str_get == "arcs":
				principle_str = "Arcs"
			elif principle_str_get == "siso":
				principle_str = "SI&SO"

			complex_add_arg(active, init, end, principle_str, apply_principle, presence)

	save_last_dir('/'.join(filepath.split('/')[:-1]) + "/")

def complex_export():
	filepath = asksaveasfilename(
		initialdir=last_dir.get(),
		defaultextension=".plan",
		filetypes=[("Plans", "*.plan"), ("All Files", "*.*")],
	)
	if not filepath:
		return

	sorted_list = sorted([ ( concat_list[key]['index'], concat_list[key] ) for key in concat_list ])
	sorted_dict = [ item[1] for item in sorted_list ]

	with open(filepath, "w+") as f:
		linestr = ""
		for item in sorted_dict:
			principle_str_get = item['principle'].get()
			principle_str = ""

			if principle_str_get == "Drag":
				principle_str = "drag"
			elif principle_str_get == "Arcs":
				principle_str = "arcs"
			elif principle_str_get == "SI&SO":
				principle_str = "siso"

			linestr += str(item['active'])+" "+ str(item['initFileName'])+ " "+ str(item['endFileName'])+ " "+ principle_str+ " "+ str(item['apply'].get())+ " "+ str(item['presence'].get()) +"\n"
		
		f.write(linestr)

def complex_reset():
	result = messagebox.askquestion("Reset", "Are You Sure?", icon='warning')
	if result == 'yes':
		for key in list(concat_list):
			complex_del(key)

def complex_del(index):
	global num_rows

	cur_index = concat_list[index]['index']

	for key in concat_list:
		key_index = concat_list[key]['index']
		if key_index > cur_index:
			concat_list[key]['index'] = key_index - 1
			concat_list[key]['label'].configure(text=str(key_index))
			for item in concat_list[key]['grid_list']:
				item.grid(row=key_index-1)

	if concat_list[index]['grid_list']:
		for item in concat_list[index]['grid_list']:
			if item:
				item.destroy()

	if concat_list[index]['var_list']:
		for item in concat_list[index]['var_list']:
			if item:
				del item

	concat_list.pop(index)

	num_rows -= 1

def complex_add_arg(active, init, end, principle, apply_principle, presence ):
	global num_rows
	index = num_rows
	btn_del_t = tk.Button( fr_rows, text="x", command= lambda: complex_del(btn_del_t))
	btn_del_t.grid(row=index, column=7, sticky="w",padx=0,pady=0)

	lbl_t = tk.Label(fr_rows,text=str(index+1))
	lbl_t.grid(row=index, column=0)
	chk_var_t = tk.IntVar()
	chk_var_t.set(int(active))
	btn_chk_t = tk.Checkbutton(fr_rows, variable=chk_var_t, command= lambda: checkbutton_callback(btn_del_t, chk_var_t))
	btn_chk_t.grid(row=index, column=1)

	txt_principle_lbl_t = tk.StringVar()
	txt_principle_lbl_t.set(principle)
	list_principle_t = tk.OptionMenu(fr_rows, txt_principle_lbl_t, *principle_list)
	list_principle_t.config(width=int(button_width/2))
	list_principle_t.grid(row=index, column=4, sticky="ew", padx=0, pady=0)

	principle_var_t = tk.IntVar()
	principle_var_t.set(int(apply_principle))
	chk_principle_t = tk.Checkbutton(fr_rows, text="Apply", variable=principle_var_t)
	chk_principle_t.grid(row=index, column=5, sticky="ew", padx=0, pady=0)

	scl_presence_t = tk.Scale(fr_rows, from_=0, to=10, tickinterval=10, orient=tk.HORIZONTAL)
	scl_presence_t.set(int(presence))
	scl_presence_t.grid(row=index, column=6, sticky="ew", padx=0, pady=0)

	txt_init_lbl_t = tk.StringVar()
	txt_init_lbl_t.set(init.split('/')[-1])

	btn_load_init_t = tk.Button(fr_rows, width=int(button_width/2), textvariable=txt_init_lbl_t, command= lambda: gload_init(btn_del_t, txt_init_lbl_t))
	btn_load_init_t.grid(row=index, column=2, sticky="ew", padx=0, pady=0)

	txt_end_lbl_t = tk.StringVar()
	txt_end_lbl_t.set(end.split('/')[-1])
	btn_load_end_t = tk.Button(fr_rows, width=int(button_width/2), textvariable=txt_end_lbl_t, command= lambda: gload_end(btn_del_t, txt_end_lbl_t))
	btn_load_end_t.grid(row=index, column=3, sticky="ew", padx=0, pady=0)

	concat_list[btn_del_t] = {
		'active': 1, 
		'index' : index, 
		'initFileName' : init,
		'endFileName' : end,
		'presence' : scl_presence_t,
		'apply' : principle_var_t,
		'principle' : txt_principle_lbl_t,
		'label': lbl_t,
		'principle_button': txt_principle_lbl, 
		'var_list': [chk_var_t, txt_principle_lbl_t,principle_var_t,txt_init_lbl_t,txt_end_lbl_t,btn_del_t], 
		'grid_list': [lbl_t, btn_chk_t, list_principle_t, chk_principle_t, scl_presence_t, btn_load_init_t, btn_load_end_t, btn_del_t]}

	num_rows += 1


def complex_add():
	global num_rows
	index = num_rows
	btn_del_t = tk.Button( fr_rows, text="x", command= lambda: complex_del(btn_del_t))
	btn_del_t.grid(row=index, column=7, sticky="w",padx=0,pady=0)

	lbl_t = tk.Label(fr_rows,text=str(index+1))
	lbl_t.grid(row=index, column=0)
	chk_var_t = tk.IntVar()
	chk_var_t.set(1)
	btn_chk_t = tk.Checkbutton(fr_rows, variable=chk_var_t, command= lambda: checkbutton_callback(btn_del_t, chk_var_t))
	btn_chk_t.grid(row=index, column=1)

	txt_principle_lbl_t = tk.StringVar()
	txt_principle_lbl_t.set("Principle")
	list_principle_t = tk.OptionMenu(fr_rows, txt_principle_lbl_t, *principle_list)
	list_principle_t.config(width=int(button_width/2))
	list_principle_t.grid(row=index, column=4, sticky="ew", padx=0, pady=0)

	principle_var_t = tk.IntVar()

	chk_principle_t = tk.Checkbutton(fr_rows, text="Apply", variable=principle_var_t)
	chk_principle_t.grid(row=index, column=5, sticky="ew", padx=0, pady=0)

	scl_presence_t = tk.Scale(fr_rows, from_=0, to=10, tickinterval=10, orient=tk.HORIZONTAL)
	scl_presence_t.grid(row=index, column=6, sticky="ew", padx=0, pady=0)

	txt_init_lbl_t = tk.StringVar()
	txt_init_lbl_t.set("Initial")
	btn_load_init_t = tk.Button(fr_rows, width=int(button_width/2), textvariable=txt_init_lbl_t, command= lambda: gload_init(btn_del_t, txt_init_lbl_t))
	btn_load_init_t.grid(row=index, column=2, sticky="ew", padx=0, pady=0)

	txt_end_lbl_t = tk.StringVar()
	txt_end_lbl_t.set("End")
	btn_load_end_t = tk.Button(fr_rows, width=int(button_width/2), textvariable=txt_end_lbl_t, command= lambda: gload_end(btn_del_t, txt_end_lbl_t))
	btn_load_end_t.grid(row=index, column=3, sticky="ew", padx=0, pady=0)

	
	concat_list[btn_del_t] = {
		'active': 1, 
		'index' : index, 
		'presence' : scl_presence_t,
		'apply' : principle_var_t,
		'principle' : txt_principle_lbl_t,
		'label': lbl_t,
		'principle_button': txt_principle_lbl, 
		'var_list': [chk_var_t, txt_principle_lbl_t,principle_var_t,txt_init_lbl_t,txt_end_lbl_t,btn_del_t], 
		'grid_list': [lbl_t, btn_chk_t, list_principle_t, chk_principle_t, scl_presence_t, btn_load_init_t, btn_load_end_t, btn_del_t]}

	num_rows += 1

def complex_plan():
	filepath = last_dir.get()+"active_plan.plan"
	sorted_list = sorted([ ( concat_list[key]['index'], concat_list[key] ) for key in concat_list ])
	sorted_dict = [ item[1] for item in sorted_list ]

	with open(filepath, "w+") as f:
		linestr = ""
		for item in sorted_dict:
			principle_str_get = item['principle'].get()
			principle_str = ""

			if principle_str_get == "Drag":
				principle_str = "drag"
			elif principle_str_get == "Arcs":
				principle_str = "arcs"
			elif principle_str_get == "SI&SO":
				principle_str = "siso"

			linestr += str(item['active'])+" "+ str(item['initFileName'])+ " "+ str(item['endFileName'])+ " "+ principle_str+ " "+ str(item['apply'].get())+ " "+ str(item['presence'].get()) +"\n"
		
		f.write(linestr)

	p = subprocess.Popen(['rostopic', 'pub', '-1', '/principle_planner_command', 'std_msgs/String', '"complex_plan '+filepath+'"'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
	out, err = p.communicate()

def plan_principle():
	command = ""

	init = txt_init_path.get()
	end = txt_end_path.get()
	presence = ""
	if principle_var.get() == 1:
		presence = str(scl_presence.get())

		if txt_principle_lbl.get() == "Drag" :
			command = "principle_drag"
		elif txt_principle_lbl.get() == "SI&SO":
			command = "principle_siso"
		if txt_principle_lbl.get() == "Arcs" : 
			command = "principle_arcs"
			presence = "1"

	else:
		if txt_principle_lbl.get() == "Arcs" : 
			command = "principle_arcs"
			presence = "0"
		else:
			command = "plan_basic"

	command_str = command + " " + init + " " + end + " "+ presence

	if command is not "":
		p = subprocess.Popen(['rostopic', 'pub', '-1', '/principle_planner_command', 'std_msgs/String', '"'+command_str+'"'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
		out, err = p.communicate()
	else:
		print("No input file selected")

def save_config():
	filepath = asksaveasfilename(
		initialdir=last_dir.get(),
		defaultextension=".txt",
		filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")],
	)
	if not filepath:
		return

	p = subprocess.Popen(['rostopic', 'pub', '-1', '/principle_planner_command', 'std_msgs/String', '"save_config '+filepath+'"'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
	out, err = p.communicate()

def save_pose():
	filepath = asksaveasfilename(
		initialdir=last_dir.get(),
		defaultextension=".txt",
		filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")],
	)
	if not filepath:
		return

	p = subprocess.Popen(['rostopic', 'pub', '-1', '/principle_planner_command', 'std_msgs/String', '"save_pose '+filepath+'"'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
	out, err = p.communicate()

def load_color():
	try:
		rospack = rospkg.RosPack()
		path = rospack.get_path('robotiq_2f_140_gripper_visualization') + "/meshes/visual_glass/robotiq_arg2f_140_left_inner_finger.stl.dae"
		ET.register_namespace('',"http://www.collada.org/2005/11/COLLADASchema")
		tree = ET.parse(path)
		root = tree.getroot()
		color_str = root[1][0][0][0][0][1][0].text
		color = color_str.split("   ")[:-1]
		color_input = list(map(lambda x: int(float(x)*255.0), color ))
		color_hex = '#%02x%02x%02x' % (color_input[0], color_input[1], color_input[2])
		btn_color.configure(bg = color_hex)
	except:
		pass

def pick_color():
	color_input = askcolor()
	if not color_input:
		return
	rospack = rospkg.RosPack()
	path = rospack.get_path('robotiq_2f_140_gripper_visualization') + "/meshes/visual_glass/robotiq_arg2f_140_left_inner_finger.stl.dae"
	color = list(map(lambda x: "{:.2f}".format(x / 255.0), color_input[0] ))
	color.append("1")
	ET.register_namespace('',"http://www.collada.org/2005/11/COLLADASchema")
	tree = ET.parse(path)
	root = tree.getroot()
	color_str = "   ".join(color)
	root[1][0][0][0][0][1][0].text = color_str
	#with open(path, "w") as f:
	#	f.write( ET.tostring )
	tree.write(path)
	#print(ET.tostring(root, encoding='utf-8').decode('utf-8'))
	btn_color.configure(bg = color_input[1])

def run_rviz():
	global is_rviz, rviz_p

	if is_rviz:
		os.kill(rviz_p.pid, 9)
		p = subprocess.Popen(['killall', 'rviz'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
		out, err = p.communicate()
		txt_rviz.set("Run RViz")
		is_rviz = False
	else:
		glass_bool = 'true' if glass_var.get() == 1 else 'false'
		rviz_p = subprocess.Popen(['roslaunch', 'ur5_moveit_config', 'demo.launch', 'limited:=true', 'glasses:='+glass_bool])
		p_planner = subprocess.Popen(['rosrun', 'ur5xprs', 'principlePlanner'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
		print("READY FOR OPERATION")
		txt_rviz.set("Close RViz")
		is_rviz = True

def node_cleanup():
	p = subprocess.Popen(['rosnode', 'kill', '/animation_principle_planner_node'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
	out, err = p.communicate()

window = tk.Tk()
window.resizable(0,0)
window.title("Animation Principle Planner")
last_dir = tk.StringVar()
last_dir.set("/home/")
load_last_dir()

atexit.register(node_cleanup)

concat_list = {}
num_rows = 0
#window.rowconfigure(0, minsize=600, weight=1)
#window.columnconfigure(0, minsize=800, weight=1)

fr_buttons = tk.Frame(master=window, width=300, bg="white", borderwidth=4, relief="solid")
fr_buttons.pack(fill=tk.BOTH, side=tk.LEFT)

button_width = 12
button_pady = 22
list_height = 300

principle_var = tk.IntVar()
glass_var = tk.IntVar()

fr_list = tk.Frame(master=window, width=520, bg="white", borderwidth=4, relief="solid")
fr_list.pack(fill=tk.BOTH, side=tk.LEFT)

fr_principle = tk.Frame(master=fr_buttons)
fr_principle.grid(row=4, column=0, columnspan=2, sticky='nesw',padx=5)

txt_principle_lbl = tk.StringVar()
txt_principle_lbl.set("Principle")

principle_list = ["Arcs", "SI&SO", "Drag"]

list_principle = tk.OptionMenu(fr_principle, txt_principle_lbl, *principle_list )
list_principle.config(width=int(button_width/2))
list_principle.grid(row=0, column=2, sticky="w", padx=10, pady=15)

#btn_set_principle = tk.Button(fr_list, width=int(button_width/2), textvariable=txt_principle_lbl, command= set_principle)
#btn_set_principle.grid(row=0, column=1, sticky="ew", padx=5, pady=25)

tk.Label(fr_list, text="Complex Principle Plan", font='Helvetica 12 bold', bg="white").grid(row=0, column=0, columnspan=5, sticky="w", padx=5, pady=9)

txt_init_path = tk.StringVar()
txt_init_path.set("")
txt_init_lbl = tk.StringVar()
txt_init_lbl.set("Initial")
btn_load_init = tk.Button(fr_principle, width=int(button_width/2), textvariable=txt_init_lbl, command=load_init)
btn_load_init.grid(row=0, column=0, sticky="w", padx=10, pady=5)

txt_end_path = tk.StringVar()
txt_end_path.set("")
txt_end_lbl = tk.StringVar()
txt_end_lbl.set("End")
btn_load_end = tk.Button(fr_principle, width=int(button_width/2), textvariable=txt_end_lbl, command=load_end)
btn_load_end.grid(row=0, column=1, sticky="w", padx=10, pady=5)

btn_plan_principle = tk.Button(fr_principle, width=int(button_width/2), text="Go", command= plan_principle)
btn_plan_principle.grid(row=1, column=2, sticky="ew", padx=5, pady=5)

chk_principle = tk.Checkbutton(fr_principle, text="Apply", variable=principle_var)
chk_principle.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

scl_presence = tk.Scale(fr_principle, from_=0, to=10, tickinterval=10, orient=tk.HORIZONTAL)
scl_presence.grid(row=1, column=1, sticky="ew", padx=5, pady=5)

fr_canvas = tk.Frame(master=fr_list, width=520, height=350)
fr_canvas.grid(row=1, column=0, columnspan=5, sticky='nesw',padx=5)
canvas = tk.Canvas(fr_canvas, width=520, height=list_height)
fr_rows = tk.Frame(master=canvas)
scrollbar = tk.Scrollbar(master=fr_canvas, orient="vertical", command=canvas.yview)
scrollbar.grid(row=1, column=5, sticky="nsew")
canvas.configure(yscrollcommand = scrollbar.set)
scrollbar.pack(side="right",fill="y")
canvas.pack(side="left", fill=tk.BOTH)
canvas.create_window((0,0), window=fr_rows, anchor='nw')
fr_rows.bind("<Configure>", scroll_function)

'''
list_concat = tk.Listbox(fr_list)
list_concat.config(width=button_width, selectmode=tk.MULTIPLE)
list_concat.grid(row=1, column=0, columnspan=2, sticky="nesw", padx=5, pady=10)
'''

btn_complex_add = tk.Button(fr_list, width=int(button_width/2), text="Add", command= complex_add)
btn_complex_add.grid(row=3, column=0, sticky="ew", padx=5, pady=23)

btn_complex_save = tk.Button(fr_list, width=int(button_width/2), text="Export", command= complex_export)
btn_complex_save.grid(row=3, column=1, sticky="ew", padx=5, pady=23)

btn_complex_import = tk.Button(fr_list, width=int(button_width/2), text="Import", command= complex_import)
btn_complex_import.grid(row=3, column=2, sticky="ew", padx=5, pady=23)

btn_complex_reset = tk.Button(fr_list, width=int(button_width/2), text="Reset", command= complex_reset)
btn_complex_reset.grid(row=3, column=3, sticky="ew", padx=5, pady=23)

btn_complex_plan = tk.Button(fr_list, width=int(button_width/2), text="Plan & Execute", command= complex_plan)
btn_complex_plan.grid(row=3, column=4, sticky="ew", padx=5, pady=23)

fr_glass = tk.Frame(master=fr_buttons)
fr_glass.grid(row=0, column=0, sticky="ew", padx=5, pady=button_pady)

chk_glass = tk.Checkbutton(fr_glass, text="Glasses", variable=glass_var)
chk_glass.grid(row=0, column=0, sticky="e", padx=0, pady=5)

btn_color = tk.Button(fr_glass, text="Color", command= pick_color)
btn_color.grid(row=0, column=1, sticky="e", padx=0, pady=0)

load_color()


btn_save_config = tk.Button(fr_buttons, width=button_width, text="Save config", command= save_config)
btn_save_config.grid(row=1, column=0, sticky="ew", padx=5, pady=button_pady)

txt_rviz = tk.StringVar()
txt_rviz.set("Run RViz")
rviz_p = None
is_rviz = False
btn_run_rviz = tk.Button(fr_buttons, width=button_width, textvariable=txt_rviz, command= run_rviz)
btn_run_rviz.grid(row=0, column=1, sticky="ew", padx=5, pady=button_pady)

btn_save_pose = tk.Button(fr_buttons, width=button_width, text="Save pose", command= save_pose)
btn_save_pose.grid(row=1, column=1, sticky="ew", padx=5, pady=button_pady)

txt_config_path = tk.StringVar()
txt_config_path.set("")
txt_config_lbl = tk.StringVar()
txt_config_lbl.set("Load config")
btn_load_config = tk.Button(fr_buttons, width=button_width, textvariable=txt_config_lbl, command=load_config)
btn_load_config.grid(row=2, column=0, sticky="ew", padx=5, pady=button_pady)

btn_go_config = tk.Button(fr_buttons, width=button_width, text="Go to config", command= go_config)
btn_go_config.grid(row=2, column=1, sticky="ew", padx=5, pady=button_pady)

txt_pose_path = tk.StringVar()
txt_pose_path.set("")
txt_pose_lbl = tk.StringVar()
txt_pose_lbl.set("Load pose")
btn_load_pose = tk.Button(fr_buttons, width=button_width, textvariable=txt_pose_lbl, command= load_pose)
btn_load_pose.grid(row=3, column=0, sticky="ew", padx=5, pady=button_pady)

btn_go_pose = tk.Button(fr_buttons, width=button_width, text="Go to pose", command= go_pose)
btn_go_pose.grid(row=3, column=1, sticky="ew", padx=5, pady=button_pady)

window.mainloop()