import time
import math
import tkinter as tk
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

CANVAS_WIDTH = 666
CANVAS_HEIGHT = 666

GRID_SIZE = 4

ROBOT_SIZE = 0.26
SCANNER_OFFSET = (0, 0.09)
SCANNER_RADIUS = 0.08

POINT_RADIUS = 8

LOCK_ROTATION = False


points = []
position = None
rotation = 0


path_points = []


MODE_VIEW = 0
MODE_PLACE_POINT = 1

active_mode = MODE_VIEW


mouse_pos = [0, 0]


def mouse_move(event):
    global mouse_pos
    mouse_pos[0] = event.x
    mouse_pos[1] = event.y


def on_mouse_left_click(event):
    global path_points
    if(active_mode == MODE_PLACE_POINT):
        # TODO
        path_points.append(mouse_pos.copy())


def rgb(r, g, b):
    if r < 0 or g < 0 or b < 0 or r > 255 or g > 255 or b > 255:
        return '#000000'
    hex_ = '#'
    if r < 16:
        hex_ += '0'
    hex_ += hex(r)[2:4]
    if g < 16:
        hex_ += '0'
    hex_ += hex(g)[2:4]
    if b < 16:
        hex_ += '0'
    hex_ += hex(b)[2:4]
    return hex_


def set_mode(mode, mode_label):
    global active_mode
    global path_points
    active_mode = mode
    if mode == MODE_VIEW:
        mode_label.config(text="Mode: VIEW")
        path_points.clear()
    elif mode == MODE_PLACE_POINT:
        mode_label.config(text="Mode: PLACE_POINT [press <V> to return to VIEW]")
    else:
        mode_label.config(text="Mode: ERR")


def gui_main():
    global points
    global path_points
    global active_mode
    
    root = tk.Tk()
    root.title("TurtlebotVis")


    
    mode_label = tk.Label(root,
                          text="Mode: VIEW",
                          font=("Courier", 14),
                          height=2)
   
    root.bind("<Motion>", mouse_move)
    root.bind("<Button-1>", on_mouse_left_click)
    root.bind("v", lambda _: set_mode(MODE_VIEW, mode_label))
    root.bind("p", lambda _: set_mode(MODE_PLACE_POINT, mode_label))

    canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg="white")
    
    mode_label.pack()
    canvas.pack()
    
    offset_x = CANVAS_WIDTH // 2
    offset_y = CANVAS_HEIGHT // 2
    
    def animation_loop():
        canvas.delete("all")  

        # Grid
        unit_size = CANVAS_WIDTH / GRID_SIZE
        for i in range(GRID_SIZE):
            canvas.create_line(i * unit_size + (offset_x % unit_size) + unit_size / 2, 0, i * unit_size + (offset_x % unit_size) + unit_size / 2, CANVAS_HEIGHT, width=1, fill=rgb(230, 230, 230))
            canvas.create_line(0, i * unit_size + (offset_y % unit_size) + unit_size / 2, CANVAS_WIDTH, i * unit_size + (offset_y % unit_size) + unit_size / 2, width=1, fill=rgb(230, 230, 230))
            canvas.create_line(i * unit_size + (offset_x % unit_size), 0, i * unit_size + (offset_x % unit_size), CANVAS_HEIGHT, width=1, fill=rgb(180, 180, 180))
            canvas.create_line(0, i * unit_size + (offset_y % unit_size), CANVAS_WIDTH, i * unit_size + (offset_y % unit_size), width=1, fill=rgb(180, 180, 180))
        
        # Points
        for p in points:
            x = p[0]
            y = p[1]
            canvas_x = offset_x + x * unit_size
            canvas_y = offset_y - y * unit_size

            canvas.create_line(offset_x, offset_x, canvas_x, canvas_y, fill=rgb(100, 180, 255))
            canvas.create_oval(canvas_x - POINT_RADIUS, canvas_y - POINT_RADIUS, canvas_x + POINT_RADIUS, canvas_y + POINT_RADIUS, fill='red', outline="")

        # Robot body
        canvas.create_rectangle(offset_x - ROBOT_SIZE / 2 * unit_size + SCANNER_OFFSET[0] * unit_size, offset_y - ROBOT_SIZE / 2 * unit_size + SCANNER_OFFSET[1] * unit_size, offset_x + ROBOT_SIZE / 2 * unit_size + SCANNER_OFFSET[0] * unit_size, offset_y + ROBOT_SIZE / 2 * unit_size + SCANNER_OFFSET[1] * unit_size, fill="red", outline='') 

        # Robot scanner
        if not LOCK_ROTATION:
            canvas.create_oval(
                offset_x - SCANNER_RADIUS / 2 * unit_size, offset_y - SCANNER_RADIUS / 2 * unit_size,
                offset_x + SCANNER_RADIUS / 2 * unit_size, offset_y + SCANNER_RADIUS / 2 * unit_size,
                fill=rgb(252, 248, 0), outline=""
            )

        if active_mode == MODE_PLACE_POINT:
            canvas.create_oval(
                mouse_pos[0] - POINT_RADIUS, mouse_pos[1] - POINT_RADIUS,
                mouse_pos[0] + POINT_RADIUS, mouse_pos[1] + POINT_RADIUS,
                fill="#fcb502", outline=""
            )
       
        # Path points
        
        if len(path_points) > 0:
            canvas.create_line(offset_x, offset_y, path_points[0][0], path_points[0][1], width=1, fill="#fcb502")
        
        for i, p in enumerate(path_points):
            canvas.create_oval(
                p[0] - POINT_RADIUS, p[1] - POINT_RADIUS,
                p[0] + POINT_RADIUS, p[1] + POINT_RADIUS,
                fill="#fcb502" if i < len(path_points) - 1 else "#0afc02", outline=""
            )
            if i < len(path_points) - 1:
                canvas.create_line(p[0], p[1], path_points[i + 1][0], path_points[i + 1][1], width=1, fill="#fcb502")


        root.after(100, animation_loop)
    
    animation_loop()
    root.mainloop()


class TurtlebotVis(Node):
    def __init__(self):
        super().__init__('scanvis')
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def scan_callback(self, msg):
        global points
        global rotation
        pts = []
        for i in range(len(msg.ranges)):
            range_ = msg.ranges[i]
            if range_ < msg.range_min or range_ > msg.range_max:
                continue
            angle = i * 2 * math.pi / len(msg.ranges) + (rotation if LOCK_ROTATION else 1.570796)
            val = [range_ * math.cos(angle), range_ * math.sin(angle)]
            pts.append(val)
        points = pts

    def odom_callback(self, msg):
        global position
        global rotation
        position = msg.pose.pose.position
        rotation = 0
        siny_cosp = 2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
        cosy_cosp = 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        rotation = math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    t = Thread(target=gui_main, args=())

    try:
        rclpy.init(args=args)
        scanvis = TurtlebotVis()
        t.start()
        rclpy.spin(scanvis)
        scanvis.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
