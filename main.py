import tkinter as tk
from time import time
import math as m

root = tk.Tk()
root.title('Virtual Lab')

phi = 0  # phase of the crankshaft
r = 1  # radius in m
l = 2  # l in m
ang_vel = -m.pi  # angular velocity in rad/s

tick = 16  # application update interval in ms

b = [0.0, 0.0]
a = [0.0, 0.0]
b_dot = [0.0, 0.0]
b_dot2 = [0.0, 0.0]
a_dot = [0.0, 0.0]
a_dot2 = [0.0, 0.0]
a_b_dot = [0.0, 0.0]
a_b_dot2_r = [0.0, 0.0]
a_b_dot2_t = [0.0, 0.0]


def vector_calculations():

    sinPhi = m.sin(phi)
    cosPhi = m.cos(phi)
    denominator = l * l - r * r * cosPhi * cosPhi
    invDenominator = 1 / denominator
    sqrtDenominator = m.sqrt(denominator)
    invSqrtDenominator = 1 / sqrtDenominator

    global b, a, b_dot, b_dot2, a_dot, a_dot2, a_b_dot, a_b_dot2_r, a_b_dot2_t

    b = [-r * sinPhi, r * cosPhi]
    a = [-r * sinPhi - sqrtDenominator, 0]

    b_dot = [-r * cosPhi * ang_vel, -r * sinPhi * ang_vel]
    b_dot2 = [r * sinPhi * ang_vel * ang_vel, -r * cosPhi * ang_vel * ang_vel]

    a_dot_x = - ang_vel * r * r * sinPhi * cosPhi * \
        invSqrtDenominator - ang_vel * r * cosPhi
    a_dot2_x = ang_vel * ang_vel * r * r * invSqrtDenominator * (
        sinPhi * sinPhi - cosPhi * cosPhi + r * r * sinPhi * sinPhi * cosPhi * cosPhi * invDenominator)\
        + ang_vel * ang_vel * r * sinPhi

    a_dot = [a_dot_x, 0]
    a_dot2 = [a_dot2_x, 0]

    a_b = [a[0] - b[0], a[1] - b[1]]
    a_b_dot = [a_dot[0] - b_dot[0], a_dot[1] - b_dot[1]]
    a_b_dot2 = [a_dot2[0] - b_dot2[0], a_dot2[1] - b_dot2[1]]

    mag_a_b_dot = a_b_dot[0] * a_b_dot[0] + a_b_dot[1] * a_b_dot[1]

    inv_l = 1 / l
    mag_a_b_dot2_r = mag_a_b_dot * inv_l

    a_b_dot2_r = [- mag_a_b_dot2_r * inv_l *
                  a_b[0], - mag_a_b_dot2_r * inv_l * a_b[1]]
    a_b_dot2_t = [a_b_dot2[0] - a_b_dot2_r[0], a_b_dot2[1] - a_b_dot2_r[1]]


def vector_addition(x, y):
    return [x[0] + y[0], x[1] - y[1]]


def scalar_multiplication(x, m):
    return [i * m for i in x]


top_frame = tk.Frame(root)
top_frame.pack(side=tk.TOP)


class PhysicalModel:
    def __init__(self):
        self.frame = tk.Frame(top_frame)
        self.frame.pack(side=tk.LEFT)

        self.width = 1280
        self.height = 720
        self.border = 160
        self.hb = self.height - 2 * self.border
        self.wb = self.width - 2 * self.border

        self.canvas = tk.Canvas(self.frame, width=self.width, height=self.height, bg='white', highlightthickness=4,
                                highlightbackground="black")
        self.canvas.pack(side=tk.LEFT, pady=5, padx=5)

        self.position_scale = self.position_scale_calculation()

        self.xd, self.yd = self.axes_translation_vector()

        # self.A, self.B = self.points()

        self.O = self.canvas_conversion([0, 0], self.position_scale)
        self.A = self.canvas_conversion(a, self.position_scale)
        self.B = self.canvas_conversion(b, self.position_scale)

        self.piston_length = 120
        self.piston_height = 80

        self.piston = self.canvas.create_rectangle(self.A[0] - self.piston_length, self.A[1] + self.piston_height,
                                                   self.A[0] + self.piston_length, self.A[1] -
                                                   self.piston_height,
                                                   fill='LightBlue3', width=2)

        self.beam_width = 60

        self.beam_OB = self.canvas.create_line(self.O[0], self.O[1], self.B[0], self.B[1], fill='LightBlue1',
                                               width=self.beam_width)
        self.beam_AB = self.canvas.create_line(self.A[0], self.A[1], self.B[0], self.B[1], fill='LightBlue1',
                                               width=self.beam_width)

        self.joint_radius = 50
        self.joint_border = 2

        self.joint_o = self.create_joint(self.O)
        self.joint_a = self.create_joint(self.A)
        self.joint_b = self.create_joint(self.B)

        self.arrow_size = [3, 10, 12, 5]
        self.arrow_size_r = [3, 10, 12, 5]

        self.arrow_radius = 150

        self.v_scale = self.arrow_radius / \
            m.sqrt(b_dot[0]*b_dot[0] + b_dot[1]*b_dot[1])
        self.a_scale = self.arrow_radius / \
            m.sqrt(b_dot2[0]*b_dot2[0] + b_dot2[1]*b_dot2[1])

        self.A = self.canvas_conversion(a, self.position_scale)
        self.B = self.canvas_conversion(b, self.position_scale)
        self.B_v = self.canvas_conversion(b_dot, self.v_scale)
        self.B_a = self.canvas_conversion(b_dot2, self.a_scale)
        self.A_v = self.canvas_conversion(a_dot, self.v_scale)
        self.A_vr = self.canvas_conversion(a_b_dot, self.v_scale)
        self.A_a = self.canvas_conversion(a_dot2, self.a_scale)
        self.A_ar = self.canvas_conversion(a_b_dot2_r, self.a_scale)
        self.A_at = self.canvas_conversion(a_b_dot2_t, self.a_scale)

        self.abs_acc_b = self.create_arrow(
            self.B, self.B, 'red', self.arrow_size)
        self.abs_vel_b = self.create_arrow(
            self.B, self.B, 'green', self.arrow_size)
        self.abs_acc_a = self.create_arrow(
            self.A, self.A, 'red', self.arrow_size)
        self.r_acc_a = self.create_arrow(
            self.A, self.A, 'orange', self.arrow_size_r)
        self.t_acc_a = self.create_arrow(
            self.A, self.A, 'orange', self.arrow_size_r)
        self.abs_vel_a = self.create_arrow(
            self.A, self.A, 'green', self.arrow_size)
        self.r_vel_a = self.create_arrow(
            self.A, self.A, 'cyan', self.arrow_size_r)

        self.point_radius = 10

        self.point_o = self.create_point(self.O)
        self.point_a = self.create_point(self.A)
        self.point_b = self.create_point(self.B)

        self.label_d = 20

        self.point_label_o = self.create_point_label("O", self.O)
        self.point_label_a = self.create_point_label("A", self.A)
        self.point_label_b = self.create_point_label("B", self.B)

        self.movement()

    def position_scale_calculation(self):
        total = l + 2 * r

        if 2 * r * self.wb < self.hb * total:
            scale = self.wb / total
        else:
            scale = self.hb / (r * 2)

        return scale

    def axes_translation_vector(self):
        xd = self.width - self.border - self.position_scale * r
        yd = self.height * 0.5
        return xd, yd

    def canvas_conversion(self, input, scale):
        return [round(scale * input[0] + self.xd), round(- scale * input[1] + self.yd)]

    def create_joint(self, point):
        return self.canvas.create_oval(point[0] - self.joint_radius, point[1] + self.joint_radius,
                                       point[0] + self.joint_radius, point[1] -
                                       self.joint_radius,
                                       fill='LightBlue2', width=self.joint_border)

    def create_arrow(self, start, end, color, size):
        return self.canvas.create_line(start[0], start[1], end[0], end[1], arrow=tk.LAST, fill=color, width=size[0],
                                       arrowshape=(size[1], size[2], size[3]))

    def move_arrow(self, arrow, start, end):
        return self.canvas.coords(arrow, start[0], start[1], end[0], end[1])

    def create_point(self, point):
        return self.canvas.create_oval(point[0] - self.point_radius, point[1] + self.point_radius,
                                       point[0] + self.point_radius, point[1] -
                                       self.point_radius,
                                       fill='black')

    def create_point_label(self, name, point):
        x = point[0] - self.label_d
        y = point[1] - self.label_d
        return self.canvas.create_text(x, y, text=name, font=("Purisa", 24))

    def set_coords(self, point, point_object, radius):
        x1 = point[0] - radius
        y1 = point[1] + radius
        x2 = point[0] + radius
        y2 = point[1] - radius
        return self.canvas.coords(point_object, x1, y1, x2, y2)

    def movement(self):

        vector_calculations()
        self.A = self.canvas_conversion(a, self.position_scale)
        self.B = self.canvas_conversion(b, self.position_scale)

        self.B_v = scalar_multiplication(b_dot, self.v_scale)
        self.B_a = scalar_multiplication(b_dot2, self.a_scale)
        self.A_v = scalar_multiplication(a_dot, self.v_scale)
        self.A_vr = scalar_multiplication(a_b_dot, self.v_scale)
        self.A_a = scalar_multiplication(a_dot2, self.a_scale)
        self.A_ar = scalar_multiplication(a_b_dot2_r, self.a_scale)
        self.A_at = scalar_multiplication(a_b_dot2_t, self.a_scale)

        self.canvas.coords(self.piston, self.A[0] - self.piston_length, self.A[1] + self.piston_height,
                           self.A[0] + self.piston_length, self.A[1] - self.piston_height)
        self.canvas.coords(
            self.beam_OB, self.O[0], self.O[1], self.B[0], self.B[1])
        self.canvas.coords(
            self.beam_AB, self.A[0], self.A[1], self.B[0], self.B[1])
        self.set_coords(self.O, self.joint_o, self.joint_radius)
        self.set_coords(self.A, self.joint_a, self.joint_radius)
        self.set_coords(self.B, self.joint_b, self.joint_radius)

        self.move_arrow(self.abs_vel_b, self.B,
                        vector_addition(self.B, self.B_v))
        self.move_arrow(self.abs_acc_b, self.B,
                        vector_addition(self.B, self.B_a))
        self.move_arrow(self.abs_vel_a, self.A,
                        vector_addition(self.A, self.A_v))
        self.move_arrow(self.r_vel_a, self.A,
                        vector_addition(self.A, self.A_vr))
        self.move_arrow(self.abs_acc_a, self.A,
                        vector_addition(self.A, self.A_a))
        self.move_arrow(self.r_acc_a, self.A,
                        vector_addition(self.A, self.A_ar))
        self.move_arrow(self.t_acc_a, self.A,
                        vector_addition(self.A, self.A_at))

        self.set_coords(self.O, self.point_o, self.point_radius)
        self.set_coords(self.A, self.point_a, self.point_radius)
        self.set_coords(self.B, self.point_b, self.point_radius)
        self.canvas.coords(self.point_label_o,
                           self.O[0] - self.label_d, self.O[1] - self.label_d)
        self.canvas.coords(self.point_label_a,
                           self.A[0] - self.label_d, self.A[1] - self.label_d)
        self.canvas.coords(self.point_label_b,
                           self.B[0] - self.label_d, self.B[1] - self.label_d)

        self.canvas.after(tick, self.movement)


diagrams = tk.Frame(top_frame)
diagrams.pack(side=tk.RIGHT, pady=5, padx=5)


class VelocityDiagram:
    def __init__(self):
        self.frame = tk.Frame(diagrams)
        self.frame.pack(side=tk.TOP, pady=5)

        self.width = 640
        self.height = 350
        self.border = 40
        self.hb = self.height - 2 * self.border
        self.wb = self.width - 2 * self.border

        self.canvas = tk.Canvas(self.frame, width=self.width, height=self.height, bg='white', highlightthickness=4,
                                highlightbackground="black")
        self.canvas.pack()

        self.scale_vectors()
        self.O = self.canvas_conversion([0, 0])
        self.A = self.canvas_conversion(a_dot)
        self.B = self.canvas_conversion(b_dot)

        # Still need arrow size calculations

        self.size = [3, 10, 12, 5]

        self.A_arrow = self.create_arrow(self.O, self.A, 'green', self.size)
        self.B_arrow = self.create_arrow(self.O, self.B, 'green', self.size)
        self.AB_arrow = self.create_arrow(self.B, self.A, 'cyan', self.size)

        self.label_d = 16

        self.O_label = self.create_point_label('O', self.O)
        self.A_label = self.create_point_label('A', self.A)
        self.B_label = self.create_point_label('B', self.B)

        self.movement()

    def create_arrow(self, start, end, color, size):
        return self.canvas.create_line(start[0], start[1], end[0], end[1], arrow=tk.LAST, fill=color, width=size[0],
                                       arrowshape=(size[1], size[2], size[3]))

    def scale_vectors(self):
        x = [0] + [a_dot[0]] + [b_dot[0]]
        min_x = min(x)
        delta_x = max(x) - min_x

        y = [0] + [a_dot[1]] + [b_dot[1]]
        max_y = max(y)
        delta_y = max_y - min(y)

        if delta_x * self.hb >= delta_y * self.wb:
            self.scale = self.wb / delta_x
        else:
            self.scale = self.hb / delta_y

        self.xd = self.border - min_x * self.scale
        self.yd = self.border + max_y * self.scale

    def canvas_conversion(self, input):
        return [round(self.scale * input[0] + self.xd), round(- self.scale * input[1] + self.yd)]

    def move_arrow(self, arrow, start, end):
        return self.canvas.coords(arrow, start[0], start[1], end[0], end[1])

    def scale_arrow(self, arrow, size):
        return self.canvas.itemconfigure(arrow, width=size[0], arrowshape=(size[1], size[2], size[3]))

    def create_point_label(self, name, point):
        x = point[0] - self.label_d
        y = point[1] - self.label_d
        return self.canvas.create_text(x, y, text=name, font=("Purisa", 12))

    def movement(self):

        self.scale_vectors()
        self.O = self.canvas_conversion([0, 0])
        self.A = self.canvas_conversion(a_dot)
        self.B = self.canvas_conversion(b_dot)

        self.move_arrow(self.A_arrow, self.O, self.A)
        self.move_arrow(self.B_arrow, self.O, self.B)
        self.move_arrow(self.AB_arrow, self.B, self.A)

        self.canvas.coords(
            self.O_label, self.O[0] - self.label_d, self.O[1] - self.label_d)
        self.canvas.coords(
            self.A_label, self.A[0] - self.label_d, self.A[1] - self.label_d)
        self.canvas.coords(
            self.B_label, self.B[0] - self.label_d, self.B[1] - self.label_d)

        self.canvas.after(tick, self.movement)


class AccelerationDiagram:
    def __init__(self):
        self.frame = tk.Frame(diagrams)
        self.frame.pack(side=tk.BOTTOM, pady=5)

        self.width = 640
        self.height = 350
        self.border = 40
        self.hb = self.height - 2 * self.border
        self.wb = self.width - 2 * self.border

        self.canvas = tk.Canvas(self.frame, width=self.width, height=self.height, bg='white', highlightthickness=4,
                                highlightbackground="black")
        self.canvas.pack()

        self.scale_vectors()
        self.O = self.canvas_conversion([0, 0])
        self.A = self.canvas_conversion(a_dot2)
        self.B = self.canvas_conversion(b_dot2)
        self.AB_r = self.canvas_conversion(a_b_dot2_r)

        # Still need arrow size calculations

        self.size = [3, 10, 12, 5]
        self.size_r = [9, 30, 12, 15]

        self.A_arrow = self.create_arrow(self.O, self.A, 'red', self.size)
        self.B_arrow = self.create_arrow(self.O, self.B, 'red', self.size)
        self.AB_r_arrow = self.create_arrow(
            self.B, self.AB_r, 'orange', self.size)
        self.AB_t_arrow = self.create_arrow(
            self.AB_r, self.A, 'orange', self.size)

        self.label_d = 16

        self.O_label = self.create_point_label('O', self.O)
        self.A_label = self.create_point_label('A', self.A)
        self.B_label = self.create_point_label('B', self.B)

        self.movement()

    def create_arrow(self, start, end, color, size):
        return self.canvas.create_line(start[0], start[1], end[0], end[1], arrow=tk.LAST, fill=color, width=size[0],
                                       arrowshape=(size[1], size[2], size[3]))

    def scale_vectors(self):
        x = [0] + [a_dot2[0]] + [b_dot2[0] + a_b_dot2_r[0]] + \
            [b_dot2[0]]  # + [a_b_dot[0]]
        min_x = min(x)
        delta_x = max(x) - min_x

        y = [0] + [a_dot2[1]] + [b_dot2[1] +
                                 a_b_dot2_r[1]] + [b_dot2[1]]  # + [a_b_dot[1]
        max_y = max(y)
        delta_y = max_y - min(y)

        if delta_x * self.hb >= delta_y * self.wb:
            self.scale = self.wb / delta_x
        else:
            self.scale = self.hb / delta_y

        self.xd = self.border - min_x * self.scale
        self.yd = self.border + max_y * self.scale

    def canvas_conversion(self, input):
        return [round(self.scale * input[0] + self.xd), round(- self.scale * input[1] + self.yd)]

    def move_arrow(self, arrow, start, end):
        return self.canvas.coords(arrow, start[0], start[1], end[0], end[1])

    def scale_arrow(self, arrow, size):
        return self.canvas.itemconfigure(arrow, width=size[0], arrowshape=(size[1], size[2], size[3]))

    def create_point_label(self, name, point):
        x = point[0] - self.label_d
        y = point[1] - self.label_d
        return self.canvas.create_text(x, y, text=name, font=("Purisa", 12))

    def movement(self):

        self.scale_vectors()
        self.O = self.canvas_conversion([0, 0])
        self.A = self.canvas_conversion(a_dot2)
        self.B = self.canvas_conversion(b_dot2)
        self.AB_r = self.canvas_conversion(
            [b_dot2[0] + a_b_dot2_r[0], b_dot2[1] + a_b_dot2_r[1]])

        self.move_arrow(self.A_arrow, self.O, self.A)
        self.move_arrow(self.B_arrow, self.O, self.B)
        self.move_arrow(self.AB_r_arrow, self.B, self.AB_r)
        self.move_arrow(self.AB_t_arrow, self.AB_r, self.A)

        self.canvas.coords(
            self.O_label, self.O[0] - self.label_d, self.O[1] - self.label_d)
        self.canvas.coords(
            self.A_label, self.A[0] - self.label_d, self.A[1] - self.label_d)
        self.canvas.coords(
            self.B_label, self.B[0] - self.label_d, self.B[1] - self.label_d)

        self.canvas.after(tick, self.movement)


bottom_frame = tk.Frame(root)
bottom_frame.pack(side=tk.BOTTOM)

system_input_frame = tk.Frame(bottom_frame)
system_input_frame.pack(side=tk.RIGHT)


class Timer:
    def __init__(self):

        self.frame = tk.Frame(system_input_frame)
        self.frame.pack(side=tk.RIGHT)

        self.t = 0
        self.LastTime = time()
        self.timescale = float(1)

        self.display = tk.Label(self.frame, width=20)
        self.display.pack()

        self.button_frame = tk.Frame(self.frame)
        self.button_frame.pack()

        self.StartButton = tk.Button(
            self.button_frame, text='Start', command=self.toggle)
        self.StartButton.pack(side=tk.LEFT)

        self.ResetButton = tk.Button(
            self.button_frame, text='Reset', command=self.reset)
        self.ResetButton.pack(side=tk.RIGHT)

        self.SpeedSlider = tk.Scale(self.frame, from_=0.1, to=1, resolution=0.1, orient=tk.HORIZONTAL,
                                    command=self.speed)
        self.SpeedSlider.set(self.timescale)
        self.SpeedSlider.pack()

        self.run = 0

        self.run_timer()

    def toggle(self):
        if self.run == 0:
            self.run = 1.0
            self.StartButton.config(text='Pause')
        else:
            self.run = 0.0
            self.StartButton.config(text='Start')

    def reset(self):
        self.run = 0.0

        # global t
        # t = 0
        self.t = 0

        global phi
        phi = 0

        self.StartButton.config(text='Start')

    def speed(self, x):
        self.timescale = float(x)

    def run_timer(self):

        time_block = time() - self.LastTime
        self.LastTime = time()

        demo_time = self.run * self.timescale * time_block

        # global t

        self.t += demo_time

        global phi

        phi += ang_vel * demo_time

        time_string = str(round(self.t, 1))

        self.display.config(text="t = " + time_string + "s")
        self.display.after(tick, self.run_timer)


vector_calculations()

Timer()

VelocityDiagram()

AccelerationDiagram()

PhysicalModel()

root.mainloop()
