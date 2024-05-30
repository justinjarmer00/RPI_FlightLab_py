import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

AOA = 1
AOA_list = np.linspace(-5,7,101)
AOA_index = np.argmin(np.abs(AOA_list - AOA))
print(AOA_index)
print(AOA_list[AOA_index])
CG = [2.188, 0]
W_AC = [2.454, 0]
MAC = 16.3/12
T_LE = 4.836
T_height = 1.043
T_AC = [T_LE + 0.448, T_height]
F_EDF = 18.88 # 84 Newtons
EDFxy = [2.51, 0.2257]
EDF_a = 0

### CLACULATING THE MOMENTS:
cm_ac_wb = -0.12 # first term
zero_lift_wign = -4.30 #look this up


def CM_lwb(a_wb, abs_AOA, h, h_ac_wb):
    """
    Calculate the moment coefficient aournd the cg from the wign via lift and leverarm

    Parameters:
    - a_wb: Lift curve slope of the wing-body combination
    - abs_AOA: absolute angle of attack for the wing-body
    - h: nondimensional dist of CG from LE
    - h_ac_wb: nondimensional dist of wing-body ac from LE

    Returns:
    - Second-Term: Pitching moment coefficient due to lift and location of cg
    """
    return a_wb * abs_AOA * (h - h_ac_wb)
    # return a_wb * abs_AOA * (h - h_ac_wb - V_H * (a_t/a_wb) * (1 - d_epsilon_d_alpha))

def CM_tail(a_t, abs_AOA, V_H, d_epsilon_d_alpha, alpha_i, epsilon_0):
    """
    Calculate the moment coefficient aournd the cg from the tail

    Parameters:
    - a_t: Lift curve slope of the tail
    - abs_AOA: absolute angle of attack for the wing-body
    - V_H: Tail volume coefficient.  
    - d_epsilon_d_alpha: Downwash gradient
    - alpha_i: Angle between zero lift line of wb and zero lift line of tail
    - epsilon_0: Downwash angle at zero lift

    Returns:
    - Second-Term: Pitching moment coefficient due to lift and location of cg
    """
    return (-1) * a_t * abs_AOA * V_H *(1 - d_epsilon_d_alpha) + a_t * V_H * (epsilon_0 + alpha_i )

def cm_to_moment(Cm, rho, V, S, c):
    """
    Convert dimensionless pitching moment coefficient to pitching moment in imperial units.

    Parameters:
    - Cm: Pitching moment coefficient (dimensionless).
    - rho: Air density (slugs/ft^3).
    - V: True airspeed (ft/s).
    - S: Wing reference area (ft^2).
    - c: Mean aerodynamic chord (ft).

    Returns:
    - M: Pitching moment (lb·ft).
    """
    q = 0.5 * rho * V**2  # Dynamic pressure (lb/ft^2)
    M = Cm * q * S * c  # Pitching moment (lb·ft)
    return M

# Initialize variables for cm_to_moment function
rho = 0.0023769  # Air density at sea level (slug/ft^3)
V = 144.36  # True airspeed (ft/s)
S = 13.08  # Wing reference area (ft^2)
T_dist_ac = 0.448 # dist tail leading edge to ac, 0.448 is a very rough approximation from NOT area weighted average and good 1/4 assumption
l_t = T_dist_ac + T_LE - CG[0] # dist between tail ac and airplane CG
s_t = 1.61 #full tail surface area, will need to be haved

# Initialize variables for second_term function
a_wb = (0.11426)*np.cos(np.deg2rad(35)) # adjust wing slope for sweep angle 
h = CG[0]/MAC  # Dimensionless distance of CG from the reference point
h_ac_wb = W_AC[0]/MAC  # Dimensionless distance of the aerodynamic center of wing-body from the reference point
V_H = (l_t*s_t)/(MAC*S)
a_t = 0.10143 # slope of tail cl curve
d_epsilon_d_alpha = 0  # Downwash gradient

# Initialize variables for third_term function
alpha_i = - zero_lift_wign  # Angle between zero lift line of wb and zero lift line of tail
epsilon_0 = 0  # Downwash angle at zero lift

moment_ac_wb = []
moment_lwb = []
moment_tail = []
moment_EDF = []
moment_total = []

force_lwb = []
force_tail = []
force_EDF = []

for i,a in enumerate(AOA_list):
    # abs_AOA = AOA - zero_lift_wign
    abs_AOA = a - zero_lift_wign

    # Call functions with initialized variables to calculate terms and moment
    cm_lwb = CM_lwb(a_wb, abs_AOA, h, h_ac_wb)
    # print(cm_lwb)
    cm_tail = CM_tail(a_t, abs_AOA, V_H, d_epsilon_d_alpha, alpha_i, epsilon_0)
    # print(cm_tail)
    moment_ac_wb.append(cm_to_moment(cm_ac_wb, rho, V, S, MAC))
    moment_lwb.append(cm_to_moment(cm_lwb, rho, V, S, MAC))
    moment_tail.append(cm_to_moment(cm_tail, rho, V, S, MAC))
    moment_EDF.append(-F_EDF*np.cos(np.deg2rad(EDF_a))*EDFxy[1] - F_EDF*np.sin(np.deg2rad(EDF_a))*EDFxy[0])
    moment_total.append(moment_ac_wb[i] + moment_lwb[i] + moment_tail[i] + moment_EDF[i])
    # print(moment_ac_wb)
    # print(moment_lwb)
    # print(moment_tail)
    # print(moment_EDF)

    force_lwb.append(moment_lwb[i]/(CG[0] - W_AC[0]))
    force_tail.append(moment_tail[i]/-l_t)
    force_EDF.append(F_EDF)

### Plot ###


### RENDER ###
# Turn on interactive mode
plt.ion()

### Moments Graph
plt.figure()
plt.plot(AOA_list, moment_ac_wb, linestyle="--", color="red", label='Wing Airfoil Moment')
plt.plot(AOA_list, moment_lwb, linestyle="--", color="green", label='WB Moment from Lift')
plt.plot(AOA_list, moment_tail, linestyle="--", color="orange", label='Tail Moment')
plt.plot(AOA_list, moment_EDF, linestyle="--", color="black", label='EDF Moment')
plt.plot(AOA_list, moment_total, linestyle="-", color="blue", label='Total Pitching Moment')
plt.grid(True)
plt.title(f"Moments vs AOA (V={V} ft/s)")
plt.xlabel("AOA")
plt.ylabel("Moment (ft*lbf)")
plt.legend()
plt.show()

### Side View Drawing
def add_ellipse_points(x, y, x_center, y_center, major_axis, minor_axis, num_points=100, half='both'):
    if half == 'front':
        theta = np.linspace(np.pi / 2, 3 * np.pi / 2, num_points)  # Front half (quadrants 2 and 3)
    elif half == 'back':
        theta = np.concatenate((np.linspace(3 * np.pi / 2, 2 * np.pi, num_points // 2),
                                np.linspace(0, np.pi / 2, num_points // 2)))  # Back half (quadrants 4 and 1)
    else:
        theta = np.linspace(0, 2 * np.pi, num_points)  # Full ellipse
    
    x_ellipse = x_center + major_axis * np.cos(theta)
    y_ellipse = y_center + minor_axis * np.sin(theta)
    x.extend(x_ellipse)
    y.extend(y_ellipse)
    
    return x, y

def rotate_points(x, y, origin, angle_degrees):
    angle_radians = np.radians(angle_degrees)  # Convert angle from degrees to radians
    x_origin, y_origin = origin

    # Function to rotate a single point
    def rotate_single_point(x, y):
        return (
            x_origin + (x - x_origin) * np.cos(angle_radians) - (y - y_origin) * np.sin(angle_radians),
            y_origin + (x - x_origin) * np.sin(angle_radians) + (y - y_origin) * np.cos(angle_radians)
        )

    # Check if x and y are iterable (lists) or single numeric values
    if np.isscalar(x) and np.isscalar(y):
        return rotate_single_point(x, y)
    else:
        # Apply rotation to each point in the list
        x_rotated, y_rotated = [], []
        for xi, yi in zip(x, y):
            xr, yr = rotate_single_point(xi, yi)
            x_rotated.append(xr)
            y_rotated.append(yr)
        return x_rotated, y_rotated

# Initialize empty lists for x and y coordinates
x1 = []
y1 = []
x2 = []
y2 = []
x23 = []
y23 = []
x24 = []
y24 = []
x3 = []
y3 = []
x4 = []
y4 = []
x5 = []
y5 = []
x6 = []
y6 = []

# Add ellipses to represent different parts of the aircraft
Fcord = 2.513
Fle1 = 0
Fthickness = 0.09*Fcord
x1, y1 = add_ellipse_points(x1, y1, Fle1 + Fcord/2, 0, Fcord/2, Fthickness)  # Fuselage

# tail

Tcord = 0.499

T_LE2 = T_LE + 0.584
Tthickness = 0.09*Tcord
x2, y2 = add_ellipse_points(x2, y2, T_LE + Tcord/2, T_height, Tcord/2, 1.5*Tthickness, half='front')  # Tail
x2, y2 = add_ellipse_points(x2, y2, T_LE2 + Tcord/2, T_height, Tcord/2, Tthickness, half='back')  # Wing
x2.append(x2[0])
y2.append(y2[0])
x23 = [T_LE, T_LE2]
y23 = [T_height,T_height]
x24, y24 = add_ellipse_points(x24, y24, T_LE2 + Tcord/2, T_height, Tcord/2, Tthickness, half='front')

# wing
cord = 1.220
LE1 = 0.850
LE2 = 3.98
thickness = 0.09*cord
x3, y3 = add_ellipse_points(x3, y3, LE1 + cord/2, 0, cord/2, thickness, half='front')  # Wing 
x3, y3 = add_ellipse_points(x3, y3, LE2 + cord/2, 0, cord/2, thickness, half='back')  # Wing
x3.append(x3[0])
y3.append(y3[0])
x4 = [LE1, LE2]
y4 = [0,0]
x5, y5 = add_ellipse_points(x3, y3, LE2 + cord/2, 0, cord/2, thickness, half='front')

x6, y6 = add_ellipse_points(x6, y6, EDFxy[0], EDFxy[1], 0.3, 0.1, half='back')
x6.append(x6[0])
y6.append(y6[0])

### AOA ROTATIONS
x1,y1 = rotate_points(x1, y1, [0,0], -AOA_list[AOA_index])
x2,y2 = rotate_points(x2, y2, [0,0], -AOA_list[AOA_index])
x23,y23 = rotate_points(x23, y23, [0,0], -AOA_list[AOA_index])
x24,y24 = rotate_points(x24, y24, [0,0], -AOA_list[AOA_index])
x3,y3 = rotate_points(x3, y3, [0,0], -AOA_list[AOA_index])
x4,y4 = rotate_points(x4, y4, [0,0], -AOA_list[AOA_index])
x5,y5 = rotate_points(x5, y5, [0,0], -AOA_list[AOA_index])
CG = rotate_points(CG[0], CG[1], [0,0], -AOA_list[AOA_index])
W_AC = rotate_points(W_AC[0], W_AC[1], [0,0], -AOA_list[AOA_index])
T_AC = rotate_points(T_AC[0], T_AC[1], [0,0], -AOA_list[AOA_index])

EDFxy  = rotate_points(EDFxy[0], EDFxy[1], [0,0], -AOA_list[AOA_index])
x6,y6 = rotate_points(x6, y6, [0,0], -AOA_list[AOA_index])
x6,y6 = rotate_points(x6, y6, [EDFxy[0], EDFxy[1]], -EDF_a)

# Plot the aircraft representation
fig, ax = plt.subplots()

forces = np.array([force_lwb[AOA_index], force_tail[AOA_index], force_EDF[AOA_index]])
max_force = np.max(np.abs(forces))
normalized_forces = 2*forces / max_force  # Normalize the forces to the largest one

# Set the arrow length so that the longest arrow does not exceed this length on the plot.
max_arrow_length = 1.0

# Calculate the actual lengths of the arrows to be drawn.
arrow_lengths = normalized_forces * max_arrow_length

# Define a function to draw the force arrows on the plot.
def draw_force_arrow(ax, origin, length, angle=0, color='blue', label=''):
    ax.arrow(origin[0], origin[1], 
             length * np.cos(np.radians(angle)), 
             length * np.sin(np.radians(angle)), 
             head_width=0.05, head_length=0.1, fc=color, ec=color, label=label)

def draw_moment(ax, center, radius, start_angle, end_angle, direction, color='blue', label=''):
    """
    Draw a moment (torque) as an arc with an arrow.

    Parameters:
    - ax: The axes object to draw the moment.
    - center: The (x, y) coordinates of the point about which the moment is applied.
    - radius: The radius of the arc representing the moment.
    - start_angle, end_angle: Angles defining the start and end points of the arc.
    - direction: Direction of the moment ('cw' for clockwise, 'ccw' for counterclockwise).
    - color: Color of the moment arc and arrow.
    - label: Label for the moment in the legend.
    """
    # Add an arc to represent the moment
    arc = patches.Arc(center, 2*radius, 2*radius, angle=0, theta1=start_angle, theta2=end_angle, color=color, label=label)
    ax.add_patch(arc)

    # Depending on the direction, choose where to place the arrow for the moment
    if direction == 'cw':
        angle = np.radians(end_angle)
        arrow_start_angle = angle - np.pi / 2
    else:  # 'ccw'
        angle = np.radians(start_angle)
        arrow_start_angle = angle + np.pi / 2

    # Calculate the position of the arrowhead
    arrow_x = center[0] + radius * np.cos(angle)
    arrow_y = center[1] + radius * np.sin(angle)
    arrow_dx = -0.01 * np.cos(arrow_start_angle)
    arrow_dy = -0.01 * np.sin(arrow_start_angle)

    ax.arrow(arrow_x, arrow_y, arrow_dx, arrow_dy, head_width=0.05, head_length=0.07, fc=color, ec=color)

# Use the function to draw the arrows.
# fig, ax = plt.subplots()
draw_force_arrow(ax, W_AC, arrow_lengths[0], angle=90 - AOA, color='green', label='Lift Wing-Body')
draw_force_arrow(ax, T_AC, arrow_lengths[1], angle=90 - AOA, color='orange', label='Lift Tail')
draw_force_arrow(ax, EDFxy, arrow_lengths[2], angle=180 - AOA - EDF_a, color='black', label='EDF Thrust')
draw_moment(ax, center=W_AC, radius=0.37, start_angle=-30, end_angle=240, direction='ccw' if -moment_ac_wb[AOA_index] < 0 else "cw", color='red', label='Moment from Airfoil')

ax.plot(x1, y1, linestyle='-') 
ax.plot(x2, y2, linestyle='-', color='orange')  
ax.plot(x23, y23, linestyle='-', color='orange')
ax.plot(x24, y24, linestyle='-', color='orange') 

ax.plot(x3, y3, linestyle='-', color='green')  
ax.plot(x4, y4, linestyle='-', color='green')
ax.plot(x5, y5, linestyle='-', color='green')   

ax.plot(x6, y6, linestyle='-', color='black')

ax.plot(CG[0], CG[1], marker='o', color='red', label='CG')
ax.plot(W_AC[0], W_AC[1], marker='o', color='green', label='Wing-Body AC')
ax.plot(T_AC[0], T_AC[1], marker='o', color='orange', label='Tail AC')

# Set title, labels, grid, and axis properties
ax.set_title(f"X-56 FBD (AOA={AOA_list[AOA_index]:.2})")
ax.set_xlabel("X-axis")
ax.axis('equal')
ax.legend()

# Show the plot
# Turn off interactive mode
plt.ioff()
plt.show()
