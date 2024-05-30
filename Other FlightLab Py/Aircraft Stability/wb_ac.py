import numpy as np
import matplotlib.pyplot as plt

class Func_LE:
    # For leading edge
    def __init__(self) -> None:
        pass
    @staticmethod
    def Value(y):
        if y <= 0.528:
            return 1.607*y
        elif y <= 0.860:
            return 0.850 + 0.700*(y - 0.528) # 0.850 + 0.619*(y - 0.528)
        elif y <= 5:
            return 1.082 + 0.700*(y - 0.860)
        else:
            return 0  # or some appropriate value for y > 5
        
class Func_CD:
    # For leading edge
    def __init__(self) -> None:
        pass
    @staticmethod
    def Value(y):
        if y <= 0.528:
            return 2.513 - 2.006*y
        elif y <= 0.860:
            return 1.453 - 0.703*(y - 0.528)
        elif y <= 4.426:
            return 1.220
        elif y <= 5:
            return 1.220 # - (0.700 + 1.428)*(y - 4.426)
        else:
            return 0  # or some appropriate value for y > 5

class Func_LE2:
    # For leading edge
    def __init__(self) -> None:
        pass
    @staticmethod
    def Value(y):
        if y <= 5:
            return 0.464 + 0.700*(y) # 0.522 + 0.619*(y)
        else:
            return 0  # or some appropriate value for y > 5
        
class Func_CD2:
    # For leading edge
    def __init__(self) -> None:
        pass
    @staticmethod
    def Value(y):
        if y <= 4.426:
            return 1.220
        elif y <= 5:
            return 1.220 # - (0.700 + 1.428)*(y - 4.426)
        else:
            return 0  # or some appropriate value for y > 5

dy = 0.001

LE = []
LE2 = []
TE = []
TE2 = []
AC = []
AC2 = []
Y = []
Y2 = []
y = 0

airfoil_ac = 0.275
S = 0
S2 = 0
LEa = 0
LEa2 = 0
MAC = 0
MAC2 = 0
ac_SUM = 0
ac_v =[]
ac_p = []
ac_s = []
ac_SUM2 = 0
ac_v2 = []
ac_p2 = []
ac_s2 = []
while y < 5:
    S += Func_CD.Value(y)*dy
    S2 += Func_CD2.Value(y)*dy
    y += dy
S = S #- 0.35
S2 = S2 #- 0.35
print(f"Wing Area:\t\t{S2:.2f}\tft^2\t\t\tWing/Body Area:\t\t{S:.2f}\tft^2")
y = 0
while y < 5:
    Y.append(y)
    LE.append(Func_LE.Value(y))
    TE.append(Func_LE.Value(y) + Func_CD.Value(y))
    Y2.append(-y)
    LE2.append(Func_LE2.Value(y))
    TE2.append(Func_LE2.Value(y) + Func_CD2.Value(y))
    AC.append(Func_LE.Value(y) + airfoil_ac*Func_CD.Value(y))
    AC2.append(Func_LE2.Value(y) + airfoil_ac*Func_CD2.Value(y))

    LEa += Func_LE.Value(y)*Func_CD.Value(y)*dy/S
    LEa2 += Func_LE2.Value(y)*Func_CD2.Value(y)*dy/S2

    MAC += Func_CD.Value(y)*Func_CD.Value(y)*dy/S
    MAC2 += Func_CD2.Value(y)*Func_CD2.Value(y)*dy/S2

    # ac plots
    ac_value = Func_LE.Value(y) + airfoil_ac*Func_CD.Value(y)
    ac_valuew = ac_value*Func_CD.Value(y)*dy/S
    ac_SUM += ac_valuew
    ac_v.append(ac_value*12)
    ac_p.append(ac_valuew)
    ac_s.append(ac_SUM*12)

    ac_value2 = Func_LE2.Value(y) + airfoil_ac*Func_CD2.Value(y)
    ac_valuew2 = ac_value2*Func_CD2.Value(y)*dy/S2
    ac_SUM2 += ac_valuew2
    ac_v2.append(ac_value2*12)
    ac_s2.append(ac_SUM2*12)
    ac_p2.append(ac_valuew2)

    cd = (Func_CD.Value(y) + Func_CD.Value(y + dy)) * dy / 2

    y += dy


print(f"Wing AC:\t\t{ac_SUM2*12:.2f}\tin\t\t\t\tWing/Body AC:\t\t{ac_SUM*12:.2f}\tin")
print(f"Wing MAC:\t\t{MAC2*12:.2f}\tin\t\t\t\tWing/Body MAC:\t\t{MAC*12:.2f}\tin")

# TAIL
T_LE = 4.836 #leading edge difference between wing and tail
Tail_x = [0 + T_LE, 0.584 + T_LE, 1.083 + T_LE, 0.751 + T_LE, 1.083 + T_LE, 0.584 + T_LE, 0 + T_LE]
Tail_y = [0, 1.25, 1.25, 0, -1.25, -1.25, 0]
T_dist_ac = 0.448 # dist tail leading edge to ac, 0.448 is a very rough approximation from NOT area weighted average and good 1/4 assumption
CG_dist = 2.188 # dist from CG to leading edge of aircraft
l_t = T_dist_ac + T_LE - CG_dist # dist between tail ac and airplane CG
s_t = 1.61 #full tail surface area, will need to be haved

v_h = (l_t*s_t)/(MAC*S*2) #S is 1/2 total wing area, where as s_t is full tail area. need to add 1/2 factor on s_t
v_h2 = (l_t*s_t)/(MAC2*S2*2)

a_t = 0.10143 # slope of tail cl curve
a_w = (0.11426)*np.cos(np.deg2rad(35)) # adjust wing slope for sweep angle

NP = ac_SUM + v_h*(a_t/a_w)
NP2 = ac_SUM2 + v_h2*(a_t/a_w)
print(f"Wing-Tail NP:\t{NP2*12:.2f}\tin\t\t\t\tWing/Body-Tail NP:\t{NP*12:.2f}\tin")

plt.figure(figsize=(10, 6))
Ytotal = Y + Y[::-1]
Edge = LE + TE[::-1]
Ytotal2 = Y2 + Y2[::-1]
Edge2 = LE2 + TE2[::-1]
plt.plot( Ytotal, Edge, linestyle='-')  # Line plot connecting the points
plt.plot(Y, AC, linestyle='-', color='red')  # Highlight the individual points
plt.plot( Ytotal2, Edge2, linestyle='-')  # Line plot connecting the points
plt.plot(Y2, AC2, linestyle='-', color='green')  # Highlight the individual points
plt.plot([0,5], [ac_SUM, ac_SUM], linestyle='--', color='red')
plt.plot([-5,0], [ac_SUM2, ac_SUM2], linestyle='--', color='green')
plt.plot([0,5], [NP, NP], linestyle='--', color='blue')
plt.plot([-5,0], [NP2, NP2], linestyle='--', color='orange')
plt.plot(Tail_y, Tail_x, color='grey')

plt.title('Plot of X vs. Y')
plt.xlabel('X axis')
plt.ylabel('Y axis')
plt.grid(True)
plt.axis('equal')  # Set equal scaling on both axes
plt.gca().invert_yaxis()  # Invert the y-axis
plt.axis('equal')  # Set equal scaling on both axes
plt.show()

# Plot vplot against tplot

fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(16, 6))  # 1 row, 3 columns

# Plot 1 on ax1
ax1.plot(Y, ac_v, label='Wing/Body')
ax1.plot(Y, ac_v2, label='Wing')
ax1.set_title('AC location')
ax1.set_xlabel('Span (ft)')
ax1.set_ylabel('AC Dist from LE (in)')
ax1.set_ylim(bottom=0)
ax1.legend()
ax1.grid(True)

# Plot 2 on ax2
ax2.plot(Y, ac_p, label='Wing/Body')
ax2.plot(Y, ac_p2, label='Wing')
ax2.set_title('Area Weighted')
ax2.set_xlabel('Span (ft)')
ax2.set_ylabel('AC dist / delta A')
ax2.set_ylim(bottom=0)
ax2.legend()
ax2.grid(True)

# Plot 3 on ax3
ax3.plot(Y, ac_s, label=f'Wing/Body: {12*ac_SUM:.2f} in')
ax3.plot(Y, ac_s2, label=f'Wing: {12*ac_SUM2:.2f} in')
ax3.set_title('AC average progression')
ax3.set_xlabel('Span (ft)')
ax3.set_ylabel('Area Weighted AC (in)')
ax3.legend()
ax3.grid(True)

# Adjust layout to prevent overlap
plt.tight_layout()
plt.show()