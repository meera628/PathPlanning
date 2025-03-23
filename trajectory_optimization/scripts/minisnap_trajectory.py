#!/usr/bin/env python3
import sympy
import numpy as np
from cvxopt import matrix, solvers
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
def load_waypoints(filename):
    """Loads waypoints from a file (space-separated)."""
    return np.loadtxt(filename)

def time_allocation(waypoints, vel_max, acc_max):
    num_seg= len(waypoints)-1
    time_alloc_arr=[]
    for i in range(num_seg):
        d_i= np.linalg.norm(waypoints[i+1] - waypoints[i])
        #print(waypoints[i], d_i)
        # Compute min distance required for full acceleration and deceleration
        d_acc = (vel_max ** 2) / (2 * acc_max)
        
        if d_i >= 2 * d_acc:
            # Full trapezoidal profile
            t_acc = vel_max / acc_max
            t_const = (d_i - 2 * d_acc) / acc_max
            t_dec = t_acc
            T_i = t_acc + t_const + t_dec
        else:
            # Triangle profile (does not reach max velocity)
            v_peak = np.sqrt(2 * acc_max * d_i)
            t_acc = v_peak / acc_max
            T_i = 2 * t_acc  # Symmetric acceleration and deceleration

        time_alloc_arr.append(T_i)
    
    return time_alloc_arr

def compute_Q_submatrix_sym():
    t, T = sympy.symbols('t T')
    p0, p1, p2, p3, p4, p5, p6, p7 = sympy.symbols('p0 p1 p2 p3 p4 p5 p6 p7')

    P = p7*t**7 + p6*t**6 + p5*t**5 + p4*t**4 + p3*t**3 + p2*t**2 + p1*t + p0
    P1 = sympy.diff(P, t) # Speed
    P2 = sympy.diff(P1, t) # Acceleration
    P3 = sympy.diff(P2, t) # Jerk
    P4 = sympy.diff(P3, t) # Snap
    J = sympy.integrate(P4**2, (t, 0, T))
    Q = sympy.hessian(J, (p0, p1, p2, p3, p4, p5, p6, p7))

    #sympy.pprint(Q)
    return Q

def get_Q(T):
    # Q_evaluated = Q.subs(T, T_value)
    # return np.array(Q_evaluated).astype(np.float64)
    return np.array([
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1152*T, 2880*T**2, 5760*T**3, 10080*T**4],
        [0, 0, 0, 0, 2880*T**2, 9600*T**3, 21600*T**4, 40320*T**5],
        [0, 0, 0, 0, 5760*T**3, 21600*T**4, 51840*T**5, 100800*T**6],
        [0, 0, 0, 0, 10080*T**4, 40320*T**5, 100800*T**6, 201600*T**7]
    ])
    
def get_coefficients(T, wp):
    N = 8 # Septic polynomial has 8 coefficients
    M = len(T) # Number of polynomial segments
    L = (M)+ (M)+ (4*(M-1)) # Number of constraints
    #print(M, L)
    # Q Matrix
    Z = np.zeros((N, N))
    Q_list = [get_Q(T[i]) for i in range(M)]
    Q = np.block([[Q_list[i] if i == j else Z for j in range(M)] for i in range(M)])

    f = np.zeros(N*M)

    # Constraints (Ap=b)
    A = np.zeros((L, N*M))
    #print(A.shape)
    for k in range(0,M):
        # Start position of polynomials
        i=k
        A[k,(8*i):(8*(i+1))]= [1, 0, 0, 0, 0, 0, 0, 0]
    
    for k in range(M, 2*M):
        # End position of polynomials
        i= k-(M)
        A[k,(8*i):(8*(i+1))]= [1, T[i], T[i]**2, T[i]**3, T[i]**4, T[i]**5, T[i]**6, T[i]**7]

    for k in range (2*M, 3*M-1):
        # Continuous velocity
        i= k- (2*M)
        A[k, (8*i): ((8*i)+16)] = [0, 1, 2*T[i], 3*T[i]**2, 4*T[i]**3, 5*T[i]**4, 6*T[i]**5, 7*T[i]**6, 0, -1, 0, 0, 0, 0, 0, 0]
    for k in range(3*M-1, 4*M-2):
        i= k- (3*M-1)
        A[k, (8*i): ((8*i)+16)] = [0, 0, 2, 6*T[i], 12*T[i]**2, 20*T[i]**3, 30*T[i]**4, 42*T[i]**5, 0, 0, -2, 0, 0, 0, 0, 0]
    for k in range(4*M-2, 5*M-3):
        i= k- (4*M-2)
        A[k, (8*i): ((8*i)+16)] = [0, 0, 0, 6, 24*T[i], 60*T[i]**2, 120*T[i]**3, 210*T[i]**4, 0, 0, 0, -6, 0, 0, 0, 0]
    for k in range(5*M-3, 6*M-4):
        i= k- (5*M-3)
        A[k, (8*i): ((8*i)+16)]=  [0, 0, 0, 0, 24, 120*T[i], 360*T[i]**2, 840*T[i]**3, 0, 0, 0, 0, -24, 0, 0, 0]


    b = np.zeros(L)
    for k in range(M):
        b[k]=wp[k]
    for k in range(M, 2*M):
        i= k- (M)
        b[k]= wp[i+1]

    sol = solvers.qp(matrix(Q), matrix(f), None, None, matrix(A), matrix(b))
    return list(sol['x'])

def plot_3d_traj(p_x,p_y, p_z,times,wp_x, wp_y, wp_z ):
    #Evaluate trajectory
    N = 500
    t = np.linspace(times[0], times[-1], N)
    positions = {'x': [], 'y': [], 'z': []}

    for i in range(N):
        j = np.nonzero(t[i] <= times)[0][0] - 1
        j = np.max([j, 0])
        ti = t[i] - times[j]

        for axis, p in zip(['x', 'y', 'z'], [p_x, p_y, p_z]):
            coeff = np.flip(p[8 * j:8 * j + 8])
            positions[axis].append(np.polyval(coeff, ti))

    # Plot trajectory
    plt.figure(figsize=(10, 5))
    ax = plt.axes(projection='3d')
    ax.plot3D(positions['x'], positions['y'], positions['z'], label="Trajectory")
    ax.scatter(wp_x, wp_y, wp_z, color='red', label="Waypoints")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.show()

def plot_1d_traj(p,wp,times):
    N = 500
    t = np.linspace(times[0], times[-1], N)
    pos = [None] * N
    vel = [None] * N
    acc = [None] * N
    jrk = [None] * N
    snp = [None] * N
    for i in range(N):
        j = np.nonzero(t[i] <= times)[0][0] - 1
        j = np.max([j, 0])
        ti = t[i] - times[j]
        x_coeff = np.flip(p[8*j:8*j+8])
        v_coeff = np.polyder(x_coeff)
        a_coeff = np.polyder(v_coeff)
        j_coeff = np.polyder(a_coeff)
        s_coeff = np.polyder(j_coeff)
        pos[i] = np.polyval(x_coeff, ti)
        vel[i] = np.polyval(v_coeff, ti)
        acc[i] = np.polyval(a_coeff, ti)
        jrk[i] = np.polyval(j_coeff, ti)
        snp[i] = np.polyval(s_coeff, ti)

    plt.subplot(5, 1, 1)
    plt.plot(t, pos, label='Position')
    plt.plot(times, wp, '.', label='Waypoint')
    plt.legend()
    plt.subplot(5, 1, 2)
    plt.plot(t, vel, label='velocity')
    plt.legend()
    plt.subplot(5, 1, 3)
    plt.plot(t, acc, label='acceleration')
    plt.legend()
    plt.subplot(5, 1, 4)
    plt.plot(t, jrk, label='jerk')
    plt.legend()
    plt.subplot(5, 1, 5)
    plt.plot(t, snp, label='snap')
    plt.legend()
    plt.xlabel('Time')
    plt.show()

def plot_magnitude_traj(p_x, p_y, p_z, times, time_alloc_arr):
    """Plots the magnitudes of velocity, acceleration, and jerk."""
    N = 500
    t = np.linspace(times[0], times[-1], N)
    
    vel_mag = np.zeros(N)
    acc_mag = np.zeros(N)
    jrk_mag = np.zeros(N)

    for i in range(N):
        j = np.nonzero(t[i] <= times)[0][0] - 1
        j = np.max([j, 0])
        ti = t[i] - times[j]

        vel_components = []
        acc_components = []
        jrk_components = []

        for p in [p_x, p_y, p_z]:
            coeff = np.flip(p[8 * j: 8 * j + 8])
            v_coeff = np.polyder(coeff)
            a_coeff = np.polyder(v_coeff)
            j_coeff = np.polyder(a_coeff)

            vel_components.append(np.polyval(v_coeff, ti))
            acc_components.append(np.polyval(a_coeff, ti))
            jrk_components.append(np.polyval(j_coeff, ti))

        vel_mag[i] = np.linalg.norm(vel_components)
        acc_mag[i] = np.linalg.norm(acc_components)
        jrk_mag[i] = np.linalg.norm(jrk_components)

    # plt.figure(figsize=(10, 6))
    # plt.plot(t, vel_mag, label="Velocity Magnitude")
    # plt.plot(t, acc_mag, label="Acceleration Magnitude")
    # plt.plot(t, jrk_mag, label="Jerk Magnitude")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Magnitude")
    # plt.legend()
    # plt.grid()
    # plt.title("Magnitudes of Velocity, Acceleration, and Jerk")
    # plt.show()
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    wp_times = [0]+ np.cumsum(time_alloc_arr)
    axes[0].plot(t, vel_mag, label='Velocity Magnitude', color='b')
    axes[1].plot(t, acc_mag, label='Acceleration Magnitude', color='g')
    axes[2].plot(t, jrk_mag, label='Jerk Magnitude', color='r')
    for ax in axes:
        for wp_time in wp_times:
            ax.axvline(wp_time, linestyle='--', color='gray', alpha=0.6, label="Waypoint")  
        #ax.legend()
        ax.grid(True)

    axes[2].set_xlabel("Time (s)")
    axes[0].set_ylabel("Velocity (m/s)")
    axes[1].set_ylabel("Acceleration (m/s²)")
    axes[2].set_ylabel("Jerk (m/s³)")
    
    plt.suptitle("Velocity, Acceleration, and Jerk Magnitudes with Waypoints")
    plt.show()

def publish_3d_traj(p_x, p_y, p_z, times, wp_x, wp_y, wp_z):
    rospy.init_node('trajectory_publisher', anonymous=True)
    path_pub = rospy.Publisher('/trajectory_path', Path, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Evaluate trajectory
    N = 500
    t = np.linspace(times[0], times[-1], N)
    positions = {'x': [], 'y': [], 'z': []}

    path_msg = Path()
    path_msg.header.frame_id = "map"  # Change to the appropriate frame

    for i in range(N):
        j = np.nonzero(t[i] <= times)[0][0] - 1
        j = max(j, 0)
        ti = t[i] - times[j]

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()

        for axis, p in zip(['x', 'y', 'z'], [p_x, p_y, p_z]):
            coeff = np.flip(p[8 * j:8 * j + 8])
            positions[axis].append(np.polyval(coeff, ti))

        pose.pose.position.x = positions['x'][-1]
        pose.pose.position.y = positions['y'][-1]
        pose.pose.position.z = positions['z'][-1]

        path_msg.poses.append(pose)

    # Publish the path
    while not rospy.is_shutdown():
        path_msg.header.stamp = rospy.Time.now()  # Update timestamp
        path_pub.publish(path_msg)
        rate.sleep()
        
raw_waypoints= load_waypoints("/home/meera/catkin_ws/src/hector_rrt_star/src/rrt_path_output.txt")
wp_x, wp_y, wp_z= [],[],[]
for i in range(len(raw_waypoints)):
    wp_x.append(raw_waypoints[i][0])
    wp_y.append(raw_waypoints[i][1])
    wp_z.append(raw_waypoints[i][2])
vel_max= 0.1
acc_max= 0.2
time_alloc_arr= time_allocation(raw_waypoints, vel_max,acc_max)
# print(wp_x)
# print(time_alloc_arr)
times = [0] + np.cumsum(time_alloc_arr).tolist()
print(times)
Q_submatrix_sym= compute_Q_submatrix_sym()
#p = get_coefficients(time_alloc_arr, wp_x)

p_x = get_coefficients(time_alloc_arr, wp_x)
p_y = get_coefficients(time_alloc_arr, wp_y)
p_z = get_coefficients(time_alloc_arr, wp_z)

#plot_3d_traj(p_x,p_y, p_z,times,wp_x, wp_y, wp_z)
# plot_1d_traj(p_x, wp_x,times)
# plot_1d_traj(p_y, wp_y,times)
# plot_1d_traj(p_z, wp_z,times)
#plot_magnitude_traj(p_x, p_y, p_z, times, time_alloc_arr)

if __name__ == "__main__":
    rospy.sleep(1) 
    publish_3d_traj(p_x, p_y, p_z, times, wp_x, wp_y, wp_z)
    #rospy.spin()