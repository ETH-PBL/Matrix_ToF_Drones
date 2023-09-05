from math import nan
import math
from re import I
import cv2 
import pickle
import numpy as np
from numpy import degrees, uint64
from numpy.core.fromnumeric import size, transpose
from numpy.lib.function_base import average, flip
import matplotlib.pyplot as plt
from numpy.lib.nanfunctions import nanmin
from numpy.lib.type_check import mintypecode, real
import psutil
import seaborn as sns; sns.set_theme()
from time import process_time, sleep
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_agg import FigureCanvasAgg
from multiprocessing import Process, cpu_count, process
import os
import datetime
import warnings
warnings.filterwarnings("ignore")

GROUND_BORDER = 6
CEILING_BORDER = 2
DIS_GROUND_MIN = 400 # mm
DIS_CEILING_MIN = 600 # mm

DIS_REACT = 2000; # mm
DIS_SLOW = 700; # mm
DIS_STOP = 400; # mm
DIS_FEAR = 150; # mm

DRONE_ZONE_tl_x = 3.0
DRONE_ZONE_tl_y = 3.0
DRONE_ZONE_br_x = 5.0
DRONE_ZONE_br_y = 4.0
CARE_ZONE_tl_x = 2.0
CARE_ZONE_tl_y = 2.0
CARE_ZONE_br_x = 5.0
CARE_ZONE_br_y = 5.0


def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color)
    plt.setp(bp['whiskers'], color=color)
    plt.setp(bp['caps'], color=color)
    plt.setp(bp['medians'], color=color)

def is_intersect(lst1, lst2):
    lst3 = [value for value in lst1 if value in lst2]
    return len(lst3)>0

def euler_to_rotMat(yaw, pitch, roll):
    Rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [          0,            0, 1]])
    Ry_pitch = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [             0, 1,             0],
        [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx_roll = np.array([
        [1,            0,             0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]])
    # R = RxRyRz
    rotMat = np.dot(Rx_roll, np.dot(Ry_pitch, Rz_yaw))
    return rotMat

def euler_from_quaternion(w, x, y, z):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        roll_x_d = math.degrees(roll_x)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        pitch_y_d = math.degrees(pitch_y)

     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        yaw_z_d = math.degrees(yaw_z)

     
        return roll_x_d, pitch_y_d, yaw_z_d # in degree

def arrow3d(ax, length=1, width=0.05, head=0.2, headwidth=2,
                theta_x=0, theta_z=0, theta_y=0, dir ='z', offset=(0,0,0), **kw):
    w = width
    h = head
    hw = headwidth
    theta_x = np.deg2rad(theta_x)
    theta_y = np.deg2rad(theta_y)
    theta_z = np.deg2rad(theta_z)

    a = [[0,0],[w,0],[w,(1-h)*length],[hw*w,(1-h)*length],[0,length]]
    a = np.array(a)

    r, theta = np.meshgrid(a[:,0], np.linspace(0,2*np.pi,30))
    if(dir == 'x'):
        x = np.tile(a[:,1],r.shape[0]).reshape(r.shape)
        y = r*np.sin(theta)
        z = r*np.cos(theta)
    elif(dir == 'y'):
        y = np.tile(a[:,1],r.shape[0]).reshape(r.shape)
        z = r*np.sin(theta)
        x = r*np.cos(theta)
    elif(dir == 'z'):
        z = np.tile(a[:,1],r.shape[0]).reshape(r.shape)
        x = r*np.sin(theta)
        y = r*np.cos(theta)
    rot_mat= euler_to_rotMat(theta_z,theta_y,theta_x)

    b = np.dot(rot_mat, np.c_[x.flatten(),y.flatten(),z.flatten()].T).T + np.array(offset)
    x = b[:,0].reshape(r.shape); 
    y = b[:,1].reshape(r.shape); 
    z = b[:,2].reshape(r.shape); 
    ax.plot_surface(x,y,z, **kw)

def make_image(all_data, indexes, data_state, address, fig):
    #----------------------------VICON------------------------------#
    
    if (data_state['vicon'] == True):
        ax = fig.add_subplot(231, projection='3d')
        for vic_obj in list(all_data['vicon'].keys()):

            pos = all_data['vicon'][vic_obj]['position'][indexes['vicon'][vic_obj]][0]
            quat = all_data['vicon'][vic_obj]['quaternion'][indexes['vicon'][vic_obj]][0]            

            roll_x, pitch_y, yaw_z = euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])
            posx = pos[0]
            posy = pos[1]
            posz = pos[2]

            arrow3d(ax, dir='z', length=.3, width=.03, color="green", facecolor='green', ec='green', theta_x=roll_x, theta_y = pitch_y, theta_z = yaw_z, offset=[posx,posy,posz]) #top Z
            arrow3d(ax, dir='y',length=.4, width=.03, color="blue",  facecolor='blue',  ec='blue',  theta_x=roll_x, theta_y = pitch_y, theta_z = yaw_z, offset=[posx,posy,posz]) #left  Y
            arrow3d(ax, dir='x',length=.6, width=.03, color="red",   facecolor='red',   ec='red',   theta_x=roll_x, theta_y = pitch_y, theta_z = yaw_z, offset=[posx,posy,posz]) #face  X
            
            ax.text(posx, posy, posz+.35, str(vic_obj), fontsize=10, color='k')


        #base cordinate
        arrow3d(ax, length=2, width=.01, color="green", facecolor='green', ec='green', theta_x=0,  theta_y = 0, theta_z = 0, offset=[0,0,0]) #z
        arrow3d(ax, length=2, width=.01,color="blue",  facecolor='blue',  ec='blue',  theta_x=-90, theta_y = 0, theta_z = 0, offset=[0,0,0]) #y
        arrow3d(ax, length=2, width=.01,color="red",   facecolor='red',   ec='red',   theta_x=0,  theta_y = 90,theta_z = 0, offset=[0,0,0]) #x
        
        ax.set_title("Vicon")

    
    #---------------------------CRAZYFLIE---------------------------#
    if (data_state['cf0'] == True and data_state['cf1'] == True):

        posx = all_data['cf0']['x'][indexes['cf0']][0]
        posy = all_data['cf0']['y'][indexes['cf0']][0]
        posz = all_data['cf0']['z'][indexes['cf0']][0]

        vx = all_data['cf0']['vx'][indexes['cf0']][0]
        vy = all_data['cf0']['vy'][indexes['cf0']][0]
        vz = all_data['cf0']['vz'][indexes['cf0']][0]

        roll_x = all_data['cf1']['roll'][indexes['cf1']][0]
        pitch_y = all_data['cf1']['pitch'][indexes['cf1']][0]
        yaw_z = all_data['cf1']['yaw'][indexes['cf1']][0]

        # yaw_z, roll_x, pitch_y = yaw_z, 0, 0

        #face to Y, top to Z
        ax = fig.add_subplot(232, projection='3d')
        arrow3d(ax, dir='z', length=.3, width=.03, color="green", facecolor='green', ec='green', theta_x=roll_x, theta_y = pitch_y, theta_z = yaw_z, offset=[posx,posy,posz]) #top Z
        arrow3d(ax, dir='y',length=.4, width=.03, color="blue",  facecolor='blue',  ec='blue',  theta_x=roll_x, theta_y = pitch_y, theta_z = yaw_z, offset=[posx,posy,posz]) #left  Y
        arrow3d(ax, dir='x',length=.6, width=.03, color="red",   facecolor='red',   ec='red',   theta_x=roll_x, theta_y = pitch_y, theta_z = yaw_z, offset=[posx,posy,posz]) #face  X
        
        ax.text(posx, posy, posz+.35, 'Drone', fontsize=10, color='k')

        #base cordinate
        arrow3d(ax, length=2, width=.01, color="green", facecolor='green', ec='green', theta_x=0,  theta_y = 0, theta_z = 0, offset=[0,0,0]) #z
        arrow3d(ax, length=2, width=.01,color="blue",  facecolor='blue',  ec='blue',  theta_x=-90, theta_y = 0, theta_z = 0, offset=[0,0,0]) #y
        arrow3d(ax, length=2, width=.01,color="red",   facecolor='red',   ec='red',   theta_x=0,  theta_y = 90,theta_z = 0, offset=[0,0,0]) #x
        # plt.show()
        ax.set_title("CrazyFlie")

        ax.plot([],[], c='red', label='Vx= ' + str(round(vx,2)))
        ax.plot([],[], c='blue', label='Vy= ' + str(round(vy,2)))
        ax.plot([],[], c='green', label='Vz= ' + str(round(vz,2)))
        plt.legend(prop={'size': 20})
        # To remove the huge white borders
        # ax.margins(0)
    
    #------------------------------TOF------------------------------#
    if (data_state['tof'] == True):
        input_tof_image = all_data['tof']['distances'][indexes['tof']][0]
        invalid_mask = all_data['tof']['invalid_pixel_masks'][indexes['tof']][0]
        #refine image
        img_arr = input_tof_image /10.0
        for i in range(len(input_tof_image)):
            if (invalid_mask[i]):
                img_arr[i] = nan
        img_arr = np.around(img_arr)
        img=np.reshape(img_arr,(8,8))

        ax = fig.add_subplot(234)
        RGcmap = sns.diverging_palette(
        10, 133, 100, 60, center='light', as_cmap=True)
        sns.heatmap(img, annot=True, fmt=".0f",
                    cmap=RGcmap,annot_kws={"size": 18},vmin=0, vmax=300)
        sns.set(font_scale=1.9)

        plt.text(8+0.25, 8+0.4, "(cm)", fontsize=20, color='orange')
        ax.set_title("ToF")

    #--------------------------desicion Making----------------------#
    desicion_making(input_tof_image,invalid_mask,fig)
    #--------------------------desicion Making----------------------#

    #-----------------------------CAMERA----------------------------#
    if (data_state['cam'] == True):
        ax = fig.add_subplot(235)
        ax.set_title("Camera")
        plt.imshow(cv2.imread(os.path.join(address,'images',str(indexes['cam'])+'.jpeg')))
        plt.grid(visible=None)
        plt.axis('off')
        # To remove the huge white borders
        # ax.margins(0)

# TODO HANNA
def object_detection(image):
    objects_pos = []
    one_indexes = np.array(np.where(image == 1))
    if len(one_indexes[0])>= 1:
        objects =[]
        objects.append([])
        objects[0].append([one_indexes[0,0],one_indexes[1,0]])
        # one_indexes =np.delete(one_indexes,0,1)
    else:
        return objects_pos
    if len(one_indexes[0]) >= 2:
        #put connected nodes in objects
        for i in range(1,len(one_indexes[0])):
            is_connected = False
            for j in range(len(objects)):
                for index in objects[j]:
                    if (abs(one_indexes[0,i] - index[0])**2 + abs(one_indexes[1,i] - index[1])**2) < 4 :
                        # node is connected
                        objects[j].append([one_indexes[0,i],one_indexes[1,i]])
                        is_connected = True
                        break
            if not is_connected:
                # seprate node --> new object
                objects.append([])
                objects[-1].append([one_indexes[0,i],one_indexes[1,i]])

        #check for objects if they are connected
        connected_object_index = []
        for i in range(len(objects)):
            for j in range(i+1,len(objects)):
                if [i,j] in connected_object_index:
                    break
                is_connected =False
                for indexi in objects[i]:
                    if is_connected==True:
                        break
                    for indexj in objects[j]:
                        if (abs(indexi[0] - indexj[0])**2 + abs(indexi[1] - indexj[1])**2) < 4 :
                        # objects are connected
                            connected_object_index.append([i,j])
                            is_connected =True
                            break
        #merge connceted objects
        is_merge = True  
        while is_merge == True:
            is_merge = False
            for i in range(len(connected_object_index)):
                if is_merge == True:
                    break
                for j in range(i+1,len(connected_object_index)):               
                    if (is_intersect(connected_object_index[i],connected_object_index[j])):
                        connected_object_index[i] = connected_object_index[i] + connected_object_index[j]
                        connected_object_index.pop(j)
                        is_merge = True
                        break

        if len(connected_object_index)==0 :
            corrected_objects = objects
        else:
            corrected_objects =[]
            for merge in connected_object_index:
                corrected_objects.append([])
                for j in set(merge):
                    corrected_objects[-1]=corrected_objects[-1]+objects[j]

        #process objects pos    
        for j in range(len(corrected_objects)):
            Xs =[]
            Ys = []
            for index in corrected_objects[j]:
                Xs.append(index[0])
                Ys.append(index[1])
            objects_pos.append([Xs,Ys])
    else:
        objects_pos.append([[one_indexes[0,0]],[one_indexes[1,0]]])
    return objects_pos
    
# TODO HANNA
def desicion_making(img_array,invalid_mask,fig):
    danger_distance= 150 #cm
    binary_imgarr = np.around(img_array/10)
    for i in range(len(binary_imgarr)):
        binary_imgarr[i] = 0 if binary_imgarr[i] > danger_distance or invalid_mask[i] else 1
    binary_img=np.reshape(binary_imgarr,(8,8))

    kernel = np.ones((2,2),np.uint8)
    opening_img = cv2.morphologyEx(binary_img, cv2.MORPH_OPEN, kernel)

    # Detect all objectes
    targets  = object_detection(opening_img)
    target_str = str(len(targets))
    # calculate the angle to the object
    img = img_array /1.0 #mm
    for i in range(len(img_array)):
        if (invalid_mask[i]):
            img[i] = nan
    img = np.around(img)
    img=np.reshape(img,(8,8))

    dis_global_min = 2**16 - 1
    selected_target = - 1
    for i in range(len(targets)):
        curr_target = targets[i]
        Xs = curr_target[0]
        Ys = curr_target[1]
        tar_distances = []
        for j in range(len(Xs)):
            tar_distances.append(img[Xs[j],Ys[j]]) #in mm
        if min(tar_distances) < dis_global_min:
            dis_global_min = min(tar_distances)
            selected_target = i

    decision_str = ""
    if(selected_target < 0): 
        # no target detected
        decision_str+= "/Forward"
    else:
        Xs = targets[selected_target][0] 
        Ys = targets[selected_target][1]
        tar_distances = []
        for j in range(len(Xs)):
            tar_distances.append(img[Xs[j],Ys[j]]) #in mm
        tar_center_pos = [average(Xs),average(Ys)]
        midx = 3.5
        midy = 3.5
        miny = 3
        minx = 2
        maxy = 5
        maxx = 4
        if tar_center_pos[0]>midx:
            lower = True
            upper = False
        else:
            lower=False
            upper = True
        if tar_center_pos[1]>midy:
            right = True
            left = False
        else:
            right=False
            left = True
        
        if (min(Xs) >= GROUND_BORDER) and nanmin(tar_distances)<DIS_GROUND_MIN and lower: #check for ground lower
            #ground should be avoided
            decision_str+= "/increase height"
            target_str += '-G'
        elif (max(Xs) <= CEILING_BORDER) and nanmin(tar_distances)<DIS_CEILING_MIN and upper: #check for celling upper
            #celling should be avoided
            decision_str+= "/Decrease height"
            target_str += '-C'
        elif nanmin(tar_distances)<DIS_FEAR:
            #fly backwards - we cannot see backwards, but we are too close to something to not crash if we don't back up anyway
            decision_str+= "/backward"
            target_str += '-F'
        elif nanmin(tar_distances)<DIS_REACT: #check for front object
            #objects which should be avoided
            if min(Ys) >= DRONE_ZONE_tl_y and min(Xs) >= DRONE_ZONE_tl_x and max(Ys) <= DRONE_ZONE_br_y and max(Xs) <= DRONE_ZONE_br_x:
                if nanmin(tar_distances)<=DIS_STOP:
                    if left:
                        decision_str+= "/stop and rotate fast right"
                        target_str += '-dzl'
                    else:
                        decision_str+= "/stop and rotate fast left"
                        target_str += '-dzr'
                if nanmin(tar_distances)<=DIS_SLOW:
                    if left:
                        decision_str+= "/slow forward and rotate fast right"
                        target_str += '-dzl'
                    else:
                        decision_str+= "/slow forward and rotate fast left"
                        target_str += '-dzr'
                else:
                    if left:
                        decision_str+= "/medium forward and rotate slow right"
                        target_str += '-dzl'
                    else:
                        decision_str+= "/medium forward and rotate slow left"
                        target_str += '-dzr'
            elif min(Ys) >= CARE_ZONE_tl_y and min(Xs) >= CARE_ZONE_tl_x and max(Ys) <= CARE_ZONE_br_y and max(Xs) <= CARE_ZONE_br_x:
                if nanmin(tar_distances)<=DIS_STOP:
                    if left:
                        decision_str+= "/stop and rotate fast right"
                        target_str += '-czl'
                    else:
                        decision_str+= "/stop and rotate fast left"
                        target_str += '-czr'
                else:
                    if left:
                        decision_str+= "/medium forward and rotate slow right"
                        target_str += '-czl'
                    else:
                        decision_str+= "/medium forward and rotate slow left"
                        target_str += '-czr'
            else:
                decision_str+= "/medium forward"
                target_str += '-nz'
            
        else: 
            #object is out of range
            decision_str+= "/medium forward"
            target_str += '-OoR'

            
    ax = fig.add_subplot(233)
    ax.set_title(decision_str)

    ax = fig.add_subplot(236)
    ax.set_title("opening_"+target_str)
    sns.heatmap(opening_img, annot=True, fmt=".0f",annot_kws={"size": 18},vmin=0, vmax=1)
    sns.set(font_scale=1.9)


def ProcessData(address, address2save,folder_name,p_index, force_replace = True):
    t1_start = process_time() 
    data_state ={
                'cam': False,
                'cf0': False, 
                'cf1': False, 
                'vicon': False, 
                'tof': False
                } 
    #------------------------------------------Read all data and classify------------------------------------------#
    
    ToF_timestamps = []
    ToF_distances = []
    ToF_targets = []
    ToF_status = []
    # ToF_timestamps ToF_distances ToF_targets ToF_status
    try:

        #read
        with open(os.path.join(address+"state_ToF.dat"), 'rb') as f:
            ToF_dat = []
            while True:
                try:
                    o = pickle.load(f)
                    ToF_dat.append(o)
                except EOFError:
                    break

        #classify
        for item in ToF_dat:
            ToF_timestamps.append(item[0])
            ToF_distances.append(item[1])
            ToF_targets.append(item[2])
            ToF_status.append(item[3])

        ToF_timestamps = np.array(ToF_timestamps).astype(uint64)
        ToF_distances = np.array(ToF_distances)
        ToF_targets = np.array(ToF_targets)
        ToF_status = np.array(ToF_status)

        ToF_invalid_mask = np.zeros((len(ToF_distances), len(ToF_distances[0])))
        for i in range(len(ToF_distances)):
            for j in range(len(ToF_distances[0])):
                if((ToF_status[i, j] != 5 and ToF_status[i, j] != 9) or ToF_targets[i, j] != 1):
                    ToF_invalid_mask[i, j] = 1

        ToF_data_pack = {
                        'timestamp': ToF_timestamps, 
                        'distances': ToF_distances, 
                        'targestNum': ToF_targets, 
                        'status': ToF_status, 
                        'invalid_pixel_masks': ToF_invalid_mask
                        }
        data_state['tof'] = True
        print(str(p_index)+'/'+str(len(ToF_dat))+" ToF_dat has been read succesfully. FR:" +
                  str(round((1000*len(ToF_dat))/(ToF_timestamps[-1]-ToF_timestamps[0]), 2)))

    except FileNotFoundError:
        print(str(p_index)+'/'+"Tof file not found!")
    except Exception as e:
        print(str(p_index)+'/'+type(e))     # the exception instance
        print(str(p_index)+'/'+e.args)      # arguments stored in .args
        print(str(p_index)+'/'+e)          # __str__ allows args to printed directly
    #----------------------------------------------------------------------------------------------------------------------------------------#
    
    crazyflie_timestamps0 = []
    crazyflie_stateEstimate_x = []
    crazyflie_stateEstimate_y = []
    crazyflie_stateEstimate_z = []
    crazyflie_stateEstimate_vx = []
    crazyflie_stateEstimate_vy = []
    crazyflie_stateEstimate_vz = []
    # timeStamp	stateEstimate.x	stateEstimate.y	stateEstimate.z	stateEstimate.vx	stateEstimate.vy	stateEstimate.vz
    try:

        #read
        with open(os.path.join(address+"state_Crazyflie_group00.dat"), 'rb') as f:
            crazyflie_dat_0 = []   
            while True:
                try:
                    o = pickle.load(f)
                    crazyflie_dat_0.append(o)
                except EOFError:
                    break

        #classify
        for item in crazyflie_dat_0:
            crazyflie_timestamps0.append(item[0])
            crazyflie_stateEstimate_x.append(item[1])
            crazyflie_stateEstimate_y.append(item[2])
            crazyflie_stateEstimate_z.append(item[3])
            crazyflie_stateEstimate_vx.append(item[4])
            crazyflie_stateEstimate_vy.append(item[5])
            crazyflie_stateEstimate_vz.append(item[6])

        crazyflie_timestamps0 = np.array (crazyflie_timestamps0).astype(uint64)
        crazyflie_stateEstimate_x = np.array (crazyflie_stateEstimate_x)
        crazyflie_stateEstimate_y = np.array (crazyflie_stateEstimate_y)
        crazyflie_stateEstimate_z = np.array (crazyflie_stateEstimate_z)
        crazyflie_stateEstimate_vx = np.array (crazyflie_stateEstimate_vx)
        crazyflie_stateEstimate_vy = np.array (crazyflie_stateEstimate_vy)
        crazyflie_stateEstimate_vz = np.array (crazyflie_stateEstimate_vz)

        CrazyFlie0_data_pack = {
                                'timestamp': crazyflie_timestamps0, 
                                'x': crazyflie_stateEstimate_x, 
                                'vx': crazyflie_stateEstimate_vx,
                                'y': crazyflie_stateEstimate_y, 
                                'vy': crazyflie_stateEstimate_vy, 
                                'z': crazyflie_stateEstimate_z, 
                                'vz': crazyflie_stateEstimate_vz
                                }
        data_state['cf0'] = True
        print(str(p_index)+'/'+str(len(crazyflie_dat_0))+" crazyflie_dat_0 has been read succesfully. FR:" +
                  str(round((1000*len(crazyflie_dat_0))/(crazyflie_timestamps0[-1]-crazyflie_timestamps0[0]), 2)))

    except FileNotFoundError:
        print(str(p_index)+'/'+"CrazyFlie0 file not found!")
    except Exception as e:
        print(str(p_index)+'/'+type(e))     # the exception instance
        print(str(p_index)+'/'+e.args)      # arguments stored in .args
        print(str(p_index)+'/'+e)          # __str__ allows args to printed directly
    #----------------------------------------------------------------------------------------------------------------------------------------#

    crazyflie_timestamps1 = []
    crazyflie_stateEstimate_roll = []
    crazyflie_stateEstimate_pitch = []
    crazyflie_stateEstimate_yaw = []
    # timeStamp	stateEstimate.roll	stateEstimate.pitch	stateEstimate.yaw
    try:

        #read
        with open(os.path.join(address+"state_Crazyflie_group01.dat"), 'rb') as f:
            crazyflie_dat_1 = []   
            while True:
                try:
                    o = pickle.load(f)
                    crazyflie_dat_1.append(o)
                except EOFError:
                    break

        #classify
        for item in crazyflie_dat_1:
            crazyflie_timestamps1.append(item[0])
            crazyflie_stateEstimate_roll.append(item[1])
            crazyflie_stateEstimate_pitch.append(item[2])
            crazyflie_stateEstimate_yaw.append(item[3])

        crazyflie_timestamps1= np.array(crazyflie_timestamps1).astype(uint64)
        crazyflie_stateEstimate_roll= np.array(crazyflie_stateEstimate_roll)
        crazyflie_stateEstimate_pitch= np.array(crazyflie_stateEstimate_pitch)
        crazyflie_stateEstimate_yaw= np.array(crazyflie_stateEstimate_yaw)

        CrazyFlie1_data_pack = {
                                'timestamp': crazyflie_timestamps1, 
                                'roll': crazyflie_stateEstimate_roll, 
                                'pitch': crazyflie_stateEstimate_pitch,
                                'yaw': crazyflie_stateEstimate_yaw
                                }
        data_state['cf1'] = True
        print(str(p_index)+'/'+str(len(crazyflie_dat_1))+" crazyflie_dat_1 has been read succesfully. FR:" +
                  str(round((1000*len(crazyflie_dat_1))/(crazyflie_timestamps1[-1]-crazyflie_timestamps1[0]), 2)))

    except FileNotFoundError:
        print(str(p_index)+'/'+"CrazyFlie1 file not found!")
    except Exception as e:
        print(str(p_index)+'/'+type(e))     # the exception instance
        print(str(p_index)+'/'+e.args)      # arguments stored in .args
        print(str(p_index)+'/'+e)          # __str__ allows args to printed directly
    #----------------------------------------------------------------------------------------------------------------------------------------#

    vicon_timestamps = []
    vicon_position = []
    vicon_quaternion = []
    vicon_name = [] 
    # timeStamp	[posx	posy	posz]	[qw	qx	qy	qz] name
    try:
        #read
        with open(os.path.join(address+"state_Vicon.dat"), 'rb') as f:
            vicon_dat = []  
            while True:
                try:
                    o = pickle.load(f)
                    vicon_dat.append(o)
                except EOFError:
                    break

        #classify 
        if vicon_dat[0][0] == 0:
            vicon_dat.pop(0)
        for item in vicon_dat:
            vicon_timestamps.append(item[0])
            vicon_position.append(item[1])
            vicon_quaternion.append(item[2])
            vicon_name.append(item[3])
        
        vicon_timestamps_all = np.array(vicon_timestamps)
        vicon_position_all = np.array(vicon_position)
        vicon_quaternion_all = np.array(vicon_quaternion)
        vicon_name_all = np.array(vicon_name)
        vicon_name_set = set(vicon_name_all)

        vicon_data_pack = {}
        for obj_name in vicon_name_set:
            indexes = np.where(vicon_name_all == obj_name)
            vicon_data_pack[obj_name] = {
                                        'timestamp': vicon_timestamps_all[indexes], 
                                        'position': vicon_position_all[indexes], 
                                        'quaternion': vicon_quaternion_all[indexes]
                                        }

        data_state['vicon'] = True
        print(str(p_index)+'/'+str(len(vicon_timestamps))+" state_Vicon has been read succesfully. FR:" +
                  str(round(((1000*len(vicon_timestamps))/(vicon_timestamps[-1]-vicon_timestamps[0]))/len(vicon_name_set), 2)))
    except FileNotFoundError:
        print("Vicon file not found!")
    except Exception as e:
        print(str(p_index)+'/'+type(e))     # the exception instance
        print(str(p_index)+'/'+e.args)      # arguments stored in .args
        print(str(p_index)+'/'+e)          # __str__ allows args to printed directly
    #----------------------------------------------------------------------------------------------------------------------------------------#

    #read all images to map to ToF data later
    image_ts = []
    allimages = [f for f in os.listdir(os.path.join(address,'images')) if os.path.isfile(os.path.join(address,'images', f))]
    if len(allimages) ==0:
        data_state['cam'] = False
    else: 
        data_state['cam'] = True
        for fname in allimages:
            image_ts.append(os.path.splitext(fname)[0])
        image_timestamps = np.array(image_ts).astype(np.uint64)
        print(str(p_index)+'/'+str(len(allimages))+" Camera pictures are available. FR:" +
                  str(round(((1000*len(image_timestamps))/(max(image_timestamps)-min(image_timestamps))), 2)))
    #----------------------------------------------------------------------------------------------------------------------------------------#
    #gather all time stamps, order, set 
    # TSs =ToF_timestamps
    # TSs =np.concatenate((TSs, crazyflie_timestamps0), axis=0)
    # TSs =np.concatenate((TSs,crazyflie_timestamps1), axis=0)
    # TSs =np.concatenate((TSs,vicon_timestamps), axis=0)
    # TSs =np.concatenate((TSs,image_timestamps), axis=0)
    # TSs_set = sorted(set(TSs))
    # print(str(len(TSs_set))+str(max(TSs)-min(TSs)/1000))
    #------------------------------------------------------------------------------------------------------------#
    #------------------------------------------------------------------------------------------------------------#
    #------------------------------------------------------------------------------------------------------------#

    iter = 0
    
    for ts in ToF_timestamps:
        iter += 1
        cpu_index = psutil.cpu_count(logical=False)
        print("\t\t"*p_index + str(p_index)+'@'+str(cpu_index)+'/'+str(round(100*iter/len(ToF_timestamps),1)) + "%",end = "\r")
        #get closest index data to that Time
        img_ts = min(image_timestamps, key=lambda x:abs(x-ts)) if (data_state['cam'] == True) else 0 
        cf_idx0 = (np.where(crazyflie_timestamps0 == min(crazyflie_timestamps0, key=lambda x:abs(x-ts))))[0] if (data_state['cf0'] == True) else 0 
        cf_idx1 = (np.where(crazyflie_timestamps1 == min(crazyflie_timestamps1, key=lambda x:abs(x-ts))))[0] if (data_state['cf1'] == True) else 0 
        tof_idx = (np.where(ToF_timestamps == ts))[0] if (data_state['tof'] == True) else 0 
        if (data_state['vicon'] == True):
            vic_idx = {}
            for vic_obj in list(vicon_data_pack.keys()):
                vic_idx[vic_obj] = (np.where(vicon_data_pack[vic_obj]['timestamp']== min(vicon_data_pack[vic_obj]['timestamp'], key=lambda x:abs(x-ts))))[0] if (data_state['vicon'] == True) else 0 
        
        # pack data to pass
        indexes  = {
                    'cam': img_ts if (data_state['cam'] == True) else None,
                    'cf0': cf_idx0 if (data_state['cf0'] == True) else None, 
                    'cf1': cf_idx1 if (data_state['cf1'] == True) else None, 
                    'vicon': vic_idx if (data_state['vicon'] == True) else None, 
                    'tof': tof_idx if (data_state['tof'] == True) else None
                    }
        all_data = {
                    'cf0': CrazyFlie0_data_pack if (data_state['cf0'] == True) else None, 
                    'cf1': CrazyFlie1_data_pack if (data_state['cf1'] == True) else None, 
                    'vicon': vicon_data_pack if (data_state['vicon'] == True) else None, 
                    'tof': ToF_data_pack if (data_state['tof'] == True) else None
                    }
        # make figure
        fig= plt.figure(figsize=(30, 20))
        canvas = FigureCanvasAgg(fig)

        # update figure with data
        make_image(all_data, indexes, data_state, address, fig)

        #render the figure
        fig.tight_layout(pad=0)
        canvas.draw()
        width,height = fig.canvas.get_width_height()
        buf = canvas.buffer_rgba()
        buf_arr = np.asarray(buf)
        brg_img = cv2.cvtColor(buf_arr, cv2.COLOR_RGBA2BGR)

        # img.show()

        #save and close data
        #make video
        if iter == 1:
            print(str(p_index)+'/'+"video will be at\n\t"+os.path.join(address2save,(folder_name+'.avi')))
            # to not make all folders again  #comment  this if, if want to update all 
            if os.path.isfile(os.path.join(address2save,(folder_name+'.avi'))) :
                if force_replace == False:
                    print(str(p_index)+'/'+"Existed, no update  --> END!")
                    return
                else :
                    os.remove(os.path.join(address2save,(folder_name+'.avi')))
                    print(folder_name+'.avi'+' has been removed')
            size = (width,height)
            filename = folder_name.replace(" ", "") + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            out_video = cv2.VideoWriter(os.path.join(address2save,filename)+".avi", 
                                                cv2.VideoWriter_fourcc(*'MJPG')
                                                ,5,size)
        out_video.write(brg_img)
        plt.close(fig)
    if iter >= 1:
        out_video.release()
    t1_stop = process_time()
    print(str(p_index)+'/'+"Elapsed time during the whole program in seconds: ",
                                        round(t1_stop-t1_start,1))


if __name__ == "__main__":
    
    # /Approach /Rotate
    folder_address = "Approach/"
    sub_folders = [name for name in os.listdir(folder_address) if os.path.isdir(os.path.join(folder_address, name))]

    processess = []
    for i in range(len(sub_folders)):
        folder_name = sub_folders[i]
        print(str(i)+'/'+str(i)+"_"+folder_name)
        address = folder_address+folder_name+"/" 
        address2save = "VisualizerResults/"
        # ProcessData(address,address2save)
        p = Process(target=ProcessData, args=(address,address2save,folder_name,i,True))
        processess.append(p)
        # Make_video(address2save)
    print("cpu Cores: " + str(cpu_count()))
    ongoing_process_num = 0
    to_start_process = 0
    while(1) :
        if ongoing_process_num > cpu_count():
            sleep(1)
            #update running processes
            ongoing_process_num = 0
            for p in processess:
                if os.path.exists("/proc/"+str(p.ident)) :
                    ongoing_process_num +=1
        else:
            p=processess[to_start_process]         
            p.start()
            to_start_process+=1
            ongoing_process_num +=1
            if to_start_process >= len(processess):
                break

    #wait for all processes to be done
    for p in processess:
        p.join()
    