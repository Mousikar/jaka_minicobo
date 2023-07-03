#! /usr/bin/env python3.8
# encoding: utf-8
import open3d as o3d
import copy
import numpy as np
import math
import rospy
from geometry_msgs.msg import PoseStamped
from pyquaternion import Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Path

## 函数区
# 考虑x/y方向的路径
def XdirectionZigPath(xmin,xmax,ymin,ymax,Nx):
    paths = []
    dx = float(xmax-xmin)/(Nx-1)  # the y step-over
    flag=1      #奇偶分别
    path=[]
    for n in range(0,Nx):
        x = xmin+n*dx              # current y-coordinate 
        if flag==1:
            p1 = [x,ymin,0]   # start-point of line
            p2 = [x,ymax,0]   # end-point of line
            # print(flag)
        if flag==-2:
            p1 = [x,ymax,0]   # start-point of line
            p2 = [x,ymin,0]   # end-point of line
            # print(flag)
        path.append(p1)       # add the line to the path
        path.append(p2)
        flag=~flag
    return path

# x从小到大，y从大到小
# def XdirectionZigPath(xmin,xmax,ymin,ymax,Nx):
#     ymax=ymax+0.005
#     ymin=ymin+0.002
#     paths = []
#     dx = float(xmax-xmin)/(Nx-1)  # the y step-over
#     flag=1      #奇偶分别
#     path=[]
#     for n in range(0,Nx):
#         x = xmin+n*dx              # current y-coordinate 
#         if flag==-2:
#             p1 = [x,ymin,0]   # start-point of line
#             p2 = [x,ymax,0]   # end-point of line
#             # print(flag)
#         if flag==1:
#             p1 = [x,ymax,0]   # start-point of line
#             p2 = [x,ymin,0]   # end-point of line
#             # print(flag)
#         path.append(p1)       # add the line to the path
#         path.append(p2)
#         flag=~flag
#     return path

# 得到步进x/y方向的路径
def feedPath(path,step,step_num):
    flag=1      #奇偶分别
    fpath=[]
    n=len(path)
    for i in range(n//2):
        for j in range(step_num):
            if flag==1:
                p=[path[2*i][0],path[2*i][1]+j*step,path[2*i][2]]
            if flag==-2:
                p=[path[2*i][0],path[2*i][1]-j*step,path[2*i][2]]
            fpath.append(p)
        flag=~flag
    return fpath

# def feedPath(path,step,step_num):
#     flag=1      #奇偶分别
#     fpath=[]
#     n=len(path)
#     for i in range(n//2):
#         for j in range(step_num):
#             if flag==-2:
#                 p=[path[2*i][0],path[2*i][1]+j*step,path[2*i][2]]
#             if flag==1:
#                 p=[path[2*i][0],path[2*i][1]-j*step,path[2*i][2]]
#             fpath.append(p)
#         flag=~flag
#     return fpath

# 依据点云坐标得到z方向的坐标
def ZdirectionPath(fpath,points,normals,scan_height,rapid_height,flag_curve):
    zpath=[]
    npath=[]
    temp_npath_x=[]
    temp_npath_y=[]
    temp_npath_z=[]
    n=len(fpath)
    # 初始点
    zpath.append([fpath[0][0],fpath[0][1],rapid_height])
    npath.append([0,0,-1])
    # 中间点
    for i in range(n):
        temp=[]
        for m in range(len(points)):
            temp.append(math.sqrt((fpath[i][0]-points[m][0])**2+(fpath[i][1]-points[m][1])**2))
        min_index=np.argmin(temp)
        # print(min_index)
        scan_z=points[min_index][2]+scan_height
        zpath.append([fpath[i][0],fpath[i][1],scan_z])
        # 曲面用这个
        if flag_curve==1:
            npath.append([normals[min_index][0],normals[min_index][1],normals[min_index][2]])
        else:
            # 保存法向量
            temp_npath_x.append(normals[min_index][0])
            temp_npath_y.append(normals[min_index][1])
            temp_npath_z.append(normals[min_index][2])
            # 记得改回来
            # npath.append([0,0,-1])
    # 法向量平均
    if flag_curve==0:
        for i in range(n):
            npath.append([np.mean(temp_npath_x),np.mean(temp_npath_y),np.mean(temp_npath_z)])
    # 结束点
    zpath.append([fpath[n-1][0],fpath[n-1][1],rapid_height])
    npath.append([0,0,-1])
    return zpath,npath

class PointCloudSubscriber(object):
    def __init__(self):
        self.sub=rospy.Subscriber("/camera/depth/color/points",PointCloud2,self.callBack)
        self.depth_pc = rospy.Publisher('/workpiece_pointcloud', PointCloud2, queue_size=1)
        self.pathplanning = rospy.Publisher('/pathplanning', Path, queue_size=10)
        
    def callBack(self,pcd_msg):
        # unregister
        self.sub.unregister()
        
        pc = point_cloud2.read_points(pcd_msg, field_names=("x", "y", "z"))
        points = []
        for p in pc:
            points.append( [p[0],p[1],p[2]] )

        # origin
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        # Create Open3D point cloud
        data = o3d.geometry.PointCloud()# 传入3d点云
        data.points = o3d.utility.Vector3dVector(points)
        # o3d.visualization.draw_geometries([data,mesh_frame])

        # INTERACT
        vis = o3d.visualization.VisualizerWithVertexSelection()
        vis.create_window(window_name='Open3D', visible=True)
        vis.add_geometry(data)
        vis.run()
        point = vis.get_picked_points()
        vis.destroy_window()

        point_temp=[]
        for i in range(len(point)):
            point_temp.append(point[i].coord)

        pcd_in_camera = o3d.geometry.PointCloud()# 传入3d点云
        pcd_in_camera.points = o3d.utility.Vector3dVector(point_temp)# point_points 二维 numpy 矩阵,将其转换为 open3d 点云格式
        # print(pcd_in_camera)
        xyz_load = np.asarray(pcd_in_camera.points)
        
        # 变换到机器人坐标系 e
        pcd_in_link0 = o3d.geometry.PointCloud()# 传入3d点云
        pts_in_link0=[]
        # -3.05247            # 0.0152302            # -1.57026     
        # 四元数是[0.035778418980991884, .0.7058019852817551, 0.7070765842057435， -0.02462044628945545]
        # r_cb=np.array([[-0.00112692, -0.99635035,  0.08535048],
        #                     [-0.99987388,  0.00247478,  0.01568788],
        #                     [-0.01584184, -0.08532204, -0.99622748]])        # 之前标定的数据
        # r_cb=np.array([[-0.0076228,  -0.99500882,  0.09949547],
        #                 [-0.99986569,  0.0090278,   0.01367874],
        #                 [-0.01450869, -0.09937783, -0.99494399]])           # 6月30日标定的第一次数据
        r_cb=np.array([[-0.01856987, -0.99540875,  0.09389662],
                        [-0.9996803,   0.02009686,  0.01534297],
                        [-0.01715956, -0.09358168, -0.99546372]])           # 6月30日标定的第2次数据
        # r_dc=np.array([[0.999999175, -0.000203376972,  0.00126802484],
        #                     [0.000187890538,  0.999925545,  0.0122012168],
        #                     [-0.00127041187, -0.0122009685, 0.999924758]])        # 之前标定的数据
        r_dc=np.array([[ 0.999925545,  0.0122012168, -0.000187890538],
                        [-0.0122009685,  0.999924758,  0.00127041187],
                        [ 0.000203376972, -0.00126802484,  0.999999175]])           # 6月30日标定的数据       ,没问题 
        # t_cb=np.array([
        #             -0.00526445, 
        #             -0.337372, 
        #             0.519186])        # 之前标定的数据
        t_cb=np.array([
                    -0.00831419,
                    -0.333338,     
                    0.521126])           # 6月30日标定的第一次数据
        # t_cb=np.array([-0.00835337,   
        #                -0.334139,     
        #                0.521118])           # 6月30日标定的第2次数据
        # t_dc=np.array([
        #             0.00061128, 
        #             -0.0146306, 
        #             0.0000789815])        # 之前标定的数据
        t_dc=np.array([0.0146306,
                       -0.0000789815,
                       0.000611228])           # 6月30日标定的数据,没问题
        t=np.dot(r_cb,t_dc)+t_cb
        rotation=np.matmul(r_cb,r_dc)
        for i in range(len(xyz_load)):
            pts_in_link0.append(np.dot(rotation,xyz_load[i,:])+t)
        pcd_in_link0.points = o3d.utility.Vector3dVector(pts_in_link0)


        # publish
        header = Header()
        header.frame_id = "Link_0" #pcd_msg.header.frame_id #"d435_depth_frame"
        header.stamp = pcd_msg.header.stamp
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
        ]
        pc = point_cloud2.create_cloud(header, fields, pcd_in_link0.points)        
        self.depth_pc.publish(pc)

        try:
            # 规划路径
            pcd_in_link0.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamKNN(knn=200)                        # 计算近邻的20个点
            )
            o3d.geometry.PointCloud.orient_normals_to_align_with_direction(pcd_in_link0, orientation_reference=np.array([0.0, 0.0, -1.0]))  #设定法向量在z轴方向上，全部z轴正方向一致
            normals = np.array(pcd_in_link0.normals)    # 法向量结果与点云维度一致(N, 3)
            points = np.array(pcd_in_link0.points)
            xmin=np.min(points[:,0])
            xmax=np.max(points[:,0])
            ymin=np.min(points[:,1])
            ymax=np.max(points[:,1])

            # 参数
            Nx=rospy.get_param_cached("Nx",10)  # number of lines in the x-direction
            step_num=rospy.get_param_cached("step_num",10) 
            scan_height_int=rospy.get_param_cached("scan_height_int",1)
            rapid_height_int=rospy.get_param_cached("rapid_height_int",20)
            flag_curve=rospy.get_param_cached("flag_curve",0) # 曲面是1，平面是0
            # 确定x方向分Nx条线
            path = XdirectionZigPath(xmin,xmax,ymin,ymax,Nx)
            
            # 设置每条线走几步
            step=(ymax-ymin)/(step_num-1)
            fpath=feedPath(path,step,step_num)

            tool_length = 0.087
            scan_height=scan_height_int/100 + tool_length
            rapid_height=rapid_height_int/100 + tool_length#0.15
            zpath,npath=ZdirectionPath(fpath,points,normals,scan_height,rapid_height,flag_curve)
            # print(zpath)
            print('x方向分',Nx,'条线,每条线走',step_num,'步,每步',step,'米,扫描高度距离工件',scan_height_int,'厘米,')

            # 将路径画在点云上,可视化
            #绘制顶点
            lines=[]
            for i in range(len(zpath)-1):
                l=[i,i+1]
                lines.append(l) #连接的顺序
            color = [[0, 0, 0.8] for i in range(len(lines))] 
            #添加顶点，点云
            points_pcd = o3d.geometry.PointCloud()# 传入3d点云
            points_pcd.points = o3d.utility.Vector3dVector(zpath)  # point_points 二维 numpy 矩阵,将其转换为 open3d 点云格式
            points_pcd.paint_uniform_color([0, 0.8, 0]) #点云颜色 
            points_pcd.normals= o3d.utility.Vector3dVector(npath)
            #绘制线条
            lines_pcd = o3d.geometry.LineSet()
            lines_pcd.lines = o3d.utility.Vector2iVector(lines)
            lines_pcd.colors = o3d.utility.Vector3dVector(color) #线条颜色
            lines_pcd.points = o3d.utility.Vector3dVector(zpath)
            # 可视化
            # o3d.visualization.draw_geometries([points_pcd,lines_pcd,pcd_in_link0,mesh_frame])
            # o3d.visualization.draw_geometries([points_pcd,lines_pcd,mesh_frame],point_show_normal=True)


            # # publish
            # header = Header()
            # header.frame_id = "Link_0" #pcd_msg.header.frame_id #"d435_depth_frame"
            # header.stamp = pcd_msg.header.stamp
            # fields = [PointField('x', 0, PointField.FLOAT32, 1),
            # PointField('y', 4, PointField.FLOAT32, 1),
            # PointField('z', 8, PointField.FLOAT32, 1)
            # ]
            # pc = point_cloud2.create_cloud(header, fields, points_pcd.points)        
            # self.depth_pc.publish(pc)

            # 发布话题
            rate=rospy.Rate(1)
            path_msg = Path()
            path_msg.header.frame_id="Link_0"
            path_msg.header.stamp=rospy.Time.now()#pcd_msg.header.stamp
            ps = PoseStamped()
            ps.header.frame_id="Link_0"
            ps.header.stamp=rospy.Time.now()#pcd_msg.header.stamp
            # print(zpath)
            for i in range(len(zpath)):
                # x/y/z坐标
                ps.pose.position.x=zpath[i][0]
                ps.pose.position.y=zpath[i][1]
                ps.pose.position.z=zpath[i][2]

                # 构建旋转矩阵
                rotate_matrix=[[0,0,0],[0,0,0],[0,0,0]]
                Z = np.array([npath[i][0], npath[i][1], npath[i][2]])
                # X = np.array([Z[1], -Z[0], 0])
                X = np.array([0, -Z[2], Z[1]])
                Z = Z/np.linalg.norm(Z)
                X = X/np.linalg.norm(X)
                rotate_matrix[0][2]=Z[0]
                rotate_matrix[1][2]=Z[1]
                rotate_matrix[2][2]=Z[2]
                rotate_matrix[0][0]=X[0]
                rotate_matrix[1][0]=X[1]
                rotate_matrix[2][0]=X[2]
                # 计算叉乘
                Y = np.cross(Z,X)
                Y = Y/np.linalg.norm(Y)
                rotate_matrix[0][1]=Y[0]
                rotate_matrix[1][1]=Y[1]
                rotate_matrix[2][1]=Y[2]
                rotateMatrix = np.array(rotate_matrix)
                # print(rotateMatrix)
                q = Quaternion(matrix=rotateMatrix)
                ps.pose.orientation.x=q.x
                ps.pose.orientation.y=q.y
                ps.pose.orientation.z=q.z
                ps.pose.orientation.w=q.w
                # print(ps)
                path_msg.poses.append(ps)
                ps = PoseStamped()
                ps.header.frame_id="Link_0"
                ps.header.stamp=rospy.Time.now()#pcd_msg.header.stamp
            # 发布话题
            self.pathplanning.publish(path_msg)
            rate.sleep()
            # rospy.loginfo("%s",str(path_msg))
            rospy.loginfo("Published {} waypoints.".format(len(path_msg.poses))) 
        except:
            print("没有读取到点，即将进入下次回调！")


# 主函数
if __name__ == "__main__": 
    rospy.init_node("pathplan")
    PointCloudSubscriber()
    rospy.spin()