import math
import numpy as np

pi = 3.1415926535897932384626433832795
# 初始姿态
offset2 = -pi/2;offset3 = -pi/2
# 机械臂DH参数
d1 = 100;a1 = 0;a2 = 100;a3 = 0;d4 = 100;dt = 100;d3 = 0

# 关节限位
jointlimit_min=[-180,-180,-180,-180,-180,-180]
jointlimit_max=[180,180,180,180,180,180]

def Set_parameter(DH,Offset,JointLimitmin,JointLimitmax):
    print("DH设置顺序为:d1,a1,a2,a3,d4,dt",DH)
    print("offset设置顺序为:offset2,offset3",Offset[0],Offset[1])
    print("JointLimitmin:",JointLimitmin)
    print("JointLimitmax:",JointLimitmax)
    global d1,a1,a2,a3,d4,dt,offset2,offset3,jointlimit_min,jointlimit_max
    # 机械臂DH参数
    d1 = DH[0]
    a1 = DH[1]
    a2 = DH[2]
    a3 = DH[3]
    d4 = DH[4]
    dt = DH[5]
    # 机械臂初始姿态（当前竖直向上）
    offset2 = Offset[0]
    offset3 = Offset[1]
    # 关节限位
    jointlimit_min=JointLimitmin
    jointlimit_max=JointLimitmax
    
def kinematics(matrix_input):

    # 创建4x4输出矩阵
    matrix_output = [[0] * 4 for _ in range(4)]

    if jointlimit_min[3] == 0 and jointlimit_max[3] == 0:
        angle = [
            matrix_input[0],
            matrix_input[1] + offset2,
            matrix_input[2] + offset3,
            0,
            matrix_input[3],
            matrix_input[4]
        ]
    else:
        # 计算带偏移的角度
        angle = [
            matrix_input[0],
            matrix_input[1] + offset2,
            matrix_input[2] + offset3,
            matrix_input[3],
            matrix_input[4],
            matrix_input[5]
        ]
        
    # 计算三角函数值
    s1, s2, s3, s4, s5, s6 = [math.sin(a) for a in angle]
    c1, c2, c3, c4, c5, c6 = [math.cos(a) for a in angle]
    if jointlimit_min[3] == 0 and jointlimit_max[3] == 0:
        s4=0
        c4=1
    # 初始化输出矩阵
    output = [[0] * 4 for _ in range(4)]
    
    # 计算位置分量
    output[0][3] = (a1*c1 + a3*(c1*c2*c3 - c1*s2*s3) - d3*s1 - 
                   d4*(c1*c2*s3 + c1*c3*s2) + a2*c1*c2)
    output[1][3] = (a1*s1 + c1*d3 + a3*(c2*c3*s1 - s1*s2*s3) - 
                   d4*(c2*s1*s3 + c3*s1*s2) + a2*c2*s1)
    output[2][3] = -a2*s2 - a3*(c2*s3 + c3*s2) - d4*(c2*c3 - s2*s3)
    
    # 计算旋转矩阵
    # R11
    output[0][0] = (s6*(c4*s1 - s4*(c1*c2*c3 - c1*s2*s3)) - 
                   c6*(s5*(c1*c2*s3 + c1*c3*s2) - 
                   c5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3))))
    
    # R21
    output[1][0] = (-c6*(s5*(c2*s1*s3 + c3*s1*s2) + 
                    c5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3))) - 
                    s6*(c1*c4 + s4*(c2*c3*s1 - s1*s2*s3)))
    
    # R31
    output[2][0] = (s4*s6*(c2*s3 + c3*s2) - 
                   c6*(s5*(c2*c3 - s2*s3) + c4*c5*(c2*s3 + c3*s2)))
    
    # R12
    output[0][1] = (s6*(s5*(c1*c2*s3 + c1*c3*s2) - 
                   c5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3))) + 
                   c6*(c4*s1 - s4*(c1*c2*c3 - c1*s2*s3)))
    
    # R22
    output[1][1] = (s6*(s5*(c2*s1*s3 + c3*s1*s2) + 
                   c5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3))) - 
                   c6*(c1*c4 + s4*(c2*c3*s1 - s1*s2*s3)))
    
    # R32
    output[2][1] = (s6*(s5*(c2*c3 - s2*s3) + c4*c5*(c2*s3 + c3*s2)) + 
                   c6*s4*(c2*s3 + c3*s2))
    
    # R13
    output[0][2] = (-c5*(c1*c2*s3 + c1*c3*s2) - 
                   s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3)))
    
    # R23
    output[1][2] = (s5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3)) - 
                   c5*(c2*s1*s3 + c3*s1*s2))
    
    # R33
    output[2][2] = c4*s5*(c2*s3 + c3*s2) - c5*(c2*c3 - s2*s3)
    
    # 齐次变换矩阵的最后一行
    output[3][3] = 1
    
    # 处理极小值并赋值
    for i in range(4):
        for j in range(4):
            if abs(output[i][j]) < 1e-10:
                output[i][j] = 0
            matrix_output[i][j] = output[i][j]
    
    Tbase = np.array([[1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d1],
        [0, 0, 0, 1]])
    Ttool = np.array([[1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, dt],
        [0, 0, 0, 1]])
    matrix_output = Tbase @ matrix_output @ Ttool
    
    # 输出矩阵
    for i in range(4):
        for j in range(4):
            print(matrix_output[i][j], end=' ')
    print()
    
    return matrix_output

def poseTorpy(Matrix44):
    """
    正向运动学函数，将4x4齐次变换矩阵转换为6维位姿
    
    参数:
    Matrix44 -- 4x4齐次变换矩阵
    
    返回:
    Matrix6 -- 6维位姿向量 [x, y, z, roll, pitch, yaw]
    """
    # 参数
    rpy = np.zeros(3)
    eps = 0.000001
    
    # 输入变换矩阵
    T = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0], 
                  [0, 0, 1, 0]])
    
    G = Matrix44
    
    # 计算位置
    H = T @ (G @ np.array([0, 0, 0, 1]))
    
    # 计算欧拉角
    rpy[1] = np.arctan2(-Matrix44[2, 0], np.sqrt(Matrix44[0, 0]*Matrix44[0, 0] + Matrix44[1, 0]*Matrix44[1, 0]))
    
    if abs(abs(rpy[1]) - np.pi / 2.0) < eps:
        if rpy[1] > 0:
            rpy[1] = np.pi / 2.0
            rpy[2] = 0.0
            rpy[0] = np.arctan2(Matrix44[0, 1], Matrix44[1, 1])
        else:
            rpy[1] = -np.pi / 2.0
            rpy[2] = 0.0
            rpy[0] = -np.arctan2(Matrix44[0, 1], Matrix44[1, 1])
    else:
        cp = np.cos(rpy[1])
        rpy[2] = np.arctan2(Matrix44[1, 0] / cp, Matrix44[0, 0] / cp)
        rpy[0] = np.arctan2(Matrix44[2, 1] / cp, Matrix44[2, 2] / cp)
    
    Matrix6 = np.concatenate([H.flatten(), rpy])
    return Matrix6

def rpyTopose(Matrix6):
    """
    反向运动学函数，将6维位姿转换为4x4齐次变换矩阵
    
    参数:
    Matrix6 -- 6维位姿向量 [x, y, z, roll, pitch, yaw]
    
    返回:
    Matrix44 -- 4x4齐次变换矩阵
    """
    # 输入
    x = Matrix6[0]
    y = Matrix6[1]
    z = Matrix6[2]
    p = Matrix6[3]  # roll
    q = Matrix6[4]  # pitch
    r = Matrix6[5]  # yaw
    
    # 输出 - 创建4x4单位矩阵
    I = np.eye(4)
    
    # 计算三角函数
    sx = np.sin(p)
    cx = np.cos(p)
    sy = np.sin(q)
    cy = np.cos(q)
    sz = np.sin(r)
    cz = np.cos(r)
    
    # 构建旋转矩阵部分
    I[0, 0] = cy * cz
    I[0, 1] = cz * sx * sy - cx * sz
    I[0, 2] = sx * sz + cx * cz * sy
    I[1, 0] = cy * sz
    I[1, 1] = cx * cz + sx * sy * sz
    I[1, 2] = cx * sy * sz - cz * sx
    I[2, 0] = -sy
    I[2, 1] = cy * sx
    I[2, 2] = cx * cy
    
    # 设置平移部分
    I[0, 3] = x
    I[1, 3] = y
    I[2, 3] = z
    I[3, 3] = 1
    
    Matrix44 = I
    return Matrix44
# 使用 git clone https://github.com/PatrickRayShaw/kinematics.git 下载程序使用
def inverse_kinematics(input_matrix):
    """
    机器人逆运动学计算
    input_matrix: 4x4齐次变换矩阵
    返回: 6个关节角度的列表
    """
    output_matrix = [0] * 6
    Radian = [0] * 6
    
    Tbase = np.array([[1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d1],
        [0, 0, 0, 1]])
    Ttool = np.array([[1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, dt],
        [0, 0, 0, 1]])
    inv_Tbase = np.linalg.inv(Tbase)
    inv_Ttool = np.linalg.inv(Ttool)
    input_matrix = np.dot(inv_Tbase, np.dot(input_matrix, inv_Ttool))

    # 初始化变量
    Radian1 = [0, 0]
    Radian2 = [[0, 0], [0, 0]]
    Radian3 = [0, 0, 0, 0]
    Radian4 = [[0, 0], [0, 0]]
    Radian5 = [[0, 0], [0, 0]]
    Radian6 = [[0, 0], [0, 0]]
    Radian23 = [[0, 0], [0, 0]]
    
    Qmin = [0] * 6
    Qmax = [0] * 6
    Radian_To_sift = [[0] * 6 for _ in range(8)]
    GroupNumber = [0] * 8
    Min = [0] * 8
    
    # 提取输入矩阵元素
    nx = input_matrix[0][0]
    ox = input_matrix[0][1]
    ax = input_matrix[0][2]
    x = input_matrix[0][3]
    
    ny = input_matrix[1][0]
    oy = input_matrix[1][1]
    ay = input_matrix[1][2]
    y = input_matrix[1][3]
    
    nz = input_matrix[2][0]
    oz = input_matrix[2][1]
    az = input_matrix[2][2]
    z = input_matrix[2][3]
    
    # 计算Radian1
    Radian1[0] = math.atan2(y, x)
    Radian1[1] = math.atan2(-y, -x)
    
    # 计算Radian3的第一组解
    K = (x*x + y*y + z*z - a2*a2 - a3*a3 - d4*d4 + a1*a1 - 
         2 * a1*x*math.cos(Radian1[0]) - 2 * a1*y*math.sin(Radian1[0])) / (2 * a2)
    
    if a3*a3 + d4*d4 - K*K >= 0:
        xx = math.sqrt(a3*a3 + d4*d4 - K*K)
        Radian3[0] = math.atan2(a3, d4) - math.atan2(K, xx)
        Radian3[1] = math.atan2(a3, d4) - math.atan2(K, -xx)
    else:
        Radian3[0] = 10000
        Radian3[1] = 10000
    
    # 计算Radian3的第二组解
    K = (x*x + y*y + z*z - a2*a2 - a3*a3 - d4*d4 + a1*a1 - 
         2 * a1*x*math.cos(Radian1[1]) - 2 * a1*y*math.sin(Radian1[1])) / (2 * a2)
    
    if a3*a3 + d4*d4 - K*K >= 0:
        xx = math.sqrt(a3*a3 + d4*d4 - K*K)
        Radian3[2] = math.atan2(a3, d4) - math.atan2(K, xx)
        Radian3[3] = math.atan2(a3, d4) - math.atan2(K, -xx)
    else:
        Radian3[2] = 10000
        Radian3[3] = 10000
    
    # 计算其他关节角度
    h = 0
    h2 = 4
    
    for i in range(2):
        for j in range(2):
            if i == 1:
                Radian3[0] = Radian3[2]
                Radian3[1] = Radian3[3]
            
            s1 = math.sin(Radian1[i])
            s3 = math.sin(Radian3[j])
            c1 = math.cos(Radian1[i])
            c3 = math.cos(Radian3[j])
            
            # 计算Radian23和Radian2
            numerator = (-a3 - a2*c3)*z - (c1*x + s1*y - a1)*(d4 - a2*s3)
            denominator = (a2*s3 - d4)*z + (a3 + a2*c3)*(c1*x + s1*y - a1)
            Radian23[i][j] = math.atan2(numerator, denominator)
            Radian2[i][j] = Radian23[i][j] - Radian3[j]
            
            s23 = math.sin(Radian23[i][j])
            c23 = math.cos(Radian23[i][j])
            
            # 计算Radian4, Radian5, Radian6
            xxx1 = -ax*s1 + ay*c1
            xxx2 = -ax*c1*c23 - ay*s1*c23 + az*s23
            
            if abs(xxx1) < 1e-10 and abs(xxx2) < 1e-10:
                Radian5[i][j] = 0
                if jointlimit_min[3] == 0 and jointlimit_max[3] == 0:
                    Radian4[i][j] = 0
                    s4 = 0  
                    c4 = 1  
                else:
                    Radian4[i][j] = Radian[3]
                    s4 = math.sin(Radian4[i][j])
                    c4 = math.cos(Radian4[i][j])
                s5 = math.sin(Radian5[i][j])
                c5 = math.cos(Radian5[i][j])
                
                xxxxx1 = -nx*(c1*c23*s4 - s1*c4) - ny*(s1*c23*s4 + c1*c4) + nz*(s23*s4)
                xxxxx2 = nx*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) + ny*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) - nz*(s23*c4*c5 + c23*s5)
                Radian6[i][j] = math.atan2(xxxxx1, xxxxx2)
            else:
                if jointlimit_min[3] == 0 and jointlimit_max[3] == 0:
                    Radian4[i][j] = 0
                    s4 = 0  
                    c4 = 1  
                else:
                    Radian4[i][j] = math.atan2(xxx1, xxx2)
                    s4 = math.sin(Radian4[i][j])
                    c4 = math.cos(Radian4[i][j])
                
                xxxx1 = -ax*(c1*c23*c4 + s1*s4) - ay*(s1*c23*c4 - c1*s4) + az*(s23*c4)
                xxxx2 = ax*(-c1*s23) + ay*(-s1*s23) + az*(-c23)
                Radian5[i][j] = math.atan2(xxxx1, xxxx2)
                
                s5 = math.sin(Radian5[i][j])
                c5 = math.cos(Radian5[i][j])
                
                xxxxx1 = -nx*(c1*c23*s4 - s1*c4) - ny*(s1*c23*s4 + c1*c4) + nz*(s23*s4)
                xxxxx2 = nx*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) + ny*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) - nz*(s23*c4*c5 + c23*s5)
                Radian6[i][j] = math.atan2(xxxxx1, xxxxx2)
            
            # 存储解
            Radian_To_sift[h][0] = Radian1[i]
            Radian_To_sift[h][1] = Radian2[i][j] - offset2
            Radian_To_sift[h][2] = Radian3[j] - offset3
            Radian_To_sift[h][3] = Radian4[i][j]
            Radian_To_sift[h][4] = Radian5[i][j]
            Radian_To_sift[h][5] = Radian6[i][j]
            
            Radian_To_sift[h2][0] = Radian1[i]
            Radian_To_sift[h2][1] = Radian2[i][j] - offset2
            Radian_To_sift[h2][2] = Radian3[j] - offset3
            Radian_To_sift[h2][3] = Radian4[i][j] + pi
            Radian_To_sift[h2][4] = -Radian5[i][j]
            Radian_To_sift[h2][5] = Radian6[i][j] + pi
            
            h2 += 1
            h += 1
    

    # 关节角度限制（弧度）- 第四关节限制为0
    Qmin = [jointlimit_min[0]*(pi/180), jointlimit_min[1]*(pi/180), jointlimit_min[2]*(pi/180), jointlimit_min[3]*(pi/180), jointlimit_min[4]*(pi/180), jointlimit_min[5]*(pi/180)]
    Qmax = [jointlimit_max[0]*(pi/180), jointlimit_max[1]*(pi/180), jointlimit_max[2]*(pi/180), jointlimit_max[3]*(pi/180), jointlimit_max[4]*(pi/180), jointlimit_max[5]*(pi/180)]
    
    # 角度调整
    for m in range(8):
        for n in range(6):
            if Radian_To_sift[m][n] < Qmin[n]:
                Radian_To_sift[m][n] += 2 * pi
            
            if Radian_To_sift[m][n] > Qmax[n]:
                Radian_To_sift[m][n] -= 2 * pi
            
            # 防止角度跳跃
            if abs(Radian_To_sift[m][n] - Radian[n]) > pi:
                if Radian_To_sift[m][n] >= 0:
                    Radian_To_sift[m][n] -= 2 * pi
                else:
                    Radian_To_sift[m][n] += 2 * pi
    
    # 检查可行解
    for h in range(8):
        if ((Radian_To_sift[h][0] >= Qmin[0] and Radian_To_sift[h][0] <= Qmax[0]) and
            (Radian_To_sift[h][1] >= Qmin[1] and Radian_To_sift[h][1] <= Qmax[1]) and
            (Radian_To_sift[h][2] >= Qmin[2] and Radian_To_sift[h][2] <= Qmax[2]) and
            (Radian_To_sift[h][3] >= Qmin[3] and Radian_To_sift[h][3] <= Qmax[3]) and
            (Radian_To_sift[h][4] >= Qmin[4] and Radian_To_sift[h][4] <= Qmax[4]) and
            (Radian_To_sift[h][5] >= Qmin[5] and Radian_To_sift[h][5] <= Qmax[5])):
            
            GroupNumber[h] = 1
            Min[h] = (abs(Radian[0] - Radian_To_sift[h][0]) +
                     abs(Radian[1] - Radian_To_sift[h][1]) +
                     abs(Radian[2] - Radian_To_sift[h][2]) +
                     abs(Radian[3] - Radian_To_sift[h][3]) +
                     abs(Radian[4] - Radian_To_sift[h][4]) +
                     abs(Radian[5] - Radian_To_sift[h][5]))
        else:
            GroupNumber[h] = 0
            Min[h] = 0
    
    # 选择最优解
    Min_ = sum(Min)
    TotalGroup = 8
    
    for h in range(8):
        if GroupNumber[h] == 1 and Min_ >= Min[h]:
            Min_ = Min[h]
            TotalGroup = h
    
    # 更新关节角度
    if TotalGroup != 8:
        for i in range(6):
            Radian[i] = Radian_To_sift[TotalGroup][i]

    
    # 处理极小值
    for i in range(6):
        if abs(Radian[i]) < 1e-5:
            Radian[i] = 0
        output_matrix[i] = Radian[i]
    
    finally_matrix = [0.0] * 5
    if jointlimit_min[3] == 0 and jointlimit_max[3] == 0:
        finally_matrix[0] = output_matrix[0]
        finally_matrix[1] = output_matrix[1]
        finally_matrix[2] = output_matrix[2]
        finally_matrix[3] = output_matrix[4]
        finally_matrix[4] = output_matrix[5]
    # 输出结果

    if jointlimit_min[3] == 0 and jointlimit_max[3] == 0:
        print(' '.join(f'{angle:.6f}' for angle in finally_matrix))
        return finally_matrix
    else :
        print(' '.join(f'{angle:.6f}' for angle in output_matrix))
        return output_matrix

def test_kinematics():
    if jointlimit_min[3] == 0 and jointlimit_max[3] == 0:
        matrix = list(map(float, input("请输入5个关节角度（空格分隔）: ").split()))
        matrix_input = [matrix[0], matrix[1], matrix[2],matrix[3], matrix[4]]
    else:
        matrix = list(map(float, input("请输入6个关节角度（空格分隔）: ").split()))
        matrix_input = [matrix[0], matrix[1], matrix[2],matrix[3], matrix[4],matrix[5]]
    result = kinematics(matrix_input)
    rpy = poseTorpy(result)
    print("xyzrpy:",rpy)
    pose = rpyTopose(rpy)
    print("pose:", pose)
    result = inverse_kinematics(pose)
    print("逆运动学结果:", result)

def final_kinematics(fk_or_ik,input_matirx):
    if fk_or_ik == "fk":
        result = kinematics(input_matirx)
        rpy = poseTorpy(result)
        print("xyzrpy:", rpy)
        return rpy
    elif fk_or_ik == "ik":
        pose = rpyTopose(input_matirx)
        result = inverse_kinematics(pose)
        print("逆运动学结果:", result)
        return result
        
if __name__ == "__main__":
    AxisNum = int(input("请输入机械臂轴数："))
    if AxisNum == 5 :
        jointlimit_min[3] = 0;jointlimit_max[3] = 0
        print("当前为五轴机械臂")
    elif AxisNum == 6 :
        jointlimit_min[3] = -180;jointlimit_max[3] = 180
        print("当前为六轴机械臂")
    else:
        print("当前支持五、六轴机械臂")
    choose = input("TestOrRun:")
    if choose == "test":
        test_kinematics()
    elif choose == "run":
        fk_or_ik = input("请输入fkOrik:")
        print(fk_or_ik)
        if fk_or_ik == "fk":
            if jointlimit_min[3] == 0 and jointlimit_max[3] == 0:
                matrix = list(map(float, input("请输入5个关节角度（空格分隔）: ").split()))
                matrix_input = [matrix[0], matrix[1], matrix[2],matrix[3], matrix[4]]
            else:
                matrix = list(map(float, input("请输入6个关节角度（空格分隔）: ").split()))
                matrix_input = [matrix[0], matrix[1], matrix[2],matrix[3], matrix[4],matrix[5]]
        elif fk_or_ik == "ik":
            if jointlimit_min[3] == 0 and jointlimit_max[3] == 0:
                matrix = list(map(float, input("请输入xyzrpy（空格分隔）: ").split()))
                matrix_input = [matrix[0], matrix[1], matrix[2],matrix[3], matrix[4],matrix[5]]
            else:
                matrix = list(map(float, input("请输入xyzrpy（空格分隔）: ").split()))
                matrix_input = [matrix[0], matrix[1], matrix[2],matrix[3], matrix[4],matrix[5]]
        final_kinematics(fk_or_ik,matrix_input)
# 使用 git clone https://github.com/PatrickRayShaw/kinematics.git 下载程序使用
        

