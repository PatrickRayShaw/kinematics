# Kinematics

To simplify the computation of kinematics, I release the related code to contribute to the advancement of the robotics industry.

为了简化运动学的计算，我公开了相关代码，为机器人工业的发展贡献绵薄之力。

## Usage
git clone https://github.com/PatrickRayShaw/kinematics.git

### Details
According to the DH parameters and the initial pose offset of your robotic arm, you can use this code to compute kinematics for four-axis, five-axis, or six-axis robotic arms.

根据DH参数和机械臂的初始姿态偏移量（默认竖直向上），可以用它来计算四轴、五轴或六轴机械臂的运动学。

#### Six-Axis Robotic Arm
- No changes needed.

#### Five-Axis Robotic Arm
- Change the fourth axis angle range to 0.

#### Four-Axis Robotic Arm
- Change the fourth axis angle range to 0 and set dt to 0.
