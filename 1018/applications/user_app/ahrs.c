/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-15     PC-COLD       the first version
 */
#include<include.h>
uint16_t beta_count=0;
static float SampleTime=0.005f;
static float q0=1.0f,q1=0,q2=0,q3=0;
static float MahonyKp =4.5f;                       // proportional gain governs rate of convergence to accelerometer/magnetometer
static float MahonyKi =0.01f;                         // integral gain governs rate of convergence of gyroscope biases
static float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
float AttMatrix[3][3];

float beta =2.0f;
float qDot1, qDot2, qDot3, qDot4;
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float s0, s1, s2, s3;
//    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))
        return;
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
        return;


    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = sqrt(ax * ax + ay * ay + az * az);
        ax /= recipNorm;
        ay /= recipNorm;
        az /= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = sqrt(mx * mx + my * my + mz * mz);
        mx /= recipNorm;
        my /= recipNorm;
        mz /= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 /= recipNorm;
        s1 /= recipNorm;
        s2 /= recipNorm;
        s3 /= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * SampleTime;
    q1 += qDot2 * SampleTime;
    q2 += qDot3 * SampleTime;
    q3 += qDot4 * SampleTime;

    // Normalise quaternion
    recipNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= recipNorm;
    q1 /= recipNorm;
    q2 /= recipNorm;
    q3 /= recipNorm;

}

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float norm_inv;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
//    if((isnanf(ax)) || (isnanf(ay)) || (isnanf(az))
//        ||(isnanf(mx)) || (isnanf(my)) || (isnanf(mz))
//        ||(isinff(ax))||(isinff(ay))||(isinff(az))
//        ||(isinff(mx))||(isinff(my))||(isinff(mz)))
//    {
//        led_warning();
//        return;
//    }
    if((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))
        return;
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
        return;
    // 先把这些用得到的值算好
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    norm_inv = 1.0f/sqrt(ax*ax + ay*ay + az*az);       //acc数据归一化
    ax = ax * norm_inv;
    ay = ay * norm_inv;
    az = az * norm_inv;

    norm_inv = 1.0f/sqrt(mx*mx + my*my + mz*mz);       //mag数据归一化
    mx = mx * norm_inv;
    my = my * norm_inv;
    mz = mz * norm_inv;
    // Reference direction of Earth's magnetic field
    hx = 2 * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2 * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    hz = 2 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5-q1q1 - q2q2));
    bx = sqrt((hx * hx) + (hy * hy));
    bz =hz;
    // estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
    vx = 2*(q1q3 - q0q2);                                               //四元素中xyz的表示
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;
    wx = 2*(bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
    wy = 2*(bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
    wz = 2*(bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

    norm_inv = 1.0f/sqrt(wx*wx + wy*wy + wz*wz);       //数据归一化
    wx = wx * norm_inv;
    wy = wy * norm_inv;
    wz = wz * norm_inv;

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy) +(my*wz - mz*wy);//                                  //向量外积在相减得到差分就是误差;
    ey = (az*vx - ax*vz) +(mz*wx - mx*wz);//
    ez = (mx*wy - my*wx) +(ax*vy - ay*vx);//

    exInt = exInt + ex * MahonyKi*SampleTime ;//                                 //对误差进行积分
    eyInt = eyInt + ey * MahonyKi*SampleTime;//
    ezInt = ezInt + ez * MahonyKi*SampleTime;//

    // adjusted gyroscope measurements
    gx = gx + MahonyKp*ex + exInt;                                                //将误差PI后补偿到陀螺仪，即补偿零点漂移
    gy = gy + MahonyKp*ey + eyInt;
    gz = gz + MahonyKp*ez + ezInt;                                            //这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

    // integrate quaternion rate and normalise                         //四元素的微分方程
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*SampleTime*0.5f;
    q1 = q1 + (q0*gx + q2*gz - q3*gy) *SampleTime*0.5f;
    q2 = q2 + (q0*gy - q1*gz + q3*gx) *SampleTime*0.5f;
    q3 = q3 + (q0*gz + q1*gy - q2*gx) *SampleTime*0.5f;


    // normalise quaternion
    norm_inv = 1.0f/sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm_inv;
    q1 = q1 * norm_inv;
    q2 = q2 * norm_inv;
    q3 = q3 * norm_inv;
}

void QuaternionReset(void)
{
    q0 = 1.0;
    q1 = 0;
    q2 = 0;
    q3 = 0;
    exInt = 0;
    eyInt = 0;
    ezInt = 0;
    beta_count=0;
}
void QuaternionUpdate(float dt)
{

    float sum;
    SampleTime = dt;
    if(beta_count < 5/dt)
    {
        beta=2.0;
        MahonyKp=10.0f;
        beta_count++;
    }
    else
    {
        sum=sensor.gyro.x*sensor.gyro.x+sensor.gyro.y*sensor.gyro.y+sensor.gyro.z*sensor.gyro.z;
        beta=0.001*sqrt(sum)+0.005;
        if(beta>0.2)
            beta=0.2;
        MahonyKp=0.02*sum+0.5;
        if(MahonyKp>2.0)
            MahonyKp=2.0;
    }
    MadgwickAHRSupdate(sensor.gyro.x,  sensor.gyro.y,  sensor.gyro.z,
                         sensor.acc.x,   sensor.acc.y,   sensor.acc.z,
                         sensor.mag.x,   sensor.mag.y,   sensor.mag.z );
}
void AHRSupdate(float dt)
{
    float rol,pit,yaw;
    float ax,ay,az;
    float Sin_Bias,Cos_Bias;

    rol   = atan2 (2*q2*q3+2*q0*q1,-2*q1*q1-2*q2*q2+1)*57.2957795f ; // roll绕x旋转
    Attitude.Roll.RawValue = rol;

    pit  = asin(-2*q1*q3+2*q0*q2)*57.2957795f ; // pitch绕y旋转
    Attitude.Pitch.RawValue = pit ;

    yaw = -atan2(2*q1*q2+2*q0*q3,-2*q2*q2-2*q3*q3+1)*57.2957795f ; //将机头x↑朝向指向地磁北
    Attitude.Yaw.RawValue = yaw;

    Attitude.Roll.Bias   =   parameters.horiz_offset.x;
    Attitude.Pitch.Bias  =   parameters.horiz_offset.y;
    Attitude.Yaw.Bias    =   parameters.horiz_offset.z;

    Attitude.Roll.Value  =   Attitude.Roll.RawValue-Attitude.Roll.Bias;//

    if(Attitude.Roll.Value>180)
        Attitude.Roll.Value=Attitude.Roll.Value-360;
    else if(Attitude.Roll.Value<-180)
        Attitude.Roll.Value=360+Attitude.Roll.Value;

    Attitude.Pitch.Value =   Attitude.Pitch.RawValue-Attitude.Pitch.Bias;//
    if(Attitude.Pitch.Value>180)
        Attitude.Pitch.Value=Attitude.Pitch.Value-360;
    else if(Attitude.Pitch.Value<-180)
        Attitude.Pitch.Value=360+Attitude.Pitch.Value;

    Attitude.Yaw.Value   =   -(Attitude.Yaw.RawValue-Attitude.Yaw.Bias);//
    if(Attitude.Yaw.Value>180)
        Attitude.Yaw.Value=Attitude.Yaw.Value-360;
    else if(Attitude.Yaw.Value<-180)
        Attitude.Yaw.Value=360+Attitude.Yaw.Value;
    //地理坐标系统一为东北天
    //东北天分别对应XYZ正方向

    //机体坐标系：机头为y正方向，机头右侧为x轴正方向，垂直于xy平面向上为z轴正方向
    //roll,pitch,yaw对应绕y,x,z旋转
    // 载体坐标下的x方向向量，单位化。

    //四元数转换矩阵
    AttMatrix[0][0] = 1 - (2*q2*q2 + 2*q3*q3);
    AttMatrix[0][1] = 2*q1*q2 - 2*q0*q3;
    AttMatrix[0][2] = 2*q1*q3 + 2*q0*q2;

    // 载体坐标下的y方向向量，单位化。
    AttMatrix[1][0] = 2*q1*q2 + 2*q0*q3;
    AttMatrix[1][1] = 1 - (2*q1*q1 + 2*q3*q3);
    AttMatrix[1][2] = 2*q2*q3 - 2*q0*q1;

    // 载体坐标下的z方向向量（等效重力向量、重力加速度向量），单位化。
    AttMatrix[2][0] = 2*q1*q3 - 2*q0*q2;
    AttMatrix[2][1] = 2*q2*q3 + 2*q0*q1;
    AttMatrix[2][2] = 1 - (2*q1*q1 + 2*q2*q2);

    ax = sensor.acc.x - AttMatrix[2][0];//
    ay = sensor.acc.y - AttMatrix[2][1];//
    az = sensor.acc.z - AttMatrix[2][2];//

    sensor.w_acc.x = (AttMatrix[0][0] * ax + AttMatrix[0][1] * ay + AttMatrix[0][2] * az)*9.8;
    sensor.w_acc.y = (AttMatrix[1][0] * ax + AttMatrix[1][1] * ay + AttMatrix[1][2] * az)*9.8;
    sensor.w_acc.z = (AttMatrix[2][0] * ax + AttMatrix[2][1] * ay + AttMatrix[2][2] * az)*9.8;//机体加速度转换到地理坐标系


    //地理坐标系加速度转换到UWB系
    Sin_Bias = sin((-Attitude.Yaw.Bias)*0.0174533f);
    Cos_Bias = cos((-Attitude.Yaw.Bias)*0.0174533f);

    sensor.uwb_acc.x = Cos_Bias*sensor.w_acc.x+Sin_Bias*sensor.w_acc.y;
    sensor.uwb_acc.y = -Sin_Bias*sensor.w_acc.x+Cos_Bias*sensor.w_acc.y;
    sensor.uwb_acc.z = sensor.w_acc.z;

    Position.x.Speed.Observe += sensor.w_acc.x * dt;
//    Position.y.Speed.Observe += sensor.uwb_acc.y * dt;
//    Position.z.Speed.Observe += sensor.w_acc.z * dt;
}
