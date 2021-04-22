float dt3=0.005;//注意：dt的取值为kalman滤波器采样时间
float angle3=0.0, angle_dot3=0.0;//角度和角速度
float P3[2][2] = {{ 1, 0 },
                 { 0, 1 }};
float Pdot3[4] ={ 0,0,0,0};
float Q_angle3=0.001, Q_gyro3=0.005; //角度数据置信度,角速度数据置信度
float R_angle3=0.5 ,C_03 = 1;
float q_bias3, angle_err3, PCt_03, PCt_13, E3, K_03, K_13, t_03, t_13;

union result3
 {
		 float d;
		 unsigned char data[4];
 }r13,t13;

void Kalman_Filter3(double angle_m3,double gyro_m3)
{
	angle3+=(gyro_m3-q_bias3) * dt3;
	angle_err3 = angle_m3 - angle3;
	Pdot3[0]=Q_angle3 - P3[0][1] - P3[1][0];
	Pdot3[1]= - P3[1][1];
	Pdot3[2]= - P3[1][1];
	Pdot3[3]=Q_gyro3;
	P3[0][0] += Pdot3[0] * dt3;
	P3[0][1] += Pdot3[1] * dt3;
	P3[1][0] += Pdot3[2] * dt3;
	P3[1][1] += Pdot3[3] * dt3;
	PCt_03 = C_03 * P3[0][0];
	PCt_13 = C_03 * P3[1][0];
	E3 = R_angle3 + C_03 * PCt_03;
	K_03 = PCt_03 / E3;
	K_13 = PCt_13 / E3;
	t_03 = PCt_03;
	t_13 = C_03 * P3[0][1];
	P3[0][0] -= K_03 * t_03;
	P3[0][1] -= K_03 * t_13;
	P3[1][0] -= K_13 * t_03;
	P3[1][1] -= K_13 * t_13;
	angle3 += K_03 * angle_err3; //最优角度
	q_bias3 += K_13 * angle_err3;
	angle_dot3 = gyro_m3-q_bias3;//最优角速度

	r13.d = angle3;
}
