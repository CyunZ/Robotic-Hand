float dt5=0.005;//注意：dt的取值为kalman滤波器采样时间
float angle5=0.0, angle_dot5=0.0;//角度和角速度
float P5[2][2] = {{ 1, 0 },
                 { 0, 1 }};
float Pdot5[4] ={ 0,0,0,0};
float Q_angle5=0.001, Q_gyro5=0.005; //角度数据置信度,角速度数据置信度
float R_angle5=0.5 ,C_05 = 1;
float q_bias5, angle_err5, PCt_05, PCt_15, E5, K_05, K_15, t_05, t_15;

union result5
 {
		 float d;
		 unsigned char data[4];
 }r15,t15;

void Kalman_Filter5(double angle_m5,double gyro_m5)
{
	angle5+=(gyro_m5-q_bias5) * dt5;
	angle_err5 = angle_m5 - angle5;
	Pdot5[0]=Q_angle5 - P5[0][1] - P5[1][0];
	Pdot5[1]= - P5[1][1];
	Pdot5[2]= - P5[1][1];
	Pdot5[3]=Q_gyro5;
	P5[0][0] += Pdot5[0] * dt5;
	P5[0][1] += Pdot5[1] * dt5;
	P5[1][0] += Pdot5[2] * dt5;
	P5[1][1] += Pdot5[3] * dt5;
	PCt_05 = C_05 * P5[0][0];
	PCt_15 = C_05 * P5[1][0];
	E5 = R_angle5 + C_05 * PCt_05;
	K_05 = PCt_05 / E5;
	K_15 = PCt_15 / E5;
	t_05 = PCt_05;
	t_15 = C_05 * P5[0][1];
	P5[0][0] -= K_05 * t_05;
	P5[0][1] -= K_05 * t_15;
	P5[1][0] -= K_15 * t_05;
	P5[1][1] -= K_15 * t_15;
	angle5 += K_05 * angle_err5; //最优角度
	q_bias5 += K_15 * angle_err5;
	angle_dot5 = gyro_m5-q_bias5;//最优角速度

	r15.d = angle5;
}
