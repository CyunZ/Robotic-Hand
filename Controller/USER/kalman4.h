float dt4=0.005;//注意：dt的取值为kalman滤波器采样时间
float angle4=0.0, angle_dot4=0.0;//角度和角速度
float P4[2][2] = {{ 1, 0 },
                 { 0, 1 }};
float Pdot4[4] ={ 0,0,0,0};
float Q_angle4=0.001, Q_gyro4=0.005; //角度数据置信度,角速度数据置信度
float R_angle4=0.5 ,C_04 = 1;
float q_bias4, angle_err4, PCt_04, PCt_14, E4, K_04, K_14, t_04, t_14;

union result4
 {
		 float d;
		 unsigned char data[4];
 }r14,t14;

void Kalman_Filter4(double angle_m4,double gyro_m4)
{
	angle4+=(gyro_m4-q_bias4) * dt4;
	angle_err4 = angle_m4 - angle4;
	Pdot4[0]=Q_angle4 - P4[0][1] - P4[1][0];
	Pdot4[1]= - P4[1][1];
	Pdot4[2]= - P4[1][1];
	Pdot4[3]=Q_gyro4;
	P4[0][0] += Pdot4[0] * dt4;
	P4[0][1] += Pdot4[1] * dt4;
	P4[1][0] += Pdot4[2] * dt4;
	P4[1][1] += Pdot4[3] * dt4;
	PCt_04 = C_04 * P4[0][0];
	PCt_14 = C_04 * P4[1][0];
	E4 = R_angle4 + C_04 * PCt_04;
	K_04 = PCt_04 / E4;
	K_14 = PCt_14 / E4;
	t_04 = PCt_04;
	t_14 = C_04 * P4[0][1];
	P4[0][0] -= K_04 * t_04;
	P4[0][1] -= K_04 * t_14;
	P4[1][0] -= K_14 * t_04;
	P4[1][1] -= K_14 * t_14;
	angle4 += K_04 * angle_err4; //最优角度
	q_bias4 += K_14 * angle_err4;
	angle_dot4 = gyro_m4-q_bias4;//最优角速度

	r14.d = angle4;
}
