float dt2=0.005;//注意：dt的取值为kalman滤波器采样时间
float angle2=0.0, angle_dot2=0.0;//角度和角速度
float P2[2][2] = {{ 1, 0 },
                 { 0, 1 }};
float Pdot2[4] ={ 0,0,0,0};
float Q_angle2=0.001, Q_gyro2=0.005; //角度数据置信度,角速度数据置信度
float R_angle2=0.5 ,C_02 = 1;
float q_bias2, angle_err2, PCt_02, PCt_12, E2, K_02, K_12, t_02, t_12;

union result2
 {
		 float d;
		 unsigned char data[4];
 }r12,t12;

void Kalman_Filter2(double angle_m2,double gyro_m2)
{
	angle2+=(gyro_m2-q_bias2) * dt2;
	angle_err2 = angle_m2 - angle2;
	Pdot2[0]=Q_angle2 - P2[0][1] - P2[1][0];
	Pdot2[1]= - P2[1][1];
	Pdot2[2]= - P2[1][1];
	Pdot2[3]=Q_gyro2;
	P2[0][0] += Pdot2[0] * dt2;
	P2[0][1] += Pdot2[1] * dt2;
	P2[1][0] += Pdot2[2] * dt2;
	P2[1][1] += Pdot2[3] * dt2;
	PCt_02 = C_02 * P2[0][0];
	PCt_12 = C_02 * P2[1][0];
	E2 = R_angle2 + C_02 * PCt_02;
	K_02 = PCt_02 / E2;
	K_12 = PCt_12 / E2;
	t_02 = PCt_02;
	t_12 = C_02 * P2[0][1];
	P2[0][0] -= K_02 * t_02;
	P2[0][1] -= K_02 * t_12;
	P2[1][0] -= K_12 * t_02;
	P2[1][1] -= K_12 * t_12;
	angle2 += K_02 * angle_err2; //最优角度
	q_bias2 += K_12 * angle_err2;
	angle_dot2 = gyro_m2-q_bias2;//最优角速度

	r12.d = angle2;
}
