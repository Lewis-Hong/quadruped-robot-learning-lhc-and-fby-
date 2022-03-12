#include <webots/robot.h>
#include <webots/motor.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
  
#define TIME_STEP 8
#define pi 3.14159
#define PI 3.14159
#define FAI	 0.5 	  



WbDeviceTag servos[12];
const char *SERVO_NAMES[] = { 
  "rf_joint_3", "rb_joint_3", "lf_joint_3", "lb_joint_3",
  "rf_joint_2", "rb_joint_2", "lf_joint_2", "lb_joint_2",
  "rf_joint_1", "rb_joint_1", "lf_joint_1", "lb_joint_1",
  NULL 
};

// 四足机器人足端坐标结构体
struct QRFoot{
  float x[4]; // 足端竖直方向x坐标
  float y[4]; // 足端竖直方向y坐标
  float z[4]; // 足端水平方向x坐标
} foot;

// 四足机器人关节角结构体
struct QRJoint{
  float theta0[4]; // 大腿外摆角
  float theta1[4]; // 大腿关节角
  float theta2[4]; // 小腿关节角
} joint;

// 四足机器人结构参数结构体
struct QRBody{
  float l0; // 髋关节长度
  float l1; // 大腿长度
  float l2; // 小腿长度
} body = {0.0241, 0.052, 0.04785};


void ik(void){
  float l0 = body.l0;
  float l1 = body.l1;
  float l2 = body.l2;

  float *x = foot.x;
  float *y = foot.y;
  float *z = foot.z;
  
  float a, a2;
  for(int i=0; i<4; i++)
    {
      a2 = y[i]*y[i]+z[i]*z[i]-l0*l0;
      a = sqrt(a2);
      if(i==1 || i==3){
        joint.theta0[i] = atan(y[i]/z[i]) + atan(l0/a);
      }
      else{
        joint.theta0[i] = - atan(y[i]/z[i]) - atan(l0/a);
      }
      joint.theta1[i] = - acos((l1+(x[i]*x[i]+a2-l1*l1-l2*l2)/(2*l1))/sqrt(x[i]*x[i]+a2)) + atan(x[i]/a);
      joint.theta2[i] = acos((x[i]*x[i]+a2-l1*l1-l2*l2)/(2*l1*l2));
    }
  //set_robot_joint
}

//步态时序、足端规划生成
void generate_time(float it, float n){

}
float t;
float init_z=-0.075;
float init_x=0.01;
float init_y=0.025;
void run_trot(float dir, float step, float lift_h, float TS){
  t = wb_robot_get_time();
  int chu = t/TS;
  t = t - chu*TS;
  
  if(t > TS){t = t - TS;}
	
  float sigma;
  float zep_b, xep_b, yep_b; 
  float xep_z, yep_z; 		
  
  if(dir != -1){
    float xs = -step*cos(dir)*0.7;
    float xf = step*cos(dir)*0.3;
    float ys = -step*sin(dir)*0.7;
    float yf = step*sin(dir)*0.3;
    // 支撑相
    if (t<=TS*FAI){
      sigma = 2*PI*t/(FAI*TS);
      zep_b = lift_h*(1-cos(sigma))/2;
      xep_b = ((xf-xs)*((sigma-sin(sigma))/(2*PI))+xs);
      yep_b = ((yf-ys)*((sigma-sin(sigma))/(2*PI))+ys); 
	
      xep_z = (xs-xf)*((sigma-sin(sigma))/(2*PI))+xf;
      yep_z = (ys-yf)*((sigma-sin(sigma))/(2*PI))+yf;
		
      foot.z[0] = init_z + zep_b;
      foot.z[3] = init_z + zep_b;
      foot.z[1] = init_z;
      foot.z[2] = init_z;
		
      foot.x[0] = init_x+xep_b;
      foot.x[1] = init_x+xep_z;
      foot.x[2] = init_x+xep_z;
      foot.x[3] = init_x+xep_b;
      
      foot.y[0] = init_y-yep_b;
      foot.y[1] = (init_y-yep_z);
      foot.y[2] = init_y+yep_z;
      foot.y[3] = (init_y+yep_b);

    }
    // 摆动相
    else if (t>TS*FAI && t<=TS){
      sigma=2*PI*(t-TS*FAI)/(FAI*TS);
		
      zep_b = lift_h*(1-cos(sigma))/2;
      xep_b = ((xf-xs)*((sigma-sin(sigma))/(2*PI))+xs);
      yep_b = ((yf-ys)*((sigma-sin(sigma))/(2*PI))+ys);
      	
      xep_z = (xs-xf)*((sigma-sin(sigma))/(2*PI))+xf;
      yep_z = (ys-yf)*((sigma-sin(sigma))/(2*PI))+yf;
	
      foot.z[1] = init_z + zep_b;
      foot.z[2] = init_z + zep_b;
      foot.z[0] = init_z;
      foot.z[3] = init_z;
		
      foot.x[0] = init_x+xep_z;
      foot.x[1] = init_x+xep_b;
      foot.x[2] = init_x+xep_b;
      foot.x[3] = init_x+xep_z;	
      
      foot.y[0] = init_y-yep_z;
      foot.y[1] = (init_y-yep_b);
      foot.y[2] = init_y+yep_b;
      foot.y[3] = (init_y+yep_z);	
    }
  }
  else if(dir == -1){
    for(int i = 0; i < 4; i++){
       foot.x[i] = init_x;
       foot.y[i] = init_y;
     }
    // 支撑相
    if (t<=TS*FAI){
      sigma = 2*PI*t/(FAI*TS);
      zep_b = lift_h*(1-cos(sigma))/2;
      
      foot.z[0] = init_z + zep_b;
      foot.z[3] = init_z + zep_b;
      foot.z[1] = init_z;
      foot.z[2] = init_z;
    }
    // 摆动相
    else if (t>TS*FAI && t<=TS){
      sigma=2*PI*(t-TS*FAI)/(FAI*TS);	
      zep_b = lift_h*(1-cos(sigma))/2;
     
      foot.z[1] = init_z + zep_b;
      foot.z[2] = init_z + zep_b;
      foot.z[0] = init_z;
      foot.z[3] = init_z;
    }
  }
     	
}

void run_test(){
  t = wb_robot_get_time();
  joint.theta2[0]= 0.5*sin(2*t);
  printf("%f\n", t);
}
  
  
  
float spd[12];
float acc[12];
void servos_other_init(){
  for(int i = 0; i < 12; i++){
    spd[i] = 10;
    acc[i] = 100;
  }
  printf("servo init done!\n");
}

void WritePosEx(int ID, float Position, float Speed, float ACC){
  wb_motor_set_velocity(servos[ID-1], Speed);
  //wb_motor_set_acceleration(servos[ID-1], ACC);
  wb_motor_set_position(servos[ID-1], Position);
}
void SyncWritePosEx(){
  for(int i = 0; i < 4; i++){
      WritePosEx(i+1, joint.theta2[i], spd[i], acc[i]);
      WritePosEx(i+5, joint.theta1[i], spd[i], acc[i]);
      WritePosEx(i+9, joint.theta0[i], spd[i], acc[i]);
    }
}



int main(int argc, char **argv)
{
  wb_robot_init();
  printf("robot inited!\n");

  //将电机设备逐一加载
  for (int i = 0; SERVO_NAMES[i]; i++) {
    servos[i] = wb_robot_get_device(SERVO_NAMES[i]);
    //assert(servos[i]);
  }
  servos_other_init();

  //进入仿真执行
  while(wb_robot_step(TIME_STEP) != -1) {
    if(wb_robot_get_time() > 0.5){break;}
  };
  
  init_x = 0; init_y = 0.015; init_z = -0.076;
  while(wb_robot_step(TIME_STEP) != -1) {
    run_trot(-1, 0.03, 0.015, 0.4);
    ik();
    SyncWritePosEx();
    if(wb_robot_get_time() > 4){break;}
  };
  
  while(wb_robot_step(TIME_STEP) != -1) {
    if(wb_robot_get_time() > 5){break;}
  };
  
  init_x = 0; init_y = 0.015; init_z = -0.076;
  while(wb_robot_step(TIME_STEP) != -1) {
    run_trot(0, 0.03, 0.014, 0.4);
    ik();
    SyncWritePosEx();
    if(wb_robot_get_time() > 12){break;}
  };
  init_x = 0; init_y = 0.015; init_z = -0.076;
  while(wb_robot_step(TIME_STEP) != -1) {
    run_trot(-1, 0.03, 0.018, 0.6);
    ik();
    SyncWritePosEx();
    if(wb_robot_get_time() > 17){break;}
  };
  
  
  init_x = 0; init_y = 0.025; init_z = -0.076;
  while(wb_robot_step(TIME_STEP) != -1) {
    run_trot(1.57, 0.025, 0.012, 0.4);
    ik();
    SyncWritePosEx();
    if(wb_robot_get_time() > 23){break;}
  };
  
  while(wb_robot_step(TIME_STEP) != -1) {
    if(wb_robot_get_time() > 23.7){break;}
  };
  
  init_x = 0; init_y = 0.015; init_z = -0.076;
  while(wb_robot_step(TIME_STEP) != -1) {
    run_trot(-1, 0.03, 0.018, 0.6);
    ik();
    SyncWritePosEx();
    if(wb_robot_get_time() > 28){break;}
  };
  
  init_x = 0; init_y = 0.025; init_z = -0.076;
  while(wb_robot_step(TIME_STEP) != -1) {
    run_trot(3.14, 0.025, 0.012, 0.4);
    ik();
    SyncWritePosEx();
    if(wb_robot_get_time() > 35){break;}
  };
  init_x = 0; init_y = 0.015; init_z = -0.076;
  while(wb_robot_step(TIME_STEP) != -1) {
    run_trot(-1, 0.03, 0.018, 0.6);
    ik();
    SyncWritePosEx();
    if(wb_robot_get_time() > 38){break;}
  };
  
  init_x = 0; init_y = 0.025; init_z = -0.076;
  while(wb_robot_step(TIME_STEP) != -1) {
    run_trot(0.8, 0.025, 0.012, 0.4);
    ik();
    SyncWritePosEx();
    if(wb_robot_get_time() > 45){break;}
  };
  //清理
  wb_robot_cleanup();
  return 0;
}






