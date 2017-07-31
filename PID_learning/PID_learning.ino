
#define PID_Uint struct pid_uint

PID_Uint{
  int U_kk;
  int ekk;
  int ekkk;
  int Ur;
  int Un;

  int Kp;    // 比例，从小往大调
  int Ti;    // 积分，从大往小调
  int Td;    // 微分，？？
  int k1;
  int k2;
  int k3;
  };

/*
 * 函 数 名：Init_PID_uint()
 * 功    能：初始化PID参数
 * 说    明：调用 本函数之前，应该先对Kp, Ti, Td 做设置
 * 入口参数：PID单元的参数结构体 地址
 * 返 回 值：无
 */
 void Init_PID_uint(PID_Uint *p)    // p 是一个指向 PID_Uint结构体 的一个指针
 {
  p->k1 = (p->Kp) + (p->Kp)*1024 / (p->Ti) + (p->Kp)*(p->Td) / 1024;
  p->k2 = (p->Kp) + 2*(p->Kp)*(p->Td)/1024;
  p->k3 = (p->Kp) * (p->Td) / 1024;
  }

/*
 * 函 数 名：reset_Uk()
 * 功    能：初始化U_kk, ekk, ekkk
 * 说    明：在初始化时调用， 改变PID参数时 有可能需要调用
 * 入口参数：PID单元的参数结构体 地址
 * 返 回 值：无
 */
 void reset_Uk(PID_Uint *p)
 {
    p->U_kk = 0;
    p->ekk = 0;
    p->ekkk = 0;
  }
/*
 * 函 数 名：PID_commen(int set, int jiance, PID_Uint *p)
 * 功    能：通用PID函数
 * 说    明：求任意单个PID的控制量
 * 入口参数：期望值， 实测值， PID单元结构体
 * 返 回 值：PID控制量
 */
 int PID_commen(int set, int jiance, PID_Uint *p)
 {
    int ek, U_k=0;
    ek = jiance - set;
    
    if((ek > (p->Un)) || (ek < -(p->Un)))      // 积分不灵敏区
         U_k = (p->U_kk) + (p->k1)*ek - (p->k2)*(p->ekk) + (p->k3)*(p->ekkk); 

    p->U_kk = U_k;
    p->ekkk = p->ekk;
    p->ekk  = ek;
    
    if( U_k > (p->Ur))
      U_k = p->Ur;
    
    if( U_k < -(p->Ur))
      U_k = -(p->Ur);

    return U_k / 1024;  // 归一化

  }
