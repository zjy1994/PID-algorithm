void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;
int SampleTime = 1000;    //采样周期 1 秒

/*
 * 函 数 名：Compute()
 * 功    能：计算Output
 * 说    明：
 * 入口参数：无
 * 返 回 值：无
 */
void Compute()
{
 /*How long since we last calculated*/  
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);
 
  if(timeChange > SampleTime)
  { 
    /*Compute all the working error variables*/
      double error = Setpoint - Input;
      errSum += error;
      double dErr = (error - lastErr);
  
      /*Compute PID Output*/
      Output = kp * error + ki * errSum + kd * dErr;
  
     /*Remember some variables for next time*/
     lastErr = error;
     lastTime = now;
  }
}
  
void SetTunings(double Kp, double Ki, double Kd)
{
      double SampleTimeInSec = ((double)SampleTime)/1000;  // 把时间转化为秒
   
      kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}

/*
 * 函 数 名：SetSampleTime()
 * 功    能：设置 采样周期
 * 说    明：
 * 入口参数：新的 采样周期
 * 返 回 值：无
 */
void SetSampleTime(int NewSampleTime)
{
  if(NewSampleTime > 0){
    double ratio = (double)NewSampleTime / (double)SampleTime;

    ki *= ratio;
    kd /= ratio;
    
    SampleTime = (unsigned long)NewSampleTime;
    }
  }


   
  
