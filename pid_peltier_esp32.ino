/*Add touch sensor for simple-experiment*/
/*ペルチェ素子をPID制御で温度を一定に保つためのプロジェクト*/

/*サーミスタの入力ピン*/
#define TEMP_1_VOLT 27

/*温度計算のための変数定義*/
#define  OUT_OF_MEASUREMENT_RANGE -1
int data_count = 1000;
double data_total ;
double time_data = 0;
double data_ave;
double Vref = 3.3;
double calib_volt;
double resistance = 10.0;
double temp_resistance;
double temp;

/*温度計測値*/
double temp_1;

/*ESP32のPWM制御のための変数定義*/
#include"esp32-hal-ledc.h"
/*PWM制御のためのセットアップ定数*/
#define pwmPin_1 14
#define pwmPin_2 2
const int pwmChannel_1 = 0;
const int pwmChannel_2 = 1;/*pwmの識別チャンネル(任意の値で構わない)*/
const int freq = 12800; /*周波数*/
const int resultion = 8; /*分解能*/

/*PIDパラメータの設定*/
double kp = 30.00;
double ki = 0;
double kd = 0.15;
double targetTemp = 40.00;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double cumError, rateError;
double pwmOut;
int pwm = 0;

/*esp32内部のタッチセンサを利用するための設定*/
const int touchPin = T0;
const int threshold = 14;
int touchValue;




void setup() {
  Serial.begin(115200);
  pinMode(pwmPin_1, OUTPUT);
  ledcSetup(pwmChannel_1, freq, resultion);
  ledcAttachPin(pwmPin_1, pwmChannel_1);
  pinMode(pwmPin_2, OUTPUT);
  ledcSetup(pwmChannel_2, freq, resultion);
  ledcAttachPin(pwmPin_2, pwmChannel_2);
}

void loop() {
  time_data = micros() / 1000000.00;
  /*温度計測*/
  temp_1 = temp_calc(TEMP_1_VOLT);

  Serial.print("temp;");
  Serial.print(temp_1);
  /*PWM制御*/
  pwmOut = biologicalTempControl(temp_1);
  if (pwmOut > 0) {
    ledcWrite(pwmChannel_1, 0);
    ledcWrite(pwmChannel_2, pwmOut);
    //ledcWrite(pwmChannel_2, 0);
  }
  //  else if (temp_3 > 37) {
  //    ledcWrite(pwmChannel_1, 255);
  //    ledcWrite(pwmChannel_2, 0);
  //  }
  else {
    ledcWrite(pwmChannel_1, abs(pwmOut));
    ledcWrite(pwmChannel_2, 0);
  }

  /*タッチセンサ*/
  touchValue = touchRead(touchPin);
  Serial.print("touch value;");
  Serial.print(touchValue);
  if( touchValue < threshold){
    Serial.println("true");
  }
  else{
    Serial.println("false");
  }
}

/*取得したデータを温度を算出する関数*/
double temp_calc(int pin_num) {
  data_total = 0.0;
  for (int i = 0; i < data_count; i++) {
    data_total += analogRead(pin_num);
  }

  /*adc値の平均の計算*/
  data_ave = data_total / data_count;

  /*adc値を電圧値とキャリブレーションをした値に変換する*/
  calib_volt = (0.9982 * data_ave * Vref / 4096) + 0.1245;

  /*電圧の出力*/
  if (0.2 < calib_volt && calib_volt <= 2.8) {
    calib_volt = (0.9982 * data_ave * Vref / 4096) + 0.1245;
  } else if (calib_volt > 2.8) {
    return OUT_OF_MEASUREMENT_RANGE;
  } else {
    return OUT_OF_MEASUREMENT_RANGE;
  }

  /*出力された電圧値からサーミスタの抵抗値を算出*/
  temp_resistance = ( Vref * resistance - resistance * calib_volt ) / calib_volt;
  //  Serial.println(temp_resistance);

  /*サーミスタの温度特性値から抵抗値と固定抵抗(resistance)の値から温度の算出*/
  temp =  (1 / ((log(temp_resistance / resistance)) / 3435.0 + 1 / (25.0 + 273.15))) - 273.15 ;

  /*温度の出力*/
  return temp;
}

/*PID制御のための関数*/
double biologicalTempControl(double currentTemp) {
  currentTime = millis();

  elapsedTime = currentTime - previousTime;
  error = targetTemp - currentTemp;

  cumError += error * elapsedTime;
  rateError = (error - lastError) / elapsedTime;

  double out = kp * error + ki * cumError + kd * rateError;

  lastError = error;
  previousTime = currentTime;

  out = constrain(out, -255, 255);
  return out;
}
