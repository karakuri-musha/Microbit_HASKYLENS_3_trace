#include "HUSKYLENS.h"                                            // HUSKYLENS制御ライブラリ
#include "LEDMtrix_Code.h"                                        // LEDマトリックス用BMP定義
#include <Adafruit_Microbit.h>                                    // micro:bitのLEDマトリックス制御ライブラリ

HUSKYLENS huskylens;                                              // HUSKYLENSの生成

Adafruit_Microbit_Matrix mb_matrix;                               // micro:bitのLEDマトリックス生成

const int buttonA = 5;                                            // A ボタン GPIO pin
const int buttonB = 11;                                           // B ボタン GPIO pin

const int motor_right_pwm = 13;                                   // motor_Right GPIO pin
const int motor_left_pwm  = 14;                                   // motor_left GPIO pin
const int motor_right_on = 15;                                    // motor_Right on/off switch
const int motor_left_on = 16;                                     // motor_Left on/off switch
                                                    
void printResult(HUSKYLENSResult result);                         // シリアルへの認識結果出力関数

int xLeft   = 160-40;                                             // 左モータの速度（PWM）格納用
int xRight  = 160+40;                                             // 右モータの速度（PWM）格納用

int  widthLevel     = 100;                                        // バンディングボックスの許容幅の格納用
bool isTurning      = false;                                      // 旋回状態の管理用
bool isTurningLeft  = true;                                       // 左旋回状態の管理用

int setup_stat  = 1;                                              // 動作モード（学習／実行）の設定用


// -------------------------------------------------------------
// 認識したバンディングボックスの中心点が指定の範囲内にあるかを判定
// -------------------------------------------------------------
bool isInside(int value, int min, int max){
    return (value >= min && value <= max);
}

// -------------------------------------------------------------
// PWM 制御によるモーター制御（ 前進　1～1023  後進　-1～-1023）
// -------------------------------------------------------------
void tamiya_robot_speed (int16_t left_speed, int16_t right_speed) {
  digitalWrite (motor_right_on, LOW);                             // 右モーターをON状態にする
  digitalWrite (motor_left_on, LOW);                              // 左モーターをON状態にする
  analogWrite (motor_right_pwm, right_speed);                     // 右モーターのスピードを設定
  analogWrite (motor_left_pwm, left_speed);                       // 左モーターのスピードを設定
  
}

// -------------------------------------------------------------
// 初期化処理
// -------------------------------------------------------------
void setup() {  
  Serial.begin(9600);                                             // シリアル接続の開始
  Serial.println("microbit is ready!");                           // シリアルコンソールへの出力
  
  mb_matrix.begin();                                              // LEDマトリックス制御の開始
  mb_matrix.print("HELLO!");                                      // LEDマトリックスに"HELLO！"をスクロール表示

  pinMode(buttonA, INPUT);                                        // micro:bitのAボタン用GPIOを入力に設定
  pinMode(buttonB, INPUT);                                        // micro:bitのBボタン用GPIOを入力に設定
  
  pinMode(motor_right_on, OUTPUT);                                // 右モーターのON/OFF状態選択用GPIOを出力に設定
  pinMode(motor_left_on, OUTPUT);                                 // 左モーターのON/OFF状態選択用GPIOを出力に設定
  digitalWrite(motor_right_on, HIGH);                             // 右モーターを停止状態に設定
  digitalWrite(motor_left_on, HIGH);                              // 左モーターを停止状態に設定

  Wire.begin();                                                   // I2C バスに接続
  while (!huskylens.begin(Wire))
  {
    Serial.println(F("Begin failed!"));                           // HUSKYLENSとI2C接続を開始できない場合
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }
}

// -------------------------------------------------------------
// 繰り返し処理
// -------------------------------------------------------------
void loop(){
  // -----------------------------------------------------------
  // microbit 「ボタンＡ」が押された場合の処理
  // -----------------------------------------------------------
  if (! digitalRead(buttonA)) { 
                                     
    if (setup_stat == 1) {                                        // 起動後モード選択で　Aボタン押下時：再学習モード（リセット）
      huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);        // アルゴリズムをObject Tracking に変更
      if (!huskylens.request(1)) {                                // HUSKYLENSの動作確認
        Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
        mb_matrix.print("ERR-1"); }

      setup_stat = 2;                                             // 動作モードを学習モードに変更
      huskylens.writeForget();                                    // HUSKYLENSのモデルをリセット
      mb_matrix.show(A_bmp);                                      // microbit のLEDマトリックスに「A」を表示
      delay(1000);
      mb_matrix.print("SETUP MODE OBJECT");                       // microbit のLEDマトリックスに設定モードであると表示

    } 
    else if (setup_stat == 2) {                                   // 学習モード中にAボタンが押された場合
      while (!huskylens.writeLearn(1)) {                          // 追尾対象のオブジェクト学習結果を保存
        Serial.println(F("learn object failed!")); 
        delay(100);
      }
      mb_matrix.print("OK");                             
      delay(2000);                                            
      huskylens.saveModelToSDCard(1);                             // 学習モデルをSDカードに保存
      mb_matrix.print("SAVE OK");                                 // 保存結果をLEDマトリックスに表示
      delay(15000);                                               // 保存待ち（15秒）
      setup_stat = 3;                                             // 実行モードに移行
      mb_matrix.print("GO");                                      // microbit のLEDマトリックスに既存モデルロードであると表示
    }
  }
  
  // -----------------------------------------------------------
  // microbit 「ボタンＢ」が押された場合の処理
  // -----------------------------------------------------------
  if (! digitalRead(buttonB)) {
                                    
    if (setup_stat == 1) {                                        // 起動後モード選択で　Aボタン押下時：再学習モード（リセット）
      huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);        // アルゴリズムをObject Tracking に変更
      if (!huskylens.request(1)) {                                // HUSKYLENSの動作確認
        Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
        mb_matrix.print("ERR-1"); }

      setup_stat = 2;                                             // 動作モードを学習モードに変更
      huskylens.writeForget();                                    // HUSKYLENSのモデルをリセット
      mb_matrix.show(B_bmp);                                      // microbit のLEDマトリックスに「A」を表示
      delay(1000);
      mb_matrix.print("SETUP MODE TAG");                          // microbit のLEDマトリックスに設定モードであると表示

    } 
    else if (setup_stat == 2) {                                   // 学習モード中にAボタンが押された場合
      while (!huskylens.writeLearn(1)) {                          // 追尾対象のオブジェクト学習結果を保存
        Serial.println(F("learn object Tag1 failed!")); 
        delay(100);
      }
      mb_matrix.print("OK");                             
      delay(2000);                                            
      huskylens.saveModelToSDCard(1);                             // 学習モデルをSDカードに保存
      mb_matrix.print("SAVE OK");                                 // 保存結果をLEDマトリックスに表示
      delay(15000);                                               // 保存待ち（15秒）
      setup_stat = 3;                                             // 実行モードに移行
      mb_matrix.print("GO");                                      // microbit のLEDマトリックスに既存モデルロードであると表示
    }
  }

  // -----------------------------------------------------------
  // 実行中ステータスの場合の処理（ロボット動作）
  // -----------------------------------------------------------
  if (setup_stat == 3) {                                          // 実行モードで処理

    //　対象のオブジェクトが認識されたかを確認
    if (!huskylens.request(1)) {Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));}
    else if(!huskylens.isLearned()) {Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));}
    else if(!huskylens.available()) {                             // 追尾対象が認識されない場合
      Serial.println(F("No block Tag1 appears on the screen!"));
      digitalWrite (motor_right_on, HIGH);                        // 右モーターをOFF状態にする
      digitalWrite (motor_left_on, HIGH);                         // 左モーターをOFF状態にする
    }
    else                                                          // 追尾対象が認識された場合
    {
      HUSKYLENSResult result = huskylens.read();                  // 追尾対象の認識結果を取得

      printResult(result);                                        // 認識結果をシリアルコンソールへ出力

      if (result.width < widthLevel){                             // 追尾対象のバンディングボックスの幅が指定値以下（対象が遠い）かの判定
                                                                  // 追尾対象が離れている（バンディングボックスの幅が、指定値よりも小さい）場合の処理
        if (isInside(result.xCenter, 0, xLeft)){                  // 追尾対象が左側にある（バンディングボックスの中心座標が0≦xCenter≦120に入る）場合の処理
          if (isTurning){                                         // isTurning がTrueの場合
            isTurning = false;                                    // isTurning をFalseへ変更
          }
          tamiya_robot_speed(250-50,250);                       // 左前方へ前進
        }
        else if (isInside(result.xCenter, xLeft, xRight)){        // 追尾対象が中心にある（バンディングボックスの中心座標が120≦xCenter≦200に入る）場合の処理
          if (isTurning){                                         // isTurningLeft がTrueの場合
            isTurning = false;                                    // isTurning をFalseへ変更
          }
          tamiya_robot_speed(255,255);                            // 前進
        }
        else if (isInside(result.xCenter, xRight, 320)){          // 追尾対象が右側にある（バンディングボックスの中心座標が200≦xCenter≦320に入る）場合の処理
          if (isTurning){                                         // isTurning がTrueの場合
            isTurning = false;                                    // isTurning をFalseへ変更
          }
          tamiya_robot_speed(250,250-50);                         // 右前方へ前進
        }
      }
      else {                                                      // 追尾対象のバンディングボックスの幅が、指定値よりも大きい（近い）場合の処理
        if (isInside(result.xCenter, 0, xLeft)){                  // 追尾対象が左側にある（バンディングボックスの中心座標が0≦xCenter≦120に入る）場合の処理
          isTurning = true;                                       // isTurning をTrueへ変更
          tamiya_robot_speed(-250,250);                           // 左旋回
        }
        else if (isInside(result.xCenter, xLeft, xRight)){        // 追尾対象が中心にある（バンディングボックスの中心座標が120≦xCenter≦200に入る）場合の処理
          digitalWrite (motor_right_on, HIGH);                    // 右モーターをOFF状態にする
          digitalWrite (motor_left_on, HIGH);                     // 左モーターをOFF状態にする
        }
        else if (isInside(result.xCenter, xRight, 320)){          // 追尾対象が右側にある（バンディングボックスの中心座標が200≦xCenter≦320に入る）場合の処理
          isTurning = true;                                       // isTurning をTrueへ変更
          tamiya_robot_speed(250,-250);                           // 右旋回
        }
      }
    }
  }
}
// -------------------------------------------------------------
// HUSKYLENS認識結果のシリアルコンソールへの出力関数
// -------------------------------------------------------------
void printResult(HUSKYLENSResult result){                         
    if (result.command == COMMAND_RETURN_BLOCK){                  // バンディングボックスを認識した場合の処理
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){             // 経路（矢印）を認識した場合の処理 
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        Serial.println("Object unknown!");                        // 認識不明の場合
    }
}
