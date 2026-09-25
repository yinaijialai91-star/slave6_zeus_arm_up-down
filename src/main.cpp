#include <DJIMotorCtrlESP.hpp>
#include <HXC_TWAI.hpp>

#define TX_PIN D2
#define RX_PIN D3

#define SLAVE1_WHEEL_CONTROL_ID 0x310    // タイヤ
#define SLAVE2_DISHES_ARM_ID 0x710       // お皿
#define SLAVE3_ZEUS_ARM_STS3215_ID 0x410 // 万能手腕
#define SLAVE4_SQUID_ARM_ID 0x110        // いかさん
#define SLAVE5_MARKER_ARM_ID 0x210       // マーカー
#define SLAVE6_ZEUS_ARM_SHOUKOU_ID 0x410 // 万能手腕昇降
#define SLAVEX_BUTSUDAN_LED_ID 0x115     // 仏壇

#define ZEUS_START_LOCATE 131776

#define ZEUS_ARM_SOREPPOITOKO 175000

#define ZEUS_TENNISBALL_GET 32790
#define ZEUS_TENNISBALL_SET 813860

#define ZEUS_BASEBALL_GET -8200
#define ZEUS_BASEBALL_SET 813860

#define ZEUS_SHUTTLE_GET 0
#define ZEUS_SHUTTLE_SET 813860

uint16_t ID = 0;
int8_t DATA[8] = {0};

TaskHandle_t motor_control_handle = NULL;
twai_handle_t receive_handle = NULL;

HXC_TWAI CAN_BUS(D0, D1, CAN_RATE_1MBIT);

M3508_P19 MOTOR(&CAN_BUS, 1);

void receive(void *pvParameters)
{
  while (1)
  {
    twai_message_t receiveframe;
    if (twai_receive_v2(receive_handle, &receiveframe, pdMS_TO_TICKS(10)) == ESP_OK)
    {
      if (receiveframe.identifier == SLAVE6_ZEUS_ARM_SHOUKOU_ID)
      {

        ID = receiveframe.identifier;
        DATA[0] = receiveframe.data[0]; // 命令の識別子
        DATA[1] = receiveframe.data[1]; // 命令の数値
        DATA[2] = receiveframe.data[2]; // ゴミ
        DATA[3] = receiveframe.data[3]; // ゴミ
        DATA[4] = receiveframe.data[4]; // ゴミ
        DATA[5] = receiveframe.data[5]; // ゴミ
        DATA[6] = receiveframe.data[6]; // ゴミ
        DATA[7] = receiveframe.data[7]; // ゴミ
        Serial.printf("ID:%x, DATA[0]:%d, DATA[1]:%d, DATA[2]:%d, DATA[3]:%d, DATA[4]:%d, DATA[5]:%d, DATA[6]:%d, DATA[7]:%d\n", ID, DATA[0], DATA[1], DATA[2], DATA[3], DATA[4], DATA[5], DATA[6], DATA[7]);
        xTaskNotifyGive(motor_control_handle);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void motor_control(void *pvParameters)
{

  while (1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    switch (DATA[0])
    {

    case 1:

      switch (DATA[1])
      {

      case 1: // 万能手腕手動操作
        switch (DATA[2])
        {
        case 1:
          MOTOR.set_speed(30);
          break;

        case 2:
          MOTOR.set_speed(-30);
          break;

        case 3:
          MOTOR.set_speed(0);
          break;

        default:
          break;
        }
        break;

      case 2: // 万能手腕拾得
        switch (DATA[2])
        {
        case 1:
          MOTOR.set_location(ZEUS_TENNISBALL_GET);
          break;

        case 2:
          MOTOR.set_location(ZEUS_BASEBALL_GET);
          break;

        case 3:
          MOTOR.set_location(ZEUS_SHUTTLE_GET);
          break;

        default:
          break;
        }
        break;

      case 3: // 万能手腕遺失
        switch (DATA[2])
        {
        case 1:
          MOTOR.set_location(ZEUS_TENNISBALL_SET);
          break;

        case 2:
          MOTOR.set_location(ZEUS_BASEBALL_SET);
          break;

        case 3:
          MOTOR.set_location(ZEUS_SHUTTLE_SET);
          break;

        default:
          break;
        }
        break;

      case 4:
        switch (DATA[2])
        {
        case 1:
          MOTOR.set_location(ZEUS_ARM_SOREPPOITOKO);
          break;

        default:
          break;
        }
        break;

      default:
        break;
      }
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup()
{

  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(200));

  if (CAN_BUS.setup() == ESP_OK)
  {
    Serial.println("CANセットアップ完了");
  }
  else
    Serial.println("失敗！！");

  MOTOR.setup();

  MOTOR.set_location_pid(3.5, 0.0, 0.1, 0.0, 1000.0); // kp, ki, 死区, 最高速度

  vTaskDelay(pdMS_TO_TICKS(1000));

  MOTOR.set_location(ZEUS_START_LOCATE);

  /***********************************CAN関連********************************************/
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT_V2(1, (gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t ret = twai_driver_install_v2(&g_config, &t_config, &f_config, &receive_handle);
  if (ret == ESP_OK)
    Serial.println("インストール完了");
  else
    Serial.println("インストール失敗");
  ret = twai_start_v2(receive_handle);
  if (ret == ESP_OK)
    Serial.println("CANスタート完了");
  else
    Serial.println("CANスタート失敗");
  /**************************************************************************************/

  xTaskCreateUniversal(
      receive,
      "receive",
      8192,
      NULL,
      2,
      NULL,
      PRO_CPU_NUM);

  xTaskCreateUniversal(
      motor_control,
      "motor_control",
      4096,
      NULL,
      1,
      &motor_control_handle,
      PRO_CPU_NUM);
}

void loop()
{
  Serial.printf("NOW_LOCATE:%d\n", MOTOR.get_location());
}