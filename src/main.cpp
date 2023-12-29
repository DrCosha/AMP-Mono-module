/*

************************************************************************
*             Firmware для управления моноблоком Hi-END Amp
*                         (с) 2023-2024, by Dr@Cosha
*                               ver 4.0
*                         hardware ver.6.0
************************************************************************

Модуль управления моноблока получает команды от следующих устройств и блоков:

- кнопка Power на лицевой панели;
- кнопка Input на лицевой панели;
- кнопка VU light на лицевой панели;
- вход TruggerIN - сигнал +12V на включение/выключение от прочих устройств
- шина OneWireBUS - подключение к общей шине управления для приема/передачи команд по OneWire
- сервер MQTT - подключение к топикам сервера для возврата статуса и получения команд

Моноблок усилителя раблотает в комплексе с аналогичными, поэтому общается с ними следующим образом:

-------- OneWire BUS:
Блоки работают на общей шине по принципу равных. Необходимость в синхронизации блоков и как следствие - передачи данных по шине 
возникает только при воздействии с внешних источников. Передача параметров по шине OneWire и их применение не приводит к повторной передаче данных по OneWire.


------------- WiFi:
Подключение к WiFi проводится с заранее установленными параметрами.  Если подключение успешно, устанавливается соединение с MQTT сервером. Если WiFi доступен (режим WF_CLIENT), а 
MQTT соединения нет, то периодически пытаемся установить это соединение.  Если WiFi соединение с точкой доступа не возможно, поднимаем свою точку доступа используя номер канала WiFi 
"по умолчанию". И имя сети "AMP_xxxxxxxx"  где xxxxxxx - MAC адрес ESP32.  Без пароля по умолчанию.  В этой сети поднимается web сервер со страничкой настроек на 
адресе gateway-я. После установки настроек, пытаемся опять соеденится с роутером в режиме клиента и поднять MQTT соединение.

----------- MQTT:
Для получения команд от MQTT устройства объединяются в группу, которая читает один топик команд.  Свой статус они сообщают в заранее настроенный топик для каждого устройства.
В качестве параметра команды может быть поднят флаг синхронизации по OneWire

Команды в топике команд:

  {clear_config:true}  - очистить Flash память и загрузится с конфигурацией по умолчанию
  {reset}              - перезагрузить контроллер управления усилителем 
  {report}             - сформировать отчет о текущем состоянии в топик REPORT  

*/

#include <Arduino.h>
#include <WiFi.h>
#include <EEPROM.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

#include "GyverButton.h"
#include <OneWireBus.h>

#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

// устанавливаем режим отладки
#define DEBUG_LEVEL_PORT                        // устанавливаем режим отладки через порт

// установка скорости передачи данных по шине OneWireBus
#define OneWireCycle OWB_DEFAULT_SPEED          // базовый цикл шины - скорость по умолчанию

// определение пинов подключения переферии
#define BTTN_POWER_PIN 26                       // пин подключения кнопки POWER 
#define BTTN_SELECTOR_PIN 4                     // пин подключения кнопки SELECTOR
#define BTTN_UV_LIGHT_PIN 33                    // пин подключения кнопки UV_LIGHT 

#define LED_POWER_GREEN_PIN 14                  // пин LED power GREEN 
#define LED_POWER_RED_PIN 2                     // пин LED power RED 
#define LED_POWER_BLUE_PIN 15                   // пин LED power BLUE

#define LED_SELECTOR_RCA_PIN 17                 // пин LED для индикации RCA
#define LED_SELECTOR_XLR_PIN 13                 // пин LED для индикации XLR

#define RELAY_POWER_PIN 22                      // пин управления реле POWER
#define RELAY_SELECTOR_PIN 23                   // пин управления реле SELECTOR

#define MUTE_VU_PIN 25                          // пин управления выключением VU индикатора

#define TRIGGER_IN_PIN 21                       // пин подключения входа TRIGGER IN
#define TRIGGER_OUT_PIN 18                      // пин управления выходом TRIGGER OUT

#define ONE_WIRE_PIN 32                         // пин шины 1Wire

#define AMBIENT_SENSOR_PIN 36                   // пин подключения датчика освещенности

#define LED_UV_LIGHT_PIN 12                     // пин подключения PWM управления для UV подсветки

// определяем константы для задержек
#define C_VU_DELAY 5000                         // задержка включения стрелочек после подачи питания (5 сек)
#define C_AMBIENT_CHECK_DELAY 5000              // через сколько миллисекунд проверять датчик внешней освещенности (5 сек)
#define C_BRIGHTNESS_SET_DELAY 100              // задержка для плавного изменения яркости от текущей до заданной (0,1 сек)
#define C_WIFI_CONNECT_TIMEOUT 30000            // задержка для установления WiFi соединения (30 сек)
#define C_MQTT_CONNECT_TIMEOUT 10000            // задержка для установления MQTT соединения (10 сек)
#define C_WIFI_AP_WAIT 180000                   // таймуат поднятой AP без соединения с клиентами (после этого опять пытаемся подключится как клиент) (180 сек)
#define C_WIFI_CYCLE_WAIT 10000                 // таймуат цикла переустановки соединения с WiFi (10 сек)

// определяем константы для уровней сигнала
#define C_MAX_PWM_VALUE 1000                    // максимальное значение яркости при регулировании подсветки
#define C_MIN_PWM_VALUE 60                      // минимальное значение яркости при регулировании подсветки
#define C_MAX_SENSOR_VALUE 4000                 // максимальное значение возвращаемое сенсором освещенности
#define C_MIN_SENSOR_VALUE 0                    // минимальное значение возвращаемое сенсором освещенности

#define INP_XLR true                            // константа выбор входа XLR
#define INP_RCA false                           // константа выбор входа RCA

// начальные параметры устройства для подключения к WiFi и MQTT

#define P_WIFI_SSID "iot_ls1"                            // SSID нашей локальной сети  
#define P_WIFI_PASSWORD "vvssoft40"                     // пароль к нашей локальной сети
#define P_MQTT_USER "mqtt_user"                         // имя пользователя для подключения к MQTT серверу
#define P_MQTT_PWD "vvssoft40"                          // пароль для подключения к MQTT серверу
#define P_MQTT_HOST IPAddress(192, 168, 10, 100)        // адрес нашего Mosquito MQTT сервера
#define P_MQTT_PORT 1883                                // порт нашего Mosquito MQTT сервера

#define P_LWT_TOPIC   "diy/hires_amp_01/LWT"            // топик публикации доступности устройства
#define P_SET_TOPIC   "diy/hires_amp_01/set"            // топик публикации команд для устройства
#define P_STATE_TOPIC "diy/hires_amp_01/state"          // топик публикации состояния устройства

// определяем константы для параметров и команд JSON формата в MQTT

#define js_RESET "reset"                        // команда "мягкой" перезагрузки устройства с закрытием соединений
#define js_CLR_CONFIG "clear_config"            // команда очистки текущей конфигурации в EPROM и перезагрузки устройства
#define js_REPORT "report"                      // команда принудительного формирования отчета в топик

// тип описывающий режим работы подсветки индикатора  
enum VU_mode_t : uint8_t {
  VL_AUTO,          // автоматический режим подсветки по датчику
  VL_LOW,           // минимальный уровень подсветки
  VL_HIGH,          // максимальный уровень подсветки
  VL_OFF            // подсветка выключена
};                                    

// тип описывающий режим работы WIFI - работа с самим WiFi и MQTT 
enum WiFi_mode_t : uint8_t {
  WF_UNKNOWN,       // режим работы WiFi еще не определен
  WF_OFF,           // при пакете ошибок при работе с WIFI - выключение WIFI и выключение режима ESP.NOW  
  WF_AP,            // поднятие собственной точки доступа со страничкой настройки   
  WF_CLIENT,        // включение WIFI в режиме клиента 
  WF_MQTT,          // соединение с MQTT сервером
  WF_IN_WORK        // все хорошо, работаем
};  

// структура данных хранимых в EEPROM
struct GlobalParams {
// параметры режима работы усилителя
  bool            inp_selector;                 // выбранный режим входа ( INP_RCA / INP_XLR )
  VU_mode_t       vu_light_mode;                // режим работы подсветки индикатора  
  bool            sync_trigger_in_out;          // режим прямой проброски триггерного входа на выход
  bool            sync_by_owb;                  // синхронизация по OneWireBus
// параметры подключения к MQTT и WiFi  
  char            wifi_ssid[20];                // строка SSID сети WiFi
  char            wifi_pwd[20];                 // пароль к WiFi сети
  char            mqtt_usr[20];                 // имя пользователя MQTT сервера
  char            mqtt_pwd[20];                 // пароль к MQTT серверу
  uint8_t         mqtt_host[4];                 // адрес сервера MQTT
  uint16_t        mqtt_port;                    // порт подключения к MQTT серверу
// параметры очередей MQTT
  char            command_topic[80];            // топик получения команд
  char            report_topic[80];             // топик отправки состояния 
  char            lwt_topic[80];                // топик доступности устройства
// контрольная сумма блока для EEPROM
  uint16_t        simple_crc16;                 // контрольная сумма блока параметров
};

// структура данных передаваемых по OneWireBUS
struct SyncBUSParams {
// параметры режима работы усилителя
  bool            power_on;                     // усилитель включен
  bool            inp_selector;                 // выбранный режим входа ( RCA-false / XLR-true )
  uint8_t         vu_light_mode;                // режим работы подсветки индикатора - определяется типом VU_mode_t
  uint16_t        vu_light_value;               // значение яркости подсветки в виде числа
// контрольная сумма блока данных               
  uint16_t        simple_crc16;                 // контрольная сумма блока параметров
};

// глобальные константы
const int c_PWM_Channel = 0;                    // общий канал управления PWM
const int c_Freq = 1000;                        // частота управления PWM
const int c_Resolution = 16;                    // разрешение PWM
const int def_WiFi_Channel = 13;                // канал WiFi по умолчанию 

// объявляем текущие переменные состояния
bool s_AmpPowerOn = false;                      // режим включения усилителя
bool s_TriggerIn = false;                       // режим включения через триггерный вход
bool s_EnableEEPROM = false;                    // глобальная переменная разрешения работы с EEPROM
bool s_VU_Enable = false;                       // разрешение работы стрелочного указателя
WiFi_mode_t s_CurrentWIFIMode = WF_UNKNOWN;     // текущий режим работы WiFI

// временные моменты наступления контрольных событий в миллисекундах 
uint32_t tm_PowerOn = 0;                        // когда включено питание
uint32_t tm_LastAmbientCheck = 0;               // последний момент проверки внешнего освещения
uint32_t tm_LastBrightnessSet = 0;              // последний момент установки яркости индикатора
uint32_t tm_LastReportToMQTT = 0;               // время последнего отчета в MQTT

// общие флаги программы - команды и изменения 
bool f_HasMQTTCommand = false;                  // флаг получения MQTT команды 
bool f_HasChanges = false;                      // флаг наличия изменений

// переменные управления яркостью индикатора
uint16_t  v_CurrAmbient = 0;                    // усредненная величина текущей яркости окружающенго освещения
uint16_t  v_GoalBrightness = 0;                 // величина рассчитанной яркости подсветки VU индикатора по текущей освещенности
uint16_t  v_CurrBrightness = 0;                 // величина текущей установленной яркости подсветки VU индикатора

// создаем буфера и структуры данных
GlobalParams  curConfig;                        // набор параметров управляющих текущей конфигурацией
SyncBUSParams  OutBuffer, InBuffer;             // буфер передаваемых и принимаемых параметров

// создаем и инициализируем объекты - кнопки
GButton bttn_power(BTTN_POWER_PIN, HIGH_PULL, NORM_OPEN);                                 // инициализируем кнопку управления питанием
GButton bttn_input(BTTN_SELECTOR_PIN, HIGH_PULL, NORM_OPEN);                              // инициализируем кнопку выбора входа
GButton bttn_light(BTTN_UV_LIGHT_PIN, HIGH_PULL, NORM_OPEN);                              // инициализируем кнопку переключения освещением

// объявляем объект MQTT клиент 
AsyncMqttClient   mqttClient;                  // MQTT клиент

// создаем объект - JSON документ для приема/передачи данных через MQTT
StaticJsonDocument<255> doc;                   // создаем json документ с буфером в 255 байт 

// =============================== общие процедуры и функции ==================================

uint16_t GetCrc16Simple( uint8_t * data, uint16_t len ) {
// ------------------- процедура упрощенного расчета CRC16 для блока данных -------------------   
  uint8_t lo;
  union // представляем crc как слово и как верхний и нижний байт
  {
    uint16_t value;
    struct { uint8_t lo, hi; } bytes;
  } crc;
 
  crc.value = 0xFFFF;  // начальное значение для расчета
  while ( len-- )
    {
        lo = crc.bytes.lo;
        crc.bytes.lo = crc.bytes.hi;
        crc.bytes.hi = lo ^ *data++;   
        uint8_t mask = 1;
        if ( crc.bytes.hi & mask ) crc.value ^= 0x0240;
        if ( crc.bytes.hi & ( mask << 1 ) ) crc.value ^= 0x0480;
        if ( crc.bytes.hi & ( mask << 2 ) ) crc.bytes.hi ^= 0x09;
        if ( crc.bytes.hi & ( mask << 3 ) ) crc.bytes.hi ^= 0x12;
        if ( crc.bytes.hi & ( mask << 4 ) ) crc.bytes.hi ^= 0x24;
        if ( crc.bytes.hi & ( mask << 5 ) ) crc.bytes.hi ^= 0x48;
        if ( crc.bytes.hi & ( mask << 6 ) ) crc.bytes.hi ^= 0x90;
        if ( crc.bytes.hi & ( mask << 7 ) ) crc.value ^= 0x2001;
    }
     return crc.value;
}

static void Halt(const char *msg) {
//  процедура аварийного останова контроллера при критических ошибках в ходе выполнения
#ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
  Serial.println(msg);        // выводим сообщение
  Serial.flush();
#endif
  esp_deep_sleep_start();     // останавливаем контроллер
}

void SetConfigByDefault() {
// ------------------- устанавливаем значения в блоке конфигурации по умолчанию --------------  
      memset((void*)&curConfig,0,sizeof(curConfig));    // обнуляем область памяти и заполняем ее значениями по умолчанию
      curConfig.inp_selector = INP_XLR;                                              // по умолчанию XLR
      curConfig.vu_light_mode = VL_AUTO;                                             // значение auto     
      memcpy(curConfig.wifi_ssid,P_WIFI_SSID,sizeof(P_WIFI_SSID));                   // сохраняем имя WiFi сети по умолчанию      
      memcpy(curConfig.wifi_pwd,P_WIFI_PASSWORD,sizeof(P_WIFI_PASSWORD));            // сохраняем пароль к WiFi сети по умолчанию
      memcpy(curConfig.mqtt_usr,P_MQTT_USER,sizeof(P_MQTT_USER));                    // сохраняем имя пользователя MQTT сервера по умолчанию
      memcpy(curConfig.mqtt_pwd,P_MQTT_PWD,sizeof(P_MQTT_PWD));                      // сохраняем пароль к MQTT серверу по умолчанию
      curConfig.mqtt_host[0] = P_MQTT_HOST[0];                                       // сохраняем адрес сервера MQTT по умолчанию
      curConfig.mqtt_host[1] = P_MQTT_HOST[1];
      curConfig.mqtt_host[2] = P_MQTT_HOST[2];
      curConfig.mqtt_host[3] = P_MQTT_HOST[3];
      memcpy(curConfig.command_topic,P_SET_TOPIC,sizeof(P_SET_TOPIC));               // сохраняем наименование командного топика
      memcpy(curConfig.report_topic,P_STATE_TOPIC,sizeof(P_STATE_TOPIC));            // сохраняем наименование топика состояния
      memcpy(curConfig.lwt_topic,P_LWT_TOPIC,sizeof(P_LWT_TOPIC));                   // сохраняем наименование топика доступности
      curConfig.mqtt_port = P_MQTT_PORT;
      // расчитываем контрольную сумму блока данных
      curConfig.simple_crc16 = GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4);     // считаем CRC16      
}

bool ReadEEPROMConfig (){
// ------------------- чтение конфигурации из EEPROM в буфер curConfig --------------------
  uint16_t tmp_CRC;

  EEPROM.get(0,curConfig);                                                 // читаем блок конфигурации из EEPROM
  tmp_CRC = GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4);     // считаем CRC16
  return (tmp_CRC==curConfig.simple_crc16);                                // возвращаем сошлась ли CRC16 

}

void CheckAndUpdateEEPROM() {
// проверяем конфигурацию и в случае необходимости - записываем новую 
  GlobalParams  oldConfig;        // это старый сохраненный конфиг
  uint16_t cur_CRC, old_CRC;      // это переменные для рассчета CRC сохраненного и текущего конфига

  if (!s_EnableEEPROM) return;    // если работаем без EEPROM - выходим сразу
  // иначе читаем старый конфиг в oldConfig, сравниваем его с текущим curConfig и если нужно, записываем в EEPROM
  EEPROM.get(0,oldConfig);                                                 // читаем блок конфигурации из EEPROM
  old_CRC = GetCrc16Simple((uint8_t*)&oldConfig, sizeof(oldConfig)-4);     // считаем CRC16 для него
  cur_CRC = GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4);     // считаем CRC16 для текущих параметров
  curConfig.simple_crc16 = cur_CRC;                                        // сохраняем CRC16 текущего блока параметров
  
#ifdef DEBUG_LEVEL_PORT                
  if (cur_CRC != old_CRC) { //  если конфигурации отличаются - сохраняем новую
      EEPROM.put(0,curConfig);     
      if (EEPROM.commit()) Serial.println("EPPROM update successful.");
         else Serial.println("Error for update configuration in EPPROM.");
    } else { 
        Serial.println(" EPPROM is not need to updated."); 
      }
#else
  if (cur_CRC != old_CRC) { //  если конфигурации отличаются - сохраняем новую
      EEPROM.put(0,curConfig);     
      EEPROM.commit();
  }    
#endif      

}

// ========================= вспомогательные задачи времени выполнения ===================================

void wifiTask(void *pvParam) {
  // задача установления и поддержания WiFi соединения  
  uint32_t StartWiFiCycle = 0;                                       // стартовый момент цикла в обработчике WiFi
  uint32_t StartMQTTCycle = 0;                                       // стартовый момент цикла подключения к MQTT
  char AP_SSID[32] = "HiAmp_";                                       // переменная в которой строим строку с именем WiFi AP 
  WiFi.macAddress().toCharArray(&AP_SSID[6],sizeof(AP_SSID)-6);      // строим имя сети для AP на основе MAC адреса ESP32
  WiFi.hostname(AP_SSID);
  s_CurrentWIFIMode = WF_UNKNOWN;
  while (true) {    
    switch (s_CurrentWIFIMode) {
    case WF_UNKNOWN:
      // включаем индикацию соединения по WiFi
      digitalWrite(LED_POWER_BLUE_PIN,HIGH);
      // начальное подключение WiFi - сброс всех соединений и новый цикл их поднятия 
      mqttClient.disconnect(true);                                  // принудительно отсоединяемся от MQTT       
      WiFi.persistent(false);
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();
      #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
      Serial.print("Try to connect WiFi: ");
      Serial.print(curConfig.wifi_ssid);
      Serial.print("[");
      #endif
      StartWiFiCycle = millis();
      // изначально пытаемся подключится в качестве клиента к существующей сети с грантами из конфигурации
      WiFi.begin(curConfig.wifi_ssid,curConfig.wifi_pwd);
      while ((! WiFi.isConnected()) && (millis() - StartWiFiCycle < C_WIFI_CONNECT_TIMEOUT)) { // ожидаем соединения с необходимой WiFi сеткой
        vTaskDelay(pdMS_TO_TICKS(1000)); // рисуем точку каждую секунду
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
        Serial.print(".");
        #endif
      } 
      #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
      Serial.println("]");
      #endif    
      // цикл окончен, проверяем соеденились или нет
      if (WiFi.isConnected())  s_CurrentWIFIMode = WF_CLIENT;       // если да - мы соеденились в режиме клиента
        else s_CurrentWIFIMode = WF_AP;                             // соеденится как клиент не смогли - нужно поднимать точку доступа
      #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
      if (WiFi.isConnected()) {
          Serial.print("Connected with IP: ");
          Serial.println(WiFi.localIP());
        }
        else Serial.println("Fail.");
      #endif    
      break;
    case WF_OFF:   
      // WiFi принудительно выключен при получении ошибок при работе с WIFI 
      mqttClient.disconnect(true);                                  // принудительно отсоединяемся от MQTT 
      WiFi.persistent(false);
      WiFi.disconnect();
      vTaskDelay(pdMS_TO_TICKS(C_WIFI_CYCLE_WAIT));                 // ждем цикл перед еще одной проверкой
      s_CurrentWIFIMode = WF_UNKNOWN;                               // уходим на пересоединение с WIFI
      break;    
    case WF_CLIENT:
      // включение WIFI в режиме клиента 
      #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
      Serial.print("Try to connect MQTT: ");
      Serial.print(curConfig.mqtt_host[0]); Serial.print("."); Serial.print(curConfig.mqtt_host[1]); Serial.print("."); Serial.print(curConfig.mqtt_host[2]); Serial.print("."); Serial.println(curConfig.mqtt_host[3]); 
      #endif     
      s_CurrentWIFIMode = WF_MQTT;
      break;    
    case WF_MQTT:
      // соединение с MQTT сервером
      StartMQTTCycle = millis();
      // пытаемся подключится к MQTT серверу в качестве клиента
      mqttClient.connect();
      while ((!mqttClient.connected()) && (millis()-StartMQTTCycle < C_MQTT_CONNECT_TIMEOUT)) { // ожидаем соединения с MQTT сервером
        vTaskDelay(pdMS_TO_TICKS(500)); 
      } 
      // цикл окончен, проверяем есть ли соединение с MQTT
      if (mqttClient.connected()) { s_CurrentWIFIMode = WF_IN_WORK;  }     // если да - то пеерходим в режим нормальной работы
        else {
          s_CurrentWIFIMode = WF_AP;                                  // иначе - уходим в режим AP проблема с окружением или конфигурацией - нужно поднимать точку доступа
          #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
          Serial.println("MQTT connection timeout..."); 
          #endif    
        }  
      break;    
    case WF_IN_WORK:  // состояние в котором ничего не делаем, так как все нужные соединения установлены
      if (!mqttClient.connected() or !WiFi.isConnected()) {              // проверяем, что соединения всё еще есть. Если они пропали, делаем таймаут на цикл C_WIFI_CYCLE_WAIT и переустанавливаем соединение
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
        if (!mqttClient.connected()) Serial.println("MQTT conneсtion lost.");
        if (!WiFi.isConnected()) Serial.println("WiFi conneсtion lost.");
        #endif
        vTaskDelay(pdMS_TO_TICKS(C_WIFI_CYCLE_WAIT)); 
        s_CurrentWIFIMode = WF_OFF;                                      // уходим на пересоединение с WIFI
      }
      break;    
    case WF_AP:
      // поднятие собственной точки доступа со страничкой настройки      
      WiFi.persistent(false);
      WiFi.mode(WIFI_AP);
      WiFi.disconnect();      
      #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
      Serial.print("Create AP with SSID: ");
      Serial.println(AP_SSID);
      #endif    
      if (WiFi.softAP(AP_SSID,NULL,def_WiFi_Channel)) {     // собственно создаем точку доступа на дефолтном канале
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
        Serial.print("AP created with IP: ");
        Serial.println(WiFi.softAPIP());
        #endif 
        // если точку доступа удалось поднять, то даем ей работать до тех пор пока не кончился таймаут C_WIFI_AP_WAIT, или есть коннекты к точке доступа  
        while ((WiFi.softAPgetStationNum()>0) && (millis() - StartWiFiCycle < C_WIFI_AP_WAIT)) {
          
          // TODO:           
          /*
               а вот здесь нужно поднять сервер со страничкой настроек для изменения конфигурации модуля ESP32
               после подтверждения выхода с этой странички, автоматически уходим на переподключение в режиме клиента

          */

          vTaskDelay(pdMS_TO_TICKS(1000));      // отдаем управление и ждем секунду перед следующей проверкой
        }
      }
      s_CurrentWIFIMode = WF_UNKNOWN;           // и опять начинаем все с начала и переключаемся в режим попытки установления связи с роутером
      break; 
    }
    // запоминаем точку конца цикла
    StartWiFiCycle = millis();
    vTaskDelay(1/portTICK_PERIOD_MS); 
  }  
}

void oneWireTask(void *pvParam) {
// задача по поддержанию работы через шину OneWire BUS
  while (true) {

    vTaskDelay(1/portTICK_PERIOD_MS); 

  }
}

// ================================== основные задачи времени выполнения =================================

void getCommandTask (void *pvParam) {
// задача получения команды от датчика, таймера, MQTT, OneWire, кнопок
  while (true) {

    //--- обработка событий получения MQTT команд в приложение 
    if ( f_HasMQTTCommand ) {                                         // превращаем события MQTT в команды для отработки приложением    

      if (doc.containsKey(js_CLR_CONFIG) and doc[js_CLR_CONFIG])  {   // послана команда обнуления конфигурации

        // TODO: Serial.println("Обнуляем конфиг и перегружаемся");

      }

      if (doc.containsKey(js_RESET) and doc[js_RESET])  {   // послана команда перезагрузки

        // TODO: Serial.println("Перегружаемся");

      }

      if (doc.containsKey(js_REPORT) and doc[js_REPORT])  {   // послана команда принудительного отчета

        // TODO: Serial.println("Готовим отчёт в топик");

      }

/*
  const char* cur_state = doc[js_CLR_CONFIG];
  if (StrCheck(cur_state,"ON")) cmdPowerON();
    else { if (StrCheck(cur_state,"OFF")) cmdPowerOFF();    
            else PassCount = 0; 
    }       
  // обработка тега BRIGHTNESS  
  if (strstr(payload,C_BRIGHTNESS) != NULL ) curr_brightness = doc[C_BRIGHTNESS];
  // обработка тега COLOR_TEMP
  if (strstr(payload,C_COLOR_TEMP) != NULL ) color_temp = map(doc[C_COLOR_TEMP],153, 500, 0, 100);
*/

      f_HasMQTTCommand = false;                                       // сбрасываем флаг наличия изменений через MQTT 
    }
    //--- опрос кнопок - получение команд от лицевой панели 
    bttn_power.tick();                                                // опрашиваем кнопку POWER
    bttn_input.tick();                                                // опрашиваем кнопку INPUT  
    bttn_light.tick();                                                // опрашиваем кнопку LIGHT  
    // однократное нажатие на кнопку POWER
    if (bttn_power.isSingle()) {        

     // TODO: включаем/выключаем усилитель

    }
    // однократное нажатие на кнопку выбора входа
    if (bttn_input.isSingle()) {        
      if (s_AmpPowerOn) {                                             // все действия, если усь включен
        curConfig.inp_selector = !curConfig.inp_selector;
        f_HasChanges = true;                                          // взводим флаг применения изменений
      }
    }
    // однократное нажатие на кнопку переключения освещения
    if (bttn_light.isSingle()) {        

     // TODO: по кругу переключаем режим освещения

    }
    // одновременное нажатие и удержание кнопок Power и Input 
    if (bttn_power.isHold() and bttn_input.isHold()) {        


     // TODO: сбрасываем параметры записанные во Flash память

    }
    // отдаем управление ядру FreeRT OS
    vTaskDelay(1/portTICK_PERIOD_MS); 
  }
}

void applayChangesTask (void *pvParam) {
// применяем изменений, и если нужно сохранение состояния в FLASH памяти
  while (true) {
    
    if (s_CurrentWIFIMode == WF_IN_WORK ) {  
      // выключаем светодиод установления связи по WiFi и MQTT при нормально установленном соединении
      digitalWrite(LED_POWER_BLUE_PIN,LOW); 
    }
    if (s_CurrentWIFIMode == WF_AP ) {  
      // включаем мигающий режим светодиода индицирующего работу по WiFi в режиме точки доступа

      // TODO: мигаем

    };
    vTaskDelay(1/portTICK_PERIOD_MS); 
  }
}

void sendCommandTask (void *pvParam) {
// шлем команду по OneWireBUS
  while (true) {
    
    vTaskDelay(1/portTICK_PERIOD_MS);     

  }
}

void reportTask (void *pvParam) {
// репортим о текущем состоянии в MQTT и если отладка то и в Serial
  while (true) {
    
    vTaskDelay(1/portTICK_PERIOD_MS);         

  }
}


// -------------------------- в этом фрагменте описываем call-back функции MQTT клиента --------------------------------------------
void onMqttConnect(bool sessionPresent) {   

  #ifdef DEBUG_LEVEL_PORT                                    
    Serial.println("Connected to MQTT.");  //  "Подключились по MQTT."
    Serial.print("Session present: ");  //  "Текущая сессия: "
    Serial.println(sessionPresent);
  #endif                

  // далее подписываем ESP32 на набор необходимых для управления топиков:
  uint16_t packetIdSub = mqttClient.subscribe(curConfig.command_topic, 0);  // подписываем ESP32 на топик SET_TOPIC

  #ifdef DEBUG_LEVEL_PORT                                      
    Serial.print("Subscribing at QoS 0, packetId: ");
    Serial.println(packetIdSub);
    Serial.print("Topic: ");
    Serial.println(curConfig.command_topic);
  #endif                  

  // сразу публикуем событие о своей активности
  mqttClient.publish(curConfig.lwt_topic, 0, true, "online");           // публикуем в топик LWT_TOPIC событие о своей жизнеспособности

  #ifdef DEBUG_LEVEL_PORT                                      
    Serial.print("Publishing LWT state in [");
    Serial.print(curConfig.lwt_topic); 
    Serial.println("]. QoS 0. "); 
  #endif                    
  
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  #ifdef DEBUG_LEVEL_PORT                                                                           
    Serial.println("Disconnected from MQTT.");                      // если отключились от MQTT
  #endif         
  s_CurrentWIFIMode = WF_OFF;                                       // переходим в режим полного реконнекта по WiFi
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  #ifdef DEBUG_LEVEL_PORT   
    Serial.println("Subscribe acknowledged.");          // подписка подтверждена
    Serial.print("  packetId: ");                       // 
    Serial.println(packetId);                           // выводим ID пакета
    Serial.print("  qos: ");                            // 
    Serial.println(qos);                                // выводим значение QoS
  #endif         
}

void onMqttUnsubscribe(uint16_t packetId) {
  #ifdef DEBUG_LEVEL_PORT     
    Serial.println("Unsubscribe acknowledged.");        // отписка подтверждена
    Serial.print("  packetId: ");                       //
    Serial.println(packetId);                           // выводим ID пакета
  #endif                     
}

void onMqttPublish(uint16_t packetId) {
  #ifdef DEBUG_LEVEL_PORT     
    Serial.println("Publish acknowledged.");            // публикация подтверждена
    Serial.print("  packetId: ");                       //
    Serial.println(packetId);                           // выводим ID пакета
  #endif                     
}


// в этой функции обрабатываем события получения данных в управляющем топике SET_TOPIC
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;

  #ifdef DEBUG_LEVEL_PORT         
    Serial.print("Get message: [");
  #endif                         

  for (int i = 0; i < len; i++) {                       // преобразуем полученные в сообщении данные в строку
    #ifdef DEBUG_LEVEL_PORT         
      Serial.print((char)payload[i]);
    #endif                         
    messageTemp += (char)payload[i];
  }
  messageTemp[len] = '\0';  

  #ifdef DEBUG_LEVEL_PORT         
    Serial.println("]");
  #endif                         

  // проверяем, что мы получили MQTT сообщение в командном топике
  if (strcmp(topic, curConfig.command_topic) == 0) {
    // разбираем MQTT сообщение и подготавливаем буфер с изменениями для формирования команд    
    deserializeJson(doc, messageTemp);                  // десерилизуем сообщение и взводим признак готовности к обработке
    // для коротких сообщений без ключей дополняем DOC объект
    if (strstr(payload,js_CLR_CONFIG) != NULL ) doc[js_CLR_CONFIG] = true;
    if (strstr(payload,js_RESET) != NULL ) doc[js_RESET] = true;
    if (strstr(payload,js_REPORT) != NULL ) doc[js_REPORT] = true;
    f_HasMQTTCommand = true;                            // взводим флаг получения команды по MQTT
  }
 
  #ifdef DEBUG_LEVEL_PORT         
    Serial.println("Publish received.");                //  выводим на консоль данные из топика
    Serial.print("  topic: ");                          //  "  топик: "
    Serial.println(topic);                              // название топика 
    Serial.print("  message: ");                        //  "  сообщение: "
    Serial.println(messageTemp);                        //  сообщение 
  #endif                         

}

// =================================== инициализация контроллера и программных модулей ======================================
// начальная инициализация программы - выполняется при подаче дежурного питания.
// дальнейшее включение усилителя - уже в рамках работающей программы
void setup() {

#ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
  // инициализируем порт отладки 
  Serial.begin(115200);
  Serial.println();
#endif

 // инициализация входов и выходов  
  pinMode(LED_POWER_GREEN_PIN, OUTPUT);       // инициализируем pin светодиода POWER GREEN 
  pinMode(LED_POWER_RED_PIN, OUTPUT);         // инициализируем pin светодиода POWER RED 
  pinMode(LED_POWER_BLUE_PIN, OUTPUT);        // инициализируем pin светодиода POWER BLUE 

  pinMode(LED_SELECTOR_RCA_PIN, OUTPUT);      // инициализируем pin светодиода SELECTOR RCA
  pinMode(LED_SELECTOR_XLR_PIN, OUTPUT);      // инициализируем pin светодиода SELECTOR XLR

  pinMode(RELAY_POWER_PIN, OUTPUT);           // инициализируем выход на реле POWER
  digitalWrite(RELAY_POWER_PIN, LOW);         // выключаем усилитель по умолчанию

  pinMode(RELAY_SELECTOR_PIN, OUTPUT);        // инициализируем выход на реле SELECTOR
  digitalWrite(RELAY_SELECTOR_PIN, LOW);      // подключаем вход RCA
  
  pinMode(MUTE_VU_PIN, OUTPUT);               // инициализируем выход выключения VU индикатора
  digitalWrite(MUTE_VU_PIN, HIGH);            // и сразу его выключаем при инициализации

  pinMode(TRIGGER_IN_PIN, INPUT);             // инициализируем вход TRIGGER_IN
  
  pinMode(TRIGGER_OUT_PIN, OUTPUT);           // инициализируем выход TRIGGER_OUT
  digitalWrite(TRIGGER_OUT_PIN, LOW);         // выключаем внешние устройства через выход TRIGGER

  // инициализируем датчик освещенности и расчитываем заначения уровня яркости посветки
  adcAttachPin(AMBIENT_SENSOR_PIN);                 // подключаем пин сенсора к ADC
  v_CurrAmbient = analogRead(AMBIENT_SENSOR_PIN);   // начитываем начальное значение для уровня освещенности
  tm_LastAmbientCheck = millis();                   // запоминаем момент последнего считывания датчика освещенности
  tm_LastBrightnessSet = millis();                  // запоминаем момент последней установки яркости
  v_GoalBrightness = 0;                             // рассчитываем целевой уровень яркости подсветки
  v_CurrBrightness = v_GoalBrightness;              // инициализируем значение текущей яркости

  // создаем и инициализируем PWM канал, отключаем его, назначаем  VU выход в канал PWM
  ledcSetup(c_PWM_Channel, c_Freq, c_Resolution);  
  //в начале выключаем генерацию PWM для канала
  ledcWrite(c_PWM_Channel, 0);
  //включаем VU подсветку в PWM канал
  ledcAttachPin(LED_UV_LIGHT_PIN, c_PWM_Channel);   
  
  // включаем индикацию
  digitalWrite(LED_POWER_GREEN_PIN, HIGH);
  digitalWrite(LED_POWER_RED_PIN, HIGH);
  digitalWrite(LED_POWER_BLUE_PIN, HIGH);  
  digitalWrite(LED_SELECTOR_RCA_PIN, HIGH);
  digitalWrite(LED_SELECTOR_XLR_PIN, HIGH);  

  // задержка для контроля индикации 
  vTaskDelay(pdMS_TO_TICKS(500));

  // гасим всю индикацию
  digitalWrite(LED_POWER_GREEN_PIN, LOW);
  digitalWrite(LED_POWER_RED_PIN, LOW);
  digitalWrite(LED_POWER_BLUE_PIN, LOW);    
  digitalWrite(LED_SELECTOR_RCA_PIN, LOW);
  digitalWrite(LED_SELECTOR_XLR_PIN, LOW);  

  // инициализируем шину 1Wire BUS

  if (!OneWireBus.InitializeBus(ONE_WIRE_PIN,BROADCAST_ADDR,OneWireCycle,sizeof(SyncBUSParams))){   // инициализируем шину OneWire
     Serial.println("Ошибка инициализации шины ONEWIREBUS!");
  }
  
  // инициализируем кнопку POWER
  bttn_power.setDebounce(50);        // настройка антидребезга (по умолчанию 80 мс)  
  bttn_power.setTimeout(500);        // настройка таймаута на удержание (по умолчанию 500 мс)
  bttn_power.setClickTimeout(200);   // настройка таймаута между кликами (по умолчанию 300 мс)
  
  // инициализируем кнопку SELECTOR
  bttn_input.setDebounce(50);        // настройка антидребезга (по умолчанию 80 мс)  
  bttn_input.setTimeout(500);        // настройка таймаута на удержание (по умолчанию 500 мс)
  bttn_input.setClickTimeout(200);   // настройка таймаута между кликами (по умолчанию 300 мс)

  // инициализируем кнопку VU_LIGHT
  bttn_light.setDebounce(50);        // настройка антидребезга (по умолчанию 80 мс)  
  bttn_light.setTimeout(500);        // настройка таймаута на удержание (по умолчанию 500 мс)
  bttn_light.setClickTimeout(200);   // настройка таймаута между кликами (по умолчанию 300 мс)

  // инициализируем блок конфигурации значениями по умолчанию
  SetConfigByDefault();

  // инициализация работы с EEPROM
  s_EnableEEPROM = EEPROM.begin(sizeof(curConfig));   // инициализируем работу с EEPROM 

#ifdef DEBUG_LEVEL_PORT    

  if (s_EnableEEPROM) {  // если инициализация успешна - то:   
    if (ReadEEPROMConfig()) { // читаем конфигурацию из EEPROM и проверяем контрольную сумму для блока данных
      // контрольная сумма блока данных верна - используем его в качестве конфигурации
      Serial.println("EEPROM config is valid. Applay config.");
      }
    else { // контрольная сумма данных не сошлась - считаем блок испорченным, инициализируем и перезаписываем его
      Serial.println("EEPROM config broken - try to rewrite.");    
      SetConfigByDefault();
      EEPROM.put(0,curConfig);     
      if (EEPROM.commit()) Serial.println("EPPROM update success.");
        else Serial.println("Error in write to EPPROM.");
    }
    Serial.println("");
    Serial.println("--- Инициализация блока управления прошла со следующими параметрами: ---");
    Serial.print("  Input RCA-1|XLR-0 : "); Serial.println(curConfig.inp_selector ? "XLR" : "RCA");
    Serial.print("  VU light mode: "); Serial.println(curConfig.vu_light_mode);    
    Serial.print("  Ext trigger sync: "); Serial.println(curConfig.sync_trigger_in_out);    
    Serial.print("  WiFi mode: "); Serial.println(s_CurrentWIFIMode);
    Serial.print("  WiFi SSid: "); Serial.println(curConfig.wifi_ssid);    
    Serial.print("  WiFi pwd: "); Serial.println(curConfig.wifi_pwd);
    Serial.print("  MQTT usr: "); Serial.println(curConfig.mqtt_usr);
    Serial.print("  MQTT pwd: "); Serial.println(curConfig.mqtt_pwd);
    Serial.print("  MQTT addr: "); Serial.print(curConfig.mqtt_host[0]); Serial.print("."); Serial.print(curConfig.mqtt_host[1]); Serial.print("."); Serial.print(curConfig.mqtt_host[2]); Serial.print("."); Serial.println(curConfig.mqtt_host[3]); 
    Serial.print("  MQTT port: "); Serial.println(curConfig.mqtt_port);
    Serial.println("---");    
    Serial.print("  COMMAND topic: "); Serial.println(curConfig.command_topic);
    Serial.print("  REPORT topic: "); Serial.println(curConfig.report_topic);
    Serial.print("  LWT topic: "); Serial.println(curConfig.lwt_topic);
    Serial.println("---");    
    Serial.print("  CRC by read: "); Serial.println(curConfig.simple_crc16,HEX);
    Serial.print("  CRC by calc: "); Serial.println(GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4),HEX);  
    Serial.println("---");
    Serial.println("");  
  }
  else { Serial.println("Warning! Блок работает без сохранения конфигурации !!!"); 
  }
  
#else

  if (s_EnableEEPROM) {  // если инициализация успешна - то:   
    if (!ReadEEPROMConfig()) { // читаем конфигурацию из EEPROM и проверяем контрольную сумму для блока данных
      // если контрольная сумма данных не сошлась - считаем блок испорченным, инициализируем и перезаписываем его
      SetConfigByDefault();
      EEPROM.put(0,curConfig);     
      EEPROM.commit();
    }
  }
  
#endif  

  // настраиваем MQTT клиента
  mqttClient.setCredentials(curConfig.mqtt_usr,curConfig.mqtt_pwd);
  mqttClient.setServer(curConfig.mqtt_host, curConfig.mqtt_port);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);

  // создаем отдельные параллельные задачи, выполняющие группы функций  
  // стартуем основные задачи
  if (xTaskCreate(getCommandTask, "command", 4096, NULL, 1, NULL) != pdPASS) { 
    // все плохо, задачу не создали
    Halt("Error: Get command task not created!");
  }
  if (xTaskCreate(applayChangesTask, "applay", 4096, NULL, 1, NULL) != pdPASS) { 
    // все плохо, задачу не создали
    Halt("Error: Applay changes task not created!");
  }
  if (xTaskCreate(sendCommandTask, "send", 4096, NULL, 1, NULL) != pdPASS) { 
    // все плохо, задачу не создали
    Halt("Error: Send command task not created!");
  }
  if (xTaskCreate(reportTask, "report", 4096, NULL, 1, NULL) != pdPASS) { 
    // все плохо, задачу не создали
    Halt("Error: Report task not created!");
  }  
  
  // стартуем коммуникационные задачи
  if (xTaskCreate(oneWireTask, "onewire", 4096, NULL, 1, NULL) != pdPASS) { 
    // все плохо, задачу не создали
    Halt("Error: OneWire communication task not created!");
  }  
  if (xTaskCreate(wifiTask, "wifi", 4096*2, NULL, 1, NULL) != pdPASS) { 
    // все плохо, задачу не создали
    Halt("Error: WiFi communication task not created!");
  }  

}


// не используемый основной цикл
void loop() {

  vTaskDelete(NULL);   // удаляем не нужную задачу loop()  

}