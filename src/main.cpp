/*

************************************************************************
*             Firmware для управления моноблоком Hi-END Amp
*                         (с) 2022, by Dr@Cosha
*                               ver 3.0
************************************************************************

Модуль управления моноблока получает команды от следующих устройств и блоков:

- кнопка Power на лицевой панели;
- кнопка Input на лицевой панели;
- кнопка VU light на лицевой панели;
- вход TruggerIN - сигнал +12V на включение/выключение от прочих устройств
- шина 1WireBUS - подключение к общей шине управления для приема/передачи команд по 1Wire
- шина ESP.NOW -  подключение к общему каналу управления для приема/передачи команд по WiFi
- сервер MQTT - подключение к топикам сервера для возврата статуса и получения команд


Моноблок усилителя раблотает в комплексе с аналогичными, поэтому он общается с ними по следующим протоколам:
  в порядке убывания приоритета:

  - 1WireBUS
  - ESP.NOW
  - WiFi
  
-------- 1Wire BUS:
В начале устройство считает себя ALONE устройством и при старте генерит случайный таймаут в течение которого ждет пакет на шине. Если его нет, посылает служебный ping-пакет, для 
проверки наличие слушателей на шине 1Wire. Такой пакет содержит свой MAC адрес и время жизни пакета. На него каждое устройство должно однократно ответить своим ping-пакетом, если 
время жизни больше 1. Ответные пакеты содержат данные: (время жизни-1) и свои MAC адреса. Если пакет получен, по MAC адресу пославшего определяем свой статус на шине, если MAC больше 
моего, то я slave - он master. После посылки всеми устройствами по одному пакету, на шине останется только один master.  Если на шине за определенный таймаут не получен ни один пакет 
с адресом старше чем у текущего устройства, то устанавливаем своё состояние в master. Если за определенное время на шине не было ни одного входящего пакета, то считаем себя ALONE 
устройством и начинаем повторный процесс поиска слушателей.  Если слушатели не нашлись, то повторяем эту процедуру через заданные промежутки времени. Возможно в этот момент это устройство 
уже осталось единственным устройством на шине.

------------- WiFi:
Параллельно идет подключение к WiFi с установленными параметрами.  Если подключение успешно, устанавливается соединение с MQTT сервером. Если WiFi доступен (режим WF_CLIENT), а 
MQTT соединения нет, то периодически пытаемся установить это соединение.  Если WiFi соединение с точкой доступа не возможно, поднимаем свою точку доступа используя номер канала WiFi 
"по умолчанию". И имя сети "AMP_xxxxxxxx"  где xxxxxxx - MAC адрес ESP32.  Без пароля по умолчанию.  В этой сети поднимается web сервер со страничкой настроек на 
адресе gateway-я. После установки настроек, пытаемся опять соеденится с роутером в режиме клиента и поднять MQTT соединение.

--------- ESP.NOW:
Параллельно с WiFi, если устройство в статусе ALONE, то оно пытается открыть канал обмена по ESP.NOW.  Для этого пытаемся по протоколу ESP.NOW  установить соединение с точкой доступа 
(сервером ESP.NOW) на дефолтном канале WiFi. Если сервер не найден, поднимаем точку ESP.NOW в режиме сервера, соединений с собой. Если соединений нет, по наступлению таймаута, сбрасываем 
режим сервера и опять пытаемся соеденится в качестве клиента. После установления соединения, объявляем себя MASTER-ом, клиентское устройство переключается в режим SLAVE (фиксируем свой 
статус в установленном соединении). Если за определенное время не было ни одного пакета, считаем, что соединение пропало, опять входим в цикл установки соединения. Если устройство перестает 
быть ALONE, то протокол ESP.NOW запрещается. Если сервер ESP.NOW получает пакет от клиента, то он перепаковывает его от своего имени и пересылает остальным клиентам. При этом он сам 
использует данные пакета для своих настроек. 

----------- MQTT:
Получение команд от MQTT и отдача статуса в MQTT возможна только для устройства, в состояниях:
   - WF_MQTT и статусе OW_MASTER для 1Wire;
   - WF_MQTT и EN_SERVER|EN_CLIENT|EN_MASTER для ESP.NOW.

*/

#include <Arduino.h>
#include <WiFi.h>
#include <EEPROM.h>
#include "GyverButton.h"
#include "GyverTransfer.h"
#include "GBus.h"
#include "GyverBus.h"

// устанавливаем режим отладки
#define DEBUG_LEVEL_PORT                        // устанавливаем режим отладки через порт

// установка режима потоковой передачи для GyverTransfer
#define GT_STREAM_MODE                          // STREAM_MODE нужен для работы GBUS

// установка адреса модуля для обмена по 1Wire (1-255) должен быть уникален
#define MODULE_ADDRESS 10                       // NB!!!  должен быть уникален для каждого модуля на шине

// определение пинов подключения переферии
#define BTTN_POWER_PIN 26                       // пин подключения кнопки POWER  (на плате разведен 5-й, но он обрезан и стоит перемычка на 26-й)
#define BTTN_SELECTOR_PIN 4                     // пин подключения кнопки SELECTOR
#define BTTN_VU_LIGHT_PIN 33                    // пин подключения кнопки VU_LIGHT (на плате разведен 16-й, но он обрезан и стоит перемычка на 33-й)

#define LED_POWER_GREEN_PIN 15                  // пин LED power GREEN  
#define LED_POWER_RED_PIN 8                     // пин LED power RED   (на плате разведен 08-й, но с он обрезан и стоит перемычка на 14-й)
#define LED_POWER_BLUE_PIN 2                    // пин LED power BLUE

#define LED_SELECTOR_RCA_PIN 17                 // пин LED для индикации RCA
#define LED_SELECTOR_XLR_PIN 13                 // пин LED для индикации XLR

#define RELAY_POWER_PIN 22                      // пин управления реле POWER
#define RELAY_SELECTOR_PIN 23                   // пин управления реле SELECTOR
#define MUTE_VU_PIN 25                          // пин управления выключением VU индикатора

#define TRIGGER_IN_PIN 21                       // пин подключения входа TRIGGER IN
#define TRIGGER_OUT_PIN 18                      // пин управления выходом TRIGGER OUT

#define ONE_WIRE_PIN 32                         // пин шины 1Wire
#define AMBIENT_SENSOR_PIN 36                   // пин подключения датчика освещенности

#define LED_VU_LIGHT_PIN 12                     // пин подключения PWM управления для подсветки стрелочного индикатора

// определяем константы для задержек
#define C_VU_DELAY 5000                         // задержка включения стрелочек после подачи питания
#define C_AMBIENT_CHECK_DELAY 5000              // через сколько миллисекунд проверять датчик внешней освещенности
#define C_BRIGHTNESS_SET_DELAY 100              // задержка для плавного изменения яркости от текущей до заданной
#define C_WIFI_CONNECT_TIMEOUT 30000            // задержка для установления WiFi соедтинения
#define C_WIFI_AP_WAIT 60000                    // таймуат поднятой AP без соединения с клиентами (после этого опять пытаемся подключится как клиент)
#define C_WIFI_CYCLE_WAIT 180000                // таймуат цикла переустановки соединения с WiFi

// определяем константы для уровней сигнала
#define C_MAX_PWM_VALUE 1000                    // максимальное значение яркости при регулировании подсветки
#define C_MIN_PWM_VALUE 60                      // минимальное значение яркости при регулировании подсветки
#define C_MAX_SENSOR_VALUE 4000                 // максимальное значение возвращаемое сенсором освещенности
#define C_MIN_SENSOR_VALUE 0                    // минимальное значение возвращаемое сенсором освещенности

#define INP_XLR true                            // константа выбор входа XLR
#define INP_RCA false                           // константа выбор входа RCA

// параметры подключения к WiFi и MQTT по умолчанию
const char c_WIFI_SSID[10] = "iot_ls";          // строка SSID сети WiFi
const char c_WIFI_PASSWORD[10] = "vvssoft40";   // пароль к WiFi сети
const char c_MQTT_USER[10] = "mqtt_user";       // имя пользователя MQTT сервера
const char c_MQTT_PWD[10] = "vvssoft40";        // пароль к MQTT серверу
const byte c_MQTT_HOST[4] = {192,168,10,100};   // адрес сервера MQTT
const int  c_MQTT_PORT = 1883;                  // порт подключения к MQTT серверу

// тип описывающий режим работы подсветки индикатора  
enum VU_mode_t : uint8_t {
  VL_AUTO,          // автоматический режим подсветки по датчику
  VL_LOW,           // минимальный уровень подсветки
  VL_HIGH,          // максимальный уровень подсветки
  VL_OFF,           // подсветка выключена
  VL_EXT            // автоматический режим по внешним командам
};                                    

// тип описывающий режим работы WIFI - по протоколу ESP.NOW
enum ESP_NOW_mode_t : uint8_t {
  EN_DISABLE,       // ESP.NOW запрещен
  EN_CLIENT,        // подключение к точке ESP.NOW на канале роутера или дефолтном WIFI канале
  EN_SERVER,        // поднятие своей точки подключения к ESP.NOW на канале роутера или дефолтном WIFI канале
  EN_SLAVE,         // установлено соединение и устройство в режиме подчиненного клиента
  EN_MASTER         // установлено соединение с клиентами и сервер стал мастером
};

// тип описывающий режим работы WIFI - работа с самим WiFi и MQTT 
enum WiFi_mode_t : uint8_t {
  WF_UNKNOWN,       // режим работы WiFi еще не определен
  WF_OFF,           // при пакете ошибок при работе с WIFI - выключение WIFI и выключение режима ESP.NOW  
  WF_AP,            // поднятие собственной точки доступа со страничкой настройки   
  WF_CLIENT,        // включение WIFI в режиме клиента 
  WF_MQTT           // соединение с MQTT сервером
};  

// тип описывающий режим работы по 1WireBUS 
enum OWB_mode_t : uint8_t {
  OW_OFF,           // запрет работы шины 1Wire BUS  
  OW_ALONE,         // пока на шине считаем себя единственными
  OW_SLAVE,         // мы на шине SLAVE
  OW_MASTER         // мы на шине MASTER
};

// структура данных хранимых в EEPROM
struct GlobalParams {
// параметры режима работы усилителя
  bool            inp_selector;                 // выбранный режим входа ( INP_RCA / INP_XLR )
  VU_mode_t       vu_light_mode;                // режим работы подсветки индикатора  
// параметры подключения к MQTT и WiFi  
  char            wifi_ssid[20];                // строка SSID сети WiFi
  char            wifi_pwd[20];                 // пароль к WiFi сети
  char            mqtt_usr[20];                 // имя пользователя MQTT сервера
  char            mqtt_pwd[20];                 // пароль к MQTT серверу
  uint8_t         mqtt_host[4];                 // адрес сервера MQTT
  uint16_t        mqtt_port;                    // порт подключения к MQTT серверу
// контрольная сумма блока для EEPROM
  uint16_t        simple_crc16;                 // контрольная сумма блока параметров
};

// структура данных передаваемых по 1WireBUS и ESP.NOW
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
const int c_freq = 1000;                        // частота управления PWM
const int c_resolution = 16;                    // разрешение PWM
const int def_WiFi_channel = 13;                // канал WiFi по умолчанию 

// объявляем текущие переменные состояния
bool s_AmpPowerOn = false;                      // режим включения усилителя
bool s_TriggerIn = false;                       // режим включения через триггерный вход
bool s_EnableEEPROM = false;                    // глобальная переменная разрешения работы с EEPROM
bool s_VU_Enable = false;                       // разрешение работы стрелочного указателя
OWB_mode_t s_CurrentOWBMode = OW_OFF;           // текущий режим работы шины 1Wire
WiFi_mode_t s_CurrentWIFIMode = WF_CLIENT;      // текущий режим работы WiFI
ESP_NOW_mode_t s_CurrentESPNMode = EN_CLIENT;   // текущий режим работы ESP.NOW


// временные моменты наступления контрольных событий в миллисекундах 
uint32_t tm_PowerOn = 0;                        // когда включено питание
uint32_t tm_LastAmbientCheck = 0;               // последний момент проверки внешнего освещения
uint32_t tm_LastBrightnessSet = 0;              // последний момент установки яркости индикатора

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
GButton bttn_light(BTTN_VU_LIGHT_PIN, HIGH_PULL, NORM_OPEN);                              // инициализируем кнопку переключения освещением

// создаем канал обмена данными между блоками
GyverTransfer<ONE_WIRE_PIN, GT_TRX> trans;                                                // создаем канал передачи на пине ONE_WIRE_PIN в режиме прием-передача
GBUS OneWireBus(&trans, MODULE_ADDRESS, sizeof(SyncBUSParams));                           // адрес MODULE_ADDRESS, буфер размером со структуру SyncBUSParams

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
      memcpy(curConfig.wifi_ssid,c_WIFI_SSID,sizeof(c_WIFI_SSID));                   // сохраняем имя WiFi сети по умолчанию      
      memcpy(curConfig.wifi_pwd,c_WIFI_PASSWORD,sizeof(c_WIFI_PASSWORD));            // сохраняем пароль к WiFi сети по умолчанию
      memcpy(curConfig.mqtt_usr,c_MQTT_USER,sizeof(c_MQTT_USER));                    // сохраняем имя пользователя MQTT сервера по умолчанию
      memcpy(curConfig.mqtt_pwd,c_MQTT_PWD,sizeof(c_MQTT_PWD));                      // сохраняем пароль к MQTT серверу по умолчанию
      memcpy(curConfig.mqtt_host,c_MQTT_HOST,sizeof(c_MQTT_HOST));                   // сохраняем адрес сервера MQTT по умолчанию
      curConfig.mqtt_port = c_MQTT_PORT;
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
  uint32_t StartWiFiCycle = 0;                // стартовый момент цикла в обработчике WiFi
  char AP_SSID[21] = "AP_";                   // переменная в которой строим строку с именем WiFi AP 

  // перед входом в цикл вычисляем общие константы и устанавливаем правильный статус контроллера
  WiFi.macAddress().toCharArray(&AP_SSID[4],sizeof(AP_SSID)-4);     // строим имя сети для AP на основе MAC адреса ESP32
  s_CurrentWIFIMode = WF_UNKNOWN;

  while (true) {    
    switch (s_CurrentWIFIMode) {
    case WF_UNKNOWN:
      // начальное подключение WiFi - сброс всех соединений и новый цикл их поднятия 
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
        vTaskDelay(pdMS_TO_TICKS(1000)); 
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
      if (WiFi.isConnected()) Serial.println("Connected.");
        else Serial.println("Fail.");
#endif    
      break;
    case WF_OFF:   
      // WiFi принудительно выключен  при получении ошибок при работе с WIFI - + выключение режима ESP.NOW  
      WiFi.persistent(false);
      WiFi.disconnect();
      s_CurrentESPNMode = EN_DISABLE;                               // запрещаем ESP.NOW      
      vTaskDelay(pdMS_TO_TICKS(C_WIFI_CYCLE_WAIT));                 // ждем цикл перед еще одной проверкой
      break;    
    case WF_CLIENT:
      // включение WIFI в режиме клиента 

      // TODO:  мы в WiFi сетке, и теперь нужно начать процесс подключения к MQTT серверу

      break;    
    case  WF_MQTT:
      // соединение с MQTT сервером
      
      // TODO:  соединение с MQTT сервером

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
      if (WiFi.softAP(AP_SSID,NULL,def_WiFi_channel)) {     // собственно создаем точку доступа на дефолтном канале
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
  }  
}

void mqttTask(void *pvParam) {
// задача установления и поддержания соединения с MQTT сервером поверх WiFi коннекта
  while (true) {



  }
}

void oneWireTask(void *pvParam) {
// задача по поддержанию работы через шину 1Wire BUS
  while (true) {



  }
}

void espNowTask(void *pvParam) {
// задача по поддержанию работы через протокол ESP.NOW
  while (true) {



  }
}


// ================================== основные задачи времени выполнения =================================

void getCommandTask (void *pvParam) {
// задача получения команды от датчика, таймера, MQTT, ESP.NOW, 1Wire, кнопок
  while (true) {


  }
}

void applayChangesTask (void *pvParam) {
// применяем изменений, и если нужно сохранение состояния в FLASH памяти
  while (true) {
    

  }
}

void sendCommandTask (void *pvParam) {
// шлем команду по 1WireBUS и ESP.NOW
  while (true) {
    

  }
}

void reportTask (void *pvParam) {
// репортим о текущем состоянии в MQTT и если отладка то и в Serial
  while (true) {
    

  }
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
  ledcSetup(c_PWM_Channel, c_freq, c_resolution);  
  //в начале выключаем генерацию PWM для канала
  ledcWrite(c_PWM_Channel, 0);
  //включаем VU подсветку в PWM канал
  ledcAttachPin(LED_VU_LIGHT_PIN, c_PWM_Channel);   
  
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

  // Enable1Wire = OneWireBus.InitializeBus(ONE_WIRE_PIN, 500, sizeof(InBuffer), sizeof(OutBuffer));      // пин шины 1Wire, таймаут передачи, размеры входного и выходного буферов
  
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
    Serial.print("  1Wire mode: "); Serial.println(s_CurrentOWBMode);
    Serial.print("  WiFi mode: "); Serial.println(s_CurrentWIFIMode);
    Serial.print("  ESP.NOW mode: "); Serial.println(s_CurrentESPNMode);    
    Serial.print("  WiFi SSid: "); Serial.println(curConfig.wifi_ssid);    
    Serial.print("  WiFi pwd: "); Serial.println(curConfig.wifi_pwd);
    Serial.print("  MQTT usr: "); Serial.println(curConfig.mqtt_usr);
    Serial.print("  MQTT pwd: "); Serial.println(curConfig.mqtt_pwd);
    Serial.print("  MQTT addr: "); Serial.print(curConfig.mqtt_host[0]); Serial.print("."); Serial.print(curConfig.mqtt_host[1]); Serial.print("."); Serial.print(curConfig.mqtt_host[2]); Serial.print("."); Serial.println(curConfig.mqtt_host[3]); 
    Serial.print("  MQTT port: "); Serial.println(curConfig.mqtt_port);
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
  if (xTaskCreate(oneWireTask, "1wire", 4096, NULL, 1, NULL) != pdPASS) { 
    // все плохо, задачу не создали
    Halt("Error: 1Wire communication task not created!");
  }  
  if (xTaskCreate(wifiTask, "wifi", 4096, NULL, 1, NULL) != pdPASS) { 
    // все плохо, задачу не создали
    Halt("Error: WiFi communication task not created!");
  }  
  if (xTaskCreate(mqttTask, "mqtt", 4096, NULL, 1, NULL) != pdPASS) { 
    // все плохо, задачу не создали
    Halt("Error: MQTT communication task not created!");
  }  
  if (xTaskCreate(espNowTask, "esp_now", 4096, NULL, 1, NULL) != pdPASS) { 
    // все плохо, задачу не создали
    Halt("Error: ESP.NOW communication task not created!");
  }  

}


// не используемый основной цикл
void loop() {

  vTaskDelete(NULL);   // удаляем не нужную задачу loop()  
}