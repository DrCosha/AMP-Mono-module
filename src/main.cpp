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

  {"clear_config"}                          - очистить Flash память и загрузится с конфигурацией по умолчанию
  {"reset"}                                 - перезагрузить контроллер управления усилителем 
  {"report"}                                - сформировать отчет о текущем состоянии в топик REPORT  
  {"power":"on"|"off"}                      - включить/выключить модуль
  {"input":"rca"|"xlr"}                     - выбор входа для усилителя  
  {"trigger_enable":"on"|"off"}             - разрешить/запретить работу триггеров
  {"owb_sync":"on"|"off"}                   - разрешение синхронизации по OneWireBUS
  {"bypass":"on"|"off"}                     - разрешение прямой проброски триггерного сигнала с входа на выход
  {"vu_light": "off"|"on_low"|"on_middle"|"on_high"|"auto"}  - режим работы подсветки VU индикатора 
  {"light_manual": [<value1>,<value2>,<value3>]}             - значения PWM для подстройки яркости освещения в режимах "on_low","on_middle","on_high"
  {"light_auto": [<min_value>,<max_value>]}                  - значения PWM для подстройки границ изменения автоматической яркости
  {"ambient": [<min_value>,<max_value>]}                     - подстройка границ входного сигнала сенсора освещенности

*/

#include <Arduino.h>
#include <WiFi.h>
#include <EEPROM.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_mac.h"
}

#include "GyverButton.h"
#include <OneWireBus.h>

#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

// устанавливаем режим отладки
#define DEBUG_LEVEL_PORT                          // устанавливаем режим отладки через порт

// установка скорости передачи данных по шине OneWireBus
#define OneWireCycle 640                          // базовый цикл шины - скорость по умолчанию

// определение пинов подключения переферии
#define BTTN_POWER_PIN 26                         // пин подключения кнопки POWER 
#define BTTN_SELECTOR_PIN 4                       // пин подключения кнопки SELECTOR
#define BTTN_UV_LIGHT_PIN 33                      // пин подключения кнопки UV_LIGHT 

#define LED_POWER_GREEN_PIN 14                    // пин LED power GREEN 
#define LED_POWER_RED_PIN 2                       // пин LED power RED 
#define LED_POWER_BLUE_PIN 15                     // пин LED power BLUE

#define LED_SELECTOR_RCA_PIN 17                   // пин LED для индикации RCA
#define LED_SELECTOR_XLR_PIN 13                   // пин LED для индикации XLR

#define RELAY_POWER_PIN 22                        // пин управления реле POWER
#define RELAY_SELECTOR_PIN 23                     // пин управления реле SELECTOR

#define MUTE_VU_PIN 25                            // пин управления выключением VU индикатора

#define TRIGGER_IN_PIN 21                         // пин подключения входа TRIGGER IN
#define TRIGGER_OUT_PIN 18                        // пин управления выходом TRIGGER OUT

#define ONE_WIRE_PIN 32                           // пин шины 1Wire

#define AMBIENT_SENSOR_PIN 36                     // пин подключения датчика освещенности

#define LED_UV_LIGHT_PIN 12                       // пин подключения PWM управления для UV подсветки

// определяем константы для задержек
#define C_VU_DELAY_ON 3000                        // подача сигнала отключения стрелок после подачи питания (3 сек)
#define C_VU_DELAY_OFF 5000                       // подача сигнала отключения стрелок после выключения питания (5 сек)
#define C_AMBIENT_CHECK_DELAY 5000                // через сколько миллисекунд проверять датчик внешней освещенности (5 сек)
#define C_BRIGHTNESS_SET_DELAY 100                // задержка для плавного изменения яркости от текущей до заданной (0,1 сек)
#define C_WIFI_CONNECT_TIMEOUT 30000              // задержка для установления WiFi соединения (30 сек)
#define C_MQTT_CONNECT_TIMEOUT 10000              // задержка для установления MQTT соединения (10 сек)
#define C_WIFI_AP_WAIT 180000                     // таймуат поднятой AP без соединения с клиентами (после этого опять пытаемся подключится как клиент) (180 сек)
#define C_WIFI_CYCLE_WAIT 10000                   // таймуат цикла переустановки соединения с WiFi (10 сек)
#define C_BLINKER_DELAY 800                       // задержка переключения блинкера
#define C_TRIGGER_ON_DEBOUNCE 200                 // задержка устранения дребезга сигнала TriggerIn при включении - нужна для подавления переходных процессов включения стойки аппаратуры
#define C_TRIGGER_OFF_DEBOUNCE 1000               // задержка устранения дребезга сигнала TriggerIn при выключении - нужна для подавления переходных процессов включения стойки аппаратуры
#define C_OWB_TRY_DELAY 100                       // задержка при ошибке отправки пакета по OneWireBUS
#define C_OWB_READ_BUS_DELAY 50                   // задержка при чтении пакетов по OneWireBUS
#define C_MAX_LOST_ON_OWB 1000                    // максимальное количество пакетов подряд после которых выключается попытка синхронизации устройств 

// задержки в формировании MQTT отчета
#define C_MQTT_REPORT_DELAY_ON    30000           // 30 секунд для включенного блока
#define C_MQTT_REPORT_DELAY_OFF  1800000          // 30 минут для выключенного 

// определяем константы для уровней сигнала
#define DEF_MAX_AUTO_PWM 2000                     // максимальное значение яркости при авто регулировании подсветки
#define DEF_MIN_AUTO_PWM 60                       // минимальное значение яркости при авто регулировании подсветки
#define DEF_MIN_MANUAL_PWM 70                     // минимальное значение PWM для ручного регулирования яркости подсветки
#define DEF_MID_MANUAL_PWM 100                    // среднее значение PWM для ручного регулирования яркости подсветки
#define DEF_MAX_MANUAL_PWM 1000                   // максимальное значение PWM для ручного регулирования яркости подсветки
#define DEF_MAX_AMBIENT_VALUE 4000                // максимальное значение возвращаемое сенсором освещенности
#define DEF_MIN_AMBIENT_VALUE 0                   // минимальное значение возвращаемое сенсором освещенности

// определяем имена входов 
#define INP_XLR true                              // константа выбор входа XLR (1)
#define INP_RCA false                             // константа выбор входа RCA (0)

// начальные параметры устройства для подключения к WiFi и MQTT
#define P_WIFI_SSID "iot_ls"                      // SSID нашей локальной сети  
#define P_WIFI_PASSWORD "vvssoft40"               // пароль к нашей локальной сети
#define P_MQTT_USER "mqtt_user"                   // имя пользователя для подключения к MQTT серверу
#define P_MQTT_PWD "vvssoft40"                    // пароль для подключения к MQTT серверу
#define P_MQTT_HOST IPAddress(192, 168, 10, 100)  // адрес нашего Mosquito MQTT сервера
#define P_MQTT_PORT 1883                          // порт нашего Mosquito MQTT сервера
#define C_MAX_FAILED_TRYS 3                       // количество попыток повтора для поднятия AP точки    

// определяем топики для работы устройства по MQTT
#define P_LWT_TOPIC   "diy/hires_amp_01/LWT"      // топик публикации доступности устройства
#define P_SET_TOPIC   "diy/hires_amp_01/set"      // топик публикации команд для устройства
#define P_STATE_TOPIC "diy/hires_amp_01/state"    // топик публикации состояния устройства

// определяем константы для параметров и команд JSON формата в MQTT
// --- имена команд ---
#define jc_RESET          "reset"                 // команда "мягкой" перезагрузки устройства с закрытием соединений
#define jc_CLR_CONFIG     "clear_config"          // команда очистки текущей конфигурации в EPROM и перезагрузки устройства
#define jc_REPORT         "report"                // команда принудительного формирования отчета в топик

// --- имена ключей ---
#define jk_POWER          "power"                 // ключ описания состояния общего включения
#define jk_SELECTOR       "input"                 // ключ описания входа RCA / XLR
#define jk_LIGHT_MODE     "vu_light"              // ключ описания режима подсветки VU индикатора
#define jk_BRIGHTNESS     "vu_brightness"         // ключ описания значения яркости подсветки
#define jk_AMBIENT        "ambient"               // ключ описания значения датчика освещенности
#define jk_TRIGGER_IN     "trigger_in"            // ключ описания значения входа триггера
#define jk_TRG_ENABLE     "trigger_enable"        // ключ описания разрешения работы триггеров
#define jk_TRIGGER_OUT    "trigger_out"           // ключ описания значения выхода триггера
#define jk_TRIGGER_BYPASS "bypass"                // ключ описания режима проброса триггерного входа
#define jk_SYNC_BY_OWB    "owb_sync"              // ключ описания режима синхронизации по OneWireBUS
#define jk_LIGHT_MANUAL_SET  "light_manual"       // ключ описания массива значений PWM для ручной подсветки
#define jk_LIGHT_AUTO_SET    "light_auto"         // ключ описания границ значений PWM для автоматической подсветки
#define jk_AMBIENT_SET       "ambient_sens"       // ключ описания границ значений ambient сенсора

// --- значения ключей и команд ---
#define jv_ONLINE         "online"                // 
#define jv_OFFLINE        "offline"               //
#define jv_ON             "on"                    //
#define jv_OFF            "off"                   //
#define jv_RCA            "rca"                   //
#define jv_XLR            "xlr"                   //
#define jv_AUTO           "auto"                  //
#define jv_MIN            "min"                   //
#define jv_MAX            "max"                   //

// тип описывающий режим работы подсветки индикатора  
#define MAX_VU_MODE 5                             // максимальное количество режимов подсветки  
const char* VU_mode_str[]  = {
  "off",                                          // подсветка выключена
  "on_low",                                       // минимальный уровень подсветки
  "on_middle",                                    // средний уровень подсветки
  "on_high",                                      // максимальный уровень подсветки
  "auto"                                          // автоматический режим подсветки по датчику
};

// тип описывающий режим работы WIFI - работа с самим WiFi и MQTT 
enum WiFi_mode_t : uint8_t {
  WF_UNKNOWN,                                     // режим работы WiFi еще не определен
  WF_OFF,                                         // при пакете ошибок при работе с WIFI - выключение WIFI и выключение режима ESP.NOW  
  WF_AP,                                          // поднятие собственной точки доступа со страничкой настройки   
  WF_CLIENT,                                      // включение WIFI в режиме клиента 
  WF_MQTT,                                        // соединение с MQTT сервером
  WF_IN_WORK                                      // все хорошо, работаем
};  

// значения базовых параметров по умолчанию
#define C_DEF_ENABLE_TRIGGER true                 // по умолчанию, через работа с триггерами разрешена
#define C_DEF_SYNC_TRIGGER_OUT true               // синхронизация входного и выходного триггера включена
#define C_DEF_SYNC_BY_ONEWIREBUS true             // синхронизация модулей через OneWireBUS

// структура данных хранимых в EEPROM
struct GlobalParams {
// параметры режима работы усилителя
  bool            inp_selector;                   // выбранный режим входа ( INP_RCA / INP_XLR )
  uint8_t         vu_light_mode;                  // режим работы подсветки индикатора  
  bool            sync_trigger_in_out;            // режим прямой проброски триггерного входа на выход
  bool            enable_triggers;                // разрешено управление через триггера
  bool            sync_by_owb;                    // синхронизация по OneWireBus
// параметры подключения к MQTT и WiFi  
  char            wifi_ssid[20];                  // строка SSID сети WiFi
  char            wifi_pwd[20];                   // пароль к WiFi сети
  char            mqtt_usr[20];                   // имя пользователя MQTT сервера
  char            mqtt_pwd[20];                   // пароль к MQTT серверу
  uint8_t         mqtt_host[4];                   // адрес сервера MQTT
  uint16_t        mqtt_port;                      // порт подключения к MQTT серверу
// параметры очередей MQTT
  char            command_topic[80];              // топик получения команд
  char            report_topic[80];               // топик отправки состояния 
  char            lwt_topic[80];                  // топик доступности устройства
// корректируемы установки используемые для подсветки индикатора
  uint16_t        _max_auto_pwm,                  // максимальное значение яркости при авто регулировании подсветки
                  _min_auto_pwm,                  // минимальное значение яркости при авто регулировании подсветки
                  _min_manual_pwm,                // минимальное значение PWM для ручного регулирования яркости подсветки
                  _mid_manual_pwm,                // среднее значение PWM для ручного регулирования яркости подсветки
                  _max_manual_pwm,                // максимальное значение PWM для ручного регулирования яркости подсветки
                  _max_ambient_value,             // максимальное значение возвращаемое сенсором освещенности
                  _min_ambient_value;             // минимальное значение возвращаемое сенсором освещенности
// контрольная сумма блока для EEPROM
  uint16_t        simple_crc16;                   // контрольная сумма блока параметров
};

// структура данных передаваемых по OneWireBUS
struct SyncBUSParams {
// параметры режима работы усилителя
  bool            power_on;                       // усилитель включен
  bool            inp_selector;                   // выбранный режим входа ( RCA-false / XLR-true )
  uint8_t         vu_light_mode;                  // режим работы подсветки индикатора - определяется типом VU_mode_t
//  uint16_t        vu_light_value;                 // значение яркости подсветки в виде числа
// контрольная сумма блока данных               
  uint16_t        simple_crc16;                   // контрольная сумма блока параметров
};

// глобальные константы
const int c_PWM_Channel = 0;                    // общий канал управления PWM
const int c_Freq = 1000;                        // частота управления PWM
const int c_Resolution = 16;                    // разрешение PWM
const int def_WiFi_Channel = 13;                // канал WiFi по умолчанию 

// объявляем текущие переменные состояния
bool s_AmpPowerOn = false;                      // режим включения усилителя
bool s_PowerOnByTriggerIn = false;              // режим включения через триггерный вход
bool s_EnableEEPROM = false;                    // глобальная переменная разрешения работы с EEPROM
bool s_VU_Enable = true;                        // разрешение работы стрелочного указателя
WiFi_mode_t s_CurrentWIFIMode = WF_UNKNOWN;     // текущий режим работы WiFI
uint8_t count_GetWiFiConfig = 0;                // счётчик повторов попыток соединения

// временные моменты наступления контрольных событий в миллисекундах 
uint32_t tm_TogglePower = 0;                    // когда переключено питание
uint32_t tm_LastAmbientCheck = 0;               // последний момент проверки внешнего освещения
uint32_t tm_LastBrightnessSet = 0;              // последний момент установки яркости индикатора
uint32_t tm_LastReportToMQTT = 0;               // время последнего отчета в MQTT
uint32_t tm_LastBlinkFire = 0;                  // время последнего переключения флага блинкера
uint32_t tm_TriggerDebounceTime = 0;            // время момента изменения сигнала TriggerIN

uint32_t cur_MQTT_REPORT_DELAY = C_MQTT_REPORT_DELAY_OFF;     // по умолчанию значение равно задержке выключенного блока

// общие флаги программы - команды и изменения 
bool f_HasMQTTCommand = false;                  // флаг получения MQTT команды 
bool f_HasChanges = false;                      // флаг наличия изменений
bool f_HasReportNow = false;                    // флаг формирования отчёта "прямо сейчас"
bool f_Blinker = false;                         // флаг "мигания" - переключается с задержкой C_BLINKER_DELAY
bool f_TriggerDebounce = false;                 // флаг нахождения в режиме устранения дребезга по триггерному входу
bool f_TriggerIn = false;                       // флаг триггерного входа
bool f_HasDataForSync = false;                  // флаг наличия данных для отправки через OneWireBUS 
bool f_HasOWBPacket = false;                    // флаг наличия пакета с командами полученными по OneWireBUS 

// переменные управления яркостью индикатора
uint16_t  v_CurrAmbient = 0;                    // усредненная величина текущей яркости окружающенго освещения
uint16_t  v_GoalBrightness = 0;                 // величина рассчитанной яркости подсветки VU индикатора по текущей освещенности
uint16_t  v_CurrBrightness = 0;                 // величина текущей установленной яркости подсветки VU индикатора
uint8_t   v_OWBErrorCount = 0;                  // количество не удачных попыток передачи через OneWireBUS подряд

// создаем буфера и структуры данных
GlobalParams   curConfig;                       // набор параметров управляющих текущей конфигурацией
SyncBUSParams  OutOWBBuffer, InOWBBuffer;       // буфер передаваемых и принимаемых параметров

// создаем и инициализируем объекты - кнопки
GButton bttn_power(BTTN_POWER_PIN, HIGH_PULL, NORM_OPEN);                                 // инициализируем кнопку управления питанием
GButton bttn_input(BTTN_SELECTOR_PIN, HIGH_PULL, NORM_OPEN);                              // инициализируем кнопку выбора входа
GButton bttn_light(BTTN_UV_LIGHT_PIN, HIGH_PULL, NORM_OPEN);                              // инициализируем кнопку переключения освещением

// объявляем объект MQTT клиент 
AsyncMqttClient   mqttClient;                  // MQTT клиент

// создаем объект - JSON документ для приема/передачи данных через MQTT
StaticJsonDocument<512> InputJSONdoc,          // создаем входящий json документ с буфером в 512 байт 
                        OutputJSONdoc;         // создаем исходящий json документ с буфером в 512 байт 

// создаем мьютексы для синхронизации доступа к данным
SemaphoreHandle_t sem_InputJSONdoc = xSemaphoreCreateBinary();                           // создаем двоичный семафор для доступа к JSON документу 
SemaphoreHandle_t sem_InputOWBPacket = xSemaphoreCreateBinary();                         // создаем двоичный семафор для доступа к входному пакету принятому по OneWireBUS

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
      curConfig.inp_selector = INP_XLR;                                               // по умолчанию XLR
      curConfig.vu_light_mode = MAX_VU_MODE-1;                                        // значение auto     
      curConfig.enable_triggers = C_DEF_ENABLE_TRIGGER;                               // разрешение работы через тригерры
      curConfig.sync_trigger_in_out = C_DEF_SYNC_TRIGGER_OUT;                         // синхронизация Trigger_OUT = Trigger_IN
      curConfig.sync_by_owb = C_DEF_SYNC_BY_ONEWIREBUS;                               // синхронизация модулей через OneWireBUS
      memcpy(curConfig.wifi_ssid,P_WIFI_SSID,sizeof(P_WIFI_SSID));                    // сохраняем имя WiFi сети по умолчанию      
      memcpy(curConfig.wifi_pwd,P_WIFI_PASSWORD,sizeof(P_WIFI_PASSWORD));             // сохраняем пароль к WiFi сети по умолчанию
      memcpy(curConfig.mqtt_usr,P_MQTT_USER,sizeof(P_MQTT_USER));                     // сохраняем имя пользователя MQTT сервера по умолчанию
      memcpy(curConfig.mqtt_pwd,P_MQTT_PWD,sizeof(P_MQTT_PWD));                       // сохраняем пароль к MQTT серверу по умолчанию
      curConfig.mqtt_host[0] = P_MQTT_HOST[0];                                        // сохраняем адрес сервера MQTT по умолчанию
      curConfig.mqtt_host[1] = P_MQTT_HOST[1];
      curConfig.mqtt_host[2] = P_MQTT_HOST[2];
      curConfig.mqtt_host[3] = P_MQTT_HOST[3];
      memcpy(curConfig.command_topic,P_SET_TOPIC,sizeof(P_SET_TOPIC));                // сохраняем наименование командного топика
      memcpy(curConfig.report_topic,P_STATE_TOPIC,sizeof(P_STATE_TOPIC));             // сохраняем наименование топика состояния
      memcpy(curConfig.lwt_topic,P_LWT_TOPIC,sizeof(P_LWT_TOPIC));                    // сохраняем наименование топика доступности
      curConfig.mqtt_port = P_MQTT_PORT;
      // инициализируем значения PWM и границ регулирования
      curConfig._max_auto_pwm = DEF_MAX_AUTO_PWM;                                     // максимальное значение яркости при авто регулировании подсветки
      curConfig._min_auto_pwm = DEF_MIN_AUTO_PWM;                                     // минимальное значение яркости при авто регулировании подсветки
      curConfig._min_manual_pwm = DEF_MIN_MANUAL_PWM;                                 // минимальное значение PWM для ручного регулирования яркости подсветки
      curConfig._mid_manual_pwm = DEF_MID_MANUAL_PWM;                                 // среднее значение PWM для ручного регулирования яркости подсветки
      curConfig._max_manual_pwm = DEF_MAX_MANUAL_PWM;                                 // максимальное значение PWM для ручного регулирования яркости подсветки
      curConfig._max_ambient_value = DEF_MAX_AMBIENT_VALUE;                           // максимальное значение возвращаемое сенсором освещенности
      curConfig._min_ambient_value = DEF_MIN_AMBIENT_VALUE;                           // минимальное значение возвращаемое сенсором освещенности
       // расчитываем контрольную сумму блока данных
      curConfig.simple_crc16 = GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4);     // считаем CRC16      
}

int CalcBrightnessByAmbient(int _Ambient) {  
// ------------------ рассчитываем уровень яркости подсветки по внешнему датчику ----------------
int ret_PWM = curConfig._min_auto_pwm;
  // нормализуем входные значения от датчика подсветки
  if (_Ambient < curConfig._min_ambient_value) _Ambient = curConfig._min_ambient_value;
  if (_Ambient > curConfig._max_ambient_value) _Ambient = curConfig._max_ambient_value;
  v_CurrAmbient = _Ambient;  
  // собственно расчитываем значение PWM от датчика
  ret_PWM = map(_Ambient, curConfig._min_ambient_value, curConfig._max_ambient_value, curConfig._min_auto_pwm, curConfig._max_auto_pwm);
  // значения по умолчанию являются наиболее широкими параметрами, поэтому для безопасности - вгоняем возвращаемое значение в эти пределы
  // т.е. нормализуем выходное значение PWM
  if (ret_PWM > DEF_MAX_AUTO_PWM) ret_PWM = DEF_MAX_AUTO_PWM;
  if (ret_PWM < DEF_MIN_AUTO_PWM) ret_PWM = DEF_MIN_AUTO_PWM;
  return ret_PWM;
}

void SetGoalBrightness() {
// ---- устанавливаем целевую яркость подсветки по текущему состоянию 
const uint16_t oldGoalBrightness = v_GoalBrightness;

  switch (curConfig.vu_light_mode) {   // "off"|"on_low"|"on_middle"|"on_high"|"auto"
    case 0:          // режим выключенной подсветки "off"
      v_GoalBrightness = 0;                                       
      break;
    case 1:          // режим минимальной ручной подсветки "on_low"
      v_GoalBrightness = curConfig._min_manual_pwm;                                       
      break;
    case 2:          // режим минимальной ручной подсветки "on_middle"
      v_GoalBrightness = curConfig._mid_manual_pwm;                                       
      break;
    case 3:          // режим минимальной ручной подсветки "on_low"
      v_GoalBrightness = curConfig._max_manual_pwm;                                       
      break;
    case 4:          // переключились в автоматический режим
      v_GoalBrightness = CalcBrightnessByAmbient(v_CurrAmbient);                        // настраиваем по текущей яркости датчика
      break;
  }
  if (!s_AmpPowerOn) v_GoalBrightness = 0;                                              // если усилитель выключен - целевая яркость = 0 
  f_HasDataForSync = f_HasDataForSync or (v_GoalBrightness != oldGoalBrightness);       // сохраняем или взводим флаг необходимости синхронизации - если значение поменялось     
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
      // начальное подключение WiFi - сброс всех соединений и новый цикл их поднятия 
      count_GetWiFiConfig++;                                        // инкрементируем счётчик попыток 
      mqttClient.disconnect(true);                                  // принудительно отсоединяемся от MQTT       
      WiFi.persistent(false);
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();
      #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
      Serial.printf("Try to connect WiFi: %s[",curConfig.wifi_ssid);
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
      if (count_GetWiFiConfig == C_MAX_FAILED_TRYS) {                // если превышено количество попыток соединения
           mqttClient.disconnect(true);                             // принудительно отсоединяемся от MQTT 
           WiFi.persistent(false);                                  // принудительно отсоединяемся от WiFi 
           WiFi.disconnect();
           count_GetWiFiConfig++;
        }   
      vTaskDelay(pdMS_TO_TICKS(C_WIFI_CYCLE_WAIT));                 // ждем цикл перед еще одной проверкой           
      break;    
    case WF_CLIENT:
      // включение WIFI в режиме клиента 
      #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
      Serial.printf("Try to connect MQTT: %u.%u.%u.%u \n", curConfig.mqtt_host[0], curConfig.mqtt_host[1], curConfig.mqtt_host[2], curConfig.mqtt_host[3]); 
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
      if (mqttClient.connected()) {                                   // если да - то переходим в режим нормальной работы
          s_CurrentWIFIMode = WF_IN_WORK;  
          f_HasReportNow = true;                                      // рапортуем в MQTT текущим состоянием
        }  
        else {
          s_CurrentWIFIMode = WF_AP;                                  // иначе - уходим в режим AP проблема с окружением или конфигурацией - нужно поднимать точку доступа
          #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
          Serial.println("MQTT connection timeout..."); 
          #endif    
        }  
      break;    
    case WF_IN_WORK:  // состояние в котором ничего не делаем, так как все нужные соединения установлены
      count_GetWiFiConfig = 0;                                           // при успешном соединении сбрасываем счётчик попыток повтора 
      if (!mqttClient.connected() or !WiFi.isConnected()) {              // проверяем, что соединения всё еще есть. Если они пропали, делаем таймаут на цикл C_WIFI_CYCLE_WAIT и переустанавливаем соединение
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода
        if (!mqttClient.connected()) Serial.println("MQTT conneсtion lost.");
        if (!WiFi.isConnected()) Serial.println("WiFi conneсtion lost.");
        #endif
        vTaskDelay(pdMS_TO_TICKS(C_WIFI_CYCLE_WAIT)); 
        s_CurrentWIFIMode = WF_UNKNOWN;                                  // уходим на пересоединение с WIFI
      }
      break;    
    case WF_AP:
      // поднятие собственной точки доступа со страничкой настройки      
      WiFi.persistent(false);
      WiFi.mode(WIFI_AP);
      WiFi.disconnect();      
      #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
      Serial.printf("Create AP with SSID: %s\n", AP_SSID);
      #endif    
      if (WiFi.softAP(AP_SSID,NULL,def_WiFi_Channel)) {     // собственно создаем точку доступа на дефолтном канале
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
        Serial.print("AP created with IP: ");
        Serial.println(WiFi.softAPIP());
        #endif 
        StartWiFiCycle = millis();                          // даем отсечку по времени для поднятия точки доступа        
        // если точку доступа удалось поднять, то даем ей работать до тех пор пока не кончился таймаут C_WIFI_AP_WAIT, или есть коннекты к точке доступа       
        while ((WiFi.softAPgetStationNum()>0) or ((millis()-StartWiFiCycle < C_WIFI_AP_WAIT))) {
          
          // TODO:           

          Serial.printf("Подключено: %d\n", WiFi.softAPgetStationNum());

          /*
               а вот здесь нужно поднять сервер со страничкой настроек для изменения конфигурации модуля ESP32
               после подтверждения выхода с этой странички, автоматически уходим на переподключение в режиме клиента

          */

          vTaskDelay(pdMS_TO_TICKS(500));       // отдаем управление и ждем 0.5 секунды перед следующей проверкой
        }
      }
      if (count_GetWiFiConfig == C_MAX_FAILED_TRYS) s_CurrentWIFIMode = WF_OFF;       // если достигнуто количество попыток соединения для получения конфигурации по WIFi - выключаем WIFI
        else s_CurrentWIFIMode = WF_UNKNOWN;                                          // если нет - переключаемся в режим попытки установления связи с роутером
      break; 
    }
    // запоминаем точку конца цикла
    StartWiFiCycle = millis();
    vTaskDelay(1/portTICK_PERIOD_MS); 
  }  
}

void oneWireTask(void *pvParam) {
// задача по поддержанию работы через шину OneWire BUS
uint16_t tmp_RecieveCRC = 0;                                                          // переменная для расчёта CRC полученного пакета

  while (true) {
    // в начале блокируем семафор доступа к входному пакету данных
    while ( xSemaphoreTake(sem_InputOWBPacket,(TickType_t) 10) != pdTRUE ) {
      vTaskDelay(1/portTICK_PERIOD_MS);         
    }
    if (OneWireBus.GetData((uint8_t*)&InOWBBuffer,sizeof(InOWBBuffer))) { // если пакет смогли прочитать - то:
      tmp_RecieveCRC = GetCrc16Simple((uint8_t*)&InOWBBuffer, sizeof(InOWBBuffer)-4);
      if (InOWBBuffer.simple_crc16 == tmp_RecieveCRC) { // если целостность пакета подтверждена CRC - то рассматриваем его как команды              
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода           
        Serial.printf("Recieve OWB packet with CRC=%04X\n",InOWBBuffer.simple_crc16);
        #endif        
        f_HasOWBPacket = true;
        // уходим в асинхронную обработку и отдадим семафор только после преобразования OWB пакета в команды
      }  
      else {
        xSemaphoreGive( sem_InputOWBPacket );  // если CRC не сошлась - снимаем семафор
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода           
        Serial.printf("!!!Bad CRC=%04X for recieve OWB packet!\n",InOWBBuffer.simple_crc16);
        #endif        
      }
    }  
    else xSemaphoreGive( sem_InputOWBPacket );    // если данных нет - так же снимаем семафор        
    vTaskDelay(pdMS_TO_TICKS(C_OWB_READ_BUS_DELAY));                // делаем задержку в чтении следующего цикла
  }
}

// ------------------------ команды, которые обрабатываются в рамках получения событий ---------------------

void cmdReset() {
// команда сброса конфигурации до состояния по умолчанию и перезагрузка
  if (mqttClient.connected()) mqttClient.publish(curConfig.lwt_topic, 0, true, jv_OFFLINE);  // публикуем в топик LWT_TOPIC событие об отключении
  ESP.restart();                                                                             // перезагружаемся  
}

void cmdClearConfig_Reset() {
// команда сброса конфигурации до состояния по умолчанию и перезагрузка
  if (s_EnableEEPROM) { // если EEPROM разрешен и есть             
      SetConfigByDefault();                                                                  // в конфигурацию записываем значения по умолчанию
      curConfig.simple_crc16 = GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4);    // считаем CRC16 для конфигурации
      EEPROM.put(0,curConfig);                                                               // записываем конфигурацию
      EEPROM.commit();                                                                       // подтверждаем изменения
    }  
  cmdReset();                                                                                // перезагружаемся  
}

void cmdSwitchInput(const bool InpMode) { // команда переключения входов усилителя  
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.printf("Try switch from %s to %s\n", curConfig.inp_selector ? jv_XLR : jv_RCA, InpMode ? jv_XLR : jv_RCA);
  #endif      
  if (InpMode != curConfig.inp_selector) {                                                   // проверяем, что необходимо переключить вход
    f_HasChanges = true;                                                                     // взводим флаг изменения
    f_HasReportNow = true;                                                                   // взводим флаг отчёта об изменении состояния
    f_HasDataForSync = true;                                                                 // взводим флаг необходимости синхронизации
    curConfig.inp_selector = InpMode;                                                        // собственно переключаем вход
    if (curConfig.inp_selector) digitalWrite(RELAY_SELECTOR_PIN, LOW);                       // подключаем вход RCA    
      else digitalWrite(RELAY_SELECTOR_PIN, HIGH);                                           // подключаем вход XLR    
  }
}

void cmdEnableTrigger(const bool _Mode) { 
// разрешение/запрещение работы триггеров
  if (curConfig.enable_triggers != _Mode) {
    curConfig.enable_triggers = _Mode;
    f_HasReportNow = true; 
  }
}

void cmdEnableOWBSync(const bool _Mode) { 
// разрешение синхронизации по OneWireBUS
  if (curConfig.sync_by_owb != _Mode) {
    curConfig.sync_by_owb = _Mode;
    f_HasReportNow = true;   
    f_HasDataForSync = curConfig.sync_by_owb;                                               // взводим флаг необходимости синхронизации    
  }
}

void cmdTriggerByPass(const bool _Mode) { 
// разрешение сквозной синхронизации триггеров
  if (curConfig.sync_trigger_in_out != _Mode) {
    curConfig.sync_trigger_in_out = _Mode;
    f_HasReportNow = true;   
  }
}

void cmdChangeVULightMode(const char * _Mode) {
// переключаем режим освещения в нужный режим     
 for (uint8_t i = 0; i < MAX_VU_MODE; i++) {    // перебираем строки пока не найдем нашу
   if (strcmp(VU_mode_str[i],_Mode) == 0) {     // если строку нашли
      curConfig.vu_light_mode = i;              // устанавливаем правильный режим
      f_HasReportNow = true;                    // отчитываемся об этом
      f_HasChanges = true;                      // устанавливаем флаг наличия изменений
      SetGoalBrightness();                      // расчитываем новое значение для текущей яркости
      f_HasDataForSync = true;                  // взводим флаг необходимости синхронизации 
      break;  
    } 
 }
}

void cmdChangeManualPWMSet(uint16_t _min, uint16_t _mid, uint16_t _max) {
// функция изменения параметров яркости ручного режима
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.printf("Set manual PWM to [%u,%u,%u] \n",_min,_mid,_max);
  #endif   
  curConfig._min_manual_pwm = _min;
  curConfig._mid_manual_pwm = _mid;
  curConfig._max_manual_pwm = _max;
  f_HasChanges = true; 
  SetGoalBrightness();                          // расчитываем новое значение для текущей яркости
}

void cmdChangeAutoPWMSet(uint16_t _min, uint16_t _max) {
// функция изменения параметров яркости автоматического режима  
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.printf("Set borders for auto PWM to [%u,%u] \n",_min,_max);
  #endif   
  curConfig._min_auto_pwm = _min;
  curConfig._max_auto_pwm = _max;
  f_HasChanges = true; 
  SetGoalBrightness();                          // расчитываем новое значение для текущей яркости
}

void cmdChangeSensorMapSet(uint16_t _min, uint16_t _max) {
// вызываем функцию изменения параметров мапировки сенсора
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.printf("Set range for sensor to [%u,%u] \n",_min,_max);
  #endif   
  curConfig._min_ambient_value = _min;
  curConfig._max_ambient_value = _max;
  f_HasChanges = true; 
  SetGoalBrightness();                          // расчитываем новое значение для текущей яркости
}

void cmdPowerON() {
// команда включения усилителя
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.printf("Switch power from %s to ON\n",s_AmpPowerOn ? jv_ON : jv_OFF);
  #endif  
  if (!s_AmpPowerOn) { // если усилитель еще не включен 
    s_AmpPowerOn = true;                                 // отмечаем текущее состояние  
    digitalWrite(RELAY_POWER_PIN, HIGH);                 // включаем основной силовой блок питания
    tm_TogglePower = millis();                           // запоминаем момент включения основного питания 
    s_VU_Enable = false;                                 // запрещаем работу стрелочного указателя на период переходных процессов    
    SetGoalBrightness();                                 // расчитываем новое значение для текущей яркости
    f_HasChanges = true;
    f_HasReportNow = true;
    f_HasDataForSync = true;                             // взводим флаг необходимости синхронизации          
  }
}

void cmdPowerOFF() {
// команда сброса конфигурации до состояния по умолчанию и перезагрузка
  #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
  Serial.printf("Switch power from %s to OFF\n",s_AmpPowerOn ? jv_ON : jv_OFF);
  #endif    
  if (s_AmpPowerOn) { // если усилитель еще не выключен 
    s_AmpPowerOn = false;                                // отмечаем текущее состояние  
    CheckAndUpdateEEPROM();                              // запоминаем текущую конфигурацию  
    digitalWrite(RELAY_POWER_PIN, LOW);                  // выключаем основной силовой блок питания  
    tm_TogglePower = millis();                           // запоминаем момент выключения основного питания 
    s_VU_Enable = false;                                 // запрещаем работу стрелочного указателя на период переходных процессов    
    f_HasChanges = true;  
    f_HasReportNow = true;  
    SetGoalBrightness();                                 // расчитываем новое значение для текущей яркости = 0       
    f_HasDataForSync = true;                             // взводим флаг необходимости синхронизации              
  }
}

// ================================== основные задачи времени выполнения =================================

void eventHandlerTask (void *pvParam) {
// задача обработки событий получения команды от датчика, таймера, MQTT, OneWire, кнопок

  while (true) {
    //-------------------- обработка команд задержки и таймера ---------------------------------------
    // обработка флага мигания
    if (millis()-tm_LastBlinkFire > C_BLINKER_DELAY) {  // переключаем блинкер по задержке 
      f_Blinker = !f_Blinker;
      tm_LastBlinkFire = millis();
    } 
    // обработка задержки включения стрелок после смены режима включения/выключения усилителя
    if (!s_VU_Enable)  { // обрабатываем задержку только если стрелочки выключены
      if (s_AmpPowerOn) s_VU_Enable = ((millis()-tm_TogglePower) > C_VU_DELAY_ON) and (curConfig.vu_light_mode != 0);      // строка для включения усилителя
        else s_VU_Enable = ((millis()-tm_TogglePower) > C_VU_DELAY_OFF) and (curConfig.vu_light_mode != 0);      // строка для выключения усилителя
      if (s_VU_Enable) {        
        #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода 
        Serial.println("VU индикатор включен...");
        #endif  
        f_HasChanges = true;
      }
    }      
    // обработка значений датчика освещенности для установки нового уровня яркости в случае необходимости
    if ((millis()-tm_LastAmbientCheck) > C_AMBIENT_CHECK_DELAY) {
      v_CurrAmbient = analogRead(AMBIENT_SENSOR_PIN);                 // начитываем значение для уровня освещенности
      SetGoalBrightness();                                            // устанавливаем текущее значение целевой яркости если нужно
      tm_LastAmbientCheck = millis();                                 // запоминаем момент обновления для следующей задержки
    }
    //-------------------- обработка событий получения MQTT команд в приложение ----------------------
    if ( f_HasMQTTCommand ) {                                         // превращаем события MQTT в команды для отработки приложением    
      // защищаем секцию работы с Static JSON DOC с помощью мьютекса
      // MQTT: clear_config
      if (InputJSONdoc.containsKey(jc_CLR_CONFIG) and InputJSONdoc[jc_CLR_CONFIG])  {   // послана команда обнуления конфигурации и перезагрузки
        cmdClearConfig_Reset();
      }
      // MQTT: reset
      if (InputJSONdoc.containsKey(jc_RESET) and InputJSONdoc[jc_RESET])  {             // послана команда перезагрузки
        cmdReset();                                                   // просто перезагружаемся  
      }
      // MQTT: report
      if (InputJSONdoc.containsKey(jc_REPORT) and InputJSONdoc[jc_REPORT])  {   // послана команда принудительного отчета
        f_HasReportNow = true;                                       // взводим флаг, что отчёт нужен сейчас
      }
      // MQTT: power on|off
      if (InputJSONdoc.containsKey(jk_POWER))  {   // послана команда включения/выключения блока
        if ((InputJSONdoc[jk_POWER] == jv_ON) and !s_AmpPowerOn)  cmdPowerON();   // если блок выключен и пришла команда включения
        if ((InputJSONdoc[jk_POWER] == jv_OFF) and s_AmpPowerOn)  cmdPowerOFF();  // если блок включен и пришла команда выключения     
      }
      // MQTT: input selector rca|xlr
      if (InputJSONdoc.containsKey(jk_SELECTOR))  {   // послана команда переключения входа 
        if ((InputJSONdoc[jk_SELECTOR] == jv_RCA) and s_AmpPowerOn)  cmdSwitchInput(INP_RCA);  // команда переключения на RCA
        if ((InputJSONdoc[jk_SELECTOR] == jv_XLR) and s_AmpPowerOn)  cmdSwitchInput(INP_XLR);  // команда переключения на XLR
      }
      // MQTT: разрешение работы выходного триггера on|off
      if (InputJSONdoc.containsKey(jk_TRG_ENABLE))  {   // послана команда включения/выключения выходного триггера
        if (InputJSONdoc[jk_TRG_ENABLE] == jv_ON) cmdEnableTrigger(true);                     // разрешаем триггера
        if (InputJSONdoc[jk_TRG_ENABLE] == jv_OFF) cmdEnableTrigger(false);                   // запрещаем триггера
      }
      // MQTT: разрешение синхронизации по OneWireBUS on|off
      if (InputJSONdoc.containsKey(jk_SYNC_BY_OWB))  {   // послана команда включения/выключения синхронизации по OneWireBUS
        if (InputJSONdoc[jk_SYNC_BY_OWB] == jv_ON) cmdEnableOWBSync(true);                   // разрешаем синхронизацию по OneWireBUS
        if (InputJSONdoc[jk_SYNC_BY_OWB] == jv_OFF) cmdEnableOWBSync(false);                 // запрещаем синхронизацию по OneWireBUS
      }
      // MQTT: разрешение сквозной синхронизации триггеров
      if (InputJSONdoc.containsKey(jk_TRIGGER_BYPASS))  {   // послана команда включения/выключения синхронизации триггеров
        if (InputJSONdoc[jk_TRIGGER_BYPASS] == jv_ON) cmdTriggerByPass(true);                // разрешаем синхронизацию триггеров
        if (InputJSONdoc[jk_TRIGGER_BYPASS] == jv_OFF) cmdTriggerByPass(false);              // запрещаем синхронизацию триггеров
      }
      // MQTT: переключение режима освещенности VU индикатора
      if (InputJSONdoc.containsKey(jk_LIGHT_MODE))  {   // послана команда переключения режима освещенности VU индикатора
        const char* _VU_Mode = InputJSONdoc[jk_LIGHT_MODE];
        if(_VU_Mode != nullptr) {                                       // если тип будет другим - будет крэш
          cmdChangeVULightMode(_VU_Mode) ;                              // переключаем режим
        }  
      }
      // MQTT: задание параметров подсветки VU индикатора в ручном режиме
      if (InputJSONdoc.containsKey(jk_LIGHT_MANUAL_SET))  {   // послан набор данных для установки параметров ручных режимов подсветки
        uint16_t _min_manual = DEF_MIN_MANUAL_PWM;
        if (InputJSONdoc[jk_LIGHT_MANUAL_SET][0].is<int16_t>()) _min_manual = InputJSONdoc[jk_LIGHT_MANUAL_SET][0];
        uint16_t _mid_manual = DEF_MID_MANUAL_PWM;
        if (InputJSONdoc[jk_LIGHT_MANUAL_SET][1].is<int16_t>()) _mid_manual = InputJSONdoc[jk_LIGHT_MANUAL_SET][1];
        uint16_t _max_manual = DEF_MAX_MANUAL_PWM;
        if (InputJSONdoc[jk_LIGHT_MANUAL_SET][2].is<int16_t>()) _max_manual = InputJSONdoc[jk_LIGHT_MANUAL_SET][2];
        if ((_min_manual<=_mid_manual) and (_mid_manual<=_max_manual)) {  // простейшая проверка на валидность значений
          cmdChangeManualPWMSet(_min_manual,_mid_manual,_max_manual);                        // вызываем функцию изменения параметров яркости ручного режима
          }
        else {
          #ifdef DEBUG_LEVEL_PORT                                            
          Serial.printf("!!! New result for manual set [%u,%u,%u] - is wrong! \n",_min_manual,_mid_manual,_max_manual);
          #endif
        }
      }
      // MQTT: задание параметров автоматической подсветки VU индикатора
      if (InputJSONdoc.containsKey(jk_LIGHT_AUTO_SET))  {   // послан набор данных для установки параметров автоматического режима подсветки
        uint16_t _min_auto = DEF_MIN_AUTO_PWM;
        if (InputJSONdoc[jk_LIGHT_AUTO_SET][0].is<int16_t>()) _min_auto = InputJSONdoc[jk_LIGHT_AUTO_SET][0];
        uint16_t _max_auto = DEF_MAX_AUTO_PWM;
        if (InputJSONdoc[jk_LIGHT_AUTO_SET][1].is<int16_t>()) _max_auto = InputJSONdoc[jk_LIGHT_AUTO_SET][1];
        if (_min_auto<=_max_auto) {  // простейшая проверка на валидность значений
          cmdChangeAutoPWMSet(_min_auto,_max_auto);                                         // вызываем функцию изменения параметров яркости автоматического режима
          }
        else {
          #ifdef DEBUG_LEVEL_PORT                                            
          Serial.printf("!!! New result for auto set [%u,%u] - is wrong! \n",_min_auto,_max_auto);
          #endif
        }
      }
      // MQTT: задание порогов сработки сенсора для автоматической подсветки VU индикатора
      if (InputJSONdoc.containsKey(jk_AMBIENT_SET))  {   // послан набор данных для установки параметров автоматического режима подсветки
        uint16_t _min_sens = DEF_MIN_AMBIENT_VALUE;
        if (InputJSONdoc[jk_AMBIENT_SET][0].is<int16_t>()) _min_sens = InputJSONdoc[jk_AMBIENT_SET][0];
        uint16_t _max_sens = DEF_MAX_AMBIENT_VALUE;
        if (InputJSONdoc[jk_AMBIENT_SET][1].is<int16_t>()) _max_sens = InputJSONdoc[jk_AMBIENT_SET][1];
        if (_min_sens<_max_sens) {  // простейшая проверка на валидность значений
          cmdChangeSensorMapSet(_min_sens,_max_sens);                                         // вызываем функцию изменения параметров мапировки сенсора
          }
        else {
          #ifdef DEBUG_LEVEL_PORT                                            
          Serial.printf("!!! New result for sensor's map [%u,%u] - is wrong! \n",_min_sens,_max_sens);
          #endif
        }
      }
      f_HasMQTTCommand = false;                                       // сбрасываем флаг наличия изменений через MQTT 
      xSemaphoreGive(sem_InputJSONdoc);                               // отпускаем семафор обработки входного сообщения
    }
    //--------------------- опрос кнопок - получение команд от лицевой панели ------------------------
    bttn_power.tick();                                                // опрашиваем кнопку POWER
    bttn_input.tick();                                                // опрашиваем кнопку INPUT  
    bttn_light.tick();                                                // опрашиваем кнопку LIGHT  
    // однократное нажатие на кнопку POWER
    if (bttn_power.isSingle()) {        
      if (s_AmpPowerOn) cmdPowerOFF();                                // если блок включен и пришла команда выключения     
        else cmdPowerON();                                            // если блок выключен и пришла команда включения
    }
    // однократное нажатие на кнопку выбора входа
    if (bttn_input.isSingle()) {        
      if (s_AmpPowerOn) {                                             // все действия, если усь включен
        cmdSwitchInput(!curConfig.inp_selector);                      // переключаемся на противоположный вход RCA<>XLR
      }
    }
    // однократное нажатие на кнопку переключения освещения
    if (bttn_light.isSingle() and s_AmpPowerOn) { 
      if (curConfig.vu_light_mode==(MAX_VU_MODE-1)) curConfig.vu_light_mode = 0;   
        else curConfig.vu_light_mode++;
      cmdChangeVULightMode(VU_mode_str[curConfig.vu_light_mode]);     // переключаем на нужный режим освещения       
    }
    // одновременное нажатие и удержание кнопок Power и Input  
    // команда сброса конфигурации до заводских параметров и перезагрузка
    if (bttn_power.isHold() and bttn_input.isHold()) {        
        cmdClearConfig_Reset();
    }
    //----------------------------- получение команды по Trigger IN ----------------------------------
    // проверяем вход TRIGGER_IN - и включаем режим устранения переходных процессов при переключении триггерного входа
    if ((f_TriggerIn == digitalRead(TRIGGER_IN_PIN)) and !f_TriggerDebounce) { // произошло изменение на входе триггера и мы не в режиме устранения дребезга - то переходим в него ( равенство потому что вход trigger_in инверсный)
      f_TriggerDebounce = true; 
      tm_TriggerDebounceTime = millis();      // запоминаем время первого изменения состояния TriggerIn      
    }
    // проверка окончания переходных процессов при включении/выключении триггерного входа
    if (f_TriggerDebounce and ((((millis()-tm_TriggerDebounceTime)> C_TRIGGER_ON_DEBOUNCE) and !f_TriggerIn) or (((millis()-tm_TriggerDebounceTime)> C_TRIGGER_OFF_DEBOUNCE) and f_TriggerIn))) {
      // сброс режима устранения дребезга
      f_TriggerDebounce = false; 
      tm_TriggerDebounceTime = 0;    
      // вот теперь можно окончательно считывать и обрабатывать сигнал TriggerIn
      // проверяем вход TRIGGER_IN и загоняем его значение в переменную
      bool tmp_TriggerIn = !digitalRead(TRIGGER_IN_PIN);          // на входе инвертированный сигнал 
      if (f_TriggerIn != tmp_TriggerIn) {                         // произошли изменения - нужна обработка
        if ((f_TriggerIn and !tmp_TriggerIn) and s_AmpPowerOn and curConfig.enable_triggers) {
          // если идет переключение с 1>>0 входа Trigger IN, и усилитель включен и триггер разрешен - то выключаем усь
          cmdPowerOFF();
          s_PowerOnByTriggerIn = false;                           // сбрасываем признак включения по триггеру
          #ifdef DEBUG_LEVEL_PORT                                 // вывод в порт при отладке кода 
          Serial.println("Power >>OFF<< by TriggerIn.");
          #endif    
        }  
        if ((!f_TriggerIn and tmp_TriggerIn) and !s_AmpPowerOn and curConfig.enable_triggers) {
          // если идет переключение с 0>>1 входа Trigger IN, усилитель выключен и триггер разрешен - то включаем усь  
          cmdPowerON();
          s_PowerOnByTriggerIn = true;                            // устанавливаем признак включения по триггеру
          #ifdef DEBUG_LEVEL_PORT                                 // вывод в порт при отладке кода 
          Serial.println("Power <<ON>> by TriggerIn.");
          #endif    
        }
        f_TriggerIn = tmp_TriggerIn;                              // назначаем текущее состояние основной переменной
      }  
    }
    //----------------------------- получение команды по OneWireBUS ----------------------------------
    if (f_HasOWBPacket) { // есть пакет для обработки принятый по OneWireBUS
      // обрабатываем команду переключения входа
      if (InOWBBuffer.power_on != s_AmpPowerOn) {              
        if (InOWBBuffer.power_on) cmdPowerON();
          else cmdPowerOFF();          
      }
      // обрабатываем команду переключения входа
      if (InOWBBuffer.inp_selector != curConfig.inp_selector) cmdSwitchInput(InOWBBuffer.inp_selector);
      // обрабатываем команду переключения режима подсветки
      if (InOWBBuffer.vu_light_mode != curConfig.vu_light_mode) cmdChangeVULightMode(VU_mode_str[InOWBBuffer.vu_light_mode]);     // переключаем на нужный режим освещения 
      #ifdef DEBUG_LEVEL_PORT                                 // вывод в порт при отладке кода 
      Serial.println("Обработали OWB пакет!");
      #endif    
      xSemaphoreGive(sem_InputOWBPacket);                     // отпускаем семафор обработки входного сообщения 
      f_HasDataForSync = false;                               // не даем формировать ответный пакет синхронизации
      f_HasOWBPacket = false;
    }
    // отдаем управление ядру FreeRT OS
    vTaskDelay(1/portTICK_PERIOD_MS); 
  }
}

void applayChangesTask (void *pvParam) {
// применяем изменения состояния, и если нужно сохранение состояния в FLASH памяти
// здесь отражаются внутренние изменения, команды выполняются в процедуре обработки команд eventHandlerTask
  while (true) {
    // в начале исполняем обработку изменений без учёта флага f_HasChanges
    if (s_CurrentWIFIMode == WF_IN_WORK ) {  
      // выключаем светодиод установления связи по WiFi и MQTT при нормально установленном соединении
      digitalWrite(LED_POWER_BLUE_PIN,LOW); 
    }
    if (s_AmpPowerOn) {
      // отображаем индикацию включения модуля
      digitalWrite(LED_POWER_GREEN_PIN,HIGH);
      digitalWrite(LED_POWER_RED_PIN,LOW);
      // отображаем селектор входов
      digitalWrite(LED_SELECTOR_RCA_PIN,!curConfig.inp_selector);     // RCA = 0 - поэтому инверсия
      digitalWrite(LED_SELECTOR_XLR_PIN,curConfig.inp_selector);      // XLR = 1 - прямая запись
      } 
    else {
      // отображаем индикацию выключения модуля
      digitalWrite(LED_POWER_GREEN_PIN,LOW);
      digitalWrite(LED_POWER_RED_PIN,HIGH);
      // выключаем селектор входов        
      digitalWrite(LED_SELECTOR_RCA_PIN,LOW); 
      digitalWrite(LED_SELECTOR_XLR_PIN,LOW);
    }
    // проверяем текущее состояние работы c WiFi
    switch (s_CurrentWIFIMode) {
      case WF_AP: // включаем мигающий режим светодиода индицирующего работу по WiFi в режиме точки доступа
        digitalWrite(LED_POWER_BLUE_PIN,f_Blinker);
        break;
      case WF_IN_WORK: // при нормально установленном соединении светодиод управляется в циклах приема и отправки сообщений        
        break;        
      case WF_OFF:     // выключаем светодиод при полном отключении WiFi
        digitalWrite(LED_POWER_BLUE_PIN,LOW); 
        break;    
      default:    // по умолчанию включаем светодиод
        digitalWrite(LED_POWER_BLUE_PIN,HIGH);
        break;
    }    
    // включаем синий светодиод при обработке сигнала триггерного входа / выключится он сам 
    if (f_TriggerDebounce) digitalWrite(LED_POWER_BLUE_PIN,HIGH);    
    // обслуживание триггерного выхода - TRIGGER_OUT
    if (curConfig.enable_triggers) { // если разрешена работа внешних триггеров то:
      // если включен режим прямой трансляции сигнала Trigger - то просто дублируем сигнал входа на выходе
      if (curConfig.sync_trigger_in_out) {  // при этом переворачиваем сигнал, так как Trigger_IN - инверсный
        if (digitalRead(TRIGGER_IN_PIN)) { digitalWrite(TRIGGER_OUT_PIN, LOW);}     // выключаем порт выхода TRIGGER_OUT  
          else {digitalWrite(TRIGGER_OUT_PIN, HIGH);}                                // включаем порт выхода TRIGGER_OUT  
        }
      else digitalWrite(TRIGGER_OUT_PIN, s_AmpPowerOn);  // иначе выход Trigger_OUT поднимаем, когда включен усилитель и разрешен Trigger_OUT
      }  
    else digitalWrite(TRIGGER_OUT_PIN, LOW);  // если внешний триггер запрещен - держим его выключенным    
  // этот блок работы с подсветкой шкалы VU индикатора 
  int  tmp_Delta = 1;
  // меняем значение подсветки на текущую  
  ledcWrite(c_PWM_Channel, v_CurrBrightness);    
  // отслеживаем изменение разницы целевой подсветки и текущей - текущую тянем к целевой с задержкой
  if (((millis()-tm_LastBrightnessSet) > C_BRIGHTNESS_SET_DELAY) and (v_CurrBrightness != v_GoalBrightness)) {
  // здесь финт ушами по плавной коррекции значения текущей подсветки - плавность зависит от задержки C_BRIGHTNESS_SET_DELAY
    tm_LastBrightnessSet = millis();                                    // запоминаем последнее изменение
    if (v_CurrBrightness > v_GoalBrightness) {                          // нужно уменьшать текущую яркость на величину ошибки
       // методом деления ошибки на пополам, приходим к нужной величине 
       tmp_Delta = (v_CurrBrightness-v_GoalBrightness)/2;               // вычисляем половинную величину ошибки
       if (tmp_Delta == 0) tmp_Delta = 1;                               // если ошибка ушла за предел точности, то ошибка = 1
       v_CurrBrightness = v_CurrBrightness - tmp_Delta;                 // уменьшаем рассчитанную текущую яркость на половину ошибки
    }
    if (v_CurrBrightness < v_GoalBrightness) {                          // увеличиваем текущую яркость на величину ошибки
       // методом деления ошибки на пополам, приходим к нужной величине 
       tmp_Delta = (v_GoalBrightness-v_CurrBrightness)/2;               // вычисляем половинную величину ошибки
       if (tmp_Delta == 0) tmp_Delta = 1;                               // если ошибка ушла за предел точности, то ошибка = 1
       v_CurrBrightness = v_CurrBrightness + tmp_Delta;                 // увеличиваем рассчитанную текущую освещенность на половину ошибки
    }
  }   
    // ----- ниже исполняется только то, что отмечено флагом изменений f_HasChanges ----
    if (f_HasChanges) {
      // включаем/выключаем стрелочки 
      if (s_VU_Enable) digitalWrite(MUTE_VU_PIN, LOW); // включаем стрелочки MUTE_VU_PIN = 0
        else digitalWrite(MUTE_VU_PIN, HIGH); // выключаем стрелочки MUTE_VU_PIN = 1
      f_HasChanges = false;
    }
    vTaskDelay(1/portTICK_PERIOD_MS); 
  }
}

void sendCommandTask (void *pvParam) {
// шлем команду по OneWireBUS
  while (true) {
    if (curConfig.sync_by_owb and f_HasDataForSync) {                                 // если есть данные для синхронизации и она разрешена
        // готовим данные для передачи
        OutOWBBuffer.inp_selector = curConfig.inp_selector;
        OutOWBBuffer.power_on = s_AmpPowerOn;
        OutOWBBuffer.vu_light_mode = curConfig.vu_light_mode;
//        OutOWBBuffer.vu_light_value = v_GoalBrightness;
        OutOWBBuffer.simple_crc16 = GetCrc16Simple((uint8_t*)&OutOWBBuffer, sizeof(OutOWBBuffer)-4);
        // и собственно передаем через OneWireBUS
        if (OneWireBus.SendData(BROADCAST_ADDR,(u_char*)&OutOWBBuffer, sizeof(OutOWBBuffer)))  {
          #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода           
          Serial.printf("Send OWB packet with CRC=%04X\n",OutOWBBuffer.simple_crc16);
          #endif        
          f_HasDataForSync = false;     // сбрасываем флаг синхронизации
          v_OWBErrorCount = 0;          // сбрасываем счётчик ошиок передачи 
          }
        else {
          v_OWBErrorCount++;
          if (v_OWBErrorCount > C_MAX_LOST_ON_OWB) {      // если счётчик подряд НЕ ОТПРАВЛЕННЫХ пакетов достиг максимума потерь      
            v_OWBErrorCount = 0;                          // сбрасываем счётчик
            curConfig.sync_by_owb = false;                // запрещаем синхронизацию
            f_HasReportNow = true;                        // рапортуем об этом
            vTaskDelay(pdMS_TO_TICKS(C_OWB_TRY_DELAY + random(v_OWBErrorCount+1)));   // делаем задержку перед следующей отправкой на случайный промежуток времени
          }
          #ifdef DEBUG_LEVEL_PORT       // вывод в порт при отладке кода                     
          Serial.println("!!!Error of send OWB packet!");
          #endif            
        }
    }
    vTaskDelay(1/portTICK_PERIOD_MS);     
  }
}

void reportTask (void *pvParam) {
// репортим о текущем состоянии в MQTT и если отладка то и в Serial
  while (true) {
    if (((millis()-tm_LastReportToMQTT)>cur_MQTT_REPORT_DELAY) || f_HasReportNow) {  // если наступило время отчёта или взведен флаг "отчёта сейчас"
      digitalWrite(LED_POWER_BLUE_PIN, HIGH);                                                       // включение через подачу 1
      if (s_AmpPowerOn) cur_MQTT_REPORT_DELAY = C_MQTT_REPORT_DELAY_ON;                             // при генерации отчёта делаем коррекцию на включение усилителя
        else cur_MQTT_REPORT_DELAY = C_MQTT_REPORT_DELAY_OFF;
      if (mqttClient.connected()) {  // если есть связь с MQTT - репорт в топик
        // чистим документ
        OutputJSONdoc.clear(); 
        // добавляем поля в документ
        OutputJSONdoc[jk_POWER] = s_AmpPowerOn ? jv_ON : jv_OFF;                                    // ключ общего включения
        OutputJSONdoc[jk_SELECTOR] = curConfig.inp_selector ? jv_XLR : jv_RCA;                      // режим входа RCA / XLR
        OutputJSONdoc[jk_LIGHT_MODE] = VU_mode_str[curConfig.vu_light_mode];                        // режим подсветки VU индикатора
        OutputJSONdoc[jk_BRIGHTNESS] = v_GoalBrightness;                                            // значение целевой яркости подсветки
        OutputJSONdoc[jk_AMBIENT] = v_CurrAmbient;                                                  // значение датчика освещенности
        OutputJSONdoc[jk_TRIGGER_IN] = f_TriggerIn ? jv_ON : jv_OFF;                                // состояние входа триггера
        OutputJSONdoc[jk_TRG_ENABLE] = curConfig.enable_triggers ? jv_ON : jv_OFF;                  // разрешение триггерных выходов/выходов 
        OutputJSONdoc[jk_TRIGGER_OUT] = digitalRead(TRIGGER_OUT_PIN) ? jv_ON : jv_OFF;              // состояние триггерного выхода 
        OutputJSONdoc[jk_TRIGGER_BYPASS] = curConfig.sync_trigger_in_out ? jv_ON : jv_OFF;          // проброс триггерного входа на выход
        OutputJSONdoc[jk_SYNC_BY_OWB] = curConfig.sync_by_owb ? jv_ON : jv_OFF;                     // синхронизация по OneWireBUS
        // добавляем массив значений ручной подсветки
        OutputJSONdoc[jk_LIGHT_MANUAL_SET][0] = curConfig._min_manual_pwm;                     
        OutputJSONdoc[jk_LIGHT_MANUAL_SET][1] = curConfig._mid_manual_pwm;                     
        OutputJSONdoc[jk_LIGHT_MANUAL_SET][2] = curConfig._max_manual_pwm;                    
        // добавляем границы автоматической подсветки
        OutputJSONdoc[jk_LIGHT_AUTO_SET][0] = curConfig._min_auto_pwm;                     
        OutputJSONdoc[jk_LIGHT_AUTO_SET][1] = curConfig._max_auto_pwm;                     
        // добавляем границы работы сенсора освещения
        OutputJSONdoc[jk_AMBIENT_SET][0] = curConfig._min_ambient_value;                     
        OutputJSONdoc[jk_AMBIENT_SET][1] = curConfig._max_ambient_value;                     
        // серилизуем в строку
        String tmpPayload;
        serializeJson(OutputJSONdoc, tmpPayload);
        // публикуем в топик P_STATE_TOPIC серилизованный json через буфер buffer
        char buffer[ tmpPayload.length()+1 ];
        tmpPayload.toCharArray(buffer, sizeof(buffer));   
        mqttClient.publish(P_STATE_TOPIC, 0, true, buffer );
      }
      #ifdef DEBUG_LEVEL_PORT 
        Serial.println();
        Serial.println("<<<< Current state report >>>>");
        Serial.printf("%s : %s\n", jk_POWER, s_AmpPowerOn ? jv_ON : jv_OFF );
        Serial.printf("%s : %s\n", jk_SELECTOR, curConfig.inp_selector ? jv_XLR : jv_RCA);
        Serial.printf("%s : %s\n", jk_LIGHT_MODE, VU_mode_str[curConfig.vu_light_mode] );
        Serial.printf("%s : %d\n", jk_BRIGHTNESS, v_GoalBrightness );
        Serial.printf("%s : %d\n", jk_AMBIENT, v_CurrAmbient );
        Serial.printf("%s : %s\n", jk_TRIGGER_IN, f_TriggerIn ? jv_ON : jv_OFF );
        Serial.printf("%s : %s\n", jk_TRG_ENABLE, curConfig.enable_triggers ? jv_ON : jv_OFF);
        Serial.printf("%s : %s\n", jk_TRIGGER_OUT, digitalRead(TRIGGER_OUT_PIN) ? jv_ON : jv_OFF);
        Serial.printf("%s : %s\n", jk_TRIGGER_BYPASS, curConfig.sync_trigger_in_out ? jv_ON : jv_OFF );
        Serial.printf("%s : %s\n", jk_SYNC_BY_OWB, curConfig.sync_by_owb ? jv_ON : jv_OFF );
        Serial.printf("%s : [%u,%u%,%u] \n", jk_LIGHT_MANUAL_SET,curConfig._min_manual_pwm,curConfig._mid_manual_pwm,curConfig._max_manual_pwm);
        Serial.printf("%s : [%u,%u] \n", jk_LIGHT_AUTO_SET,curConfig._min_auto_pwm,curConfig._max_auto_pwm);
        Serial.printf("%s : [%u,%u] \n", jk_AMBIENT_SET,curConfig._min_ambient_value,curConfig._max_ambient_value);                
        Serial.println("<<<< End of current report >>>>");
      #endif                
      tm_LastReportToMQTT = millis();           // взводим интервал отсчёта
      f_HasReportNow = false;                   // сбрасываем флаг
      digitalWrite(LED_POWER_BLUE_PIN, LOW);    // выключение индикации передачи по MQTT через подачу 0             
    }
    vTaskDelay(1/portTICK_PERIOD_MS);         
  }
}

// -------------------------- в этом фрагменте описываем call-back функции MQTT клиента --------------------------------------------
void onMqttConnect(bool sessionPresent) {   
  // обработчик подключения к MQTT
  #ifdef DEBUG_LEVEL_PORT                                    
    Serial.println("Connected to MQTT.");  //  "Подключились по MQTT."
  #endif                
  // далее подписываем ESP32 на набор необходимых для управления топиков:
  uint16_t packetIdSub = mqttClient.subscribe(curConfig.command_topic, 0);  // подписываем ESP32 на топик SET_TOPIC
  #ifdef DEBUG_LEVEL_PORT                                      
    Serial.printf("Subscribing at QoS 0, packetId: %d on topic :[%s]\n", packetIdSub, curConfig.command_topic);
  #endif                  
  // сразу публикуем событие о своей активности
  mqttClient.publish(curConfig.lwt_topic, 0, true, jv_ONLINE);           // публикуем в топик LWT_TOPIC событие о своей жизнеспособности
  #ifdef DEBUG_LEVEL_PORT                                      
    Serial.printf("Publishing LWT state in [%s]. QoS 0. ", curConfig.lwt_topic); 
  #endif                     
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  // обработчик отключения от MQTT  
  #ifdef DEBUG_LEVEL_PORT                                                                           
    Serial.println("Disconnected from MQTT.");                      // если отключились от MQTT
  #endif         
  s_CurrentWIFIMode = WF_UNKNOWN;                                   // переходим в режим полного реконнекта по WiFi
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  // обработка подтверждения подписки на топик
  #ifdef DEBUG_LEVEL_PORT   
    Serial.printf("Subscribe acknowledged. \n  packetId: %d\n  qos: %d\n", packetId, qos);  
  #endif         
}

void onMqttUnsubscribe(uint16_t packetId) {
  // обработка подтверждения отписки от топика  
  #ifdef DEBUG_LEVEL_PORT     
    Serial.printf("Unsubscribe acknowledged.\n  packetId: %d\n", packetId); 
  #endif                     
}

void onMqttPublish(uint16_t packetId) {
  // обработка подтверждения публикации
  #ifdef DEBUG_LEVEL_PORT     
    Serial.printf("Publish acknowledged.\n  packetId: %d\n", packetId);   
  #endif                     
}


// в этой функции обрабатываем события получения данных в управляющем топике SET_TOPIC
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;
  
  for (int i = 0; i < len; i++) {                       // преобразуем полученные в сообщении данные в строку при этом выкидываем символы кавычек
      messageTemp += (char)payload[i];
  }
  messageTemp[len] = '\0';  
  // проверяем, что мы получили MQTT сообщение в командном топике
  if (strcmp(topic, curConfig.command_topic) == 0) {
    // разбираем MQTT сообщение и подготавливаем буфер с изменениями для формирования команд    
    // для этого получаем семафор обработки входного JSON документа
    while ( xSemaphoreTake(sem_InputJSONdoc,(TickType_t) 10) != pdTRUE ) {
      vTaskDelay(1/portTICK_PERIOD_MS);         
    }
    DeserializationError err = deserializeJson(InputJSONdoc, messageTemp);                    // десерилизуем сообщение и взводим признак готовности к обработке
    if (err) {
      #ifdef DEBUG_LEVEL_PORT         
      Serial.print(F("Error of deserializeJson(): "));
      Serial.println(err.c_str());
      #endif
      }
    else {  
      // для коротких сообщений без ключей дополняем DOC объект
      if (strstr(payload,jc_CLR_CONFIG) != NULL ) InputJSONdoc[jc_CLR_CONFIG] = true;
      if (strstr(payload,jc_RESET) != NULL ) InputJSONdoc[jc_RESET] = true;
      if (strstr(payload,jc_REPORT) != NULL ) InputJSONdoc[jc_REPORT] = true;
      f_HasMQTTCommand = true;                            // взводим флаг получения команды по MQTT
      // отдадим семафор обработки документа только после преобразования JSON документа в команды
    }
  }
  #ifdef DEBUG_LEVEL_PORT         
  Serial.printf("Publish received.\n  topic: %s\n  message: [", topic);
  Serial.print(messageTemp); Serial.println("]");
  #endif
}

// =================================== инициализация контроллера и программных модулей ======================================
// начальная инициализация программы - выполняется при подаче дежурного питания.
// дальнейшее включение усилителя - уже в рамках работающей программы
void setup() {
uint8_t MacAddress[8];                        // временная переменная для MAC адреса текущей ESP

#ifdef DEBUG_LEVEL_PORT                       // вывод в порт при отладке кода
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

  // инициализация генератора случайных чисел MAC адресом
  if (esp_efuse_mac_get_default(MacAddress) == ESP_OK) randomSeed(MacAddress[5]);
    else randomSeed(millis());

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
     Serial.println("Ошибка инициализации шины OneWireBUS!");
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
    Serial.printf("\n--- Инициализация блока управления прошла со следующими параметрами: ---\n");
    Serial.printf("  Input RCA-1|XLR-0 : %s\n", curConfig.inp_selector ? "XLR" : "RCA");
    Serial.printf("  VU light mode: %s\n", VU_mode_str[curConfig.vu_light_mode]);    
    Serial.printf("  Ext trigger sync:  %s\n", curConfig.sync_trigger_in_out ? "true" : "false");  
    Serial.printf("  WiFi mode: %d \n", s_CurrentWIFIMode);
    Serial.printf("  WiFi SSid: %s\n", curConfig.wifi_ssid);    
    Serial.printf("  WiFi pwd: %s\n", curConfig.wifi_pwd);
    Serial.printf("  MQTT usr: %s\n", curConfig.mqtt_usr);
    Serial.printf("  MQTT pwd: %s\n", curConfig.mqtt_pwd);
    Serial.printf("  MQTT addr: %u.%u.%u.%u\n", curConfig.mqtt_host[0], curConfig.mqtt_host[1], curConfig.mqtt_host[2], curConfig.mqtt_host[3]); 
    Serial.printf("  MQTT port: %d\n", curConfig.mqtt_port);
    Serial.println("---");    
    Serial.printf("  COMMAND topic: %s\n", curConfig.command_topic);
    Serial.printf("  REPORT topic: %s\n", curConfig.report_topic);
    Serial.printf("  LWT topic: %s\n", curConfig.lwt_topic);
    Serial.println("---");    
    Serial.printf("  CRC by read: %04X\n",curConfig.simple_crc16);
    Serial.printf("  CRC by calc: %04X\n",GetCrc16Simple((uint8_t*)&curConfig, sizeof(curConfig)-4));  
    Serial.printf("---\n\n");
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

  // настраиваем семафоры - сбрасываем их
  xSemaphoreGive( sem_InputJSONdoc );
  xSemaphoreGive( sem_InputOWBPacket );

  // создаем отдельные параллельные задачи, выполняющие группы функций  
  // стартуем основные задачи
  if (xTaskCreate(eventHandlerTask, "events", 4096, NULL, 1, NULL) != pdPASS) Halt("Error: Event handler task not created!");     // все плохо, задачу не создали
  if (xTaskCreate(applayChangesTask, "applay", 4096, NULL, 1, NULL) != pdPASS) Halt("Error: Applay changes task not created!");   // все плохо, задачу не создали
  if (xTaskCreate(sendCommandTask, "send", 4096, NULL, 1, NULL) != pdPASS) Halt("Error: Send command task not created!");         // все плохо, задачу не создали
  if (xTaskCreate(reportTask, "report", 4096, NULL, 1, NULL) != pdPASS) Halt("Error: Report task not created!");                  // все плохо, задачу не создали
  // стартуем коммуникационные задачи
  if (xTaskCreate(oneWireTask, "onewire", 4096, NULL, 1, NULL) != pdPASS) Halt("Error: OneWire communication task not created!"); // все плохо, задачу не создали
  if (xTaskCreate(wifiTask, "wifi", 4096*2, NULL, 1, NULL) != pdPASS) Halt("Error: WiFi communication task not created!");        // все плохо, задачу не создали

}

// не используемый основной цикл
void loop() {
  vTaskDelete(NULL);   // удаляем не нужную задачу loop()  
}