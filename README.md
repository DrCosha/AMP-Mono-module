# Модуль управления оконечным усилителем
## (Amplifiler control unit)

Модуль управления оконечным усилителем с индикатором выходной мощности. 
Управляющая программа с поддержкой межмодульных линков, и управлением через MQTT.

## Общее описание

Модуль управляет усилителем мощности со следующими возможностями:

- работа в режимах **standby** и **powerOn**;
- индикация мощности стрелочным индикатором с подсветкой;
- поддержка входов формата **RCA** и **XLR**, с переключением между ними;
- установка режимов подсветки стрелочного индикатора как вручную, так и по сенсору освещенности;
- защита акустических систем от постоянного напряжения и перегрузки;
- принудительное отключение стрелочного индикатора в моменты переходных процессов включения/выключения усилителя.

<div align="center"><img alt="Overview" width="800" src="/images/ACU_main_circuit.png" />&emsp;&emsp;&emsp;</div>
<img width="40%" src="/images/3D_acu_pcb_view.png">
<img width="40%" src="/images/amp_main_view.png"/>

Общие возможности: управление осуществляется в двух основных режимах - standby


Модуль управления моноблока получает команды от следующих устройств и блоков:

Лицевая панель управления:
- кнопка Power на лицевой панели;
- кнопка Input на лицевой панели;
- кнопка VU light на лицевой панели;
- датчик освещенности 

Входы/выходы управления на задней панели:
- вход TriggerIN - подача питания от +5..12V на вход (5-15mA), приводит к сработке входного триггера. На входе есть задержка от дребезга контактов, а также
  задержка выключения устройства для подавления переходных процессов при включении/выключении внешнего управления и переключении входов на внешних устройствах.
- выход TriggerOUT - работает в режиме подачи сигнала +12V (до 100mA) при включении модуля, либо в режиме проброски сигнала с TriggerIN на TriggerOUT.
- шина OneWireBUS - подключение к общей шине управления для приема/передачи команд по OneWire. Однопроводная шина с подключением до 254 блоков параллельно
  для синхронизации сигналов управления. Все блоки могут асинхронно передавать данные в режиме peer2peer (все-всем). На шине нет устройства "мастера", все равнозначны.

Управление через WiFi:
- сервер MQTT - подключение к топикам сервера для возврата статуса и получения команд.
- WEB сервер со страницей настроек доступа к WiFi и MQTT. Страница доступна по IP адресу подключения модуля. 

Моноблок усилителя раблотает в комплексе с аналогичными, поэтому общается с ними следующим образом:

-------- OneWire BUS:
Блоки работают на общей шине по принципу равных. Необходимость в синхронизации блоков и как следствие - передачи данных по шине 
возникает только при воздействии с внешних источников. Передача параметров по шине OneWire и их применение не приводит к повторной передаче данных по OneWire. По шине идет передача только
статуса включения/выключения блока, выбора источника и установленного режима подсветки. Т.е. синхронизация базового управления.

-------- WiFi:
Подключение к WiFi проводится с заранее установленными параметрами.  Если подключение успешно, устанавливается соединение с MQTT сервером. При первичной настройке или не доступности WiFi 
сети или MQTT сервера, поднимается собственная точка доступа с именем HiAMP_xxxx - где xxxx - это 4 последних цифры MAC адреса. Без пароля по умолчанию.  При этом, если есть соединение 
с WiFi, но нет соединения с MQTT, идут попытки установки соединения с ним в рамках заданной сети. Если это не происходит за 10 повторов, то повторно поднимается собственная точка доступа 
с возможностью подключения к ней и изменения настроек. Если никто не соединяется с этой точкой в течение 3 минут, то цикл попытки установки WiFi соединения и соединения c MQTT повторяется. 
После 3-х полных циклов, если нет успешного соединения с WiFi и/или MQTT сервером в сети, модуль связи с WiFi выключается до полной перезагрузки устройства. Индикация того, что идет 
установление соединения в рамках заданной в настройках WiFi сети - постоянное свечение синего LED, работа модуля в режиме точки доступа - мигание синего LED. 

-------- MQTT:
Для получения команд от MQTT устройство подписывается на топик команд. Его имя можно задать через параметры на WEB странице устройства. Если работает несколько устройств, они могут быть 
подписаны на один топик команд и тем самам быть объединены в группу. При этом синхронизации между устройствами по MQTT при управлении с лицевой панели не будет. Свой статус устройство 
сообщает в топики статуса. Возвращаемый статус устройства разделен на два топика - основное состояние и  вспомогательные данные. 
Формат данных в топике основного статуса идентичен формату команд в топике принимаемых команд, поэтому возможно включение модулей таким образом, что топик команд следующего устройства 
является топиком основного статуса предыдущего. Топик статуса последнего в цепочке может быть топиком команд первого. По сути создается кольцевая связь устройств.  Так как данные в отчет 
выводятся только при реальной смене состояния модуля, или по таймауту, поэтому такое включение будет синхронизировать модули между собой без возникновения "шторма" сообщений.  
Внешнее управление при этом можно осуществлять подавая команду в любой топик этой цепочки, либо с лицевой панели любого модуля.

Команды в топике команд:

  // команда управления конфигурацией
  {"clear_config"}                                          - очистить Flash память и загрузится с конфигурацией по умолчанию

  // основные команды управления  
  {"reset"}                                                 - перезагрузить контроллер управления усилителем 
  {"report"}                                                - сформировать отчет о текущем состоянии в топик REPORT  

  {"power":"on"|"off"}                                      - включить/выключить модуль
  {"input":"rca"|"xlr"}                                     - выбор входа для усилителя  
  {"vu_light": "off"|"on_low"|"on_middle"|"on_high"|"auto"} - режим работы подсветки VU индикатора 
  
  //         последние 3 команды возвращаются модулем в качестве текущего состояния модуля в основной топик статуса, поэтому могут являться командами для следующего модуля в цепочке

  // команды настройки параметров
  {"trigger_enable":"on"|"off"}                             - разрешить/запретить работу триггеров
  {"owb_sync":"on"|"off"}                                   - разрешение синхронизации по OneWireBUS
  {"bypass":"on"|"off"}                                     - разрешение прямой проброски триггерного сигнала с входа на выход
  {"light_manual": [<value1>,<value2>,<value3>]}            - значения PWM для подстройки яркости освещения в режимах "on_low","on_middle","on_high"
  {"light_auto": [<min_value>,<max_value>]}                 - значения PWM для подстройки границ изменения автоматической яркости
  {"ambient_sens": [<min_value>,<max_value>]}               - подстройка границ входного сигнала сенсора освещенности
