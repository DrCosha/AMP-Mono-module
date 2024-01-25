# Модуль управления оконечным усилителем
## Amplifiler control unit

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

<div align="center"><img alt="Overview" width="800" src="/images/ACU_main_circuit_short.png" />&emsp;&emsp;&emsp;</div>
<div align="center"><img height="250" src="/images/3D_acu_pcb_view.png"/>&emsp; <img height="250" src="/images/amp_main_view.png"/></div>

## Способы управления
Модуль управления моноблока получает команды от следующих устройств и блоков:

**Лицевая панель управления:**
- кнопка Power на лицевой панели;
- кнопка Input на лицевой панели;
- кнопка VU light на лицевой панели;
- датчик освещенности 

**Входы/выходы управления на задней панели:**
- вход `TriggerIN` - подача питания от __+5..12V__ на вход __(5-15mA)__, приводит к сработке входного триггера. На входе есть задержка от дребезга контактов, а также
  задержка выключения устройства для подавления переходных процессов при включении/выключении внешнего управления и переключении входов на внешних устройствах.
- выход `TriggerOUT` - работает в режиме подачи сигнала __+12V (до 100mA)__ при включении модуля, либо в режиме проброски сигнала с `TriggerIN` на `TriggerOUT`.
- шина OneWireBUS - подключение к общей шине управления для приема/передачи команд по `OneWire`. Однопроводная шина с подключением до 254 блоков параллельно
  для синхронизации сигналов управления. Все блоки могут асинхронно передавать данные в режиме __peer2peer__ (все-всем). На шине нет устройства "мастера", все равнозначны.

**Управление через WiFi:**
- __сервер MQTT__ - подключение к топикам сервера для возврата статуса и получения команд.
- __WEB сервер__ со страницей настроек доступа к __WiFi__ и __MQTT__. Страница доступна по IP адресу подключения модуля. 


## Коммуникация
Моноблок усилителя раблотает в комплексе с аналогичными, поэтому общается с ними следующим образом:

**OneWire BUS:**

Блоки работают на общей шине по принципу равных. Необходимость в синхронизации блоков и как следствие - передачи данных по шине 
возникает только при воздействии с внешних источников. Передача параметров по шине `OneWire` и их применение не приводит к повторной передаче данных по `OneWire`. По шине идет передача только
статуса включения/выключения блока, выбора источника и установленного режима подсветки. Т.е. синхронизация базового управления.

**WiFi:**

Подключение к __WiFi__ проводится с заранее установленными параметрами.  Если подключение успешно, устанавливается соединение с __MQTT__ сервером. При первичной настройке или не доступности __WiFi__ 
сети или __MQTT__ сервера, поднимается собственная точка доступа с именем ___HiAMP_xxxx___ - где ___xxxx___ - это 4 последних цифры MAC адреса. Без пароля по умолчанию.  При этом, если есть соединение 
с __WiFi__, но нет соединения с __MQTT__, идут попытки установки соединения с ним в рамках заданной сети. Если это не происходит за 10 повторов, то повторно поднимается собственная точка доступа 
с возможностью подключения к ней и изменения настроек. Если никто не соединяется с этой точкой в течение 3 минут, то цикл попытки установки __WiFi__ соединения и соединения c __MQTT__ повторяется. 
После __3-х__ полных циклов, если нет успешного соединения с __WiFi__ и/или __MQTT__ сервером в сети, модуль связи с __WiFi__ выключается до полной перезагрузки устройства. Индикация того, что идет 
установление соединения в рамках заданной в настройках __WiFi__ сети - постоянное свечение синего ___LED___, работа модуля в режиме точки доступа - мигание синего ___LED___. 

**MQTT:**

Для получения команд от __MQTT__ устройство подписывается на топик команд. Его имя можно задать через параметры на WEB странице устройства. Если работает несколько устройств, они могут быть 
подписаны на один топик команд и тем самам быть объединены в группу. При этом синхронизации между устройствами по __MQTT__ при управлении с лицевой панели не будет. Свой статус устройство 
сообщает в топики статуса. Возвращаемый статус устройства разделен на два топика - основное состояние и  вспомогательные данные.
! Ниже будет описан способ синхронизации устройств через MQTT.

## Управление через MQTT

**Команды в топике команд:**

Топик команд описывается в конфигурации как топик __[SET]__.  Ниже приведена таблица команд, которые можно в нем применять:

| Команда | Описание |
|---------|----------|
|{"clear_config"}| очистить Flash память и загрузится с конфигурацией по умолчанию|
|{"reset"}| перезагрузить контроллер управления усилителем|
|{"report"}| сформировать отчет о текущем состоянии в топик REPORT|
|{"power":"___xxx___"}|включить модуль - "___on___" / выключить модуль - "___off___"|
|{"input":"___nnn___"}|выбор входа для усилителя "___rca___" или "___xlr___"|
|{"vu_light": "___aaaaaaa___"}|режим работы подсветки VU индикатора: "___off___","___on_low___","___on_middle___","___on_high___","___auto___"|
|{"trigger_enable":"___xxx___"|разрешить - "___on___" / запретить работу триггеров - "___off___"|
|{"owb_sync":"___xxx___"}|разрешение синхронизации по OneWireBUS "___on___" / "___off___"|
|{"bypass":"___xxx___"}|разрешение прямой проброски триггерного сигнала с входа на выход - "___on___" / "___off___"|
|{"light_manual": [___n1,n2,n3___]}|значения PWM для указания яркости освещения в режимах соответсвенно в режимах "___on_low___","___on_middle___","___on_high___" (возможный диапазон __60..2000__)|
|{"light_auto": [___min_value___,___max_value___]}|значения PWM для подстройки границ изменения автоматической яркости (возможный диапазон __60..2000__)|
|{"ambient_sens": [___min_value___,___max_value___]}|подстройка границ входного сигнала сенсора освещенности (возможный диапазон __0..4000__)|

Команды в топике команд могут комбинироваться в единый JSON. Пример:

```

{"power":"on","input":"rca","bypass":"off","trigger_enable":"off"}

```

**Топики статуса**

Как уже упоминалось, для получения статуса устройства используются два топика:
- __[STATE]__ - в который выводится основное состояние;
- __[MISC]__ - для вспомогательные данных и настроек.

Ниже приведены примеры данных, возвращаемых в топиках:

**Основное состояние**:
```
{"power":"on","input":"rca","vu_light":"on_middle"}
```

Формат данных в топике основного статуса и их набор идентичны формату команд в топике принимаемых команд, поэтому возможно включение модулей таким образом, что топик команд следующего устройства 
является топиком основного статуса предыдущего. Топик статуса последнего в цепочке может быть топиком команд первого. По сути создается кольцевая связь устройств.  Так как данные в отчет 
выводятся только при реальной смене состояния модуля, или по таймауту, то такое включение будет синхронизировать модули между собой без возникновения "шторма" сообщений.  
Внешнее управление при этом можно осуществлять подавая команду в любой топик этой цепочки, либо с лицевой панели любого модуля.


**Вспомогательные данные и настройки**:
```
{"vu_brightness":0,"ambient":10,"trigger_in":"on","trigger_enable":"off","trigger_out":"off","bypass":"off","owb_sync":"on","light_manual":[62,80,500],"light_auto":[70,500],"ambient_sens":[10,200]}
```

Данные в этом топике отчасти совпадают с настройками, которые можно задать через топик команд, отчасти содержат текущие данные модуля управления. Ниже приведена таблица с тегами и описанием по 
этим текущим данным:

| Тег | Описание |
|---------|----------|
|"vu_brightness":___nnn___| ___nnn___ текущее значение PWM для текущего уровня подсветки (может быть вычислено в режиме __"auto"__)|
|"ambient":___mmm___| ___mmm___ текущее значение сенсора освещенности скорректированное по границам, заданным в параметре __"ambient_sens"__ |
|"trigger_in":"___xxx___"| "___xxx___" = "on"/"off" - текущее значение триггерного входа `IN` |
|"trigger_out":"___xxx___"| "___xxx___" = "on"/"off" - текущее значение триггерного входа `OUT` |



