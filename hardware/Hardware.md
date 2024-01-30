## Amplifiler control unit - hardware parts

В данной части будут приведены схемы и печатные платы, касающиеся модуля управления и его исполнительных и контрольных узлов.
Схема самого усилителя, основного блока питания и индикации остается для самостоятельной реализации.

## Cхема модуля управления

<div align="center"><img alt="Overview" width="800" src="/images/ACU_main_circuit_short.png" />&emsp;&emsp;&emsp;</div>

Общий вид 3D модели платы:

<div align="center"><img alt="Overview" width="600" src="/images/3D_acu_pcb_view.png" />&emsp;&emsp;&emsp;</div>

Собранная плата выглядит следующим образом:

<div align="center"><img alt="Overview" width="600" src="/images/acu_assembled_view.png" />&emsp;&emsp;&emsp;</div>


Основные сигналы модуля выведены на следующие разъемы:

|Номер разъема|Назначение|Распиновка|
|-------------|----------|----------|
|JP1|`TRIG_IN` - Trigger IN - входящий сигнал внешнего включения. Уровень сигнала +5..12В. Потребляемый ток до 20mA. | 1-**GND**; 2-**NC**; 3-**trig_in**|
|JP2|`TRIG_OUT` - Trigger OUT - исходящий сигнал для включения дочерних устройств. +12В, нагрузка до 100mA| 1-**GND**; 2-**NC**; 3-**trig_out**|
|JP3|`1Wire` - Шина OneWire - однопроводная шина Peer2Peer для синхронизации модулей между собой. До 10 устройств на одной шине.| 1-**GND**; 2-**NC**; 3-**1Wire**|
|N1|`VU_Mute` - Сигнал выключения модуля стрелочной индикации в моменты включения/выключения усилителя для подавления переходных процессов. [^1]| 1-**VU_Mute**; 2-**GND**|
|N3|`LED_PWM` - Питание подсветки стрелочного индикатора. PWM сигнал напряжением +12V. Управление по шине GND.| 1-**+12V**; 2-**PWM**|  
|N4|`SELECTOR` - Сигнал управления реле с обмоткой расчитанной на 5V. Переключение реле выбирает XLR или RCA вход.| 1-**+5V**; 2-**SELECT**|  

[^1]: Внимание: Сигнал является прымым выходом GPIO микроконтроллера и не должен нагружатся более чем на 20mA!

## Модуль питания и основное силовое реле

<div align="center"><img alt="Overview" width="800" src="/images/p&r_circuits_short.png" />&emsp;&emsp;&emsp;</div>

## Схема платы индикации и включения, а так же платы селектора и датчика освещенности

<div align="center"><img height="250" src="/images/power_panel_short.png"/>&emsp; <img height="250" src="/images/selector_panel_short.png"/></div>




Общее описание и настройки здесь: [README.md](https://github.com/DrCosha/AMP-Mono-module/blob/master/README.md)
