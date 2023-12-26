// ---------------------------------------------------
//                      One Wire BUS
//            One Wire BUS without master device  
//                  (c) Dr.Cosha, 2023
//                    version 2.1.0
// ---------------------------------------------------

//
// Библиотека One Wire шины с раноправными участниками - без выделения MASTER устройства.
// По части аппартного обеспечения - идентична стандартной шине OneWire, по программной 
// части полностью свой протокол.
//
//
//

#include <OneWireBus.h> 
#include "driver/timer.h"
#include "soc/rtc_wdt.h"

#define DEFAULT_OWB_TIMER 0    		// номер базового таймера который мы используем - 0 (из 4-х доступных для ESP32 от 0 до 3)

// создаем сам объект ONE WIRE BUS по умолчанию не привязанный на шину и не имеющий собственного адреса
OneWireBusClass OneWireBus = OneWireBusClass();

// мьютекс для обслуживания обработки критических секций 
portMUX_TYPE OWB_Mux = portMUX_INITIALIZER_UNLOCKED;    	

// таймер для обслуживания шины
hw_timer_t *OWB_base_timer = NULL;                              						// базовый таймер шины 1Wire  

// ----- глобальные обработчики прерываний, на которые повешены методы объекта OneWireBus ---------------

void IRAM_ATTR onOWB_Timer(){
// ---- описание обработчика прерывания по сработке таймера ---	
	portENTER_CRITICAL_ISR(&OWB_Mux);            										// входим в критическую секцию обработчика прерывания
	OneWireBus.ISRTimerFire();															// вызываем метод класса, обрабатывающий прерывание по таймеру
	portEXIT_CRITICAL_ISR(&OWB_Mux);                       								// выходим из критической секции прерывания
}

void IRAM_ATTR onOWB_GPIO(){
// ---- описание обработчика прерывания по изменению уровня на pin-е шины OneWire ---	
	OneWireBus.ISRPinFire();  															// вызываем метод класса, обрабатывающий прерывание на ножке контроллера
}

// -------------------------------- методы объекта класса OneWireBusClass -------------------------------

void IRAM_ATTR OneWireBusClass::ISRTimerFire( void ) {
// --- метод обработки прерывания таймера в классе описания шины ---
bool		_cur_bit 		= true;														// временная переменная для хранения текущего состояния шины	
uint16_t 	tmpPacketCRC 	= 0;														// временная переменная для расчёта CRC пакета.  Считаем с учётом заголовка. 
uint8_t		tmpPacketToAddr	= 0;														// временная переменная для определения адреса получателя пакета

	switch (_mode) {
	case owbm_NotInitialize:
	    // для не инициализированного объекта - будет просто "холостой" цикл сработки таймера
        timerStop(OWB_base_timer);														// останавливаем таймер
		break;
    case owbm_StartSendPacket:
	    // режим нужен подготовки режима передачи и его синхронизации с таймером 
        _mode = owbm_SendPacket; 														// собственно переключение в режим передачи
		pinMode(_bus_pin,OUTPUT);														// шину в режим выхода
		digitalWrite(_bus_pin,HIGH); 													// начальное состояние - высокий уровень
		break;		
	case owbm_SendPacket:
	    // процесс передачи пакета
		digitalWrite(_bus_pin,!bitRead(buffer[_byteCounter],_bitCounter));   	// выставляем на шине бит
		_bitCounter++;																	// переходим к следующему биту
		if (_bitCounter>7) { 															// переходим к следующему байту
			_byteCounter++;											
			_bitCounter = 0;															// не забываем сбрасывать счётчик бит	
		}
        if ((_byteCounter>_CurPacketSize) && (_bitCounter>0)) {   				    	// за концом пакета есть один 0-й бит с выравниванием шины
          	_mode = owbm_SendComplete; 
		}
		break;		
	case owbm_SendComplete:
	    // окончание процесса передачи пакета
		digitalWrite(_bus_pin,HIGH);													// переводим GPIO в высокий уровень	
		pinMode(_bus_pin,INPUT);														// переключаем пин в режим слушателя
		_bitCounter = 0;															    // сбрасываем счётчики приема/передачи - бит
		_byteCounter = 0;															    // -\\- байт
		_mode = owbm_Listen;															// переходим в режим прослушки
		break;		
	case owbm_StartRecievePacket:
		// в этой точке мы таймер синхронизирован с входным импульсом и сдвинут относительно него для чтения данных
        // читаем заголовок и проверяем его валидность
		if (_bitCounter == 1) {															// в самом начале - выравниваем такт
			timerStop(OWB_base_timer);
			timerAlarmWrite(OWB_base_timer,_bus_speed,true);							// задержка опять = скорости
        	timerRestart(OWB_base_timer);
			timerStart(OWB_base_timer);
		}
		_cur_bit = !digitalRead(_bus_pin);	
		if ((_bitCounter == 1) && _cur_bit) {  // второй бит заголовка должен быть 1
        	_bitCounter++;
			break;
		}
		if ((_bitCounter == 2) && !_cur_bit) {  // третий бит заголовка должен быть 0
        	_bitCounter++;
			break;
		}
        if ((_bitCounter == 3) && _cur_bit) {  // четвертый бит заголовка должен быть 1
        	_bitCounter++;
            _mode = owbm_RecievePacket;  												// переключаемся в режим приема основного пакета
			buffer[0] = 0b00001011;			
			break;
		}
        // если дошли до этого места, с заголовком что то не и прием пакета можно прекращать
        _mode = owbm_Listen;
	    break;	
	case owbm_RecievePacket:
        // получаем данные пакета начиная с 4-го бита 0-го байта и далее
		_cur_bit = !digitalRead(_bus_pin);	        									// читаем бит из GPIO
        bitWrite(buffer[_byteCounter],_bitCounter,_cur_bit);				 			// пишем бит в буфер
		_bitCounter++;																	// инкрементируем счётчик битов
		if (_bitCounter > 7) {  // переполнение счётчика битов
			_bitCounter = 0;    														// сбрасывем счётчик битов
			_byteCounter++;																// инкременируем счётчик байтов
			// по окончанию третьего байта инициализируем счётчик чтения
			if (_byteCounter == 3) {
				_CurPacketSize = buffer[2]+PACKET_OVERHEAD;								// присваиваем длину пакета исходя из размера данных пакета и служебной части
				if (_CurPacketSize>buffer_len) { _CurPacketSize=buffer_len; }			// если не влезает в наш буфер, размер пакета не больше буфера
			}		
		}
        if (_byteCounter > _CurPacketSize) {   // конец чтения пакета по достижении текущего размера
           _mode = owbm_PacketRecieved;													// переключаемся в режим принятого пакет      
		   _phase = aph_CheckCRC; 
		}
        break;
	case owbm_PacketRecieved:
	   	// получен пакет данных, прием приостановлен, нужен анализ полученного пакета нужно сделать действия:
		// 1. Пакет является целым - CRC сошлась.
		// 2. Пакет предназначен этому устройству или это броадкаст или стоит флаг получения всех пакетов
		// 3. Если 1 и 2 = Ок, то проверяем наличие предыдущего пакета. Если он есть, увеличиваем счётчик потерянных пакетов
		// 4. Если выходной буфер пуст, или флаг перезаписи взведен - копируем пакет в буфер хранения
		// обработку ведем по отдельным фазам для уменьшения времени нахождения в критической секции обработки прерывания
        switch (_phase)	{
		case aph_CheckCRC:		// проверяем CRC пакета, если Ок - идем дальше. Иначе сразу уходим в переключение в режим прослушки
          	tmpPacketCRC = GetCrc16Simple(buffer,_CurPacketSize-3);						// считаем CRC для пакета данных
            if ((buffer[_CurPacketSize-3] == highByte(tmpPacketCRC)) and 				// проверяем совпадение CRC - старшего и
				(buffer[_CurPacketSize-2] = lowByte(tmpPacketCRC))) {					// младшего байта
		    	_phase = aph_CheckAddress;    											// если ок - переходим в следующую фазу - проверку адреса получателя
			} else _phase = aph_Switch2Listen;
			break;		
	    case aph_CheckAddress:  // здесь проверяем адрес и понимаем, наш ли это пакет
			// определяем адрес получателя пакета
			tmpPacketToAddr = buffer[0] >> 4;											// старшая тетрада - это адрес получателя пакета
            if ((tmpPacketToAddr == _self_addr) or 										// если адрес получателя совпал с нашим
				 recieve_all or															// или стоит флаг читать все пакеты
				(tmpPacketToAddr == BROADCAST_ADDR)) {  								// или это broadcast пакет
				_phase = aph_Copy2PktBuf;												// то переходим к шагу копирования его в буфер полученных данных
			} else _phase = aph_Switch2Listen;											// иначе просто идем на шаг перехода к ожиданию следующего пакета
			break;
		case aph_Copy2PktBuf:  	// здесь копируем полученный пакет в буфер данных и если нужно увеличиваем счетчик потерянных пакетов
			// проверяем, что у нас есть уже данные в буфере 
            if (bitRead(packet_buffer[0],0)) {											// если у нас есть уже пакет полученный ранее (есть 1 в младшем бите нулевого байта)			
				_drop_pkt_count++;														// то в любом случае мы потеряем один пакет			
				if (!overwrite_packet) {  												// если не стоит флаг перезаписи, то копировать не нужно
					_phase = aph_Switch2Listen;											// ставим следующую фазу	
					break;																// заканчиваем текущую фазу
				}																		
			} 
			// если дошли сюда, то в собственно копируем пакет с данными из оперативного буфера в буфер полученных данных
			memcpy((void*)&packet_buffer[0],(void*)&buffer[0],_CurPacketSize);  		// копируем данные размером с пакет передачи
			_phase = aph_Switch2Listen;
			break;
		case aph_Switch2Listen: // переключаемся в режим прослушки
		    _mode = owbm_Listen;														// режим - прослушка шины
			_phase = aph_Nop;															// фаза по умолчанию - до следующего пакета
			break;
		case aph_Nop:			// здесь ничего не делаем		
		default:				// как и здесь
			break;
		}  
        break;
	case owbm_Listen:
	    // мы находимся в режиме прослушки шины в нем есть 
		// тики таймера и можно обрабатывать дополнительную логику
		break;
	default:
		break;
	}	  
}

void IRAM_ATTR OneWireBusClass::ISRPinFire( void ) {                                     	
// --- метод обработки прерывания по GPIO в классе описания шины ---
    // обработчик очень компактный и не делает ничего кроме переключения режима обработки цикла таймера
	// начинаем чтение пакета только в режиме Listen
	if (_mode == owbm_Listen) {
	 	// синхронизируем следующий импульс по таймеру + добавляем выравнивание на середину импульса		
		timerStop(OWB_base_timer);
		timerAlarmWrite(OWB_base_timer,_bus_speed + 4,true);							// сдвижка такта на 4uS
        timerRestart(OWB_base_timer);
		timerStart(OWB_base_timer);
		// переходим в режим начала чтения пакета данных
		_mode = owbm_StartRecievePacket;												// переходим в режим старта приема пакета	
		_bitCounter = 1;															    // предустанавливаем счётчики входного буфера - битовый
		_byteCounter = 0;																//	-\\- - байтовый
		_phase = aph_Nop;																// фаза обработки пакета - ничего не делаем		
		_CurPacketSize = PACKET_OVERHEAD;                                               // размер пакета по умолчанию равен служебной части пакета
	}	
}

OneWireBusClass::OneWireBusClass() {  
// --- конструктор класса OneWireBusClass ---
	_mode = owbm_NotInitialize; 														// состояние шины - не проинициализирована
	_self_addr = BROADCAST_ADDR;														// адрес по умолчанию - broadcast	
	_bus_pin = OWB_NO_PIN;																// по умолчанию PIN для шины не назначен	
	OWB_base_timer = timerBegin(DEFAULT_OWB_TIMER, 80, true);         					// инициализируем таймер базового цикла
    timerAttachInterrupt(OWB_base_timer, &onOWB_Timer, true);							// привязываем процедуру - обработчик прерывания
    timerAlarmWrite(OWB_base_timer, OWB_DEFAULT_SPEED, false);				        	// загружаем в таймер базовый цикл по умолчанию и запускаем 	
    timerAlarmEnable(OWB_base_timer);      												// разрешаем прерывание таймера
}

OneWireBusClass::~OneWireBusClass() {  
// --- деструктор класса OneWireBusClass ---
	portENTER_CRITICAL_ISR(&OWB_Mux);            										// входим в критическую секцию изменений
    timerDetachInterrupt(OWB_base_timer);												// отключаем обработчик прерываний таймера - метод класса
	if (_bus_pin != OWB_NO_PIN) { detachInterrupt(_bus_pin); }							// отключаем обработчик на GPIO - метод класса
	portEXIT_CRITICAL_ISR(&OWB_Mux);                       								// выходим из критической секции изменений		
	if ( buffer != nullptr ) {												       		// освобождаем память из-под операционного буфера данных
		free(buffer);
		buffer_len = 0;
	}
	if ( packet_buffer != nullptr ) {													// освобождаем память из-под буфера принятого пакета данных
		free(packet_buffer);
		packet_buf_len = 0;
	}
}


bool OneWireBusClass::InitializeBus(int8_t t_PIN, uint8_t t_addr, uint32_t  t_bus_speed, uint8_t t_buf_len) {      
// --- инициализация шины OneWire на нужный pin ввода/вывода c нужной скоростью и заданным собственным адресом устройства
	// 0. увеличиваем период работы WDT
	rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
	rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
	rtc_wdt_set_time(RTC_WDT_STAGE0, 250);	
	// 1. Останавливаем все обработчики, запрещаем прерывания	
	portENTER_CRITICAL_ISR(&OWB_Mux);            										// входим в критическую секцию изменений
	timerStop(OWB_base_timer); 															// останавливаем таймер
	timerAlarmDisable(OWB_base_timer);													// запрещаем обработчик таймера	
	if (_bus_pin!=OWB_NO_PIN) { // если GPIO уже был назначен
		detachInterrupt(_bus_pin);														// отключаем прерывание по GPIO
		pinMode(_bus_pin,INPUT);														// переключаемся в режим слушателя на GPIO
	  } 
	portEXIT_CRITICAL_ISR(&OWB_Mux);                       								// выходим из критической секции изменений	
    // 2. Назначаем свойства, которые не влияют на прерывания 
	_drop_pkt_count = 0;																// обнуляем счётчик потерянных пакетов
    _mode = owbm_NotInitialize;															// переводим объект в режим - не инициализирован	
    _self_addr = t_addr;																// назначаем адрес устройства	
	// выделяем буфера приема/передачи для сообщений 	
	if (((t_buf_len << 1) + (PACKET_OVERHEAD << 1)) > ESP.getFreeHeap()) {				// проверяем, что у нас есть место под операционный и выходной буфера	
	    log_e("Not enough memory for I/O buffer for OneWireBus.");			
		return false;																	// возвращаем ошибку инициализации если памяти нет
	}	
	// устанавливаем скорость шины (скорость - указывается как величина задержки цикла таймера, поэтому чем меньше тем быстрее )
	_bus_speed = t_bus_speed; 															// устанавливаем скорость и далее корректируем если выходит за пределы  
	if (t_bus_speed>OWB_MIN_SPEED) { _bus_speed = OWB_MIN_SPEED; } 						// если указана слишком низкая скорость, ставим ее не ниже минимальной
	if (t_bus_speed<OWB_MAX_SPEED) { _bus_speed = OWB_MAX_SPEED; } 						// если указана слишком высокая скорость, ставим ее не выше максимальной
	// выделяем место под операционный буфер сообщений
	buffer_len = t_buf_len+PACKET_OVERHEAD;  											// добавляем размерность данных необходимых для построения транспортного пакета и назначаем размер операционного буфера
	buffer = (uint8_t *) malloc(buffer_len);           									// выделяем память под операционный буфер приема/передачи
	// выделяем место под буфер хранения принятого сообщения
	packet_buf_len = t_buf_len+PACKET_OVERHEAD;											// добавляем размерность данных необходимых для построения транспортного пакета и назначаем размер буфера хранения
	packet_buffer = (uint8_t *) malloc(packet_buf_len); 								// выделяем память под буфер хранения принятых данных
	packet_buffer[0] = 0;																// обнуляем младший байт буфера - признак наличия пакета	
	// обнуляем счётчик байт и счётчик бит для ядра приемо/передатчика
    _bitCounter = 0;	
	_byteCounter = 0;
	_CurPacketSize = 0;
	// 3. Назначаем свойства, которые влияют на прерывания и обработчики
    _bus_pin = t_PIN;                                                           		// присваиваем GPIO для шины OWB
	// 4. Обнуляем все свойства и стартуем обработчики шины
    if (_bus_pin!=OWB_NO_PIN) {       // если GPIO для шины назначен, то шину можно переключить в режим слушателя
		// переходим в режим прослушки GPIO
		digitalWrite(_bus_pin,HIGH);													// для первого вызова GPIO может быть не инициализирован	
		pinMode(_bus_pin,INPUT);														// включаем GPIO как вход 
		_mode = owbm_Listen;															// включаем режим слушателя на шине		
		attachInterrupt(_bus_pin,&onOWB_GPIO,FALLING);									// назначаем прерывание на GPIO по ниспадающему фронту
	} else {
		// иначе остаемся в режиме owbm_NotInitialize - и возвращаем ошибку инициализации по GPIO
        log_e("GPIO must be defined for OneWireBus.");		
		return false;
	}
    timerAlarmWrite(OWB_base_timer, _bus_speed, true);				        			// загружаем в таймер стартовый цикл и запускаем на повторение
    timerAlarmEnable(OWB_base_timer);      												// а потом разрешаем прерывание таймера    
	timerStart(OWB_base_timer);                                                         // запускаем таймер        
	return true;
}


bool OneWireBusClass::HasData() {
// --- функция возвращает true если в буфере есть полученные данные
	return bitRead(packet_buffer[0],0);													// если младший бит нулевого байта в буфере не пуст - есть полученный пакет
}

bool OneWireBusClass::SendData(uint8_t _to_addr, uint8_t *data, uint8_t len) {
// --- функция асинхронного запуска передачи данных  (кому, что, сколько)    

	uint16_t tmpPacketCRC = 0;															// временная переменная для расчёта CRC пакета.  Считаем с учётом заголовка. 

	if (!ReadyToSend()) return false;													// первым делом проверяем, не занята ли у нас шина

	// проверяем достаточность размеров буферов для передачи
	if ((len+PACKET_OVERHEAD)>buffer_len) return false;									// буфер отправки слишком мал для передачи 
	portENTER_CRITICAL_ISR(&OWB_Mux);            								    	// входим в критическую секцию изменений	

    // готовим буфер для передачи данных передача идет от младшего бита к старшему в инверсии (0 на шине это высокий уровень), поэтому 1011 - это стартовая последовательность 0010
	buffer[0] = 0b00001011 + (_to_addr << 4);											// заголовок и адрес  получателя													
	// синхронизация и адрес отправителя									
	buffer[1] = 0b01000001 + (_self_addr << 2);											// указываем собственный адрес в пакете и синхро импульсы
    // размер пакета
	buffer[2] = len;																	// указываем размер пакета

	// собственно копируем данные в буфер
	memcpy((void*)&buffer[3],data,len);  												// копируем данные в пакет с размером указанным для передачи
    // присваиваем контрольную сумму последнему байту пакета
    tmpPacketCRC = GetCrc16Simple(buffer,len+PACKET_OVERHEAD-3);						// считаем CRC для пакета данных

    buffer[len+PACKET_OVERHEAD-3] = highByte(tmpPacketCRC);								// старший байт CRC - предпоследний байт пакета
	buffer[len+PACKET_OVERHEAD-2] = lowByte(tmpPacketCRC);								// младший байт CRC - последний байт пакета
	buffer[len+PACKET_OVERHEAD-1] = 0b00000000;											// завершаем пакет нулевой посылкой	- нужен для установки последнего бита в 0

	// обнуляем счётчик байт и счётчик бит для ядра приемо/передатчика
    _bitCounter = 0;																	// счётчик бит
	_byteCounter = 0;																	// счётчик байт
	_CurPacketSize = len+PACKET_OVERHEAD-2;												// текущий размер буфера передачи - размер текущего пакета

	// собственно переводим шину в режим передачи
	_mode = owbm_StartSendPacket;
	portEXIT_CRITICAL_ISR(&OWB_Mux);                       								// выходим из критической секции изменений	
	return true;																		// посылка начата
}

bool OneWireBusClass::GetData(uint8_t *data, uint8_t maxlen) {
// --- функция чтения полученных данных	
uint8_t 	_size_of_data	= 0;														// размер данных для передачи
    if (HasData()) {
		portENTER_CRITICAL_ISR(&OWB_Mux);            									// входим в критическую секцию изменений
		// проверяем, сколько данных мы можем передать - выбираем меньшее из 
		// размера буфера, который получен по OWB и maxlen
		_size_of_data = packet_buffer[2];		
		if (maxlen < _size_of_data) { _size_of_data = maxlen; }							// определяем минимальный размер данных для передачи
		// собственно копируем данные из буфера
	    memcpy(data,(void*)&packet_buffer[3],_size_of_data);    						// копируем данные в пакет с размером указанным для передачи
		packet_buffer[0] = 0;															// чистим буфер полученного пакета
		portEXIT_CRITICAL_ISR(&OWB_Mux);                       							// выходим из критической секции изменений	    		
		return true;																	// данные получены	
	} else return false;																// если данных нет - возвращаем FALSE	
}

bool OneWireBusClass::ReadyToSend() {
// --- возвращаем состоянии шины - занята она или нет
    if ((_mode != owbm_Listen)) return false;   // если мы в режиме слушателя, то шина свободна для передачи
	return true;
}	

uint16_t OneWireBusClass::GetCrc16Simple( uint8_t * data, uint16_t len ) {
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

OWB_Mode_t OneWireBusClass::GetCurrentMode() {                           
// --- возвращаем текущий режим работы шины
   return _mode;
}

void OneWireBusClass::SetRecievePolitic(bool t_rcv_all, bool t_overwrite_pkt) {
// задание политики работы шины - принимать все сообщения/только свои, перезаписывать сообщения новыми/хранить до чтения
	recieve_all = t_rcv_all;                                     						// принимать все пакеты, а не только адресованные этому устройству
	overwrite_packet = t_overwrite_pkt;                          						// перетирать ранее полученные пакеты новыми или ожидать их вычитки
}

uint16_t OneWireBusClass::GetDropPktCount() {
// вернуть количество потерянных пакетов с момента начала работы/последнего обращения к функции                               
uint16_t	_t;
    _t = _drop_pkt_count;																// просто возвращаем счетчик потерянных пакетов 
	_drop_pkt_count = 0;																// и обнуляем его
	return _t;
}
