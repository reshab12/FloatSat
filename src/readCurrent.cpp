#include "readCurrent.hpp"

HAL_ADC mainCurrent(ADC_IDX2);
HAL_ADC voltage(ADC_IDX1);

void initADCPins(){
    //mainCurrent.init(ADC_CH_000);
    mainCurrent.init(ADC_CH_004);
    mainCurrent.init(ADC_CH_010);
    mainCurrent.init(ADC_CH_002);
    voltage.init(ADC_CH_012);
}

void readADCPins(additional_sensor_data* data){
    float boardVoltage = 0;
    uint16_t voltageADC = voltage.read(ADC_CH_012);
    data->boardVoltage = (voltageADC/ADCRes) * ADCRef;
    data->batterieVoltage = data->boardVoltage/0.108712121;
    AT(NOW() + 3 * MILLISECONDS);

    uint16_t motorADCValue = mainCurrent.read(ADC_CH_004);
	data->motorCurrent = (((motorADCValue / ADCRes) * ADCRef))/ NewCurrentVoltage;
    AT(NOW() + 3 * MILLISECONDS);

    uint16_t magADCValue = mainCurrent.read(ADC_CH_000);
	data->magTorquerCurrent = ((magADCValue / ADCRes) * ADCRef );//-2.5)/ CurrentVoltageRatio;
    AT(NOW() + 10*MILLISECONDS);

    uint16_t boardADCValue = mainCurrent.read(ADC_CH_010);
	data->boardCurrent = ((boardADCValue / ADCRes) * ADCRef -2.5) /CurrentVoltageRatio;
    AT(NOW() + 3 * MILLISECONDS);

    readSolarPanel(data->solarPanel);

    //data->allCurrent = motorCurrent + magCurrent + boardCurrent;
}

ReadADCPins::ReadADCPins(const char* name, int32_t priority):Subscriber(topic_telecommand_uplink,name),StaticThread(name, priority),safetyPin(GPIO_062){}

void ReadADCPins::init(){
    initializeMotor();
    safetyPin.init(true, 1,0);
    initADCPins();
}

void ReadADCPins::run(){
    safetyPin.setPins(0);
    motor_control_value motor;
    additional_sensor_data data;
    int64_t last_time = NOW();
    int64_t integ_currents = 0; // in As e-6
    bool safetyPinOn = false;
    RingBuffer<additional_sensor_data,1> ringbuffer;
    TIME_LOOP(0, 500 * MILLISECONDS){
        readADCPins(&data);
        safetyPinMsgBuffer.getOnlyIfNewData(safetyPinOn);
        ringbuffer.add(data);

        data.batterieVoltage = 0;
        data.boardCurrent = 0;
        data.boardVoltage = 0;
        data.magTorquerCurrent = 0;
        data.motorCurrent = 0;
        for(int i = 0; i<ringbuffer.getNumElements();i++){
            data.batterieVoltage += ringbuffer.getElement(i).batterieVoltage;
            data.boardCurrent += ringbuffer.getElement(i).boardCurrent;
            data.boardVoltage += ringbuffer.getElement(i).boardVoltage;
            data.magTorquerCurrent += ringbuffer.getElement(i).magTorquerCurrent;
            data.motorCurrent += ringbuffer.getElement(i).motorCurrent;
        }
        data.batterieVoltage = data.batterieVoltage/ringbuffer.getNumElements();
        data.boardCurrent = data.boardCurrent/ringbuffer.getNumElements();
        data.boardVoltage = data.boardVoltage/ringbuffer.getNumElements();
        data.magTorquerCurrent = data.magTorquerCurrent/ringbuffer.getNumElements();
        data.motorCurrent = data.motorCurrent/ringbuffer.getNumElements();
        

        int64_t time = NOW();
        integ_currents += (data.boardCurrent + data.magTorquerCurrent + data.motorCurrent) * 1000000.0 * (time - last_time) * 1.0 / SECONDS;
        last_time = time;
        data.allCurrent = integ_currents / 3600000.0;
        topic_additional_sensor_data.publish(data);
        
        if(NOW() > 1 * HOURS || safetyPinOn)
             safetyPin.setPins(1);   
    }
}

uint32_t ReadADCPins::put(const uint32_t topicId, const size_t len, void *data, [[gnu::unused]] const NetMsgInfo &netMsgInfo) {
    if ((*(telecommand *) data).command_id == command_id_safetyPin){
        safetyPinMsgBuffer.put(true);
    }
    return 1;
}
