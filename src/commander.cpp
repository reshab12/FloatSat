#include "commander.hpp"

static HAL_UART ras_stm(UART_IDX1); // Rx: PB7 (PA10)  Tx: PB6 (PA9)
static LinkinterfaceUART link_name_not_imp_2(&ras_stm, 115200, 3, 10);
static Gateway gw_name_not_imp_2(&link_name_not_imp_2, true);

CommBuffer<position_data> cb_position_data_commander_thread;
Subscriber sub_position_data_commander_thread(topic_position_data, cb_position_data_commander_thread);

CommBuffer<satellite_mode> cb_satellite_mode_commander_thread;
Subscriber sub_satellite_mode_commander_thread(topic_satellite_mode, cb_satellite_mode_commander_thread);


Commander::Commander(int32_t priority):
        StaticThread<>("raspberryReceiverThread",priority),
    	Subscriber(topic_raspberry_receive, "raspberryReceiver") 
{

};

void Commander::init(){
        ras_stm.init(115200);
};

void Commander::run(){
    requested_conntrol requested_conntrol;
    position_data pose;
    satellite_mode mode;
        // 0=start; 1=waiting; 2=setNextStep;

        uint16_t number_of_pictures = 10;
        float heading = 0.0f;
    TIME_LOOP(0, 100 * MILLISECONDS)
    {
        cb_satellite_mode_commander_thread.get(mode);

        if((mode.mission_mode != mission_mode_star_mapper) && (status != -1))
            status = 3;//stop mission
        switch (status)
        {
        case -1:
            if(mode.mission_mode == mission_mode_star_mapper)
                status = 0;
            break;
        case 0://start     
            cb_position_data_commander_thread.get(pose);
            heading=pose.heading;

            requested_conntrol.requested_angle = heading;
            topic_requested_conntrol.publish(requested_conntrol);

            status = 1;
            break;

        case 1://check pose
            cb_position_data_commander_thread.get(pose);
            if(!pose.moving && (pose.heading-heading)<1){
                status = 10;
                //send command to raspberry:
                raspberry_command command;
                command.status = 1;
                topic_raspberry_command.publish(command);
            }

            break;

        case 2://set new reqested pose
            heading += 360/number_of_pictures;
            if(heading>180)
                heading-=360;
            else if(heading<-180)
                heading+=360;
            requested_conntrol.requested_angle = heading;
            topic_requested_conntrol.publish(requested_conntrol); 
            status = 1;
        case 3://stop
            //send command to raspberry:
            raspberry_command command;
            command.status = 0;
            topic_raspberry_command.publish(command);
            status = -1;
            break;
        case 4://map rady to be used
            mode.mission_mode = mission_mode_standby;
            topic_satellite_mode.publish(mode);
            status = -1;
            break;
        case 10://wait for picture
            break;

        default:
            break;
        }
    }
};

uint32_t Commander::put(const uint32_t topic_id, const size_t len, void *msg, const NetMsgInfo &) {
        raspberry_receive *received = (raspberry_receive *)msg;
        switch (received->status)
        {
        case 0://map rady to be used
            status = 4;
            break;
        
        case 1://picture made
            if(status=10)
                status=2;
            break;

        default:
            break;
        }
};
