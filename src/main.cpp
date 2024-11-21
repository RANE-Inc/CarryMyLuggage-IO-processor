#include <Arduino.h>

#include "cobs.h"
#include <pb_decode.h>
#include <pb_encode.h>

#include "wheel.h"
#include "estop.h"
#include "wheel_commands.pb.h"
#include "wheel_states.pb.h"

#define COM_SERIAL Serial1
#define DEBUG_SERIAL Serial

uint8_t incoming_byte;
uint8_t incoming_buffer[256], outgoing_buffer[256]/*, msg_buffer[256]*/;
uint8_t msg_commands_buffer[256], msg_states_buffer[256];
uint16_t incoming_buffer_len;

carry_my_luggage_WheelCommands msg_commands;
pb_istream_t istream;
cobs_decode_result decode_result;

carry_my_luggage_WheelStates msg_states;
pb_ostream_t ostream;
cobs_encode_result encode_result;

bool initComplete;

mutex_t encoder_mutex{0}; 

Wheel wheel_L(17, 16, 18, 10, 11);
Wheel wheel_R(14, 15, 13, 20, 21);
EStop estop(19);


void printWheelCommands() {
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("Wheel commands: ");
    DEBUG_SERIAL.print(msg_commands.velocity_left);
    DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.println(msg_commands.velocity_right);
}

void printWheelStates() {
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print("Wheel positions: ");
    DEBUG_SERIAL.print(msg_states.position_left);
    DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.println(msg_states.position_right);
    DEBUG_SERIAL.print("Wheel velocities: ");
    DEBUG_SERIAL.print(msg_states.velocity_left);
    DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.println(msg_states.velocity_right);
}

void printBufferDec(uint8_t* buf, size_t len) {
    for(size_t i = 0; i < len-1; i++) {
        DEBUG_SERIAL.print(buf[i], DEC);
        DEBUG_SERIAL.print(", ");
    }
    if(len >= 1) DEBUG_SERIAL.println(buf[len-1], DEC);

    DEBUG_SERIAL.println();
}

void printBufferHex(uint8_t* buf, size_t len) {
    for(size_t i = 0; i < len-1; i++) {
        if(buf[i] < 16) DEBUG_SERIAL.print("0");
        DEBUG_SERIAL.print(buf[i], HEX);
        DEBUG_SERIAL.print(" ");
    }
    if(len >= 1) {
        if(buf[len-1] < 16) DEBUG_SERIAL.print("0");
        DEBUG_SERIAL.println(buf[len-1], HEX);
    }

    DEBUG_SERIAL.println();
}

void readCommands() {
    while(COM_SERIAL.available()){
        incoming_byte = COM_SERIAL.read();

        if(incoming_byte == '\0') {
            if(incoming_buffer_len == 0) break;

            // printBufferHex(incoming_buffer, incoming_buffer_len);

            // decode_result = cobs_decode(msg_buffer, 256, incoming_buffer, incoming_buffer_len & 0x00FF);
            decode_result = cobs_decode(msg_commands_buffer, 256, incoming_buffer, incoming_buffer_len & 0x00FF);
                
            if(decode_result.status == cobs_decode_status::COBS_DECODE_OK) {
                // istream = pb_istream_from_buffer(msg_buffer, decode_result.out_len);
                istream = pb_istream_from_buffer(msg_commands_buffer, decode_result.out_len);

                if(pb_decode(&istream, carry_my_luggage_WheelCommands_fields, &msg_commands)) {
                    wheel_L.update(msg_commands.velocity_left);
                    wheel_R.update(msg_commands.velocity_right);

                    printWheelCommands();                
                }
                else {
                    // TODO: Logging
                }

            }
            else {
                // TODO: Logging
            }

            incoming_buffer_len = 0;
        }
        else {
            if(incoming_buffer_len >= 256) {
                // TODO: Logging
            }

            incoming_buffer[incoming_buffer_len & 0x00FF] = incoming_byte;
            incoming_buffer_len++;
        }
    }

    // Run update in case of E-Stop trigger or parse incomplete
    wheel_L.update();
    wheel_R.update();
}

void writeStates() {
    {
        CoreMutex mutex(&encoder_mutex);
        msg_states.position_left = wheel_L.get_position();
        msg_states.velocity_left = wheel_L.get_velocity();
        msg_states.position_right = wheel_R.get_position();
        msg_states.velocity_right = wheel_R.get_velocity();
    }

    // ostream = pb_ostream_from_buffer(msg_buffer, 256);
    ostream = pb_ostream_from_buffer(msg_states_buffer, 256);

    pb_encode(&ostream, carry_my_luggage_WheelStates_fields, &msg_states);

    // encode_result = cobs_encode(outgoing_buffer, 256, msg_buffer, ostream.bytes_written);
    encode_result = cobs_encode(outgoing_buffer, 256, msg_states_buffer, ostream.bytes_written);

    if(encode_result.status == cobs_encode_status::COBS_ENCODE_OK) {
        COM_SERIAL.write(outgoing_buffer, encode_result.out_len);
        COM_SERIAL.write((uint8_t)0);
        
        // printBufferHex(outgoing_buffer, encode_result.out_len);
    }
    else {
        switch(encode_result.status) {
            case cobs_encode_status::COBS_ENCODE_NULL_POINTER:
                // TODO: Logging
                break;
            case cobs_encode_status::COBS_ENCODE_OUT_BUFFER_OVERFLOW:
                // TODO: Logging
                break;
            default:
                // TODO: Logging
                break;
        }
    }

    // printWheelStates();
}


void setup() {
    mutex_init(&encoder_mutex);

    COM_SERIAL.begin(115200);
    DEBUG_SERIAL.begin(115200);

    wheel_L.begin();
    wheel_R.begin();
    estop.begin();

    initComplete = true;
}

void loop() {
    readCommands();
    writeStates();
    // delayMicroseconds(1000);
}

void setup1() {
    while(!initComplete) delay(10); // I tried so hard to get CoreMutex working believe me, I know this is not the proper way
}

void loop1() {
    // Read encoders as quickly as possible to make sure getCumulativePosition and getAngularSpeed don't skip counts at high RPM
    CoreMutex mutex(&encoder_mutex);
    wheel_L.read_encoder();
    wheel_R.read_encoder();
}
