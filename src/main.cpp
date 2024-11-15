#include <Arduino.h>

#include "cobs.h"
#include <pb_decode.h>
#include <pb_encode.h>

#include "wheel.h"
#include "estop.h"
#include "wheel_commands.pb.h"
#include "wheel_states.pb.h"


uint8_t incoming_byte;
uint8_t* incoming_buffer = new uint8_t[256];
uint16_t incoming_buffer_len;

_carry_my_luggage_WheelCommands msg_commands;
uint8_t* msg_commands_buffer = new uint8_t[256];
pb_istream_t istream;
cobs_decode_result decode_result;

_carry_my_luggage_WheelStates msg_states;
uint8_t* msg_states_buffer = new uint8_t[256];
pb_ostream_t ostream;
cobs_encode_result encode_result;

uint8_t* outgoing_buffer = new uint8_t[256];

bool initComplete;

mutex_t encoder_mutex = {0};

Wheel* wheel_L;
Wheel* wheel_R;
EStop* estop;


void readCommands() {
    while(Serial.available()){
        incoming_byte = Serial.read();

        if(incoming_byte == '\0') {
            if(incoming_buffer_len == 0) return;

            decode_result = cobs_decode(msg_commands_buffer, 256, incoming_buffer, incoming_buffer_len & 0x00FF);
                
            if(decode_result.status != cobs_decode_status::COBS_DECODE_OK) {
                // TODO: Logging

                return;
            }
            
            istream = pb_istream_from_buffer(msg_commands_buffer, decode_result.out_len);

            if(pb_decode(&istream, carry_my_luggage_WheelCommands_fields, &msg_commands)) {
                wheel_L->update(msg_commands.velocity_left);
                wheel_R->update(msg_commands.velocity_right);

                // Serial.print(msg.velocity_left);
                // Serial.print(" ");
                // Serial.println(msg.velocity_right);
            }

            incoming_buffer_len = 0;
        }
        else {
            if(incoming_buffer_len >= 256) {
                // Serial.println("Incoming buffer overflow detected");
            }

            incoming_buffer[incoming_buffer_len & 0x00FF] = incoming_byte;
            incoming_buffer_len++;
        }
    }
}

void writePositions() {
    {
        // CoreMutex mutex(&encoder_mutex); // FIXME: Idk why this doesn't work
        // wheel_L->read_encoder();
        // wheel_R->read_encoder();
        msg_states.position_left = wheel_L->get_position();
        msg_states.velocity_left = wheel_L->get_velocity();
        msg_states.position_right = wheel_R->get_position();
        msg_states.velocity_right = wheel_R->get_velocity();
    }

    ostream = pb_ostream_from_buffer(msg_states_buffer, 256);

    pb_encode(&ostream, carry_my_luggage_WheelStates_fields, &msg_states);

    encode_result = cobs_encode(outgoing_buffer, 256, msg_states_buffer, ostream.bytes_written);

    if(encode_result.status == cobs_encode_status::COBS_ENCODE_OK) {
        Serial.write(outgoing_buffer, encode_result.out_len);
        Serial.write((uint8_t)0);
        // for(int i=0; i<encode_result.out_len; i++) Serial.print((int)outgoing_buffer[i]);
        // Serial.print(0);
        // Serial.println();
    }
    else {
        switch(encode_result.status) {
            case cobs_encode_status::COBS_ENCODE_NULL_POINTER:
                Serial.println("cobs_encode_status::COBS_ENCODE_NULL_POINTER");
                break;
            case cobs_encode_status::COBS_ENCODE_OUT_BUFFER_OVERFLOW:
                Serial.println("cobs_encode_status::COBS_ENCODE_OUT_BUFFER_OVERFLOW");
                break;
            default:
                Serial.println("Unknown error during cobs encode");
        }
    }

    // Serial.println();
    // Serial.print(msg_states.position_left);
    // Serial.print(" ");
    // Serial.println(msg_states.position_right);
    // Serial.print(msg_states.velocity_left);
    // Serial.print(" ");
    // Serial.println(msg_states.velocity_right);
}

void setup() {
    Serial.begin(115200);

    wheel_L = new Wheel(17, 16, 18, 10, 11);
    wheel_R = new Wheel(14, 15, 13, 20, 21);
    estop = new EStop(19);

    wheel_L->begin();
    wheel_R->begin();
    estop->begin();

    initComplete = true;
}

void loop() {
    readCommands();
    writePositions();
    // delayMicroseconds(1000);
}

void setup1() {
    while(!initComplete) delay(10); // I tried so hard to get CoreMutex working believe me, I know this is not the proper way
}

void loop1() {
//     // Read encoders as quickly as possible to make sure getCumulativePosition and getAngularSpeed don't skip counts at high RPM
//     // CoreMutex mutex(&encoder_mutex); // Screw it, memory access is atomic either way
    wheel_L->read_encoder();
    wheel_R->read_encoder();
}
