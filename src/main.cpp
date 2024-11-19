#include <Arduino.h>

#include "cobs.h"
#include <pb_decode.h>
#include <pb_encode.h>

#include "wheel.h"
#include "estop.h"
#include "wheel_commands.pb.h"
#include "wheel_states.pb.h"


uint8_t incoming_byte;
uint8_t incoming_buffer[256];
uint16_t incoming_buffer_len;

carry_my_luggage_WheelCommands msg_commands;
uint8_t msg_commands_buffer[256];
pb_istream_t istream;
cobs_decode_result decode_result;

carry_my_luggage_WheelStates msg_states;
uint8_t msg_states_buffer[256];
pb_ostream_t ostream;
cobs_encode_result encode_result;

uint8_t outgoing_buffer[256];

bool initComplete;

// mutex_t encoder_mutex = {0}; 

Wheel wheel_L(17, 16, 18, 10, 11);
Wheel wheel_R(14, 15, 13, 20, 21);
EStop estop(19);


void printWheelCommands() {
    Serial.println();
    Serial.print("Wheel commands: ");
    Serial.print(msg_commands.velocity_left);
    Serial.print(" ");
    Serial.println(msg_commands.velocity_right);
}

void printWheelStates() {
    Serial.println();
    Serial.print("Wheel positions: ");
    Serial.print(msg_states.position_left);
    Serial.print(" ");
    Serial.println(msg_states.position_right);
    Serial.print("Wheel velocities: ");
    Serial.print(msg_states.velocity_left);
    Serial.print(" ");
    Serial.println(msg_states.velocity_right);
}

void printBufferDec(uint8_t* buf, size_t len) {
    for(size_t i = 0; i < len-1; i++) {
        Serial.print(buf[i], DEC);
        Serial.print(", ");
    }
    if(len >= 1) Serial.println(buf[len-1], DEC);

    Serial.println();
}

void printBufferHex(uint8_t* buf, size_t len) {
    for(size_t i = 0; i < len; i++) {
        Serial.print(buf[i], HEX);
        Serial.print(" ");
    }
    if(len >= 1) Serial.println(buf[len-1], HEX);

    Serial.println();
}

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
                wheel_L.update(msg_commands.velocity_left);
                wheel_R.update(msg_commands.velocity_right);

                // printWheelCommands();                
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
}

void writePositions() {
    {
        // CoreMutex mutex(&encoder_mutex); // FIXME: Idk why this doesn't work
        // wheel_L.read_encoder();
        // wheel_R.read_encoder();
        msg_states.position_left = wheel_L.get_position();
        msg_states.velocity_left = wheel_L.get_velocity();
        msg_states.position_right = wheel_R.get_position();
        msg_states.velocity_right = wheel_R.get_velocity();
    }

    ostream = pb_ostream_from_buffer(msg_states_buffer, 256);

    pb_encode(&ostream, carry_my_luggage_WheelStates_fields, &msg_states);

    encode_result = cobs_encode(outgoing_buffer, 256, msg_states_buffer, ostream.bytes_written);

    if(encode_result.status == cobs_encode_status::COBS_ENCODE_OK) {
        Serial.write(outgoing_buffer, encode_result.out_len);
        Serial.write((uint8_t)0);
        
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
        }
    }

    // printWheelStates();
}


void setup() {
    Serial.begin(115200);

    wheel_L.begin();
    wheel_R.begin();
    estop.begin();

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
    // Read encoders as quickly as possible to make sure getCumulativePosition and getAngularSpeed don't skip counts at high RPM
    // CoreMutex mutex(&encoder_mutex); // Screw it, memory access is atomic either way
    wheel_L.read_encoder();
    wheel_R.read_encoder();
}
