#include <Arduino.h>

#include "cobs.h"
#include <pb_decode.h>

#include "wheel.h"
#include "wheel_commands.pb.h"


uint8_t incoming_byte;
uint8_t* incoming_buffer = new uint8_t[256];
uint16_t incoming_buffer_len;

_carry_my_luggage_WheelCommands msg_commands;
uint8_t* msg_commands_buffer = new uint8_t[256];
pb_istream_t istream;
cobs_decode_result decode_result;

Wheel* wheel_L;
Wheel* wheel_R;


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

void setup() {
    Serial.begin(115200);

    wheel_L = new Wheel(3, 2, 6);
    wheel_R = new Wheel(4, 5, 7);

    wheel_L->begin();
    wheel_R->begin();
}

void loop() {
    readCommands();

}
