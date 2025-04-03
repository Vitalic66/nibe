#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>

#include <ftdi.h>
#include "MQTTClient.h"

#define MQTT_ADDRESS    "tcp://192.168.178.92:1883"
#define CLIENTID        "NibeDisplayClient"
#define QOS             1
#define TIMEOUT         10000L
#define BUFFER_SIZE     1024

MQTTClient client;

void publish_mqtt(const char* topic, const char* message) {
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    pubmsg.payload = (void*)message;
    pubmsg.payloadlen = (int)strlen(message);
    pubmsg.qos = QOS;
    pubmsg.retained = 1;
    int rc = MQTTClient_publishMessage(client, topic, &pubmsg, &token);
    if (rc != MQTTCLIENT_SUCCESS) {
        printf("Failed to publish message, return code %d\n", rc);
    } else {
        printf("Published '%s' to topic '%s'\n", message, topic);
    }
    MQTTClient_waitForCompletion(client, token, TIMEOUT);
}

void publish_ascii_message_with_subtopic(uint8_t identifier, uint8_t* payload, size_t payload_len);

int main(void) {
    int ret;
    struct ftdi_context *ftdi = ftdi_new();
    if (ftdi == NULL) {
        fprintf(stderr, "ftdi_new failed\n");
        return EXIT_FAILURE;
    }

    ret = ftdi_set_interface(ftdi, INTERFACE_ANY);
    if (ret < 0) {
        fprintf(stderr, "ftdi_set_interface failed: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }

    ret = ftdi_usb_open_dev(ftdi, ftdi_usb_get_device_by_path(ftdi, "/dev/ttyUSB1"));
    if (ret < 0) {
        fprintf(stderr, "Unable to open /dev/ttyUSB1: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }
    printf("FTDI device at /dev/ttyUSB1 opened successfully.\n");

    ret = ftdi_set_baudrate(ftdi, 19200);
    if (ret < 0) {
        fprintf(stderr, "Unable to set baudrate: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        ftdi_usb_close(ftdi);
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }

    ret = ftdi_set_line_property(ftdi, BITS_8, STOP_BIT_1, SPACE);
    if (ret < 0) {
        fprintf(stderr, "Unable to set line properties: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        ftdi_usb_close(ftdi);
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }

    ftdi->usb_read_timeout = 3000;

    MQTTClient_create(&client, MQTT_ADDRESS, CLIENTID,
                      MQTTCLIENT_PERSISTENCE_NONE, NULL);
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    int attempts = 0;
    while ((ret = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect to MQTT broker, return code %d. Retrying in 5 seconds...\n", ret);
        sleep(5);
        if (++attempts >= 10) {
            fprintf(stderr, "Unable to connect to MQTT broker after multiple attempts.\n");
            ftdi_usb_close(ftdi);
            ftdi_free(ftdi);
            MQTTClient_destroy(&client);
            return EXIT_FAILURE;
        }
    }
    printf("Connected to MQTT broker at %s.\n", MQTT_ADDRESS);

    uint8_t buffer[BUFFER_SIZE];
    size_t buffer_len = 0;

    while (1) {
        if (buffer_len < BUFFER_SIZE) {
            ret = ftdi_read_data(ftdi, buffer + buffer_len, BUFFER_SIZE - buffer_len);
            if (ret > 0) buffer_len += ret;
        }

        size_t pos = 0;
        while (pos + 7 <= buffer_len) {
            if (buffer[pos] == 0x00 && buffer[pos+1] == 0xF9 &&
                buffer[pos+2] == 0x06 &&
                (buffer[pos+3] == 0x50 || buffer[pos+3] == 0x51 ||
                 buffer[pos+3] == 0x52 || buffer[pos+3] == 0x53)) {

                if (pos + 7 > buffer_len) break;
                uint8_t msg_length = buffer[pos+6];
                size_t total_msg_len = 4 + 3 + (msg_length + 1);
                if (pos + total_msg_len > buffer_len) break;

                uint8_t computed_crc = 0;
                computed_crc ^= buffer[pos+3];
                computed_crc ^= buffer[pos+4];
                computed_crc ^= buffer[pos+5];
                computed_crc ^= buffer[pos+6];
                for (size_t i = pos+7; i < pos+7+msg_length; i++) {
                    computed_crc ^= buffer[i];
                }
                uint8_t reported_crc = buffer[pos+7+msg_length];
                if (computed_crc == reported_crc) {
                    publish_ascii_message_with_subtopic(buffer[pos+3], buffer + pos + 7, msg_length);
                } else {
                    printf("CRC error: computed 0x%02X, reported 0x%02X\n", computed_crc, reported_crc);
                }
                pos += total_msg_len;
            } else {
                pos++;
            }
        }
        if (pos > 0) {
            memmove(buffer, buffer + pos, buffer_len - pos);
            buffer_len -= pos;
        }
        usleep(10000);
    }

    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);
    ftdi_usb_close(ftdi);
    ftdi_free(ftdi);
    return EXIT_SUCCESS;
}
