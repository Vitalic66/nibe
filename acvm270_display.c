
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

// Global MQTT client handle
MQTTClient client;

/* Publiziert eine MQTT-Nachricht an das angegebene Topic */
void publish_mqtt(const char* topic, const char* message) {
    /*
    if (message == NULL || strlen(message) == 0) {
        printf("Leere Nachricht – nicht publizieren.\n");
        return;
    }
    */
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

/* Bestimmt anhand des Identifiers das Subtopic, wandelt die Payload in einen ASCII-String um und publiziert */
void publish_ascii_message_with_subtopic(uint8_t identifier, uint8_t* payload, size_t payload_len) {
    // Falls der Identifier 0x50 ist und wir mindestens 3 Bytes Payload haben, verarbeite diese als Bitmatrix
    if (identifier == 0x50 && payload_len >= 3) {
        // Annahme: Bits werden von MSB (Bit 1) bis LSB (Bit 8) nummeriert.
        // Byte 1 (payload[0]):
        //   Bit1 (0x80): Verdichter
        //   Bit5 (0x08): Blitz
        //   Bit6 (0x04): Stufe 1
        //   Bit7 (0x02): Stufe 2
        //   Bit8 (0x01): Stufe 3
        int verdichter = (payload[0] & 0x80) ? 1 : 0;
        int byte1_A    = (payload[0] & 0x40) ? 1 : 0;
        int blitz      = (payload[0] & 0x08) ? 1 : 0;
        int stufe1     = (payload[0] & 0x04) ? 1 : 0;
        int stufe2     = (payload[0] & 0x02) ? 1 : 0;
        int stufe3     = (payload[0] & 0x01) ? 1 : 0;

        // Byte 2 (payload[1]):
        //   Bit1 (0x80): Wasserhahn
        int wasserhahn = (payload[1] & 0x80) ? 1 : 0;
        int byte2_A    = (payload[1] & 0x40) ? 1 : 0;

        // Byte 3 (payload[2]):
        //   Bit1 (0x80): Pumpe
        //   Bit5 (0x08): Heizung
        //   Bit8 (0x01): Tropfen
        int pumpe   = (payload[2] & 0x80) ? 1 : 0;
        int heizung = (payload[2] & 0x08) ? 1 : 0;
        int tropfen = (payload[2] & 0x01) ? 1 : 0;

        // Erstelle einen JSON-String
        char json[256];
        snprintf(json, sizeof(json),
                 "{\"verdichter\": %d, \"byte1_A\": %d, \"blitz\": %d, \"stufe1\": %d, \"stufe2\": %d, \"stufe3\": %d, \"wasserhahn\": %d, \"byte2_A\": %d, \"pumpe\": %d, \"heizung\": %d, \"tropfen\": %d}",
                 verdichter, byte1_A, blitz, stufe1, stufe2, stufe3, wasserhahn, byte2_A, pumpe, heizung, tropfen);

        // Für Identifier 0x50 verwenden wir z. B. das Topic "nibe_2/display/line1"
        publish_mqtt("nibe/display/line1", json);
        printf("Published JSON message '%s' to topic '%s'\n", json, "nibe/display/line1");
    } else if (identifier == 0x53) {
        // Zunächst Payload in einen nullterminierten String kopieren (maximal 255 Zeichen)
        char temp[256];
        size_t len = payload_len < sizeof(temp) - 1 ? payload_len : sizeof(temp) - 1;
        memcpy(temp, payload, len);
        temp[len] = '\0';

        // Suche nach dem ersten Leerzeichen, das als Trenner dient
        char *first_token = temp;
        // Überspringe führende Leerzeichen (falls vorhanden)
        while (*first_token && isspace((unsigned char)*first_token)) {
            first_token++;
        }
        char *p = first_token;
        // Finde das Ende des ersten Tokens (erstes Leerzeichen)
        while (*p && !isspace((unsigned char)*p)) {
            p++;
        }
        if (*p) {
            *p = '\0';  // Token beenden
            p++;        // Starte zweiten Token
        }
        // Überspringe etwaige Leerzeichen vor dem zweiten Token
        while (*p && isspace((unsigned char)*p)) {
            p++;
        }
        char *second_token = p;

        // Entferne innerhalb beider Tokens alle Leerzeichen
        char token1[256] = {0}, token2[256] = {0};
        size_t j = 0;
        for (char *s = first_token; *s; s++) {
            if (!isspace((unsigned char)*s)) {
                token1[j++] = *s;
            }
        }
        token1[j] = '\0';
        j = 0;
        for (char *s = second_token; *s; s++) {
            if (!isspace((unsigned char)*s)) {
                token2[j++] = *s;
            }
        }
        token2[j] = '\0';

        // Sende die beiden Teilstrings an unterschiedliche Topics
        publish_mqtt("nibe/display/line4a", token1);
        publish_mqtt("nibe/display/line4b", token2);
        printf("Published split messages: '%s' to nibe/display/line4a and '%s' to nibe/display/line4b\n", token1, token2);
    } else {
        // Für alle anderen Identifier: ASCII-Umwandlung wie bisher
        const char* topic;
        switch(identifier) {
            //case 0x50: topic = "nibe/display/line1"; break;
            case 0x51: topic = "nibe/display/line2"; break;
            case 0x52: topic = "nibe/display/line3"; break;
            //case 0x53: topic = "nibe/display/line4"; break;
            default:   topic = "nibe/display/unknown"; break;
        }
        char ascii_payload[256];
        size_t out_index = 0;
        // Durchlaufe den Payload und konvertiere, falls 0xB0 vorkommt
        for (size_t i = 0; i < payload_len && out_index < sizeof(ascii_payload) - 1; i++) {
            switch (payload[i]) {
                case 0xB0:
                    if (out_index + 2 < sizeof(ascii_payload) - 1) {
                        ascii_payload[out_index++] = 0xC2;
                        ascii_payload[out_index++] = 0xB0;
                    }
                    break;
                case 0xDF: // ß
                    if (out_index + 2 < sizeof(ascii_payload) - 1) {
                        ascii_payload[out_index++] = 0xC3;
                        ascii_payload[out_index++] = 0x9F;
                    }
                    break;
                case 0xE4: // ä
                    if (out_index + 2 < sizeof(ascii_payload) - 1) {
                        ascii_payload[out_index++] = 0xC3;
                        ascii_payload[out_index++] = 0xA4;
                    }
                    break;
                case 0xFC: // ü
                    if (out_index + 2 < sizeof(ascii_payload) - 1) {
                        ascii_payload[out_index++] = 0xC3;
                        ascii_payload[out_index++] = 0xBC;
                    }
                    break;
                case 0xF6: // ö
                    if (out_index + 2 < sizeof(ascii_payload) - 1) {
                        ascii_payload[out_index++] = 0xC3;
                        ascii_payload[out_index++] = 0xB6;
                    }
                    break;
                default:
                    ascii_payload[out_index++] = payload[i];
                    break;
        }
    }
        ascii_payload[out_index] = '\0';
        publish_mqtt(topic, ascii_payload);
        printf("Published ASCII message '%s' to topic '%s'\n", ascii_payload, topic);
    }
}

int main(void) {
    int ret;
    struct ftdi_context *ftdi = ftdi_new();
    if (ftdi == NULL) {
        fprintf(stderr, "ftdi_new failed\n");
        return EXIT_FAILURE;
    }

    /*
    // Definiere Vendor und Produkt ID (z.B. FT232: 0x0403/0x6001)
    int vendor = 0x0403, product = 0x6001;

    // Suche nach allen FTDI-Geräten
    struct ftdi_device_list *devlist, *curdev;
    ret = ftdi_usb_find_all(ftdi, &devlist, vendor, product);
    if (ret < 0) {
        fprintf(stderr, "ftdi_usb_find_all failed: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }
    printf("Found %d FTDI device(s).\n", ret);

    int deviceFound = 0;
    curdev = devlist;
    while (curdev) {
        char serial[128] = {0};
        ret = ftdi_usb_get_strings(ftdi, curdev->dev, NULL, 0, NULL, 0, serial, sizeof(serial));
        if (ret < 0) {
            fprintf(stderr, "ftdi_usb_get_strings failed: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
            break;
        }
        printf("Device: Serial: %s\n", serial);
        if (strcmp(serial, "B001YIW0") == 0) {
            ret = ftdi_usb_open_dev(ftdi, curdev->dev);
            if (ret < 0) {
                fprintf(stderr, "ftdi_usb_open_dev failed: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
                ftdi_list_free(&devlist);
                ftdi_free(ftdi);
                return EXIT_FAILURE;
            }
            printf("Device with serial B001YIW0 successfully opened.\n");
            deviceFound = 1;
            break;
        }
        curdev = curdev->next;
    }
    if (!deviceFound) {
        fprintf(stderr, "Device with serial B001YIW0 not found.\n");
        ftdi_list_free(&devlist);
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }
    ftdi_list_free(&devlist);
    */
    /*
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
    */

    // Open specific device by serial number
    ret = ftdi_usb_open_desc(ftdi, 0x0403, 0x6001, NULL, "B001YIW0");
    if (ret < 0) {
        fprintf(stderr, "Unable to open FTDI device with serial B001YIW0: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }
    printf("FTDI device with serial B001YIW0 opened successfully.\n");
    
    ret = ftdi_set_baudrate(ftdi, 19200);
    if (ret < 0) {
        fprintf(stderr, "Unable to set baudrate: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        ftdi_usb_close(ftdi);
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }

    // Setze 8 Datenbits, 1 Stoppbit und Space-Parität erzwingen
    ret = ftdi_set_line_property(ftdi, BITS_8, STOP_BIT_1, SPACE);
    if (ret < 0) {
        fprintf(stderr, "Unable to set line properties: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        ftdi_usb_close(ftdi);
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }

    ftdi->usb_read_timeout = 3000;
    printf("FTDI device opened successfully.\n");

    MQTTClient_create(&client, MQTT_ADDRESS, CLIENTID,
                      MQTTCLIENT_PERSISTENCE_NONE, NULL);
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    /*
    ret = MQTTClient_connect(client, &conn_opts);
    if (ret != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect to MQTT broker, return code %d\n", ret);
        ftdi_usb_close(ftdi);
        ftdi_free(ftdi);
        MQTTClient_destroy(&client);
        return EXIT_FAILURE;
    }
    printf("Connected to MQTT broker at %s.\n", MQTT_ADDRESS);
    */

    int attempts = 0;
    while ((ret = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect to MQTT broker, return code %d. Retrying in 5 seconds...\n", ret);
        sleep(5);
        if (++attempts >= 10) {  // nach 10 Versuchen ggf. doch abbrechen
            fprintf(stderr, "Unable to connect to MQTT broker after multiple attempts.\n");
            ftdi_usb_close(ftdi);
            ftdi_free(ftdi);
            MQTTClient_destroy(&client);
            return EXIT_FAILURE;
        }
    }
    printf("Connected to MQTT broker at %s.\n", MQTT_ADDRESS);
    
    // Pufferbasierte Verarbeitung
    uint8_t buffer[BUFFER_SIZE];
    size_t buffer_len = 0;

    while (1) {
        // Lese verfügbare Daten in den Puffer
        if (buffer_len < BUFFER_SIZE) {
            ret = ftdi_read_data(ftdi, buffer + buffer_len, BUFFER_SIZE - buffer_len);
            if (ret > 0) {
                buffer_len += ret;
            }
        }
        // Suche im Puffer nach gültigen Startsequenzen
        size_t pos = 0;
        while (pos + 7 <= buffer_len) { // mindestens 7 Bytes für Startseq + Extra
            if (buffer[pos] == 0x00 && buffer[pos+1] == 0xF9 &&
                buffer[pos+2] == 0x06 &&
                (buffer[pos+3] == 0x50 || buffer[pos+3] == 0x51 ||
                 buffer[pos+3] == 0x52 || buffer[pos+3] == 0x53)) {
                // Stelle sicher, dass 3 Extra-Bytes vorhanden sind
                if (pos + 7 > buffer_len)
                    break;
                uint8_t msg_length = buffer[pos+6]; // drittes Extra-Byte
                size_t total_msg_len = 4 + 3 + (msg_length + 1);
                if (pos + total_msg_len > buffer_len)
                    break; // vollständiger Frame noch nicht verfügbar

                // Berechne CRC: Identifier (buffer[pos+3]), Extra-Bytes (pos+4, pos+5, pos+6) und Payload
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
        // Verschiebe unvollständige Daten an den Anfang des Puffers
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
