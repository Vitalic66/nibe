#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <ftdi.h>
#include "MQTTClient.h"

#define MQTT_ADDRESS    "tcp://192.168.178.92:1883"
#define CLIENTID        "NibeDisplayClient"
#define QOS             1
#define TIMEOUT         10000L

// Global MQTT client handle
MQTTClient client;

/* Publiziert eine MQTT-Nachricht an das angegebene Topic */
void publish_mqtt(const char* topic, const char* message) {
    // Prüfe, ob die Nachricht leer ist
    if (message == NULL || strlen(message) == 0) {
        printf("Leere Nachricht – nicht publizieren.\n");
        return;
    }
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
        int blitz      = (payload[0] & 0x08) ? 1 : 0;
        int stufe1     = (payload[0] & 0x04) ? 1 : 0;
        int stufe2     = (payload[0] & 0x02) ? 1 : 0;
        int stufe3     = (payload[0] & 0x01) ? 1 : 0;

        // Byte 2 (payload[1]):
        //   Bit1 (0x80): Wasserhahn
        int wasserhahn = (payload[1] & 0x80) ? 1 : 0;

        // Byte 3 (payload[2]):
        //   Bit1 (0x80): Pumpe
        //   Bit5 (0x08): Heizung
        //   Bit8 (0x01): Tropfen
        int pumpe   = (payload[2] & 0x80) ? 1 : 0;
        int heizung = (payload[2] & 0x08) ? 1 : 0;
        int tropfen  = (payload[2] & 0x01) ? 1 : 0;

        // Erstelle einen JSON-String
        char json[256];
        snprintf(json, sizeof(json),
                 "{\"verdichter\": %d, \"blitz\": %d, \"stufe1\": %d, \"stufe2\": %d, \"stufe3\": %d, \"wasserhahn\": %d, \"pumpe\": %d, \"heizung\": %d, \"tropfen\": %d}",
                 verdichter, blitz, stufe1, stufe2, stufe3, wasserhahn, pumpe, heizung, tropfen);

        // Für Identifier 0x50 verwenden wir z. B. das Topic "nibe_2/display/line1"
        publish_mqtt("nibe/display/line1", json);
        printf("Published JSON message '%s' to topic '%s'\n", json, "nibe_2/display/line1");
    } else {
        // Für alle anderen Identifier: ASCII-Umwandlung wie bisher
        const char* topic;
        switch(identifier) {
            case 0x50: topic = "nibe/display/line1"; break;
            case 0x51: topic = "nibe/display/line2"; break;
            case 0x52: topic = "nibe/display/line3"; break;
            case 0x53: topic = "nibe/display/line4"; break;
            default:   topic = "nibe/display/unknown"; break;
        }
        char ascii_payload[256];
        size_t out_index = 0;
        // Durchlaufe den Payload und konvertiere, falls 0xB0 vorkommt
        for (size_t i = 0; i < payload_len && out_index < sizeof(ascii_payload) - 1; i++) {
            if (payload[i] == 0xB0) {
                // Ersetze 0xB0 durch UTF-8: 0xC2 0xB0, sofern genügend Platz im Puffer ist
                if (out_index + 2 < sizeof(ascii_payload) - 1) {
                    ascii_payload[out_index++] = 0xC2;
                    ascii_payload[out_index++] = 0xB0;
                } else {
                    break; // Nicht genügend Platz, also abbrechen
                }
            } else {
                ascii_payload[out_index++] = payload[i];
            }
        }
        ascii_payload[out_index] = '\0';
        publish_mqtt(topic, ascii_payload);
        printf("Published ASCII message '%s' to topic '%s'\n", ascii_payload, topic);
    }
}

int main(void) {
    int ret;
    struct ftdi_context *ftdi;
    ftdi = ftdi_new();
    if (ftdi == NULL) {
        fprintf(stderr, "ftdi_new failed\n");
        return EXIT_FAILURE;
    }

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

    // Setze Baudrate auf 19200
    ret = ftdi_set_baudrate(ftdi, 19200);
    if (ret < 0) {
        fprintf(stderr, "Unable to set baudrate: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        ftdi_usb_close(ftdi);
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }

    // Setze 8 Datenbits, 1 Stoppbit und Space-Parität (statt NONE)
    ret = ftdi_set_line_property(ftdi, BITS_8, STOP_BIT_1, SPACE);
    if (ret < 0) {
        fprintf(stderr, "Unable to set line properties: %d (%s)\n", ret, ftdi_get_error_string(ftdi));
        ftdi_usb_close(ftdi);
        ftdi_free(ftdi);
        return EXIT_FAILURE;
    }

    // Setze einen Lese-Timeout (in Millisekunden)
    ftdi->usb_read_timeout = 3000;

    printf("FTDI device opened successfully.\n");

    // MQTT-Verbindung initialisieren und herstellen
    MQTTClient_create(&client, MQTT_ADDRESS, CLIENTID,
                      MQTTCLIENT_PERSISTENCE_NONE, NULL);
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    ret = MQTTClient_connect(client, &conn_opts);
    if (ret != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect to MQTT broker, return code %d\n", ret);
        ftdi_usb_close(ftdi);
        ftdi_free(ftdi);
        MQTTClient_destroy(&client);
        return EXIT_FAILURE;
    }
    printf("Connected to MQTT broker at %s.\n", MQTT_ADDRESS);

    /* Hauptschleife:
       Suche die Startsequenz mittels eines Sliding Windows.
       Gesucht wird das Muster: 0x00, 0xF9, 0x06, XX   (XX muss 0x50, 0x51, 0x52 oder 0x53 sein)
    */
    while (1) {
        uint8_t window[4] = {0};
        int count = 0;
        // Fülle zuerst das Fenster mit 4 Bytes (falls möglich)
        while (count < 4) {
            ret = ftdi_read_data(ftdi, &window[count], 1);
            if (ret < 1) {
                usleep(100000);
                continue;
            }
            count++;
        }
        // Jetzt suche im kontinuierlichen Stream das Muster
        while (1) {
            if (window[0] == 0x00 && window[1] == 0xF9 && window[2] == 0x06 &&
                (window[3] == 0x50 || window[3] == 0x51 || window[3] == 0x52 || window[3] == 0x53)) {
                break;  // Startsequenz gefunden
            }
            // Verschiebe das Fenster um ein Byte: window[0] <- window[1], window[1] <- window[2], etc.
            window[0] = window[1];
            window[1] = window[2];
            window[2] = window[3];
            ret = ftdi_read_data(ftdi, &window[3], 1);
            if (ret < 1) {
                usleep(100000);
                continue;
            }
        }
        uint8_t identifier = window[3];
        printf("Start sequence found with identifier 0x%02X\n", identifier);

        // Lese 3 weitere Bytes
        uint8_t extra[3];
        ret = ftdi_read_data(ftdi, extra, 3);
        if (ret < 3) {
            printf("Failed to read 3 extra bytes.\n");
            continue;
        }
        // Das dritte Extra-Byte bestimmt die Länge der Payload
        uint8_t msg_length = extra[2];
        printf("Message length determined: %d bytes\n", msg_length);

        // Lese anschliessend (msg_length + 1) Bytes: Payload plus gemeldeter CRC
        size_t payload_block_len = msg_length + 1;
        uint8_t *payload_block = malloc(payload_block_len);
        if (!payload_block) {
            fprintf(stderr, "Memory allocation error\n");
            continue;
        }
        ret = ftdi_read_data(ftdi, payload_block, payload_block_len);
        if (ret < payload_block_len) {
            printf("Failed to read payload block (expected %zu, got %d)\n", payload_block_len, ret);
            free(payload_block);
            continue;
        }
        // CRC-Berechnung: aus dem Identifier (aus dem Startfenster), den 3 Extra-Bytes und den Payload-Bytes (ohne letztes Byte)
        uint8_t computed_crc = 0;
        computed_crc ^= identifier;
        for (int i = 0; i < 3; i++) {
            computed_crc ^= extra[i];
        }
        for (size_t i = 0; i < msg_length; i++) {
            computed_crc ^= payload_block[i];
        }
        uint8_t reported_crc = payload_block[msg_length];
        if (computed_crc != reported_crc) {
            printf("CRC error: computed 0x%02X, expected 0x%02X\n", computed_crc, reported_crc);
            free(payload_block);
            continue;
        }
        printf("CRC check passed.\n");

        // Die Payload ist payload_block[0 .. msg_length-1]
        publish_ascii_message_with_subtopic(identifier, payload_block, msg_length);
        free(payload_block);

        // Optional: Empfangspuffer leeren
        ftdi_usb_purge_rx_buffer(ftdi);
    }

    // Aufräumen
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);
    ftdi_usb_close(ftdi);
    ftdi_free(ftdi);
    return EXIT_SUCCESS;
}
