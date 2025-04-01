#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pigpio.h>
#include <sys/time.h>
#include "MQTTClient.h"   // Paho MQTT

// MQTT-Konfiguration
#define ADDRESS     "tcp://192.168.178.92:1883"
#define CLIENTID    "NibeKeyClient"
#define TOPIC       "nibe/key"
#define QOS         1
#define TIMEOUT     10000L

// Hardware-/UART-Konfiguration
#define TX_GPIO 23
#define TX_ENABLE_GPIO 4
#define BAUD_RATE 19200
#define BIT_TIME_US (1000000 / BAUD_RATE)

// Globale Variablen für die MQTT-Nachricht
volatile unsigned int input_byte = 0;
volatile int taste_empfangen = 0;

// --- Paho MQTT Callback für empfangene Nachrichten ---
int messageArrived(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    if(message->payloadlen) {
        // Annahme: Der Payload wird als Hex-String gesendet (z. B. "1" oder "20")
        input_byte = (unsigned int)strtol((char *)message->payload, NULL, 16);
        taste_empfangen = 1;
        printf("MQTT: Empfangen Taste: %02X\n", input_byte);
    }
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

void connectionLost(void *context, char *cause) {
    printf("Verbindung verloren, Ursache: %s\n", cause);
}

// --- UART-Initialisierung für RX (8N1) ---  von alten versionen
void init_uart(int fd) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        exit(1);
    }
    cfsetospeed(&tty, B19200);
    cfsetispeed(&tty, B19200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 Datenbits
    tty.c_cflag &= ~PARENB;  // keine Parität
    tty.c_cflag &= ~CSTOPB;  // 1 Stopbit
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;  // 0,5 Sek. Timeout
    if(tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        exit(1);
    }
}

//#############################################################################################

unsigned char calculate_checksum(unsigned char *data, int len) {
    unsigned char sum = 0;
    for (int i = 0; i < len; i++) {
        sum ^= data[i];
    }
    return sum;
}

//#############################################################################################

int wait_for_response_value(int uart_fd) {
    unsigned char buf[8];
    int index = 0;
    struct timeval start, now;
    gettimeofday(&start, NULL);

    while (1) {
        int n = read(uart_fd, &buf[index], 1);
        if (n > 0) {
            if (index == 0 && buf[0] != 0x40) continue;
            index++;
            if (index == 8) break;
        }
        gettimeofday(&now, NULL);
        long elapsed = (now.tv_sec - start.tv_sec)*1000 + (now.tv_usec - start.tv_usec)/1000;
        if (elapsed > 1000) return -1;
    }
    return buf[5];
}

//#############################################################################################

int buildMultiByteWave(unsigned char *data, int count, int parity_mode) {
    int total_pulses = count * 11;
    gpioPulse_t *pulses = malloc(sizeof(gpioPulse_t) * total_pulses);
    if (!pulses) return -1;
    int pulseCount = 0;

    for (int j = 0; j < count; j++) {
        unsigned char b = data[j];
        pulses[pulseCount++] = (gpioPulse_t){.gpioOn = 0, .gpioOff = 1 << TX_GPIO, .usDelay = BIT_TIME_US};
        for (int i = 0; i < 8; i++) {
            int bit = (b >> i) & 1;
            pulses[pulseCount++] = bit ?
                (gpioPulse_t){.gpioOn = 1 << TX_GPIO, .gpioOff = 0, .usDelay = BIT_TIME_US} :
                (gpioPulse_t){.gpioOn = 0, .gpioOff = 1 << TX_GPIO, .usDelay = BIT_TIME_US};
        }
        pulses[pulseCount++] = parity_mode ?
            (gpioPulse_t){.gpioOn = 1 << TX_GPIO, .gpioOff = 0, .usDelay = BIT_TIME_US} :
            (gpioPulse_t){.gpioOn = 0, .gpioOff = 1 << TX_GPIO, .usDelay = BIT_TIME_US};
        pulses[pulseCount++] = (gpioPulse_t){.gpioOn = 1 << TX_GPIO, .gpioOff = 0, .usDelay = BIT_TIME_US};
    }

    gpioWaveAddNew();
    gpioWaveAddGeneric(pulseCount, pulses);
    int waveId = gpioWaveCreate();
    free(pulses);
    return waveId;
}

//#############################################################################################

// --- Warten auf ruhigen Bus ---
// Diese Funktion überwacht den UART-FD und gibt zurück, sobald für quiet_ms (z. B. 50 ms)
// keine Daten empfangen wurden.
int wait_for_quiet(int fd, int quiet_ms) {
    fd_set read_fds;
    struct timeval tv;

    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        tv.tv_sec = quiet_ms / 1000;
        tv.tv_usec = (quiet_ms % 1000) * 1000;

        int ret = select(fd + 1, &read_fds, NULL, NULL, &tv);
        if(ret < 0) {
            perror("select");
            return 0;
        }
        // Wenn select() 0 zurückgibt, gab es während des gesamten Intervalls keine Daten.
        if(ret == 0) {
            return 1; // Bus war quiet_ms lang still
        }
        // Falls Daten da sind, lese sie (und verwerfe sie) und wiederhole.
        char dummy[256];
        read(fd, dummy, sizeof(dummy));
    }
}

//#############################################################################################

// --- ACK-Warten ---
// Liest vom UART, bis ein ACK (0x06) empfangen wird oder das Timeout erreicht ist.
int wait_for_ack(int fd, int timeout_ms) {
    unsigned char buf;
    struct timeval start, now;
    gettimeofday(&start, NULL);
    while (1) {
        int n = read(fd, &buf, 1);
        if(n > 0 && buf == 0x06) {
            return 1; // ACK erhalten
        }
        gettimeofday(&now, NULL);
        long elapsed = (now.tv_sec - start.tv_sec)*1000 + (now.tv_usec - start.tv_usec)/1000;
        if(elapsed >= timeout_ms)
            return 0;
        //usleep(1000); // 1 ms warten
    }
}

//#############################################################################################

// --- ENQ-Warten ---
// Liest vom UART, bis ein ENQ (0x05) empfangen wird oder das Timeout erreicht ist.
int wait_for_enq(int fd, int timeout_ms) {
    unsigned char buf;
    struct timeval start, now;
    gettimeofday(&start, NULL);
    while (1) {
        int n = read(fd, &buf, 1);
        if(n > 0 && buf == 0x05) {
            return 1; // ENQ erhalten
        }
        gettimeofday(&now, NULL);
        long elapsed = (now.tv_sec - start.tv_sec)*1000 + (now.tv_usec - start.tv_usec)/1000;
        if(elapsed >= timeout_ms)
            return 0;
        usleep(1000); // 1 ms warten
    }
}

//#############################################################################################

// --- ETX-Warten ---
// Liest vom UART, bis ein ETX (0x03) empfangen wird oder das Timeout erreicht ist.
int wait_for_etx(int fd, int timeout_ms) {
    unsigned char buf;
    struct timeval start, now;
    gettimeofday(&start, NULL);
    while (1) {
        int n = read(fd, &buf, 1);
        if(n > 0 && buf == 0x03) {
            return 1; // ENQ erhalten
        }
        gettimeofday(&now, NULL);
        long elapsed = (now.tv_sec - start.tv_sec)*1000 + (now.tv_usec - start.tv_usec)/1000;
        if(elapsed >= timeout_ms)
            return 0;
        usleep(1000); // 1 ms warten
    }
}

//#############################################################################################

// Liest vom UART, bis ein 00 F9 empfangen wird oder das Timeout erreicht ist.

int wait_for_sequence(int fd, int timeout_ms) {
    unsigned char buf;
    struct timeval start, now;
    int state = 0;  // state 0: erwartet 0x00, state 1: erwartet 0xF9
    gettimeofday(&start, NULL);
    while (1) {
        int n = read(fd, &buf, 1);
        if(n > 0) {
            if(state == 0) {
                if(buf == 0x00)
                    state = 1;  // 0x00 gefunden, jetzt 0xF9 erwarten
            } else if(state == 1) {
                if(buf == 0xF9)
                    return 1;  // Sequenz 0x00 0xF9 gefunden
                else {
                    // Falls nicht 0xF9, prüfen, ob das aktuelle Byte eventuell wieder 0x00 ist
                    if(buf == 0x00)
                        state = 1;
                    else
                        state = 0;
                }
            }
        }
        gettimeofday(&now, NULL);
        long elapsed = (now.tv_sec - start.tv_sec)*1000 + (now.tv_usec - start.tv_usec)/1000;
        if(elapsed >= timeout_ms)
            return 0;
        usleep(1000); // 1 ms warten
    }
}

//#############################################################################################

int wait_for_start_sequence(int fd, int timeout_ms) {
    unsigned char buf;
    struct timeval start, now;
    int state = 0;  // state 0: erwartet 0x00, state 1: erwartet 0xF9
    gettimeofday(&start, NULL);
    while (1) {
        int n = read(fd, &buf, 1);
        if(n > 0) {
            if(state == 0) {
                if(buf == 0x00)
                    state = 1;  // 0x00 gefunden, jetzt 0xF9 erwarten
            } else if(state == 1) {
                if(buf == 0xF9)
                    return 1;  // Sequenz 0x00 0xF9 gefunden
                else {
                    // Falls nicht 0xF9, prüfen, ob das aktuelle Byte eventuell wieder 0x00 ist
                    if(buf == 0x00)
                        state = 1;
                    else
                        state = 0;
                }
            }
        }
        gettimeofday(&now, NULL);
        long elapsed = (now.tv_sec - start.tv_sec)*1000 + (now.tv_usec - start.tv_usec)/1000;
        if(elapsed >= timeout_ms)
            return 0;
        //usleep(1000); // 1 ms warten
    }
}

//#############################################################################################

int main() {

    // --- MQTT-Initialisierung mit Paho ---
    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    int rc;

    MQTTClient_create(&client, ADDRESS, CLIENTID,
                      MQTTCLIENT_PERSISTENCE_NONE, NULL);
    MQTTClient_setCallbacks(client, NULL, connectionLost, messageArrived, NULL);

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        fprintf(stderr, "Fehler: Verbindung zum Broker konnte nicht hergestellt werden, RC: %d\n", rc);
        exit(EXIT_FAILURE);
    }
    printf("Verbunden mit MQTT Broker: %s\n", ADDRESS);

    // Auf das Topic "device/taste" abonnieren
    if ((rc = MQTTClient_subscribe(client, TOPIC, QOS)) != MQTTCLIENT_SUCCESS) {
        fprintf(stderr, "Fehler: Abonnement fehlgeschlagen, RC: %d\n", rc);
        exit(EXIT_FAILURE);
    }
    printf("Abonniert Topic: %s\n", TOPIC);
/*
    // Warte, bis per MQTT eine Taste empfangen wurde
    printf("Warte auf MQTT-Nachricht mit der Taste (z.B. plus=1, minus=2, home=20, enter=4, brauchwasser=10)...\n");
    while(!taste_empfangen) {
        usleep(100000);  // 100 ms warten
    }
    // Nun enthält input_byte den per MQTT empfangenen Wert
*/

    // --- Hardware-Initialisierung (pigpio, UART) ---
    //if (gpioInitialise() < 0) return 1;
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Fehler: pigpio konnte nicht initialisiert werden.\n");
        exit(EXIT_FAILURE);
    }
    gpioSetMode(TX_GPIO, PI_OUTPUT);
    gpioSetMode(TX_ENABLE_GPIO, PI_OUTPUT);
    gpioWrite(TX_GPIO, 1);
    gpioWrite(TX_ENABLE_GPIO, 0);

    int uart_fd = open("/dev/serial0", O_RDWR | O_NOCTTY | O_SYNC);
    //if (uart_fd < 0) return 1;
    if (uart_fd < 0) {
        perror("Fehler beim Öffnen des UART");
        exit(EXIT_FAILURE);
    }

    init_uart(uart_fd); //von alten versionen übernommen

/*
    printf("Taste (plus=1, minus=2, home=20, enter=4, brauchwasser=10): ");
    unsigned int input_byte;
    scanf("%x", &input_byte);
*/

    // Endlosschleife: Permanent auf MQTT-Nachrichten warten und den Übertragungsprozess starten
    while (1) {
        // Warte auf eine neue MQTT-Nachricht
        printf("Warte auf MQTT-Nachricht mit der Taste (z.B. plus=1, minus=2, ...)...\n");
        while (!taste_empfangen) {
            usleep(100000);  // 100 ms warten
        }
        // Sobald eine Nachricht empfangen wurde, wird input_byte im Callback gesetzt.
        unsigned int key = input_byte;
        // Flag zurücksetzen, damit auf die nächste Nachricht gewartet werden kann
        taste_empfangen = 0;

        printf("Verwende per MQTT empfangene Taste: %02X\n", key);

//#############################################################################################

        // --- Warte, bis der Bus 50 ms lang ruhig ist ---
        printf("Warte 70 ms auf ruhigen Bus...\n");
        if(!wait_for_quiet(uart_fd, 70)) {
            printf("Fehler: Bus wurde nicht ruhig.\n");
            // Wir fahren trotzdem fort.
        }

//#############################################################################################

        // Sende 00 F9 mit Mark-Parität
        unsigned char t1[] = {0x00, 0xF9};
        printf("Transmission 1: Sende 0x00, 0xF9 mit Mark-Parität...\n");
        gpioWrite(TX_ENABLE_GPIO, 1);
        int wave = buildMultiByteWave(t1, 2, 1);
        gpioWaveTxSend(wave, PI_WAVE_MODE_ONE_SHOT);
        while (gpioWaveTxBusy()) gpioDelay(100);
        gpioWaveDelete(wave);
        gpioWrite(TX_ENABLE_GPIO, 0);

        //gpioDelay(1000); // kurze Pause
        gpioDelay(200);

//#############################################################################################

        printf("Warte auf ACK...\n");
        if (!wait_for_ack(uart_fd, 3))
            printf("Warnung: Kein ACK erhalten!\n");

        // Kurze Pause von 1 ms nach ACK
        gpioDelay(1100);

//#############################################################################################

        // Sende 40 00 F9 00 B9 mit Space-Parität
        unsigned char t2[] = {0x40, 0x00, 0xF9, 0x00, 0xB9};
        printf("Transmission 2: Sende 0x40, 0x00, 0xF9, 0x00, 0xB9 mit Space-Parität...\n");
        gpioWrite(TX_ENABLE_GPIO, 1);
        wave = buildMultiByteWave(t2, 5, 0);
        gpioWaveTxSend(wave, PI_WAVE_MODE_ONE_SHOT);
        while (gpioWaveTxBusy()) gpioDelay(100);
        gpioWaveDelete(wave);
        gpioWrite(TX_ENABLE_GPIO, 0);

//#############################################################################################

        printf("Warte auf ENQ ...\n");
        if (!wait_for_enq(uart_fd, 3))
            printf("Warnung: Kein ENQ erhalten!\n");

        // Kurze Pause von 1 ms nach ACK
        gpioDelay(100);

//#############################################################################################

        gpioDelay(1500);

//#############################################################################################

        // Sende 06 mit Space-Parität
        unsigned char t3[] = {06};
        printf("Transmission 3: Sende 0x06 mit Space-Parität...\n");
        gpioWrite(TX_ENABLE_GPIO, 1);
        wave = buildMultiByteWave(t3, 1, 0);
        gpioWaveTxSend(wave, PI_WAVE_MODE_ONE_SHOT);
        while (gpioWaveTxBusy()) gpioDelay(100);
        gpioWaveDelete(wave);
        gpioWrite(TX_ENABLE_GPIO, 0);

//#############################################################################################

        gpioDelay(7000);

//#############################################################################################
/*
    // Sende 0x05 (ENQ) mit Space
    unsigned char enq = 0x05;
    gpioWrite(TX_ENABLE_GPIO, 1);
    wave = buildMultiByteWave(&enq, 1, 0);
    gpioWaveTxSend(wave, PI_WAVE_MODE_ONE_SHOT);
    while (gpioWaveTxBusy()) gpioDelay(100);
    gpioWaveDelete(wave);
    gpioWrite(TX_ENABLE_GPIO, 0);
*/

//#############################################################################################
/*
    // Warte auf Antwort und extrahiere Wert xx
    unsigned char xx = wait_for_response_value(uart_fd);
    printf("extrahierer Wert: %02X \n", xx);
*/
    // Warte auf Antwort und extrahiere Wert xx
        int result = wait_for_response_value(uart_fd);
        if (result < 0) {
            fprintf(stderr, "Fehler: Keine gültige Antwort erhalten. Breche ab.\n");
            close(uart_fd);
            gpioTerminate();
            return 1;
        }
        unsigned char xx = (unsigned char)result;
        printf("Extrahierter Wert: %02X\n", xx);

//#############################################################################################
/*
    // Nachricht zusammensetzen und Prüfsumme berechnen
    unsigned char msg[8] = {0x40, 0x00, 0xF9, 0x04, input_byte, xx, 0x03, 0xFF};
    //printf("Nachricht ohne CC: ", msg);
    printf("Nachricht ohne CC: ");
    for (int i = 0; i < 8; i++) printf("%02X ", msg[i]);
    printf("\n");
*/
        // Hier wird input_byte (empfangen per MQTT) in die Nachricht eingebunden:
        printf("Verwende per MQTT empfangene Taste: %02X\n", input_byte);
        //unsigned char xx = (unsigned char)wait_for_response_value(uart_fd);
        unsigned char msg[8] = {0x40, 0x00, 0xF9, 0x04, (unsigned char)input_byte, xx, 0x03, 0xFF};
        printf("Nachricht ohne CC: ");
        for (int i = 0; i < 8; i++) printf("%02X ", msg[i]);
        printf("\n");


//#############################################################################################

        unsigned char checksum = calculate_checksum(msg, 8);
        //printf("Berechnetes CC: ", checksum);
        printf("Berechnetes CC: %02X\n", checksum);

//#############################################################################################

        // Gesamtnachricht
        unsigned char full_msg[9];
        memcpy(full_msg, msg, 8);
        full_msg[8] = checksum;
        //printf("Nachricht mit CC: ", full_msg);
        printf("Nachricht mit CC: ");
        for (int i = 0; i < 9; i++) printf("%02X ", full_msg[i]);
        printf("\n");

//#############################################################################################

         // --- Dauerschleife für kontinuierliche Suche nach `00 F9` ---
        while (1) {
            printf("Warte auf Start-Sequenz `00 F9`...\n");
            if (!wait_for_start_sequence(uart_fd, 500)) {  // 0,5 Sekunden Timeout
                printf("`00 F9` nicht empfangen. Neustart der Suche...\n");
                continue;  // Erneuter Versuch
            } else {
                printf("Sequenz `00 F9` erkannt! Starte Übertragung...\n");
                break; // Fortfahren mit dem Senden
            }
        }

//#############################################################################################

        gpioDelay(600);

 //#############################################################################################

        // Sende 05 mit Space-Parität
        unsigned char t4[] = {0x05};
        printf("Transmission 4: Sende 0x05 mit Space-Parität...\n");
        gpioWrite(TX_ENABLE_GPIO, 1);
        wave = buildMultiByteWave(t4, 1, 0);
        gpioWaveTxSend(wave, PI_WAVE_MODE_ONE_SHOT);
        while (gpioWaveTxBusy()) gpioDelay(100);
        gpioWaveDelete(wave);
        gpioWrite(TX_ENABLE_GPIO, 0);

//#############################################################################################

        printf("Warte auf ACK ...\n");
        if (!wait_for_ack(uart_fd, 3))
            printf("Warnung: Kein ACK erhalten!\n");

        // Kurze Pause von 1 ms nach ACK
        gpioDelay(600);

//#############################################################################################

        printf("Sende Nachricht mit Space-Parität: ");
        for (int i = 0; i < 9; i++) printf("%02X ", full_msg[i]);
        printf("\n");

        for (int i = 0; i < 9; i++) {
            gpioWrite(TX_ENABLE_GPIO, 1);
            int waveId = buildMultiByteWave(&full_msg[i], 1, 0);
            gpioWaveTxSend(waveId, PI_WAVE_MODE_ONE_SHOT);
            while (gpioWaveTxBusy()) gpioDelay(100);
            gpioWaveDelete(waveId);
            gpioWrite(TX_ENABLE_GPIO, 0);
            gpioDelay(50); // 50 ms Pause zwischen Bytes
        }

//#############################################################################################

        printf("Warte auf ACK ...\n");
        if (!wait_for_ack(uart_fd, 3))
            printf("Warnung: Kein ACK erhalten!\n");

        // Kurze Pause von 1 ms nach ACK
        gpioDelay(100);

//#############################################################################################

        // Sende 03 mit Space-Parität
        unsigned char t5[] = {0x03};
        printf("Transmission 5: Sende 0x03 mit Space-Parität...\n");
        gpioWrite(TX_ENABLE_GPIO, 1);
        wave = buildMultiByteWave(t5, 1, 0);
        gpioWaveTxSend(wave, PI_WAVE_MODE_ONE_SHOT);
        while (gpioWaveTxBusy()) gpioDelay(100);

        gpioDelay(500);

        gpioWaveDelete(wave);
        gpioWrite(TX_ENABLE_GPIO, 0);

//#############################################################################################

        printf("Warte auf 0x00 0xF9 ...\n");
        if (!wait_for_sequence(uart_fd, 3))
            printf("Warnung: Kein 00 F9  erhalten!\n");

        gpioDelay(500);

//#############################################################################################

        // Sende 06 mit Space-Parität
        unsigned char t6[] = {0x06};
        printf("Transmission 6: Sende 0x06 mit Space-Parität...\n");
        gpioWrite(TX_ENABLE_GPIO, 1);
        wave = buildMultiByteWave(t6, 1, 0);
        gpioWaveTxSend(wave, PI_WAVE_MODE_ONE_SHOT);
        while (gpioWaveTxBusy()) gpioDelay(100);
        gpioWaveDelete(wave);
        gpioWrite(TX_ENABLE_GPIO, 0);

//#############################################################################################

    }

    gpioTerminate();
    close(uart_fd);
    MQTTClient_disconnect(client, TIMEOUT);
    MQTTClient_destroy(&client);
    return 0;
}
