#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>

#define SERIAL_PORT "/dev/ttyUSB0"
#define BUFFER_SIZE 13 // Länge der erwarteten Sequenz
#define TIMEOUT_MS 70  // Wartezeit nach Sequenz-Erkennung in Millisekunden

// Erwartete Sequenz mit Wildcards an Position 5,6,7
unsigned char expected_sequence[BUFFER_SIZE] = {0xA0, 0x00, 0x59, 0x02, 0x00, 0x00, 0x00, 0x06, 0x03, 0x00, 0xF9, 0x06, 0x03};

// Funktion zum Setzen der Parität (MARK oder SPACE)
void set_serial_parity(int fd, int mark) {
    struct termios options;
    tcgetattr(fd, &options);

    if (mark) {
        options.c_cflag |= PARENB;  // Parity aktivieren
        options.c_cflag |= PARODD;  // MARK Parität
    } else {
        options.c_cflag |= PARENB;  // Parity aktivieren
        options.c_cflag &= ~PARODD; // SPACE Parität
    }

    tcsetattr(fd, TCSANOW, &options);
    usleep(5000); // **5-ms-Verzögerung nach Paritätswechsel**
}

// Funktion zum Senden von Bytes mit der passenden Parität
void send_bytes(int fd, unsigned char *bytes, int len, int mark_first, int mark_last) {
    for (int i = 0; i < len; i++) {
        if ((mark_first && i < 2) || (mark_last && i == len - 1)) {
            set_serial_parity(fd, 1);  // Parity Mark für erste zwei Vorbytes & letztes Nachbyte
        } else {
            set_serial_parity(fd, 0);  // Parity Space für alle anderen
        }

        write(fd, &bytes[i], 1);
        tcdrain(fd);  // Blockieren, bis das Byte gesendet wurde
        usleep(1000); // Kleine Verzögerung für Stabilität
    }
}

// XOR-Checksumme berechnen
unsigned char calculate_xor_checksum(unsigned char *data, int length) {
    unsigned char checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Funktion zum Überprüfen der Sequenz mit Wildcards
int check_sequence(unsigned char *buffer) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (i >= 4 && i <= 6) continue;  // Wildcards für Position 5,6,7
        if (buffer[i] != expected_sequence[i]) {
            return 0;
        }
    }
    return 1;
}

// **Funktion, die wartet, bis für mindestens 70 ms kein Byte empfangen wurde**
void wait_for_quiet(int fd) {
    struct timeval start, now;
    gettimeofday(&start, NULL);

    while (1) {
        unsigned char byte;
        int n = read(fd, &byte, 1);
        gettimeofday(&now, NULL);

        // Falls ein Byte empfangen wurde -> Timer zurücksetzen
        if (n > 0) {
            gettimeofday(&start, NULL);
        }

        // Differenz in Millisekunden berechnen
        long elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 + (now.tv_usec - start.tv_usec) / 1000;

        if (elapsed_ms >= TIMEOUT_MS) {
            printf("\u23f3 70 ms Stille erkannt. Nachricht wird gesendet...\n");
            return;
        }
    }
}

// Funktion, die auf die erwartete Sequenz wartet
int wait_for_sequence(int fd) {
    unsigned char buffer[BUFFER_SIZE] = {0};
    int index = 0;

    while (1) {
        unsigned char byte;
        int n = read(fd, &byte, 1);
        if (n > 0) {
            // FIFO-Puffer aktualisieren
            memmove(buffer, buffer + 1, BUFFER_SIZE - 1);
            buffer[BUFFER_SIZE - 1] = byte;

            printf("\ud83d\udce5 Empfangenes Byte: %02X (Index %d)\n", byte, index++);

            // Sequenzprüfung
            if (check_sequence(buffer)) {
                printf("\u2705 Erwartete Sequenz empfangen!\n");
                wait_for_quiet(fd);
                return 1;
            }
        }
    }
    return 0;
}

int main() {
    int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("\u274c Fehler: Serieller Port konnte nicht geöffnet werden");
        return 1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);
    options.c_cflag |= CS8 | CLOCAL | CREAD; // 8 Bit, Lokale Steuerung, Empfang aktiviert
    tcsetattr(fd, TCSANOW, &options);

    if (wait_for_sequence(fd)) {
        unsigned char message[] = {0xD0, 0x00, 0x59, 0x07, 0x25, 0x03, 0x03, 0x10, 0x50, 0x00, 0x01};
        unsigned char checksum = calculate_xor_checksum(message, sizeof(message));
        unsigned char final_message[20];
        int final_len = 0;

        memcpy(final_message, message, sizeof(message));
        final_len += sizeof(message);
        final_message[final_len++] = checksum;
        final_message[final_len++] = 0x06;
        final_message[final_len++] = 0x03;

        send_bytes(fd, final_message, final_len, 1, 1);
    }

    close(fd);
    return 0;
}
