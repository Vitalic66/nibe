#!/usr/bin/env python3
import serial
import argparse

# Erwartete Byte-Sequenz vor dem Senden (komplette Übereinstimmung)
EXPECTED_SEQUENCE = bytes.fromhex("A0 00 59 02 26 3E E3 06 03 00 F9 06 03")

def calculate_crc(msg):
    """ Berechnet das XOR-CRC über alle Bytes außer dem letzten """
    crc = 0
    for byte in msg:
        crc ^= byte
    return crc

def wait_for_sequence(ser):
    """ Wartet, bis die erwartete Sequenz empfangen wurde """
    print("🔍 Warte auf die Sequenz:", EXPECTED_SEQUENCE.hex(' '))
    buffer = bytearray()

    while True:
        byte = ser.read(1)  # Ein einzelnes Byte lesen
        if not byte:
            continue  # Falls Timeout oder keine Daten -> weiter warten

        buffer.append(byte[0])

        # Prüfen, ob das Ende der erwarteten Sequenz erreicht wurde
        if len(buffer) > len(EXPECTED_SEQUENCE):
            buffer.pop(0)  # Erstes Element entfernen, um Fenstergröße konstant zu halten

        if bytes(buffer) == EXPECTED_SEQUENCE:
            print("✅ Erwartete Sequenz empfangen!")
            return True

def send_message(serial_mark, serial_space, message_bytes):
    """ Sendet eine Byte-Nachricht über zwei serielle Schnittstellen mit spezifischen Paritäten """

    try:
        # Öffne `/dev/ttyUSB0` für PARITY_MARK
        with serial.Serial(serial_mark, 19200, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_MARK, timeout=3) as ser_m:
            # Warte auf Sequenz
            if not wait_for_sequence(ser_m):
                return

            # 1️⃣ Senden der ersten beiden Bytes "00 F9" mit PARITY_MARK
            ser_m.write(bytes.fromhex("00 F9"))
            ser_m.flush()
            print(f"📤 Gesendet mit PARITY_MARK auf {serial_mark}: 00 F9")

        # 2️⃣ Senden des Bytes "06" mit PARITY_SPACE über `/dev/ttyUSB1`
        with serial.Serial(serial_space, 19200, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_SPACE, timeout=3) as ser_s:
            ser_s.write(bytes.fromhex("06"))
            ser_s.flush()
            print(f"📤 Gesendet mit PARITY_SPACE auf {serial_space}: 06")

            # 3️⃣ Senden der Nachricht mit CRC mit PARITY_SPACE
            ser_s.write(message_bytes)
            ser_s.flush()
            print(f"📤 Nachricht mit PARITY_SPACE gesendet auf {serial_space}: {message_bytes.hex(' ')}")

            # 4️⃣ Senden des zweiten "06" mit PARITY_SPACE
            ser_s.write(bytes.fromhex("06"))
            ser_s.flush()
            print(f"📤 Gesendet mit PARITY_SPACE auf {serial_space}: 06")

        # 5️⃣ Senden des letzten Bytes "03" mit PARITY_MARK über `/dev/ttyUSB0`
        with serial.Serial(serial_mark, 19200, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_MARK, timeout=3) as ser_m:
            ser_m.write(bytes.fromhex("03"))
            ser_m.flush()
            print(f"📤 Gesendet mit PARITY_MARK auf {serial_mark}: 03")

    except Exception as e:
        print(f"❌ Fehler beim Senden der Nachricht: {e}")

def main():
    parser = argparse.ArgumentParser(description="Sendet eine Byte-Nachricht mit CRC über zwei serielle Schnittstellen.")
    parser.add_argument("--port_mark", type=str, default="/dev/ttyUSB0", help="Serieller Port für PARITY_MARK (Standard: /dev/ttyUSB0)")
    parser.add_argument("--port_space", type=str, default="/dev/ttyUSB1", help="Serieller Port für PARITY_SPACE (Standard: /dev/ttyUSB1)")
    parser.add_argument("--message", type=str, required=True, help="Nachricht als Hex-String, z.B. '55 00 59 02 C2 2E E2'")

    args = parser.parse_args()

    # Umwandlung von Hex-String zu Bytearray
    try:
        msg = bytes.fromhex(args.message)
    except ValueError:
        print("❌ Fehler: Die Nachricht muss ein gültiger Hex-String sein.")
        return

    # Überprüfung, ob das vierte Byte mit der Nachricht übereinstimmt
    if len(msg) < 4:
        print("❌ Fehler: Die Nachricht muss mindestens 4 Byte lang sein.")
        return

    length_from_msg = msg[3]
    actual_length = len(msg) - 1  # Ohne CRC

    if length_from_msg != actual_length:
        print(f"⚠️ Warnung: Das vierte Byte ({length_from_msg}) passt nicht zur tatsächlichen Länge ({actual_length}). Wird angepasst.")

    # CRC berechnen und anfügen (über alle Bytes)
    crc = calculate_crc(msg)
    message_with_crc = msg + bytes([crc])

    print(f"✅ CRC berechnet: {crc:02X}")
    print(f"📤 Nachricht zum Senden bereit: {message_with_crc.hex(' ')}")

    # Nachricht senden (wenn Sequenz empfangen wurde)
    send_message(args.port_mark, args.port_space, message_with_crc)

if __name__ == "__main__":
    main()
