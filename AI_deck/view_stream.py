import socket
import struct
import cv2
import numpy as np
import os
import time

# --- CONFIG ---
IP = "192.168.4.1"
PORT = 5000
OUTPUT_DIR = "calib_images"
MAX_IMAGE_SIZE = 100000  # Maximale erlaubte Bildgröße in Bytes (100KB reicht locker für 320x240)

if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

print(f"Verbinde zu {IP}:{PORT}...")

def connect_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0) # Wichtig: Timeout, damit wir nicht ewig hängen
    try:
        s.connect((IP, PORT))
        print("Verbunden!")
        return s
    except:
        print("Verbindung fehlgeschlagen, versuche es gleich nochmal...")
        return None

def rx_bytes(sock, size):
    data = bytearray()
    while len(data) < size:
        try:
            chunk = sock.recv(size - len(data))
            if not chunk:
                return None # Verbindung weg
            data.extend(chunk)
        except socket.timeout:
            return None
    return data

client_socket = connect_socket()
img_count = 0

while True:
    # Auto-Reconnect Logik
    if client_socket is None:
        time.sleep(2)
        client_socket = connect_socket()
        continue

    try:
        # 1. Info Header lesen (4 Bytes)
        packetInfoRaw = rx_bytes(client_socket, 4)
        if packetInfoRaw is None: raise ConnectionError("Keine Daten")
        
        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

        # SANITY CHECK 1: Ist die Header-Länge plausibel?
        if length > 1000 or length < 2:
            print(f"Warnung: Header Länge unplausibel ({length}). Resync...")
            client_socket.close()
            client_socket = None
            continue

        # 2. Bild Header lesen
        imgHeader = rx_bytes(client_socket, length - 2)
        if imgHeader is None: raise ConnectionError("Header unvollständig")
        
        [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

        # SANITY CHECK 2: Magic Byte und Bildgröße
        if magic != 0xBC:
            print("Warnung: Falsches Magic Byte. Resync...")
            client_socket.close()
            client_socket = None
            continue
            
        if size > MAX_IMAGE_SIZE:
            print(f"ALARM: Angebliches Bild ist {size/1024:.2f} KB groß. Das ist ein Fehler! Ignoriere...")
            # Wir müssen die Verbindung trennen, um den Müll aus dem Puffer zu kriegen
            client_socket.close()
            client_socket = None
            continue

        # 3. Bilddaten lesen (Nur wenn Größe okay ist!)
        imgStream = bytearray()
        # Hier laden wir die Daten in Chunks
        remaining = size
        failed = False
        while remaining > 0:
            # Info für den Chunk
            chunkInfo = rx_bytes(client_socket, 4)
            if chunkInfo is None: 
                failed = True
                break
            [chunkLen, dst, src] = struct.unpack('<HBB', chunkInfo)
            
            # Daten des Chunks
            chunkData = rx_bytes(client_socket, chunkLen - 2)
            if chunkData is None:
                failed = True
                break
            
            imgStream.extend(chunkData)
            remaining -= (chunkLen - 2)

        if failed:
            raise ConnectionError("Chunk Fehler")

        # 4. Umwandlung
        frame = None
        if format == 0: # RAW Bayer
            bayer_img = np.frombuffer(imgStream, dtype=np.uint8)
            if bayer_img.size == width * height:
                bayer_img.shape = (height, width)
                frame = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGR)
        else: # JPEG
            nparr = np.frombuffer(imgStream, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        # 5. Anzeigen
        if frame is not None:
            # Optional: Leicht vergrößern für bessere Sichtbarkeit (ohne Absturzrisiko)
            display = cv2.resize(frame, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
            cv2.putText(display, f"Saved: {img_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            
            cv2.imshow('Robust Stream', display)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == 13: # ENTER
                fname = os.path.join(OUTPUT_DIR, f"img_{img_count:03d}.png")
                # WICHTIG: Wir speichern das ORIGINAL (frame), nicht die vergrößerte Version!
                cv2.imwrite(fname, frame)
                print(f"Gespeichert: {fname}")
                img_count += 1
                # Feedback
                cv2.imshow('Robust Stream', 255 - display)
                cv2.waitKey(50)

    except (ConnectionError, struct.error, socket.timeout) as e:
        print(f"Stream Fehler ({e}). Reconnect...")
        if client_socket: client_socket.close()
        client_socket = None
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(f"Unerwarteter Fehler: {e}")
        if client_socket: client_socket.close()
        client_socket = None

if client_socket: client_socket.close()
cv2.destroyAllWindows()