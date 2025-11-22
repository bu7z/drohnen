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
MAX_IMAGE_SIZE = 100000  # 100KB Limit

if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

print(f"Verbinde zu {IP}:{PORT}...")

def connect_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
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
            if not chunk: return None
            data.extend(chunk)
        except socket.timeout:
            return None
    return data

client_socket = connect_socket()
img_count = 0

print("--- STEUERUNG ---")
print(" [ENTER] : Foto speichern")
print(" [q]     : Beenden")
print("-----------------")

while True:
    if client_socket is None:
        time.sleep(2)
        client_socket = connect_socket()
        continue

    try:
        # 1. Info Header
        packetInfoRaw = rx_bytes(client_socket, 4)
        if packetInfoRaw is None: raise ConnectionError("Keine Daten")
        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)

        if length > 1000 or length < 2:
            client_socket.close()
            client_socket = None
            continue

        # 2. Bild Header
        imgHeader = rx_bytes(client_socket, length - 2)
        if imgHeader is None: raise ConnectionError("Header incomplete")
        [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

        if magic != 0xBC or size > MAX_IMAGE_SIZE:
            client_socket.close()
            client_socket = None
            continue

        # 3. Bilddaten
        imgStream = bytearray()
        remaining = size
        failed = False
        while remaining > 0:
            chunkInfo = rx_bytes(client_socket, 4)
            if chunkInfo is None: 
                failed = True
                break
            [chunkLen, dst, src] = struct.unpack('<HBB', chunkInfo)
            
            chunkData = rx_bytes(client_socket, chunkLen - 2)
            if chunkData is None:
                failed = True
                break
            
            imgStream.extend(chunkData)
            remaining -= (chunkLen - 2)

        if failed: raise ConnectionError("Chunk Error")

        # 4. Dekodieren
        frame = None
        if format == 0: # RAW
            bayer_img = np.frombuffer(imgStream, dtype=np.uint8)
            if bayer_img.size == width * height:
                bayer_img.shape = (height, width)
                frame = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGR)
        else: # JPEG
            nparr = np.frombuffer(imgStream, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        # 5. Anzeigen (Clean)
        if frame is not None:
            # 2x Zoom für bessere Sichtbarkeit, aber OHNE Text
            display = cv2.resize(frame, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
            
            cv2.imshow('Clean Stream', display)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == 13: # ENTER
                fname = os.path.join(OUTPUT_DIR, f"img_{img_count:03d}.png")
                # Speichert das saubere Original
                cv2.imwrite(fname, frame)
                print(f" -> Bild {img_count} gespeichert: {fname}")
                img_count += 1
                
                # Visuelles Feedback: Weißer Blitz (Invertieren)
                cv2.imshow('Clean Stream', 255 - display)
                cv2.waitKey(50)

    except (ConnectionError, struct.error, socket.timeout):
        if client_socket: client_socket.close()
        client_socket = None
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(f"Error: {e}")
        if client_socket: client_socket.close()
        client_socket = None

if client_socket: client_socket.close()
cv2.destroyAllWindows()