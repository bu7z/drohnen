import socket
import argparse
import numpy as np
import cv2
import os
import time  # Für FPS

parser = argparse.ArgumentParser(description='Capture calibration images from AI-deck')
parser.add_argument("-n", default="192.168.4.1", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default=5000, metavar="port", help="AI-deck port")
parser.add_argument("-o", default="calib_images", metavar="out", help="Output directory")
args = parser.parse_args()

ip = args.n
port = args.p
out_dir = args.o
os.makedirs(out_dir, exist_ok=True)

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.bind(('', port))
client_socket.settimeout(5.0)

print(f"Verbinde zu AI-deck Stream {ip}:{port}")
print("Drücke ENTER zum Speichern, q zum Beenden.")

img_id = 0
buffer = b""
start_time = time.time()
frame_count = 0

while True:
    try:
        data, _ = client_socket.recvfrom(2048)
        buffer += data
        
        # Buffer-Limit, um Overflow zu vermeiden
        if len(buffer) > 100000:
            buffer = b""
            print("Buffer overflow cleared")
        
        if b'\xff\xd9' in buffer:
            jpg_end = buffer.index(b'\xff\xd9') + 2
            jpg = buffer[:jpg_end]
            buffer = buffer[jpg_end:]
            
            img_array = np.frombuffer(jpg, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)  # Oder IMREAD_GRAYSCALE für Kalibrierung
            
            if frame is not None:
                disp = frame.copy()
                cv2.putText(disp, f"Bild {img_id}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # FPS anzeigen
                frame_count += 1
                if time.time() - start_time > 1:
                    fps = frame_count / (time.time() - start_time)
                    print(f"FPS: {fps:.2f}")
                    start_time = time.time()
                    frame_count = 0
                
                cv2.imshow("AI-Deck Stream", disp)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == 13:  # ENTER
                    fname = os.path.join(out_dir, f"img_{img_id:03d}.png")
                    cv2.imwrite(fname, frame)
                    print(f"Gespeichert: {fname}")
                    img_id += 1
    except socket.timeout:
        print("Warte auf Frames...")
    except Exception as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        break

client_socket.close()
cv2.destroyAllWindows()
