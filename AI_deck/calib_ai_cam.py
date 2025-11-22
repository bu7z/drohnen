import cv2
import numpy as np
import glob
import os

# --- DEINE KONFIGURATION ---
# Innere Ecken (Breite, Höhe) -> Du hast 8x7 Felder, also 7x6 innere Punkte
CHECKERBOARD = (7, 6) 
# Größe eines Quadrats in Metern (WICHTIG für korrekte Distanzmessung!)
SQUARE_SIZE = 0.023  # 2.3 cm

# Abbruchkriterien für den Sub-Pixel Algorithmus (für höhere Präzision)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 3D Punkte in der echten Welt vorbereiten (0,0,0), (1,0,0), (2,0,0) ...
# Wir nehmen an, das Brett ist flach (z=0)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# Listen für Ergebnisse
objpoints = [] # 3D Punkte im Raum
imgpoints = [] # 2D Punkte im Bild

# Bilder laden
images = glob.glob("calib_images/*.png")
print(f"Habe {len(images)} Bilder gefunden. Starte Analyse...")

found = 0
for fname in images:
    img = cv2.imread(fname)
    if img is None:
        continue
        
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Schachbrett finden
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret == True:
        found += 1
        objpoints.append(objp)

        # Ecken verfeinern (Subpixel Genauigkeit)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Optional: Feedback zeichnen
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('Calibration Check', img)
        cv2.waitKey(50) # Kurz anzeigen
    else:
        print(f"WARNUNG: Muster in {fname} nicht gefunden (oder nicht vollständig).")

cv2.destroyAllWindows()

if found < 10:
    print("\nFEHLER: Zu wenig Bilder erfolgreich erkannt! Mach mehr/bessere Fotos.")
    exit()

print(f"\nKalibrierung läuft mit {found} Bildern...")

# Die eigentliche Magie: Matrix berechnen
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("\n" + "="*40)
print(f"KALIBRIERUNG ERFOLGREICH!")
print(f"Durchschnittlicher Fehler (RMS): {ret:.4f}")
print("="*40)

if ret > 1.0:
    print("ACHTUNG: Der Fehler ist recht hoch (> 1 Pixel). Distanzmessung könnte ungenau sein.")
else:
    print("SUPER: Der Fehler ist sehr klein. Das wird präzise!")

print("\nKameramatrix (mtx):\n", mtx)
print("\nVerzerrungskoeffizienten (dist):\n", dist)

# Speichern für später
np.savez("camera_params.npz", mtx=mtx, dist=dist)
print("\nGespeichert als 'camera_params.npz'.")
print("Du kannst jetzt mit dem Pose-Test weitermachen.")