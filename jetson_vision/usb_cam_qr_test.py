"""USB摄像头 QR 快速测试（Windows + OpenCV）"""
import cv2
import time

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

detector = cv2.QRCodeDetector()
last_qr = None
last_qr_n = 0
last_send_ms = 0
cooldown_ms = 2000

print("USB Camera QR Test (640x480)")
print("Hold QR code in front of camera. Press ESC to quit.\n")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    data, pts, _ = detector.detectAndDecode(gray)

    now = int(time.time() * 1000)
    if pts is not None and data:
        data = data.strip()
        if data in ("1","2","3","4","5","6"):
            w = int(max(pts[:,0,1]) - min(pts[:,0,1]))
            h = int(max(pts[:,0,0]) - min(pts[:,0,0]))
            box = pts.reshape(-1,2).astype(int)
            cv2.polylines(frame, [box], True, (0,255,0), 2)
            cv2.putText(frame, f"QR {data} ({w}x{h})", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

            if data == last_qr:
                last_qr_n += 1
            else:
                last_qr = data
                last_qr_n = 1

            if last_qr_n >= 1 and (now - last_send_ms) >= cooldown_ms:
                print(f"  >>> SEND action={data}  size={w}x{h}")
                last_send_ms = now
                last_qr_n = 0
        else:
            last_qr = None
            last_qr_n = 0
    else:
        cv2.putText(frame, "No QR", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        last_qr = None
        last_qr_n = 0

    cv2.imshow("USB Camera QR Test", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
