import cv2
import picamera2
import numpy as np
from flask import Flask, render_template, Response

app = Flask(__name__)

camera = picamera2.Picamera2()
camera.start()

def generate_frames():
    while True:
        image = camera.capture_array()
        
        ret, buffer = cv2.imencode(".jpg", image)

        if not ret:
            print("Failed to encode frame")
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"
        )


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)