import cv2
import ffmpeg
import numpy as np
from flask import Flask, render_template, Response

app = Flask(__name__)


def generate_frames():
    process = (
        ffmpeg.input("/tmp/video_stream.jpg")
        .output("pipe:1", format="rawvideo", pix_fmt="bgr24")
        .run_async(pipe_stdout=True, pipe_stderr=True)
    )

    while True:
        in_bytes = process.stdout.read(640 * 480 * 3)

        if len(in_bytes) < 640 * 480 * 3:
            break

        frame = np.frombuffer(in_bytes, np.uint8).reshape([480, 640, 3])
        ret, buffer = cv2.imencode(".jpg", frame)

        # in_bytes shape: 921600, frame shape: (480, 640, 3), buffer shape: (47269,) but buffer shape always change
        print(
            f"in_bytes shape: {len(in_bytes)}, frame shape: {frame.shape}, buffer shape: {buffer.shape}"
        )

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