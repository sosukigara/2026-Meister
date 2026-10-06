# TEMP spin-check UI for the M0 sensorless firmware (O/N commands only).
# Delete together with the N command. Opens the serial port (resets the board:
# motor stops), so connect only with the winch free and the 24 V supply current-limited.
import queue
import threading
import tkinter as tk

import serial

import glob

# The ESP32 re-enumerates between ttyUSB0 and ttyUSB1 across replugs, so do not pin one.
_cands = sorted(glob.glob("/dev/ttyUSB*"))
PORT = _cands[0] if _cands else "/dev/ttyUSB0"
BAUD = 115200


class UI:
    def __init__(self):
        self.ser = None
        self.rx = queue.Queue()
        self.root = tk.Tk()
        self.root.title("TEMP winch spin check (M0 sensorless)")
        self.status = tk.Label(self.root, text="disconnected", fg="red")
        self.status.pack()
        row = tk.Frame(self.root)
        row.pack()
        tk.Label(row, text="port").pack(side=tk.LEFT)
        self.port = tk.Entry(row, width=14)
        self.port.insert(0, PORT)
        self.port.pack(side=tk.LEFT)
        tk.Button(row, text="connect", command=self.connect).pack(side=tk.LEFT)
        self.speed = tk.Scale(self.root, from_=-3.0, to=3.0, resolution=0.1,
                              orient=tk.HORIZONTAL, length=300, label="rev/s (O, max 3)")
        self.speed.pack()
        self.speed.bind("<ButtonRelease-1>", lambda _e: self.send_ovel())
        tk.Label(self.root, text="SAFE: 0.3A cap, 10s auto-stop, 10s cooldown",
                 fg="dark green").pack()
        brow = tk.Frame(self.root)
        brow.pack()
        tk.Button(brow, text="run O", command=self.send_ovel).pack(side=tk.LEFT)
        tk.Button(brow, text="0速", command=self.zero).pack(side=tk.LEFT)
        tk.Label(brow, text="sine amp").pack(side=tk.LEFT)
        self.amp = tk.Entry(brow, width=5)
        self.amp.insert(0, "1")
        self.amp.pack(side=tk.LEFT)
        tk.Button(brow, text="run N", command=self.send_sine).pack(side=tk.LEFT)
        tk.Button(brow, text="STOP (S)", command=self.stop).pack(side=tk.LEFT)
        grow = tk.Frame(self.root)  # TEMP G
        grow.pack()
        tk.Label(grow, text="angle rad").pack(side=tk.LEFT)
        self.angle = tk.Entry(grow, width=7)
        self.angle.insert(0, "3.14")
        self.angle.pack(side=tk.LEFT)
        tk.Button(grow, text="go G", command=self.send_angle).pack(side=tk.LEFT)
        tk.Button(grow, text="diag D", command=self.send_diag).pack(side=tk.LEFT)  # TEMP D
        self.tele = tk.Label(self.root, text="vel=--- angle=--- mode=--- vq=---",
                             font=("monospace", 11))
        self.tele.pack()
        self.log = tk.Text(self.root, height=8, width=60)
        self.log.pack()
        self.root.after(100, self.drain)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def connect(self):
        try:
            self.ser = serial.Serial(self.port.get().strip(), BAUD, timeout=0.1)
        except Exception as e:  # noqa: BLE001 - show it, nothing to recover
            self.log.insert(tk.END, f"connect failed: {e}\n")
            return
        self.status.config(text="connected (board was reset)", fg="green")
        threading.Thread(target=self.reader, daemon=True).start()

    def reader(self):
        while self.ser is not None:
            try:
                line = self.ser.readline()
            except Exception:  # noqa: BLE001 - port gone, drop the thread
                return
            if line:
                self.rx.put(line.decode("utf-8", "replace").rstrip())

    def send(self, text):
        if self.ser is None:
            self.log.insert(tk.END, "not connected\n")
            return
        self.ser.write((text + "\n").encode())

    def send_ovel(self):
        self.send(f"O {self.speed.get():.1f}")

    def zero(self):
        self.speed.set(0.0)
        self.send("O 0")

    def send_sine(self):
        self.send(f"N {self.amp.get().strip()}")

    def send_angle(self):  # TEMP G
        self.send(f"G {self.angle.get().strip()}")

    def send_diag(self):  # TEMP D
        self.send("D")

    def stop(self):
        self.speed.set(0.0)
        self.send("S")

    def drain(self):
        while True:
            try:
                line = self.rx.get_nowait()
            except queue.Empty:
                break
            if line.startswith("#"):
                self.log.insert(tk.END, line + "\n")
                self.log.see(tk.END)
            else:
                f = line.split(",")
                if len(f) == 8:
                    self.tele.config(
                        text=f"vel={f[2]}rps angle={f[1]}rad mode={f[4]} vq={f[6]}V")
        self.root.after(100, self.drain)

    def close(self):
        try:
            if self.ser is not None:
                self.ser.write(b"S\n")
                self.ser.close()
        finally:
            self.ser = None
            self.root.destroy()


UI().root.mainloop()
