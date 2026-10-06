# TEMP spin-check UI for the M1 sensorless firmware (O/N/G commands only).
# Delete together with the N and G commands. Opening the serial port resets the board
# (the motor stops), so connect only with the winch free and the 24 V supply
# current-limited.
import glob
import queue
import threading
import tkinter as tk

import serial

# The ESP32 re-enumerates between ttyUSB0 and ttyUSB1 across replugs, so do not pin one.
_cands = sorted(glob.glob("/dev/ttyUSB*"))
PORT = _cands[0] if _cands else "/dev/ttyUSB0"
BAUD = 115200

SAFETY = (
    "実行前にベンチ電源のCC(電流制限)を 0.5A にすること。\n"
    "ファーム側のソフト過電流チョップ(0.5A / #OC)は保険であって保証ではない。\n"
    "各実行は10秒で自動停止、停止後10秒は再実行を拒否(クールダウン)。\n"
    "シリアルを開くとESP32がリセットし回転が止まる。回転中に接続しないこと。\n"
    "速度0は S(停止) を送る。O 0 は DC 注入で発熱するので使わない。"
)


class UI:
    def __init__(self):
        self.ser = None
        self.rx = queue.Queue()
        self.root = tk.Tk()
        self.root.title("winch spin check — MKS ESP32 FOC M1 (sensorless)")

        row = tk.Frame(self.root)
        row.pack(fill=tk.X, padx=6, pady=4)
        tk.Label(row, text="port").pack(side=tk.LEFT)
        self.port = tk.Entry(row, width=14)
        self.port.insert(0, PORT)
        self.port.pack(side=tk.LEFT)
        tk.Button(row, text="接続", command=self.connect).pack(side=tk.LEFT)
        self.status = tk.Label(row, text="未接続", fg="red")
        self.status.pack(side=tk.LEFT)

        tk.Label(self.root, text=SAFETY, fg="dark red", justify=tk.LEFT,
                 font=("sans-serif", 9)).pack(fill=tk.X, padx=6)

        v = tk.LabelFrame(self.root, text="速度 (O, rev/s)")
        v.pack(fill=tk.X, padx=6, pady=3)
        self.speed = tk.Scale(v, from_=-5.0, to=5.0, resolution=0.1,
                              orient=tk.HORIZONTAL, length=380)
        self.speed.pack(fill=tk.X)
        self.speed.bind("<ButtonRelease-1>", lambda _e: self.on_slider())
        vrow = tk.Frame(v)
        vrow.pack(fill=tk.X)
        tk.Label(vrow, text="値").pack(side=tk.LEFT)
        self.speed_entry = tk.Entry(vrow, width=7)
        self.speed_entry.insert(0, "1.0")
        self.speed_entry.pack(side=tk.LEFT)
        tk.Button(vrow, text="回す (O)", command=self.send_ovel).pack(side=tk.LEFT)
        tk.Button(vrow, text="0 は停止 (S)", command=self.stop).pack(side=tk.LEFT)

        a = tk.LabelFrame(self.root, text="角度 (G, rad)")
        a.pack(fill=tk.X, padx=6, pady=3)
        self.angle = tk.Scale(a, from_=0.0, to=6.28, resolution=0.01,
                              orient=tk.HORIZONTAL, length=380)
        self.angle.pack(fill=tk.X)
        arow = tk.Frame(a)
        arow.pack(fill=tk.X)
        tk.Label(arow, text="値").pack(side=tk.LEFT)
        self.angle_entry = tk.Entry(arow, width=7)
        self.angle_entry.insert(0, "3.14")
        self.angle_entry.pack(side=tk.LEFT)
        tk.Button(arow, text="角度 (G)", command=self.send_angle).pack(side=tk.LEFT)

        brow = tk.Frame(self.root)
        brow.pack(fill=tk.X, padx=6, pady=3)
        tk.Label(brow, text="sine amp").pack(side=tk.LEFT)
        self.amp = tk.Entry(brow, width=5)
        self.amp.insert(0, "1")
        self.amp.pack(side=tk.LEFT)
        tk.Button(brow, text="サイン (N)", command=self.send_sine).pack(side=tk.LEFT)
        tk.Button(brow, text="診断 (D)", command=self.send_diag).pack(side=tk.LEFT)
        tk.Button(brow, text="■ STOP (S)", command=self.stop, fg="white", bg="red",
                  font=("sans-serif", 11, "bold")).pack(side=tk.RIGHT, ipadx=12)

        self.tele = tk.Label(self.root, text="vel=--- angle=--- mode=--- vq=---",
                             font=("monospace", 11))
        self.tele.pack(fill=tk.X, padx=6)
        self.log = tk.Text(self.root, height=10, width=66)
        self.log.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)

        self.root.after(100, self.drain)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def connect(self):
        try:
            self.ser = serial.Serial(self.port.get().strip(), BAUD, timeout=0.1)
        except Exception as e:  # noqa: BLE001 - show it, nothing to recover
            self.log.insert(tk.END, f"connect failed: {e}\n")
            return
        self.status.config(text="接続済み (基板はリセットされた)", fg="green")
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
            self.log.insert(tk.END, "未接続\n")
            return
        self.ser.write((text + "\n").encode())

    def _speed_value(self):
        try:
            return float(self.speed_entry.get().strip())
        except ValueError:
            return self.speed.get()

    def _angle_value(self):
        try:
            r = float(self.angle_entry.get().strip())
        except ValueError:
            r = self.angle.get()
        return max(0.0, min(6.28, r))

    def on_slider(self):
        self.speed_entry.delete(0, tk.END)
        self.speed_entry.insert(0, f"{self.speed.get():.1f}")
        self.send_ovel()

    def send_ovel(self):
        v = self._speed_value()
        if abs(v) < 1e-6:
            self.log.insert(tk.END, "0 = 停止 (S)\n")
            self.stop()
            return
        self.speed.set(v)
        self.send(f"O {v:.1f}")

    def send_angle(self):
        r = self._angle_value()
        self.angle_entry.delete(0, tk.END)
        self.angle_entry.insert(0, f"{r:.3f}")
        self.angle.set(r)
        self.send(f"G {r:.3f}")

    def send_sine(self):
        self.send(f"N {self.amp.get().strip()}")

    def send_diag(self):  # TEMP D
        self.send("D")

    def stop(self):
        self.speed.set(0.0)
        self.speed_entry.delete(0, tk.END)
        self.speed_entry.insert(0, "0.0")
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
