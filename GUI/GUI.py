import tkinter as tk
from tkinter import messagebox
from tkintermapview import TkinterMapView
import socket
import json
import math
import threading
from PIL import Image, ImageTk, ImageOps
import os

# Tài khoản mẫu
accounts = {
    "operator": "op123",
    "supervisor": "sv123"
}

current_user = None

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Drone System")
        self.geometry("650x700")

        self.frames = {}
        self.station_offsets = {}
        self.selected_station_ids = []
        self.selected_station_names = []
        self.pi_ip = "10.225.48.96"
        self.pi_port = 5001
        self.home_position = None

        container = tk.Frame(self)
        container.pack(expand=True, fill="both")
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        for F in (LoginPage, MainMenu, HomePage, StationPage):
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame(LoginPage)  # 👈 Trang đầu tiên là LoginPage

    def show_frame(self, page_class):
        frame = self.frames[page_class]
        frame.tkraise()

class LoginPage(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        self.logo_images = []

        # ======== VÙNG LOGO ========
        logo_bg = tk.Frame(self)
        logo_bg.pack(fill="x")

        try:
            # Logo RIC (bên trái)
            ric_img = Image.open("logo1.png").resize((200, 100))
            ric_photo = ImageTk.PhotoImage(ric_img)
            self.logo_images.append(ric_photo)
            tk.Label(logo_bg, image=ric_photo).pack(side="left", padx=20, pady=10)

            # Logo HCMUTE & FIE (bên phải)
            hcmute_fie_frame = tk.Frame(logo_bg)
            hcmute_fie_frame.pack(side="right", padx=20, pady=10)

            for i in [2, 3]:
                img = Image.open(f"logo{i}.png").resize((100, 100))
                photo = ImageTk.PhotoImage(img)
                self.logo_images.append(photo)
                tk.Label(hcmute_fie_frame, image=photo).pack(side="left", padx=5)
        except Exception as e:
            print(f"Lỗi khi load logo: {e}")

        # ======== HÌNH CHỮ NHẬT NGĂN CÁCH (MÀU XANH DƯƠNG) ========
        separator = tk.Frame(self, height=30, bg="#1E90FF")  # DeepSkyBlue
        separator.pack(fill="x", padx=0, pady=(0, 10))

        # ======== TÊN TRƯỜNG ========
        tk.Label(
            self,
            text="HCMC University of Technology and Education",
            font=("Helvetica", 12, "bold"),
        ).pack(pady=(5, 2))

        # ======== TÊN ĐỀ TÀI ========
        title = (
            "DESIGN AND IMPLEMENTATION OF AN AUTONOMOUS DRONE SYSTEM UTILIZING GPS AND "
            "IMAGE PROCESSING FOR OBJECT TRACKING AND OBJECT DELIVERY"
        )
        tk.Label(self, text=title, font=("Helvetica", 12, "bold"), wraplength=580, justify="center").pack(pady=(5, 15))

        # ======== PHẦN THÂN: DRONE + LOGIN ========
        body_frame = tk.Frame(self)
        body_frame.pack(expand=True, fill="both", padx=20, pady=(0, 30))  # ⬅️ tăng chiều cao

        # Trái: hình drone
        left_frame = tk.Frame(body_frame)
        left_frame.pack(side="left", expand=True, ipadx=10, ipady=10)

        try:
            drone_img = Image.open("drone.png").resize((200, 200))
            drone_photo = ImageTk.PhotoImage(drone_img)
            self.logo_images.append(drone_photo)
            tk.Label(left_frame, image=drone_photo).pack()
        except Exception as e:
            print(f"Lỗi drone.png: {e}")

        # Phải: Form login
        right_frame = tk.Frame(body_frame)
        right_frame.pack(side="right", expand=True, padx=40, ipadx=10, ipady=10)

        tk.Label(right_frame, text="Login", font=("Helvetica", 16)).pack(pady=10)

        tk.Label(right_frame, text="Account").pack()
        self.username_entry = tk.Entry(right_frame)
        self.username_entry.pack()

        tk.Label(right_frame, text="Password").pack()
        self.password_entry = tk.Entry(right_frame, show="*")
        self.password_entry.pack()

        tk.Button(right_frame, text="Login", command=self.login).pack(pady=10)

        # ======== TÊN GIẢNG VIÊN & NHÓM DƯỚI CÙNG ========
        tk.Label(self, text="Advisor: Assoc. Prof. PhD. Trần Đức Thiện", font=("Helvetica", 12)).pack(pady=(20, 5))
        tk.Label(
            self,
            text="Student: Phạm Quang Vinh - 21151437 | Đặng Trung Anh - 21151002",
            font=("Helvetica", 12)
        ).pack(pady=(0, 20))

    def login(self):
        global current_user
        username = self.username_entry.get()
        password = self.password_entry.get()

        if username in accounts and accounts[username] == password:
            current_user = username
            self.controller.show_frame(MainMenu)
        else:
            messagebox.showerror("Error", "Account or password is not true")


class MainMenu(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        tk.Label(self, text="Main menu", font=("Helvetica", 16)).pack(pady=20)
        tk.Button(self, text="🏠 Home", width=20, command=lambda: self.controller.show_frame(HomePage)).pack(pady=5)
        tk.Button(self, text="📍 Station", width=20, command=self.go_station).pack(pady=5)
        tk.Button(self, text="← Logout", width=20, command=lambda: self.controller.show_frame(LoginPage)).pack(pady=10)
        
    def go_station(self):
        if current_user == "operator":
            self.controller.show_frame(StationPage)
        else:
            messagebox.showinfo("Announcement", "Access denied")

class HomePage(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        tk.Label(self, text="Home", font=("Helvetica", 14)).pack(pady=5)

        self.map_widget = TkinterMapView(self, width=600, height=450, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_tile_server("https://a.tile.openstreetmap.de/{z}/{x}/{y}.png")
        self.map_widget.set_zoom(16)

        self.info_label = tk.Label(self, text="Altitude: -- | Velocity: --", font=("Helvetica", 12))
        self.info_label.pack(pady=5)

        btn_frame = tk.Frame(self)
        btn_frame.pack()

        tk.Button(btn_frame, text="← Back", command=lambda: self.controller.show_frame(MainMenu)).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Cập nhật vị trí ban đầu", command=self.set_home_position).pack(side="left", padx=5)
        tk.Button(btn_frame, text="🚨", fg="white", bg="red", font=("Helvetica", 10, "bold"),
                  command=self.emergency_action).pack(side="left", padx=5)

        self.home_position = None
        self.station_position = None
        self.station_markers = []
        self.path_line = None
        self.drone_marker = None
        self.drone_position = None

        self.flight_path = []  # ✅ Lưu lại đường bay thực tế của drone

        # Load icon máy bay
        self.drone_icon_base = Image.open("plane.png").resize((40, 40))
        self.drone_icon = ImageTk.PhotoImage(self.drone_icon_base)

        # Load icon trạm và home nhỏ
        self.location_icon_base = Image.open("locationicon.png").resize((25, 25))
        self.location_icon = ImageTk.PhotoImage(self.location_icon_base)

        threading.Thread(target=self.receive_position_loop, daemon=True).start()

    def set_home_position(self):
        print(f"Kiểm tra drone_position hiện tại: {self.drone_position}")
        if self.drone_position:
            lat, lon = self.drone_position
            self.home_position = (lat, lon)
            self.controller.home_position = (lat, lon)
            self.map_widget.set_position(lat, lon)

            if hasattr(self, "home_marker") and self.home_marker:
                self.home_marker.delete()

            self.home_marker = self.map_widget.set_marker(lat, lon, text="Home", icon=self.location_icon)
            self.flight_path.clear()  # ✅ Reset đường bay nếu cập nhật lại vị trí ban đầu

            print(f"Đã cập nhật vị trí ban đầu: {lat}, {lon}")
        else:
            messagebox.showinfo("Thông báo", "Chưa nhận được vị trí drone từ Raspberry Pi.")

    def receive_position_loop(self):
        host_ip = "0.0.0.0"
        port = 9999
        
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((host_ip, port))
            while True:
                try:
                    data, _ = sock.recvfrom(1024)
                    lat, lon, alt, yaw, vel = map(float, data.decode().split(","))
                    self.update_drone_display(lat, lon, alt, yaw, vel)
                except Exception as e:
                    print(f"Receive error: {e}")

    def update_drone_display(self, lat, lon, alt, yaw, vel):
        print(f"🛰️ Tọa độ mới: lat={lat}, lon={lon}, yaw={yaw}, vel={vel}")

        def update():
            self.info_label.config(
                text=f"Altitude: {alt:.1f}m | Velocity: {vel:.1f}m/s"
            )

            self.drone_position = (lat, lon)

            # Xoay icon theo yaw
            rotated_icon = self.drone_icon_base.rotate(-yaw)
            new_icon = ImageTk.PhotoImage(rotated_icon)
            self.drone_icon = new_icon  # giữ tham chiếu

            if self.drone_marker:
                self.drone_marker.delete()
            self.drone_marker = self.map_widget.set_marker(lat, lon, text="", icon=self.drone_icon)

            # ✅ Cập nhật đường bay thực tế
            self.flight_path.append((lat, lon))
            self.draw_path_line_from_list(self.flight_path)

        self.after(0, update)

    def draw_path_line_from_list(self, points):
        if self.path_line:
            self.map_widget.delete(self.path_line)
        self.path_line = self.map_widget.set_path(points)

    def update_station_markers(self):
        for marker in self.station_markers:
            marker.delete()
        self.station_markers.clear()

        if not self.home_position:
            messagebox.showinfo("Thông báo", "Cần cập nhật vị trí ban đầu trước.")
            return

        home_lat, home_lon = self.home_position
        selected = self.controller.selected_station_names
        offsets = self.controller.station_offsets

        for name in selected:
            if name in offsets:
                dx, dy = offsets[name]
                lat = home_lat + dy / 111320
                lon = home_lon + dx / (40075000 * math.cos(math.radians(home_lat)) / 360)

                marker = self.map_widget.set_marker(lat, lon, text=name, icon=self.location_icon)
                self.station_markers.append(marker)
                self.station_position = (lat, lon)  # Cập nhật trạm cuối (để dùng cho vẽ đường nếu cần)

    def emergency_action(self):
        messagebox.showwarning("EMERGENCY", "RTL được kích hoạt!")


class StationPage(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.controller = controller

        tk.Label(self, text="Choosing the station", font=("Helvetica", 14)).pack(pady=10)

        self.stations = [
            {"name": "S1", "id": 1},
            {"name": "S2", "id": 2}
        ]

        self.selected_station_ids = []
        self.station_buttons = {}

        for station in self.stations:
            btn = tk.Button(self, text=station["name"], width=20,
                            command=lambda s=station: self.toggle_station(s))
            btn.pack(pady=2)
            self.station_buttons[station["id"]] = btn

        tk.Button(self, text="✅ Confirm the route", command=self.confirm_mission).pack(pady=5)
        tk.Button(self, text="❌ Clear selections", command=self.clear_selection).pack(pady=5)
        tk.Button(self, text="🚨 EMERGENCY", fg="white", bg="red", font=("Helvetica", 12, "bold"),
                  command=self.emergency_action).pack(pady=5)
        tk.Button(self, text="← Turn back", command=lambda: self.controller.show_frame(MainMenu)).pack(side="left", padx=10, pady=5)
        tk.Button(self, text="🚨", fg="white", bg="red", font=("Helvetica", 10, "bold"),
                  command=self.emergency_action).pack(side="left", padx=10, pady=5)
        tk.Button(self, text="🛫 Start", fg="white", bg="green", font=("Helvetica", 12, "bold"),
          command=self.send_start_signal).pack(pady=5)

        self.mission_output = tk.Text(self, height=10, width=60)
        self.mission_output.pack(pady=10)

    def send_start_signal(self):
        if not self.selected_station_ids:
            self.mission_output.insert(tk.END, "⚠️ There are no selections!\n")
            return
        try:
            pi_ip = self.controller.pi_ip
            pi_port = self.controller.pi_port  # ban đầu là 5001
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((pi_ip, pi_port))
                s.sendall("start".encode())
                response = s.recv(1024).decode()
                self.mission_output.insert(tk.END, f"🛫 Send START → Response: {response}\n")

            # ✅ Nếu thành công, tăng port cho lần sau
            self.controller.pi_port += 1
            print(f"Next port to use: {self.controller.pi_port}")

        except Exception as e:
            self.mission_output.insert(tk.END, f"❌ Error when sending START: {e}\n")

    def toggle_station(self, station):
        sid = station["id"]
        name = station["name"]
        btn = self.station_buttons[sid]

        if sid not in self.selected_station_ids:
            self.selected_station_ids.append(sid)
            btn.config(bg="lightblue")
            self.mission_output.insert(tk.END, f"✅ Select {name}\n")
        else:
            self.selected_station_ids.remove(sid)
            btn.config(bg="SystemButtonFace")
            self.mission_output.insert(tk.END, f"❌ Clear selection {name}\n")

    def clear_selection(self):
        self.selected_station_ids.clear()
        for btn in self.station_buttons.values():
            btn.config(bg="SystemButtonFace")
        self.mission_output.insert(tk.END, "✅ All selections cleared \n")

    def emergency_action(self):
        self.mission_output.insert(tk.END, "🚨 EMERGENCY! RTL!\n")

    def meters_to_latlon(self, dx, dy, home_lat, home_lon):
        dlat = dy / 111320
        dlon = dx / (40075000 * math.cos(math.radians(home_lat)) / 360)
        return home_lat + dlat, home_lon + dlon

    def confirm_mission(self):
        if not self.selected_station_ids:
            self.mission_output.insert(tk.END, "⚠️ There are no stations!\n")
            return

        if not self.controller.home_position:
            self.mission_output.insert(tk.END, "⚠️ Chưa có vị trí ban đầu!\n")
            return
        
        home_lat, home_lon = self.controller.home_position
        self.controller.selected_station_names.clear()
        self.controller.station_offsets = {}

        try:
            pi_ip = self.controller.pi_ip  # cần khai báo ở App
            pi_port = self.controller.pi_port
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((pi_ip, pi_port))
                message = ",".join(str(i) for i in self.selected_station_ids)
                s.sendall(message.encode())
                response = s.recv(1024).decode()
                station_xy_dict = json.loads(response)
        except Exception as e:
            self.mission_output.insert(tk.END, f"❌ Sending Error: {e}\n")
            return

        for sid in self.selected_station_ids:
            sid_str = str(sid)
            if sid_str in station_xy_dict:
                dy, dx = station_xy_dict[sid_str]
                lat, lon = self.meters_to_latlon(dx, dy, home_lat, home_lon)
                name = next(st["name"] for st in self.stations if st["id"] == sid)
                self.controller.selected_station_names.append(name)
                self.controller.station_offsets[name] = (dx, dy) 
                self.mission_output.insert(tk.END, f"→ Flying to {name} tại ({dx:.6f}, {dy:.6f})\n")
                self.mission_output.insert(tk.END, f"   ...Fly...\n")
                self.mission_output.insert(tk.END, f"← Turn back ({home_lat}, {home_lon})\n\n")
            else:
                self.mission_output.insert(tk.END, f"⚠️ Don't have coordinate of station {sid}\n")
        self.controller.frames[HomePage].update_station_markers()

if __name__ == "__main__":
    app = App()
    app.mainloop()
