import subprocess
import time
import os
import webbrowser
import signal
import sys

# --- Configuration ---
SERVER_SCRIPT = "c2_server.py"
INTERFACE_FILE = "c2_interface.html"

def get_abs_path(relative_path):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), relative_path)

def baslat():
    print("🚀 BlueBoat C2 Yer İstasyonu Başlatılıyor...")
    
    # 1. C2 Server'ı Başlat
    server_path = get_abs_path(SERVER_SCRIPT)
    print(f"📡 WebSocket Sunucusu Başlatılıyor: {SERVER_SCRIPT}")
    
    try:
        # Sunucuyu ayrı bir süreçte başlat
        server_process = subprocess.Popen([sys.executable, server_path])
        
        # Sunucunun ayağa kalkması için kısa bir bekleme
        time.sleep(2)
        
        # 2. Arayüzü Tarayıcıda Aç
        interface_path = get_abs_path(INTERFACE_FILE)
        if os.path.exists(interface_path):
            print(f"💻 Arayüz Açılıyor: {INTERFACE_FILE}")
            webbrowser.open(f"file://{interface_path}")
        else:
            print(f"⚠️ Hata: {INTERFACE_FILE} bulunamadı!")

        print("\n------------------------------------------------")
        print("✅ Sistem Çalışıyor. Durdurmak için CTRL+C yapın.")
        print("------------------------------------------------\n")

        # Süreçleri izle
        while True:
            if server_process.poll() is not None:
                print("❌ Sunucu süreci durdu!")
                break
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n🛑 Sistem kapatılıyor...")
        server_process.terminate()
        server_process.wait()
        print("👋 Görüşmek üzere!")
    except Exception as e:
        print(f"❌ Beklenmedik bir hata oluştu: {e}")

if __name__ == "__main__":
    baslat()
