import socket

def diagnose():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 1. Send "command" to wake up the telemetry stream
    print(f"Sturen van 'command' naar 192.168.10.1:8889...")
    sock.sendto(b'command', ('192.168.10.1', 8889))
    
    # 2. Listen for the response or state
    sock.settimeout(5.0)
    print("Luisteren op poort 8890 voor Tello state pakketten...")
    print("(Zorg dat de tello_driver NIET draait, anders is de poort bezet)")
    
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            print(f"Ontvangen van {addr}: {data.decode('utf-8')[:50]}...")
    except socket.timeout:
        print("FOUT: Geen data ontvangen binnen 5 seconden.")
        print("Check of de drone aan staat en of Norton poort 8890 blokkeert.")
    except Exception as e:
        print(f"Fout: {e}")

if __name__ == "__main__":
    diagnose()
