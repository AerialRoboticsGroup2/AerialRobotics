import socket
import time

def test_tello_connection():
    # Tello IP and Port
    TELLO_IP = '192.168.10.1'
    TELLO_PORT = 8889
    
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', 9000)) # Listen on local port 9000
    sock.settimeout(3.0)
    
    try:
        print(f"Versturen van 'command' naar {TELLO_IP}:{TELLO_PORT}...")
        sock.sendto(b'command', (TELLO_IP, TELLO_PORT))
        
        print("Wachten op antwoord ('ok')...")
        data, server = sock.recvfrom(1024)
        print(f"Ontvangen van {server}: {data.decode('utf-8')}")
        
        if data.decode('utf-8').strip() == 'ok':
            print("SUCCES! De drone is bereikbaar en de SDK is actief.")
        else:
            print(f"Onverwacht antwoord: {data.decode('utf-8')}")
            
    except socket.timeout:
        print("FOUT: Timeout. De drone reageert niet. Check Wi-Fi en Windows Firewall.")
    except Exception as e:
        print(f"Fout: {e}")
    finally:
        sock.close()

if __name__ == '__main__':
    test_tello_connection()
