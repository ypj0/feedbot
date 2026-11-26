#!/usr/bin/env python3
"""RK loopback-only SSH relay for legacy ADB implementations lacking raw exec."""
import select,socket,threading

def relay(client):
    upstream=None
    try:
        upstream=socket.create_connection(('10.7.5.88',22),timeout=8)
        upstream.settimeout(None)
        while True:
            ready,_,_=select.select([client,upstream],[],[])
            for source in ready:
                data=source.recv(65536)
                if not data:return
                (upstream if source is client else client).sendall(data)
    except OSError:pass
    finally:
        client.close()
        if upstream:upstream.close()

server=socket.socket();server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
server.bind(('127.0.0.1',22088));server.listen(8)
while True:
    client,_=server.accept()
    threading.Thread(target=relay,args=(client,),daemon=True).start()
