#!/usr/bin/env python3
import argparse
import socket
import threading
import time


def _log(prefix, data):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    text = data.decode("utf-8", errors="replace")
    printable = text.replace("\r", "\\r").replace("\n", "\\n")
    hex_preview = data[:64].hex()
    if len(data) > 64:
        hex_preview += "..."
    print(f'[{ts}] {prefix} bytes={len(data)} text="{printable}" hex={hex_preview}')


def _pipe(src, dst, label):
    try:
        while True:
            data = src.recv(4096)
            if not data:
                break
            _log(label, data)
            dst.sendall(data)
    except OSError:
        pass
    finally:
        try:
            dst.shutdown(socket.SHUT_WR)
        except OSError:
            pass


def _handle_client(client, target_host, target_port):
    with client:
        try:
            remote = socket.create_connection((target_host, target_port))
        except OSError:
            return
        with remote:
            t1 = threading.Thread(target=_pipe, args=(client, remote, "C->S"))
            t2 = threading.Thread(target=_pipe, args=(remote, client, "S->C"))
            t1.start()
            t2.start()
            t1.join()
            t2.join()


def main():
    parser = argparse.ArgumentParser(description="TCP proxy with logging")
    parser.add_argument("--listen", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=5761)
    parser.add_argument("--target", default="10.0.0.1")
    parser.add_argument("--target-port", type=int, default=5761)
    args = parser.parse_args()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.listen, args.listen_port))
        server.listen(128)
        print(
            f"Listening on {args.listen}:{args.listen_port} -> {args.target}:{args.target_port}"
        )
        while True:
            client, addr = server.accept()
            print(f"Accepted {addr[0]}:{addr[1]}")
            t = threading.Thread(
                target=_handle_client,
                args=(client, args.target, args.target_port),
                daemon=True,
            )
            t.start()


if __name__ == "__main__":
    main()
