"""
Receiver de baixa latência / máximo FPS para o stream TCP do ESP32-S3-CAM.

Pipeline de 3 estágios, cada um em sua própria thread e SEM esperar pelo próximo:

  [Thread NET]     lê o socket continuamente e publica só o JPEG mais recente.
        |
  [Thread DECODE]  decodifica (cv2.imdecode) sempre o JPEG mais recente e publica
        |          só o frame decodificado mais recente.
  [Thread MAIN]    imshow/waitKey sempre do frame decodificado mais recente.

Cada estágio descarta o que for antigo, então o lag nunca acumula, e a rede,
a decodificação e a exibição trabalham em paralelo (imdecode/imshow liberam a GIL).

Uso:
    pip install opencv-python numpy
    python receiver.py 192.168.1.106 [--port 5000] [--reduce 2] [--no-hud]
Teclas: q / ESC para sair.
"""
import argparse
import socket
import struct
import threading
import time
import cv2
import numpy as np

MAX_FRAME = 500_000      # sanidade: tamanho máximo aceitável de um JPEG (bytes)
RCVBUF = 512 * 1024      # buffer de recepção do socket (absorve rajadas de Wi-Fi)
FPS_ALPHA = 0.2          # suavização (média móvel exponencial) do FPS exibido

# Decodificação reduzida: libjpeg decodifica direto em 1/2, 1/4 ou 1/8 da
# resolução (muito mais rápido que decodificar cheio e redimensionar).
REDUCE_FLAGS = {
    1: cv2.IMREAD_COLOR,
    2: cv2.IMREAD_REDUCED_COLOR_2,
    4: cv2.IMREAD_REDUCED_COLOR_4,
    8: cv2.IMREAD_REDUCED_COLOR_8,
}


class LatestSlot:
    """
    Slot de 1 item: put() sobrescreve, get() devolve o item mais novo.
    Quem consome acorda por Condition (sem busy-loop) e só recebe itens
    ainda não vistos (compara o número de sequência).
    """

    def __init__(self):
        self._cond = threading.Condition()
        self._item = None
        self.seq = 0  # total de itens publicados (também serve de contador de FPS)

    def put(self, item):
        with self._cond:
            self._item = item
            self.seq += 1
            self._cond.notify_all()

    def get(self, last_seq, timeout):
        """Devolve (seq, item). item é None se nada novo chegou dentro do timeout."""
        with self._cond:
            if self.seq == last_seq:
                self._cond.wait(timeout)
                if self.seq == last_seq:
                    return last_seq, None
            return self.seq, self._item


class FrameReceiver(threading.Thread):
    """Estágio 1: rede. Só lê o socket e publica o JPEG mais recente."""

    def __init__(self, host, port, out: LatestSlot):
        super().__init__(daemon=True)
        self.host, self.port = host, port
        self.out = out
        self.connected = False
        self.running = True

    @staticmethod
    def _recv_exact(sock, view):
        """Preenche 'view' por completo, sem alocar/copiar buffers intermediários."""
        n = len(view)
        got = 0
        while got < n:
            r = sock.recv_into(view[got:], n - got)
            if r == 0:
                raise ConnectionError("conexão fechada pelo ESP32")
            got += r

    def _open(self):
        sock = socket.create_connection((self.host, self.port), timeout=3)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCVBUF)
        except OSError:
            pass
        if hasattr(socket, "TCP_QUICKACK"):  # Linux: ACK imediato, ESP32 envia sem esperar
            try:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
            except OSError:
                pass
        sock.settimeout(3)  # sem dados por 3 s => considera queda
        return sock

    def run(self):
        backoff = 0.5
        header = bytearray(4)
        header_view = memoryview(header)
        while self.running:
            sock = None
            try:
                print(f"[net] conectando em {self.host}:{self.port} ...")
                sock = self._open()
                self.connected = True
                backoff = 0.5
                print("[net] conectado")
                while self.running:
                    self._recv_exact(sock, header_view)
                    (n,) = struct.unpack_from("<I", header)
                    if n == 0 or n > MAX_FRAME:
                        raise ValueError(f"tamanho de frame inválido: {n}")
                    # Um bytearray novo por frame: é entregue direto ao decoder
                    # (np.frombuffer aceita bytearray), então NÃO há cópia extra.
                    data = bytearray(n)
                    self._recv_exact(sock, memoryview(data))
                    self.out.put(data)  # sobrescreve: só o mais novo importa
            except (OSError, ConnectionError, ValueError, struct.error) as e:
                print(f"\n[net] desconectado: {e}")
            finally:
                self.connected = False
                if sock:
                    try:
                        sock.close()
                    except OSError:
                        pass
            if self.running:
                time.sleep(backoff)
                backoff = min(backoff * 2, 3.0)  # backoff exponencial até 3 s

    def stop(self):
        self.running = False


class FrameDecoder(threading.Thread):
    """Estágio 2: decodifica sempre o JPEG mais recente, em loop livre."""

    def __init__(self, src: LatestSlot, out: LatestSlot, reduce=1):
        super().__init__(daemon=True)
        self.src, self.out = src, out
        self.flags = REDUCE_FLAGS[reduce]
        self.running = True

    def run(self):
        seq = 0
        while self.running:
            seq, jpg = self.src.get(seq, 0.1)
            if jpg is None:
                continue
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), self.flags)
            if img is not None:  # JPEG corrompido => ignora e segue
                self.out.put(img)

    def stop(self):
        self.running = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("host", help="IP do ESP32-S3-CAM")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--reduce", type=int, choices=sorted(REDUCE_FLAGS), default=1,
                    help="decodifica em 1/N da resolução (2, 4, 8): bem mais rápido, "
                         "mas a imagem exibida fica menor. Padrão: 1 (cheia)")
    ap.add_argument("--no-hud", action="store_true",
                    help="não desenha o contador de FPS sobre a imagem")
    args = ap.parse_args()

    cv2.setUseOptimized(True)

    jpeg_slot = LatestSlot()     # NET    -> DECODE
    frame_slot = LatestSlot()    # DECODE -> MAIN

    rx = FrameReceiver(args.host, args.port, jpeg_slot)
    dec = FrameDecoder(jpeg_slot, frame_slot, args.reduce)
    rx.start()
    dec.start()

    win = "ESP32-S3-CAM"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    placeholder = np.zeros((240, 320, 3), np.uint8)
    cv2.putText(placeholder, "Aguardando camera...", (30, 125),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow(win, placeholder)
    showing_placeholder = True

    show_hud = not args.no_hud
    seq = 0
    net_fps = dec_fps = disp_fps = 0.0
    disp_count = 0
    rx_last = dec_last = 0
    t_last = time.perf_counter()

    try:
        while True:
            # Espera (sem busy-loop) por um frame novo; o timeout curto garante que
            # o waitKey abaixo continue processando eventos da janela.
            seq, img = frame_slot.get(seq, 0.03)
            if img is not None:
                if show_hud:
                    # O frame pertence só a esta thread agora: desenhar in-place é seguro.
                    cv2.putText(img, f"RX {net_fps:.1f} | dec {dec_fps:.1f} | disp {disp_fps:.1f} fps",
                                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (0, 255, 0), 1, cv2.LINE_AA)
                cv2.imshow(win, img)
                showing_placeholder = False
                disp_count += 1
            elif not rx.connected and not showing_placeholder:
                cv2.imshow(win, placeholder)
                showing_placeholder = True

            now = time.perf_counter()
            dt = now - t_last
            if dt >= 1.0:
                # FPS com média móvel exponencial (como no CameraStream do facetracking)
                rx_seq, dec_seq = jpeg_slot.seq, frame_slot.seq
                net_i, dec_i, disp_i = (rx_seq - rx_last) / dt, (dec_seq - dec_last) / dt, disp_count / dt
                if net_fps == 0.0:
                    net_fps, dec_fps, disp_fps = net_i, dec_i, disp_i
                else:
                    a = FPS_ALPHA
                    net_fps += a * (net_i - net_fps)
                    dec_fps += a * (dec_i - dec_fps)
                    disp_fps += a * (disp_i - disp_fps)
                rx_last, dec_last, disp_count, t_last = rx_seq, dec_seq, 0, now
                print(f"\rRX: {net_fps:5.1f} | decode: {dec_fps:5.1f} | exibido: {disp_fps:5.1f} fps   ",
                      end="", flush=True)

            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        rx.stop()
        dec.stop()
        dec.join(timeout=1)   # evita encerrar o interpretador no meio de um imdecode
        rx.join(timeout=1)
        cv2.destroyAllWindows()
        print()


if __name__ == "__main__":
    main()