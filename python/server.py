"""
Servidor de recepção da caixa de areia (ESP32-C3: INMP441 + HX711 + AHT10).

Protocolo (uma única conexão WebSocket em /ws/caixa_areia):
  texto  "start_audio"        -> abre um novo arquivo WAV
  binário (PCM 16 bit, mono, 16 kHz, little-endian) -> anexado ao WAV aberto
  texto  "stop_audio"         -> fecha o WAV, grava metadados e responde "salvo <arquivo>"
  texto  '{"tipo":"telemetria", ...}' -> pacote JSON periódico (peso, temperatura,
                                         umidade, presença do gato). Por enquanto
                                         só é mantido em memória (último valor).

Princípio: o WAV salvo é o áudio BRUTO, exatamente como chegou. Nenhum corte de
silêncio, normalização ou filtro é aplicado (sons de gato podem ser baixos,
curtos e raros).

Execução:
  pip install fastapi "uvicorn[standard]" numpy tzdata
  uvicorn server:app --host 0.0.0.0 --port 8080

Variáveis de ambiente opcionais:
  AUDIO_DIR        pasta de saída dos áudios (padrão: ./audios)
  FUSO_HORARIO     nome IANA do fuso (padrão: America/Fortaleza)
  LOG_TELEMETRIA   "1" para imprimir cada pacote de telemetria no log
"""
import asyncio
import json
import logging
import math
import os
import wave
from datetime import datetime, timedelta, timezone

import numpy as np
from fastapi import APIRouter, FastAPI, WebSocket, WebSocketDisconnect

SAMPLE_RATE = 16000
SAMPLE_WIDTH = 2  # 16-bit PCM
CANAIS = 1
BUFFER_ARQUIVO = 64 * 1024  # escrita em disco em blocos grandes (chega ~1 KB a cada 32 ms)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUDIO_DIR = os.environ.get("AUDIO_DIR", os.path.join(BASE_DIR, "audios"))
LOG_TELEMETRIA = os.environ.get("LOG_TELEMETRIA", "1") == "1"
os.makedirs(AUDIO_DIR, exist_ok=True)


# ---------------------------------------------------------------------
#  Fuso horário e log
# ---------------------------------------------------------------------
def _obter_fuso():
    nome = os.environ.get("FUSO_HORARIO", "America/Fortaleza")
    try:
        from zoneinfo import ZoneInfo
        return ZoneInfo(nome)
    except Exception:
        # Sem o pacote tzdata (comum no Windows): usa UTC-3 fixo
        # (Fortaleza não tem horário de verão, então o deslocamento é sempre -03:00).
        return timezone(timedelta(hours=-3), "UTC-03")


FUSO = _obter_fuso()


def agora() -> datetime:
    """Data/hora atual no fuso local, já com o deslocamento (timezone-aware)."""
    return datetime.now(FUSO)


class _FormatadorLocal(logging.Formatter):
    def formatTime(self, record, datefmt=None):
        return datetime.fromtimestamp(record.created, FUSO).strftime("%Y-%m-%d %H:%M:%S")


_handler = logging.StreamHandler()
_handler.setFormatter(_FormatadorLocal("%(asctime)s [%(levelname)s] %(message)s"))
log = logging.getLogger("caixa_areia")
log.addHandler(_handler)
log.setLevel(logging.INFO)
log.propagate = False

router = APIRouter()


# ---------------------------------------------------------------------
#  Gravação de áudio
# ---------------------------------------------------------------------
def _dbfs(valor: float):
    return round(20 * math.log10(valor / 32768), 1) if valor > 0 else None


class Gravacao:
    """Uma gravação em andamento (do 'start_audio' até o 'stop_audio')."""

    __slots__ = ("inicio", "nome", "caminho", "_arquivo", "_wav", "total_bytes", "resto",
                 "pico", "soma_quadrados", "amostras", "saturadas", "chunks", "fechada")

    def __init__(self):
        self.inicio = agora()
        self.nome = f"gato_{self.inicio:%Y%m%d_%H%M%S_%f}.wav"
        self.caminho = os.path.join(AUDIO_DIR, self.nome)

        self._arquivo = open(self.caminho, "wb", buffering=BUFFER_ARQUIVO)
        self._wav = wave.open(self._arquivo, "wb")
        self._wav.setnchannels(CANAIS)
        self._wav.setsampwidth(SAMPLE_WIDTH)
        self._wav.setframerate(SAMPLE_RATE)

        self.total_bytes = 0
        self.resto = b""            # byte sobrando caso um pacote chegue com tamanho ímpar
        self.pico = 0
        self.soma_quadrados = 0     # inteiro Python: exato, sem estouro
        self.amostras = 0
        self.saturadas = 0
        self.chunks = 0
        self.fechada = False

    def adicionar(self, dados: bytes):
        if self.resto:
            dados = self.resto + dados
            self.resto = b""
        if len(dados) & 1:
            self.resto = dados[-1:]
            dados = dados[:-1]
        if not dados:
            return

        # Gravação incremental: nada fica acumulado na RAM, mesmo em sessões longas
        self._wav.writeframesraw(dados)
        self.total_bytes += len(dados)
        self.chunks += 1

        # Estatísticas (apenas observação; não alteram o áudio gravado)
        x = np.frombuffer(dados, dtype="<i2")
        minimo, maximo = int(x.min()), int(x.max())
        self.pico = max(self.pico, maximo, -minimo)
        x64 = x.astype(np.int64)
        self.soma_quadrados += int(np.dot(x64, x64))
        self.saturadas += int(np.count_nonzero((x == 32767) | (x == -32768)))
        self.amostras += x.size

    def fechar(self, motivo: str) -> dict:
        """Fecha o WAV e grava o .json de metadados. Bloqueante: chame via to_thread."""
        if self.fechada:
            return {}
        self.fechada = True
        self._wav.close()       # corrige o cabeçalho com o tamanho real
        self._arquivo.close()   # o wave não fecha arquivos que ele não abriu

        fim = agora()
        duracao = self.total_bytes / (SAMPLE_RATE * SAMPLE_WIDTH)
        rms = math.sqrt(self.soma_quadrados / self.amostras) if self.amostras else 0.0

        meta = {
            "arquivo": f"audios/{self.nome}",
            "inicio": self.inicio.isoformat(timespec="milliseconds"),
            "fim": fim.isoformat(timespec="milliseconds"),
            "fuso": str(FUSO),
            "duracao_s": round(duracao, 2),
            "sample_rate": SAMPLE_RATE,
            "bytes": self.total_bytes,
            "chunks": self.chunks,
            "pico": self.pico,
            "pico_dbfs": _dbfs(self.pico),
            "rms_dbfs": _dbfs(rms),
            "amostras_saturadas": self.saturadas,
            "motivo_fechamento": motivo,
            # Só sinalizam; o arquivo é mantido de qualquer forma
            "suspeita_mic_mudo": self.amostras > 0 and self.pico < 20,
            "suspeita_clipping": self.amostras > 0 and (self.saturadas / self.amostras) > 0.001,
        }
        with open(self.caminho[:-4] + ".json", "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        return meta


# ---------------------------------------------------------------------
#  Telemetria (por enquanto só em memória)
# ---------------------------------------------------------------------
ULTIMA_TELEMETRIA: dict | None = None


def _tratar_telemetria(texto: str):
    global ULTIMA_TELEMETRIA
    try:
        dados = json.loads(texto)
    except json.JSONDecodeError:
        log.warning("Telemetria com JSON inválido descartada: %.80s", texto)
        return
    if not isinstance(dados, dict) or dados.get("tipo") != "telemetria":
        return
    dados["recebido_em"] = agora().isoformat(timespec="milliseconds")
    ULTIMA_TELEMETRIA = dados
    if LOG_TELEMETRIA:
        log.info("Telemetria: %s", dados)


# ---------------------------------------------------------------------
#  WebSocket
# ---------------------------------------------------------------------
@router.websocket("/ws/caixa_areia")
async def websocket_caixa_areia(websocket: WebSocket):
    await websocket.accept()
    log.info("Cliente conectado: %s", websocket.client)

    gravacao: Gravacao | None = None
    avisou_sem_start = False  # evita repetir o aviso ~31 vezes por segundo

    try:
        while True:
            mensagem = await websocket.receive()
            if mensagem["type"] == "websocket.disconnect":
                break

            # ---------- Áudio PCM (o caso mais frequente: vem primeiro) ----------
            dados = mensagem.get("bytes")
            if dados is not None:
                if gravacao is not None:
                    gravacao.adicionar(dados)
                elif not avisou_sem_start:
                    avisou_sem_start = True
                    log.warning("Áudio recebido sem 'start_audio'. Descartando.")
                continue

            texto = mensagem.get("text")
            if not texto:
                continue
            comando = texto.strip()

            # ---------- Telemetria ----------
            if comando.startswith("{"):
                _tratar_telemetria(comando)

            # ---------- Controle de gravação ----------
            elif comando.startswith("start_audio"):
                if gravacao is not None:
                    # Reconexão no meio de uma visita: fecha a parte anterior e abre outra
                    meta = await asyncio.to_thread(gravacao.fechar, "reiniciada_por_novo_start")
                    log.warning("start_audio com gravação aberta. Parte anterior salva: %s", meta.get("arquivo"))
                gravacao = Gravacao()
                avisou_sem_start = False
                log.info("Gato entrou. Gravando em %s", gravacao.nome)

            elif comando.startswith("stop_audio"):
                if gravacao is None:
                    log.warning("stop_audio sem gravação aberta. Ignorado.")
                    continue
                meta = await asyncio.to_thread(gravacao.fechar, "stop_audio")  # I/O fora do event loop
                gravacao = None
                log.info("Gato saiu. %s s | pico %s dBFS | rms %s dBFS -> %s",
                         meta["duracao_s"], meta["pico_dbfs"], meta["rms_dbfs"], meta["arquivo"])
                if meta["suspeita_mic_mudo"]:
                    log.warning("Áudio praticamente mudo: verifique a fiação do INMP441.")
                if meta["suspeita_clipping"]:
                    log.warning("Muitas amostras saturadas: considere aumentar AUDIO_SHIFT_BITS no firmware.")
                await websocket.send_text(f"salvo {meta['arquivo']}")

                # >>> Ponto de extensão: aqui entrará o disparo da análise de ML <<<

    except WebSocketDisconnect:
        pass
    except Exception:
        log.exception("Erro inesperado na conexão %s", websocket.client)
    finally:
        # Queda de conexão no meio da gravação: preserva o que já chegou
        if gravacao is not None:
            meta = gravacao.fechar("conexao_encerrada")
            log.warning("Conexão caiu durante a gravação. Parcial salvo: %s (%s s)",
                        meta["arquivo"], meta["duracao_s"])
        log.info("Conexão encerrada: %s", websocket.client)


app = FastAPI(title="Caixa de areia - recepção de dados")
app.include_router(router)


@app.get("/telemetria")
def ultima_telemetria():
    """Último pacote de telemetria recebido (apenas em memória; útil para conferir o firmware)."""
    return ULTIMA_TELEMETRIA or {"status": "nenhum pacote recebido ainda"}