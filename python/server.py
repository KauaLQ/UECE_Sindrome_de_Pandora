"""
python -m uvicorn server:app --host 0.0.0.0 --port 8080

Servidor de recepção de áudio da caixa de areia (ESP32-C3 + INMP441).

Protocolo com o firmware:
  texto  "start_audio"  -> abre um novo arquivo WAV
  binário (PCM 16 bit, mono, 16 kHz, little-endian) -> anexado ao arquivo
  texto  "stop_audio"   -> fecha o arquivo, grava metadados e responde "salvo <arquivo>"

Princípio: o WAV salvo é o áudio BRUTO, exatamente como chegou. Nenhum corte de
silêncio, normalização ou filtro é aplicado (sons de gato podem ser baixos,
curtos e raros). Qualquer pré-processamento para ML deve ser feito depois, em
uma cópia, a partir deste arquivo original.
"""
import json
import os
import wave
from datetime import datetime, timezone

import numpy as np
from fastapi import APIRouter, FastAPI, WebSocket, WebSocketDisconnect

SAMPLE_RATE = 16000
SAMPLE_WIDTH = 2  # 16-bit PCM
CANAIS = 1

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AUDIO_DIR = os.environ.get("AUDIO_DIR", os.path.join(BASE_DIR, "audios"))
os.makedirs(AUDIO_DIR, exist_ok=True)

router = APIRouter()


class Gravacao:
    """Uma gravação em andamento (do 'start_audio' até o 'stop_audio')."""

    def __init__(self, motivo_abertura: str = "start_audio"):
        self.inicio = datetime.now(timezone.utc)
        self.nome = f"gato_{self.inicio.strftime('%Y%m%d_%H%M%S_%f')}.wav"
        self.caminho = os.path.join(AUDIO_DIR, self.nome)
        self.motivo_abertura = motivo_abertura

        self.wav = wave.open(self.caminho, "wb")
        self.wav.setnchannels(CANAIS)
        self.wav.setsampwidth(SAMPLE_WIDTH)
        self.wav.setframerate(SAMPLE_RATE)

        self.total_bytes = 0
        self.resto = b""           # byte sobrando caso um pacote chegue com tamanho ímpar
        self.pico = 0
        self.soma_quadrados = 0.0
        self.amostras = 0
        self.saturadas = 0
        self.chunks = 0

    def adicionar(self, dados: bytes):
        dados = self.resto + dados
        if len(dados) % SAMPLE_WIDTH:
            self.resto = dados[-1:]
            dados = dados[:-1]
        else:
            self.resto = b""
        if not dados:
            return

        # Gravação incremental: nada fica acumulado na RAM, mesmo em sessões longas
        self.wav.writeframes(dados)
        self.total_bytes += len(dados)
        self.chunks += 1

        # Estatísticas (apenas observação; não alteram o áudio gravado)
        x = np.frombuffer(dados, dtype="<i2").astype(np.int32)
        self.pico = max(self.pico, int(np.max(np.abs(x))))
        self.soma_quadrados += float(np.sum(x.astype(np.float64) ** 2))
        self.amostras += len(x)
        self.saturadas += int(np.sum(np.abs(x) >= 32767))

    def fechar(self, motivo: str) -> dict:
        self.wav.close()
        fim = datetime.now(timezone.utc)
        duracao = self.total_bytes / (SAMPLE_RATE * SAMPLE_WIDTH)

        rms = (self.soma_quadrados / self.amostras) ** 0.5 if self.amostras else 0.0
        rms_dbfs = 20 * np.log10(rms / 32768) if rms > 0 else None
        pico_dbfs = 20 * np.log10(self.pico / 32768) if self.pico > 0 else None

        meta = {
            "arquivo": f"audios/{self.nome}",
            "inicio_utc": self.inicio.isoformat(),
            "fim_utc": fim.isoformat(),
            "duracao_s": round(duracao, 2),
            "sample_rate": SAMPLE_RATE,
            "bytes": self.total_bytes,
            "chunks": self.chunks,
            "pico": self.pico,
            "pico_dbfs": None if pico_dbfs is None else round(float(pico_dbfs), 1),
            "rms_dbfs": None if rms_dbfs is None else round(float(rms_dbfs), 1),
            "amostras_saturadas": self.saturadas,
            "motivo_fechamento": motivo,
            # Só sinaliza; o arquivo é mantido de qualquer forma
            "suspeita_mic_mudo": self.pico < 20 and self.amostras > 0,
            "suspeita_clipping": self.amostras > 0 and (self.saturadas / self.amostras) > 0.001,
        }
        with open(self.caminho.replace(".wav", ".json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        return meta


@router.websocket("/ws/caixa_areia")
async def websocket_caixa_areia(websocket: WebSocket):
    await websocket.accept()
    print(f"[NET] Cliente conectado: {websocket.client}")

    gravacao: Gravacao | None = None

    try:
        while True:
            mensagem = await websocket.receive()
            if mensagem.get("type") == "websocket.disconnect":
                break

            # ---------- Comandos de controle ----------
            texto = mensagem.get("text")
            if texto is not None:
                comando = texto.strip()

                if comando.startswith("start_audio"):
                    if gravacao is not None:
                        # Reconexão no meio de uma visita: fecha a parte anterior e abre outra
                        meta = gravacao.fechar("reiniciada_por_novo_start")
                        print(f"[WARN] start_audio com gravação aberta. Parte anterior salva: {meta['arquivo']}")
                    gravacao = Gravacao()
                    print(f"[INFO] Gato entrou. Gravando em {gravacao.nome}")

                elif comando.startswith("stop_audio"):
                    if gravacao is None:
                        print("[WARN] stop_audio sem gravação aberta. Ignorado.")
                        continue
                    meta = gravacao.fechar("stop_audio")
                    gravacao = None
                    print(f"[INFO] Gato saiu. {meta['duracao_s']} s | pico {meta['pico_dbfs']} dBFS | "
                          f"rms {meta['rms_dbfs']} dBFS -> {meta['arquivo']}")
                    if meta["suspeita_mic_mudo"]:
                        print("[ALERTA] Áudio praticamente mudo: verifique a fiação do INMP441.")
                    if meta["suspeita_clipping"]:
                        print("[ALERTA] Muitas amostras saturadas: considere aumentar AUDIO_SHIFT_BITS no firmware.")
                    await websocket.send_text(f"salvo {meta['arquivo']}")

                    # >>> Ponto de extensão: aqui entrará o disparo da análise de ML <<<
                    # ex.: fila.put(meta["arquivo"])

            # ---------- Áudio PCM ----------
            dados = mensagem.get("bytes")
            if dados is not None:
                if gravacao is None:
                    print("[WARN] Áudio recebido sem 'start_audio'. Descartado.")
                else:
                    gravacao.adicionar(dados)

    except WebSocketDisconnect:
        pass
    finally:
        # Queda de conexão no meio da gravação: preserva o que já chegou
        if gravacao is not None:
            meta = gravacao.fechar("conexao_encerrada")
            print(f"[WARN] Conexão caiu durante a gravação. Parcial salvo: {meta['arquivo']} ({meta['duracao_s']} s)")
        print(f"[NET] Conexão encerrada: {websocket.client}")


app = FastAPI(title="Caixa de areia - recepção de áudio")
app.include_router(router)