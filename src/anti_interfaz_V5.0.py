import csv
import math
import sys
import re
import time
import struct          # <-- NUEVO: Para decodificar binario
import threading       # <-- NUEVO: Para leer sin congelar la GUI
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt6 import QtCore, QtGui, QtWidgets
try:
    import serial  # noqa: F401
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

import ctypes
try:
    from ctypes import wintypes
except ImportError:
    wintypes = None


# =====================================================================
#  CONFIGURACIÓN
# =====================================================================
FS_HZ = 200                   # telemetría del firmware: 1 / PEND_TS_MS (5 ms) = 200 Hz
MANUAL_KEEPALIVE_MS = 150     # refresco de la orden manual, deadman
PLOT_HZ = 20                  # refresco visual: 20 Hz es fluido y deja libre el event loop
PLOT_DT_MS = round(1000 / PLOT_HZ)
STATUS_HZ = 2                 # la barra de estado no necesita actualizarse 50 veces/s
HISTORIAL_S = 30              # segundos de histórico en pantalla
timeout_handshake = 10000
ping_interval_ms = 500     # el firmware pasa a STANDBY tras 1.5 s sin tráfico con el motor activo
timeout_ack = 2000

#Matrices K_d de observadores (4x2)
#NL
OBS_KEYS = ("k11", "k12", "k21", "k22", "k31", "k32", "k41", "k42")
OBS_DEFAULT = dict(zip(OBS_KEYS, (0.0437, 0.0000, 0.0978, 0.0001, 0.0000, 0.0338, 0.0000, 0.0181)))
#KAL
KAL_KEYS = ("ka11", "ka12", "ka21", "ka22", "ka31", "ka32", "ka41", "ka42")
KAL_DEFAULT = dict(zip(KAL_KEYS, (0.0437, 0.0000, 0.0978, 0.0001, 0.0000, 0.0338, 0.0000, 0.0181)))


# Estilos Claro
ESTILO_GLOBAL_CLARO = """
    QMainWindow, QDialog, QMessageBox { background-color: #f5f5f5; color: #212121; }
    QWidget { font-size: 10pt; color: #212121; }
    QScrollArea, QScrollArea > QWidget > QWidget { background-color: transparent; border: none; }
    QGroupBox {
        font-weight: bold; border: 1px solid #dcdcdc; border-radius: 6px;
        margin-top: 8px; padding-top: 12px; background-color: #ffffff;
    }
    QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; color: #333333; }
    QLabel { background-color: transparent; }
    QCheckBox { color: #212121; background-color: transparent; }
    QCheckBox::indicator { width: 14px; height: 14px; border-radius: 2px; border: 1px solid #999999; background: #ffffff; }
    QCheckBox::indicator:checked { background: #2196F3; border: 1px solid #1976D2; }
    QTabWidget::pane { border: 1px solid #dcdcdc; background-color: #ffffff; }
    QTabBar::tab { background: #e0e0e0; color: #333333; padding: 6px 14px; border-top-left-radius: 4px; border-top-right-radius: 4px; }
    QTabBar::tab:selected { background: #ffffff; font-weight: bold; }
    QLineEdit, QDoubleSpinBox, QComboBox {
        background-color: #ffffff; color: #111111; border: 1px solid #cccccc; border-radius: 4px; padding: 3px 6px;
    }
    QLineEdit:focus, QDoubleSpinBox:focus, QComboBox:focus { border: 1px solid #2196F3; }
    QComboBox::drop-down { border: none; }
    QComboBox QAbstractItemView { background-color: #ffffff; color: #111111; selection-background-color: #2196F3; selection-color: white; }
    QPushButton { border-radius: 4px; padding: 4px 8px; background-color: #e0e0e0; color: #111111; border: 1px solid #cccccc; }
    QPushButton:hover { background-color: #d5d5d5; }
    QPushButton:disabled { background-color: #eeeeee; color: #999999; border: 1px solid #dcdcdc; }
    QPushButton#btnModo { background-color: #eceff1; color: #37474f; font-weight: normal; border: none; }
    QPushButton#btnModo:checked { background-color: #2196F3; color: white; font-weight: bold; }
    QStatusBar { background: #eeeeee; color: #212121; border-top: 1px solid #dcdcdc; }
    QScrollBar:vertical { background: #f0f0f0; width: 10px; margin: 0; }
    QScrollBar::handle:vertical { background: #cdcdcd; min-height: 20px; border-radius: 5px; }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
    QToolTip { background-color: #ffffff; color: #111111; border: 1px solid #cccccc; }
"""

# Estilos Oscuro / Modo Oscuro (Azul)
ESTILO_GLOBAL_BATMAN = """
    QMainWindow, QDialog, QMessageBox { background-color: #121212; color: #e0e0e0; }
    QWidget { font-size: 10pt; color: #e0e0e0; }
    QScrollArea, QScrollArea > QWidget > QWidget { background-color: transparent; border: none; }
    QGroupBox {
        font-weight: bold; border: 1px solid #2d2d2d; border-radius: 6px;
        margin-top: 8px; padding-top: 12px; background-color: #1a1a1a;
    }
    QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; color: #00A8FF; }
    QLabel { background-color: transparent; }
    QCheckBox { color: #e0e0e0; background-color: transparent; }
    QCheckBox::indicator { width: 14px; height: 14px; border-radius: 2px; border: 1px solid #555555; background: #222222; }
    QCheckBox::indicator:checked { background: #00A8FF; border: 1px solid #00A8FF; }
    QTabWidget::pane { border: 1px solid #2d2d2d; background-color: #1a1a1a; }
    QTabBar::tab { background: #252525; color: #aaaaaa; padding: 6px 14px; border-top-left-radius: 4px; border-top-right-radius: 4px; }
    QTabBar::tab:selected { background: #1a1a1a; color: #00A8FF; font-weight: bold; }
    QLineEdit, QDoubleSpinBox, QComboBox {
        background-color: #252525; color: #ffffff; border: 1px solid #3d3d3d; border-radius: 4px; padding: 3px 6px;
    }
    QLineEdit:focus, QDoubleSpinBox:focus, QComboBox:focus { border: 1px solid #00A8FF; }
    QComboBox::drop-down { border: none; }
    QComboBox QAbstractItemView { background-color: #252525; color: #ffffff; selection-background-color: #00A8FF; selection-color: white; }
    QPushButton { border-radius: 4px; padding: 4px 8px; background-color: #2a2a2a; color: #e0e0e0; border: 1px solid #383838; }
    QPushButton:hover { background-color: #383838; }
    QPushButton:disabled { background-color: #1a1a1a; color: #555555; border: 1px solid #2d2d2d; }
    QPushButton#btnModo { background-color: #222222; color: #bbbbbb; font-weight: normal; border: 1px solid #333333; }
    QPushButton#btnModo:checked { background-color: #00A8FF; color: #ffffff; font-weight: bold; border: 1px solid #0099FF; }
    QStatusBar { background: #0d0d0d; color: #cccccc; border-top: 1px solid #222222; }
    QScrollBar:vertical { background: #181818; width: 10px; margin: 0; }
    QScrollBar::handle:vertical { background: #333333; min-height: 20px; border-radius: 5px; }
    QScrollBar::handle:vertical:hover { background: #444444; }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
    QToolTip { background-color: #1a1a1a; color: #e0e0e0; border: 1px solid #3d3d3d; }
"""

ESTILO_ENVIAR_CLARO = """
    QPushButton          { background-color: #2196F3; color: white; border: none; font-weight: bold; }
    QPushButton:hover    { background-color: #1976D2; }
    QPushButton:disabled { background-color: #BDBDBD; color: #F5F5F5; }
"""
ESTILO_ENVIAR_BATMAN = """
    QPushButton          { background-color: #00A8FF; color: white; border: none; font-weight: bold; }
    QPushButton:hover    { background-color: #0099FF; }
    QPushButton:disabled { background-color: #1a1a1a; color: #555555; border: 1px solid #2d2d2d; }
"""

ESTILO_GRABAR = """
    QPushButton { background-color: #4CAF50; color: white; font-weight: bold; border: none; }
    QPushButton:hover { background-color: #43A047; }
"""
ESTILO_KILL = """
    QPushButton         { background-color: #f44336; color: white; font-weight: bold;
                          padding: 15px; font-size: 16px; border: none; }
    QPushButton:pressed { background-color: #c62828; }
"""

# Alias para compatibilidad
ESTILO_GLOBAL = ESTILO_GLOBAL_CLARO
ESTILO_ENVIAR = ESTILO_ENVIAR_CLARO

# =====================================================================
#  WIDGET: TOGGLE SWITCH (Estilo iOS / Material)
# =====================================================================
class ToggleSwitch(QtWidgets.QAbstractButton):
    def __init__(self, parent=None, track_radius=11, thumb_radius=8):
        super().__init__(parent)
        self.setCheckable(True)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Fixed)
        self._track_radius = track_radius
        self._thumb_radius = thumb_radius
        self._offset = float(thumb_radius)

        # Animación suave del thumb
        self._anim = QtCore.QPropertyAnimation(self, b"offset", self)
        self._anim.setDuration(160)
        self._anim.setEasingCurve(QtCore.QEasingCurve.Type.InOutQuad)
        self.toggled.connect(self._start_transition)

    @QtCore.pyqtProperty(float)
    def offset(self):
        return self._offset

    @offset.setter
    def offset(self, val):
        self._offset = val
        self.update()

    def sizeHint(self):
        return QtCore.QSize(2 * self._track_radius + 24, 2 * self._track_radius + 4)

    def _start_transition(self, checked):
        end_val = self.width() - self._thumb_radius - 4 if checked else self._thumb_radius + 4
        self._anim.stop()
        self._anim.setStartValue(self._offset)
        self._anim.setEndValue(float(end_val))
        self._anim.start()

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._offset = self.width() - self._thumb_radius - 4 if self.isChecked() else self._thumb_radius + 4

    def paintEvent(self, e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        # Colores según estado
        if self.isChecked():
            track_color = QtGui.QColor("#00A8FF")      # Azul Material
            thumb_color = QtGui.QColor("#FFFFFF")      # Blanco
        else:
            track_color = QtGui.QColor("#cfd8dc")      # Claro
            thumb_color = QtGui.QColor("#ffffff")

        track_rect = QtCore.QRectF(
            2, (self.height() - 2 * self._track_radius) / 2.0,
            self.width() - 4, 2 * self._track_radius
        )
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QBrush(track_color))
        p.drawRoundedRect(track_rect, self._track_radius, self._track_radius)

        # Thumb
        thumb_center_y = self.height() / 2.0
        p.setBrush(QtGui.QBrush(thumb_color))
        p.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0, 40), 1))
        p.drawEllipse(QtCore.QPointF(self._offset, thumb_center_y), self._thumb_radius, self._thumb_radius)
        p.end()

# =====================================================================
#  Funcional, para que sea mas amigable
# =====================================================================
class FlexibleDoubleSpinBox(QtWidgets.QDoubleSpinBox):
    """Acept ',' o '.' como separador decimal."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setLocale(QtCore.QLocale.c())  # Forzar punto como separador decimal
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)

    def wheelEvent(self, event):
        """Evita que la rueda zoomee."""
        event.ignore()

    def keyPressEvent(self, event):
        if event.text() == ',':
            event = QtGui.QKeyEvent(event.type(),
                    QtCore.Qt.Key.Key_Period, event.modifiers(), '.')
        super().keyPressEvent(event)

class NoWheelComboBox(QtWidgets.QComboBox):
    """Evita que la rueda cambie la selección."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.StrongFocus)

    def wheelEvent(self, event):
        event.ignore()

class ScrollAwarePlotWidget(pg.PlotWidget):
    """Fix para el scroll en las gráficas."""
    def __init__(self, *args, auto_scale_getter=None, paused_getter=None, **kwargs):
        super().__init__(*args, **kwargs)
        self._auto_scale_getter = auto_scale_getter or (lambda: False)
        self._paused_getter     = paused_getter     or (lambda: False)

    def wheelEvent(self, event):
        # Zoom sólo si el usuario controla el viewport
        can_zoom = (not self._auto_scale_getter()) or self._paused_getter()
        if can_zoom:
            super().wheelEvent(event)      # zoom nativo de pyqtgraph
            return

        # Auto-scale activo y no pausado → scrollear el panel
        sa = self._find_ancestor_scroll_area()
        if sa is None:
            event.ignore()                 # no hay a dónde scrollear
            return

        bar = sa.verticalScrollBar()
        pd  = event.pixelDelta()           # trackpads
        ad  = event.angleDelta()           # rueda clásica (120 u = 1 notch)
        if not pd.isNull():
            bar.setValue(bar.value() - pd.y())
            event.accept()
        elif ad.y() != 0:
            step = bar.singleStep() or 20
            bar.setValue(bar.value() - int(ad.y() / 120.0 * step * 3))
            event.accept()
        else:
            event.ignore()

    def _find_ancestor_scroll_area(self):
        w = self.parentWidget()
        while w is not None:
            if isinstance(w, QtWidgets.QScrollArea):
                return w
            w = w.parentWidget()
        return None

# =====================================================================
#  CAPA DE COMUNICACIÓN UART 
# =====================================================================
class SerialManager(QtCore.QObject):
    #Signals
    telemetria = QtCore.pyqtSignal(float, float, float, float,float, float, float)  # t, theta, theta_dot, v_disco, u, vel_other, v_disco_other
    desconectado = QtCore.pyqtSignal(str)
    handshake_ok = QtCore.pyqtSignal(dict)
    handshake_perdido = QtCore.pyqtSignal()                
    ganancias_ack = QtCore.pyqtSignal(bool, str, dict)
    respuesta_exc = QtCore.pyqtSignal(bool, str)   # ACK/NACK de excitaciones 
    wmaxi_ack = QtCore.pyqtSignal(bool, float, str)                 # WMAXI recibido del firmware
    est_ack = QtCore.pyqtSignal(bool, str)
    obs_ack = QtCore.pyqtSignal(bool, dict, str)
    kal_ack = QtCore.pyqtSignal(bool, dict, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._port = None
        self.connected = False
        self._hilo_lectura = None
        self._detener = threading.Event()
        self._tx_lock = threading.Lock()
        self.t_acumulado = 0.0
        self.handshake_completo = False 
        self.firmware_version = "?"     

    # ---- Conexión ----
    def conectar(self, puerto, baud=115200):
        if not SERIAL_AVAILABLE:
            print("[UART] Error: pyserial no está instalado.")
            return False
            
        try:
            print(f"[UART] Conectando a {puerto} @ {baud} bps ...")
            # timeout=0.5 permite al hilo no quedarse bloqueado eternamente
            self._port = serial.Serial(puerto, baud, timeout=0.5)
            self.connected = True
            self._detener.clear()
            self.t_acumulado = 0.0  # Reiniciar tiempo

            self.handshake_completo = False
            self.firmware_version = "?"

            time.sleep(0.1)
            self._port.reset_input_buffer()
            self._escribir(b"HELLO?\n")
            print("[UART] -> HELLO? (reset solicitado)")

            # Lanzar el hilo receptor
            self._hilo_lectura = threading.Thread(target=self._leer_uart, daemon=True)
            self._hilo_lectura.start()
            return True
        except Exception as e:
            print(f"[UART] Error al conectar: {e}")
            self.connected = False
            return False

    def desconectar(self):
        print("[UART] Desconectando ...")
        self.connected = False
        self._detener.set() # Avisa al hilo que debe terminar
        if self._hilo_lectura:
            self._hilo_lectura.join(timeout=1.0)
        if self._port and self._port.is_open:
            self._port.close()
        self.handshake_completo = False

    # ---- Envíos ----
    def _escribir(self, datos: bytes):
        """Escritura serializada: la GUI y el hilo lector (READY) escriben al mismo puerto."""
        with self._tx_lock:
            if self._port and self._port.is_open:
                try:
                    self._port.write(datos)
                    self._port.flush()
                except Exception as e:
                    print(f"[UART] Error escribiendo: {e}")

    def _send(self, cmd: str, silencioso: bool = False):
        if not silencioso:
            print(f"[UART TX] -> {cmd}")
        self._escribir((cmd + '\n').encode())

    def enviar_ganancias(self, gains: dict):
        cmd = "L:" + ",".join(f"{gains[k]:.6f}" for k in sorted(gains))
        self._send(cmd)

    def enviar_modo(self, modo: str):
        self._send(f"MODE:{modo}")

    def enviar_manual(self, direccion: str):
        self._send(f"MANUAL:{direccion}", silencioso=True)   # se repite cada 150 ms

    def enviar_estop(self):
        self._send("ESTOP")

    def enviar_zero (self):
        """Envía el comando de cero al ESP32."""
        self._send("ZERO")

    def enviar_wmaxi(self, wmaxi: float):
        """Actualiza el multiplicador de velocidad máxima del péndulo en el firmware."""
        self._send(f"WMAXI:{wmaxi:.3f}")

    def enviar_ping(self):
        """Heartbeat silencioso. No imprime en consola para no saturar."""
        self._escribir(b"PING\n")

    def enviar_excitacion(self, tipo: str, params: dict):
        """Envía el comando de excitación al ESP32."""
        if tipo == "SQUARE":
            cmd = f"SQUARE:{params['freq']},{params['amp']},{params['T']}"
        elif tipo == "CHIRP":
            cmd = f"CHIRP:{params['f0']},{params['f1']},{params['T']},{params['amp']}"
        elif tipo == "PRBS":
            cmd = f"PRBS:{params['bitrate']},{params['amp']},{params['T']}"
        else:
            raise ValueError(f"Tipo desconocido: {tipo}")
        self._send(cmd)

    def enviar_stop_excitacion(self):
        self._send("EXC_STOP")

    def enviar_est(self, est: str):
        self._send(f"EST:{est}")

    def enviar_obs(self, obs: dict):
        cmd = "OBS:" + ",".join(f"{float(obs[k]):.8g}" for k in OBS_KEYS)
        self._send(cmd)

    def enviar_kal(self, kal: dict):
        cmd = "KAL:" + ",".join(f"{float(kal[k]):.8g}" for k in KAL_KEYS)
        self._send(cmd)

    # ---- Hilo Receptor (binario + ASCII) ----
    _HDR = b'\xAA\xBB'
    _PKT_LEN = 28            # 2 bytes de cabecera + 26 de carga (<ffffh)

    def _leer_uart(self):
        """
        Hilo lector. Lee en bloques con timeout (sin busy-wait: antes el bucle
        giraba al 100 % de CPU y competía por el GIL con la GUI) y separa
        telemetría binaria (0xAA 0xBB + 18 B) de líneas ASCII (HELLO/STATE/ACK).
        """
        buf = bytearray()
        while not self._detener.is_set():
            try:
                chunk = self._port.read(max(1, self._port.in_waiting))
            except Exception as e:
                if not self._detener.is_set():
                    print(f"[UART] Excepción leyendo puerto: {e}")
                    self.desconectado.emit(str(e))
                break
            if not chunk:
                continue
            buf.extend(chunk)
            self._consumir(buf)

    def _consumir(self, buf: bytearray):
        """Extrae de 'buf' todos los paquetes y líneas completos."""
        while buf:
            i_hdr = buf.find(self._HDR)
            cand = [i for i in (buf.find(b'\n'), buf.find(b'\r')) if i >= 0]
            i_nl = min(cand) if cand else -1

            # Línea de texto completa antes de la próxima cabecera binaria
            if i_nl >= 0 and (i_hdr < 0 or i_nl < i_hdr):
                linea = bytes(buf[:i_nl]).decode('ascii', errors='ignore').strip()
                del buf[:i_nl + 1]
                if linea and linea.isprintable():
                    self._procesar_linea_ascii(linea)
                continue

            if i_hdr >= 0:
                if i_hdr > 0:
                    del buf[:i_hdr]           # basura antes de la cabecera
                if len(buf) < self._PKT_LEN:
                    return                    # paquete incompleto: esperar más datos
                dt, theta, theta_dot, omega, theta_dot_other, omega_other, u_raw = struct.unpack(
                    '<ffffffh', bytes(buf[2:self._PKT_LEN]))
                if self._paquete_plausible(dt, theta, theta_dot, omega, theta_dot_other, omega_other):
                    del buf[:self._PKT_LEN]
                    self.t_acumulado += dt
                    self.telemetria.emit(self.t_acumulado, theta, theta_dot,
                                         omega, u_raw / 100.0, theta_dot_other, omega_other)
                else:
                    del buf[:1]               # falso positivo: resincronizar
                continue

            # Ni cabecera ni fin de línea: texto a medias (o ruido)
            if len(buf) > 256:
                buf.clear()
            return

    @staticmethod
    def _paquete_plausible(dt, theta, theta_dot, omega, theta_dot_other, omega_other):
        return (0.0 < dt < 1.0
                and all(math.isfinite(v) and abs(v) < 1e5
                        for v in (theta, theta_dot, omega, 
                                  theta_dot_other, omega_other)))

    def _procesar_linea_ascii(self, linea: str):
        if linea.startswith("HELLO,"):
            partes = linea.split(',')
            self.firmware_version = partes[1] if len(partes) > 1 else "?"
            print(f"[UART] HELLO recibido (firmware {self.firmware_version}). Enviando READY...")
            
            #para hello perdido
            if self.handshake_completo:
                self.handshake_perdido.emit()

            self.handshake_completo = False
            if self._port and self._port.is_open:
                self._escribir(b"READY\n")

        elif linea.startswith("STATE,"):
            try:
                # Formato: STATE,L=1.0,1.0,1.0;MODE=STANDBY
                cuerpo = linea[6:]
                kv = cuerpo.split(';')
                k_part = kv[0].split('=')[1]     # "1.0,1.0,1.0"
                modo = kv[1].split('=')[1] if len(kv) > 1 else "?"
                l1, l2, l3 = map(float, k_part.split(','))

                wmaxi = None
                est = None
                obs = None
                kal = None
                for item in kv[2:]:
                    if item.startswith("WMAXI="):
                        try:
                            wmaxi = float(item[6:])
                        except ValueError:
                            wmaxi = None
                    elif item.startswith("EST="):
                        est = item[4:].strip()
                    elif item.startswith("OBS="):
                        try:
                            vals = [float(x) for x in item[4:].strip().split(',')]
                            if len(vals) == len(OBS_KEYS):
                                obs = dict(zip(OBS_KEYS, vals))
                        except ValueError:
                            obs = None
                    elif item.startswith("KAL="):
                        try:
                            vals = [float(x) for x in item[4:].strip().split(',')]
                            if len(vals) == len(KAL_KEYS): kal = dict(zip(KAL_KEYS, vals))

                        except ValueError: kal = None
                
                self.handshake_completo = True
                self.handshake_ok.emit({
                    'firmware': self.firmware_version,
                    'L1': l1, 'L2': l2, 'L3': l3,
                    'mode': modo,
                    'wmaxi': wmaxi,
                    'EST': est,
                    'OBS': obs,
                    'KAL': kal,
                })
                print(f"[UART] STATE: L=({l1}, {l2}, {l3}), modo={modo}")
            except Exception as e:
                print(f"[UART] STATE mal formado '{linea}': {e}")
        elif linea.startswith("ACK,EXC,"):
            tipo = linea[8:].strip()
            print(f"[UART] ACK excitación: {tipo}")
            self.respuesta_exc.emit(True, f"Excitación {tipo}")

        elif linea.startswith("NACK,NOT_OPEN_LOOP"):
            print("[UART] NACK: no está en modo OPEN_LOOP")
            self.respuesta_exc.emit(False, "El ESP32 no está en modo Lazo Abierto")

        elif linea.startswith("ACK,ZERO"):
            print("[UART] ACK ZERO")
            self.respuesta_exc.emit(True, "Cero del encoder aplicado")

        elif linea.startswith("ACK,WMAXI,"):
            try: 
                val = float(linea[10:].strip())
                self.wmaxi_ack.emit(True, val, "")
                print(f"[UART] ACK WMAXI: {val:.3f}")
            except Exception as e:
                self.wmaxi_ack.emit(False, 0.0, f"ACK WMAXI mal formado: {linea}")
                print(f"[UART] ACK WMAXI mal formado '{linea}': {e}")

        elif linea.startswith("ACK,EST,"):
            val = linea[8:].strip()
            print(f"[UART] ACK estimador: {val}")
            self.est_ack.emit(True, val)

        elif linea.startswith("ACK,OBS="):
            try:
                vals = [float(x) for x in linea[8:].strip().split(',')]
                if len(vals)!=len(OBS_KEYS):
                    raise ValueError(f"se esperaban {len(OBS_KEYS)} valores")
                d = dict(zip(OBS_KEYS, vals))
                self.obs_ack.emit(True, d, "")
                print(f"[UART] ACK observador: {d}")
            except Exception as e:
                self.obs_ack.emit(False, {}, f"ACK OBS mal formado: {linea}")
                print(f"[UART] ACK OBS mal formado '{linea}': {e}")
        
        elif linea.startswith("ACK,KAL="):
            try:
                vals = [float(x) for x in linea[8:].strip().split(',')]
                self.kal_ack.emit(True, dict(zip(KAL_KEYS, vals)), "")
            except Exception as e: self.kal_ack.emit(False, {}, f"Fallo el Karma: {linea}")
        elif linea.startswith("NACK,KAL_UNSTABLE"):
            self.kal_ack.emit(False, {}, "KAL_UNSTABLE")
        
        elif linea.startswith("NACK,OBS_UNSTABLE"):
            print("[UART] NACK observador inestable")
            self.obs_ack.emit(False, {}, "OBS_UNSTABLE")

        elif linea.startswith("ACK,"):
        # "ACK,L=1.0,2.0,3.0"
            try:
                cuerpo = linea[4:]                    # "L=1.0,2.0,3.0"
                k, v = cuerpo.split('=', 1)
                l1, l2, l3 = map(float, v.split(','))
                self.ganancias_ack.emit(True, "", {'L1': l1, 'L2': l2, 'L3': l3})
                print(f"[UART] ACK ganancias: L=({l1}, {l2}, {l3})")
            except Exception as e:
                self.ganancias_ack.emit(False, f"ACK mal formado: {linea}", {})
                print(f"[UART] ACK mal formado '{linea}': {e}")

        elif linea.startswith("NACK,"):
            motivo = linea[5:].strip() or "desconocido"
            self.ganancias_ack.emit(False, motivo, {})
            print(f"[UART] NACK ganancias: {motivo}")

# =====================================================================
#  PANEL DE GRÁFICA  (pausa, auto-seguir X, autoescala Y)
# =====================================================================
class PlotPanel(QtWidgets.QWidget):
    def __init__(self, titulo, ylabel, color, color_dark=None, allow_overlay=False, parent=None):
        super().__init__(parent)
        self.paused = False
        self._color = color
        self._color_dark = color_dark if color_dark is not None else color
        self._allow_overlay = allow_overlay
        self._overlay_enabled = False

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # Opciones de la gráfica (fila compacta sobre el título)
        opciones = QtWidgets.QHBoxLayout()
        opciones.setContentsMargins(0, 0, 4, 0)
        opciones.addStretch()

        self.chk_pause = QtWidgets.QCheckBox("Pausar")
        self.chk_auto = QtWidgets.QCheckBox("Auto escalado")
        self.chk_auto.setChecked(True)
        self.chk_auto.setToolTip(
            "Ajusta automáticamente X e Y a los datos más recientes.\n"
            "Desmárcalo para hacer zoom/pan manual sobre el histórico."
        )

        for chk in (self.chk_pause, self.chk_auto):
            chk.setStyleSheet("font-size: 9pt;")
            opciones.addWidget(chk)
        layout.addLayout(opciones)

        # Gráfica estilo osciloscopio (fondo blanco, trazo grueso)
        self.plot = ScrollAwarePlotWidget(
            title=titulo,
            auto_scale_getter=lambda: self.chk_auto.isChecked(),
            paused_getter=lambda: self.paused,
        )
        self.plot.setLabel('left', ylabel)
        self.plot.setLabel('bottom', 'Tiempo (s)')
        for eje in ('left', 'bottom'):
            self.plot.getAxis(eje).enableAutoSIPrefix(False)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setDownsampling(auto=True, mode='peak')
        self.plot.setClipToView(True)
        self.curve = self.plot.plot(pen=pg.mkPen(color, width=2))
        self.curve.setSkipFiniteCheck(True)

        self.curve_other= None
        if allow_overlay:
            self.curve_other = self.plot.plot(pen=pg.mkPen(color, width=1.5, style=QtCore.Qt.PenStyle.DashLine))
            self.curve_other.setSkipFiniteCheck(True)
            self.curve_other.setVisible(False)
        
        layout.addWidget(self.plot)

        # Desactivamos el auto-range interno de pyqtgraph: nosotros gestionamos
        # los rangos desde update_data cuando "Auto escalado" está marcado.
        # Si lo dejáramos activo, competiría con el zoom/pan del usuario cuando
        # la casilla esté desmarcada.
        vb = self.plot.getViewBox()
        vb.enableAutoRange(axis=pg.ViewBox.XAxis, enable=False)
        vb.enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)
        self.setMinimumHeight(180)
        self.chk_pause.toggled.connect(self._on_pause)

    def set_theme(self, dark: bool):
        bg = '#121212' if dark else 'w'
        fg = '#e0e0e0' if dark else 'k'
        grid_alpha = 0.2 if dark else 0.25
        self.plot.setBackground(bg)
        for eje in ('left', 'bottom'):
            ax = self.plot.getAxis(eje)
            ax.setPen(pg.mkPen(fg, width=1))
            ax.setTextPen(pg.mkPen(fg))
        self.plot.showGrid(x=True, y=True, alpha=grid_alpha)
        # Título
        title_item = self.plot.plotItem.titleLabel
        if title_item:
            title_item.setText(title_item.text, color=fg)
        self.curve.setPen(pg.mkPen(self._color_dark if dark else self._color, width=2))

        if self.curve_other is not None:
            self.curve_other.setPen(pg.mkPen(self._color_dark if dark else self._color,
            width=1.5, style=QtCore.Qt.PenStyle.DashLine))

    def _on_pause(self, activo):
        self.paused = activo

    def set_overlay(self, enabled: bool):
        if self.curve_other is None:
            return
        self._overlay_enabled = enabled
        self.curve_other.setVisible(enabled)
        if not enabled:
            self.curve_other.setData([], [])

    def update_data(self, t_array, y_array, y_other=None):
        """Actualiza la curva. Si 'Auto escalado' está marcado, ajusta X e Y."""
        if self.paused:
            return
        self.curve.setData(t_array, y_array)

        if self._overlay_enabled and self.curve_other is not None and y_other is not None:
            self.curve_other.setData(t_array, y_other)
        elif self.curve_other is not None:
            self.curve_other.setData([], [])

        if not self.chk_auto.isChecked():
            return                       # el usuario manda: no tocar los rangos

        if t_array.size < 2:
            return

        t0, t1 = float(t_array[0]), float(t_array[-1])
        # Protección: t0 >= t1 indica un buffer momentáneamente no-monotónico
        # (p. ej. reconexión sin limpiar histórico). No fijamos rango inválido.
        if t1 <= t0:
            return

        vb = self.plot.getViewBox()
        vb.setXRange(t0, t1, padding=0.0)

        ymin = float(np.min(y_array))
        ymax = float(np.max(y_array))
        
        if (self._overlay_enabled and y_other is not None and y_other.size > 0):
            ymin = min(ymin, float(np.min(y_other)))
            ymax = max(ymax, float(np.max(y_other)))
        
        if ymax > ymin:
            pad = 0.05 * (ymax - ymin)   # 5 % de aire arriba y abajo
            vb.setYRange(ymin - pad, ymax + pad, padding=0.0)
        else:
            # Señal plana (típico en θ ≈ 0 durante estabilización): evitamos
            # un rango de altura cero que aplastaría la traza contra el eje.
            vb.setYRange(ymin - 0.5, ymax + 0.5, padding=0.0)


# =====================================================================
#  PANEL DE PLANO DE FASE  (velocidad vs posición de la barra)
# =====================================================================
class PhasePanel(QtWidgets.QWidget):
    """
    Plano de fase de la barra: eje X = posición, eje Y = velocidad angular.

      · «θ»  (por defecto): el ángulo tal cual lo manda el firmware. 0 = barra colgando;
             la vertical hacia arriba está en ±π.
      · «e»  : error respecto a la vertical, envuelto a [-π, π). 0 = barra arriba.
             Al cruzar ±π la traza se corta (no se dibuja la línea horizontal falsa).

    La estela se dibuja en N_SEG tramos con opacidad creciente (lo más viejo, más tenue);
    el punto rojo es la muestra actual.
    """
    N_SEG = 4
    ALFAS = (45, 95, 170, 255)
    COLOR = (25, 100, 200)
    C_ARRIBA = (200, 40, 40)      # equilibrio inestable
    C_ABAJO = (40, 150, 70)       # equilibrio estable

    def __init__(self, parent=None):
        super().__init__(parent)
        self.paused = False
        self.modo_x = 0           # 0 = θ, 1 = e
        self._datos = None        # (t, theta, omega) de la última actualización

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # ---- fila de opciones ----
        fila = QtWidgets.QHBoxLayout()
        fila.setContentsMargins(4, 2, 4, 0)
        fila.addWidget(QtWidgets.QLabel("Posición:"))
        self.cmb_x = NoWheelComboBox()
        self.cmb_x.addItems(["θ (rad) · 0 = colgando", "e (rad) · 0 = vertical arriba"])
        self.cmb_x.setToolTip(
            "θ: ángulo continuo del encoder (la vertical está en ±π).\n"
            "e: error respecto a la vertical, envuelto a [-π, π): útil para ver el balanceo."
        )
        self.cmb_x.currentIndexChanged.connect(self._on_modo_x)
        fila.addWidget(self.cmb_x)
        fila.addSpacing(12)
        fila.addWidget(QtWidgets.QLabel("Estela:"))
        self.sp_ventana = FlexibleDoubleSpinBox()
        self.sp_ventana.setRange(0.5, float(HISTORIAL_S))
        self.sp_ventana.setValue(5.0)
        self.sp_ventana.setSingleStep(0.5)
        self.sp_ventana.setSuffix(" s")
        self.sp_ventana.setToolTip("Cuántos segundos de trayectoria se dibujan.")
        self.sp_ventana.valueChanged.connect(lambda _v: self._dibujar())
        fila.addWidget(self.sp_ventana)
        fila.addStretch()

        self.chk_pause = QtWidgets.QCheckBox("Pausar")
        self.chk_auto = QtWidgets.QCheckBox("Auto escalado")
        self.chk_auto.setChecked(True)
        self.chk_auto.setToolTip("Ajusta X e Y a la estela visible.\n"
                                 "Desmárcalo para hacer zoom/pan manual.")
        for chk in (self.chk_pause, self.chk_auto):
            chk.setStyleSheet("font-size: 9pt;")
            fila.addWidget(chk)
        self.chk_pause.toggled.connect(self._on_pause)
        layout.addLayout(fila)

        # ---- gráfica ----
        self.plot = ScrollAwarePlotWidget(
            title="Plano de fase de la barra",
            auto_scale_getter=lambda: self.chk_auto.isChecked(),
            paused_getter=lambda: self.paused,
        )
        self.plot.setLabel('left', 'ω barra (rad/s)')
        self.plot.setLabel('bottom', 'θ (rad)')
        for eje in ('left', 'bottom'):
            self.plot.getAxis(eje).enableAutoSIPrefix(False)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        vb = self.plot.getViewBox()
        vb.enableAutoRange(axis=pg.ViewBox.XAxis, enable=False)
        vb.enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)

        # Referencias: ω = 0 y los equilibrios (θ = 0, ±π)
        trazo = QtCore.Qt.PenStyle.DashLine
        self.ref_h = pg.InfiniteLine(pos=0, angle=0,
                                     pen=pg.mkPen((130, 130, 130), width=1, style=trazo))
        self.plot.addItem(self.ref_h, ignoreBounds=True)
        self.ref_v = []
        for pos in (-math.pi, 0.0, math.pi):
            ln = pg.InfiniteLine(pos=pos, angle=90, pen=pg.mkPen((130, 130, 130)),
                                 label="", labelOpts={'position': 0.96, 'color': (80, 80, 80)})
            self.plot.addItem(ln, ignoreBounds=True)
            self.ref_v.append(ln)
        self._aplicar_referencias()

        # Estela (tramos con opacidad creciente) y punto actual
        self.curvas = [self.plot.plot(pen=pg.mkPen(self.COLOR + (a,), width=2))
                       for a in self.ALFAS]
        self.punto = pg.ScatterPlotItem(size=11, pen=pg.mkPen('k', width=1),
                                        brush=pg.mkBrush(220, 30, 30))
        self.plot.addItem(self.punto)
        layout.addWidget(self.plot, stretch=1)

        # ---- lectura numérica ----
        self.lbl_lectura = QtWidgets.QLabel("θ = —    e = —    ω = —")
        self.lbl_lectura.setStyleSheet("font-family: monospace; font-size: 10pt; padding: 2px 6px;")
        layout.addWidget(self.lbl_lectura)
        self.setMinimumHeight(260)

    # ---- eventos de la UI ----
    def _on_pause(self, activo):
        self.paused = activo

    def _on_modo_x(self, idx):
        self.modo_x = idx
        self.plot.setLabel('bottom', 'θ (rad)' if idx == 0 else 'e = θ − π, envuelto (rad)')
        self._aplicar_referencias()
        self._dibujar()

    def _aplicar_referencias(self):
        """Etiqueta y color de las líneas verticales según lo que representa cada eje X."""
        textos = ("arriba", "abajo", "arriba") if self.modo_x == 0 else ("abajo", "arriba", "abajo")
        for ln, txt in zip(self.ref_v, textos):
            color = self.C_ARRIBA if txt == "arriba" else self.C_ABAJO
            ln.setPen(pg.mkPen(color, width=1, style=QtCore.Qt.PenStyle.DashLine))
            try:
                ln.label.setFormat(txt)
                ln.label.setColor(color)
            except Exception:
                pass

    def set_theme(self, dark: bool):
        """Adapta fondo, ejes, título y lecturas al tema activo."""
        bg = '#121212' if dark else 'w'
        fg = '#e0e0e0' if dark else 'k'
        grid_alpha = 0.2 if dark else 0.25
        self.plot.setBackground(bg)
        for eje in ('left', 'bottom'):
            ax = self.plot.getAxis(eje)
            ax.setPen(pg.mkPen(fg, width=1))
            ax.setTextPen(pg.mkPen(fg))
        self.plot.showGrid(x=True, y=True, alpha=grid_alpha)
        title_item = self.plot.plotItem.titleLabel
        if title_item:
            title_item.setText(title_item.text, color=fg)
        self.lbl_lectura.setStyleSheet(
            f"font-family: monospace; font-size: 10pt; padding: 2px 6px; color: {fg};"
        )

    def limpiar(self):
        self._datos = None
        for c in self.curvas:
            c.setData([], [])
        self.punto.setData([], [])
        self.lbl_lectura.setText("θ = —    e = —    ω = —")
        vb = self.plot.getViewBox()
        vb.setXRange(-math.pi, math.pi, padding=0.0)
        vb.setYRange(-1, 1, padding=0.0)

    # ---- datos ----
    def update_data(self, t, theta, omega):
        """t [s], theta [rad], omega [rad/s]: arrays con el histórico completo."""
        self._datos = (t, theta, omega)
        if not self.paused:
            self._dibujar()

    def _dibujar(self):
        if self._datos is None:
            return
        t, theta, omega = self._datos
        if t.size < 2:
            return
        i0 = int(np.searchsorted(t, t[-1] - self.sp_ventana.value()))
        th, om = theta[i0:], omega[i0:]
        if th.size < 2:
            return

        if self.modo_x == 0:
            x, y = th, om
        else:
            e = np.mod(th, 2.0 * math.pi) - math.pi
            cortes = np.flatnonzero(np.abs(np.diff(e)) > math.pi) + 1
            x = np.insert(e, cortes, np.nan)       # corta la traza al cruzar ±π
            y = np.insert(om, cortes, np.nan)

        n = x.size
        lim = np.linspace(0, n - 1, self.N_SEG + 1).astype(int)
        for k, curva in enumerate(self.curvas):
            a, b = lim[k], lim[k + 1] + 1          # el tramo arranca donde acabó el anterior
            curva.setData(x[a:b], y[a:b], connect='finite')
        self.punto.setData([x[-1]], [y[-1]])

        e_deg = math.degrees((float(th[-1]) % (2.0 * math.pi)) - math.pi)
        self.lbl_lectura.setText(
            f"θ = {th[-1]:+8.3f} rad    e = {e_deg:+8.2f}°    ω = {om[-1]:+8.3f} rad/s")

        if self.chk_auto.isChecked():
            self._autoescala(x, y)

    def _autoescala(self, x, y):
        vb = self.plot.getViewBox()
        for datos, minspan, fijar in ((x, 0.01, vb.setXRange), (y, 0.05, vb.setYRange)):
            lo, hi = float(np.nanmin(datos)), float(np.nanmax(datos))
            mitad = 0.55 * max(hi - lo, minspan)    # 5 % de aire a cada lado
            centro = 0.5 * (lo + hi)
            fijar(centro - mitad, centro + mitad, padding=0.0)


# =====================================================================
#  PANEL DE ESQUEMA / ANIMACIÓN 2D DEL PÉNDULO CON DISCO
# =====================================================================
class PendulumCanvas(QtWidgets.QWidget):
    """
    Área de dibujo vectorial (QPainter) para el péndulo invertido de disco de inercia.
    Coordenadas físicas:
      - theta = 0 rad: barra colgando hacia abajo
      - theta = ±pi rad: barra vertical invertida (arriba)
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 320)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Expanding)
        self.theta = 0.0
        self.phi_disco = 0.0
        self.dark = True

    def set_angles(self, theta: float, phi_disco: float):
        self.theta = theta
        self.phi_disco = phi_disco
        self.update()

    def set_theme(self, dark: bool):
        self.dark = dark
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()
        cx = w / 2.0
        cy = h / 2.0

        # Fondo del lienzo
        bg_color = QtGui.QColor('#121212' if self.dark else '#ffffff')
        p.fillRect(self.rect(), bg_color)

        # Dimensiones escaladas relativas al tamaño del widget
        base_dim = min(w, h)
        L = base_dim * 0.36
        r_pivot = max(6.0, base_dim * 0.022)
        r_disco = max(18.0, base_dim * 0.05)
        ancho_barra = max(5.0, base_dim * 0.022)

        # Paleta de colores
        if self.dark:
            c_grid = QtGui.QColor(255, 255, 255, 25)
            c_ref_text = QtGui.QColor(160, 160, 160)
            c_base = QtGui.QColor(70, 70, 70)
            c_barra = QtGui.QColor(100, 110, 120)
            c_barra_borde = QtGui.QColor(60, 70, 80)
            
            c_disco_bg = QtGui.QColor(15, 25, 15)
            c_neon = QtGui.QColor(30, 255, 90)
            
            c_pivot_fill = QtGui.QColor(40, 40, 40)
            c_pivot_edge = QtGui.QColor(120, 120, 120)
        else:
            c_grid = QtGui.QColor(0, 0, 0, 25)
            c_ref_text = QtGui.QColor(110, 110, 110)
            c_base = QtGui.QColor(190, 190, 190)
            c_barra = QtGui.QColor(140, 155, 170)
            c_barra_borde = QtGui.QColor(70, 85, 100)
            
            c_disco_bg = QtGui.QColor(240, 255, 240)
            c_neon = QtGui.QColor(33, 200, 80)
            
            c_pivot_fill = QtGui.QColor(245, 245, 245)
            c_pivot_edge = QtGui.QColor(80, 80, 80)

        # 1. Círculo y líneas de referencia
        pen_ref = QtGui.QPen(c_grid, 1.2, QtCore.Qt.PenStyle.DashLine)
        p.setPen(pen_ref)
        p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        p.drawEllipse(QtCore.QPointF(cx, cy), L, L)
        p.drawLine(QtCore.QPointF(cx, cy - L - 15), QtCore.QPointF(cx, cy + L + 15))
        p.drawLine(QtCore.QPointF(cx - L - 15, cy), QtCore.QPointF(cx + L + 15, cy))

        font_ref = QtGui.QFont("sans-serif", 8)
        p.setFont(font_ref)
        p.setPen(QtGui.QPen(c_ref_text))
        p.drawText(int(cx + 6), int(cy - L - 4), "θ = 180° (Invertido)")

        # 2. Soporte/base fija del pivote
        p.setPen(QtGui.QPen(c_base, 1.5))
        p.setBrush(QtGui.QBrush(c_base))
        base_h = r_pivot * 1.6
        base_w = r_pivot * 2.8
        soporte = QtGui.QPolygonF([
            QtCore.QPointF(cx, cy),
            QtCore.QPointF(cx - base_w / 2, cy + base_h),
            QtCore.QPointF(cx + base_w / 2, cy + base_h),
        ])
        p.drawPolygon(soporte)

        # 3. Cinemática de la barra
        tip_x = cx + L * math.sin(self.theta)
        tip_y = cy + L * math.cos(self.theta)

        p.save()
        p.translate(cx, cy)
        p.rotate(-math.degrees(self.theta))

        pen_barra = QtGui.QPen(c_barra_borde, 1.5)
        p.setPen(pen_barra)
        p.setBrush(QtGui.QBrush(c_barra))
        rect_barra = QtCore.QRectF(-ancho_barra / 2.0, -r_pivot * 0.5, ancho_barra, L + r_pivot * 0.5)
        p.drawRoundedRect(rect_barra, ancho_barra / 2.0, ancho_barra / 2.0)
        p.restore()

        # 4. Dibujo del Disco de inercia (Símbolo Unificado)
        p.save()
        p.translate(tip_x, tip_y)
        p.rotate(-math.degrees(self.phi_disco))

        # Fondo oscuro circular (tapa la barra del péndulo por detrás)
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QBrush(c_disco_bg))
        p.drawEllipse(QtCore.QPointF(0, 0), r_disco, r_disco)

        # Dimensiones proporcionales del logo
        T = r_disco * 0.18
        R_out = r_disco
        R_in = R_out - T
        r_out = r_disco * 0.42
        r_in = r_out - T

        # A) Anillo exterior
        path_outer = QtGui.QPainterPath()
        path_outer.addEllipse(QtCore.QPointF(0, 0), R_out, R_out)
        path_hole_outer = QtGui.QPainterPath()
        path_hole_outer.addEllipse(QtCore.QPointF(0, 0), R_in, R_in)
        path_outer = path_outer.subtracted(path_hole_outer)

        # B) Anillo interior
        path_inner = QtGui.QPainterPath()
        path_inner.addEllipse(QtCore.QPointF(0, 0), r_out, r_out)
        path_hole_inner = QtGui.QPainterPath()
        path_hole_inner.addEllipse(QtCore.QPointF(0, 0), r_in, r_in)
        path_inner = path_inner.subtracted(path_hole_inner)

        # C) Barras horizontales conectando los anillos
        path_bars = QtGui.QPainterPath()
        
        # Superposición intencional (Overlap): hace que la barra invada el anillo 
        # para que al unirlos no queden líneas vacías (aliasing)
        overlap = r_disco * 0.05
        
        # Ancho de las barras: ya NO tocan el círculo exterior (R_in), 
        # apenas sobrepasan el círculo interior (r_out) por una fracción de T
        bar_hw = r_out + T * 0.75 
        bar_w = bar_hw * 2

        # Rectángulo superior (extendemos su altura hacia abajo para solapar)
        path_bars.addRect(QtCore.QRectF(-bar_hw, -(r_out + T), bar_w, T + overlap))
        
        # Rectángulo inferior (subimos el inicio hacia arriba y mantenemos la altura extra)
        path_bars.addRect(QtCore.QRectF(-bar_hw, r_out - overlap, bar_w, T + overlap))

        # D) UNIFICACIÓN GEOMÉTRICA (Fusión perfecta)
        logo_path = path_outer.united(path_inner).united(path_bars).simplified()

        # E) Dibujar el resplandor difuminado (Halo)
        glow_color = QtGui.QColor(c_neon)
        glow_color.setAlpha(25)
        p.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        
        # Trazamos el borde con opacidades múltiples para un brillo suave
        for thickness in [T * 1.8, T * 1.0, T * 0.4]:
            pen_glow = QtGui.QPen(glow_color, thickness)
            pen_glow.setJoinStyle(QtCore.Qt.PenJoinStyle.RoundJoin)
            p.strokePath(logo_path, pen_glow)

        # F) Dibujar el logo verde principal (Sólido)
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.fillPath(logo_path, QtGui.QBrush(c_neon))

        p.restore()

        # 5. Pivote central fijo
        p.setPen(QtGui.QPen(c_pivot_edge, 2.0))
        p.setBrush(QtGui.QBrush(c_pivot_fill))
        p.drawEllipse(QtCore.QPointF(cx, cy), r_pivot, r_pivot)
        p.setBrush(QtGui.QBrush(c_barra_borde))
        p.drawEllipse(QtCore.QPointF(cx, cy), r_pivot * 0.4, r_pivot * 0.4)

        p.end()

class SchematicPanel(QtWidgets.QWidget):
    """
    Panel contenedor de la pestaña de esquema animado en tiempo real.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.paused = False

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # Fila superior
        top_bar = QtWidgets.QHBoxLayout()
        top_bar.setContentsMargins(4, 2, 4, 2)

        self.lbl_titulo = QtWidgets.QLabel("Animación en tiempo real del péndulo")
        self.lbl_titulo.setStyleSheet("font-weight: bold; font-size: 11pt;")
        top_bar.addWidget(self.lbl_titulo)
        top_bar.addStretch()

        self.chk_pause = QtWidgets.QCheckBox("Pausar")
        self.chk_pause.setStyleSheet("font-size: 9pt;")
        self.chk_pause.toggled.connect(self._on_pause)
        top_bar.addWidget(self.chk_pause)
        layout.addLayout(top_bar)

        # Canvas de dibujo 2D
        self.canvas = PendulumCanvas(self)
        layout.addWidget(self.canvas, stretch=1)

        # Tarjetas de datos mínimos
        info_frame = QtWidgets.QFrame()
        info_frame.setObjectName("infoFrame")
        info_layout = QtWidgets.QHBoxLayout(info_frame)
        info_layout.setContentsMargins(12, 6, 12, 6)
        info_layout.setSpacing(16)

        self.lbl_deg = QtWidgets.QLabel("θ: 0.0°")
        self.lbl_deg.setStyleSheet("font-family: monospace; font-size: 12pt; font-weight: bold;")
        self.lbl_deg.setToolTip("Ángulo continuo de la barra en grados.")

        self.lbl_rad = QtWidgets.QLabel("θ: 0.000 rad")
        self.lbl_rad.setStyleSheet("font-family: monospace; font-size: 10pt;")

        self.lbl_omega = QtWidgets.QLabel("ω: 0.000 rad/s")
        self.lbl_omega.setStyleSheet("font-family: monospace; font-size: 10pt;")

        self.lbl_vdisco = QtWidgets.QLabel("ω_disco: 0.0 rad/s")
        self.lbl_vdisco.setStyleSheet("font-family: monospace; font-size: 10pt;")

        self.lbl_estado = QtWidgets.QLabel("Estado: Colgando")
        self.lbl_estado.setStyleSheet("font-size: 10pt; font-style: italic;")

        info_layout.addWidget(self.lbl_deg)
        info_layout.addWidget(self.lbl_rad)
        info_layout.addWidget(self.lbl_omega)
        info_layout.addWidget(self.lbl_vdisco)
        info_layout.addStretch()
        info_layout.addWidget(self.lbl_estado)

        layout.addWidget(info_frame)

    def _on_pause(self, activo):
        self.paused = activo

    def set_theme(self, dark: bool):
        self.canvas.set_theme(dark)
        color_texto = "#00A8ff" if dark else "#000000"
        color_sub = "#e0e0e0" if dark else "#333333"
        bg_frame = "#1a1a1a" if dark else "#f0f4f8"
        border_frame = "#2d2d2d" if dark else "#dcdcdc"

        self.lbl_deg.setStyleSheet(f"font-family: monospace; font-size: 12pt; font-weight: bold; color: {color_texto};")
        for lbl in (self.lbl_rad, self.lbl_omega, self.lbl_vdisco, self.lbl_estado, self.lbl_titulo):
            lbl.setStyleSheet(f"color: {color_sub}; font-size: 10pt;")
        self.lbl_titulo.setStyleSheet(f"color: {color_sub}; font-weight: bold; font-size: 11pt;")

        info_frame = self.findChild(QtWidgets.QFrame, "infoFrame")
        if info_frame:
            info_frame.setStyleSheet(
                f"#infoFrame {{ background-color: {bg_frame}; border: 1px solid {border_frame}; border-radius: 6px; }}"
            )

    def update_telemetria(self, theta: float, omega: float, v_disco: float, dt: float, modo_sist: str):
        """Actualiza el esquema con los valores instantáneos más recientes."""
        if self.paused:
            return

        self.canvas.phi_disco = (self.canvas.phi_disco + v_disco * dt) % (2.0 * math.pi)
        self.canvas.set_angles(theta, self.canvas.phi_disco)

        deg = math.degrees(theta)
        e_deg = math.degrees((theta % (2.0 * math.pi)) - math.pi)

        self.lbl_deg.setText(f"θ: {deg:+7.1f}°")
        self.lbl_rad.setText(f"θ: {theta:+7.3f} rad")
        self.lbl_omega.setText(f"ω: {omega:+7.2f} rad/s")
        self.lbl_vdisco.setText(f"ω_d: {v_disco:+7.1f} rad/s")

        if abs(e_deg) < 10.0:
            cabe_zona = "Vertical Invertida"
        elif abs(deg % 360.0) < 15.0:
            cabe_zona = "Colgan2"
        else:
            cabe_zona = "En movimiento"
        self.lbl_estado.setText(f"Modo: {modo_sist} | Zona: {cabe_zona}")

    def limpiar(self):
        self.canvas.set_angles(0.0, 0.0)
        self.lbl_deg.setText("θ: 0.0°")
        self.lbl_rad.setText("θ: 0.000 rad")
        self.lbl_omega.setText("ω: 0.000 rad/s")
        self.lbl_vdisco.setText("ω_d: 0.0 rad/s")
        self.lbl_estado.setText("Estado: Reposo")

# =====================================================================
#  VENTANA PRINCIPAL
# =====================================================================
class InterfazPendulo(QtWidgets.QMainWindow):

    MODOS = [
        ("STANDBY",       "En Espera / Stop"),
        ("MANUAL",        "Modo Manual (Teclas)"),
        ("OPEN_LOOP",     "Lazo Abierto (Identificación)"),
        ("STABILIZATION", "Lazo Cerrado (Estabilización)"),
        ("AUTO",          "Automático (Swing-up + Estab.)"),
    ]

    # Teclas del modo manual
    TECLAS_MANUAL = {
        QtCore.Qt.Key.Key_Left: "LEFT",
        QtCore.Qt.Key.Key_Right: "RIGHT",
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control de Péndulo Invertido - Osciloscopio")
        self.resize(1280, 820)

        # ---------- Estado ----------
        self.mode = "STANDBY"
        self.motor_moving = False
        self.recording = False
        self.recorded_data = []
        self.t = 0.0
        self.gains = {"L1": 0.0, "L2": 0.0, "L3": 0.0}
        self.obs_gains = dict(OBS_DEFAULT)
        self.kal_gains = dict(KAL_DEFAULT)
        # ACK de ganancias
        self._ack_pendiente = False
        self._ack_timer = QtCore.QTimer(self)
        self._ack_timer.setSingleShot(True)
        self._ack_timer.timeout.connect(self._on_ack_timeout)

        # Control manual: qué entradas (botón / tecla) están sostenidas
        self._manual_held = set()
        self._manual_last = "STOP"
        self._manual_timer = QtCore.QTimer(self)
        self._manual_timer.timeout.connect(self._manual_keepalive)

        # Sesión / handshake
        self._sincronizado = False          # ¿ya se cargaron las ganancias del firmware?
        self._hs_timer = QtCore.QTimer(self)
        self._hs_timer.setSingleShot(True)
        self._hs_timer.timeout.connect(self._check_handshake_timeout)
        self._evento_timer = QtCore.QTimer(self)
        self._evento_timer.setSingleShot(True)
        self._evento_timer.timeout.connect(lambda: self.lbl_evento.setText(""))

        # ---------- Buffers (histórico deslizante) ----------
        n = HISTORIAL_S * FS_HZ
        self.t_buf = deque(maxlen=n)
        self.pos_buf = deque(maxlen=n)
        self.vel_buf = deque(maxlen=n)
        self.disco_buf = deque(maxlen=n)
        self.u_buf= deque(maxlen=n)
        self.vel_other_buf = deque(maxlen=n)
        self.disco_other_buf = deque(maxlen=n)

        # ---------- Comunicación ----------
        self.serial = SerialManager(self)
        self.serial.telemetria.connect(self._on_muestra)
        self.serial.desconectado.connect(self._on_serial_desconectado)
        self.serial.handshake_ok.connect(self._on_handshake)
        self.serial.handshake_perdido.connect(self._on_handshake_perdido)
        self.serial.ganancias_ack.connect(self._on_ganancias_ack)
        self.serial.respuesta_exc.connect(self._on_respuesta_exc)
        self.serial.wmaxi_ack.connect(self._on_wmaxi_ack)
        self.serial.est_ack.connect(self._on_est_ack)
        self.serial.obs_ack.connect(self._on_obs_ack)
        self.serial.kal_ack.connect(self._on_kal_ack)

        # ---------- UI ----------
        self._build_ui()
        self.setStyleSheet(ESTILO_GLOBAL_BATMAN)
        self.toggle_batman.blockSignals(True)
        self.toggle_batman.setChecked(True)
        self.toggle_batman.blockSignals(False)
        self._on_batman_toggled(True)

        self._on_mode_changed("STANDBY", forzar=True)
        self._set_controles_conectados(False)

        # Captura de flechas ← → en modo manual (ver eventFilter)
        QtWidgets.QApplication.instance().installEventFilter(self)

        # Atajo de emergencia: Esc = KILL SWITCH
        self._sc_kill = QtGui.QShortcut(QtGui.QKeySequence("Esc"), self)
        self._sc_kill.activated.connect(self._on_kill_switch)

        # ---------- Temporizadores ----------
        # La telemetría llega por UART a FS_HZ (100 Hz) desde el hilo lector; el
        # repintado se hace a PLOT_HZ (20 Hz) para no saturar el event loop.

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.setTimerType(QtCore.Qt.TimerType.CoarseTimer)
        self.plot_timer.timeout.connect(self._plot_tick)
        self.plot_timer.start(PLOT_DT_MS)
        self.port_timer = QtCore.QTimer(self)
        self.port_timer.timeout.connect(self._refresh_puertos)
        self.port_timer.start(1500)   # cada 1.5 s
        self.ping_timer = QtCore.QTimer(self)
        self.ping_timer.timeout.connect(self._send_ping)
        self.ping_timer.start(ping_interval_ms)   # cada 0.5 s

        self._status_div = 0

    # -----------------------------------------------------------------
    #  Construcción de la interfaz
    # -----------------------------------------------------------------
    def _build_ui(self):
        widget_central = QtWidgets.QWidget()
        self.setCentralWidget(widget_central)
        layout_principal = QtWidgets.QHBoxLayout(widget_central)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)

        # --- Panel izquierdo: selector de gráficas + osciloscopio con scroll ---
        panel_graficas = QtWidgets.QWidget()
        layout_graficas = QtWidgets.QVBoxLayout(panel_graficas)
        layout_graficas.setContentsMargins(0, 0, 0, 0)
        layout_graficas.setSpacing(6)

        # Selector de gráficas visibles (fijo arriba, no scrollea)
        layout_graficas.addWidget(self._crear_grupo_seleccion_graficas())

        # Área de scroll que contiene las gráficas
        self.scroll_graficas = QtWidgets.QScrollArea()
        self.scroll_graficas.setWidgetResizable(True)
        self.scroll_graficas.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self.scroll_graficas.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll_graficas.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAsNeeded)

        # Contenedor interno del scroll
        self.widget_graficas = QtWidgets.QWidget()
        self.layout_paneles = QtWidgets.QVBoxLayout(self.widget_graficas)
        self.layout_paneles.setContentsMargins(0, 0, 6, 0)
        self.layout_paneles.setSpacing(6)

        # Crear los 4 paneles
        self.panel_pos_barra = PlotPanel("Posición de la Barra (rad)", "θ (rad)", 'b', color_dark='#4fc3f7')  # azul
        self.panel_vel_barra = PlotPanel("Velocidad de la Barra (rad/s)", "ω (rad/s)", 'r', color_dark='#ff5252', allow_overlay=True)  # rojo
        self.panel_vel_disco = PlotPanel("Velocidad del Disco / Puente H (rad/s)",
                                        "ω disco (rad/s)", 'g', color_dark='#69f0ae', allow_overlay=True)  # verde
        self.panel_u         = PlotPanel("Señal de control · u", "u (unid. de control)", (255, 140, 0), color_dark='#ffb74d')  # naranja

        # Diccionario para acceder por clave desde el selector
        self._paneles_graficas = {
            "pos":   self.panel_pos_barra,
            "vel":   self.panel_vel_barra,
            "disco": self.panel_vel_disco,
            "u":     self.panel_u,
        }

        # Añadir al layout interno
        for p in self._paneles_graficas.values():
            self.layout_paneles.addWidget(p)

        self.scroll_graficas.setWidget(self.widget_graficas)
        layout_graficas.addWidget(self.scroll_graficas, stretch=1)

        # --- Panel derecho: controles con scroll + Seguridad siempre visible ---
        panel_derecho = QtWidgets.QWidget()
        panel_derecho.setMinimumWidth(300)
        layout_derecho = QtWidgets.QVBoxLayout(panel_derecho)
        layout_derecho.setContentsMargins(0, 0, 0, 0)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        contenido = QtWidgets.QWidget()
        layout_controles = QtWidgets.QVBoxLayout(contenido)
        layout_controles.setContentsMargins(0, 0, 6, 0)
        layout_controles.addWidget(self._crear_grupo_conexion())
        layout_controles.addWidget(self._crear_grupo_modos())
        layout_controles.addWidget(self._crear_grupo_parametros())
        layout_controles.addWidget(self._crear_grupo_observador())
        layout_controles.addWidget(self._crear_grupo_lazo_abierto())
        layout_controles.addWidget(self._crear_grupo_grabacion())
        layout_controles.addStretch()
        scroll.setWidget(contenido)

        layout_derecho.addWidget(self._crear_grupo_apariencia())
        layout_derecho.addWidget(scroll, stretch=1)
        # El KILL SWITCH queda fuera del scroll: nunca puede quedar oculto
        layout_derecho.addWidget(self._crear_grupo_seguridad())

        # Pestañas: señales en el tiempo / plano de fase de la barra
        self.tabs_graficas = QtWidgets.QTabWidget()
        self.tabs_graficas.setDocumentMode(True)
        # Sin foco: las flechas ← → del modo manual no deben cambiar de pestaña
        self.tabs_graficas.tabBar().setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        self.tabs_graficas.addTab(panel_graficas, "Señales")
        self.panel_fase = PhasePanel()
        self.tabs_graficas.addTab(self.panel_fase, "Plano de fase")
        self.panel_esquema = SchematicPanel() 
        self.tabs_graficas.addTab(self.panel_esquema, "Esquema Péndulo")  # ← NUEVO
        splitter.addWidget(self.tabs_graficas)
        splitter.addWidget(panel_derecho)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([930, 350])
        layout_principal.addWidget(splitter)

        # Mensajes de evento (no los pisa el refresco de la barra de estado)
        self.lbl_evento = QtWidgets.QLabel("")
        self.lbl_evento.setStyleSheet("color: #b00; padding-right: 8px;")
        self.statusBar().addPermanentWidget(self.lbl_evento)

    # ---- Grupo: conexión UART ----
    def _crear_grupo_conexion(self):
        g = QtWidgets.QGroupBox("Conexión UART")
        v = QtWidgets.QVBoxLayout(g)

        self.btn_zero = QtWidgets.QPushButton("Reset (poner cero)")
        self.btn_zero.setToolTip(
            "Fija la posición actual de la barra como θ = 0 (colgando).\n"
            "La vertical hacia arriba queda en θ = ±π.\n"
            "Solo disponible en STANDBY con la barra quieta."
        )
        self.btn_zero.clicked.connect(self._on_zero)
        v.addWidget(self.btn_zero)

        h = QtWidgets.QHBoxLayout()
        h.addWidget(QtWidgets.QLabel("Puerto:"))
        self.cmb_port = NoWheelComboBox()
        self.cmb_port.addItems(self._listar_puertos())
        h.addWidget(self.cmb_port, stretch=1)
        v.addLayout(h)

        self.btn_connect = QtWidgets.QPushButton("Conectar")
        self.btn_connect.clicked.connect(self._on_connect)
        v.addWidget(self.btn_connect)
        return g

    def _crear_grupo_seleccion_graficas(self):
        g = QtWidgets.QGroupBox("Gráficas visibles")
        h = QtWidgets.QHBoxLayout(g)
        h.setContentsMargins(6, 4, 6, 4)
        h.setSpacing(12)

        self.chk_overlay = QtWidgets.QCheckBox("Superponer observadores")
        self.chk_overlay.setToolTip(
            "Si está marcado, las gráficas de ω barra y ω disco muestran\n"
            "también la estimación del otro observador (trazo discontinuo)."
        )
        
        self.chk_overlay.setChecked(False)
        self.chk_overlay.toggled.connect(self._on_overlay_toggled)
        h.addWidget(self.chk_overlay)
        
        self.chk_graf = {}
        for clave, etiqueta in [
            ("pos",   "θ barra"),
            ("vel",   "ω barra"),
            ("disco", "ω disco"),
            ("u",     "u control"),
        ]:
            chk = QtWidgets.QCheckBox(etiqueta)
            chk.setChecked(True)
            chk.toggled.connect(
                lambda visible, k=clave: self._on_grafica_toggled(k, visible)
            )
            h.addWidget(chk)
            self.chk_graf[clave] = chk

        h.addStretch()
        return g

    def _on_overlay_toggled(self, activo: bool):
        self.panel_vel_barra.set_overlay(activo)
        self.panel_vel_disco.set_overlay(activo)
        self._plot_tick()   # redibujar inmediatamente
    
    def _on_grafica_toggled(self, clave, visible):
        """Muestra u oculta una gráfica. El layout recoloca el resto automáticamente."""
        panel = self._paneles_graficas[clave]
        panel.setVisible(visible)

    #Met para el refresh de coms
    def _refresh_puertos(self):
        if self.serial.connected:
            return                              # no tocar mientras hay sesión activa
        actuales = self._listar_puertos()
        existentes = [self.cmb_port.itemText(i) for i in range(self.cmb_port.count())]
        if actuales == existentes:
            return                              # nada cambió, no repintar

        sel = self.cmb_port.currentText()
        self.cmb_port.blockSignals(True)
        self.cmb_port.clear()
        self.cmb_port.addItems(actuales)
        idx = self.cmb_port.findText(sel)
        if idx >= 0:
            self.cmb_port.setCurrentIndex(idx)
        self.cmb_port.blockSignals(False)


    @staticmethod
    def _listar_puertos():
        if not SERIAL_AVAILABLE:
            return ["(pyserial no instalado)", "COM3 (demo)", "/dev/ttyUSB0 (demo)"]
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return ports if ports else ["(sin puertos)"]

    # ---- Grupo: modos de funcionamiento ----
    def _crear_grupo_modos(self):
        g = QtWidgets.QGroupBox("Modos de Funcionamiento")
        v = QtWidgets.QVBoxLayout(g)

        self.mode_group = QtWidgets.QButtonGroup(self)
        self.mode_group.setExclusive(True)
        self.mode_buttons = {}

        for clave, etiqueta in self.MODOS:
            btn = QtWidgets.QPushButton(etiqueta)
            btn.setObjectName("btnModo")
            btn.setCheckable(True)
            btn.setMinimumHeight(30)
            if clave == "MANUAL":
                btn.setToolTip("Mantén pulsada ← o → para mover el disco; al soltar, el motor se detiene.\n"
                               "Si la barra pasa por la vertical, el firmware pasa solo a Lazo Cerrado.")
            btn.clicked.connect(lambda _, k=clave: self._on_mode_changed(k))
            self.mode_group.addButton(btn)
            self.mode_buttons[clave] = btn
            v.addWidget(btn)

        self.mode_buttons["STANDBY"].setChecked(True)
        return g

    # ---- Grupo: lazo abierto ----
    def _crear_grupo_lazo_abierto(self):
        g = QtWidgets.QGroupBox("Lazo Abierto (Identificación)")
        v = QtWidgets.QVBoxLayout(g)

        # --- Aviso si no está en OpenLoop ---
        self.lbl_ol_aviso = QtWidgets.QLabel(
            "⚠ Debe activar el modo «Lazo Abierto» para habilitar excitaciones."
        )
        self.lbl_ol_aviso.setStyleSheet("color: #b00; font-size: 9pt;")
        self.lbl_ol_aviso.setWordWrap(True)
        v.addWidget(self.lbl_ol_aviso)

        # --- Selector de tipo de señal ---
        v.addWidget(QtWidgets.QLabel("Tipo de excitación:"))
        self.cmb_exc = NoWheelComboBox()
        self.cmb_exc.addItems([
            "Onda cuadrada",
            "Chirp logarítmico",
            "Secuencia PRBS",
        ])
        self.cmb_exc.currentIndexChanged.connect(self._on_exc_type_changed)
        v.addWidget(self.cmb_exc)

        # --- Parámetros de onda cuadrada ---
        self.grp_square = QtWidgets.QGroupBox("Parámetros · Onda cuadrada")
        f_sq = QtWidgets.QFormLayout(self.grp_square)
        self.sp_sq_freq = FlexibleDoubleSpinBox()
        self.sp_sq_freq.setRange(0.05, 20.0); self.sp_sq_freq.setValue(0.5)
        self.sp_sq_freq.setSuffix(" Hz")
        self.sp_sq_amp = FlexibleDoubleSpinBox()
        self.sp_sq_amp.setRange(1, 100); self.sp_sq_amp.setValue(10)
        self.sp_sq_amp.setSuffix(" %")
        self.sp_sq_T = FlexibleDoubleSpinBox()
        self.sp_sq_T.setRange(1, 300); self.sp_sq_T.setValue(20)
        self.sp_sq_T.setSuffix(" s")
        f_sq.addRow("Frecuencia:", self.sp_sq_freq)
        f_sq.addRow("Amplitud:",   self.sp_sq_amp)
        f_sq.addRow("Duración:",   self.sp_sq_T)
        v.addWidget(self.grp_square)

        # --- Parámetros de chirp ---
        self.grp_chirp = QtWidgets.QGroupBox("Parámetros · Chirp logarítmico")
        f_ch = QtWidgets.QFormLayout(self.grp_chirp)
        self.sp_ch_f0 = FlexibleDoubleSpinBox()
        self.sp_ch_f0.setRange(0.01, 10); self.sp_ch_f0.setValue(0.15)
        self.sp_ch_f0.setSuffix(" Hz")
        self.sp_ch_f1 = FlexibleDoubleSpinBox()
        self.sp_ch_f1.setRange(0.5, 25); self.sp_ch_f1.setValue(10.0)
        self.sp_ch_f1.setSuffix(" Hz")
        self.sp_ch_T = FlexibleDoubleSpinBox()
        self.sp_ch_T.setRange(5, 300); self.sp_ch_T.setValue(60)
        self.sp_ch_T.setSuffix(" s")
        self.sp_ch_amp = FlexibleDoubleSpinBox()
        self.sp_ch_amp.setRange(1, 100); self.sp_ch_amp.setValue(5)
        self.sp_ch_amp.setSuffix(" %")
        f_ch.addRow("f inicial:", self.sp_ch_f0)
        f_ch.addRow("f final:",   self.sp_ch_f1)
        f_ch.addRow("Duración:",  self.sp_ch_T)
        f_ch.addRow("Amplitud:",  self.sp_ch_amp)
        v.addWidget(self.grp_chirp)

        # --- Parámetros de PRBS ---
        self.grp_prbs = QtWidgets.QGroupBox("Parámetros · PRBS")
        f_pr = QtWidgets.QFormLayout(self.grp_prbs)
        self.sp_pr_br = FlexibleDoubleSpinBox()
        self.sp_pr_br.setRange(0.1, 10.0); self.sp_pr_br.setValue(2.0)
        self.sp_pr_br.setSuffix(" Hz")
        self.sp_pr_br.setToolTip("Frecuencia de cambio de bit.\n"
                                "Mantener ≤ 20 Hz para buena resolución a 200 Hz de muestreo.")
        self.sp_pr_amp = FlexibleDoubleSpinBox()
        self.sp_pr_amp.setRange(1, 100); self.sp_pr_amp.setValue(5)
        self.sp_pr_amp.setSuffix(" %")
        self.sp_pr_T = FlexibleDoubleSpinBox()
        self.sp_pr_T.setRange(1, 300); self.sp_pr_T.setValue(30)
        self.sp_pr_T.setSuffix(" s")
        f_pr.addRow("Bitrate:",  self.sp_pr_br)
        f_pr.addRow("Amplitud:", self.sp_pr_amp)
        f_pr.addRow("Duración:", self.sp_pr_T)
        v.addWidget(self.grp_prbs)

        # --- Botones ---
        h = QtWidgets.QHBoxLayout()
        self.btn_exc_start = QtWidgets.QPushButton("▶ Iniciar excitación")
        self.btn_exc_start.setStyleSheet(ESTILO_ENVIAR)
        self.btn_exc_start.clicked.connect(self._on_exc_start)
        self.btn_exc_stop = QtWidgets.QPushButton("■ Detener (u=0)")
        self.btn_exc_stop.clicked.connect(self._on_exc_stop)
        h.addWidget(self.btn_exc_start)
        h.addWidget(self.btn_exc_stop)
        v.addLayout(h)

        self._on_exc_type_changed(0)

        for sp in (self.sp_sq_freq, self.sp_sq_amp, self.sp_sq_T,
                self.sp_ch_f0, self.sp_ch_f1, self.sp_ch_T, self.sp_ch_amp,
                self.sp_pr_br, self.sp_pr_amp, self.sp_pr_T):
            sp.returnPressed.connect(self._on_exc_start)
    
        return g

    def _on_exc_type_changed(self, idx):
        self.grp_square.setVisible(idx == 0)
        self.grp_chirp.setVisible(idx == 1)
        self.grp_prbs.setVisible(idx == 2)

    def _set_controles_ol_habilitados(self, habilitado: bool):
        """Habilita los controles de excitación solo si estamos en OPEN_LOOP y conectados."""
        self.cmb_exc.setEnabled(habilitado)
        self.btn_exc_start.setEnabled(habilitado)
        self.btn_exc_stop.setEnabled(habilitado)
        self.grp_square.setEnabled(habilitado)
        self.grp_chirp.setEnabled(habilitado)
        self.grp_prbs.setEnabled(habilitado)
        self.lbl_ol_aviso.setVisible(not habilitado)

    def _on_exc_start(self):
        if self.mode != "OPEN_LOOP":
            QtWidgets.QMessageBox.warning(
                self, "Modo incorrecto",
                "Debe activar «Lazo Abierto» antes de iniciar una excitación."
            )
            return
        idx = self.cmb_exc.currentIndex()
        if idx == 0:  # square
            self.serial.enviar_excitacion("SQUARE", {
                "freq": self.sp_sq_freq.value(),
                "amp":  self.sp_sq_amp.value(),
                "T":    self.sp_sq_T.value(),
            })
        elif idx == 1:  # chirp
            self.serial.enviar_excitacion("CHIRP", {
                "f0":  self.sp_ch_f0.value(),
                "f1":  self.sp_ch_f1.value(),
                "T":   self.sp_ch_T.value(),
                "amp": self.sp_ch_amp.value(),
            })
        else:  # prbs
            self.serial.enviar_excitacion("PRBS", {
                "bitrate": self.sp_pr_br.value(),
                "amp":     self.sp_pr_amp.value(),
                "T":       self.sp_pr_T.value(),
            })

    def _on_exc_stop(self):
        self.serial.enviar_stop_excitacion()

    # ---- Grupo: parámetros de control (matriz K) ----
    def _crear_grupo_parametros(self):
        g = QtWidgets.QGroupBox("Parámetros de Control")
        v = QtWidgets.QVBoxLayout(g)

        v.addWidget(QtWidgets.QLabel("Vector L:"))

        fila_k = QtWidgets.QHBoxLayout()
        fila_k.setSpacing(8)
        self.inputs_L = {}                       # {"L1": QLineEdit, "L2": ..., "L3": ...}

        for key in ("L1", "L2", "L3"):
            columna = QtWidgets.QVBoxLayout()
            columna.setSpacing(2)

            lbl = QtWidgets.QLabel(key)
            lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet("font-size: 9pt;")

            le = QtWidgets.QLineEdit(f"{self.gains[key]:g}")
            le.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
            le.setMinimumWidth(60)
            le.setToolTip(f"Ganancia {key}")
            le.returnPressed.connect(self._on_send_gains)

            columna.addWidget(lbl)
            columna.addWidget(le)
            fila_k.addLayout(columna)
            self.inputs_L[key] = le

        v.addLayout(fila_k)

        self.btn_send_gains = QtWidgets.QPushButton("Enviar al ESP32")
        self.btn_send_gains.setObjectName("btnEnviar")
        self.btn_send_gains.setStyleSheet(ESTILO_ENVIAR)
        self.btn_send_gains.clicked.connect(self._on_send_gains)
        v.addWidget(self.btn_send_gains)

        self.lbl_gains_lock = QtWidgets.QLabel("")
        self.lbl_gains_lock.setStyleSheet("color: #b00; font-size: 9pt;")
        self.lbl_gains_lock.setWordWrap(True)
        v.addWidget(self.lbl_gains_lock)

        v.addWidget(QtWidgets.QLabel("Multiplicador WMAXI:"))
        self.sp_wmaxi = FlexibleDoubleSpinBox()
        self.sp_wmaxi.setRange(0.1, 20.0)
        self.sp_wmaxi.setValue(2.1)
        self.sp_wmaxi.setSingleStep(0.1)
        self.sp_wmaxi.setDecimals(3)
        self.sp_wmaxi.setToolTip(
            "Multiplicador de la energía objetivo del swing-up (2.1 por defecto).\n"
            "Mayor = swing-up más agresivo; menor = más suave.\n"
            "Se puede cambiar con el motor en marcha; se aplica en el próximo ciclo."
        )
        v.addWidget(self.sp_wmaxi)
        self.sp_wmaxi.returnPressed.connect(self._on_send_wmaxi)

        self.btn_send_wmaxi = QtWidgets.QPushButton("Enviar WMAXI")
        self.btn_send_wmaxi.setStyleSheet(ESTILO_ENVIAR)
        self.btn_send_wmaxi.clicked.connect(self._on_send_wmaxi)
        v.addWidget(self.btn_send_wmaxi)

        return g

    # ---- Grupo: observador ----
    def _crear_editor_4x2(self, titulo, keys, valores, prefijo_tip, placeholder, tip_paste, on_send, on_paste):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        v.addWidget(QtWidgets.QLabel(titulo))
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(4)
        for col, txt in ((1, "innov. barra"), (2, "innov. disco")):
            lbl = QtWidgets.QLabel(txt)
            lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet("font-size: 9pt;")
            grid.addWidget(lbl, 0, col)
        filas = ("pos barra", "vel barra", "pos disco", "vel disco")
        inputs = {}
        for idx, key in enumerate(keys):
            r, c = divmod(idx, 2)
            if c == 0:
                lbl = QtWidgets.QLabel(filas[r])
                lbl.setStyleSheet("font-size: 9pt;")
                grid.addWidget(lbl, r + 1, 0)
            le = QtWidgets.QLineEdit(f"{valores[key]:.6g}")
            le.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
            le.setMinimumWidth(60)
            le.setToolTip(f"{prefijo_tip}{key.upper()}: corrige «{filas[r]}» con la innovación de {'la barra' if c == 0 else 'el disco'}")
            le.returnPressed.connect(on_send)
            grid.addWidget(le, r + 1, c + 1)
            inputs[key] = le
        v.addLayout(grid)

        paste = QtWidgets.QLineEdit()
        paste.setPlaceholderText(placeholder)
        paste.setToolTip(tip_paste)
        paste.returnPressed.connect(on_paste)
        v.addWidget(paste)

        btn = QtWidgets.QPushButton("Enviar al ESP32")
        btn.setStyleSheet(ESTILO_ENVIAR)
        btn.clicked.connect(on_send)
        v.addWidget(btn)
        return w, inputs, paste, btn

    def _crear_grupo_observador(self):
        g = QtWidgets.QGroupBox("Observador")
        v = QtWidgets.QVBoxLayout(g)

        v.addWidget(QtWidgets.QLabel("Modo de estimación:"))
        h_est = QtWidgets.QHBoxLayout()
        self.cmb_est = NoWheelComboBox()
        self.cmb_est.addItems(["DIF", "OBS", "KAL"])
        h_est.addWidget(self.cmb_est, stretch=1)
        self.btn_apply_est = QtWidgets.QPushButton("Aplicar estimador")
        self.btn_apply_est.setStyleSheet(ESTILO_ENVIAR)
        self.btn_apply_est.clicked.connect(self._on_apply_est)
        h_est.addWidget(self.btn_apply_est)
        v.addLayout(h_est)

        self.lbl_dif_info = QtWidgets.QLabel("DIF (diferencias finitas) no requiere ganancias.")
        self.lbl_dif_info.setWordWrap(True)
        self.lbl_dif_info.setStyleSheet("font-size: 9pt; font-style: italic;")
        v.addWidget(self.lbl_dif_info)

        self.w_obs, self.inputs_obs, self.le_obs_paste, self.btn_send_obs = self._crear_editor_4x2(
            "Matriz K_d del observador (4x2):", OBS_KEYS, self.obs_gains, "",
            "Pegar 8 valores por filas (K11,K12,K21,...) y Enter",
            "MATLAB:  fprintf('%.8g,', K_d.')",
            self._on_send_obs, self._on_obs_pegar)
        v.addWidget(self.w_obs)

        self.w_kal, self.inputs_kal, self.le_kal_paste, self.btn_send_kal = self._crear_editor_4x2(
            "Matriz L_k del Kalman (4x2):", KAL_KEYS, self.kal_gains, "Kalman ",
            "Pegar 8 valores por filas (L11,L12,L21,...) y Enter",
            "MATLAB:  fprintf('%.8g,', L_k.')",
            self._on_send_kal, self._on_kal_pegar)
        v.addWidget(self.w_kal)

        self.cmb_est.currentTextChanged.connect(self._actualizar_panel_ganancias_est)
        self._actualizar_panel_ganancias_est()
        return g

    def _actualizar_panel_ganancias_est(self, _txt=None):
        sel = self.cmb_est.currentText()
        self.lbl_dif_info.setVisible(sel == "DIF")
        self.w_obs.setVisible(sel == "OBS")
        self.w_kal.setVisible(sel == "KAL")

    def _crear_grupo_apariencia(self):
        g = QtWidgets.QGroupBox("Apariencia")
        h = QtWidgets.QHBoxLayout(g)
        h.setContentsMargins(8, 4, 8, 4)
        h.setSpacing(8)

        h.addWidget(QtWidgets.QLabel("Modo Batman"))
        h.addStretch()

        self.toggle_batman = ToggleSwitch()
        self.toggle_batman.setFocusPolicy(QtCore.Qt.FocusPolicy.NoFocus)
        self.toggle_batman.setToolTip(
            "Activa la paleta oscura «Modo Batman».\n"
            "Desactivalo para volver al modo claro."
        )
        self.toggle_batman.toggled.connect(self._on_batman_toggled)
        h.addWidget(self.toggle_batman)
        return g

    # ---- Grupo: grabación ----
    def _crear_grupo_grabacion(self):
        g = QtWidgets.QGroupBox("Grabación y Exportación")
        v = QtWidgets.QVBoxLayout(g)

        self.btn_record = QtWidgets.QPushButton("● Iniciar Grabación")
        self.btn_record.setMinimumHeight(34)
        self.btn_record.setStyleSheet(ESTILO_GRABAR)
        self.btn_record.clicked.connect(self._on_toggle_record)
        v.addWidget(self.btn_record)

        self.lbl_record = QtWidgets.QLabel("Muestras grabadas: 0")
        v.addWidget(self.lbl_record)
        return g

    # ---- Grupo: seguridad ----
    def _crear_grupo_seguridad(self):
        g = QtWidgets.QGroupBox("Seguridad")
        v = QtWidgets.QVBoxLayout(g)

        self.btn_kill = QtWidgets.QPushButton("BATISEÑAL")
        self.btn_kill.setStyleSheet(ESTILO_KILL)
        self.btn_kill.setToolTip("Apaga el puente H inmediatamente (comando prioritario por UART). Atajo: Esc")
        self.btn_kill.clicked.connect(self._on_kill_switch)
        v.addWidget(self.btn_kill)
        return g

    # -----------------------------------------------------------------
    #  Helpers de estado
    # -----------------------------------------------------------------
    def _update_gain_button_state(self):
        puede = (not self.motor_moving) and self.serial.handshake_completo
        self.btn_send_gains.setEnabled(puede)
        self.btn_zero.setEnabled(puede)               # <-- NUEVO
        for le in self.inputs_L.values():
            le.setEnabled(puede)

        # WMAXI: se puede ajustar con el motor en marcha si hay handshake
        puede_wmaxi = self.serial.handshake_completo
        self.sp_wmaxi.setEnabled(puede_wmaxi)
        self.btn_send_wmaxi.setEnabled(puede_wmaxi)

        # Observador
        self.btn_send_obs.setEnabled(puede)
        for le in self.inputs_obs.values():
            le.setEnabled(puede)
        self.le_obs_paste.setEnabled(puede)

        # Kalman
        self.btn_send_kal.setEnabled(puede)
        for le in self.inputs_kal.values(): le.setEnabled(puede)
        self.le_kal_paste.setEnabled(puede)

        puede_est = self.serial.handshake_completo
        self.cmb_est.setEnabled(puede_est)
        self.btn_apply_est.setEnabled(puede_est)

        if puede:
            self.lbl_gains_lock.setText("")
        elif not self.serial.handshake_completo:
            self.lbl_gains_lock.setText("⚠ Sin conexión: handshake no establecido.")
        else:
            self.lbl_gains_lock.setText(
                "⚠ Envío de ganancias bloqueado. ")

    def _set_motor_moving(self, moving: bool):
        if self.motor_moving != moving:
            self.motor_moving = moving
            self._update_gain_button_state()

    def _update_status_bar(self):
        estado = "conectado" if self.serial.connected else "desconectado"
        hs = (f"handshake OK (fw {self.serial.firmware_version})"
              if self.serial.handshake_completo else "sin handshake")
        self.statusBar().showMessage(
            f"Estado: {estado} | {hs}  |  Modo: {self.mode}  |  "
            f"Muestras: {len(self.t_buf)}  |  f_s = {FS_HZ} Hz"
        )

    def _leer_ganancias_ui(self) -> dict:
        """Lee los 3 campos L1/L2/L3. Lanza ValueError con mensaje claro si alguno falla."""
        valores = {}
        for key, le in self.inputs_L.items():
            texto = le.text().strip().replace(",", ".")   # tolera coma decimal
            try:
                val = float(texto)
            except ValueError:
                raise ValueError(f"{key}: '{le.text()}' no es un número válido.")
            if not math.isfinite(val) or abs(val) > 1e6:
                raise ValueError(f"{key}: el valor debe ser finito y estar entre -1e6 y 1e6.")
            valores[key] = val
        return valores

    def _set_controles_conectados(self, conectado: bool):
        """Habilita o bloquea los controles que requieren sesión activa con el ESP32."""
        for btn in self.mode_buttons.values():
            btn.setEnabled(conectado)
        self.btn_record.setEnabled(conectado)
        self.btn_kill.setEnabled(self.serial.connected)   # siempre disponible con el puerto abierto
        self._set_controles_ol_habilitados(conectado and self.mode == "OPEN_LOOP")
        self._update_gain_button_state()

    def _forzar_standby(self):
        """Pone la UI y el firmware en STANDBY. Se usa cuando se pierde la conexión."""
        if self.mode != "STANDBY":
            if self.mode == "OPEN_LOOP" and self.serial.handshake_completo:
                self.serial.enviar_stop_excitacion()
            # El modo cambia sin escribir a UART si ya no hay handshake
            self.mode_buttons["STANDBY"].setChecked(True)
            self._on_mode_changed("STANDBY")

    def _detener_grabacion(self) -> bool:
        """Detiene la grabación si estaba activa. Devuelve True si efectivamente la detuvo.
        No exporta: eso lo decide el caller según el contexto."""
        if not self.recording:
            return False
        self.recording = False
        self.btn_record.setText("● Iniciar Grabación")
        print(f"[UI] Grabación detenida (auto). Total: {len(self.recorded_data)} muestras")
        return True

    def _ofrecer_exportar_tras_corte(self):
        """Pregunta al usuario si quiere guardar lo grabado antes del corte."""
        if not self.recorded_data:
            return
        resp = QtWidgets.QMessageBox.question(
            self, "Grabación interrumpida",
            f"Se cortó la conexión con el ESP32 mientras grababas.\n\n"
            f"Se alcanzaron a registrar {len(self.recorded_data)} muestras.\n"
            "¿Querés guardarlas en un CSV?",
            QtWidgets.QMessageBox.StandardButton.Yes
            | QtWidgets.QMessageBox.StandardButton.No,
            QtWidgets.QMessageBox.StandardButton.Yes,
        )
        if resp == QtWidgets.QMessageBox.StandardButton.Yes:
            self._exportar_csv()

    # -----------------------------------------------------------------
    #  Control manual (botones + flechas del teclado)
    # -----------------------------------------------------------------
    def _manual_set(self, direccion: str, activo: bool):
        if activo:
            self._manual_held.add(direccion)
        else:
            self._manual_held.discard(direccion)
        self._manual_actualizar()

    def _manual_actualizar(self):
        if self._manual_held == {"LEFT"}:
            cmd = "LEFT"
        elif self._manual_held == {"RIGHT"}:
            cmd = "RIGHT"
        else:
            cmd = "STOP"
        if cmd != self._manual_last:
            self._manual_last = cmd
            self.serial.enviar_manual(cmd)
        # Keep-alive: el firmware corta el modo manual si la orden no se refresca
        # (deadman), así un comando perdido nunca deja el motor andando.
        if cmd == "STOP":
            self._manual_timer.stop()
        elif not self._manual_timer.isActive():
            self._manual_timer.start(MANUAL_KEEPALIVE_MS)

    def _manual_keepalive(self):
        if (self._manual_last != "STOP" and self.mode == "MANUAL"
                and self.serial.handshake_completo):
            self.serial.enviar_manual(self._manual_last)

    def _liberar_manual(self):
        self._manual_held.clear()
        self._manual_actualizar()

    def eventFilter(self, obj, event):
        """Convierte las flechas ← → en órdenes manuales (solo en modo MANUAL)."""
        if self.mode != "MANUAL":          # filtro global: salir rápido en el 99 % de los eventos
            return False
        tipo = event.type()
        if (tipo in (QtCore.QEvent.Type.KeyPress, QtCore.QEvent.Type.KeyRelease)
                and self.isActiveWindow()
                and event.key() in self.TECLAS_MANUAL):
            foco = QtWidgets.QApplication.focusWidget()
            if not isinstance(foco, (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox,
                                     QtWidgets.QComboBox)):
                if not event.isAutoRepeat():
                    self._manual_set(self.TECLAS_MANUAL[event.key()],
                                    tipo == QtCore.QEvent.Type.KeyPress)
                return True
        return super().eventFilter(obj, event)

    def showEvent(self, event):
        super().showEvent(event)
        # Reaplicar el tema del título tras el primer show: DWM a veces
        # resetea el atributo al mostrar la ventana por primera vez.
        self._aplicar_titulo_ventana_so(self.toggle_batman.isChecked())
    
    def changeEvent(self, event):
        # Seguridad: si la ventana pierde el foco con una tecla sostenida, detener.
        if event.type() == QtCore.QEvent.Type.ActivationChange and not self.isActiveWindow():
            self._liberar_manual()
        super().changeEvent(event)

    def closeEvent(self, event):
        """Al cerrar la ventana: parar el motor y cerrar el puerto limpiamente."""
        try:
            if self.serial.connected:
                self.serial.enviar_estop()
                self.serial.desconectar()
        finally:
            super().closeEvent(event)

    # -----------------------------------------------------------------
    #  Callbacks de la UI
    # -----------------------------------------------------------------
    def _on_connect(self):
        if self.serial.connected:
            self._hs_timer.stop()
            self._sincronizado = False
            self._forzar_standby()
            self.serial.desconectar()
            detenida = self._detener_grabacion() 
            self._set_controles_conectados(False)
            self.btn_connect.setText("Conectar")
            self._update_status_bar()
            if detenida:
                self._ofrecer_exportar_tras_corte()
            return

        puerto = self.cmb_port.currentText()
        if not puerto or puerto.startswith("("):
            QtWidgets.QMessageBox.warning(self, "Sin puerto",
                                          "No hay un puerto serie válido seleccionado.")
            return
        if self.serial.conectar(puerto):
            self._reset_datos_sesion()
            self.btn_connect.setText("Desconectar")
            self._set_controles_conectados(False)      # nada funciona hasta handshake
            self._hs_timer.start(timeout_handshake)
        else:
            QtWidgets.QMessageBox.critical(
                self, "Error de conexión",
                "No se pudo abrir el puerto serie. Verifique que el puerto sea "
                "correcto y que no esté siendo usado por otro programa."
            )
        self._update_status_bar()

    @QtCore.pyqtSlot(str)
    def _on_serial_desconectado(self, motivo: str):
        """Se dispara cuando el hilo de lectura muere por su cuenta (cable
        desconectado, driver, puerto cerrado por otro proceso, etc.)."""
        print(f"[UI] UART caído: {motivo}")
        self._hs_timer.stop()
        self._sincronizado = False

        # Cerrar limpio. Como el hilo ya salió del while, el join() interno
        # de desconectar() retorna de inmediato (no bloquea la GUI).
        if self.serial.connected:
            self.serial.desconectar()

        self._forzar_standby()
        detenida = self._detener_grabacion()
        self._set_controles_conectados(False)
        self.btn_connect.setText("Conectar")
        self._update_status_bar()

        if detenida and self.recorded_data:
            resp = QtWidgets.QMessageBox.question(
                self, "Conexión perdida",
                f"Se perdió la comunicación con el ESP32:\n{motivo}\n\n"
                f"Se cortó la grabación con {len(self.recorded_data)} muestras.\n"
                "¿Querés guardarlas antes de reconectar?",
                QtWidgets.QMessageBox.StandardButton.Yes
                | QtWidgets.QMessageBox.StandardButton.No,
                QtWidgets.QMessageBox.StandardButton.Yes,
            )
            if resp == QtWidgets.QMessageBox.StandardButton.Yes:
                self._exportar_csv()
        else:
            QtWidgets.QMessageBox.warning(
                self, "Conexión perdida",
                f"Se perdió la comunicación con el ESP32:\n{motivo}\n\n"
                "Revise el cable y vuelva a conectar."
            )

    def _check_handshake_timeout(self):
        if self.serial.connected and not self.serial.handshake_completo:
            QtWidgets.QMessageBox.warning(
                self, "Sin respuesta",
                "El puerto se abrió pero el ESP32 no respondió al handshake "
                f"en {timeout_handshake/1000:.0f} s.\n"
                "Verifique que el firmware tenga el protocolo HELLO/READY/STATE."
            )

    @QtCore.pyqtSlot(dict)
    def _on_handshake(self, info):
        print(f"[UI] Handshake OK con firmware {info['firmware']}")
        self._hs_timer.stop()

        self.gains = {'L1': info['L1'], 'L2': info['L2'], 'L3': info['L3']}
        primer_state = not self._sincronizado
        modo_ui_antes = self.mode
        if not self._sincronizado:
            # Solo en el primer STATE de la sesión: los siguientes (uno por cada
            # cambio de modo) no deben pisar lo que el usuario esté escribiendo.
            if info.get('wmaxi') is not None:
                self.sp_wmaxi.setValue(float(info['wmaxi']))
            for key, le in self.inputs_L.items():
                le.setText(f"{self.gains[key]:g}")
            if info.get('EST') is not None:
                idx = self.cmb_est.findText(str(info['EST']))
                if idx >= 0:
                    self.cmb_est.setCurrentIndex(idx)
            if info.get('OBS') is not None:
                obs = info['OBS']
                if isinstance(obs, dict):
                    self.obs_gains = obs
                    for k in OBS_KEYS:
                        if k in obs:
                            self.inputs_obs[k].setText(f"{obs[k]:.6g}")
            if info.get('KAL') is not None:
                kal = info['KAL']
                if isinstance(kal, dict):
                    self.kal_gains = kal
                    for k in KAL_KEYS:
                        if k in kal: self.inputs_kal[k].setText(f"{kal[k]:.6g}")
            self._sincronizado = True

        self._set_controles_conectados(True)

        # Sincronizar la UI con el modo que reporta el firmware
        modo_fw = info.get('mode', 'STANDBY')
        if modo_fw in self.mode_buttons:
            if self.mode == "MANUAL" and modo_fw!="MANUAL":
                self._liberar_manual();
            self.mode_buttons[modo_fw].setChecked(True)
            self.mode = modo_fw
            self._set_controles_ol_habilitados(modo_fw == "OPEN_LOOP")
            self._set_motor_moving(modo_fw != "STANDBY")
        

        if (not primer_state) and modo_fw != modo_ui_antes:
            # Cambio hecho por el firmware (barra capturada en manual / barra caída)
            self._mostrar_evento(
                f"⚠ El firmware cambió el modo: {modo_ui_antes} → {modo_fw}",
                ms=8000, color="#e65100"
            )
        else:
            self._mostrar_evento(
                f"Conectado a {info['firmware']} | {modo_fw} | "
                f"L=({info['L1']:.4f}, {info['L2']:.4f}, {info['L3']:.4f})",
                ms=8000, color="#2e7d32"
            )
    
    @QtCore.pyqtSlot()
    def _on_handshake_perdido(self):
        print("[UI] Handshake perdido — bloqueando controles")
        self._sincronizado = False
        self._forzar_standby()
        detenida = self._detener_grabacion()
        self._set_controles_conectados(False)
        self._mostrar_evento("⚠ Handshake perdido con el ESP32", ms=10000)
        if detenida:
            self._ofrecer_exportar_tras_corte()

    def _reset_datos_sesion(self):
        """Limpia el histórico de las gráficas y reinicia la base de tiempo local."""
        self.t = 0.0
        self.t_buf.clear()
        self.pos_buf.clear()
        self.vel_buf.clear()
        self.disco_buf.clear()
        self.u_buf.clear()
        self.vel_other_buf.clear()
        self.disco_other_buf.clear()
        self.panel_fase.limpiar()
        self.panel_esquema.limpiar() 
        for panel in self._paneles_graficas.values():     # incluye la gráfica de u
            panel.curve.setData([], [])
            if panel.curve_other is not None:
                panel.curve_other.setData([], [])
            vb = panel.plot.getViewBox()
            vb.setXRange(0, 1, padding=0.0)
            vb.setYRange(-1, 1, padding=0.0)
        self._status_div = 0

    def _on_mode_changed(self, modo, forzar=False):
        if modo != "STANDBY" and not self.serial.handshake_completo:
            self.mode_buttons["STANDBY"].setChecked(True)
            return
        if modo == self.mode and not forzar:
            # Clic sobre el modo ya activo: no reenviar MODE (el firmware reinicia
            # estimador y fase AUTO con cada MODE; un doble clic tiraría el péndulo).
            return

        self._liberar_manual()
        self.mode = modo
        print(f"[UI] Cambio de modo -> {modo}")
        self.serial.enviar_modo(modo)

        self._set_controles_ol_habilitados(
            modo == "OPEN_LOOP" and self.serial.handshake_completo
        )

        self._set_motor_moving(modo != "STANDBY")
        self._update_status_bar()

    def _send_ping(self):
        if self.serial.connected:
            self.serial.enviar_ping()

    def _on_send_gains(self):
        if not self.serial.handshake_completo:
            QtWidgets.QMessageBox.warning(
                self, "Sin conexión",
                "No hay handshake establecido con el ESP32."
            )
            return
        if self.motor_moving:
            QtWidgets.QMessageBox.warning(
                self, "Bloqueado",
                "No se pueden enviar ganancias mientras el motor esté en movimiento."
            )
            return

        try:
            nuevas = self._leer_ganancias_ui()
        except ValueError as e:
            QtWidgets.QMessageBox.critical(
                self, "Error",
                f"Valor de ganancia inválido.\n\n{e}"
            )
            return

        # Enviar y esperar confirmación
        self._ack_pendiente = True
        self.btn_send_gains.setEnabled(False)
        for le in self.inputs_L.values():
            le.setEnabled(False)
        self.lbl_gains_lock.setText("⏳ Esperando confirmación del ESP32...")

        self.serial.enviar_ganancias(nuevas)
        self._ack_timer.start(timeout_ack)

    def _on_zero(self):
        if not self.serial.handshake_completo or self.motor_moving:
            return
        
        self.serial.enviar_zero()

    def _on_send_wmaxi(self):
        if not self.serial.handshake_completo:
            QtWidgets.QMessageBox.warning(
                self, "Sin conexión",
                "No hay handshake establecido con el ESP32."
            )
            return
        val = self.sp_wmaxi.value()
        self.serial.enviar_wmaxi(val)

    @QtCore.pyqtSlot(bool, float, str)
    def _on_wmaxi_ack(self, ok: bool, val: float, motivo: str):
        if ok:
            self.sp_wmaxi.blockSignals(True)
            self.sp_wmaxi.setValue(val)
            self.sp_wmaxi.blockSignals(False)
            self._mostrar_evento(f"WMAXI confirmado: {val:.3f}", ms=6000, color="#2e7d32")
        else:
            self._mostrar_evento(f"⚠ WMAXI rechazado: {motivo or 'BAD_VALUE'}",
                                 ms=8000, color="#b00")

    def _on_apply_est(self):
        if not self.serial.handshake_completo:
            QtWidgets.QMessageBox.warning(
                self, "Sin conexión",
                "No hay handshake establecido con el ESP32."
            )
            return
        modo_est = self.cmb_est.currentText()
        self.serial.enviar_est(modo_est)

    def _leer_obs_ui(self) -> dict:
        """Lee los campos de K. Lanza ValueError con mensaje claro si alguno falla."""
        valores = {}
        for key in OBS_KEYS:
            le = self.inputs_obs[key]
            texto = le.text().strip().replace(",", ".")
            try:
                val = float(texto)
            except ValueError:
                raise ValueError(f"{key}: '{le.text()}' no es un número válido.")
            if not math.isfinite(val) or abs(val)>1e4:
                raise ValueError(f"{key.upper()}: el valor debe ser finito y |valor|<1e4.")
            valores[key] = val
        return valores

    def _on_obs_pegar(self):
        """Reparte 8 números separados por coma/espacio/; en la grilla (orden por filas)."""
        partes = [p for p in re.split(r"[,;\s]+", self.le_obs_paste.text().strip()) if p]
        try:
            vals = [float(p) for p in partes]
        except ValueError:
            vals = []
        if len(vals) != len(OBS_KEYS):
            QtWidgets.QMessageBox.warning(
                self, "Formato",
                f"Se necesitan exactamente {len(OBS_KEYS)} números (K11,K12,K21,K22,K31,K32,K41,K42).")
            return
        for k, val in zip(OBS_KEYS, vals):
            self.inputs_obs[k].setText(f"{val:.8g}")
        self.le_obs_paste.clear()


    def _on_send_obs(self):
        if not self.serial.handshake_completo:
            QtWidgets.QMessageBox.warning(
                self, "Sin conexión",
                "No hay handshake establecido con el ESP32."
            )
            return
        if self.motor_moving:
            QtWidgets.QMessageBox.warning(
                self, "Bloqueado",
                "No se pueden enviar ganancias del observador mientras el motor esté en movimiento."
            )
            return

        try:
            nuevas = self._leer_obs_ui()
        except ValueError as e:
            QtWidgets.QMessageBox.critical(
                self, "Error",
                f"Valor de ganancia del observador inválido.\n\n{e}"
            )
            return

        self.serial.enviar_obs(nuevas)

    @QtCore.pyqtSlot(bool, str)
    def _on_est_ack(self, ok: bool, val: str):
        if ok:
            idx = self.cmb_est.findText(val)
            if idx >= 0:
                self.cmb_est.setCurrentIndex(idx)
            self._mostrar_evento(f"Estimador aplicado: {val}", ms=6000, color="#2e7d32")
        else:
            self._mostrar_evento(f"⚠ Estimador rechazado: {val}", ms=8000, color="#b00")

    @QtCore.pyqtSlot(bool, dict, str)
    def _on_obs_ack(self, ok: bool, obs: dict, motivo: str):
        if ok:
            if all(k in obs for k in OBS_KEYS):
                self.obs_gains = obs
                for k in OBS_KEYS:
                    self.inputs_obs[k].setText(f"{obs[k]:.6g}")
            self._mostrar_evento("Observador K_d confirmado por el ESP32", ms=6000, color="#2e7d32")
        else:
            for k in OBS_KEYS:
                self.inputs_obs[k].setText(f"{self.obs_gains[k]:.6g}")
            self._mostrar_evento(
                f"⚠ Observador rechazado por el ESP32: {motivo or 'OBS_UNSTABLE'}",
                ms=8000, color="#b00")
    
    def _leer_kal_ui(self) -> dict:
        valores = {}
        for key in KAL_KEYS:
            le = self.inputs_kal[key]
            texto = le.text().strip().replace(",", ".")
            try: val = float(texto)
            except ValueError: raise ValueError(f"{key}: '{le.text()}' no es un número válido.")
            if not math.isfinite(val) or abs(val) > 1e4: raise ValueError(f"{key.upper()}: debe ser finito y |valor|<1e4.")
            valores[key] = val
        return valores

    def _on_kal_pegar(self):
        partes = [p for p in re.split(r"[,;\s]+", self.le_kal_paste.text().strip()) if p]
        try: vals = [float(p) for p in partes]
        except ValueError: vals = []
        if len(vals) != len(KAL_KEYS):
            QtWidgets.QMessageBox.warning(self, "Formato", f"Se necesitan exactamente {len(KAL_KEYS)} números.")
            return
        for k, val in zip(KAL_KEYS, vals): self.inputs_kal[k].setText(f"{val:.8g}")
        self.le_kal_paste.clear()

    def _on_send_kal(self):
        if not self.serial.handshake_completo or self.motor_moving: return
        try: nuevas = self._leer_kal_ui()
        except ValueError as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Valor inválido.\n\n{e}")
            return
        self.serial.enviar_kal(nuevas)

    @QtCore.pyqtSlot(bool, dict, str)
    def _on_kal_ack(self, ok: bool, kal: dict, motivo: str):
        if ok:
            if all(k in kal for k in KAL_KEYS):
                self.kal_gains = kal
                for k in KAL_KEYS: self.inputs_kal[k].setText(f"{kal[k]:.6g}")
            self._mostrar_evento("Kalman Ka confirmado", ms=6000, color="#2e7d32")
        else:
            for k in KAL_KEYS: self.inputs_kal[k].setText(f"{self.kal_gains[k]:.6g}")
            self._mostrar_evento(f"⚠ Kalman rechazado: {motivo}", ms=8000, color="#b00")

    @QtCore.pyqtSlot(bool, str, dict)
    def _on_ganancias_ack(self, ok: bool, motivo: str, gains: dict):
        if not self._ack_pendiente:
            # NACK sin ganancias pendientes (p. ej. una excitación rechazada)
            if not ok and motivo:
                self._mostrar_evento(f"⚠ El ESP32 rechazó el comando: {motivo}", ms=8000)
            return
        self._ack_pendiente = False
        self._ack_timer.stop()

        if ok and all(k in gains for k in ("L1", "L2", "L3")):
            self.gains = gains              # valores confirmados por el firmware
            self._restaurar_campos_ganancias()
            QtWidgets.QMessageBox.information(
                self, "Confirmado",
                f"El ESP32 confirmó las ganancias:\n"
                f"L = ({gains['L1']:.4f}, {gains['L2']:.4f}, {gains['L3']:.4f})"
            )
        else:
            self._restaurar_campos_ganancias()   # volver a las últimas confirmadas
            QtWidgets.QMessageBox.critical(
                self, "Rechazado por el ESP32",
                f"El ESP32 no aceptó las ganancias.\n\nMotivo: {motivo or 'respuesta incompleta'}"
            )

        self._update_gain_button_state()

    def _restaurar_campos_ganancias(self):
        """Deja en los campos las últimas ganancias confirmadas por el firmware."""
        for key, le in self.inputs_L.items():
            le.setText(f"{self.gains[key]:g}")

    def _on_ack_timeout(self):
        if not self._ack_pendiente:
            return
        self._ack_pendiente = False
        self._restaurar_campos_ganancias()
        QtWidgets.QMessageBox.warning(
            self, "Sin respuesta",
            f"El ESP32 no confirmó las ganancias en {timeout_ack/1000:.1f} s.\n"
            "Se restauraron las últimas ganancias confirmadas.\n"
            "Verifique el cable o el estado del firmware."
        )
        self._update_gain_button_state()

    @QtCore.pyqtSlot(bool, str)
    def _on_respuesta_exc(self, ok: bool, texto: str):
        self._mostrar_evento(("✔ " if ok else "⚠ ") + texto, ms=6000,
                             color="#2e7d32" if ok else "#b00")

    def _mostrar_evento(self, texto: str, ms: int = 6000, color: str = "#b00"):
        self.lbl_evento.setStyleSheet(f"color: {color}; padding-right: 8px;")
        self.lbl_evento.setText(texto)
        self._evento_timer.start(ms)

    def _on_kill_switch(self):
        print("[UI] ¡KILL SWITCH PRESIONADO!")
        self.serial.enviar_estop()
        self.mode_buttons["STANDBY"].setChecked(True)
        self._on_mode_changed("STANDBY")
        self._set_motor_moving(False)

    def _aplicar_titulo_ventana_so(self, dark: bool):
            """Pinta la barra de título del SO en modo oscuro/claro.
            Sólo Windows 10 1809+ / Windows 11. No-op en otras plataformas."""
            if sys.platform != "win32" or wintypes is None:
                return
            try:
                hwnd   = int(self.winId())
                dwmapi = ctypes.windll.dwmapi
                user32 = ctypes.windll.user32
            except (AttributeError, OSError):
                return
    
            valor = ctypes.c_int(1 if dark else 0)
            aplicado = False
            for attr in (20, 19):                  # 20 = W10 20H1+ / W11 ; 19 = W10 1809–1909
                try:
                    hr = dwmapi.DwmSetWindowAttribute(
                        wintypes.HWND(hwnd),
                        ctypes.c_uint(attr),
                        ctypes.byref(valor),
                        ctypes.sizeof(valor),
                    )
                    if hr == 0:                    # S_OK
                        aplicado = True
                        break
                except OSError:
                    continue
    
            if aplicado:
                # Forzar repintado del marco no-cliente: sin esto a veces el color
                # no se ve hasta que el usuario interactúa con la ventana.
                SWP_NOMOVE       = 0x0002
                SWP_NOSIZE       = 0x0001
                SWP_NOZORDER     = 0x0004
                SWP_FRAMECHANGED = 0x0020
                try:
                    user32.SetWindowPos(
                        wintypes.HWND(hwnd), None, 0, 0, 0, 0,
                        SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_FRAMECHANGED
                    )
                except OSError:
                    pass
    
    def _on_batman_toggled(self, activo: bool):
        """Cambia toda la paleta de colores (claro <-> Batman)."""
        # 1) Gráficas (fondo, ejes, títulos y color de traza)
        for panel in self._paneles_graficas.values():
            panel.set_theme(activo)
        self.panel_fase.set_theme(activo)
        self.panel_esquema.set_theme(activo) 

        # 2) Hoja de estilos global del QMainWindow
        self.setStyleSheet(ESTILO_GLOBAL_BATMAN if activo else ESTILO_GLOBAL_CLARO)

        # 3) Botones "Enviar" (usan hoja propia, no la global)
        estilo_enviar = ESTILO_ENVIAR_BATMAN if activo else ESTILO_ENVIAR_CLARO
        for btn in (self.btn_exc_start, self.btn_send_gains,
                    self.btn_send_wmaxi, self.btn_send_obs, self.btn_send_kal, self.btn_apply_est):
            btn.setStyleSheet(estilo_enviar)

        # 4) Avisos rojo-oscuro: sobre fondo negro casi no se leen
        aviso = "#ff5252" if activo else "#b00"
        self.lbl_ol_aviso.setStyleSheet(f"color: {aviso}; font-size: 9pt;")
        self.lbl_gains_lock.setStyleSheet(f"color: {aviso}; font-size: 9pt;")

        # 5) Config global de pyqtgraph (por si se crean más plots en el futuro)
        pg.setConfigOptions(
            background='#121212' if activo else 'w',
            foreground='#e0e0e0' if activo else 'k',
        )

        self._aplicar_titulo_ventana_so(activo)

    def _on_toggle_record(self):
        if not self.recording:
            self.recording = True
            self.recorded_data = []
            self.btn_record.setText("■ Detener y Exportar CSV")
            self.lbl_record.setText("Muestras grabadas: 0")
            print("[UI] Grabación iniciada")
        else:
            self.recording = False
            self.btn_record.setText("● Iniciar Grabación")
            print(f"[UI] Grabación detenida. Total: {len(self.recorded_data)} muestras")
            self._exportar_csv()

    def _exportar_csv(self):
        if not self.recorded_data:
            QtWidgets.QMessageBox.information(self, "Sin datos",
                                              "No hay muestras para exportar.")
            return
        fname, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Guardar registro", "registro_laboratorio.csv",
            "Archivos CSV (*.csv)"
        )
        if not fname:
            return
        try:
            with open(fname, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["t_s", "theta_rad", "omega_rad_s", "v_disco_rad_s", "u_ctrl", "omega_other_rad_s", "v_disco_other_rad_s"])
                w.writerows(self.recorded_data)
            QtWidgets.QMessageBox.information(
                self, "Exportado",
                f"Se guardaron {len(self.recorded_data)} muestras en:\n{fname}"
            )
        except OSError as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"No se pudo guardar:\n{e}")

    # -----------------------------------------------------------------
    #  Telemetría y refresco
    # -----------------------------------------------------------------
    @QtCore.pyqtSlot(float, float, float, float, float, float, float)
    def _on_muestra(self, t, theta, omega, v_disco, u, omega_other, v_disco_other):
        """Punto único de entrada de datos."""
        self.t_buf.append(t)
        self.pos_buf.append(theta)
        self.vel_buf.append(omega)
        self.disco_buf.append(v_disco)
        self.u_buf.append(u)
        self.vel_other_buf.append(omega_other)
        self.disco_other_buf.append(v_disco_other)

        if self.recording:
            self.recorded_data.append((t, theta, omega, v_disco, u, omega_other, v_disco_other))

    def _plot_tick(self):
        """Refresco visual desacoplado de la adquisición para mantener la UI fluida."""
        if not self.t_buf:
            return

        # Si las tres curvas están pausadas, ni siquiera construimos arrays NumPy.
        # Asociación panel ↔ buffer

        paneles = [
            (self.panel_pos_barra,  self.pos_buf, None),
            (self.panel_vel_barra,  self.vel_buf, self.vel_other_buf),
            (self.panel_vel_disco,  self.disco_buf, self.disco_other_buf),
            (self.panel_u,          self.u_buf, None),
        ]

        # Solo construimos el array de tiempo si hay al menos un panel activo
        hay_activo = any(p.isVisible() and not p.paused for p, _ , _ in paneles)
        if hay_activo:
            tx = np.fromiter(self.t_buf, dtype=np.float64, count=len(self.t_buf))
            overlay = self.chk_overlay.isChecked()
            for panel, buf, buf_other in paneles:
                if not panel.isVisible() or panel.paused:
                    continue
                ty = np.fromiter(buf, dtype=np.float64, count=len(buf))
                ty_other= None
                if overlay and buf_other is not None:
                    ty_other = np.fromiter(buf_other, dtype=np.float64, count=len(buf_other))
                panel.update_data(tx, ty, ty_other)

        # Plano de fase de la barra (solo si su pestaña está visible)
        if self.panel_fase.isVisible():
            n = len(self.t_buf)
            self.panel_fase.update_data(
                np.fromiter(self.t_buf,   dtype=np.float64, count=n),
                np.fromiter(self.pos_buf, dtype=np.float64, count=n),
                np.fromiter(self.vel_buf, dtype=np.float64, count=n),
            )

        #Esquema de la barra en tiempo real
        if self.panel_esquema.isVisible() and len(self.t_buf) > 0:
            theta_act = float(self.pos_buf[-1])
            omega_act = float(self.vel_buf[-1])
            disco_act = float(self.disco_buf[-1])
            if len(self.t_buf) >= 2:
                dt = float(self.t_buf[-1] - self.t_buf[-2])
                if dt <= 0 or dt > 0.5:
                    dt = 1.0 / FS_HZ
            else:
                dt = 1.0 / FS_HZ
            self.panel_esquema.update_telemetria(theta_act, omega_act, disco_act, dt, self.mode)

        # Textos de UI a baja frecuencia
        self._status_div += 1
        cada = max(1, round(PLOT_HZ / STATUS_HZ))
        if self._status_div >= cada:
            self._status_div = 0
            if self.recording:
                self.lbl_record.setText(f"Muestras grabadas: {len(self.recorded_data)}")
            self._update_status_bar()


# =====================================================================
#  Entry point
# =====================================================================
def main():
    pg.setConfigOptions(antialias=False, background='#121212', foreground='#e0e0e0')

    app = QtWidgets.QApplication(sys.argv)

    app.setStyle("Fusion")
    
    ventana = InterfazPendulo()
    ventana.show()
    #Orde de destrucción
    exit_code = app.exec()
    try:
        app.removeEventFilter(ventana)
    except (RuntimeError, AttributeError):
        pass
    del ventana

    return exit_code


if __name__ == "__main__":
    sys.exit(main())