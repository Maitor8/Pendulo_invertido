# -*- coding: utf-8 -*-
import csv
import math
import time
import sys
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


# =====================================================================
#  CONFIGURACIÓN
# =====================================================================
FS_HZ = 100                    # frecuencia de adquisición / telemetría
PLOT_HZ = 20                  # refresco visual: 20 Hz es fluido y deja libre el event loop
PLOT_DT_MS = round(1000 / PLOT_HZ)
MANUAL_KEEPALIVE_MS = 150
STATUS_HZ = 2                 # la barra de estado no necesita actualizarse 50 veces/s
HISTORIAL_S = 30              # segundos de histórico en pantalla
timeout_handshake = 10000
ping_interval_ms= 2000
timeout_ack = 2000

# Estilos 
ESTILO_GLOBAL = """
    QPushButton#btnModo:checked {
        background-color: #2196F3; color: white; font-weight: bold;
    }
"""
ESTILO_ENVIAR = """
    QPushButton          { background-color: #2196F3; color: white; }
    QPushButton:disabled { background-color: #BDBDBD; color: #F5F5F5; }
"""
ESTILO_GRABAR = """
    QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }
"""
ESTILO_KILL = """
    QPushButton         { background-color: #f44336; color: white; font-weight: bold;
                          padding: 15px; font-size: 16px; }
    QPushButton:pressed { background-color: #c62828; }
"""


# =====================================================================
#  CAPA DE COMUNICACIÓN UART  (a futuro: ESP32)
# =====================================================================
class SerialManager(QtCore.QObject):
    #Signals
    telemetria = QtCore.pyqtSignal(float, float, float, float,float)
    desconectado = QtCore.pyqtSignal(str)
    handshake_ok = QtCore.pyqtSignal(dict)
    handshake_perdido = QtCore.pyqtSignal()                
    ganancias_ack = QtCore.pyqtSignal(bool, str, dict) 
    respuesta_exc = QtCore.pyqtSignal(bool, str)
    _HDR = b'\xAA\xBB'
    _PKT_LEN = 20

    def __init__(self, parent=None):
        super().__init__(parent)
        self._port = None
        self.connected = False
        self._hilo_lectura = None
        self._detener = threading.Event()
        self.t_acumulado = 0.0
        self.handshake_completo = False 
        self.firmware_version = "?"   
        self._tx_lock= threading.Lock()  

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

    def _send(self, cmd: str, silencioso: bool=False):
        if not silencioso:
            print(f"[UART TX] - > {cmd}")
        self._escribir((cmd+ '\n').encode())

    def enviar_ganancias(self, gains: dict):
        cmd = "L:" + ",".join(f"{gains[k]:.6f}" for k in sorted(gains))
        self._send(cmd)

    def enviar_modo(self, modo: str):
        self._send(f"MODE:{modo}")

    def enviar_manual(self, direccion: str):
        self._send(f"MANUAL:{direccion}", silencioso=True)

    # def enviar_lazo_abierto(self, voltaje: float):
    #     self._send(f"OPENLOOP:{voltaje:.3f}")

    def enviar_estop(self):
        self._send("ESTOP")   

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

    # ---- Hilo Receptor de Binarios ----
    def _leer_uart(self):
        """Hilo lector. Lee en bloques con timeout; separa binario de ASCII."""
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

            if i_nl >= 0 and (i_hdr < 0 or i_nl < i_hdr):
                linea = bytes(buf[:i_nl]).decode('ascii', errors='ignore').strip()
                del buf[:i_nl + 1]
                if linea and linea.isprintable():
                    self._procesar_linea_ascii(linea)
                continue

            if i_hdr >= 0:
                if i_hdr > 0:
                    del buf[:i_hdr]
                if len(buf) < self._PKT_LEN:
                    return
                dt, theta, theta_dot, omega, u_raw = struct.unpack(
                    '<ffffh', bytes(buf[2:self._PKT_LEN]))
                if self._paquete_plausible(dt, theta, theta_dot, omega):
                    del buf[:self._PKT_LEN]
                    self.t_acumulado += dt
                    self.telemetria.emit(self.t_acumulado, theta, theta_dot,
                                        omega, u_raw / 100.0)
                else:
                    del buf[:1]
                continue

            if len(buf) > 256:
                buf.clear()
            return

    @staticmethod
    def _paquete_plausible(dt, theta, theta_dot, omega):
        return (0.0 < dt < 1.0
                and all(math.isfinite(v) and abs(v) < 1e5
                        for v in (theta, theta_dot, omega)))

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

                self.handshake_completo = True
                self.handshake_ok.emit({
                    'firmware': self.firmware_version,
                    'L1': l1, 'L2': l2, 'L3': l3,
                    'mode': modo,
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
    def __init__(self, titulo, ylabel, color, parent=None):
        super().__init__(parent)
        self.paused = False

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
        self.plot = pg.PlotWidget(title=titulo)
        self.plot.setLabel('left', ylabel)
        self.plot.setLabel('bottom', 'Tiempo (s)')
        for eje in ('left', 'bottom'):
            self.plot.getAxis(eje).enableAutoSIPrefix(False)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setDownsampling(auto=True, mode='peak')
        self.plot.setClipToView(True)
        self.curve = self.plot.plot(pen=pg.mkPen(color, width=2))
        self.curve.setSkipFiniteCheck(True)
        layout.addWidget(self.plot)

        vb = self.plot.getViewBox()
        vb.enableAutoRange(axis=pg.ViewBox.XAxis, enable=False)
        vb.enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)
        self.setMinimumHeight(180)
        self.chk_pause.toggled.connect(self._on_pause)

    def _on_pause(self, activo):
        self.paused = activo

    def update_data(self, t_array, y_array):
        """Actualiza la curva. Si 'Auto escalado' está marcado, ajusta X e Y."""
        if self.paused:
            return
        self.curve.setData(t_array, y_array)

        if not self.chk_auto.isChecked():
            return                       # el usuario manda: no tocar los rangos

        if t_array.size < 2:
            return

        t0, t1 = float(t_array[0]), float(t_array[-1])

        if t1 <= t0:
            return

        vb = self.plot.getViewBox()
        vb.setXRange(t0, t1, padding=0.0)

        ymin = float(np.min(y_array))
        ymax = float(np.max(y_array))
        if ymax > ymin:
            pad = 0.05 * (ymax - ymin)  
            vb.setYRange(ymin - pad, ymax + pad, padding=0.0)
        else:
            vb.setYRange(ymin - 0.5, ymax + 0.5, padding=0.0)


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
        # ACK de ganancias
        self._ack_pendiente = False
        self._ack_timer = QtCore.QTimer(self)
        self._ack_timer.setSingleShot(True)
        self._ack_timer.timeout.connect(self._on_ack_timeout)
        self._sincronizado = False
        self._hs_timer = QtCore.QTimer(self)
        self._hs_timer.setSingleShot(True)
        self._hs_timer.timeout.connect(self._check_handshake_timeout)
        self._evento_timer = QtCore.QTimer(self)
        self._evento_timer.setSingleShot(True)
        self._evento_timer.timeout.connect(lambda: self.lbl_evento.setText(""))

        # Control manual: qué entradas (botón / tecla) están sostenidas
        self._manual_held = set()
        self._manual_last = "STOP"
        self._manual_timer = QtCore.QTimer(self)
        self._manual_timer.timeout.connect(self._manual_keepalive)  

        # ---------- Buffers (histórico deslizante) ----------
        n = HISTORIAL_S * FS_HZ
        self.t_buf = deque(maxlen=n)
        self.pos_buf = deque(maxlen=n)
        self.vel_buf = deque(maxlen=n)
        self.disco_buf = deque(maxlen=n)
        self.u_buf= deque(maxlen=n)

        # ---------- Comunicación ----------
        self.serial = SerialManager(self)
        self.serial.telemetria.connect(self._on_muestra)
        self.serial.desconectado.connect(self._on_serial_desconectado)
        self.serial.handshake_ok.connect(self._on_handshake)
        self.serial.handshake_perdido.connect(self._on_handshake_perdido)
        self.serial.ganancias_ack.connect(self._on_ganancias_ack)
        self.serial.respuesta_exc.connect(self._on_respuesta_exc)

        # ---------- UI ----------
        self._build_ui()
        self.setStyleSheet(ESTILO_GLOBAL)
        self._on_mode_changed("STANDBY", forzar=True)
        self._set_controles_conectados(False)

        # Captura de flechas ← → en modo manual (ver eventFilter)
        QtWidgets.QApplication.instance().installEventFilter(self)
        self._sc_kill = QtGui.QShortcut(QtGui.QKeySequence("Esc"), self)
        self._sc_kill.activated.connect(self._on_kill_switch)

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.setTimerType(QtCore.Qt.TimerType.CoarseTimer)
        self.plot_timer.timeout.connect(self._plot_tick)
        self.plot_timer.start(PLOT_DT_MS)
        self.port_timer = QtCore.QTimer(self)
        self.port_timer.timeout.connect(self._refresh_puertos)
        self.port_timer.start(1500)   # cada 1.5 s
        self.ping_timer = QtCore.QTimer(self)
        self.ping_timer.timeout.connect(self._send_ping)
        self.ping_timer.start(ping_interval_ms)   # cada 2 segundos

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
        self.panel_pos_barra = PlotPanel("Posición de la Barra (rad)", "θ (rad)", 'b')
        self.panel_vel_barra = PlotPanel("Velocidad de la Barra (rad/s)", "ω (rad/s)", 'r')
        self.panel_vel_disco = PlotPanel("Velocidad del Disco / Puente H (rad/s)",
                                        "ω disco (rad/s)", 'g')
        self.panel_u         = PlotPanel("Excitación · u (V)", "u (V)", (255, 140, 0))  # naranja

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
        layout_controles.addWidget(self._crear_grupo_manual())
        layout_controles.addWidget(self._crear_grupo_lazo_abierto())
        layout_controles.addWidget(self._crear_grupo_parametros())
        layout_controles.addWidget(self._crear_grupo_grabacion())
        layout_controles.addStretch()
        scroll.setWidget(contenido)

        layout_derecho.addWidget(scroll, stretch=1)
        # El KILL SWITCH queda fuera del scroll: nunca puede quedar oculto
        layout_derecho.addWidget(self._crear_grupo_seguridad())

        splitter.addWidget(panel_graficas)
        splitter.addWidget(panel_derecho)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([930, 350])
        layout_principal.addWidget(splitter)

        #Mensajes de evento
        self.lbl_evento = QtWidgets.QLabel("")
        self.lbl_evento.setStyleSheet("color: #b00; padding-right: 8px;")
        self.statusBar().addPermanentWidget(self.lbl_evento)

    # ---- Grupo: conexión UART ----
    def _crear_grupo_conexion(self):
        g = QtWidgets.QGroupBox("Conexión UART")
        v = QtWidgets.QVBoxLayout(g)

        h = QtWidgets.QHBoxLayout()
        h.addWidget(QtWidgets.QLabel("Puerto:"))
        self.cmb_port = QtWidgets.QComboBox()
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
        h.setSpacing(10)

        self.chk_graf = {}
        for clave, etiqueta in [
            ("pos",   "θ barra"),
            ("vel",   "ω barra"),
            ("disco", "ω disco"),
            ("u",     "u excitación"),
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
            btn.clicked.connect(lambda _, k=clave: self._on_mode_changed(k))
            self.mode_group.addButton(btn)
            self.mode_buttons[clave] = btn
            v.addWidget(btn)

        self.mode_buttons["STANDBY"].setChecked(True)
        return g

    # ---- Grupo: control manual ----
    def _crear_grupo_manual(self):
        g = QtWidgets.QGroupBox("Control Manual")
        v = QtWidgets.QVBoxLayout(g)

        ayuda = QtWidgets.QLabel(
            "Usa las flechas ← → del teclado.\n"
            "Mantén pulsada la flecha para mover; al soltarla el motor se detiene."
        )
        ayuda.setWordWrap(True)
        ayuda.setStyleSheet("color: #666; font-size: 9pt;")
        v.addWidget(ayuda)
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
        self.cmb_exc = QtWidgets.QComboBox()
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
        self.sp_sq_freq = QtWidgets.QDoubleSpinBox()
        self.sp_sq_freq.setRange(0.05, 20.0); self.sp_sq_freq.setValue(0.5)
        self.sp_sq_freq.setSuffix(" Hz")
        self.sp_sq_amp = QtWidgets.QDoubleSpinBox()
        self.sp_sq_amp.setRange(1, 100); self.sp_sq_amp.setValue(10)
        self.sp_sq_amp.setSuffix(" %")
        self.sp_sq_T = QtWidgets.QDoubleSpinBox()
        self.sp_sq_T.setRange(1, 300); self.sp_sq_T.setValue(20)
        self.sp_sq_T.setSuffix(" s")
        f_sq.addRow("Frecuencia:", self.sp_sq_freq)
        f_sq.addRow("Amplitud:",   self.sp_sq_amp)
        f_sq.addRow("Duración:",   self.sp_sq_T)
        v.addWidget(self.grp_square)

        # --- Parámetros de chirp ---
        self.grp_chirp = QtWidgets.QGroupBox("Parámetros · Chirp logarítmico")
        f_ch = QtWidgets.QFormLayout(self.grp_chirp)
        self.sp_ch_f0 = QtWidgets.QDoubleSpinBox()
        self.sp_ch_f0.setRange(0.01, 10); self.sp_ch_f0.setValue(0.15)
        self.sp_ch_f0.setSuffix(" Hz")
        self.sp_ch_f1 = QtWidgets.QDoubleSpinBox()
        self.sp_ch_f1.setRange(0.5, 50); self.sp_ch_f1.setValue(10.0)
        self.sp_ch_f1.setSuffix(" Hz")
        self.sp_ch_T = QtWidgets.QDoubleSpinBox()
        self.sp_ch_T.setRange(5, 300); self.sp_ch_T.setValue(60)
        self.sp_ch_T.setSuffix(" s")
        self.sp_ch_amp = QtWidgets.QDoubleSpinBox()
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
        self.sp_pr_br = QtWidgets.QDoubleSpinBox()
        self.sp_pr_br.setRange(0.1, 10.0); self.sp_pr_br.setValue(2.0)
        self.sp_pr_br.setSuffix(" Hz")
        self.sp_pr_br.setToolTip("Frecuencia de cambio de bit.\n"
                                "Mantener ≤ 10 Hz para buena resolución a 100 Hz de muestreo.")
        self.sp_pr_amp = QtWidgets.QDoubleSpinBox()
        self.sp_pr_amp.setRange(1, 100); self.sp_pr_amp.setValue(5)
        self.sp_pr_amp.setSuffix(" %")
        self.sp_pr_T = QtWidgets.QDoubleSpinBox()
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
            lbl.setStyleSheet("font-size: 9pt; color: #555;")

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
        self.btn_kill.setToolTip("Apaga el puente H inmediatamente (comando prioritario por UART)")
        self.btn_kill.clicked.connect(self._on_kill_switch)
        v.addWidget(self.btn_kill)
        return g

    # -----------------------------------------------------------------
    #  Helpers de estado
    # -----------------------------------------------------------------
    def _update_gain_button_state(self):
        puede = (not self.motor_moving) and self.serial.handshake_completo
        self.btn_send_gains.setEnabled(puede)
        for le in self.inputs_L.values():
            le.setEnabled(puede)
        if puede:
            self.lbl_gains_lock.setText("")
        elif not self.serial.handshake_completo:
            self.lbl_gains_lock.setText("⚠ Sin conexión:handshake no establecido.")
        else:
            self.lbl_gains_lock.setText(
                "⚠ Bloqueado: el motor está en movimiento. "
                "Detenga el sistema (En Espera) para enviar nuevas ganancias."
            )

    def _set_motor_moving(self, moving: bool):
        if self.motor_moving != moving:
            self.motor_moving = moving
            self._update_gain_button_state()

    def _update_status_bar(self):
        estado = "conectado" if self.serial.connected else "desconectado"
        self.statusBar().showMessage(
            f"Estado: {estado}  |  Modo: {self.mode}  |  "
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
        self.btn_kill.setEnabled(conectado)
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
        tipo = event.type()
        if (tipo in (QtCore.QEvent.Type.KeyPress, QtCore.QEvent.Type.KeyRelease)
                and self.mode == "MANUAL" and self.isActiveWindow()
                and event.key() in self.TECLAS_MANUAL):
            foco = QtWidgets.QApplication.focusWidget()
            if not isinstance(foco, (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox,
                                     QtWidgets.QComboBox)):
                if not event.isAutoRepeat():
                    self._manual_set(self.TECLAS_MANUAL[event.key()],
                                    tipo == QtCore.QEvent.Type.KeyPress)
                return True
        return super().eventFilter(obj, event)

    def changeEvent(self, event):
        # Seguridad: si la ventana pierde el foco con una tecla sostenida, detener.
        if event.type() == QtCore.QEvent.Type.ActivationChange and not self.isActiveWindow():
            self._liberar_manual()
        super().changeEvent(event)

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
                "El puerto se abrió pero el ESP32 no respondió al handshake en "
                f"{timeout_handshake/1000:.0f} s.\n"
                "Verifique que el firmware tenga el protocolo HELLO/READY/STATE."
            )

    @QtCore.pyqtSlot(dict)
    def _on_handshake(self, info):
        print(f"[UI] Handshake OK con firmware {info['firmware']}")
        self._hs_timer.stop()
        self.gains = {'L1': info['L1'], 'L2': info['L2'], 'L3': info['L3']}
        if not self._sincronizado:
            for key, le in self.inputs_L.items():
                le.setText(f"{self.gains[key]:g}")
            self._sincronizado = True

        self._set_controles_conectados(True)

        # Sincronizar la UI con el modo que reporta el firmware
        modo_fw = info.get('mode', 'STANDBY')
        if modo_fw in self.mode_buttons:
            # setChecked no dispara `clicked`, así que no se reenvía MODE al firmware
            self.mode_buttons[modo_fw].setChecked(True)
            self.mode = modo_fw
            self._set_controles_ol_habilitados(modo_fw == "OPEN_LOOP")
            self._set_motor_moving(modo_fw != "STANDBY")

        self.statusBar().showMessage(
            f"Conectado a {info['firmware']} | {modo_fw} | "
            f"L=({info['L1']:.4f}, {info['L2']:.4f}, {info['L3']:.4f})"
        )
    
    @QtCore.pyqtSlot(bool, str)
    def _on_respuesta_exc(self, ok: bool, texto: str):
        self._mostrar_evento(("✔ " if ok else "⚠ ") + texto, ms=6000,
                            color="#2e7d32" if ok else "#b00")

    def _mostrar_evento(self, texto: str, ms: int = 6000, color: str = "#b00"):
        self.lbl_evento.setStyleSheet(f"color: {color}; padding-right: 8px;")
        self.lbl_evento.setText(texto)
        self._evento_timer.start(ms)

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
        for panel in self._paneles_graficas.values():
            panel.curve.setData([], [])
            vb = panel.plot.getViewBox()
            vb.setXRange(0, 1, padding=0.0)
            vb.setYRange(-1, 1, padding=0.0)
        self._status_div = 0

    def _on_mode_changed(self, modo, forzar=False):
        if modo != "STANDBY" and not self.serial.handshake_completo:
            self.mode_buttons["STANDBY"].setChecked(True)
            return
        if modo == self.mode and not forzar:
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
        self.gains = nuevas
        self._ack_pendiente = True
        self.btn_send_gains.setEnabled(False)
        for le in self.inputs_L.values():
            le.setEnabled(False)
        self.lbl_gains_lock.setText("⏳ Esperando confirmación del ESP32...")

        self.serial.enviar_ganancias(self.gains)
        self._ack_timer.start(timeout_ack)

    @QtCore.pyqtSlot(bool, str, dict)
    def _on_ganancias_ack(self, ok: bool, motivo: str, gains: dict):
        if not self._ack_pendiente:
            return                          # ACK fuera de contexto: ignorar
        self._ack_pendiente = False
        self._ack_timer.stop()

        if ok:
            self.gains = gains     
            self._restaurar_campos_ganancias()
            QtWidgets.QMessageBox.information(
                self, "Confirmado",
                f"El ESP32 confirmó las ganancias:\n"
                f"L = ({gains['L1']:.4f}, {gains['L2']:.4f}, {gains['L3']:.4f})"
            )
        else:
            self._restaurar_campos_ganancias()
            QtWidgets.QMessageBox.critical(
                self, "Rechazado por el ESP32",
                f"El ESP32 no aceptó las ganancias.\n\nMotivo: {motivo}"
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
            "Verifique el cable o el estado del firmware."
        )
        self._update_gain_button_state()

    def _on_kill_switch(self):
        print("[UI] ¡Batiseñal activada!")
        self.serial.enviar_estop()
        self.mode_buttons["STANDBY"].setChecked(True)
        self._on_mode_changed("STANDBY")
        self._set_motor_moving(False)

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
                w.writerow(["t_s", "theta_rad", "omega_rad_s", "v_disco_rad_s", "u_v"])
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
    @QtCore.pyqtSlot(float, float, float, float, float)
    def _on_muestra(self, t, theta, omega, v_disco, u):
        """Punto único de entrada de datos (simulados hoy, UART mañana)."""
        self.t_buf.append(t)
        self.pos_buf.append(theta)
        self.vel_buf.append(omega)
        self.disco_buf.append(v_disco)
        self.u_buf.append(u)

        if self.recording:
            self.recorded_data.append((t, theta, omega, v_disco, u))

    def _plot_tick(self):
        """Refresco visual desacoplado de la adquisición para mantener la UI fluida."""
        if not self.t_buf:
            return

        paneles = [
            (self.panel_pos_barra,  self.pos_buf),
            (self.panel_vel_barra,  self.vel_buf),
            (self.panel_vel_disco,  self.disco_buf),
            (self.panel_u,          self.u_buf),
        ]

        
        hay_activo = any(p.isVisible() and not p.paused for p, _ in paneles)
        if hay_activo:
            tx = np.fromiter(self.t_buf, dtype=np.float64, count=len(self.t_buf))
            for panel, buf in paneles:
                if not panel.isVisible() or panel.paused:
                    continue
                ty = np.fromiter(buf, dtype=np.float64, count=len(buf))
                panel.update_data(tx, ty)

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
    pg.setConfigOptions(antialias=False, background='w', foreground='k')

    app = QtWidgets.QApplication(sys.argv)
    ventana = InterfazPendulo()
    ventana.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()