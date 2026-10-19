/* Firmware ESP32 - Péndulo invertido con disco de inercia (v2.3, base: main_1.c)
 * v2.3: estimador de velocidades seleccionable desde la GUI (EST:DIF / EST:OBS).
 *   - DIF: diferencias finitas + promedio móvil (el de siempre).
 *   - OBS: observador de Luenberger discreto con el modelo no lineal (q1,q2,q3,rho) y las
 *     mediciones de ambos encoders. Ganancias editables: OBS:lb1,lb2,ld1,ld2 (solo en STANDBY).
 *   Los dos corren siempre en paralelo; el elegido alimenta al control y a la telemetría.
 * Correcciones sobre main_1.c:
 *  1. _Static_assert del tick de FreeRTOS: con PEND_TS_MS=5 hace falta CONFIG_FREERTOS_HZ>=200
 *     (ESP-IDF trae 100 por defecto: pdMS_TO_TICKS(5)=0 y la tarea de control no cede la CPU).
 *  2. Freno por sobrevelocidad restaurado (estaba calculado y sin usar), con histéresis.
 *  3. Cambios de modo automáticos (MANUAL->STABILIZATION->STANDBY) hechos con comparar-y-asignar
 *     bajo spinlock: antes podían pisar un ESTOP/MODE llegado por la UART en ese instante.
 *  4. PWM_MIN_BALANCE ahora se usa; zona muerta distinta en balanceo y en swing-up/manual.
 *  5. Watchdog: con el motor activo, TIMEOUT_MOTOR_MS sin tráfico -> STANDBY (revisado cada 250 ms).
 *  6. Macros duplicadas e inconsistentes (PEND_*_DEG, DEG_POR_CUENTA) eliminadas.
 *  7. (v2.3) motor_set() devuelve la u efectiva (con zona muerta): es la entrada del observador.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"

//===================
//Defines
//===================
//Filtro
#define FD_FILTRO_N   5

//Pines
#define ENC_BARRA_A      GPIO_NUM_21
#define ENC_BARRA_B      GPIO_NUM_22
#define ENC_DISCO_A      GPIO_NUM_18
#define ENC_DISCO_B      GPIO_NUM_19

#define RPWM_PIN         GPIO_NUM_26
#define LPWM_PIN         GPIO_NUM_27
#define PIN_EN           GPIO_NUM_14

//Control
#define PI_F             ((float)M_PI)
#define PEND_TS_MS       5  //5ms = 200Hz
#define PEND_TS_S        (PEND_TS_MS / 1000.0f)
#define PEND_UMax        12.0f
#define PEND_ZONA2       (988.0f * PI_F / 1000.0f) //pocos grados para la captura
//Zonas expresadas como error angular respecto a la vertical:
//captura ±2.2°,  salida del LQR ±5.7°
#define PEND_CAPT_RAD    (PI_F - PEND_ZONA2)
#define PEND_SALIDA_RAD  (PI_F - 0.98f * PEND_ZONA2)
#define U_MANUAL         -0.8f

//PWMs
#define PWM_FREQ_HZ      20000
#define PWM_RES          LEDC_TIMER_10_BIT
#define PWM_MAX          1023

//Encoders
#define ENC_LIMIT        30000        
#define PEND_CPR         2000.0f        
#define RAD_POR_CUENTA   (2.0f * PI_F / PEND_CPR)
#define Filter_ns        100 //el otro día era de 50, mucho ruido subirlo

//Uart
#define UART_NUM              UART_NUM_0
#define UART_BUF_SIZE         2048
#define TIMEOUT_HANDSHAKE_MS  5000
#define PING_HELLO_MS         1000
#define MANUAL_DEADMAN_MS     700     //sin manual en x tiempo, motor a 0
#define TIMEOUT_MOTOR_MS      1500    //motor activo y sin tráfico de la GUI -> STANDBY
#define WATCHDOG_PERIODO_MS   250     //cada cuánto se revisa el timeout
#define SOBREVEL_ACTIVO       1       //0 desactiva el freno por RPM_MAX (no recomendado)

//Ctes de energía para el swing-up
#define PEND_Q1          26.0736f//param q1
#define PEND_Q2          0.0189f//q2
#define PEND_Q3          0.3819f//q3
#define PEND_RHO         241.5725f //param rho

#define PEND_WMAXI_BASE  (PEND_Q1 * PEND_RHO)             
#define PEND_KMOT   (1.0f / (PEND_RHO * PEND_Q3 * 0.08f))   
#define PEND_CERO   (0.2f * sqrtf(PEND_Q1))//1.02 rad/s; si el swing no arranca, bajarlo

//Motor
#define PWM_MIN_SWING    100
#define PWM_MIN_BALANCE  100
#define RPM_MAX 3000

//Inversiones, encoder y motor
#define INVERTIR_BARRA   1.0f
#define INVERTIR_DISCO   -1.0f
#define SWING_SIGN    1.0f
#define BALANCE_SIGN  1.0f

#define DIFERENCIAS 3

//Núcleos (control aislado de la comunicación)
//vTaskDelayUntil trabaja en ticks: el periodo de control debe ser un número entero de ticks.
//Con CONFIG_FREERTOS_HZ=100 (por defecto) 5 ms = 0 ticks. Subirlo a 1000 en menuconfig.

#if CONFIG_FREERTOS_UNICORE
  #define CORE_CTRL 0
  #define CORE_COMM 0
#else
  #define CORE_CTRL 1
  #define CORE_COMM 0
#endif


//============================
//Tipos
//============================
//Modos de funcionamiento (nombres = los que envía la GUI)
typedef enum {
    MODO_STANDBY = 0,
    MODO_MANUAL,
    MODO_OPEN_LOOP,
    MODO_STABILIZATION,
    MODO_AUTO,
    MODO_N
} modo_t;

static const char *const MODO_NOMBRE[MODO_N] = {
    "STANDBY", "MANUAL", "OPEN_LOOP", "STABILIZATION", "AUTO"
};

//Ensayo de lazo abierto
typedef enum {
    EXC_NINGUNA = 0,
    EXC_SQUARE,
    EXC_CHIRP,
    EXC_PRBS,
} exc_tipo_t;

 //config para las EXCitaciones
typedef struct {
    exc_tipo_t tipo;
    float freq;       
    float amp_pct;    
    float f0, f1;     
    float dur;       
    float bitrate;    
} exc_cfg_t;

//Paquete binario de las señales
#pragma pack(push, 1)
typedef struct {
    uint8_t h1;             //h1
    uint8_t h2;             //h2
    float   dt;             //periodo
    float   theta;          //pos barra rad
    float   theta_dot;      //vel barra rad/s
    float   omega_rueda;    //vel disco rad/s
    int16_t u;              //u*100 (u en unidades de control, ±PEND_UMax; no en volt)
} paquete_t;
#pragma pack(pop)
_Static_assert(sizeof(paquete_t) == 20, "paquete_t debe medir 20 bytes (la GUI lee 2 + 18)");

//Balanceo
typedef enum {
    AUTO_SWINGUP = 0,
    AUTO_LQR,
} auto_fase_t;

typedef struct { float l1, l2, l3; } ganancias_t;

//Estimador de velocidades
typedef enum { EST_DIF = 0, EST_OBS = 1 } est_modo_t;
static const char *const EST_NOMBRE[] = { "DIF", "OBS" };
//Ganancias CONTINUAS del observador: barra (lb1: posición, lb2: velocidad) y disco (ld1, ld2).
//Críticamente amortiguado con ancho de banda wo:  l1 = 2*wo,  l2 = wo^2
typedef struct { float lb1, lb2, ld1, ld2; } obs_gains_t;

//============================
//ESTADO GLOBAL
//============================
/* Regla: lo que está bajo "estado_mux" lo escribe RX/hello y lo LEE la tarea
   de control en una copia local. Lo demás (estimador, auto_fase, exc_act)
   es exclusivo de la tarea de control. */
static portMUX_TYPE estado_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile modo_t  modo            = MODO_STANDBY;
static ganancias_t      ganancias       =  { -248.9894f, -48.6771f, -0.1108f};
static volatile bool    req_reset       = false; //pedido del reset
static exc_cfg_t        exc_pend;                  //excitacion pedida por rx
static volatile bool    exc_req_start   = false;
static volatile bool    exc_req_stop    = false;

//Pedidos de la GUI
static volatile bool req_zero = false; //pedido de cero de la barra
static volatile bool req_wmaxi = false; //pedido de wmaxi
static volatile float wmaxi_pend_mult = 2.1f; //multiplicador de wmaxi, para el swing-up

//Estimador de velocidades (lo escribe RX solo en STANDBY; la tarea de control lee una copia)
static volatile est_modo_t est_modo = EST_DIF;
static obs_gains_t obs_g = { 80.0f, 1600.0f, 60.0f, 900.0f };  //wo barra = 40 rad/s, wo disco = 30 rad/s

//Handshake
static volatile bool     handshake_ok        = false;
static volatile uint32_t t_ultimo_comando_ms = 0;

//Control manual
static volatile bool     manual_izq   = false;
static volatile bool     manual_dcha   = false;
static volatile uint32_t t_manual_ms  = 0;

//Hardware
static pcnt_unit_handle_t pcnt_barra = NULL;
static pcnt_unit_handle_t pcnt_disco = NULL;
static SemaphoreHandle_t  motor_mtx  = NULL;

// Exclusivo de la tarea de control 
static auto_fase_t auto_fase = AUTO_SWINGUP;
static bool    stab_capturada = false; //si se capturó la barra en vertical

static int32_t cnt_b_act = 0, cnt_d_act = 0;   //cuentas acumuladas
static float   pb = 0.0f;                      //ángulo barra [rad] 

static int32_t cnt_b_ref = 0; //cuentas de la barra 
static float wmaxi_mult = 2.1f; //multiplicador vigente


static int32_t cb_hist[DIFERENCIAS] = {0};               //[k-1], [k], [k+1] en cuentas
static int32_t cd_hist[DIFERENCIAS] = {0};
static float   vbarra_hist[FD_FILTRO_N] = {0};
static float   vdisco_hist[FD_FILTRO_N] = {0};
static int     fd_idx = 0, fd_count = 0, ma_n = 0;
static float   vb_est = 0.0f; //vel estimadas (las del estimador elegido; las usa el control)
static float   vd_est = 0.0f;
static float   vb_fd = 0.0f, vd_fd = 0.0f;     //salidas de las diferencias finitas
static float   vb_obs = 0.0f, vd_obs = 0.0f;   //salidas del observador

//Estado del observador. Se guarda delta = (estimado) - (última medición) en vez del ángulo
//absoluto: el disco puede dar miles de vueltas y float32 perdería resolución.
static bool    obs_init = false;
static int32_t obs_cb_prev = 0, obs_cd_prev = 0;   //cuentas de la muestra anterior
static float   obs_pb_prev = 0.0f;                 //ángulo medido de la barra, muestra anterior
static float   obs_db = 0.0f, obs_dd = 0.0f;       //delta barra / disco [rad]
static float   obs_wb = 0.0f, obs_wd = 0.0f;       //velocidades estimadas [rad/s]
static float   u_aplicada = 0.0f;                  //u efectiva del ciclo anterior (entrada del modelo)

static exc_cfg_t exc_act;                      
static int64_t   exc_t_inicio_us = 0;
static uint32_t  prbs_lfsr = 0xACE1u;
static int64_t   prbs_t_ultimo_bit_us = 0;
static int       prbs_bit_actual = 1;

//============================
//Utilidades
//============================
static inline uint32_t ms_now(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void responder(const char *s)
{
    uart_write_bytes(UART_NUM, s, strlen(s));
}

static inline float sat(float x, float lim)
{
    if (x >  lim) return  lim;
    if (x < -lim) return -lim;
    return x;
}

//=====================
//Configuraciones
//=====================
//Uart
static void configurar_uart(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
}

//Pwm del puente/motor
static void configurar_pwm(void)
{
    motor_mtx = xSemaphoreCreateMutex();

    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    //ch0 para dcha
    ledc_channel_config_t ch_r = {
        .gpio_num = RPWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel  = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0,
        .duty = 0, .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_r));
    //ch1 para izq
    ledc_channel_config_t ch_l = {
        .gpio_num = LPWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel  = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0,
        .duty = 0, .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_l));

    //puente H (EN recién después de tener los PWM en 0)
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_EN),
        .mode         = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(PIN_EN, 1);
}

//encoders
static void configurar_encoder(int pin_a, int pin_b, pcnt_unit_handle_t *handle)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit = ENC_LIMIT,
        .low_limit  = -ENC_LIMIT,
        .flags.accum_count = true,   /* suma el desborde en la ISR -> contador de 32 bits */
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, handle));

    //Config en x4
    //Canal A: cuenta sobre A, dirección por B
    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num  = pin_a,
        .level_gpio_num = pin_b,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*handle, &chan_a_cfg, &chan_a));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Canal B: cuenta sobre B, dirección por A
    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num  = pin_b,
        .level_gpio_num = pin_a,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(*handle, &chan_b_cfg, &chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    //Filtro anti-rebote
    pcnt_glitch_filter_config_t filter = { .max_glitch_ns = Filter_ns };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*handle, &filter));

    //Para que accum_count compense el desborde hay que vigilar ambos límites
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*handle,  ENC_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*handle, -ENC_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(*handle));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*handle));
    ESP_ERROR_CHECK(pcnt_unit_start(*handle));
}

//============================
//Funciones varias
//============================
//Aplicar u  (segura entre tareas: ESTOP la llama desde RX)
static float motor_set(float u, int dmin)
{
    if (!isfinite(u)) u = 0.0f;
    u = sat(u, PEND_UMax);

    //3 simples 
    int duty = (int)(fabsf(u) / PEND_UMax * PWM_MAX);
    if (duty > PWM_MAX) duty = PWM_MAX;

    //zona muerta
    if (duty > 0 && duty < dmin) duty = dmin;

    int d0 = 0, d1 = 0;
    if      (u >  0.5f) d0 = duty;
    else if (u < -0.5f) d1 = duty;

    xSemaphoreTake(motor_mtx, portMAX_DELAY);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, d0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, d1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    xSemaphoreGive(motor_mtx);

    //u efectiva realmente aplicada (con la zona muerta ya compensada)
    float u_ef = (float)duty / (float)PWM_MAX * PEND_UMax;
    return (d1 > 0) ? -u_ef : ((d0 > 0) ? u_ef : 0.0f);
}

//send state
static void enviar_estado(void)
{
    ganancias_t g;
    modo_t m;
    float wm;
    est_modo_t em;
    obs_gains_t og;
    portENTER_CRITICAL(&estado_mux);
    g = ganancias;
    m = modo;
    wm = wmaxi_pend_mult; //valor pendiente
    em = est_modo;
    og = obs_g;
    portEXIT_CRITICAL(&estado_mux);

    char buf[256];
    int n = snprintf(buf, sizeof(buf),
                     "STATE,L=%.4f,%.4f,%.4f;MODE=%s;WMAXI=%.3f;EST=%s;OBS=%.3f,%.3f,%.3f,%.3f\n",
                     g.l1, g.l2, g.l3, MODO_NOMBRE[m], wm, EST_NOMBRE[em],
                     og.lb1, og.lb2, og.ld1, og.ld2);
    if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
    if (n > 0) uart_write_bytes(UART_NUM, buf, n);
}

//Lleva todo a reposo. Se puede llamar desde cualquier tarea
static void ir_a_standby(void)
{
    portENTER_CRITICAL(&estado_mux);
    modo          = MODO_STANDBY;
    manual_izq    = false;
    manual_dcha    = false;
    req_reset     = true;
    exc_req_start = false;
    exc_req_stop  = true;
    portEXIT_CRITICAL(&estado_mux);
}

//Excitación de Lazo abierto  (solo tarea de control)
static int prbs_next_bit(void)
{
    uint32_t bit = ((prbs_lfsr >> 0) ^ (prbs_lfsr >> 2) ^
                    (prbs_lfsr >> 3) ^ (prbs_lfsr >> 5)) & 1u;
    prbs_lfsr = (prbs_lfsr >> 1) | (bit << 15);
    return prbs_lfsr & 1u;
}

static bool exc_cfg_valida(const exc_cfg_t *c)
{
    if (!isfinite(c->amp_pct) || c->amp_pct <= 0.0f || c->amp_pct > 100.0f) return false;
    if (!isfinite(c->dur)     || c->dur     <= 0.0f || c->dur     > 600.0f) return false;
    switch (c->tipo) {
    case EXC_SQUARE:
        return isfinite(c->freq) && c->freq > 0.0f && c->freq <= 50.0f;
    case EXC_CHIRP:
        return isfinite(c->f0) && isfinite(c->f1) &&
               c->f0 > 0.0f && c->f1 > 0.0f && c->f0 <= 50.0f && c->f1 <= 50.0f &&
               fabsf(c->f1 - c->f0) > 1e-3f;
    case EXC_PRBS:
        return isfinite(c->bitrate) && c->bitrate > 0.0f && c->bitrate <= 50.0f;
    default:
        return false;
    }
}

static float exc_generar_u(void)
{
    if (exc_act.tipo == EXC_NINGUNA) return 0.0f;

    int64_t ahora = esp_timer_get_time();
    float t = (ahora - exc_t_inicio_us) / 1e6f;

    if (t >= exc_act.dur) {
        exc_act.tipo = EXC_NINGUNA;
        return 0.0f;
    }

    float amp = (exc_act.amp_pct / 100.0f) * PEND_UMax;

    switch (exc_act.tipo) {
    case EXC_SQUARE: {
        float fase = 2.0f * PI_F * exc_act.freq * t;
        return (sinf(fase) >= 0.0f) ? amp : -amp;
    }
    case EXC_CHIRP: {
        float K    = exc_act.dur / logf(exc_act.f1 / exc_act.f0);
        float fase = 2.0f * PI_F * exc_act.f0 * K * (expf(t / K) - 1.0f);
        return amp * sinf(fase);
    }
    case EXC_PRBS: {
        int64_t periodo_us = (int64_t)(1e6f / exc_act.bitrate);
        if ((ahora - prbs_t_ultimo_bit_us) >= periodo_us) {
            prbs_bit_actual      = prbs_next_bit();
            prbs_t_ultimo_bit_us = ahora;
        }
        return prbs_bit_actual ? amp : -amp;
    }
    default:
        return 0.0f;
    }
}

//Para resetear el observador trucho
static void resetear_estimador(void)
{
    for (int i = 0; i < DIFERENCIAS; i++) { cb_hist[i] = cnt_b_act; cd_hist[i] = cnt_d_act; }
    for (int i = 0; i < FD_FILTRO_N; i++) {
        vbarra_hist[i] = 0.0f;
        vdisco_hist[i] = 0.0f;
    }
    fd_idx = 0;
    fd_count = 0;
    ma_n = 0;
    vb_est = 0.0f;
    vd_est = 0.0f;
    vb_fd = vd_fd = 0.0f;
    vb_obs = vd_obs = 0.0f;
    obs_init = false;          //el observador arranca de la próxima medición
}

// Aplica los pedidos que dejó RX. Se llama al inicio de cada ciclo.
static void aplicar_comandos_pendientes(void)
{
    bool reset, ex_start, ex_stop, zero, wm_req;
    exc_cfg_t cfg;
    float wm_val;

    portENTER_CRITICAL(&estado_mux);
    reset         = req_reset;      req_reset     = false;
    ex_stop       = exc_req_stop;   exc_req_stop  = false;
    ex_start      = exc_req_start;  exc_req_start = false;
    zero          = req_zero;       req_zero      = false;
    wm_req        = req_wmaxi;      req_wmaxi     = false;
    wm_val        = wmaxi_pend_mult;
    cfg           = exc_pend;
    portEXIT_CRITICAL(&estado_mux);

    if (reset) {
        resetear_estimador();
        auto_fase = AUTO_SWINGUP;
        stab_capturada = false; 
    }
    if(zero && modo == MODO_STANDBY){
        int cb = 0;
        pcnt_unit_get_count(pcnt_barra, &cb);
        cnt_b_ref = cb;
        obs_init = false;      //pb saltó: el observador se reinicia
    }
    if(wm_req){
        wmaxi_mult = wm_val;
    }
    if (ex_stop) {
        exc_act.tipo = EXC_NINGUNA;
    }
    if (ex_start) {
        exc_act              = cfg;
        prbs_lfsr            = 0xACE1u;
        prbs_bit_actual      = 1;
        exc_t_inicio_us      = esp_timer_get_time();
        prbs_t_ultimo_bit_us = exc_t_inicio_us;
    }
}

//lectura de encoders (el PCNT ya devuelve el valor acumulado de 32 bits)
static void leer_encoders(void)
{
    int cb = 0, cd = 0;
    pcnt_unit_get_count(pcnt_barra, &cb);
    pcnt_unit_get_count(pcnt_disco, &cd);
    cnt_b_act = cb;
    cnt_d_act = cd;
    pb = INVERTIR_BARRA * (float)(cb - cnt_b_ref) * RAD_POR_CUENTA;
}

//Error angular respecto a la vertical más cercana 
static float error_vertical(float ang)
{
    float k   = roundf((ang - PI_F) / (2.0f * PI_F));
    float ref = PI_F + 2.0f * PI_F * k;
    return ang - ref; //prácticamente, siempre ang - pi
}

//Swing-up por energía
static float control_energia(void)
{
    /* Discriminante: cuánta energía falta para llegar a Wmaxi */
    float PEND_WMAXI = wmaxi_mult * PEND_WMAXI_BASE;

    float discri = 2.0f * PEND_WMAXI
                 - vb_est * vb_est * PEND_RHO
                 - 2.0f * PEND_Q1 * PEND_RHO * (1.0f - cosf(pb));

    float u;

    if (discri < 0.0f) {
        /* Energía excesiva: amortiguar */
        u = -vb_est;
    } else {
        /* Velocidades objetivo del disco para inyectar energía */
        float raiz = sqrtf(discri);
        float Vel1 = -vb_est + raiz;
        float Vel2 = -vb_est - raiz;
        float Vel  = 0.0f;

        if (vb_est >  PEND_CERO) Vel = Vel2;
        if (vb_est < -PEND_CERO) Vel = Vel1;

        //Realim de estados
        u = PEND_KMOT * (Vel - vd_est) + (PEND_Q2 / PEND_Q3) * vd_est;
    }

    return SWING_SIGN * sat(u, PEND_UMax);
}

//Realimentación de estados (LQR)
static float control_lqr(const ganancias_t *g)
{
    float e = error_vertical(pb);
    if (fabsf(e) > PEND_SALIDA_RAD) {
        //considerar cambiar a modo standby
        return 0.0f;                     //lejos de la vertical
    }

    float u = -(g->l1 * e + g->l2 * vb_est + g->l3 * vd_est);
    return BALANCE_SIGN * sat(u, PEND_UMax);
}

//Atrapar
static float control_auto(const ganancias_t *g)
{
    float e_abs = fabsf(error_vertical(pb));

    if (auto_fase == AUTO_SWINGUP) {
        if (e_abs < PEND_CAPT_RAD) {     //cerca de la vertical?
            auto_fase = AUTO_LQR;
            return control_lqr(g);
        }
        return control_energia();
    }
    //se cayó?
    if (e_abs > PEND_SALIDA_RAD) {
        auto_fase = AUTO_SWINGUP;
        return control_energia();
    }
    return control_lqr(g);
}

//Control por modo
static float calcular_control(modo_t m, const ganancias_t *g)
{
    switch (m) {
    case MODO_MANUAL:
        //Deadman: si la GUI dejó de refrescar la orden, parar
        if ((uint32_t)(ms_now() - t_manual_ms) > MANUAL_DEADMAN_MS) return 0.0f;
        if (manual_izq && !manual_dcha) return  -U_MANUAL * PEND_UMax;
        if (manual_dcha && !manual_izq) return U_MANUAL * PEND_UMax;
        return 0.0f;
    case MODO_OPEN_LOOP:     return exc_generar_u();
    case MODO_STABILIZATION: return control_lqr(g);
    case MODO_AUTO:          return control_auto(g);
    default:                 return 0.0f;   
    }
}

//Filtro promedio movil (n = muestras válidas)
static float promedio_movil(const float *buf, int n)
{
    if (n <= 0) return 0.0f;
    float suma = 0.0f;
    for (int i = 0; i < n; i++) suma += buf[i];
    return suma / n;
}

//Estimador: diferencia centrada sobre cuentas enteras + promedio móvil
static void actualizar_estimador_fd(int32_t cb, int32_t cd)
{
    cb_hist[0] = cb_hist[1];  cb_hist[1] = cb_hist[2];  cb_hist[2] = cb;
    cd_hist[0] = cd_hist[1];  cd_hist[1] = cd_hist[2];  cd_hist[2] = cd;

    if (fd_count < 1000) fd_count++;

    //Necesitamos al menos 3 muestras para diferencia centrada
    //Almenos 2 para diferencia atras
    if (fd_count < DIFERENCIAS) return;

    /* (x[k+1] - x[k-1]) / (2T), con la resta hecha en enteros */
    const float k = RAD_POR_CUENTA / (2.0f * PEND_TS_S);
    float vb_raw = INVERTIR_BARRA * (float)(cb_hist[2] - cb_hist[0]) * k;
    float vd_raw = INVERTIR_DISCO * (float)(cd_hist[2] - cd_hist[0]) * k;
;

    vbarra_hist[fd_idx] = vb_raw;
    vdisco_hist[fd_idx] = vd_raw;
    fd_idx = (fd_idx + 1) % FD_FILTRO_N;
    if (ma_n < FD_FILTRO_N) ma_n++;

    vb_fd = promedio_movil(vbarra_hist, ma_n);
    vd_fd = promedio_movil(vdisco_hist, ma_n);
}

//============================
//Observador de velocidades
//============================
//Estabilidad de un sistema discreto 2x2 (criterio de Jury): ambos polos dentro del círculo unidad
static bool par_estable(float a11, float a12, float a21, float a22)
{
    float tr  = a11 + a22;
    float det = a11 * a22 - a12 * a21;
    return (fabsf(det) < 1.0f) && (1.0f - tr + det > 0.0f) && (1.0f + tr + det > 0.0f);
}

//Dinámica del error de un canal (posición + velocidad): (I - K*H) * F, con K = [alfa; kv]
static bool canal_estable(float alfa, float kv, float f11, float f12, float f21, float f22)
{
    return par_estable((1.0f - alfa) * f11, (1.0f - alfa) * f12,
                       -kv * f11 + f21,     -kv * f12 + f22);
}

//Valida las ganancias: error de la barra estable colgando y en la vertical, y error del disco estable
static bool obs_validar(const obs_gains_t *g)
{
    const float Ts = PEND_TS_S;
    const float gv[4] = { g->lb1, g->lb2, g->ld1, g->ld2 };
    for (int i = 0; i < 4; i++)
        if (!(gv[i] > 0.0f && gv[i] <= 1e5f)) return false;     //también rechaza NaN

    const float ab = g->lb1 * Ts, kb = g->lb2 * Ts;
    const float ad = g->ld1 * Ts, kd = g->ld2 * Ts;
    if (ab >= 2.0f || ad >= 2.0f) return false;

    for (int s = -1; s <= 1; s += 2) {          //s=-1: colgando (c=-q1), s=+1: vertical (c=+q1)
        float c = (float)s * PEND_Q1;
        if (!canal_estable(ab, kb, 1.0f + 0.5f * Ts * Ts * c, Ts, Ts * c, 1.0f)) return false;
    }
    const float gd = PEND_RHO * PEND_Q2;        //decaimiento del disco por fricción/contra-fem
    return canal_estable(ad, kd, 1.0f, Ts - 0.5f * Ts * Ts * gd, 0.0f, 1.0f - gd * Ts);
}

//Observador de Luenberger discreto (predicción con el modelo + corrección con ambos encoders):
//  barra:  th'' = -q1*sin(th) - q3*u + q2*w
//  disco:  w'   = rho*(q3*u - q2*w) - th''        (w = velocidad del disco relativa a la barra)
//Las ganancias son continuas: pos += lX1*Ts*r ;  vel += lX2*Ts*r   (r = innovación)
static void actualizar_observador(const obs_gains_t *og)
{
    const float Ts = PEND_TS_S;

    if (!obs_init) {
        obs_cb_prev = cnt_b_act;  obs_cd_prev = cnt_d_act;  obs_pb_prev = pb;
        obs_db = obs_dd = obs_wb = obs_wd = 0.0f;
        vb_obs = vd_obs = 0.0f;
        obs_init = true;
        return;
    }

    //Variación medida desde la muestra anterior (resta en enteros: exacta)
    const float dyb = INVERTIR_BARRA * (float)(cnt_b_act - obs_cb_prev) * RAD_POR_CUENTA;
    const float dyd = INVERTIR_DISCO * (float)(cnt_d_act - obs_cd_prev) * RAD_POR_CUENTA;

    //Modelo evaluado en el estado estimado anterior, con la u realmente aplicada
    const float u  = u_aplicada;
    const float th = obs_pb_prev + obs_db;
    const float ab = -PEND_Q1 * sinf(th) - PEND_Q3 * u + PEND_Q2 * obs_wd;
    const float ad = PEND_RHO * (PEND_Q3 * u - PEND_Q2 * obs_wd) - ab;

    //Predicción (desplazamiento respecto a la última medición y velocidad)
    const float db_p = obs_db + Ts * obs_wb + 0.5f * Ts * Ts * ab;
    const float wb_p = obs_wb + Ts * ab;
    const float dd_p = obs_dd + Ts * obs_wd + 0.5f * Ts * Ts * ad;
    const float wd_p = obs_wd + Ts * ad;

    //Innovación = medición actual - predicción
    const float rb = dyb - db_p;
    const float rd = dyd - dd_p;

    //Corrección:  x_hat = x_pred + K*r ;  delta = x_hat - y = -(1 - alfa)*r
    obs_db = -(1.0f - og->lb1 * Ts) * rb;
    obs_wb = wb_p + og->lb2 * Ts * rb;
    obs_dd = -(1.0f - og->ld1 * Ts) * rd;
    obs_wd = wd_p + og->ld2 * Ts * rd;

    obs_cb_prev = cnt_b_act;  obs_cd_prev = cnt_d_act;  obs_pb_prev = pb;

    //Protección: si algo diverge, se reinicia en vez de propagar basura al control
    if (!isfinite(obs_wb) || !isfinite(obs_wd) || fabsf(obs_wb) > 1e3f || fabsf(obs_wd) > 1e4f) {
        obs_init = false;
        vb_obs = vd_obs = 0.0f;
        return;
    }
    vb_obs = obs_wb;
    vd_obs = obs_wd;
}

//============================
//Comandos por UART
//============================
static void cmd_excitacion(const exc_cfg_t *cfg, const char *ack)
{
    if (!handshake_ok || modo != MODO_OPEN_LOOP) {
        responder("NACK,NOT_OPEN_LOOP\n");
        return;
    }
    if (!exc_cfg_valida(cfg)) {
        responder("NACK,BAD_VALUE\n");
        return;
    }
    portENTER_CRITICAL(&estado_mux);
    exc_pend      = *cfg;
    exc_req_start = true;
    exc_req_stop  = false;
    portEXIT_CRITICAL(&estado_mux);
    responder(ack);
}

static void procesar_linea(const char *l)
{
    if (strcmp(l, "PING") == 0) {
        //Heartbeat, solo refresca t_ultimo_comando_ms
        //tal vez quitarlo, ya se hace en recepción
    }
    else if (strcmp(l, "READY") == 0) {
        if (!handshake_ok) {
            handshake_ok = true;
            ir_a_standby();
        }
        enviar_estado();
    }
    else if (strcmp(l, "HELLO?") == 0) {
        handshake_ok = false;
        ir_a_standby();
        motor_set(0.0f, 0);
    }
    else if (strncmp(l, "SQUARE:", 7) == 0) {
        exc_cfg_t c = { .tipo = EXC_SQUARE };
        if (sscanf(l + 7, "%f,%f,%f", &c.freq, &c.amp_pct, &c.dur) == 3)
            cmd_excitacion(&c, "ACK,EXC,SQUARE\n");
        else
            responder("NACK,BAD_FORMAT\n");
    }
    else if (strncmp(l, "CHIRP:", 6) == 0) {
        exc_cfg_t c = { .tipo = EXC_CHIRP };
        if (sscanf(l + 6, "%f,%f,%f,%f", &c.f0, &c.f1, &c.dur, &c.amp_pct) == 4)
            cmd_excitacion(&c, "ACK,EXC,CHIRP\n");
        else
            responder("NACK,BAD_FORMAT\n");
    }
    else if (strncmp(l, "PRBS:", 5) == 0) {
        exc_cfg_t c = { .tipo = EXC_PRBS };
        if (sscanf(l + 5, "%f,%f,%f", &c.bitrate, &c.amp_pct, &c.dur) == 3)
            cmd_excitacion(&c, "ACK,EXC,PRBS\n");
        else
            responder("NACK,BAD_FORMAT\n");
    }
    else if (strcmp(l, "EXC_STOP") == 0) {
        portENTER_CRITICAL(&estado_mux);
        exc_req_start = false;
        exc_req_stop  = true;
        portEXIT_CRITICAL(&estado_mux);
        responder("ACK,EXC,STOP\n");
    }
    else if (strncmp(l, "MODE:", 5) == 0) {
        if (!handshake_ok) {
            responder("NACK,NO_HANDSHAKE\n");
            return;
        }
        modo_t nuevo = MODO_STANDBY;
        bool ok = false;
        for (int i = 0; i < MODO_N; i++) {
            if (strcmp(l + 5, MODO_NOMBRE[i]) == 0) { nuevo = (modo_t)i; ok = true; break; }
        }
        if (!ok) {
            responder("NACK,BAD_MODE\n");
            return;
        }
        portENTER_CRITICAL(&estado_mux);
        modo          = nuevo;
        manual_izq    = false;
        manual_dcha    = false;
        req_reset     = true;          //estimador + fase AUTO */
        exc_req_start = false;
        exc_req_stop  = true;          //cambiar de modo corta cualquier excitación
        portEXIT_CRITICAL(&estado_mux);
        enviar_estado();
    }
    else if (strncmp(l, "L:", 2) == 0) {
        if (!handshake_ok) {
            responder("NACK,NO_HANDSHAKE\n");
        } else if (modo != MODO_STANDBY) {
            responder("NACK,MOTOR_RUNNING\n");
        } else {
            ganancias_t g;
            if (sscanf(l + 2, "%f,%f,%f", &g.l1, &g.l2, &g.l3) != 3) {
                responder("NACK,BAD_FORMAT\n");
            } else if (!isfinite(g.l1) || !isfinite(g.l2) || !isfinite(g.l3) ||
                       fabsf(g.l1) > 1e6f || fabsf(g.l2) > 1e6f || fabsf(g.l3) > 1e6f) {
                responder("NACK,BAD_VALUE\n");
            } else {
                portENTER_CRITICAL(&estado_mux);
                ganancias = g;
                portEXIT_CRITICAL(&estado_mux);
                char ack[64];
                int n = snprintf(ack, sizeof(ack), "ACK,L=%.4f,%.4f,%.4f\n", g.l1, g.l2, g.l3);
                if (n > (int)sizeof(ack) - 1) n = (int)sizeof(ack) - 1;
                if (n > 0) uart_write_bytes(UART_NUM, ack, n);
            }
        }
    }
    else if (strncmp(l, "EST:", 4) == 0) {
        if (!handshake_ok) {
            responder("NACK,NO_HANDSHAKE\n");
        } else if (modo != MODO_STANDBY) {
            responder("NACK,MOTOR_RUNNING\n");
        } else {
            int sel = -1;
            if      (strcmp(l + 4, "DIF") == 0) sel = EST_DIF;
            else if (strcmp(l + 4, "OBS") == 0) sel = EST_OBS;
            if (sel < 0) {
                responder("NACK,BAD_VALUE\n");
            } else {
                portENTER_CRITICAL(&estado_mux);
                est_modo  = (est_modo_t)sel;
                req_reset = true;          //estimadores y fase AUTO desde cero
                portEXIT_CRITICAL(&estado_mux);
                char ack[24];
                int n = snprintf(ack, sizeof(ack), "ACK,EST,%s\n", EST_NOMBRE[sel]);
                if (n > 0) uart_write_bytes(UART_NUM, ack, n);
            }
        }
    }
    else if (strncmp(l, "OBS:", 4) == 0) {
        if (!handshake_ok) {
            responder("NACK,NO_HANDSHAKE\n");
        } else if (modo != MODO_STANDBY) {
            responder("NACK,MOTOR_RUNNING\n");
        } else {
            obs_gains_t g;
            if (sscanf(l + 4, "%f,%f,%f,%f", &g.lb1, &g.lb2, &g.ld1, &g.ld2) != 4) {
                responder("NACK,BAD_FORMAT\n");
            } else if (!obs_validar(&g)) {
                responder("NACK,OBS_UNSTABLE\n");   //fuera de rango o polos discretos inestables
            } else {
                portENTER_CRITICAL(&estado_mux);
                obs_g     = g;
                req_reset = true;
                portEXIT_CRITICAL(&estado_mux);
                char ack[64];
                int n = snprintf(ack, sizeof(ack), "ACK,OBS=%.3f,%.3f,%.3f,%.3f\n",
                                 g.lb1, g.lb2, g.ld1, g.ld2);
                if (n > (int)sizeof(ack) - 1) n = (int)sizeof(ack) - 1;
                if (n > 0) uart_write_bytes(UART_NUM, ack, n);
            }
        }
    }
    else if (strcmp(l, "ESTOP") == 0) {
        ir_a_standby();
        motor_set(0.0f, 0);
        if (handshake_ok) enviar_estado();
    }
    else if (strcmp(l, "ZERO") == 0) {
        if (!handshake_ok) {
            responder("NACK,NO_HANDSHAKE\n");
        } else if (modo != MODO_STANDBY) {
            responder("NACK,NOT_STANDBY\n");
        } else {
            portENTER_CRITICAL(&estado_mux);
            req_zero = true;
            portEXIT_CRITICAL(&estado_mux);
            responder("ACK,ZERO\n");            
        }
    }
    else if (strncmp(l, "WMAXI:", 6) == 0) {
        if (!handshake_ok) {
            responder("NACK,NO_HANDSHAKE\n");
        } else {
            float m_val;
            if (sscanf(l + 6, "%f", &m_val) != 1 ||
                !isfinite(m_val) || m_val < 0.1f || m_val > 20.0f) {
                responder("NACK,BAD_VALUE\n");
            } else {
                //Enter critical para que no se cambie wmaxi_pend_mult mientras la tarea de control lo lee
                portENTER_CRITICAL(&estado_mux);
                wmaxi_pend_mult = m_val;
                req_wmaxi  = true;
                portEXIT_CRITICAL(&estado_mux);
                char ack[32];
                int n = snprintf(ack, sizeof(ack), "ACK,WMAXI,%.3f\n", m_val);
                if (n > 0) uart_write_bytes(UART_NUM, ack, n);
            }
        }
    }
    else if (strncmp(l, "MANUAL:", 7) == 0) {
        if (handshake_ok && modo == MODO_MANUAL) {
            const char *d = l + 7;
            if      (strcmp(d, "LEFT")  == 0) { manual_izq = true;  manual_dcha = false; }
            else if (strcmp(d, "RIGHT") == 0) { manual_izq = false; manual_dcha = true;  }
            else if (strcmp(d, "STOP")  == 0) { manual_izq = false; manual_dcha = false; }
            t_manual_ms = ms_now();    
        }
    }
    
}

//============================
//Tareas
//============================
//Rx
static void tarea_recepcion(void *param)
{
    char    rx_buffer[128];
    int     rx_idx = 0;
    bool    descartando = false;
    uint8_t byte_rx;

    while (1) {
        int len = uart_read_bytes(UART_NUM, &byte_rx, 1, pdMS_TO_TICKS(100));
        if (len <= 0) continue;

        t_ultimo_comando_ms = ms_now();

        if (byte_rx == '\n' || byte_rx == '\r') {
            if (!descartando && rx_idx > 0) {
                rx_buffer[rx_idx] = '\0';
                procesar_linea(rx_buffer);
            }
            rx_idx = 0;
            descartando = false;
        }
        else if (rx_idx < (int)sizeof(rx_buffer) - 1) {
            rx_buffer[rx_idx++] = (char)byte_rx;
        }
        else {
            descartando = true;          //línea demasiado larga
        }
    }
}

//Control y Tx
static void tarea_control(void *param)
{
    const TickType_t periodo  = pdMS_TO_TICKS(PEND_TS_MS);
    TickType_t last_wake      = xTaskGetTickCount();
    int64_t    t_prev_us      = esp_timer_get_time();
    paquete_t  pkt;
    pkt.h1 = 0xAA;
    pkt.h2 = 0xBB;
    bool sobrevel_latch = false;

    while (1) {
        vTaskDelayUntil(&last_wake, periodo);

        int64_t t_now_us = esp_timer_get_time();
        float dt = (t_now_us - t_prev_us) / 1e6f;
        t_prev_us = t_now_us;
        if (dt <= 0.0f || dt > 0.5f) dt = PEND_TS_S;

        aplicar_comandos_pendientes();
        leer_encoders();

        modo_t      m;
        ganancias_t g;
        est_modo_t  em;
        obs_gains_t og;
        portENTER_CRITICAL(&estado_mux);
        m  = modo;
        g  = ganancias;
        em = est_modo;
        og = obs_g;
        portEXIT_CRITICAL(&estado_mux);

        //Los dos estimadores corren siempre (cambiar desde la GUI no produce saltos);
        //el control y la telemetría usan el elegido.
        actualizar_estimador_fd(cnt_b_act, cnt_d_act);
        actualizar_observador(&og);
        if (em == EST_OBS) { vb_est = vb_obs; vd_est = vd_obs; }
        else               { vb_est = vb_fd;  vd_est = vd_fd;  }

        //Atrapar la barra en manual
        if (m==MODO_MANUAL){
            float e_abs = fabsf(error_vertical(pb));
            //Considerar PEND_CAPT_RAD * 2 para que se más fácil en manual?
            if(e_abs<PEND_CAPT_RAD){
                //Solo se cambia si el modo SIGUE siendo MANUAL: si justo llegó un ESTOP o un
                //MODE: por la UART, no hay que pisarlo.
                bool cambio = false;
                portENTER_CRITICAL(&estado_mux);
                if (modo == MODO_MANUAL) {
                    modo = MODO_STABILIZATION;
                    manual_izq = false;
                    manual_dcha = false;
                    cambio = true;
                }
                m = modo;
                portEXIT_CRITICAL(&estado_mux);
                if (cambio) enviar_estado();
            }
        }

       //Si se cae la barra en manual
        if (m == MODO_STABILIZATION) {
            float e_abs = fabsf(error_vertical(pb));

            if (e_abs < PEND_CAPT_RAD) {
                stab_capturada = true;
            }
            //Solo pasa a standby si hubo captura antes
            
            if (stab_capturada && e_abs > PEND_SALIDA_RAD) {
                bool cambio = false;
                portENTER_CRITICAL(&estado_mux);
                if (modo == MODO_STABILIZATION) {
                    modo        = MODO_STANDBY;
                    manual_izq  = false;
                    manual_dcha = false;
                    req_reset   = true;
                    cambio = true;
                }
                m = modo;
                portEXIT_CRITICAL(&estado_mux);
                if (cambio) enviar_estado();
            }
        }

        //Control según modo
        float u;
        float rpm_disco = vd_est * 60.0f / (2.0f * PI_F);
#if SOBREVEL_ACTIVO
        //histéresis: frena al superar RPM_MAX y suelta al bajar del 90 %
        if (fabsf(rpm_disco) > RPM_MAX)             sobrevel_latch = true;
        else if (fabsf(rpm_disco) < 0.9f * RPM_MAX) sobrevel_latch = false;
#endif
        bool  sobrevel  = sobrevel_latch;

        if (sobrevel) {
            //Protección: frenar el disco (contra-torque máximo)
            u = (rpm_disco > 0.0f) ? -PEND_UMax : PEND_UMax;
        } else if (!handshake_ok || m == MODO_STANDBY) {
            u = 0.0f;
        } else {
            u = calcular_control(m, &g);
        }

        if (!isfinite(u)) u = 0.0f;
        u = sat(u, PEND_UMax);

        //Rechequeo 
        if (!sobrevel && (modo == MODO_STANDBY || !handshake_ok)) u = 0.0f;

        bool balanceando = (m == MODO_STABILIZATION) ||
                           (m == MODO_AUTO && auto_fase == AUTO_LQR);
        u_aplicada = motor_set(u, balanceando ? PWM_MIN_BALANCE : PWM_MIN_SWING);

        //telemetría siempre
        pkt.dt          = dt;
        pkt.theta       = pb;
        pkt.theta_dot   = vb_est;  
        pkt.omega_rueda = vd_est; 
        pkt.u           = (int16_t)(u * 100.0f);

        uart_write_bytes(UART_NUM, (const char *)&pkt, sizeof(pkt));
    }
}

//Handshake + watchdog de comunicación
static void tarea_hello(void *param)
{
    vTaskDelay(pdMS_TO_TICKS(300));
    uint32_t t_ult_hello = 0;

    while (1) {
        uint32_t ahora = ms_now();

        if (handshake_ok) {
            //Con el motor activo el límite es más corto: si la GUI se cuelga o se suelta el
            //cable en pleno AUTO, el sistema no debe seguir actuando 5 s.
            uint32_t limite = (modo != MODO_STANDBY) ? TIMEOUT_MOTOR_MS : TIMEOUT_HANDSHAKE_MS;
            if ((uint32_t)(ahora - t_ultimo_comando_ms) > limite) {
                handshake_ok = false;
                ir_a_standby();
                motor_set(0.0f, 0);
            }
        }
        if (!handshake_ok && (uint32_t)(ahora - t_ult_hello) >= PING_HELLO_MS) {
            t_ult_hello = ahora;
            responder("HELLO,v2.3,PENDULO_ESP32\n");
        }
        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_PERIODO_MS));
    }
}

void app_main(void)
{
    //Desactivar logs de ESP-IDF
    esp_log_level_set("*", ESP_LOG_NONE);//para que no se mezclen con la telemetría
    configurar_uart();
    configurar_pwm();
    configurar_encoder(ENC_BARRA_A, ENC_BARRA_B, &pcnt_barra);
    configurar_encoder(ENC_DISCO_A, ENC_DISCO_B, &pcnt_disco);

    t_ultimo_comando_ms = ms_now();

    xTaskCreatePinnedToCore(tarea_recepcion, "recepcion", 6144, NULL, 5, NULL, CORE_COMM);
    xTaskCreatePinnedToCore(tarea_control,   "control",   8192, NULL, 6, NULL, CORE_CTRL);
    xTaskCreatePinnedToCore(tarea_hello,     "hello",     4096, NULL, 3, NULL, CORE_COMM);
}
