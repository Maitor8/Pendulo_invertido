#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"

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
#define PEND_TS_MS       10 //100Hz
#define PEND_TS_S        (PEND_TS_MS / 1000.0f)
#define PEND_UMax        60.0f
#define PEND_ZONA2       (988.0f * (float)M_PI / 1000.0f) //pocos grados para la captura
//PWMs
#define PWM_FREQ_HZ      20000
#define PWM_RES          LEDC_TIMER_10_BIT
#define PWM_MAX          1023

//Encoders
#define ENC_LIMIT        30000
#define ENC_FULL_RANGE   (2 * ENC_LIMIT)
#define ENC_WRAP_THR     (ENC_FULL_RANGE - 1000)
#define PEND_CPR         2000.0f   
#define Filter_ns        100 //el otro día era de 50, mucho ruido subirlo

//Uart
#define UART_NUM              UART_NUM_0
#define UART_BUF_SIZE         2048
#define TIMEOUT_HANDSHAKE_MS  5000
#define PING_HELLO_MS         1000

//Ctes de energía para el swing-up
#define PEND_Q1          30.0f//param q1
#define PEND_WMAXI       15390.0f//depende de los param, =2*q1*rho + margen
#define PEND_KMOT        1.5f//considerar bajarlo, = 1/(rho * q3 * tau), tau=0.07?
#define PEND_RHO         250.0f //param rho
#define PEND_Q2          0.0245f//q2
#define PEND_Q3          0.0393f//q3
#define PEND_CERO        2.5f//tal vez modificarlo según fricción, swing up

// #define PEND_WMAXI  (2.1f * PEND_Q1 * PEND_RHO)              /* +5% margen */
// #define PEND_KMOT   (1.0f / (PEND_RHO * PEND_Q3 * 0.08f))    /* τ = 80 ms */
// #define PEND_CERO   (0.4f * sqrtf(PEND_Q1))

//Motor
#define PWM_MIN_SWING    0
#define PWM_MIN_BALANCE  0
#define RPM_MAX 3000

//Inversiones, encoder y motor
#define INVERTIR_BARRA   1.0f
#define INVERTIR_DISCO   1.0f
#define SWING_SIGN    1.0f
#define BALANCE_SIGN  1.0f


//============================
//Tipos
//============================
//Ensayo de lazo abierto
typedef enum {
    EXC_NINGUNA = 0,
    EXC_SQUARE,
    EXC_CHIRP,
    EXC_PRBS,
} exc_tipo_t;

//Paquete binario de las señales
#pragma pack(push, 1)
typedef struct {
    uint8_t h1;             //h1
    uint8_t h2;             //h2
    float   dt;             //periodo
    float   theta;          //pos barra rad
    float   theta_dot;      //vel barra rad/s
    float   omega_rueda;    //vel disco rad/s
    int16_t u;              //u en función del umax no en volt
} paquete_t;
#pragma pack(pop)

//Balanceo
typedef enum {
    AUTO_SWINGUP = 0,
    AUTO_LQR,
} auto_fase_t;

//Encoders
typedef struct{
    pcnt_unit_handle_t unit;
    volatile int64_t;
} encoder_pcnt_t ;


//============================
//ESTADO GLOBAL
//============================
//balance
static volatile auto_fase_t auto_fase = AUTO_SWINGUP;

//Handshake
static volatile bool    handshake_ok    = false;
static char             modo_actual[16] = "STANDBY";
static volatile int32_t t_ultimo_comando_ms = 0;

//Ganancias
static volatile float L1 = -4941.4f;
static volatile float L2 = -985.4f;
static volatile float L3 = -1.4f;

//Hardware
static pcnt_unit_handle_t pcnt_barra = NULL;
static pcnt_unit_handle_t pcnt_disco = NULL;

//Pos fís
static volatile float pb     = 0.0f;
static volatile float pd     = 0.0f;
static volatile float pbo    = 0.0f;   //offset, z=x1-x0, alrededor del eq
static volatile int32_t pdma = 0;
static volatile int32_t roscas = 1;

//Posiciones para las diferencias
static float pb_hist[3] = {0};   /* [k-1], [k], [k+1] */
static float pd_hist[3] = {0};

//Para el promedio
static float vbarra_hist[FD_FILTRO_N] = {0};
static float vdisco_hist[FD_FILTRO_N] = {0};
static int   fd_idx = 0;
static int   fd_count = 0;

//Vel filtradas
static volatile float vb_est = 0.0f;   /* velocidad barra */
static volatile float vd_est = 0.0f;   /* velocidad disco */

//control
static volatile float u_ctrl = 0.0f;

//Lazo abierto
static volatile exc_tipo_t exc_tipo     = EXC_NINGUNA;
static volatile float      exc_amp_pct  = 0.0f;
static volatile float      exc_freq     = 0.0f;
static volatile float      exc_f0       = 0.0f;
static volatile float      exc_f1       = 0.0f;
static volatile float      exc_duracion = 0.0f;
static volatile float      exc_bitrate  = 0.0f;
static volatile int64_t    exc_t_inicio_us = 0;

/* PRBS (LFSR 16 bits) */
static uint32_t prbs_lfsr           = 0xACE1u;
static int64_t  prbs_t_ultimo_bit_us = 0;
static int      prbs_bit_actual     = 1;

//Control Manual
static volatile bool manual_izq = false;
static volatile bool manual_der = false;

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
    uart_param_config(UART_NUM, &cfg);
    uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
}

//Pwm del puente/motor
static void configurar_pwm(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);
    //ch0 para dcha
    ledc_channel_config_t ch_r = {
        .gpio_num = RPWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel  = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0,
        .duty = 0, .hpoint = 0,
    };
    ledc_channel_config(&ch_r);
    //ch1 para izq
    ledc_channel_config_t ch_l = {
        .gpio_num = LPWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel  = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0,
        .duty = 0, .hpoint = 0,
    };
    ledc_channel_config(&ch_l);

    //puente H
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_EN),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(PIN_EN, 1);
}

//encoders
static void configurar_encoder(int pin_a, int pin_b, pcnt_unit_handle_t *handle)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit =  ENC_LIMIT,
        .low_limit  = -ENC_LIMIT,
    };
    pcnt_new_unit(&unit_cfg, handle);

    //Config en x4
    /* ---- Canal A: cuenta sobre A, dirección por B ---- */
    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num  = pin_a,
        .level_gpio_num = pin_b,
    };
    pcnt_channel_handle_t chan_a = NULL;
    pcnt_new_channel(*handle, &chan_a_cfg, &chan_a);
    pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,  
        PCNT_CHANNEL_EDGE_ACTION_INCREASE); 
    pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,    
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);  

    /* ---- Canal B: cuenta sobre B, dirección por A ---- */
    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num  = pin_b,
        .level_gpio_num = pin_a,
    };
    pcnt_channel_handle_t chan_b = NULL;
    pcnt_new_channel(*handle, &chan_b_cfg, &chan_b);
    pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,  
        PCNT_CHANNEL_EDGE_ACTION_DECREASE); 
    pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* Filtro anti-rebote */
    pcnt_glitch_filter_config_t filter = { .max_glitch_ns = Filter_ns};
    pcnt_unit_set_glitch_filter(*handle, &filter);

    pcnt_unit_enable(*handle);
    pcnt_unit_clear_count(*handle);
    pcnt_unit_start(*handle);
}

//============================
//Funciones varias
//============================
//Aplicar u
static void motor_set(float u)
{
    if (u >  PEND_UMax) u =  PEND_UMax;
    if (u < -PEND_UMax) u = -PEND_UMax;

    int duty = (int)(fabsf(u) / PEND_UMax * PWM_MAX);

    //solo si hay sona muerta
    // if (duty>0 && duty <PWM_MIN_SWING) duty= PWM_MIN_SWING;

    if (u > 0.5f) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    } else if (u < -0.5f) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

//send state
static void enviar_estado(void)
{
    char buf[128];
    int n = snprintf(buf, sizeof(buf),
                     "STATE,L=%.4f,%.4f,%.4f;MODE=%s\n",
                     L1, L2, L3, modo_actual);
    uart_write_bytes(UART_NUM, buf, n);
}

//Excitación de Lazo abierto
static int prbs_next_bit(void)
{
    uint32_t bit = ((prbs_lfsr >> 0) ^ (prbs_lfsr >> 2) ^
                    (prbs_lfsr >> 3) ^ (prbs_lfsr >> 5)) & 1u;
    prbs_lfsr = (prbs_lfsr >> 1) | (bit << 15);
    return prbs_lfsr & 1u;
}

static float exc_generar_u(void)
{
    if (exc_tipo == EXC_NINGUNA) return 0.0f;

    int64_t ahora = esp_timer_get_time();
    float t = (ahora - exc_t_inicio_us) / 1e6f;

    if (t >= exc_duracion) {
        exc_tipo = EXC_NINGUNA;
        return 0.0f;
    }

    float amp = (exc_amp_pct / 100.0f) * PEND_UMax;

    switch (exc_tipo) {
    case EXC_SQUARE: {
        float fase = 2.0f * (float)M_PI * exc_freq * t;
        return (sinf(fase) >= 0.0f) ? amp : -amp;
    }
    case EXC_CHIRP: {
        if (exc_f1 == exc_f0) exc_f1 += 0.01f;   /* protección */
        float K    = exc_duracion / logf(exc_f1 / exc_f0);
        float fase = 2.0f * (float)M_PI * exc_f0 * K * (expf(t / K) - 1.0f);
        return amp * sinf(fase);
    }
    case EXC_PRBS: {
        int64_t periodo_us = (int64_t)(1e6f / exc_bitrate);
        if ((ahora - prbs_t_ultimo_bit_us) >= periodo_us) {
            prbs_bit_actual     = prbs_next_bit();
            prbs_t_ultimo_bit_us = ahora;
        }
        return prbs_bit_actual ? amp : -amp;
    }
    default:
        return 0.0f;
    }
}

static void exc_detener(void)
{
    exc_tipo = EXC_NINGUNA;
}

static bool exc_iniciar(exc_tipo_t tipo)
{
    if (strncmp(modo_actual, "OPEN_LOOP", 9) != 0) return false;
    exc_detener();
    prbs_lfsr            = 0xACE1u;
    prbs_bit_actual      = 1;
    prbs_t_ultimo_bit_us = esp_timer_get_time();
    exc_t_inicio_us      = esp_timer_get_time();
    exc_tipo             = tipo;
    return true;
}

//Para resetear el observador trucho
static void resetear_estimador(void)
{
    pb_hist[0] = pb_hist[1] = pb_hist[2] = pb;
    pd_hist[0] = pd_hist[1] = pd_hist[2] = pd;
    for (int i = 0; i < FD_FILTRO_N; i++) {
        vbarra_hist[i] = 0.0f;
        vdisco_hist[i] = 0.0f;
    }
    fd_idx = 0;
    fd_count = 0;
    vb_est = 0.0f;
    vd_est = 0.0f;
}

//lectura de encoders
static void leer_encoders(void)
{
    int cnt_barra, cnt_disco;
    pcnt_unit_get_count(pcnt_barra, &cnt_barra);
    pcnt_unit_get_count(pcnt_disco, &cnt_disco);

    //  Unwrap del disco 
    int32_t difpd = cnt_disco - pdma;
    if (difpd >  ENC_WRAP_THR) roscas++;
    if (difpd < -ENC_WRAP_THR) roscas--;
    pdma = cnt_disco;

    int32_t abs_disco = cnt_disco - (roscas - 1) * ENC_FULL_RANGE;

    pd = INVERTIR_DISCO * (float)abs_disco * (float)M_PI / (PEND_CPR / 2.0f);
    pb = INVERTIR_BARRA * (float)cnt_barra * (float)M_PI / (PEND_CPR / 2.0f);
}

//Swing-up por energía
static float control_energia(void)
{
    /* Discriminante: cuánta energía falta para llegar a Wmaxi */
    float discri = 2.0f * PEND_WMAXI
                 - vb_est * vb_est * PEND_RHO
                 - 2.0f * PEND_Q1 * PEND_RHO * (1.0f - cosf(pb));

    float u;

    if (discri < 0.0f) {
        /* Energía excesiva: amortiguar */
        u = -vb_est;
    } else {
        /* Velocidades objetivo del disco para inyectar energía */
        float Vel1 = -vb_est + sqrtf(discri);
        float Vel2 = -vb_est - sqrtf(discri);
        float Vel  = 0.0f;

        if (vb_est >  PEND_CERO) Vel = Vel2;
        if (vb_est < -PEND_CERO) Vel = Vel1;

        /* Ley de control: proporcional a la velocidad del disco
           + feedforward de fricción */
        u = PEND_KMOT * (Vel - vd_est) + (PEND_Q2 / PEND_Q3) * vd_est;
    }

    /* Saturación */
    if (u >  PEND_UMax) u =  PEND_UMax;
    if (u < -PEND_UMax) u = -PEND_UMax;

    return SWING_SIGN * u;
}

//Realim- Ganancias viejas
static float control_lqr(void)
{
    if (fabsf(pb) < PEND_ZONA2 * 0.9f) {
        return 0.0f;
    }
    
    float u = L1 * (pb + pbo) + L2 * vb_est + L3 * vd_est;
    u = -u;

    if (u >  PEND_UMax) u =  PEND_UMax;
    if (u < -PEND_UMax) u = -PEND_UMax;

    return BALANCE_SIGN * u;
}

//Atrapar
static float control_auto(void)
{
    float pb_abs = fabsf(pb);

    if (auto_fase == AUTO_SWINGUP) {
        /* ¿Llegó cerca de vertical? */
        if (pb_abs > PEND_ZONA2) {
            auto_fase = AUTO_LQR;
            /* pbo = referencia para el LQR: -π si llegó por arriba,
                                     +π si llegó por abajo */
            pbo = (pb > 0.0f) ? -M_PI : M_PI;
        }
        return control_energia();
    } else {
        /* Está en LQR: verificar si se cayó */
        if (pb_abs < PEND_ZONA2 * 0.9f) {   
            auto_fase = AUTO_SWINGUP;
            pbo = 0.0f;
        }
        return control_lqr();
    }
}

//Control por modo
static float calcular_control(void)
{
    if (strncmp(modo_actual, "MANUAL", 6) == 0) {
        if (manual_izq && !manual_der) return  0.3f * PEND_UMax;
        if (manual_der && !manual_izq) return -0.3f * PEND_UMax;
        return 0.0f;
    }
    if (strncmp(modo_actual, "OPEN_LOOP", 9) == 0) {
        return exc_generar_u();
    }
    if (strncmp(modo_actual, "STABILIZATION", 13) == 0) {
        return control_lqr();
    }
    if (strncmp(modo_actual, "AUTO", 4) == 0) {
        /* TODO Fase 2/3: swing-up + LQR */
        return control_auto();
    }
    /* STANDBY */
    return 0.0f;
}

//Filtro promedio movil
static float promedio_movil(const float *buf, int n)
{
    float suma = 0.0f;
    for (int i = 0; i < n; i++) suma += buf[i];
    return suma / n;
}

//Estimador
static void actualizar_estimador_fd(float pb_nuevo, float pd_nuevo)
{
    /* Correr el historial */
    pb_hist[0] = pb_hist[1];
    pb_hist[1] = pb_hist[2];
    pb_hist[2] = pb_nuevo;

    pd_hist[0] = pd_hist[1];
    pd_hist[1] = pd_hist[2];
    pd_hist[2] = pd_nuevo;

    fd_count++;

    /* Necesitamos al menos 3 muestras para diferencia centrada */
    if (fd_count < 3) return;

    /* Diferencia centrada: (x[k+1] - x[k-1]) / (2T) */
    float vb_raw = (pb_hist[2] - pb_hist[0]) / (2.0f * PEND_TS_S);
    float vd_raw = (pd_hist[2] - pd_hist[0]) / (2.0f * PEND_TS_S);

    /* Actualizar buffer del promedio móvil */
    vbarra_hist[fd_idx] = vb_raw;
    vdisco_hist[fd_idx] = vd_raw;
    fd_idx = (fd_idx + 1) % FD_FILTRO_N;

    /* Salida filtrada */
    vb_est = promedio_movil(vbarra_hist, FD_FILTRO_N);
    vd_est = promedio_movil(vdisco_hist, FD_FILTRO_N);
}


//============================
//Tareas
//============================
//Rx
static void tarea_recepcion(void *param)
{
    uint8_t rx_buffer[256];
    int     rx_idx = 0;
    uint8_t byte_rx;

    while (1) {
        int len = uart_read_bytes(UART_NUM, &byte_rx, 1, pdMS_TO_TICKS(100));
        if (len <= 0) continue;

        t_ultimo_comando_ms = (int32_t)(esp_timer_get_time() / 1000);

        if (byte_rx == '\n' || byte_rx == '\r') {
            rx_buffer[rx_idx] = '\0';

            if (strncmp((char*)rx_buffer, "PING", 4) == 0) {
                /* Heartbeat, no hace nada */
            }
            else if (strncmp((char*)rx_buffer, "READY", 5) == 0) {
                handshake_ok = true;
                exc_detener();
                u_ctrl = 0.0f;
                resetear_estimador();
                enviar_estado();
            }
            else if (strncmp((char*)rx_buffer, "HELLO?", 6) == 0) {
                handshake_ok = false;
                exc_detener();
            }
            else if (strncmp((char*)rx_buffer, "SQUARE:", 7) == 0) {
                float f, a, T;
                if (sscanf((char*)rx_buffer + 7, "%f,%f,%f", &f, &a, &T) == 3) {
                    exc_freq = f; exc_amp_pct = a; exc_duracion = T;
                    if (exc_iniciar(EXC_SQUARE))
                        uart_write_bytes(UART_NUM, "ACK,EXC,SQUARE\n", 15);
                    else
                        uart_write_bytes(UART_NUM, "NACK,NOT_OPEN_LOOP\n", 19);
                } else {
                    uart_write_bytes(UART_NUM, "NACK,BAD_FORMAT\n", 16);
                }
            }
            else if (strncmp((char*)rx_buffer, "CHIRP:", 6) == 0) {
                float f0, f1, T, a;
                if (sscanf((char*)rx_buffer + 6, "%f,%f,%f,%f", &f0, &f1, &T, &a) == 4) {
                    exc_f0 = f0; exc_f1 = f1; exc_duracion = T; exc_amp_pct = a;
                    if (exc_iniciar(EXC_CHIRP))
                        uart_write_bytes(UART_NUM, "ACK,EXC,CHIRP\n", 14);
                    else
                        uart_write_bytes(UART_NUM, "NACK,NOT_OPEN_LOOP\n", 19);
                } else {
                    uart_write_bytes(UART_NUM, "NACK,BAD_FORMAT\n", 16);
                }
            }
            else if (strncmp((char*)rx_buffer, "PRBS:", 5) == 0) {
                float br, a, T;
                if (sscanf((char*)rx_buffer + 5, "%f,%f,%f", &br, &a, &T) == 3) {
                    exc_bitrate = br; exc_amp_pct = a; exc_duracion = T;
                    if (exc_iniciar(EXC_PRBS))
                        uart_write_bytes(UART_NUM, "ACK,EXC,PRBS\n", 13);
                    else
                        uart_write_bytes(UART_NUM, "NACK,NOT_OPEN_LOOP\n", 19);
                } else {
                    uart_write_bytes(UART_NUM, "NACK,BAD_FORMAT\n", 16);
                }
            }
            else if (strncmp((char*)rx_buffer, "EXC_STOP", 8) == 0) {
                exc_detener();
                uart_write_bytes(UART_NUM, "ACK,EXC,STOP\n", 13);
            }
            else if (strncmp((char*)rx_buffer, "MODE:", 5) == 0) {
                if (handshake_ok) {
                    snprintf(modo_actual, sizeof(modo_actual),
                             "%.15s", (char*)rx_buffer + 5);
                    auto_fase = AUTO_SWINGUP; 
                    resetear_estimador();
                    pbo       = 0.0f;
                    if (strncmp(modo_actual, "OPEN_LOOP", 9) != 0)
                        exc_detener();
                    enviar_estado();
                }
            }
            else if (strncmp((char*)rx_buffer, "L:", 2) == 0) {
                if (!handshake_ok) {
                    uart_write_bytes(UART_NUM, "NACK,NO_HANDSHAKE\n", 18);
                } else {
                    float a, b, c;
                    if (sscanf((char*)rx_buffer + 2, "%f,%f,%f", &a, &b, &c) == 3) {
                        L1 = a; L2 = b; L3 = c;
                        char ack[64];
                        int n = snprintf(ack, sizeof(ack),
                                         "ACK,L=%.4f,%.4f,%.4f\n", L1, L2, L3);
                        uart_write_bytes(UART_NUM, ack, n);
                    } else {
                        uart_write_bytes(UART_NUM, "NACK,BAD_FORMAT\n", 16);
                    }
                }
            }
            else if (strncmp((char*)rx_buffer, "ESTOP", 5) == 0) {
                auto_fase = AUTO_SWINGUP;
                resetear_estimador();
                pbo = 0.0f;
                manual_izq = false;
                manual_der = false;
                strncpy(modo_actual, "STANDBY", sizeof(modo_actual) - 1);
                modo_actual[sizeof(modo_actual) - 1] = '\0';
                exc_detener();
                motor_set(0);
                if (handshake_ok) enviar_estado();
            }
            else if (strncmp((char*)rx_buffer, "MANUAL:LEFT", 11) == 0) {
                if (handshake_ok) { manual_izq = true;  manual_der = false; }
            }
            else if (strncmp((char*)rx_buffer, "MANUAL:RIGHT", 12) == 0) {
                if (handshake_ok) { manual_izq = false; manual_der = true;  }
            }
            else if (strncmp((char*)rx_buffer, "MANUAL:STOP", 11) == 0) {
                if (handshake_ok) { manual_izq = false; manual_der = false; }
            }

            rx_idx = 0;
        }
        else if (rx_idx < (int)sizeof(rx_buffer) - 1) {
            rx_buffer[rx_idx++] = byte_rx;
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

    while (1) {
        vTaskDelayUntil(&last_wake, periodo);

        int64_t t_now_us = esp_timer_get_time();
        float dt = (t_now_us - t_prev_us) / 1e6f;
        t_prev_us = t_now_us;
        if (dt <= 0.0f || dt > 0.5f) dt = PEND_TS_S; 

        leer_encoders();

        actualizar_estimador_fd(pb,pd);

        //Control según modo
        float u;

        float rpm_disco = vd_est*60.0f/(2.0f*(float)M_PI);
        if (fabsf(rpm_disco)>RPM_MAX){
            u = (rpm_disco>0.0f)? -PEND_UMax: PEND_UMax;
        }else if (!handshake_ok) {
            u = 0.0f;
        }else{
            u= calcular_control();
        }

        motor_set(u);

        u_ctrl = u;


        //telemetría siempre
        pkt.h1          = 0xAA;
        pkt.h2          = 0xBB;
        pkt.dt          = dt;
        pkt.theta       = pb;
        pkt.theta_dot   = vb_est;   /* velocidad estimada barra */
        pkt.omega_rueda = vd_est;   /* velocidad estimada disco */
        pkt.u           = (int16_t)(u * 100.0f);

        uart_write_bytes(UART_NUM, (const char *)&pkt, sizeof(pkt));
    }
}

//Handshake 
static void tarea_hello(void *param)
{
    vTaskDelay(pdMS_TO_TICKS(300));  /* deja estabilizar la UART */

    while (1) {
        if (handshake_ok) {
            int32_t ahora = (int32_t)(esp_timer_get_time() / 1000);
            if ((ahora - t_ultimo_comando_ms) > TIMEOUT_HANDSHAKE_MS) {
                handshake_ok = false;
                manual_izq   = false;
                manual_der   = false;
                strncpy(modo_actual, "STANDBY", sizeof(modo_actual) - 1);
                modo_actual[sizeof(modo_actual) - 1] = '\0';
                auto_fase = AUTO_SWINGUP;
                resetear_estimador();
                pbo = 0.0f;
                exc_detener();
            }
        }
        if (!handshake_ok) {
            const char *msg = "HELLO,v2.0,PENDULO_ESP32\n";
            uart_write_bytes(UART_NUM, msg, strlen(msg));
        }
        vTaskDelay(pdMS_TO_TICKS(PING_HELLO_MS));
    }
}

void app_main(void)
{
    configurar_uart();
    configurar_pwm();
    configurar_encoder(ENC_BARRA_A, ENC_BARRA_B, &pcnt_barra);
    configurar_encoder(ENC_DISCO_A, ENC_DISCO_B, &pcnt_disco);

    /* Inicialización del unwrap: leer el estado actual del disco */
    {
        int tmp;
        pcnt_unit_get_count(pcnt_disco, &tmp);
        pdma = tmp;
    }

    xTaskCreate(tarea_recepcion, "recepcion", 4096, NULL, 5, NULL);
    xTaskCreate(tarea_control,   "control",   8192, NULL, 6, NULL);
    xTaskCreate(tarea_hello,     "hello",     4096, NULL, 3, NULL);
}
