# Documentación de código — VCU (`mart-cockpit`, `feature/modJoseSTM32`)

Guía de **onboarding** del firmware de la VCU: qué hace, cómo encaja y qué es delicado.
El código fuente está muy comentado; esto es la **vista de pájaro**.

## Propósito
La VCU (STM32G474RE) lee el **acelerador (APPS)**, el **freno**, la **dirección** y la
**velocidad de ruedas**, decide el **Ready-to-Drive (R2D)** y manda al inversor **DTI
HV-500** la consigna de par por CAN, con un **límite dinámico de potencia** según el
voltaje del pack. Recibe el estado del SDC del BMS por CAN y publica telemetría al
datalogger.

## Flujo (`loop()` → `controlInverter()`)
```
1. stsStart  = botón Start                        (PB7, activo a 0)
2. APPS      → appsSensor.update() → throttle 0..1000 + appsOk (plausibilidad)
3. PowerLimiter → capa el throttle según V_dc/eRPM del inversor   ← límite dinámico
4. stsR2D    = r2dSM.update(SDC, Start, freno)    ← máquina de estados R2D
5. driveEnabled = stsR2D && appsOk && !inverterFault   ← fuente ÚNICA de verdad
6. digitalWrite(PC6, driveEnabled)                ← DriveEnable SIEMPRE por pin
7. if (MODE_CAN) inverter.sendCommands(driveEnabled, throttle, maxAC, maxDC)
```
El `stsSDC` viene del **BMS por CAN** (ID 0x0A, bit 2), con watchdog: si el BMS calla
2,5 s → `stsSDC=false` → R2D cae (fail-safe).

## Módulos / clases (las que tocamos)
| Clase / fichero | Qué hace |
|---|---|
| **`InverterController.h`** | Encapsula la E/S CAN del DTI: lee `0x401` (eRPM/Vin), `0x441` (fault), `0x481` (drive enable); manda `0x181/0x0A1/0x101/0x141`. Getters `dcVoltage()`, `erpm()`, `faultCode()`, `isFresh()`. |
| **`PowerLimiter.h`** | Límite dinámico de corriente AC: `i_ac_max = (V_dc·I_fuse·η)/(K_T·ω)`, clampeado a `P_MAX` y a `I_motor`. Sin field weakening (el rango queda por debajo de base). |
| **`R2DStateMachine.h`** | Máquina R2D: `idle →(SDC)→ wait →(Start+freno)→ active` (+ buzzer). Si cae el SDC → idle. Estado explícito (antes era `static` oculto). |
| **`SteeringSensor.h`** | PSC-360 (string pot) → % de −100 (izq) a +100 (der) con calibración izq/centro/der. |
| **`WheelSpeed.h`** | Velocidad de rueda por conteo de pulsos Hall (SNDH) → RPM. ISR externo cuenta; `updateRpm()` calcula sobre la ventana. |

## Modos de control (`controlMode`)
- **`MODE_CAN` (por defecto):** el acelerador va al inversor como **Set Relative Current**
  (`0x0A1`, 0..1000 = 0..100 % de los 190 Arms). **Aquí actúa el PowerLimiter.**
- **`MODE_DIRECT`:** el pedal va **analógico** directo al DTI; la VCU solo habilita por el
  pin PC6. El PowerLimiter **no** actúa (no se manda throttle por CAN).

## Mapa CAN del inversor (DTI V2.5, Standard ID, node 1)
| ID | Sentido | Contenido |
|---|---|---|
| `0x401` | RX | eRPM (b0-3), Duty (b4-5), **Vin** (b6-7, V) |
| `0x441` | RX | temps + **fault code** (b4) |
| `0x481` | RX | throttle/brake/**drive enable** (b3) |
| `0x181` | TX | Drive enable |
| `0x0A1` | TX | Set Relative current (% ×10) |
| `0x101` | TX | Set max AC (Apk ×10) |
| `0x141` | TX | Set max DC (Adc ×10) |

Telemetría VCU publicada: `0x488` (diag: `faultCause`, `heartbeat`, `resetCause`),
`0x48B` APPS, `0x48C` freno, `0x48D` dirección+ruedas, `0x48E` señales.

## Seguridad
- **APPS plausibilidad (EV.5.5):** dos sensores; si difieren >10 % durante >100 ms →
  `throttle=0` y `driveEnabled=false` (lo hace `PairedAnalogSensor`).
- **APPS/Brake (EV.5.7):** implementado en **hardware** (no en la VCU).
- **Watchdogs:** BMS CAN (2,5 s → SDC false), inversor CAN (1 s → `stop()`), micro IWDG.
- **Corte por subtensión:** si `V_dc < V_PACK_MIN_OP` → throttle 0.

## Configuración (`include/Config.h`)
Pines, canales ADC (APPS SINGLE_2/3, freno 4/5, Vbat 6, dirección 0), IDs CAN, límites
(`CURRENT_AC_MAX` 190 Arms, `_APK` 269, `CURRENT_DC_MAX` 125 = fusible), constantes del
limitador (`KT_EFF`, `I_FUSE_MAX_A`, `P_MAX_W`, `POLE_PAIRS`, `V_PACK_MIN_OP`).

## ⚠ Placeholders a calibrar en banco
`STEER_ADC_LEFT/CENTER/RIGHT` · `WHEEL_TEETH` (dientes de la corona) · `PIN_WHEEL_L/R`
(ajustar a la PCB) · `VBAT_DIVIDER` · `V_PACK_MIN_OP` (292 si 10 módulos) · escala de
`Vin` del 0x401 (confirmada en manual, validar en banco) · calibración APPS.

## Gotchas
- El PowerLimiter **solo actúa en `MODE_CAN`** (el coche corre en CAN).
- La VCU manda **solo Set Relative Current** — NO `Set ERPM`/`Set Position` (cambiarían
  el modo de control del DTI cada ciclo). Para tests de "motor energizado sin girar",
  comandar desde el DTI CAN Tool.
- `cfgCurrentACMAX` (190 Arms) es la **referencia** del limitador; `cfgCurrentACMAXApk`
  (269) es el **comando** Set Max AC (unidades distintas: Arms vs Apk).
- Config del DTI: CAN2 a **125 k**, map **V25**, **Drive enable via CAN2 = On**, node 1.
