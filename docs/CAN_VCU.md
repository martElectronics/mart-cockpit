# VCU — Mapa CAN completo

**Fuente:** firmware VCU (mart-cockpit), rama `feature/modJoseSTM32` · STM32G474RE · MART Formula Student
**Generado desde el código:** `include/Config.h`, `src/InverterControl.cpp`

---

## 1. Parámetros del bus

| | Valor |
|---|---|
| Bitrate | **125 kbps** (`cfg::CAN_SPEED_KBPS`) |
| Formato ID | **Standard 11-bit** |
| Byte order (multi-byte) | **Big-Endian (Motorola)** — MART_CAN convierte por defecto |
| Node ID (VCU / inversor) | **1** (`cfg::NODE_ID`) |
| DLC | 8 bytes; lo no usado va a 0x00 (telemetría VCU) o 0xFF (comandos inversor) |

> Los IDs de comando/estado del inversor se calculan como `combineInts(PacketID, NodeID) = (PacketID << 5) | NodeID` (modo Standard del HV-500). Con NodeID=1.

---

## 2. Resumen de IDs

### 2.1 Transmitido por la VCU

| ID dec | ID hex | Periodo | Contenido |
|---|---|---|---|
| **1160** | 0x488 | cada loop | **VCU_DIAG** — diagnóstico/post-mortem |
| **1163** | 0x48B | cada loop | APPS1%, APPS2%, APPS1 raw, APPS2 raw |
| **1164** | 0x48C | cada loop | Brake1%, Brake2%, Brake1 raw, Brake2 raw |
| **1166** | 0x48E | cada loop | Vbat, SDC, Start, R2D, estado APPS |

### 2.2 Comandos de la VCU al inversor (solo en R2D + APPS OK + modo CAN)

| ID dec | ID hex | PacketID | Comando |
|---|---|---|---|
| **961** | 0x3C1 | 0x1E | Set Relative current (consigna de par, %) |
| **1025** | 0x401 | 0x20 | Set max AC current |
| **1089** | 0x441 | 0x22 | Set max DC current |
| **1153** | 0x481 | 0x24 | Drive enable |

> Cuando no hay par habilitado, la VCU **deja de enviar** estos comandos (`removePacket`) y el inversor entra en *timeout* → free running. `Set ERPM (0x381)` **no se envía** (control solo por corriente).

### 2.3 Recibido por la VCU

| ID dec | ID hex | Origen | Contenido usado |
|---|---|---|---|
| **10** | 0x0A | BMS | SDC (byte 0, bit 2) |
| **65** | 0x41 | Inversor (PID 0x02) | Fault code (byte 4) |
| **129** | 0x81 | Inversor (PID 0x04) | Drive enable (byte 3) |

---

## 3. ID 1160 (0x488) — VCU_DIAG · 8 bytes

Trama de diagnóstico para post-mortem. Ocupa el hueco reservado "FAIL CODES" del Excel del equipo.

```
B0: causa de no-par
B1: tipo de implausibilidad del par APPS
B2: flags (comms / modo / estado) — bit a bit
B3: causa del último reset del micro
B4: fault code del inversor (eco)
B5: throttle comandado (%)
B6-7: heartbeat (u16 BE)
```

| Canal | Short | Byte | Bit | Len | Tipo | Descripción |
|---|---|---|---|---|---|---|
| VCU_FaultCause | FCAU | 0 | 0 | 8 | uint8 | 0=OK/conduciendo, 1=sin R2D, 2=SDC abierto, 3=APPS implausible, 4=fallo inversor, 5=comms BMS perdidas |
| VCU_AppsImpl   | AIMP | 1 | 0 | 8 | uint8 | 0=NONE, 1=Sensor1, 2=Sensor2, 3=ambos, 4=desviación >10% |
| VCU_BmsFresh   | BMSF | 2 | 0 | 1 | bool | Trama del BMS reciente (no stale) |
| VCU_InvFresh   | INVF | 2 | 1 | 1 | bool | Estado del inversor reciente |
| VCU_Simulating | SIMU | 2 | 2 | 1 | bool | Modo simulación activo |
| VCU_ModeCAN    | MCAN | 2 | 3 | 1 | bool | 1=control por CAN, 0=DIRECTO |
| VCU_Debug      | DBG_ | 2 | 4 | 1 | bool | Debug serie activo |
| VCU_DriveEN    | DREN | 2 | 5 | 1 | bool | Par habilitado (fuente única de verdad) |
| VCU_ResetCause | RCAU | 3 | 0 | 8 | uint8 | 1=power/BOR, 2=pin NRST, 3=software, **4=IWDG (loop colgado)**, 5=WWDG, 6=low-power, 0=desconocido |
| VCU_InvFault   | IFLT | 4 | 0 | 8 | uint8 | Fault code del inversor (eco de 0x41 B4) |
| VCU_Throttle   | THR_ | 5 | 0 | 8 | uint8 | Consigna de acelerador, 0–100 % |
| VCU_Heartbeat  | HBT_ | 6 | 0 | 16 | uint16 BE | Cuenta de loop; **se congela si el firmware se cuelga** |

> Para diagnosticar un fallo en pista: `VCU_ResetCause=4` → el watchdog reseteó (loop colgado); si `VCU_Heartbeat` deja de incrementar → cuelgue; `VCU_FaultCause` te dice de un vistazo por qué se cortó el par.

---

## 4. ID 1163 (0x48B) — APPS · 8 bytes

```
Bytes 0-1: APPS1 escalado (u16 BE)
Bytes 2-3: APPS2 escalado (u16 BE)
Bytes 4-5: APPS1 crudo ADC (u16 BE)
Bytes 6-7: APPS2 crudo ADC (u16 BE)
```

| Canal | Short | Byte | Bit | Len | Tipo | Escala | Unidad |
|---|---|---|---|---|---|---|---|
| APPS1_pct | A1PC | 0 | 0 | 16 | uint16 BE | 0.1 | % |
| APPS2_pct | A2PC | 2 | 0 | 16 | uint16 BE | 0.1 | % |
| APPS1_raw | A1RW | 4 | 0 | 16 | uint16 BE | 1 | cuentas ADC |
| APPS2_raw | A2RW | 6 | 0 | 16 | uint16 BE | 1 | cuentas ADC |

> El escalado va de 0 a 1000 (= 0–100.0 %, escala 0.1). El crudo es la lectura del MCP3208 (0–4095).

---

## 5. ID 1164 (0x48C) — Freno · 8 bytes

```
Bytes 0-1: Brake1 % (u16 BE)  — PENDIENTE: ahora va a 0 (no se calcula el %)
Bytes 2-3: Brake2 % (u16 BE)  — PENDIENTE: ahora va a 0
Bytes 4-5: Brake1 crudo ADC (u16 BE)
Bytes 6-7: Brake2 crudo ADC (u16 BE)
```

| Canal | Short | Byte | Bit | Len | Tipo | Escala | Unidad |
|---|---|---|---|---|---|---|---|
| Brake1_pct | B1PC | 0 | 0 | 16 | uint16 BE | 0.1 | % (TODO: hoy 0) |
| Brake2_pct | B2PC | 2 | 0 | 16 | uint16 BE | 0.1 | % (TODO: hoy 0) |
| Brake1_raw | B1RW | 4 | 0 | 16 | uint16 BE | 1 | cuentas ADC |
| Brake2_raw | B2RW | 6 | 0 | 16 | uint16 BE | 1 | cuentas ADC |

> El freno de R2D usa `Brake2_raw` con umbral `cfg::BRAKE_TH` (550). El cálculo del % está pendiente (bytes 0-3 a 0).

---

## 6. ID 1166 (0x48E) — Señales VCU · 8 bytes

```
Bytes 0-1: Vbat crudo (u16 BE)
Byte 2: SDC
Byte 3: Start
Byte 4: R2D
Byte 5: estado APPS
Bytes 6-7: sin usar (0)
```

| Canal | Short | Byte | Bit | Len | Tipo | Descripción |
|---|---|---|---|---|---|---|
| VCU_Vbat   | VBAT | 0 | 0 | 16 | uint16 BE | Lectura cruda de batería (MCP3208). TODO: escalar a V con el divisor real |
| VCU_SDC    | SDC_ | 2 | 0 | 8  | uint8 | SDC presente (recibido del BMS, ID 10 b2) |
| VCU_Start  | STRT | 3 | 0 | 8  | uint8 | Pulsador Start |
| VCU_R2D    | R2D_ | 4 | 0 | 8  | uint8 | Ready-to-Drive |
| VCU_AppsSt | APST | 5 | 0 | 8  | uint8 | Estado APPS: 0=NORMAL, 1=IMPLAUSIBLE, 2=PENDING |

> ⚠️ Layout cambiado respecto al Excel original (Vbat ahora 2 bytes; TSON sustituido por SDC; añadido estado APPS). Actualizar la config del receptor (dashboard/AiM) a esta tabla.

---

## 7. Comandos al inversor (DTI HV-500, manual V2.3)

Solo se envían con par habilitado (`driveEnabled = R2D && APPS OK && !faultInversor`) y en modo CAN. Multibyte en Big-Endian, valor escalado ×10 salvo indicación.

### 7.1 ID 961 (0x3C1) — Set Relative current (PID 0x1E)
| Byte | Dato | Len | Escala | Rango | Unidad |
|---|---|---|---|---|---|
| 0-1 | Corriente relativa (par) | int16 BE | 10 | -1000…1000 (=-100…100%) | % |

> La VCU envía 0…1000 (= 0…100%) desde la media del par APPS. El inversor conmuta a *control por corriente* al recibirlo.

### 7.2 ID 1025 (0x401) — Set max AC current (PID 0x20)
| Byte | Dato | Len | Escala | Unidad |
|---|---|---|---|---|
| 0-1 | Máx. corriente AC | int16 BE | 10 | Apk |

VCU envía `cfg::CURRENT_AC_MAX × 10` (190 A → 1900).

### 7.3 ID 1089 (0x441) — Set max DC current (PID 0x22)
| Byte | Dato | Len | Escala | Unidad |
|---|---|---|---|---|
| 0-1 | Máx. corriente DC | int16 BE | 10 | Adc |

VCU envía `cfg::CURRENT_DC_MAX × 10` (60 A → 600).

### 7.4 ID 1153 (0x481) — Drive enable (PID 0x24)
| Byte | Dato | Len | Descripción |
|---|---|---|---|
| 0 | Drive enable | uint8 | 1 = par permitido, 0 = no. Debe enviarse periódicamente |

---

## 8. Recibido por la VCU

### 8.1 ID 10 (0x0A) — Estado del BMS (1 byte)
| Canal | Byte | Bit | Descripción |
|---|---|---|---|
| BMS_SDC | 0 | 2 | SDC presente. La VCU lo usa como precondición de R2D. Watchdog: sin trama en `cfg::BMS_WD_MS` (2.5 s) → SDC = falso (fail-safe) |

### 8.2 ID 65 (0x41) — Inversor, PID 0x02 (8 bytes)
| Canal | Byte | Len | Descripción |
|---|---|---|---|
| Ctlr_Temp | 0-1 | int16 BE | Temperatura controlador (×0.1 °C) — no usado aún |
| Motor_Temp | 2-3 | int16 BE | Temperatura motor (×0.1 °C) — no usado aún |
| Inv_FaultCode | 4 | uint8 | **Fault code** (la VCU lo usa para cortar par y para VCU_DIAG B4) |

### 8.3 ID 129 (0x81) — Inversor, PID 0x04 (8 bytes)
| Canal | Byte | Bit | Descripción |
|---|---|---|---|
| Inv_DriveEnable | 3 | 24 | Estado de drive enable del inversor (la VCU lo lee para debug) |

---

## 9. Notas de integración

- **Inversor (DTI CAN Tool):** Node ID = **1**; CAN2 a **125 kbit/s**; activar **"Drive enable via CAN2"** y **"Send CAN2 Status"**. Si el Node ID real ≠ 1, cambiar `cfg::NODE_ID`.
- **El Excel `CAN_Data.xlsx` del equipo está obsoleto para el inversor** (numeración VESC vieja: ERPM=97, DriveEnable=385). Los IDs correctos son los de este documento (manual DTI V2.3).
- IDs VCU reservados en el Excel y **no implementados** aún: 400 (0x190), 1161-1162 (FAIL CODES libres), 1165 (steer/ruedas), 1167 (PWM).
- Fault codes del inversor (B4 de 0x41 / VCU_DIAG B4): 0=none, 1=Overvoltage, 2=Undervoltage, 3=DRV, 4=ABS overcurrent, 5=CTLR overtemp, 6=Motor overtemp, 7=Sensor wire, 8=Sensor general, 9=CAN cmd error, 10=Analog input error.
