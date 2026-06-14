//==============================================================================
// Tests unitarios (Unity) de SafetyLogic — funciones puras de seguridad del VCU.
// Entorno: native_test (HOST). Sin hardware: son funciones de datos->datos.
//   pio test -e native_test
//
// #2 Watchdog del BMS (bmsWatchdogExpired): fija como CONTRATO el fail-safe que
//    hoy es frágil (consume-on-read del CAN + el ID 10 puede tardar hasta 800 ms
//    por el timer de prioridad + BMS_WD_MS=2500). El margen 800 < 2500 debe
//    sostenerse: una trama cada 800 ms NO debe declarar caído al BMS.
// #3 Rampa de subtensión (underVoltageScale): corte suave 0..1 del throttle.
//==============================================================================
#include <unity.h>
#include "SafetyLogic.h"

// SafetyLogic no usa millis(), pero el shim lo declara; lo definimos para enlazar.
unsigned long millis() { return 0; }

void setUp(void)    {}
void tearDown(void) {}

// ---------- #2 Watchdog del BMS ----------
static const uint32_t WD = 2500;   // BMS_WD_MS real

void test_wd_fresh_not_expired(void) {
    // Trama recién recibida (now == last): no expira.
    TEST_ASSERT_FALSE(bmsWatchdogExpired(1000, 1000, WD));
}

void test_wd_within_window(void) {
    // Justo en el límite (now-last == timeout): NO expira (es '>' estricto).
    TEST_ASSERT_FALSE(bmsWatchdogExpired(0, WD, WD));
}

void test_wd_expires_past_window(void) {
    // Un ms más allá del timeout: expira -> fail-safe.
    TEST_ASSERT_TRUE(bmsWatchdogExpired(0, WD + 1, WD));
}

void test_wd_800ms_cadence_ok(void) {
    // El peor caso del timer de prioridad del BMS: ID 10 cada 800 ms. Tres tramas
    // seguidas a 800 ms NO deben disparar el watchdog (margen 800 << 2500).
    unsigned long last = 0;
    for (unsigned long t = 800; t <= 2400; t += 800) {
        TEST_ASSERT_FALSE(bmsWatchdogExpired(last, t, WD)); // aún fresco
        last = t;                                           // llega la trama
    }
}

void test_wd_silence_trips(void) {
    // BMS se calla tras la última trama en t=0: a los 2501 ms cae el SDC.
    TEST_ASSERT_TRUE(bmsWatchdogExpired(0, 2501, WD));
}

void test_wd_millis_wrap(void) {
    // Robustez ante el wrap de millis() (uint32_t, ~49 días): last cerca del techo
    // de 32 bits, now ya dio la vuelta. Con tipos uint32_t la resta envuelve en 32
    // bits en CUALQUIER host (también LP64), igual que en el STM32 -> delta = 356 ms.
    uint32_t last = 0xFFFFFF00UL;   // ~justo antes del wrap
    uint32_t now  = 0x00000064UL;   // 100 ms después del wrap
    TEST_ASSERT_FALSE(bmsWatchdogExpired(last, now, WD)); // delta real = 0x164 = 356 ms
}

// ---------- #3 Rampa de subtensión ----------
static const float VMIN = 350.0f;
static const float RAMP = 15.0f;   // -> par pleno >=365, nulo <=350, 50% a 357.5

void test_uv_full_above_ramp(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, underVoltageScale(400.0f, VMIN, RAMP));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, underVoltageScale(365.0f, VMIN, RAMP)); // borde superior
}

void test_uv_zero_below_min(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, underVoltageScale(300.0f, VMIN, RAMP));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, underVoltageScale(350.0f, VMIN, RAMP)); // borde inferior
}

void test_uv_half_at_midpoint(void) {
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, underVoltageScale(357.5f, VMIN, RAMP));
}

void test_uv_linear_quarter(void) {
    // A 1/4 de la rampa (353.75 V) -> factor 0.25.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.25f, underVoltageScale(353.75f, VMIN, RAMP));
}

void test_uv_zero_ramp_is_hard_cut(void) {
    // ramp=0 (sin rampa): corte duro. >=vmin -> 1, <vmin -> 0. Sin div por cero.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, underVoltageScale(351.0f, VMIN, 0.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, underVoltageScale(349.0f, VMIN, 0.0f));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_wd_fresh_not_expired);
    RUN_TEST(test_wd_within_window);
    RUN_TEST(test_wd_expires_past_window);
    RUN_TEST(test_wd_800ms_cadence_ok);
    RUN_TEST(test_wd_silence_trips);
    RUN_TEST(test_wd_millis_wrap);
    RUN_TEST(test_uv_full_above_ramp);
    RUN_TEST(test_uv_zero_below_min);
    RUN_TEST(test_uv_half_at_midpoint);
    RUN_TEST(test_uv_linear_quarter);
    RUN_TEST(test_uv_zero_ramp_is_hard_cut);
    return UNITY_END();
}
