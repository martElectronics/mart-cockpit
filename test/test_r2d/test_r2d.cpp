//==============================================================================
// Tests unitarios (Unity) de R2DStateMachine — la máquina de Ready-to-Drive.
// Entorno: native_test (HOST). Es lógica pura (FSM IDLE->WAIT->ACTIVE), sin
// hardware real: el buzzer se observa vía el registro de pines del shim y
// millis() es controlable.
//   pio test -e native_test
//
// Contratos que se fijan (los que cuestan un fallo en pista/scrutineering):
//   · NO arma R2D sin la secuencia correcta (SDC presente + start + freno).
//   · NO arma si falta el freno o el start.
//   · NO arma sin SDC aunque haya start + freno.
//   · La caída de SDC saca de R2D y obliga a re-secuenciar.
//   · Una vez en R2D, soltar el freno/start NO lo desactiva (solo el SDC).
//   · El buzzer suena al armar y se apaga tras buzzerOnMs.
//==============================================================================
#include <unity.h>
#include "R2DStateMachine.h"

// Tiempo simulado: implementa el millis() declarado en el shim de Arduino.
static unsigned long g_millis = 0;
unsigned long millis() { return g_millis; }

static const uint8_t  PIN_BUZZ = 7;
static const uint32_t BUZZ_MS  = 1500;

void setUp(void)    { g_millis = 0; for (auto &p : _pinState) p = 0; }
void tearDown(void) {}

// Lleva la FSM a ACTIVE (R2D) con la secuencia correcta.
static void arm(R2DStateMachine &sm) {
    TEST_ASSERT_FALSE(sm.update(true, false, false)); // IDLE -> WAIT (aún no R2D)
    TEST_ASSERT_TRUE (sm.update(true, true,  true));  // WAIT -> ACTIVE (R2D)
}

// 1) Arranca en reposo: un único update con SDC entra en WAIT pero NO arma.
void test_starts_idle(void) {
    R2DStateMachine sm(PIN_BUZZ, BUZZ_MS);
    TEST_ASSERT_FALSE(sm.update(false, false, false));
    TEST_ASSERT_FALSE(sm.update(true,  false, false)); // IDLE -> WAIT, no R2D
}

// 2) Sin SDC no arma aunque haya start + freno.
void test_no_arm_without_sdc(void) {
    R2DStateMachine sm(PIN_BUZZ, BUZZ_MS);
    TEST_ASSERT_FALSE(sm.update(false, true, true));
    TEST_ASSERT_FALSE(sm.update(false, true, true));
}

// 3) Secuencia correcta arma R2D y se mantiene.
void test_arms_with_sequence(void) {
    R2DStateMachine sm(PIN_BUZZ, BUZZ_MS);
    arm(sm);
    TEST_ASSERT_TRUE(sm.update(true, true, true));
}

// 4) En WAIT, ni el freno solo ni el start solo arman: hacen falta ambos.
void test_needs_both_start_and_brake(void) {
    R2DStateMachine sm(PIN_BUZZ, BUZZ_MS);
    TEST_ASSERT_FALSE(sm.update(true, false, false)); // -> WAIT
    TEST_ASSERT_FALSE(sm.update(true, false, true));  // solo freno
    TEST_ASSERT_FALSE(sm.update(true, true,  false)); // solo start
    TEST_ASSERT_TRUE (sm.update(true, true,  true));  // ambos -> R2D
}

// 5) Caída de SDC saca de R2D y exige re-secuenciar (un solo update no re-arma).
void test_sdc_loss_drops_and_requires_resequence(void) {
    R2DStateMachine sm(PIN_BUZZ, BUZZ_MS);
    arm(sm);
    TEST_ASSERT_FALSE(sm.update(false, true, true)); // SDC cae -> IDLE
    TEST_ASSERT_FALSE(sm.update(true,  true, true)); // sdc vuelve: IDLE->WAIT, NO arma de golpe
    TEST_ASSERT_TRUE (sm.update(true,  true, true)); // ahora sí
}

// 6) Una vez en R2D, soltar el freno/start NO lo desactiva (solo el SDC).
void test_r2d_persists_on_brake_release(void) {
    R2DStateMachine sm(PIN_BUZZ, BUZZ_MS);
    arm(sm);
    TEST_ASSERT_TRUE(sm.update(true, false, false)); // freno y start sueltos -> sigue R2D
    TEST_ASSERT_TRUE(sm.update(true, false, true));
}

// 7) El buzzer suena al armar y se apaga tras buzzerOnMs (requisito de scrutineering).
void test_buzzer_pulse_duration(void) {
    R2DStateMachine sm(PIN_BUZZ, BUZZ_MS);
    g_millis = 0;
    sm.update(true, false, false);              // WAIT
    sm.update(true, true,  true);               // ACTIVE -> buzzer ON
    TEST_ASSERT_EQUAL_UINT8(HIGH, _pinState[PIN_BUZZ]);

    g_millis = BUZZ_MS - 1; sm.update(true, true, true);
    TEST_ASSERT_EQUAL_UINT8(HIGH, _pinState[PIN_BUZZ]); // dentro de la ventana: sigue sonando

    g_millis = BUZZ_MS; sm.update(true, true, true);
    TEST_ASSERT_EQUAL_UINT8(LOW,  _pinState[PIN_BUZZ]); // cumplida la ventana: se apaga
}

// 8) reset() vuelve a IDLE (hay que re-secuenciar para volver a armar).
void test_reset_returns_to_idle(void) {
    R2DStateMachine sm(PIN_BUZZ, BUZZ_MS);
    arm(sm);
    sm.reset();
    TEST_ASSERT_FALSE(sm.update(true, true, true)); // IDLE -> WAIT, no arma de golpe
    TEST_ASSERT_TRUE (sm.update(true, true, true));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_idle);
    RUN_TEST(test_no_arm_without_sdc);
    RUN_TEST(test_arms_with_sequence);
    RUN_TEST(test_needs_both_start_and_brake);
    RUN_TEST(test_sdc_loss_drops_and_requires_resequence);
    RUN_TEST(test_r2d_persists_on_brake_release);
    RUN_TEST(test_buzzer_pulse_duration);
    RUN_TEST(test_reset_returns_to_idle);
    return UNITY_END();
}
