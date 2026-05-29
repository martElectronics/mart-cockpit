//==============================================================================
// Tests unitarios (Unity) de AnalogSensor y PairedAnalogSensor.
// Entorno: native_test (HOST). Requiere un compilador nativo (gcc/clang/MinGW).
//   pio test -e native_test
// Usa un millis() controlable (g_millis) para validar los timeouts de forma
// determinista, sin esperas reales.
//==============================================================================
#include <unity.h>
#include "PairedAnalogSensor.h"

// --- Tiempo simulado: implementa el millis() declarado en el shim de Arduino. ---
static unsigned long g_millis = 0;
unsigned long millis() { return g_millis; }

// Config de un AnalogSensor lineal y determinista (sin filtro, sin márgenes).
static AnalogSensorConfig makeCfg(uint16_t rest, uint16_t full) {
    AnalogSensorConfig c;
    c.cfgAdcMinNormal       = rest;
    c.cfgAdcMaxNormal       = full;
    c.cfgScaledOutputMin    = 0;
    c.cfgScaledOutputMax    = 1000;
    c.cfgLowerMarginPercent = 0.0f;
    c.cfgUpperMarginPercent = 0.0f;
    c.cfgFilterType         = FilterType::NO_FILTER;
    c.cfgAdcRangeTolerance  = 50;
    c.cfgImplausibilityTimeout = 100;
    c.cfgAdcShortGND        = 10;
    c.cfgAdcShortVCC        = 4085;
    return c;
}

void setUp(void)    { g_millis = 0; }
void tearDown(void) {}

// ---------- AnalogSensor: escalado ----------
void test_scaling_normal(void) {
    AnalogSensorConfig cfg = makeCfg(1000, 3000);
    AnalogSensor s(cfg);
    float f, sc; SensorState st;
    s.update(2000, f, sc, st); TEST_ASSERT_FLOAT_WITHIN(1.0f, 500.0f,  sc);
    s.update(1000, f, sc, st); TEST_ASSERT_FLOAT_WITHIN(1.0f,   0.0f,  sc);
    s.update(3000, f, sc, st); TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, sc);
}

void test_scaling_inverse(void) {
    // Sensor inverso: reposo = ADC alto (3000), fondo = ADC bajo (1000).
    AnalogSensorConfig cfg = makeCfg(3000, 1000);
    AnalogSensor s(cfg);
    float f, sc; SensorState st;
    s.update(3000, f, sc, st); TEST_ASSERT_FLOAT_WITHIN(1.0f,   0.0f,  sc);
    s.update(1000, f, sc, st); TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, sc);
    s.update(2000, f, sc, st); TEST_ASSERT_FLOAT_WITHIN(1.0f, 500.0f,  sc);
}

// ---------- AnalogSensor: implausibilidades ----------
void test_out_of_range_tolerance(void) {
    AnalogSensorConfig cfg = makeCfg(1000, 3000);  // tol = 50 -> OOR fuera de [950, 3050]
    AnalogSensor s(cfg);
    float f, sc; SensorState st;

    // Dentro de la tolerancia (960 < min pero > min-tol): NORMAL.
    g_millis = 0; s.update(960, f, sc, st);
    TEST_ASSERT_TRUE(st == SensorState::NORMAL);

    // Fuera de rango: primero PENDING, luego IMPLAUSIBILITY al superar el timeout.
    g_millis = 0;   s.update(900, f, sc, st); TEST_ASSERT_TRUE(st == SensorState::PENDING);
    g_millis = 50;  s.update(900, f, sc, st); TEST_ASSERT_TRUE(st == SensorState::PENDING);
    g_millis = 150; s.update(900, f, sc, st); TEST_ASSERT_TRUE(st == SensorState::IMPLAUSIBILITY);
    TEST_ASSERT_TRUE(s.getImplausibilityType() == ImplausibilityType::OUT_OF_RANGE);
}

void test_short_to_gnd(void) {
    AnalogSensorConfig cfg = makeCfg(1000, 3000);
    AnalogSensor s(cfg);
    float f, sc; SensorState st;
    g_millis = 0;   s.update(5, f, sc, st);
    g_millis = 150; s.update(5, f, sc, st);
    TEST_ASSERT_TRUE(st == SensorState::IMPLAUSIBILITY);
    TEST_ASSERT_TRUE(s.getImplausibilityType() == ImplausibilityType::SHORT_TO_GND);
}

// ---------- PairedAnalogSensor ----------
static PairedAnalogSensorConfig makePairedCfg() {
    PairedAnalogSensorConfig c;
    c.cfgSensor1 = makeCfg(1000, 3000);
    c.cfgSensor2 = makeCfg(1000, 3000);
    c.cfgMaxDeviationPercent = 10.0f;
    c.cfgDeviationTimeout    = 100;
    return c;
}

// Regresión de los arreglos #1/#2: las medias escalada/filtrada NO deben salir cruzadas.
void test_paired_mean_not_swapped(void) {
    PairedAnalogSensorConfig cfg = makePairedCfg();
    PairedAnalogSensor p(cfg);
    float mf, ms, sf, ss; SensorState st;
    p.update(2000, 2000, mf, ms, sf, ss, st);

    TEST_ASSERT_TRUE(st == SensorState::NORMAL);
    // Media ESCALADA ~500 (0..1000); media FILTRADA ~2000 (cuentas ADC).
    TEST_ASSERT_FLOAT_WITHIN(1.0f,  500.0f, ms);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 2000.0f, mf);
    // Sensible: filtrado ~2000, escalado ~500 (no cruzados).
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 2000.0f, sf);
    TEST_ASSERT_FLOAT_WITHIN(1.0f,  500.0f, ss);
}

void test_paired_deviation_fault(void) {
    PairedAnalogSensorConfig cfg = makePairedCfg();
    PairedAnalogSensor p(cfg);
    float mf, ms, sf, ss; SensorState st;

    // scaled1=500, scaled2=700 -> desviación 20% > 10%.
    g_millis = 0;   p.update(2000, 2400, mf, ms, sf, ss, st);
    TEST_ASSERT_TRUE(st == SensorState::NORMAL);            // dentro de la ventana
    g_millis = 150; p.update(2000, 2400, mf, ms, sf, ss, st);
    TEST_ASSERT_TRUE(st == SensorState::IMPLAUSIBILITY);
    TEST_ASSERT_TRUE(p.getImplausibilityType() == PairedImplausibilityType::DEVIATION_FAULT);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ms);             // salida segura (0) en fallo
}

void test_paired_single_sensor_fault(void) {
    PairedAnalogSensorConfig cfg = makePairedCfg();
    PairedAnalogSensor p(cfg);
    float mf, ms, sf, ss; SensorState st;

    // Sensor 1 en corto a GND, sensor 2 normal.
    g_millis = 0;   p.update(5, 2000, mf, ms, sf, ss, st);
    g_millis = 150; p.update(5, 2000, mf, ms, sf, ss, st);
    TEST_ASSERT_TRUE(st == SensorState::IMPLAUSIBILITY);
    TEST_ASSERT_TRUE(p.getImplausibilityType() == PairedImplausibilityType::SENSOR1_FAULT);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ms);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_scaling_normal);
    RUN_TEST(test_scaling_inverse);
    RUN_TEST(test_out_of_range_tolerance);
    RUN_TEST(test_short_to_gnd);
    RUN_TEST(test_paired_mean_not_swapped);
    RUN_TEST(test_paired_deviation_fault);
    RUN_TEST(test_paired_single_sensor_fault);
    return UNITY_END();
}
