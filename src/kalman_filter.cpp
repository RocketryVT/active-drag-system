#include "kalman_filter.hpp"

static void kalman_verticality_init() {
    /************************************************************************/
    /* initialize the filter structures                                     */
    /************************************************************************/
    kalman_filter_initialize(&kf, KALMAN_NUM_STATES, KALMAN_NUM_INPUTS);
    kalman_observation_initialize(&kfm, KALMAN_NUM_STATES, KALMAN_NUM_MEASUREMENTS);

    /************************************************************************/
    /* set initial state                                                    */
    /************************************************************************/
    mf16 *x = kalman_get_state_vector(&kf);
    x->data[0][0] = 0; // s_i
    x->data[1][0] = 0; // v_i

    /************************************************************************/
    /* set state transition                                                 */
    /************************************************************************/
    mf16 *A = kalman_get_state_transition(&kf);
    
    // set time constant
#if (DEBUG == 1)
    const fix16_t T = fix16_from_float(0.02f);
#else
    const fix16_t T = fix16_from_float(0.02f);
#endif
    const fix16_t Tsquare = fix16_sq(T);

    // helper
    const fix16_t fix16_half = fix16_from_float(0.5);

    // transition of x to s think this should be x to v
    matrix_set(A, 0, 0, fix16_one);   // 1
    matrix_set(A, 0, 1, T);   // T
    
    // transition of x to v think this should be v to acel
    matrix_set(A, 1, 0, 0);   // 0
    matrix_set(A, 1, 1, fix16_one);   // 1


    /************************************************************************/
    /* set covariance                                                       */
    /************************************************************************/
    mf16 *P = kalman_get_system_covariance(&kf);

//    matrix_set_symmetric(P, 0, 0, fix16_half);   // var(s)
//    matrix_set_symmetric(P, 0, 1, 0);   // cov(s,v)
//    matrix_set_symmetric(P, 0, 2, 0);   // cov(s,g)
//
//    matrix_set_symmetric(P, 1, 1, fix16_one);   // var(v)
//    matrix_set_symmetric(P, 1, 2, 0);   // cov(v,g)
//
//    matrix_set_symmetric(P, 2, 2, fix16_one);   // var(g)

    /************************************************************************/
    /* set input covariance                                                 */
    /************************************************************************/
    mf16 *Q = kalman_get_input_covariance(&kf);
//    mf16_fill_diagonal(Q, fix16_one);
    mf16_mul_bt(Q, A, A);
    mf16_mul_s(Q, Q, fix16_from_int(10*10));

    /************************************************************************/
    /* set control input transformation                                       */
    /************************************************************************/
    mf16 *B = kalman_get_input_transition(&kf);
    matrix_set(B, 0, 0, fix16_mul(fix16_half, Tsquare)); // u = 0*s 
    matrix_set(B, 1, 0, T);                              //   + 0*v

    /************************************************************************/
    /* set measurement transformation                                       */
    /************************************************************************/
    mf16 *H = kalman_get_observation_transformation(&kfm);
    matrix_set(H, 0, 0, fix16_one);     // z = 1*s 
    matrix_set(H, 0, 1, 0);             //   + 0*v

    /************************************************************************/
    /* set process noise                                                    */
    /************************************************************************/
    mf16 *R = kalman_get_observation_process_noise(&kfm);

    matrix_set(R, 0, 0, fix16_from_int(36));     // var(s)
}

volatile bool derate_baro_sensor = false;
const fix16_t baro_velocity_derate = F16(160.f);

void kalman_update(fix16_t altitude, fix16_t vertical_acceleration) {
    static mf16* control = kalman_get_input_vector(&kf);
    static mf16* measurement = kalman_get_observation_vector(&kfm);
    static mf16* Q = kalman_get_input_covariance(&kf);
    static mf16 *R = kalman_get_observation_process_noise(&kfm);
    static mf16* state_vector = kalman_get_state_vector(&kf);

    if (state_vector->data[1][0] >= baro_velocity_derate && !derate_baro_sensor) {
        mf16_div_s(Q, Q, fix16_from_int(3));
        mf16_mul_s(R, R, fix16_from_int(4));
        derate_baro_sensor = true;
    } else if (state_vector->data[1][0] < baro_velocity_derate && derate_baro_sensor) {
        mf16_mul_s(Q, Q, fix16_from_int(3));
        mf16_div_s(R, R, fix16_from_int(4));
        derate_baro_sensor = false;
    }
    matrix_set(control, 0, 0, vertical_acceleration);
    kalman_predict(&kf);
    matrix_set(measurement, 0, 0, altitude);
    kalman_correct(&kf, &kfm);
}

fix16_t calculate_drag_force(fix16_t deployment_percentage, fix16_t vertical_velocity) {
    static const fix16_t p00 = F16(125.f);
    static const fix16_t p10 = F16(-3.286f);
    static const fix16_t p01 = F16(-1.803f);
    static const fix16_t p20 = F16(0.01675f);
    static const fix16_t p11 = F16(0.02687f);
    static const fix16_t p02 = F16(0.008441f);

    fix16_t term1 = fix16_mul(p10, deployment_percentage);
    fix16_t term2 = fix16_mul(p01, vertical_velocity);
    fix16_t term3 = fix16_mul(p20, fix16_sq(deployment_percentage));
    fix16_t term4 = fix16_mul(fix16_mul(p11, vertical_velocity), deployment_percentage);
    fix16_t term5 = fix16_mul(fix16_mul(p02, vertical_velocity), vertical_velocity);

    fix16_t drag_force = fix16_add(p00, term1);
            drag_force = fix16_add(drag_force, term2);
            drag_force = fix16_add(drag_force, term3);
            drag_force = fix16_add(drag_force, term4);
            drag_force = fix16_add(drag_force, term5);

    return drag_force;
}

fix16_t predict_apogee(fix16_t altitude, fix16_t vertical_velocity, fix16_t drag_force) {
    static const fix16_t gravity = F16(9.81f);
    static const fix16_t mass = F16(21.8f);

    fix16_t nal_log_internal = fix16_div(gravity, fix16_add(gravity, fix16_div(drag_force, mass)));
    fix16_t nal_log_scale = fix16_mul(fix16_mul(fix16_div(vertical_velocity, fix16_mul(F16(2), drag_force)), vertical_velocity), mass);
    fix16_t apogee_prediction = fix16_sub(altitude, fix16_mul(nal_log_scale, fix16_log(nal_log_internal)));
    return apogee_prediction;
}

fix16_t calculate_deployment_percentage(fix16_t drag_force, fix16_t vertical_velocity) {
    static const fix16_t p00 = F16(79.05f);
    static const fix16_t p10 = F16(1.057f);
    static const fix16_t p01 = F16(-1.049f);
    static const fix16_t p20 = F16(-7.296e-5f);
    static const fix16_t p11 = F16(-0.003321f);
    static const fix16_t p02 = F16(0.002322f);


    fix16_t term1 = fix16_mul(p10, drag_force);
    fix16_t term2 = fix16_mul(p01, vertical_velocity);
    fix16_t term3 = fix16_mul(fix16_mul(p20, drag_force), drag_force);
    fix16_t term4 = fix16_mul(fix16_mul(p11, drag_force), vertical_velocity);
    fix16_t term5 = fix16_mul(fix16_mul(p02, vertical_velocity), vertical_velocity);

    fix16_t deployment_percentage = fix16_add(p00, term1);
            deployment_percentage = fix16_add(deployment_percentage, term2);
            deployment_percentage = fix16_add(deployment_percentage, term3);
            deployment_percentage = fix16_add(deployment_percentage, term4);
            deployment_percentage = fix16_add(deployment_percentage, term5);
            deployment_percentage = fix16_clamp(deployment_percentage, 0, fix16_from_int(100));
    return deployment_percentage;
}

fix16_t calculate_desired_drag_force(fix16_t altitude, fix16_t vertical_velocity) {
    static const fix16_t p00 = F16(-2.042e+01);
    static const fix16_t p10 = F16(2.879e+01);
    static const fix16_t p01 = F16(2.391e+02);
    static const fix16_t p20 = F16(-1.265e+01);
    static const fix16_t p11 = F16(-2.499e+02);
    static const fix16_t p02 = F16(-1.063e+03);
    static const fix16_t p30 = F16(1.774);
    static const fix16_t p21 = F16(7.604e+01);
    static const fix16_t p12 = F16(7.028e+02);
    static const fix16_t p03 = F16(2.135e+03);
    static const fix16_t p31 = F16(-6.349);
    static const fix16_t p22 = F16(-1.049e+02);
    static const fix16_t p13 = F16(-6.41e+02);
    static const fix16_t p04 = F16(-1.604e+03);

    fix16_t altitude_km = fix16_div(altitude, fix16_from_int(1000));
    fix16_t vertical_velocity_km = fix16_div(vertical_velocity_km, fix16_from_int(1000));

    fix16_t term01 = fix16_mul(p10, altitude_km);
    fix16_t term02 = fix16_mul(p01, vertical_velocity_km);
    fix16_t term03 = fix16_mul(fix16_mul(p20, altitude_km), altitude_km);
    fix16_t term04 = fix16_mul(fix16_mul(p11, altitude_km), vertical_velocity_km);
    fix16_t term05 = fix16_mul(fix16_mul(p02, vertical_velocity_km), vertical_velocity_km);
    fix16_t term06 = fix16_mul(fix16_mul(fix16_mul(p30, altitude_km), altitude_km), altitude_km);
    fix16_t term07 = fix16_mul(fix16_mul(fix16_mul(p21, altitude_km), altitude_km), vertical_velocity_km);
    fix16_t term08 = fix16_mul(fix16_mul(fix16_mul(p12, altitude_km), vertical_velocity_km), vertical_velocity_km);
    fix16_t term09 = fix16_mul(fix16_mul(fix16_mul(p03, vertical_velocity_km), vertical_velocity_km), vertical_velocity_km);
    fix16_t term10 = fix16_mul(fix16_mul(fix16_mul(fix16_mul(p31, altitude_km), altitude_km), altitude_km), vertical_velocity_km);
    fix16_t term11 = fix16_mul(fix16_mul(fix16_mul(fix16_mul(p22, altitude_km), altitude_km), vertical_velocity_km), vertical_velocity_km);
    fix16_t term12 = fix16_mul(fix16_mul(fix16_mul(fix16_mul(p13, altitude_km), vertical_velocity_km), vertical_velocity_km), vertical_velocity_km);
    fix16_t term13 = fix16_mul(fix16_mul(fix16_mul(fix16_mul(p04, vertical_velocity_km), vertical_velocity_km), vertical_velocity_km), vertical_velocity_km);

    fix16_t desired_drag_force = fix16_add(p00, term01);
            desired_drag_force = fix16_add(desired_drag_force, term02);
            desired_drag_force = fix16_add(desired_drag_force, term03);
            desired_drag_force = fix16_add(desired_drag_force, term04);
            desired_drag_force = fix16_add(desired_drag_force, term05);
            desired_drag_force = fix16_add(desired_drag_force, term06);
            desired_drag_force = fix16_add(desired_drag_force, term07);
            desired_drag_force = fix16_add(desired_drag_force, term08);
            desired_drag_force = fix16_add(desired_drag_force, term09);
            desired_drag_force = fix16_add(desired_drag_force, term10);
            desired_drag_force = fix16_add(desired_drag_force, term11);
            desired_drag_force = fix16_add(desired_drag_force, term12);
            desired_drag_force = fix16_add(desired_drag_force, term13);

            desired_drag_force = fix16_mul(desired_drag_force, fix16_from_int(1000));

    return desired_drag_force;
}
