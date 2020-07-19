/*
#include "MPC.h"


mpc::mpc(){}
mpc::~mpc(){}


size_t N = 10; // Number of steps to Obstacle.
double dt = 0.05; //simulation step time

std::vector<double> mpc::Solve(gsl_vector *state, gsl_vector *coeffs){

    double x = gsl_vector_get(state, 0);
    double y = gsl_vector_get(state, 1);
    double psi = gsl_vector_get(state,2);
    double v = gsl_vector_get(state,3);
    double cte = gsl_vector_get(state,4);
    double epsi = gsl_vector_get(state,5);

    size_t n_vars = state->size;
    size_t n_constraints = N * state->size;


    const gsl_multiroot_fsolver_type * T;
    gsl_multiroot_fsolver *s;
    int status;
    size_t i, iter = 0;
    const size_t n = 2;



}
*/
