/*
 *  ACADO MPC code generator for kinematic bicycle model
 * http://acado.sourceforge.net/doc/html/d4/d26/example_013.html
 * http://acado.sourceforge.net/doc/html/db/daf/cgt_getting_started.html
 */

#include <acado_toolkit.hpp>
#include <acado_code_generation.hpp>

using namespace std;

USING_NAMESPACE_ACADO

int main( )
{
    // INTRODUCE THE VARIABLES (acadoVariables.x):
    // -------------------------
    DifferentialState x;
    DifferentialState y;
    DifferentialState v;
    DifferentialState phi;
    DifferentialState delta;

    Control a;
    Control deltarate;

    double L = 2.875;	  // vehicle wheel base

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    DifferentialEquation f;
    // model equations
    f << dot(x) == v*cos(phi);
    f << dot(y) == v*sin(phi);
    f << dot(v) == a;
    f << dot(phi) == v*tan(delta)/L;
    f << dot(delta) == deltarate;

    //
    // Weighting matrices and reference functions (acadoVariables.y)
    //
    Function rf;
    Function rfN;

    rf  << x << y << v << phi << a << deltarate;
    rfN << x << y << v << phi;

    const int N  = 10;
    const int Ni = 4;
    const double Ts = 0.1;

    // Provide defined weighting matrices:
    BMatrix W = eye<bool>(rf.getDim());
    BMatrix WN = eye<bool>(rfN.getDim());

    OCP ocp(0, N * Ts, N);

    ocp.subjectTo( f );
    // control constraints
    ocp.subjectTo( -1.0 <= a <= 1.0 );
    //ocp.subjectTo( 0.05*0.05 <= v*v <= 1.0*1.0 );
    ocp.subjectTo( -M_PI/2 <= delta <= M_PI/2 );
    ocp.subjectTo( -M_PI/4 <= deltarate <= M_PI/4 );

    ocp.minimizeLSQ(W, rf);
    ocp.minimizeLSQEndTerm(WN, rfN);

    //
    // Export the code:
    //
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    //mpc.set(DISCRETIZATION_TYPE, SINGLE_SHOOTING);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    //mpc.set(INTEGRATOR_TYPE, INT_RK4);
    mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA3);
    mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
    //	mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
    mpc.set(QP_SOLVER, QP_QPOASES);
    //	mpc.set(QP_SOLVER, QP_FORCES);
    mpc.set(MAX_NUM_QP_ITERATIONS, 40);
    mpc.set(HOTSTART_QP, YES);
    //	mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);
    //	mpc.set(LEVENBERG_MARQUARDT, 1.0e-10);
    mpc.set(GENERATE_TEST_FILE, YES);
    mpc.set(GENERATE_MAKE_FILE, YES);
    mpc.set(GENERATE_MATLAB_INTERFACE, YES);
    //	mpc.set(USE_SINGLE_PRECISION, YES);
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
    //mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
    //	mpc.set(CG_USE_OPENMP, YES);
    // NOTE: This is crucial for export of MHE!
    //mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
    mpc.set(FIX_INITIAL_STATE, YES);

    if (mpc.exportCode("simple_mpc_export") != SUCCESSFUL_RETURN)
            exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;
}
