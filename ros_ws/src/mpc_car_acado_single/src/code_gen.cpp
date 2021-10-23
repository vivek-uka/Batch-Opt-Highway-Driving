#include <acado_code_generation.hpp>
#include <iostream>

using namespace std;
int main( )
{
	USING_NAMESPACE_ACADO
	string path ="/home/vivek/On-Codes/Backup/Batch_traj_opt/ros_ws/src/mpc_car_acado_single";
	// Variables:

	double lx = 5.6/2, ly = 3.0/2;
	double aa = (lx + lx) * (lx + lx);
	double bb = (ly + ly) * (ly + ly);
		
	OnlineData xo_1;
	OnlineData yo_1;
	OnlineData xo_2;
	OnlineData yo_2;
	OnlineData xo_3;
	OnlineData yo_3;
	OnlineData xo_4;
	OnlineData yo_4;
	OnlineData xo_5;
	OnlineData yo_5;
	OnlineData xo_6;
	OnlineData yo_6;

	// Variables:
	DifferentialState   x    ;  // pos
	DifferentialState   y    ;  // pos 
	DifferentialState   theta  ;  // yaw
	DifferentialState 	v; 		// linear vel
	DifferentialState 	w;

	Control             j    ;  // angular acc
	Control 			a	;	// linear acc
			
	// Model equations:
	DifferentialEquation f; 

	f << dot( x ) == v * cos(theta);
	f << dot( y ) == v * sin(theta);
	f << dot( theta ) == w;
	f << dot( v ) == a;
	f << dot( w ) == j;
	
	
	Expression dist1 = sqrt(pow(x - xo_1, 2) + pow(y - yo_1, 2));//(pow(x - xo_1, 2)/aa + pow(y - yo_1, 2)/bb) ;
	Expression dist2 = sqrt(pow(x - xo_2, 2) + pow(y - yo_2, 2));//(pow(x - xo_2, 2)/aa + pow(y - yo_2, 2)/bb) ;
	Expression dist3 = sqrt(pow(x - xo_3, 2) + pow(y - yo_3, 2));//(pow(x - xo_3, 2)/aa + pow(y - yo_3, 2)/bb) ;
	Expression dist4 = sqrt(pow(x - xo_4, 2) + pow(y - yo_4, 2));//(pow(x - xo_4, 2)/aa + pow(y - yo_4, 2)/bb) ;
	Expression dist5 = sqrt(pow(x - xo_5, 2) + pow(y - yo_5, 2));//(pow(x - xo_5, 2)/aa + pow(y - yo_5, 2)/bb) ;
	Expression dist6 = sqrt(pow(x - xo_6, 2) + pow(y - yo_6, 2));//(pow(x - xo_6, 2)/aa + pow(y - yo_6, 2)/bb) ;
	
	double prox = 8.0;//4.0;//3.5;
	Expression obs1 = (1/( pow(x - xo_1, 2)/aa + pow(y - yo_1, 2)/bb )) * (1 - (dist1 - prox)/(0.00001 + sqrt(pow((dist1 - prox),2))));
	Expression obs2 = (1/( pow(x - xo_2, 2)/aa + pow(y - yo_2, 2)/bb )) * (1 - (dist2 - prox)/(0.00001 + sqrt(pow((dist2 - prox),2))));
	Expression obs3 = (1/( pow(x - xo_3, 2)/aa + pow(y - yo_3, 2)/bb )) * (1 - (dist3 - prox)/(0.00001 + sqrt(pow((dist3 - prox),2))));
	Expression obs4 = (1/( pow(x - xo_4, 2)/aa + pow(y - yo_4, 2)/bb )) * (1 - (dist4 - prox)/(0.00001 + sqrt(pow((dist4 - prox),2))));
	Expression obs5 = (1/( pow(x - xo_5, 2)/aa + pow(y - yo_5, 2)/bb )) * (1 - (dist5 - prox)/(0.00001 + sqrt(pow((dist5 - prox),2))));
	Expression obs6 = (1/( pow(x - xo_6, 2)/aa + pow(y - yo_6, 2)/bb )) * (1 - (dist6 - prox)/(0.00001 + sqrt(pow((dist6 - prox),2))));
	
	

	// Reference functions and weighting matrices:
	Function h, hN;
	h << x << y << v << a << j << obs1 << obs2 << obs3 << obs4 << obs5 << obs6;				
	hN << x << y << theta << w;

	// Provide defined weighting matrices:
	BMatrix W = eye<bool>( h.getDim() );
	BMatrix WN = eye<bool>( hN.getDim() );
		

	OCP ocp(0.0, 8.0, 100); // start, final, steps
	ocp.setNOD(12);
	ocp.subjectTo( f );
	
	// ocp.subjectTo(-pow(x - xo_1, 2)/aa - pow(y - yo_1, 2)/bb <= -1.00000000000001);
	// ocp.subjectTo(-pow(x - xo_2, 2)/aa - pow(y - yo_2, 2)/bb <= -1.00000000000001);
	// ocp.subjectTo(-pow(x - xo_3, 2)/aa - pow(y - yo_3, 2)/bb <= -1.00000000000001);
	// ocp.subjectTo(-pow(x - xo_4, 2)/aa - pow(y - yo_4, 2)/bb <= -1.00000000000001);
	// ocp.subjectTo(-pow(x - xo_5, 2)/aa - pow(y - yo_5, 2)/bb <= -1.00000000000001);
	// ocp.subjectTo(-pow(x - xo_6, 2)/aa - pow(y - yo_6, 2)/bb <= -1.00000000000001);
	
	ocp.minimizeLSQ(W, h);
	ocp.minimizeLSQEndTerm(WN, hN);
	// ocp.subjectTo( -5 <= w <= 5 );
	ocp.subjectTo( 0 <= v <= 24 );
	ocp.subjectTo(-4 <= a <= 4);
	ocp.subjectTo(-10 <= y <= 10);
	// ocp.subjectTo( -13 * M_PI/180 <= theta <= 13 * M_PI/180);
	// ocp.subjectTo(-2 <= j <= 2);
	
	
	// Export the code:
	OCPexport mpc( ocp );

	mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
	mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );
	mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
	mpc.set( NUM_INTEGRATOR_STEPS,        30              );
	mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX,         YES);
	mpc.set( QP_SOLVER,                   QP_QPOASES      );
// 	mpc.set( HOTSTART_QP,                 YES             );

	mpc.set( GENERATE_TEST_FILE,          NO             );
	mpc.set( GENERATE_MAKE_FILE,          NO             );
	mpc.set( GENERATE_MATLAB_INTERFACE,   NO             );
	mpc.set( GENERATE_SIMULINK_INTERFACE, NO             );
	

	if (mpc.exportCode( path + "/model/codegen" ) != SUCCESSFUL_RETURN)
	//  if (mpc.exportCode( "acado_threading" ) != SUCCESSFUL_RETURN)
  		exit( EXIT_FAILURE );

	mpc.printDimensionsQP( );

	return EXIT_SUCCESS;
}
