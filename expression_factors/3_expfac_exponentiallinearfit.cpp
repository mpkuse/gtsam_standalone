// given observations (x0,y0), (x1,y1), ..., (xn,yn) fit for params a, b
//      y = exp( ax+ b )
#include<iostream>
#include <vector>
#include <random>

#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
using namespace std;
using namespace gtsam;

void generate_data( int N, vector<double>& x, vector<double>& y )
{
    x.clear(); y.clear();

    double secret_a = 2.0;
    double secret_b = 1.0; //1.0;

    std::default_random_engine generator;
    double mean = 0.0, stddev = 3;
    std::normal_distribution<double> dist(mean, stddev);
    double noise = 0.0;

    cout << "secret_a=" << secret_a << "\t";
    cout << "secret_b=" << secret_b << "\t";
    cout << endl;
    cout << "noise: (mean,stddev)=" << mean << ", " << stddev << endl;

    for( int i=0 ; i<N ; i++ )
    {
        // noise = dist( generator );
        noise = 0.0;
        double _x = drand48() * 5;
        double _y = exp( secret_a * _x + secret_b + noise );

        x.push_back( _x ); y.push_back( _y );
    }
}

void print_vec( string msg, const vector<double>& X )
{
    cout << msg << ": size=" << X.size() << endl;
    for( int i=0 ; i<X.size() ; i++ )
    {
        cout << X[i] << ", ";
        if( i%10 == 0 && i > 0  )
            cout << endl;
    }
    cout << endl;
}

double predict_exp(
    const double& a, const double& b, const double x,
    OptionalJacobian<1, 1> H_a = boost::none,
    OptionalJacobian<1, 1> H_b = boost::none,
    OptionalJacobian<1, 1> H_x = boost::none
    )
{
    double f = exp( a*x + b );
    if( H_a ) *H_a = ( Vector(1) << x*f ).finished() ;
    if( H_b ) *H_b = ( Vector(1) << f ).finished() ;
    if( H_x ) *H_x = ( Vector(1) << a*f ).finished() ;

    return f;
}

int main()
{
    // generate data
    vector<double> X, Y;
    int N=25;
    generate_data( N, X, Y );

    print_vec( "X", X );
    print_vec( "Y", Y );

    // gtsam defs
    ExpressionFactorGraph graph;
    // Expression<double> a('a', 1), b('b', 1); // opt variable

    auto n_model =gtsam::noiseModel::Isotropic::Sigma(1, 0.5); //< note that these noise factors depend on dimension of residue function

    for( int i=0 ; i<N ; i++ )
    {
        auto h = Expression<double>( &predict_exp,
            Expression<double>( Symbol('a') ), Expression<double>( Symbol('b') ), Expression<double>(X[i]) );
        graph.addExpressionFactor( h, Y[i], n_model);
    }


    Values initialEstimate;
    initialEstimate.insert(  Symbol('a'), 2.2 );
    initialEstimate.insert(  Symbol('b'), 1.2 );


    // solver
    GaussNewtonOptimizer optimizer( graph, initialEstimate );
    Values result = optimizer.optimize();
    result.print( "Final result:\n" );

}
