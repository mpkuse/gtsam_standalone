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


void generate_data( int N, vector<Point2>& X )
{
    X.clear();

    std::default_random_engine generator;
    double mean = 0.0, stddev = 3;
    std::normal_distribution<double> dist(mean, stddev);
    double noise = 0.0;
    cout << "noise: (mean,stddev)=" << mean << ", " << stddev << endl;

    for( int i=0 ; i<N ; i++ )
    {
        // noise = dist( generator );
        noise = 0.0;
        double _x = (int) (drand48() * 100);
        double _y = (int) (drand48() * 100);

        X.push_back( Point2(_x,_y) );
    }
}

void secret_transform( const vector<Point2>& X, vector<Point2>& Y )
{
    Pose2 secret_transform( 0.5, 0, 0 );

    Y.clear();
    for( int i=0 ; i<X.size() ; i++ )
    {
        Point2 tmp = secret_transform * X[i];
        Y.push_back( tmp );
    }
}

void print_vec( const string msg, const vector<Point2>&  _F )
{
    cout << msg << ": size=" << _F.size() << "\n";
    for( int i=0 ; i<_F.size() ; i++ )
    {
        cout << "\t" << i << ": (" <<  _F[i].x() << "," << _F[i].y() << ")";
        if( i%5 == 0 )
            cout << endl;
    }
    cout << endl;
}


// https://bitbucket.org/gtborg/gtsam/wiki/GTSAM_Expressions
// we will wrap this function (which can compute its local derivatives)
// to a expression function.
Point2 residue( const Pose2& Tr, const Point2 p,
        OptionalJacobian<2, 3> H_Tr = boost::none,
        OptionalJacobian<2, 2> H_p = boost::none
    )
{
    return Tr.transform_to( p, H_Tr, H_p);
    // return Tr * p;
}


int main()
{
    // generate data
    vector<Point2>X, Y;
    int N=5;
    generate_data( N, X );
    secret_transform( X, Y );

    print_vec( "X", X );
    print_vec( "Y", Y );


    // gtsam defs
    ExpressionFactorGraph graph;
    Expression<Pose2> T('T'); // opt variable

    auto n_model = noiseModel::Diagonal::Sigmas( Vector2(0.2,0.2) ); //< note that these noise factors depend on dimension of residue function
    for( int i=0 ; i<N; i++ )
    {
        auto h = Expression<Point2>( &residue, T, Expression<Point2>(X[i]) );
        graph.addExpressionFactor( h, Y[i], n_model );
    }
    graph.print( "Factor graph:\n");


    Values initialEstimate;
    initialEstimate.insert( 'T', Pose2(0,0,0) );
    initialEstimate.print( "initialEstimate:\n");

    // solver
    GaussNewtonOptimizer optimizer( graph, initialEstimate );
    Values result = optimizer.optimize();
    result.print( "Final result:\n" );

}
