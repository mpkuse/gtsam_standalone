#include <iostream>
#include <vector>
#include <random>
using namespace std;


#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// #include <gtsam/linear/NoiseModel.h>
#include <boost/make_shared.hpp>



using namespace gtsam;

//  this is the type of optimization variable
//                                          vvvvv
class LinearFactor: public NoiseModelFactor1<double> {
    typedef NoiseModelFactor1<double> Base;

    // measurements
    double x;
    double y;

public:
    LinearFactor(
        const SharedNoiseModel& model,  ///< Noise model (related ti weighting)
        const Key& key, ///< optimization variable
        const double _x, const double _y ///< observations
    ): Base(model,key), x(_x), y(_y) {

    }



    // evaluate error
    virtual Vector evaluateError(const double& a, boost::optional<Matrix&> H =
        boost::none) const {

        if( H ) *H = ( Vector(1) << -x ).finished() ;

        return ( Vector(1) << y - a*x ).finished();
    }

};

void generate_data( int N, vector<double>& x, vector<double>& y )
{
    x.clear(); y.clear();

    double secret_a = 12.0;
    double secret_b = 0.0; //1.0;

    std::default_random_engine generator;
    double mean = 0.0, stddev = 5;
    std::normal_distribution<double> dist(mean, stddev);
    double noise = 0.0;

    cout << "secret_a=" << secret_a << "\t";
    cout << "secret_b=" << secret_b << "\t";
    cout << endl;
    cout << "noise: (mean,stddev)=" << mean << ", " << stddev << endl;

    for( int i=0 ; i<N ; i++ )
    {
        noise = dist( generator );
        double _x = drand48() * 100;
        double _y = secret_a * _x + secret_b + noise;

        x.push_back( _x ); y.push_back( _y );
    }
}

int main()
{
    cout << "1 fitline....given several (x_i,y_i), try to fit y=ax\n";

    cout << "---generate_data\n";
    vector<double> x, y;
    generate_data( 10, x, y );


    // create graph
    NonlinearFactorGraph graph;


    // SharedDiagonal measurementNoise = gtsam::noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.5));
    SharedDiagonal measurementNoise = gtsam::noiseModel::Isotropic::Sigma(1, 0.5);

    // add factor to graph
    Symbol key('a', 1);
    key.print("key: ");
    for( int i=0 ; i<x.size() ; i++ )
        graph.emplace_shared<LinearFactor>( measurementNoise, key, x[i], y[i]  );

    Values initial;
    initial.insert( key, 3.0  );
    cout << "---initial estimate " << endl;
    initial.print( "initial estimates:\n");



    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Final result:\n");



}
