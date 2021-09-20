#include <iostream>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
// #include <gtsam/linear/NoiseModel.h>
#include <boost/make_shared.hpp>
#include <gtsam/geometry/Pose2.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

int main()
{
    std::cout << "jlblanco_1\n" << std::endl;
    gtsam::NonlinearFactorGraph graph; 

    // Add a gaussian prior 
    gtsam::Pose2 priormean( 0,0, 0 ); 
    auto priornoise = gtsam::noiseModel::Diagonal::Sigmas( gtsam::Vector3(.3,.3,.1)); 
    auto priorFactor = gtsam::PriorFactor<gtsam::Pose2>(1,priormean, priornoise);
    graph.add( priorFactor );

    
    // Odometgry factors 
    gtsam::Pose2 odommean( 2.0,0, 0 ); 
    auto odomnoise = gtsam::noiseModel::Diagonal::Sigmas( gtsam::Vector3(.3,.3,.1)); 
    auto fac1 = gtsam::BetweenFactor<gtsam::Pose2>( 1,2, odommean, odomnoise );
    auto fac2 = gtsam::BetweenFactor<gtsam::Pose2>( 2,3, odommean, odomnoise );
    graph.add( fac1 );
    graph.add( fac2 );
    graph.print( "graph: ");


    // Initial values 
    gtsam::Values initial; 
    initial.insert( 1, gtsam::Pose2( 0.1,0.2,0.01) );
    initial.insert( 2, gtsam::Pose2( 2.2,0.1,0.1) );
    initial.insert( 3, gtsam::Pose2( 4.1,0.,0.05) );
    initial.print( "initial: ");

    // Solve
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer( graph, initial).optimize( );
    result.print( "result: ");

    // Query the marginals
    gtsam::Marginals marginals(graph, result);
    std::cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << std::endl;
    std::cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << std::endl;
    std::cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << std::endl;
}