// borrowed from: https://github.com/borglab/gtsam/blob/develop/examples/Pose2SLAMExampleExpressions.cpp
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
using namespace std;
using namespace gtsam;

int main()
{
    // definations
    ExpressionFactorGraph graph;
    Expression<Pose2> x1(1), x2(2), x3(3); //< opt variables

    // prior on x1
    auto priorNoise = noiseModel::Diagonal::Sigmas( Vector3(0.3, 0.3, 1 ) );
    graph.addExpressionFactor( x1, Pose2(0,0,0), priorNoise );

    // define factor graph
    auto n_model = noiseModel::Diagonal::Sigmas( Vector3(0.2,0.2,0.1) ); //< note that these noise factors depend on dimension of residue function
    graph.addExpressionFactor( between(x1, x2), Pose2(2, 0, 0     ), n_model );
    graph.addExpressionFactor( between(x2, x3), Pose2(2, 0, M_PI_2), n_model );
    //                          ^^residue          ^^observation        ^^noise model
    graph.print( "Factor graph:\n");

    // initial estimates
    Values initialEstimate;
    initialEstimate.insert( 1, Pose2( 0.5,0, 0.2 ) );
    initialEstimate.insert( 2, Pose2( 2.3,0.1, -0.2 ) );
    initialEstimate.insert( 3, Pose2( 4.1,0.1, M_PI_2 ) );
    initialEstimate.print( "initial estimate:\n");

    // solver
    GaussNewtonOptimizer optimizer( graph, initialEstimate );
    Values result = optimizer.optimize();
    result.print( "Final result:\n" );

    // retrive result
    Pose2 r_x1 = result.at<Pose2>(3);
    r_x1.print( "r_x1:");
    // cout << "r_x1:" << r_x1 << endl;
}
