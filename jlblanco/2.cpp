// Define a custom factor - easy to get Jacobian
#include <iostream>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose2.h>


class UnaryFactor: public gtsam::NoiseModelFactor1<gtsam::Pose2> {
public:
    //constructor 
    UnaryFactor( gtsam::Key k, gtsam::Vector2 _observation, const gtsam::SharedNoiseModel& model ):
        gtsam::NoiseModelFactor1<gtsam::Pose2>( model, k ),
        observation( _observation )
        { }

    // evaluateError
    gtsam::Vector evaluateError( const gtsam::Pose2& q, boost::optional<gtsam::Matrix&> H = boost::none ) const 
    {
        if( H ) {
            // jacobian
            (*H) = (gtsam::Matrix(2,3)<< 
                    1.0,0.0,0.0, 
                    0.0,1.0,0.0).finished();
        }

        return (gtsam::Vector(2) << q.x() - observation.x() , q.y() - observation.y()).finished();

    }

private:
    gtsam::Vector2 observation;
};

int main()
{
    std::cout << "jlblanco_2_customfactor\n" << std::endl;
    gtsam::NonlinearFactorGraph graph; 

    // Unary factors
    auto obs_noise = gtsam::noiseModel::Diagonal::Sigmas( gtsam::Vector2(.2,.2 )); 
    auto fac1 = UnaryFactor( 1, gtsam::Vector2(0,0), obs_noise );
    auto fac2 = UnaryFactor( 2, gtsam::Vector2(2,0), obs_noise );
    auto fac3 = UnaryFactor( 3, gtsam::Vector2(3,0), obs_noise );
    graph.add( fac1 );
    graph.add( fac2 );
    graph.add( fac3 );
    graph.print( "graph: " );

     // Initial values 
    gtsam::Values initial; 
    initial.insert( 1, gtsam::Pose2( 0.1,0.2,0.01) );
    initial.insert( 2, gtsam::Pose2( 2.2,0.1,0.1) );
    initial.insert( 3, gtsam::Pose2( 4.1,0.,0.05) );
    initial.print( "initial: ");

    // Solve
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer( graph, initial).optimize( );
    result.print( "result: ");

}