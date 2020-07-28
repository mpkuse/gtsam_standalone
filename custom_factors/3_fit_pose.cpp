/**
    Given 2 sets of points
    X: { (x0,y0,z0), (x1,y1,z1), ..., (xn,yn,zn) }
    X':{ (x0',y0',z0'), (x1',y1',z1'), ..., (xn',yn',zn') }

    Find the relative pose between the point sets.
    Pose : [ Rot, translation ]
*/


#include <iostream>
#include <vector>
#include <random>
using namespace std;


#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// #include <gtsam/linear/NoiseModel.h>
#include <boost/make_shared.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

using namespace gtsam;


//--- Eigen3
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;


class Utils {
public:
    /** @brief converts a euler angle representation to rotation matrix
    output rpy need to be in radians.
    */
    static Eigen::Matrix3d eulerZXY2Rotation( const Eigen::Vector3d& e  )
    {

        double phi   = e(0);
        double theta = e(1);
        double psi   = e(2);


        Eigen::Quaterniond q = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY());

        Eigen::Matrix3d Rzxy = q.toRotationMatrix();

        return Rzxy;
    }



    /** @brief converts a rotation matrix to ZXY Euler angles.
    output rpy need to be in radians.
    */
    static Eigen::Vector3d rotation2EulerZXY(const Eigen::Matrix3d& R)
    {
      #if 0
      double phi = asin(R(2, 1));
      // double theta = atan(-R(2, 0) / R(2, 2));
      // double psi = atan(-R(0, 1) / R(1, 1));
      double theta = atan2(-R(2, 0), R(2, 2));
      double psi = atan2(-R(0, 1), R(1, 1));

      return Eigen::Vector3d(phi, theta, psi);
      #endif


      #if 1
      const double pi = 3.14159265359;
      double thetaX, thetaY, thetaZ;
      if( R(2,1)< +1 ){

      	if( R(2,1) > -1 )
      	{
      		thetaX=asin(R(2,1)) ;
      		thetaZ=atan2(-R(0,1) , R(1,1) ) ;
      		thetaY=atan2(-R(2,0) , R(2,2) ) ;
      	}
      	else //  r21 =−1
      	{
      		// Not a  unique  s o l u t i o n :   thetaY−thetaZ = atan2 ( r02 , r00 )
      		thetaX =-pi /2;
      		thetaZ =-atan2( R(0,2) , R(0,0) ) ;
      		thetaY = 0;
      	}
      }
      else //  r21 = +1
      {
      	// Not a  unique  solution:   thetaY + thetaZ = atan2 ( r02 , r00 )
      	thetaX = +pi /2;
      	thetaZ = atan2(R(0,2),R(0,0)) ;
      	thetaY = 0;
      }

      double phi = thetaX;
      double theta = thetaY;
      double psi = thetaZ;
      return Eigen::Vector3d(phi, theta, psi);

      #endif


    }


    static string prettyPrintPose( const Eigen::Matrix4d& M )
    {
        stringstream ss;

        Matrix3d R = M.topLeftCorner<3,3>();
        Vector3d rpy_rads = rotation2EulerZXY( R );

        double PI = 3.14159265359;
        ss << "rpy_deg: " << rpy_rads.transpose() / PI * 180. << ", ";
        ss << "t: " << M.col(3).topRows(3).transpose() ;
        return ss.str();
    }

    static void generate_data( int N, vector< Point3 > &X, vector< Point3 > &Y )
    {
        X.clear(); Y.clear();

        //-- make a pose
        Vector3d rpy_deg = Vector3d( 0,0,1. );
        Vector3d translation = Vector3d( 0, 0.0, 0.5 );
        double PI = 3.14159265359;
        Matrix3d R = eulerZXY2Rotation( rpy_deg/180.0*PI );
        cout << "construct secret pose rpy_deg=" << rpy_deg.transpose() << "\t"
             << "trans=" << translation.transpose() << endl;

        auto R_gtsam = Rot3( R );
        auto t_gtsam = Point3( translation );
        // Pose3 T( Rot3(R), Point3(translation) );
        Pose3 T(R_gtsam, t_gtsam);
        cout << "secret Pose3: " << T << endl;

        //-- noise
        std::default_random_engine generator;
        double mean = 0.0, stddev = 5;
        cout << "mean: " << mean << "\t" << "stddev: " << stddev << endl;
        std::normal_distribution<double> dist(mean, stddev);


        double f = 100.0;
        cout << "x in range : (-" << f << ", " << f << ")" << endl;

        for( int i=0 ; i<N ; i++ )
        {
            double noise_x = dist(generator);
            double noise_y = dist(generator);
            double noise_z = dist(generator);
            // Point3 noise = Point3( noise_x, noise_y, noise_z );
            Point3 noise = Point3( 0,0,0 );
            Point3 _tmp_x( drand48()*f, drand48()*f, drand48()*f );
            Point3 _tmp_y = T * _tmp_x + noise;

            X.push_back( _tmp_x );
            Y.push_back( _tmp_y );
        }

        cout << "...done, X.size() " << X.size() << "\tY.size() " << Y.size() << endl;

    }
};

class PoseFactor: public NoiseModelFactor1<Pose3> {
    typedef NoiseModelFactor1<Pose3> Base;


    // measurements
    Point3 X_i, Y_i;

public:
    PoseFactor(
        const SharedNoiseModel& model,
        const Key& key,
        const Point3 _X_i, const Point3 _Y_i
    ): Base( model, key), X_i( _X_i), Y_i( _Y_i)
    {
    }



    // evaluate error
    Vector evaluateError(const Pose3& Tr, boost::optional<gtsam::Matrix&> H =boost::none) const {

        // if( H ) *H = ( Vector(1) << 2.0).finished() ;

        // return Y_i - Tr.rotation()*Y_i - Tr.translation();
        return Tr.transform_from( X_i, H ) - Y_i;
    }


};


int main()
{
    cout << "fit SE3\n";

    cout << "---generate_data\n";
    vector< Point3 > X, Y;
    Utils::generate_data( 100, X, Y );


    // create graph
    cout << "---create non-linear factor graph\n";
    NonlinearFactorGraph graph;


    // SharedDiagonal measurementNoise = gtsam::noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.5));
    SharedDiagonal measurementNoise = gtsam::noiseModel::Isotropic::Sigma(3, .9 );


    // add factor to graph
    Symbol key('T', 1);
    key.print("key: ");
    for( int i=0 ; i<X.size() ; i++ ) {
        if( i==0 ) {
            cout << "add PoseFactor obs=X_"<< i << "  Y_" << i << "\t";
            key.print("opt_var:");
            graph.emplace_shared<PoseFactor>( measurementNoise, key, X[i], Y[i]  );
        }
        if( i==X.size() -1 )
            cout << "....i=(0, " << X.size() << ")\n";
    }


    Values initial;
    initial.insert( key, Pose3( Rot3(), Point3(0,0,0.45) ) );
    cout << "---initial estimate " << endl;
    initial.print( "initial estimates:\n");




    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Final result:\n");
    Pose3 result_Pose3 = result.at<Pose3>( key );
    Matrix4d result_T = result_Pose3.matrix();

    cout << "result_T:\n" << Utils::prettyPrintPose(result_T) << endl;




}
