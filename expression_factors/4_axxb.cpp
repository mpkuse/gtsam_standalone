// Try to solve hand eye calibration problem, AX = XB in gtsam
// synthetic data generation>:
//
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

// generate random poses A_T_ai
void generate_track_a( const int N, vector<Pose2>& track_a )
{

    Pose2 a_T_0 = Pose2( 2.0, 1.0, M_PI / 10 );
    track_a.push_back( a_T_0 );
    for( int i=1 ; i<N; i++ )
    {
        double dx = 1.0; drand48() ;
        double dy = 0.0; drand48() ;
        double dtheta = 0.0; drand48() ;

        Pose2 del = Pose2( dx, dy, dtheta );

        Pose2 a_T_i = *( track_a.rbegin() ) * del ;
        track_a.push_back( a_T_i );
    }
}

// generate a random A_T_B, then generate B_T_bi using A_T_ai
void generate_track_b( const vector<Pose2>& track_a, vector<Pose2>& track_b )
{
    track_b.clear();
    Pose2 secret_A_T_B = Pose2( 1.0, 0.0, 0.0 );
    for( int i=0 ; i<track_a.size() ; i++ )
    {
        Pose2 B_T_bi = secret_A_T_B.inverse() * track_a[i] * secret_A_T_B;
        track_b.push_back( B_T_bi );
    }
}

void print_track( const string msg, vector<Pose2>& track )
{
    cout << msg << ": size=" << track.size() << endl;
    for( int i=0 ; i<track.size() ; i++ )
    {
        // cout << i << ":" << track[i] << endl;
        track[i].print( msg+"={"+to_string( i )+"} " );
    }
}

int main()
{
    // generate data
    vector<Pose2> A_T_ai, B_T_bi;
    generate_track_a( 12, A_T_ai );
    print_track( "A_T_ai", A_T_ai);
    generate_track_b( A_T_ai, B_T_bi );
    print_track( "B_T_bi", B_T_bi);

    // relative poses
    vector<Pose2> ai_T_aj, bi_T_bj;
    for( int i=1; i<A_T_ai.size() ; i++ )
    {
        ai_T_aj.push_back( A_T_ai[i-1].inverse() * A_T_ai[i] );
        bi_T_bj.push_back( B_T_bi[i-1].inverse() * B_T_bi[i] );
    }
    print_track( "ai_T_aj", ai_T_aj );
    print_track( "bi_T_bj", bi_T_bj );

    // something wrong in my understanding of hand-eye calib...come back to this at some later time :( 

    // gtsam defs


    // add expression factors to graph



    // initial estimates

    // solve

    // retrive result and verify

}
