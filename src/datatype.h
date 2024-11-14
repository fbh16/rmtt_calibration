#pragma once

#define DRONE_ID 1

#if DRONE_ID == 1
    // #define FOCAL 914.906
    double F1 = 914.88132800541325;
    double F2 = 914.92976110902441;
    double U0 = 489.97683918793922;
    double V0 = 362.16172014200299;

    double K1 = 6.0594966908665794e-03;
    double K2 = -3.5065358890188622e-02;
    double Tx = 0.0428637;
    double Ty = -0.00237436;
    double Tz = -0.0735719;
    double Qx = 0.537014;
    double Qy = -0.531148;
    double Qz = 0.458368;
    double Qw = -0.468398;

#elif DRONE_ID == 2
    // #define FOCAL 917.637
    #define F1 914.88132800541325
    #define F2 914.92976110902441
    #define U0 489.97683918793922
    #define V0 362.16172014200299

    #define K1 1.5998754266660215e-02
    #define K2 -1.8561517886118395e-01
    #define T1 0.0428637
    #define T2 -0.00237436
    #define T3 -0.0735719
    #define QX 0.537014
    #define QY -0.531148
    #define QZ 0.458368
    #define QW -0.468398

#elif DRONE_ID == 3
    // #define FOCAL 916.284
    #define F1 914.88132800541325
    #define F2 914.92976110902441
    #define U0 489.97683918793922
    #define V0 362.16172014200299

    #define K1 -1.7567251806403478e-02
    #define K2 5.9906133698215652e-02
    #define T1 0.0428637
    #define T2 -0.00237436
    #define T3 -0.0735719
    #define QX 0.537014
    #define QY -0.531148
    #define QZ 0.458368
    #define QW -0.468398

#else
    #error "Unsupported ID"
#endif



#define DEBUG false