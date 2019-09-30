// FILE			: thruster.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, September 30 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  In direction we use right have rule to assing direction
//  And control ENU system base to coordinate because rviz use ENU system or right up

// REFERENCE
//  ref01   : https://www.boost.org/doc/libs/1_71_0/libs/qvm/doc/html/index.html#determinant
//  ref02   : https://math.stackexchange.com/questions/1335693/invertible-matrix-of-non-square-matrix
//  ref03   : https://www.boost.org/doc/libs/1_71_0/libs/numeric/ublas/doc/matrix.html
//  ref04   : https://www.boost.org/doc/libs/1_71_0/libs/numeric/ublas/doc/matrix_expression.html#matrix_expression
//  ref05   : https://www.boost.org/doc/libs/1_71_0/libs/numeric/ublas/doc/matrix_proxy.html#matrix_row
//  ref06   : https://www.boost.org/doc/libs/1_64_0/libs/numeric/ublas/doc/vector_expression.html

// MACRO SET

// MACRO CONDITION

extern _ublas::matrix< double > inverse_characteristic;
extern _ublas::matrix< double > characteristic;
extern _ublas::matrix< double > target_summation;
extern _ublas::matrix< double > thruster_direction;
extern _ublas::matrix< double > thruster_distance;
extern _ublas::matrix< double > thruster_arm;

const static double cos45 = cos( zeabus::math::PI / 4 );
const static double sin45 = sin( zeabus::math::PI / 4 );

static double direction[8][3] = 
{
    0 , 0 , 1 ,
    0 , 0 , 1 ,
    0 , 0 , 1 ,
    0 , 0 , 1 ,
    cos45 , -sin45 , 0 ,
    cos45 , sin45 , 0 ,
    -cos45 , -sin45 , 0 ,
    -cos45 , sin45 , 0  
};

static double distance[8][3] = 
{
    +0.440 , +0.320 , -0.023 ,
    +0.440 , -0.320 , -0.023 ,
    -0.440 , +0.320 , -0.023 ,
    -0.440 , -0.320 , -0.023 ,
    +0.460 , +0.340 , -0.023 ,
    +0.460 , -0.340 , -0.023 ,
    -0.460 , +0.340 , -0.023 ,
    -0.460 , -0.340 , -0.023 ,
};


// below function use setup matrix for calcualte
void update_characteristic()
{
    for( unsigned int run_thruster = 0 ; run_thruster < thruster_number ; run_thruster++ )
    {
        // Put data about direction to characteristic
        characteristic( 0 , run_thruster ) = thruster_direction( run_thruster , 0 );
        characteristic( 1 , run_thruster ) = thruster_direction( run_thruster , 1 );
        characteristic( 2 , run_thruster ) = thruster_direction( run_thruster , 2 );
        // Put data about arm of moment to characteristic
        characteristic( 3 , run_thruster ) = thruster_arm( run_thruster , 0 );
        characteristic( 4 , run_thruster ) = thruster_arm( run_thruster , 1 );
        characteristic( 5 , run_thruster ) = thruster_arm( run_thruster , 2 );

    } 
} // update_characteristic

void set_direction()
{
    for( unsigned int row = 0 ; row < thruster_direction.size1() ; row++ )
    {
        for( unsigned int column = 0 ; column < thruster_direction.size2() ; column++ )
        {
            thruster_direction( row , column ) = direction[ row ][ column ];
        }
    }
} // set_direction

void set_distance()
{
    for( unsigned int row = 0 ; row < thruster_distance.size1() ; row++ )
    {
        for( unsigned int column = 0 ; column < thruster_distance.size2() ; column++ )
        {
            thruster_distance( row , column ) = distance[ row ][ column ];
        }
    }
} // set_distance

void calculate_arm()
{
    for( unsigned int row = 0 ; row < thruster_number ; row++ )
    {
        thruster_arm( row , 0  ) = thruster_distance( row , 1 ) * thruster_direction( row , 2 )
            - thruster_distance( row , 2 ) * thruster_direction( row , 1 );

        thruster_arm( row , 1  ) = thruster_distance( row , 2 ) * thruster_direction( row , 0 )
            - thruster_distance( row , 0 ) * thruster_direction( row , 2 ); 

        thruster_arm( row , 2 ) = thruster_distance( row , 0 ) * thruster_direction( row , 1 )
            - thruster_distance( row , 1 ) * thruster_direction( row , 0 ); 
    }
} // calculate_arm

void set_default()
{
    set_direction();
    set_distance();
    calculate_arm();
    update_characteristic();
} // set_default
 
