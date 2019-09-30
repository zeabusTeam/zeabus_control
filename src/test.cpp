// FILE			: test.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, September 30 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  ref01   : https://www.boost.org/doc/libs/1_71_0/libs/numeric/ublas/doc/matrix_expression.html
//  ref02   : https://www.boost.org/doc/libs/1_71_0/libs/numeric/ublas/doc/matrix_proxy.html#matrix_row

// MACRO SET

// MACRO CONDITION

extern _ublas::matrix< double > inverse_characteristic;
extern _ublas::matrix< double > characteristic;
extern _ublas::matrix< double > target_summation;
extern _ublas::matrix< double > thruster_direction;
extern _ublas::matrix< double > thruster_distance;
extern _ublas::matrix< double > thruster_arm;

void check_direction()
{
    std::cout   << "MATRIX DIRECTION dimension ( " << thruster_direction.size1() 
                << " , " << thruster_direction.size2() << ")\n";
    for( unsigned row = 0 ; row < thruster_direction.size1()  ; row++ )
    {
        _ublas::matrix_row< 
                _ublas::matrix< double > > data ( thruster_direction , row );
        std::cout   << "thruster number " << row << " : " << data << "\n";
    }
    std::cout   << "\n";
} // check_direction

void check_distance()
{
    std::cout   << "MATRIX DIRECTION dimension ( " << thruster_distance.size1() 
                << " , " << thruster_distance.size2() << ")\n";
    for( unsigned row = 0 ; row < thruster_distance.size1()  ; row++ )
    {
        _ublas::matrix_row< _ublas::matrix< double > > data ( thruster_distance , row );
        std::cout   << "thruster number " << row << " : " << data << "\n";
    }
    std::cout   << "\n";
} // check_distance

void check_arm()
{
    std::cout   << "MATRIX DIRECTION dimension ( " << thruster_arm.size1() 
                << " , " << thruster_arm.size2() << ")\n";
    for( unsigned row = 0 ; row < thruster_arm.size1()  ; row++ )
    {
        _ublas::matrix_row< _ublas::matrix< double > > data ( thruster_arm , row );
        std::cout   << "thruster number " << row << " : " << data << "\n";
    }
    std::cout   << "\n";
} // check_arm

void check_characteristic()
{
    std::cout   << "MATRIX DIRECTION dimension ( " << characteristic.size1() 
                << " , " << characteristic.size2() << ")\n";
    std::cout   << "linear x : " << _ublas::row( characteristic , 0 ) << "\n"
                << "linear y : " << _ublas::row( characteristic , 1 ) << "\n"
                << "linear z : " << _ublas::row( characteristic , 2 ) << "\n"
                << "moment x : " << _ublas::row( characteristic , 3 ) << "\n"
                << "moment y : " << _ublas::row( characteristic , 4 ) << "\n"
                << "moment z : " << _ublas::row( characteristic , 5 ) << "\n\n";
} // check_characteristic();

void check_matrix()
{
    check_direction();
    check_distance();
    check_arm();
    check_characteristic();
} // check_matrix

