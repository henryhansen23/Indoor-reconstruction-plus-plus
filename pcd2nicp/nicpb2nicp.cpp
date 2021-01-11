//
// nicpb2nicp
// Convert a binary .nicp file to textual .nicp
//
// nicp is the input for the nicp tools https://github.com/yorsh87/nicp
//
// Carsten Griwodz
//

#include <string>
#include <fstream>
#include <iostream>
#include <iomanip> // setprecision

#include <cstdlib>
#include <cstring>

using namespace std;

bool readFirstline( istream& istr, size_t& numpoints )
{
    char buf[1024];
    istr.getline( buf, 1024 );

    istringstream rstr( buf );
    string cc;
    bool   binary;
    
    rstr >> cc;
    if( cc != "NICPCLOUD" )
    {
        cerr << "Input file does not start with the CC NICPCLOUD." << endl;
        return false;
    }

    rstr >> numpoints >> binary;

    if( binary != 1 )
    {
        cerr << "Input file is already in test format, nothing to do." << endl;
        return false;
    }

    return true;
}

bool readSecondLine( istream& istr, int* transform )
{
    char buf[1024];
    istr.getline( buf, 1024 );

    istringstream rstr( buf );

    rstr >> transform[0] >> transform[1] >> transform[2] >> transform[3] >> transform[4] >> transform[5];

    cerr << "Transform line contains "
         << transform[0] << " " << transform[1] << " " << transform[2] << " "
         << transform[3] << " " << transform[4] << " " << transform[1] << endl;

    return true;
}

void usage( const char* name )
{
    cerr << "Usage: " << name << " [options] <binary-in>.nicp <ascii-out>.nicp" << endl
         << endl
         << "       Takes a binary input file in NICP format and converts it" << endl
         << "       to a NICP file in text format." << endl
         << "options:" << endl
         << "       -b - output as binary NICP file" << endl
         << endl;
    exit(-1);
}

int main(int argc, char **argv)
{
    const char* progname      = argv[0];
    bool        binary_output = false;

    if( argc == 4 )
    {
        if( string(argv[1]) == "-b" )
        {
            argc--;
            argv++;
            binary_output = true;
        }
        else
        {
            usage( progname );
        }
    }

    if (argc != 3)
    {
        usage( progname );
    }

    string infile_name( argv[1] );
    string outfile_name( argv[2] );

    ifstream istr( infile_name );
    if( ! istr.good() )
    {
        cerr << "Failed to open input file named " << infile_name << " to read NICP point cloud input" << endl;
        return -1;
    }

    ofstream ostr( outfile_name );

    if( ! ostr.good() )
    {
        cerr << "Failed to open output file named " << outfile_name << " to write NICP point cloud output" << endl;
        return -1;
    }

    size_t numPoints = 0;

    if( ! readFirstline( istr, numPoints ) )
    {
        return -1;
    }
    cerr << "Input file contains " << numPoints << " samples." << endl;

    if( binary_output )
        ostr << "NICPCLOUD " << numPoints << " 1" << endl;
    else
        ostr << "NICPCLOUD " << numPoints << " 0" << endl;

    int transform[6];

    if( ! readSecondLine( istr, transform ) )
    {
        return -1;
    }

    ostr << transform[0] << " " << transform[1] << " " << transform[2] << " "
         << transform[3] << " " << transform[4] << " " << transform[1] << " "
         << endl;

    cerr << "Reading " << numPoints << " points." << endl;

    for( size_t i=0; i<numPoints; i++ )
    {
        float values[4+4+24];
        const size_t sz = (4+4+24)*sizeof(float);
        istr.read( (char*)values, sz );
        if( !istr.good() )
        {
            cerr << "Failed to read point set " << i << " from the input file" << endl;
            break;
        }

        if( binary_output )
        {
            ostr.write( (char*)values, sz );
        }
        else
        {
            ostr << "POINTWITHSTATS";
            for( int j=0; j<3; j++ )
            {
                float v = values[j];
                if( std::abs(v) < 0.000001 ) v = 0;
                ostr << " " << setprecision(3) << v;
            }
            for( int j=0; j<3; j++ )
            {
                float v = values[4+j];
                if( std::abs(v) < 0.000001 ) v = 0;
                ostr << " " << setprecision(3) << v;
            }
            for( int r=0; r<4; r++ )
            {
                for( int c=0; c<4; c++ )
                {
                    float v = values[8+r+c*4];
                    if( std::abs(v) < 0.000001 ) v = 0;
                    ostr << " " << setprecision(3) << v;
                }
            }

            ostr << endl;
        }
    }
    if( istr.good() )
    {
        size_t rest = 0;
        while( istr.good() )
        {
            float in;
            istr >> in;
            rest++;
        }
        cerr << "Read " << rest << " floats from the input after the expected nunmber of samples."
             << endl;
    }

    return 0;
}

