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

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cerr << "Usage: " << argv[0] << " <binary-in>.nicp <ascii-out>.nicp" << endl
             << "       Takes a binary input file in NICP format and converts it" << endl
             << "       to a NICP file in text format." << endl;
        return -1;
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

    ostr << "NICPCLOUD " << numPoints << " 0" << endl;

    int transform[6];
    istr >> transform[0] >> transform[1] >> transform[2] >> transform[3] >> transform[4] >> transform[1];

    cerr << "Transform line contains "
         << transform[0] << " " << transform[1] << " " << transform[2] << " "
         << transform[3] << " " << transform[4] << " " << transform[1] << endl;

    ostr << transform[0] << " " << transform[1] << " " << transform[2] << " "
         << transform[3] << " " << transform[4] << " " << transform[1] << endl;

    cerr << "Reading " << numPoints << " points." << endl;

    for( size_t i=0; i<numPoints; i++ )
    {
        float values[3+3+16];
        const size_t sz = (3+3+16)*sizeof(float);
        istr.read( (char*)values, sz );
        if( !istr.good() )
        {
            cerr << "Failed to read point set " << i << " from the input file" << endl;
            break;
        }

        ostr << "POINTWITHSTATS";
        for( int j=0; j<3+3+16; j++ )
        {
            float v = values[j];
            if( std::abs(v) < 0.000001 ) v = 0;
            ostr << " " << setprecision(3) << v;
        }
        ostr << endl;
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

