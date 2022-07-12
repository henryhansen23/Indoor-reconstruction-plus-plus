#include "reco_types.h"

std::ostream& operator<<( std::ostream& ostr, const quat_t& q )
{
    ostr << "(" << q.w() << ", "
                << q.x() << ", "
                << q.y() << ", "
                << q.z() << ")";
    return ostr;
}

