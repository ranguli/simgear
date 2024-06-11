#pragma once

namespace simgear
{

class ReportBadAllocGuard
{
public:
    ReportBadAllocGuard();
    virtual ~ReportBadAllocGuard();

    static bool isSet();
};


}
