// HTTPRepository.hxx - plain HTTP TerraSync remote server client
//
// Copyright (C) 2016  James Turner <zakalawe@mac.com>
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.


#ifndef SG_IO_HTTP_REPOSITORY_HXX
#define SG_IO_HTTP_REPOSITORY_HXX

#include <simgear/io/AbstractRepository.hxx>

namespace simgear  {

class HTTPRepoPrivate;

class HTTPRepository : public AbstractRepository
{
public:

    HTTPRepository(const SGPath& root, HTTP::Client* cl);
    virtual ~HTTPRepository();

    virtual SGPath fsBase() const;

    virtual void setBaseUrl(const std::string& url);
    virtual std::string baseUrl() const;

    virtual HTTP::Client* http() const;

    virtual void update();

    virtual bool isDoingSync() const;

    virtual ResultCode failure() const;
private:
    bool isBare() const;

    std::auto_ptr<HTTPRepoPrivate> _d;
};

} // of namespace simgear

#endif // of HTTPRepository