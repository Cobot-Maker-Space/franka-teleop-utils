/*
franka-utils
Copyright (C) 2025  Cobot Maker Space, University of Nottinghm

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <kj/io.h>

class MemoryOutputStream : public kj::OutputStream
{
public:
	MemoryOutputStream(kj::byte* buffer);
	~MemoryOutputStream() noexcept(false);
	void write(kj::ArrayPtr<const kj::byte> data);
	void write(kj::ArrayPtr<const kj::ArrayPtr<const kj::byte>> pieces);
	void write(const void* buffer, size_t size);
	size_t getSize();
	void reset();
private:
	kj::byte* buffer;
	size_t pos = 0;
};
