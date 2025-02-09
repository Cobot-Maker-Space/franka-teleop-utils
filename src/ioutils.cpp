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
#include <ioutils.h>

MemoryOutputStream::MemoryOutputStream(kj::byte* buffer) : buffer(buffer) {}

MemoryOutputStream::~MemoryOutputStream() {}

void MemoryOutputStream::write(kj::ArrayPtr<const kj::byte> data) {
  memcpy(buffer + pos, data.begin(), data.size());
  pos += data.size();
}

void MemoryOutputStream::write(kj::ArrayPtr<
  const kj::ArrayPtr<const kj::byte>> pieces) {
  for (size_t i = 0; i < pieces.size(); i++) {
    write(pieces[i]);
  }
}

void MemoryOutputStream::write(const void* buffer, size_t size) {
  memcpy(this->buffer + pos, buffer, size);
  pos += size;
}

size_t MemoryOutputStream::getSize() {
  return pos;
}

void MemoryOutputStream::reset() {
  memset(buffer, 0, pos);
  pos = 0;
}
