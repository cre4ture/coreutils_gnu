/* du -- summarize device usage
   Copyright (C) 1988-2023 Free Software Foundation, Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.  */

#include <config.h>
#include "du_physical_extents.h"

#include <sstream>

#include <map>
#include <vector>
#include <string>
#include <limits>
#include <memory>

#include <linux/fiemap.h>
#include <linux/fs.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <error.h>

typedef u_int64_t usize;

namespace du::du_physical_extents {

  template<int count_>
  struct fiemap_with_extents
  {
    enum { count = count_ };
    fiemap header;
    fiemap_extent elements[count];

    fiemap_with_extents()
      : header{0}
      , elements{0}
    {
      header.fm_length = std::numeric_limits<usize>::max();
      header.fm_extent_count = count;
    }

    const fiemap_extent* begin() const {
      return &header.fm_extents[0];
    }

    const fiemap_extent* end() const {
      return &header.fm_extents[std::min<usize>(count, header.fm_mapped_extents)];
    }
  };

  struct range_t {
    usize start;
    usize end;
  };

  typedef std::pair<usize, std::string> error_type;

  class seen_physical_extents {
  public:
    error_type fetch_extents(const std::string& path);

    usize get_overlapping_and_insert(const range_t& range);

    std::pair<usize, std::vector<error_type>> get_total_overlap_and_insert(const std::string& path);

  private:
    std::map<usize, usize> ranges;
    fiemap_with_extents<8> fm;
  };

struct file_descriptor_raii
{
  int m_fd;

  file_descriptor_raii(const char* filename)
  : m_fd(open(filename, O_RDONLY))
  {}

  ~file_descriptor_raii() {
    close(m_fd);
  }
};

error_type seen_physical_extents::fetch_extents(const std::string& path)
{
  file_descriptor_raii f(path.c_str());

  auto err_code = ioctl(f.m_fd, FS_IOC_FIEMAP, &fm);
  if (err_code != 0) {
    std::ostringstream stream;
    stream << "Unable to FIEMAP (err_code: " << err_code << "): " << path;
    return std::make_pair(err_code, stream.str());
  }

  return std::make_pair(0, "");
}

  std::pair<usize, std::vector<error_type>>
  seen_physical_extents::get_total_overlap_and_insert(const std::string& path)
  {
    std::vector<error_type> errors;

    auto error = fetch_extents(path); // result stored in this->fm
    if (error.first != 0) {
      errors.push_back(error);
      return std::make_pair(0, errors);
    }

    usize total_overlapping = 0;

    for (const auto& extent: fm)
    {
      if (!(extent.fe_flags & FIEMAP_EXTENT_UNKNOWN) && // the record doesn't contain valid information (yet)
          !(extent.fe_flags & FIEMAP_EXTENT_DATA_INLINE) && // the data so less that is put as part of the metadata, fe invalid
            (extent.fe_flags & FIEMAP_EXTENT_SHARED)) // performance: only with this bit set, extents are relevant for us
      {
        auto range = range_t{
            start: extent.fe_physical,
            end: extent.fe_physical + extent.fe_length,
        };

        total_overlapping += get_overlapping_and_insert(range);
      }
    }

    return std::make_pair(total_overlapping, errors);
  }

  usize seen_physical_extents::get_overlapping_and_insert(const range_t& range)
  {
    auto same_or_before = ranges.upper_bound(range.start+1);

    auto need_new_entry = true;
    usize overlapping_sum = 0;
    if (same_or_before != ranges.begin())
    {
      auto& element = *--same_or_before;
      const auto start = element.first;
      auto& end = element.second;

      if (end >= range.end) {
          return range.end - range.start; // fully covered, no new entry needed
      }
      if (end >= range.start) {
          overlapping_sum += end - range.start;
          end = range.end;     // partially covered from begin.
                                // Extend existing entry.
          need_new_entry = false;
      }
    }

    if (need_new_entry) {
        // element before doesn't exist or doesn't overlap, insert new
        ranges.insert(std::make_pair(range.start, range.end));
    }

    usize current_pos = range.start+1;
    while(true) {
      auto after = ranges.lower_bound(current_pos);

      if (after != ranges.end()) {
        auto& element = *after;
        const auto start = element.first;
        auto& end = element.second;

        if (start >= range.end) {
              return overlapping_sum; // fully outside, done
          }

          if (end > range.end) {
              overlapping_sum += range.end - start;
              auto new_start = range.end;
              auto new_end = end;
              ranges.insert(std::make_pair(new_start, new_end));
              ranges.erase(start);
              return overlapping_sum; // partially outside, adapt, done
          }

          overlapping_sum += end - start; // fully inside, remove, continue
          current_pos = end;
          ranges.erase(start);
      }
      else {
        return overlapping_sum;
      }
    }
  }
}

struct seen_physical_extents_c : du::du_physical_extents::seen_physical_extents {};

seen_physical_extents_t* new_seen_physical_extents() {
  return new seen_physical_extents_c;
}

void delete_seen_physical_extents(seen_physical_extents_t* instance) {
  delete instance;
}

static std::map<uint64_t, std::unique_ptr<seen_physical_extents_t>> s_map_instance;

void clear_automatic_cache(void) {
  s_map_instance.clear();
}

seen_physical_extents_t* fetch_or_create_seen_physical_extents_for_dev_id(uint64_t dev_id)
{
  auto it = s_map_instance.find(dev_id);
  if (it == s_map_instance.end())
  {
    s_map_instance[dev_id] = std::make_unique<seen_physical_extents_t>();
  }

  return s_map_instance[dev_id].get();
}

uint64_t seen_physical_extents_get_total_overlap_and_insert(seen_physical_extents_t* instance, const char* path)
{
  auto result = instance->get_total_overlap_and_insert(path);

  for (const auto& error_entry: result.second) {
    error(0, error_entry.first, "\"%s\"", error_entry.second.c_str());
  }

  return result.first;
}