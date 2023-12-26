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

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct seen_physical_extents_c seen_physical_extents_t;

seen_physical_extents_t* new_seen_physical_extents(void);
void delete_seen_physical_extents(seen_physical_extents_t* instance);

// memory management is automatic
seen_physical_extents_t* fetch_or_create_seen_physical_extents_for_dev_id(uint64_t dev_id);
void clear_automatic_cache(void);

uint64_t seen_physical_extents_get_total_overlap_and_insert(seen_physical_extents_t* instance, const char* path);


#ifdef __cplusplus
}
#endif