/*
* Copyright (c) 2023, Raspberry Pi Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions, and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions, and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the copyright holder nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _UTIL_H
#define _UTIL_H

#include <stdint.h>
#include <stddef.h>


#define INVALID_ADDRESS ((uint64_t)~0)
#define ROUND_UP(n, d) ((((n) + (d) - 1) / (d)) * (d))
#define UNUSED(x) (void)(x)

typedef struct dt_subnode_iter *DT_SUBNODE_HANDLE;
char *read_text_file(const char *fname, size_t *plen);

void *read_file(const char *fname, size_t *plen);

void dt_set_path(const char *path);

char *dt_read_prop(const char *node, const char *prop, size_t *len);

uint32_t *dt_read_cells(const char *node, const char *prop, unsigned *num_cells);

uint64_t dt_extract_num(const uint32_t *cells, int size);

uint64_t dt_read_num(const char *node, const char *prop, size_t size);

uint32_t dt_read_u32(const char *node, const char *prop);

uint64_t dt_parse_addr(const char *node);

void dt_free(void *value);

DT_SUBNODE_HANDLE dt_open_subnodes(const char *node);
const char *dt_next_subnode(DT_SUBNODE_HANDLE handle);
void dt_close_subnodes(DT_SUBNODE_HANDLE handle);

#endif
