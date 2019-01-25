/*
 * Copyright (C) 2015  Mozilla Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pdu/pdu.h"
#include <assert.h>
#include <stdarg.h>
#include "log.h"

void
init_pdu(struct pdu* pdu, uint8_t service, uint8_t opcode)
{
  assert(pdu);

  pdu->service = service;
  pdu->opcode = opcode;
  pdu->len = 0;
}

size_t
pdu_size(const struct pdu* pdu)
{
  return sizeof(*pdu) + pdu->len;
}

static ptrdiff_t
memdiff(const void* low, const void* high)
{
  return ((const char*)high) - ((const char*)low);
}

static long
read_pdu_at_va(const struct pdu* pdu, unsigned long offset,
               const char* fmt, va_list ap)
{
  void** mem;
  void* chr;
  void* dst;
  size_t len;

  for (; *fmt; ++fmt) {
    switch (*fmt) {
      case 'c': /* signed 8 bit */
        dst = va_arg(ap, int8_t*);
        len = sizeof(int8_t);
        break;
      case 'C': /* unsigned 8 bit*/
        dst = va_arg(ap, uint8_t*);
        len = sizeof(uint8_t);
        break;
      case 's': /* signed 16 bit */
        dst = va_arg(ap, int16_t*);
        len = sizeof(int16_t);
        break;
      case 'S': /* unsigned 16 bit */
        dst = va_arg(ap, uint16_t*);
        len = sizeof(uint16_t);
        break;
      case 'i': /* signed 32 bit */
        dst = va_arg(ap, int32_t*);
        len = sizeof(int32_t);
        break;
      case 'I': /* unsigned 32 bit */
        dst = va_arg(ap, uint32_t*);
        len = sizeof(uint32_t);
        break;
      case 'l': /* signed 64 bit */
        dst = va_arg(ap, int64_t*);
        len = sizeof(int64_t);
        break;
      case 'L': /* unsigned 64 bit */
        dst = va_arg(ap, uint64_t*);
        len = sizeof(uint64_t);
        break;
      case 'f': /* 32-bit floating point */
        dst = va_arg(ap, float*);
        len = sizeof(float);
        break;
      case 'd': /* 64-bit floating point */
        dst = va_arg(ap, double*);
        len = sizeof(double);
        break;
      case 'm': /* raw memory + length */
        dst = va_arg(ap, void*);
        len = va_arg(ap, size_t);
        break;
      case 'M': /* unallocated raw memory + length */
        mem = va_arg(ap, void**);
        len = va_arg(ap, size_t);
        dst = malloc(len);
        if (!dst) {
          ALOGE_ERRNO("malloc");
          return -1;
        }
        *mem = dst;
        break;
      case '0': /* raw 0-terminated memory */
        mem = va_arg(ap, void**);
        chr = memchr(pdu->data + offset, '\0', pdu->len - offset);
        if (!chr) {
          ALOGE("string not terminated");
          return -1;
        }
        len = memdiff(pdu->data + offset, chr) + 1; /* include \0 byte */
        dst = malloc(len);
        if (!dst) {
          ALOGE_ERRNO("malloc");
          return -1;
        }
        *mem = dst;
        break;
      default:
        ALOGE("invalid format character %c", *fmt);
        return -1;
    }
    if (offset+len > pdu->len) {
      ALOGE("PDU overflow");
      return -1;
    }
    memcpy(dst, pdu->data+offset, len);
    offset += len;
  }

  return offset;
}

long
read_pdu_at(const struct pdu* pdu, unsigned long offset, const char* fmt, ...)
{
  va_list ap;
  long res;

  va_start(ap, fmt);
  res = read_pdu_at_va(pdu, offset, fmt, ap);
  va_end(ap);

  return res;
}

static long
write_pdu_at_va(struct pdu* pdu, unsigned long offset, const char* fmt,
                va_list ap)
{
  int8_t c;
  uint8_t C;
  int16_t s;
  uint16_t S;
  int32_t i;
  uint32_t I;
  int64_t l;
  uint64_t L;
  float f;
  double d;
  const void* src;
  const void* chr;
  size_t len;

  for (; *fmt; ++fmt) {
    switch (*fmt) {
      case 'c': /* signed 8 bit */
        c = (int8_t)va_arg(ap, int);
        src = &c;
        len = sizeof(c);
        break;
      case 'C': /* unsigned 8 bit */
        C = (uint8_t)va_arg(ap, int);
        src = &C;
        len = sizeof(C);
        break;
      case 's': /* signed 16 bit */
        s = (int16_t)va_arg(ap, int);
        src = &s;
        len = sizeof(s);
        break;
      case 'S': /* unsigned 16 bit */
        S = (uint16_t)va_arg(ap, int);
        src = &S;
        len = sizeof(S);
        break;
      case 'i': /* signed 32 bit */
        i = va_arg(ap, int32_t);
        src = &i;
        len = sizeof(i);
        break;
      case 'I': /* unsigned 32 bit */
        I = va_arg(ap, uint32_t);
        src = &I;
        len = sizeof(I);
        break;
      case 'l': /* signed 64 bit */
        l = va_arg(ap, int64_t);
        src = &l;
        len = sizeof(l);
        break;
      case 'L': /* unsigned 64 bit */
        L = va_arg(ap, uint64_t);
        src = &L;
        len = sizeof(L);
        break;
      case 'f': /* 32-bit floating point */
        f = (float)va_arg(ap, double);
        src = &f;
        len = sizeof(f);
        break;
      case 'd': /* 64-bit floating point */
        d = va_arg(ap, double);
        src = &d;
        len = sizeof(d);
        break;
      case 'm': /* raw memory + length */
      case 'M': /* raw memory + length */
        src = va_arg(ap, void*);
        len = va_arg(ap, size_t);
        break;
      case '0': /* raw 0-terminated memory */
        src = va_arg(ap, void*);
        chr = strchr(src, '\0');
        len = memdiff(src, chr) + 1; /* include \0 byte */
        break;
      default:
        ALOGE("invalid format character %c", *fmt);
        return -1;
    }
    memcpy(pdu->data+offset, src, len);
    offset += len;
  }

  return offset;
}

long
write_pdu_at(struct pdu* pdu, unsigned long offset, const char* fmt, ...)
{
  va_list ap;
  long res;

  va_start(ap, fmt);
  res = write_pdu_at_va(pdu, offset, fmt, ap);
  va_end(ap);

  return res;
}

long
append_to_pdu(struct pdu* pdu, const char* fmt, ...)
{
  va_list ap;
  long res;

  va_start(ap, fmt);
  res = write_pdu_at_va(pdu, pdu->len, fmt, ap);
  va_end(ap);

  if (res > 0)
    pdu->len = res;

  return res;
}
