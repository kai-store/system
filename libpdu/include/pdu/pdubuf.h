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

#pragma once

#include <sys/queue.h>
#include <sys/socket.h>
#include "pdu.h"

/*
 * The data types |struct pdu_rbuf| and |struct pdu_wbuf| represent PDU
 * containers for reading and writing PDUs from and to a socket. When
 * working with PDUs, you should use these types for allocating PDUs
 * instead of creating |struct pdu| directly.
 *
 * The type |struct pdu_rbuf| contains the maximum length and the length
 * of the valid data in the PDU. These numbers include the 4 bytes of the
 * PDU's header.
 *
 * The type |struct pdu_wbuf| contains the maximum length of the PDU plus
 * the offset of the 'tail data.' This memory block can contain arbitrary
 * data and is useful for storing ancillary data. Transferring ancillary
 * data is a feature of some socket protocols (such as UNIX domain sockets)
 * for sending messages out-of-band. Users can set |build_ancillary_data|
 * to a function that packs the ancillary message. A call to |send_pdu_wbuf|
 * will then invoke the callback function as part of the sending process.
 */

struct pdu_rbuf {
  unsigned long maxlen;
  unsigned long len;
  union {
    struct pdu pdu;
    unsigned char raw[0];
  } buf;
};

struct pdu_rbuf*
create_pdu_rbuf(unsigned long maxdatalen);

void
destroy_pdu_rbuf(struct pdu_rbuf* rbuf);

int
pdu_rbuf_has_pdu_hdr(const struct pdu_rbuf* rbuf);

int
pdu_rbuf_has_pdu(const struct pdu_rbuf* rbuf);

int
pdu_rbuf_is_full(const struct pdu_rbuf* rbuf);

struct pdu_wbuf {
  STAILQ_ENTRY(pdu_wbuf) stailq;
  unsigned long tailoff;
  unsigned long maxlen;
  int (*build_ancillary_data)(struct pdu_wbuf*, struct msghdr*);
  union {
    struct pdu pdu;
    unsigned char raw[0];
  } buf;
  unsigned char tail[0];
};

struct pdu_wbuf*
create_pdu_wbuf(unsigned long maxdatalen, unsigned long taillen,
                int (*build_ancillary_data)(struct pdu_wbuf*, struct msghdr*));

void
destroy_pdu_wbuf(struct pdu_wbuf* wbuf);

ssize_t
send_pdu_wbuf(struct pdu_wbuf* wbuf, int fd, int flags);

int
pdu_wbuf_consumed(const struct pdu_wbuf* wbuf);

void*
pdu_wbuf_tail(struct pdu_wbuf* wbuf);
