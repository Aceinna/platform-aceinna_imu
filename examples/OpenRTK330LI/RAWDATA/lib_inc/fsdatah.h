#ifndef __FSDATAH_H__
#define __FSDATAH_H__

#include "lwip/opt.h"
#include "fs.h"

struct fsdata_file {
  const struct fsdata_file *next;
  const unsigned char *name;
  const unsigned char *data;
  int len;
  u8_t http_header_included;
#if HTTPD_PRECALCULATED_CHECKSUM
  u16_t chksum_count;
  const struct fsdata_chksum *chksum;
#endif /* HTTPD_PRECALCULATED_CHECKSUM */
};

#endif /* __FSDATA_H__ */